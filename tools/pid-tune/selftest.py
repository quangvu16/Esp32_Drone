#!/usr/bin/env python3
"""Offline tests for tools/pid-tune. Run with:

    python3 selftest.py

No drone needed: a fake firmware answers on UDP exactly as Esp32_drone.ino
does, so the wire layer (framing, TOCs, parameter read/write, log blocks,
telemetry decode) is exercised end to end. The Bluetooth links (btlink.py) get
the same treatment through their pure halves — BLE segment packing/reassembly
against the firmware's bleRxSeg(), and SPP stream resync against btFrameLen().
The optimisers are then run against a synthetic cost surface with a known
minimum, which is the only way to tell a faithful port from a plausible-looking
one without spinning motors.

The firmware has its own suite (Esp32_drone/test/) and the apps have theirs;
this is the tuner's half of the same arrangement.
"""
import asyncio
import math
import os
import socket
import struct
import sys
import tempfile
import threading
import time
import traceback

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import btlink as B        # noqa: E402
import drone as D          # noqa: E402
import ml_tune as M        # noqa: E402

FAILS = []


def expect(what, want, got):
    if want != got:
        FAILS.append(what)
        print(f"FAIL {what}: want {want!r}, got {got!r}")
    else:
        print(f"ok   {what}")


def close(what, want, got, tol):
    ok = abs(got - want) <= tol
    if not ok:
        FAILS.append(what)
        print(f"FAIL {what}: want {want} +- {tol}, got {got}")
    else:
        print(f"ok   {what} ({got:.4g})")


def check(what, cond):
    if not cond:
        FAILS.append(what)
        print(f"FAIL {what}")
    else:
        print(f"ok   {what}")


# ============================================================ the fake firmware

PARAM_TOC = [
    ("cpu", "flash", 0x0A, 4, True, 0.0, 0.0),
    ("deck", "bcZRanger", 0x08, 1, True, 0.0, 0.0),
    ("ring", "neffect", 0x08, 1, True, 0.0, 0.0),
    ("pid", "roll_kp", 0x06, 4, False, 0.0, 100.0),
    ("pid", "roll_ki", 0x06, 4, False, 0.0, 20.0),
    ("pid", "roll_kd", 0x06, 4, False, 0.0, 200.0),
    ("pid", "pitch_kp", 0x06, 4, False, 0.0, 100.0),
    ("pid", "pitch_ki", 0x06, 4, False, 0.0, 20.0),
    ("pid", "pitch_kd", 0x06, 4, False, 0.0, 200.0),
    ("pid", "yaw_kp", 0x06, 4, False, 0.0, 100.0),
    ("pid", "yaw_ki", 0x06, 4, False, 0.0, 20.0),
    ("pid", "yaw_kd", 0x06, 4, False, 0.0, 200.0),
    ("flight", "hover_thrust", 0x09, 2, False, 8000.0, 55000.0),
    ("flight", "takeoff_ms", 0x09, 2, False, 100.0, 20000.0),
    ("flight", "land_ms", 0x09, 2, False, 100.0, 20000.0),
    ("nav", "kp", 0x06, 4, False, 0.0, 20.0),
]
LOG_TOC = [
    ("pm", "vbat", 0x07, 4), ("stabilizer", "roll", 0x07, 4),
    ("stabilizer", "pitch", 0x07, 4), ("stabilizer", "yaw", 0x07, 4),
    ("controller", "roll", 0x07, 4), ("controller", "pitch", 0x07, 4),
    ("controller", "yaw", 0x07, 4), ("motor", "m1", 0x02, 2),
    ("motor", "m2", 0x02, 2), ("motor", "m3", 0x02, 2), ("motor", "m4", 0x02, 2),
    ("flight", "mode", 0x01, 1), ("flight", "auto_state", 0x01, 1),
    ("flight", "armed", 0x01, 1), ("flight", "heading", 0x07, 4),
    ("nav", "pos_x", 0x07, 4), ("nav", "pos_y", 0x07, 4),
    ("nav", "vel_x", 0x07, 4), ("nav", "vel_y", 0x07, 4),
    ("nav", "tgt_x", 0x07, 4), ("nav", "tgt_y", 0x07, 4),
    ("pid", "roll_p", 0x07, 4), ("pid", "roll_i", 0x07, 4), ("pid", "roll_d", 0x07, 4),
    ("pid", "pitch_p", 0x07, 4), ("pid", "pitch_i", 0x07, 4), ("pid", "pitch_d", 0x07, 4),
    ("pid", "yaw_p", 0x07, 4), ("pid", "yaw_i", 0x07, 4), ("pid", "yaw_d", 0x07, 4),
]


class FakeFirmware:
    """Answers CRTP on UDP the way crtpHandleLogging / the param TOC do.

    Everything here is transcribed from Esp32_drone.ino's reply shapes, so if
    the client and this disagree, one of them is wrong against the real board.
    """

    def __init__(self, device_port, app_port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", device_port))
        self.sock.settimeout(0.05)
        self.app = ("127.0.0.1", app_port)
        self.values = {}
        for i, (g, n, t, size, ro, lo, hi) in enumerate(PARAM_TOC):
            self.values[i] = 1 + i          # an int packs into every param fmt
                                            # một số int đóng gói được vào mọi định dạng tham số
        self.blocks = {}                  # id -> {"vars": [...], "started": bool}
        self.fc_seen = []                 # raw port-7 packets, header included
        self.thrusts = []                 # thrust of every CommanderPacket
        self.armed = False
        self.thrust_was_low = False
        self.arm_events = []
        self.running = True
        self.thread = threading.Thread(target=self.serve, daemon=True)
        self.thread.start()

    def close(self):
        self.running = False
        try:
            self.sock.close()
        except OSError:
            pass

    @staticmethod
    def _csum(raw):
        return sum(raw) & 0xFF

    def _send(self, port, channel, payload):
        raw = bytes([((port & 0xF) << 4) | (channel & 0xF)]) + bytes(payload)
        try:
            self.sock.sendto(raw + bytes([self._csum(raw)]), self.app)
        except OSError:
            pass                            # closed under us at teardown
                                            # socket bị đóng ngay khi teardown

    def serve(self):
        while self.running:
            try:
                data, _ = self.sock.recvfrom(512)
            except socket.timeout:
                self.push_frames()
                continue
            except OSError:
                break
            if len(data) < 2 or self._csum(data[:-1]) != data[-1]:
                continue
            raw = data[:-1]
            port, ch = raw[0] >> 4, raw[0] & 0xF
            p = raw                      # the firmware indexes from the header
                                         # firmware đánh chỉ mục từ header
            if ch == 0 and (port == D.PORT_PARAMS or port == D.PORT_LOG):
                self.toc(port, p)
            elif port == D.PORT_PARAMS:
                self.param(ch, p)
            elif port == D.PORT_LOG and ch == 1:
                self.log_ctrl(p)
            elif port == D.PORT_FLIGHT:
                self.flight_ctrl(p)
            elif port == D.PORT_COMMANDER:
                self.commander(p)

    # --- TOC ---
    def toc(self, port, p):
        toc = PARAM_TOC if port == D.PORT_PARAMS else LOG_TOC
        if p[1] == D.CMD_TOC_INFO:
            self._send(port, 0, bytes([D.CMD_TOC_INFO, len(toc)]) + struct.pack("<I", 0xDEADBEEF))
        elif p[1] == D.CMD_TOC_ELEMENT and len(p) >= 3:
            i = p[2]
            if i >= len(toc):
                return
            group, name, ctype, _size = toc[i][0], toc[i][1], toc[i][2], toc[i][3]
            access = 0x40 if (port == D.PORT_PARAMS and toc[i][4]) else 0x00
            body = bytes([D.CMD_TOC_ELEMENT, i, ctype | access]) \
                + group.encode() + b"\0" + name.encode() + b"\0"
            self._send(port, 0, body)

    # --- parameters ---
    def param(self, ch, p):
        if len(p) < 2:
            return
        i = p[1]
        if i >= len(PARAM_TOC):
            return
        _g, _n, ctype, size, _ro, lo, hi = PARAM_TOC[i]
        fmt = D.PARAM_TYPES[ctype][0]
        if ch == D.CH_READ:
            self._send(D.PORT_PARAMS, D.CH_READ, bytes([i]) + struct.pack(fmt, self.values[i]))
        elif ch == D.CH_WRITE and len(p) >= 2 + size:
            v = struct.unpack(fmt, bytes(p[2:2 + size]))[0]
            v = min(max(v, lo), hi)            # paramClampF
            if fmt[-1] not in "fd":
                v = int(v)                     # then (uint16_t) for integer params
                                               # rồi (uint16_t) cho các tham số số nguyên
            self.values[i] = v
            self._send(D.PORT_PARAMS, D.CH_WRITE, bytes([i]) + struct.pack(fmt, v))

    # --- logging ---
    def log_ctrl(self, p):
        if len(p) < 2:
            return
        cmd, bid = p[1], p[2] if len(p) > 2 else 0
        if cmd == D.LOG_RESET:
            self.blocks.clear()
            self._send(D.PORT_LOG, 1, bytes([cmd, 0, 0]))
            return
        if cmd in (D.LOG_CREATE, D.LOG_APPEND):
            ids = []
            # i = 3 matches the firmware: p[0] header, p[1] cmd, p[2] id
            # i = 3 khớp với firmware: p[0] header, p[1] cmd, p[2] id
            for i in range(3, len(p) - 1, 2):
                if p[i] == 0:
                    break
                ids.append(p[i + 1])
            if not ids:
                self._send(D.PORT_LOG, 1, bytes([cmd, bid, 7]))
                return
            total = sum(LOG_TOC[i][3] for i in ids)
            if total > 25:
                self._send(D.PORT_LOG, 1, bytes([cmd, bid, 7]))
                return
            if bid in self.blocks:
                same = self.blocks[bid]["vars"] == ids
                self._send(D.PORT_LOG, 1, bytes([cmd, bid, 0 if same else 17]))
                return
            self.blocks[bid] = {"vars": ids, "started": False}
            self._send(D.PORT_LOG, 1, bytes([cmd, bid, 0]))
        elif cmd == D.LOG_START:
            if bid not in self.blocks:
                self._send(D.PORT_LOG, 1, bytes([cmd, bid, 2]))
                return
            self.blocks[bid]["started"] = True
            self._send(D.PORT_LOG, 1, bytes([cmd, bid, 0]))
        elif cmd == D.LOG_STOP:
            if bid in self.blocks:
                self.blocks[bid]["started"] = False
            self._send(D.PORT_LOG, 1, bytes([cmd, bid, 0]))
        elif cmd == D.LOG_DELETE:
            self.blocks.pop(bid, None)
            self._send(D.PORT_LOG, 1, bytes([cmd, bid, 0]))

    # Deterministic, distinguishable values per log variable, so a decode test
    # that sees the wrong number knows which field landed where.
    # Giá trị tất định, phân biệt được cho từng biến log, để bài kiểm tra giải mã
    # mà thấy số sai sẽ biết trường nào đã rơi nhầm chỗ.
    LOG_VALUES = {
        "pm.vbat": 3.9, "stabilizer.roll": 1.5, "stabilizer.pitch": -2.5,
        "stabilizer.yaw": 12.0, "controller.roll": 7.0, "controller.pitch": -7.0,
        "controller.yaw": 7.5, "motor.m1": 1500, "motor.m2": 1501,
        "motor.m3": 1502, "motor.m4": 1503, "flight.mode": 1, "flight.armed": 1,
        "flight.auto_state": 2, "flight.heading": 90.0,
        "pid.roll_p": 1.0, "pid.roll_i": 2.0, "pid.roll_d": 3.0,
        "pid.pitch_p": 1.1, "pid.pitch_i": 2.1, "pid.pitch_d": 3.1,
        "pid.yaw_p": 1.2, "pid.yaw_i": 2.2, "pid.yaw_d": 3.2,
    }

    def push_frames(self):
        """Emit one frame per running block, as logUpdate() would."""
        for bid, b in list(self.blocks.items()):
            if not b["started"]:
                continue
            vals = b""
            for i in b["vars"]:
                group, name, ctype, size = LOG_TOC[i]
                v = self.LOG_VALUES.get(group + "." + name, 0.0)
                fmt = D.LOG_TYPES[ctype][0]
                vals += struct.pack(fmt, v)
            self._send(D.PORT_LOG, D.LOG_CH_DATA, bytes([bid, 0, 0, 0]) + vals)

    # crtpHandleCommander() + armUpdate()'s CRTP branch: 15 bytes, thrust at
    # p+13; arms on thrust-up after a low thrust, disarms at <= 500.
    def commander(self, p):
        if len(p) < 15:
            return
        thrust = struct.unpack("<H", bytes(p[13:15]))[0]
        self.thrusts.append(thrust)
        if not self.armed:
            if thrust <= 500:
                self.thrust_was_low = True
            if thrust > 1000 and self.thrust_was_low:
                self.armed, self.thrust_was_low = True, False
                self.arm_events.append("arm")
        elif thrust <= 500:
            self.armed = False
            self.arm_events.append("disarm")

    # crtpHandleFlightControl(): p[0] is the 0x70 header, p[1] the command;
    # the reply is <0x70><cmd><status>, status 1 for an unknown command.
    FC_KNOWN = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x10}

    def flight_ctrl(self, p):
        # p is the full packet: p[0]=header 0x70, p[1]=cmd, p[2..]=args. The firmware
        # reads cmd=p[1] and replies <cmd><status>. Mirror that exactly — an extra
        # FC_MAGIC here once hid a real framing bug from the drone side.
        # p là gói đầy đủ: p[0]=header 0x70, p[1]=cmd, p[2..]=args. Firmware đọc
        # cmd=p[1] và trả lời <cmd><status>. Sao chép đúng như vậy — một FC_MAGIC
        # thừa từng che một lỗi đóng khung phía drone.
        if len(p) < 2:
            return
        cmd = p[1]
        self.fc_seen.append(bytes(p))
        self._send(D.PORT_FLIGHT, 0, bytes([cmd, 0 if cmd in self.FC_KNOWN else 1]))


# ============================================================ wire tests


def test_wire():
    print("\n--- framing ---")
    raw = bytes([0x21, 0x03, 0x01, 0x02])
    expect("checksum is the low byte of the sum", (sum(raw) & 0xFF), D.Drone._checksum(raw))
    raw = bytes([0x51]) + bytes([0xFF] * 30)
    expect("checksum wraps at 255", (sum(raw) & 0xFF), D.Drone._checksum(raw))

    print("\n--- struct budgets (PROTOCOL.md: 25 B of values max) ---")
    expect("status block is 15 B", 15, D.STATUS_STRUCT.size)
    expect("tune block is 22 B", 22, D.TUNE_STRUCT.size)
    check("status block fits", D.STATUS_STRUCT.size <= 25)
    check("tune block fits", D.TUNE_STRUCT.size <= 25)
    expect("status vars match its layout", 6, len(D.STATUS_VARS))
    for axis in D.AXES:
        expect(f"{axis} tune block has 6 vars", 6, len(D.TUNE_VARS[axis]))

    print("\n--- gain naming ---")
    expect("roll gain 0", "pid.roll_kp", D.gain_name("roll", 0))
    expect("yaw gain 2", "pid.yaw_kd", D.gain_name("yaw", 2))
    expect("pitch axis names", ["pid.pitch_kp", "pid.pitch_ki", "pid.pitch_kd"],
           D.axis_names("pitch"))

    print("\n--- the firmware's clamp is what the tuner uses ---")
    for k, label in enumerate(D.GAIN_LABELS):
        close(f"P_MIN[{label}] is 0", 0.0, D.P_MIN[k], 0)
    close("P_MAX[kp]", 100.0, D.P_MAX[0], 0)
    close("P_MAX[ki]", 20.0, D.P_MAX[1], 0)
    close("P_MAX[kd]", 200.0, D.P_MAX[2], 0)


def test_link():
    print("\n--- link against a fake firmware ---")
    dev, app = 24090, 24099
    fw = FakeFirmware(dev, app)
    d = D.Drone("127.0.0.1", dev, app, timeout=1.5)
    try:
        d.fetch_toc(D.PORT_PARAMS, d.param_toc)
        expect("param TOC size", len(PARAM_TOC), len(d.param_toc.by_id))
        expect("param TOC crc carried", 0xDEADBEEF, d.param_toc.crc)
        expect("roll_kp is id 3", 3, d.param_toc.id_of("pid.roll_kp"))
        expect("yaw_kd is id 11", 11, d.param_toc.id_of("pid.yaw_kd"))
        check("roll_kp is writable", not d.param_toc.by_id[3][4])
        check("cpu.flash is read-only", d.param_toc.by_id[0][4])

        d.fetch_toc(D.PORT_LOG, d.log_toc)
        expect("log TOC size", len(LOG_TOC), len(d.log_toc.by_id))
        expect("motor.m1 is log id 7", 7, d.log_toc.id_of("motor.m1"))
        expect("stabilizer.yaw is log id 3", 3, d.log_toc.id_of("stabilizer.yaw"))

        print("\n--- parameter read / write ---")
        close("read back the seed value", 4.0, d.get_param("pid.roll_kp"), 1e-6)
        close("write clamped to P_MAX", 100.0, d.set_param("pid.roll_kp", 500.0), 1e-6)
        close("write clamped to P_MIN", 0.0, d.set_param("pid.roll_kp", -5.0), 1e-6)
        close("a normal write sticks", 12.5, d.set_param("pid.roll_kp", 12.5), 1e-6)

        print("\n--- log blocks ---")
        D.open_tune_blocks(d, "roll", 50)
        expect("both blocks exist", 2, len(fw.blocks))
        expect("the status block is 15 B", 15, sum(LOG_TOC[i][3] for i in fw.blocks[0]["vars"]))
        expect("the roll tune block is 22 B", 22, sum(LOG_TOC[i][3] for i in fw.blocks[1]["vars"]))
        # re-creating an identical block must re-ack OK, not raise
        # tạo lại một khối giống hệt phải được ack lại OK, không ném lỗi
        d.log_create(D.TUNE_BLOCK_ID, D.TUNE_VARS["roll"], 50)
        print("ok   identical CREATE re-acks OK")
        # a CREATE with *different* variables over a live block is refused (17),
        # which is why open_tune_blocks() STOP+DELETEs before switching axis
        # một CREATE với biến *khác* trên block đang sống bị từ chối (17), đó là
        # lý do open_tune_blocks() STOP+DELETE trước khi đổi trục
        try:
            d.log_create(D.TUNE_BLOCK_ID, D.TUNE_VARS["pitch"], 50)
            check("different CREATE is refused (err 17)", False)
        except RuntimeError as e:
            check("different CREATE is refused (err 17)", "17" in str(e))

        print("\n--- flight control (payload <cmd>, reply <cmd><status>) ---")
        check("arm acked", d.arm())
        check("motor test acked", d.motor_test(1, 1200, 1000, 1000, 1000))
        check("motor test stop acked", d.motor_test(0))
        check("disarm acked", d.disarm())

        print("\n--- telemetry decode ---")
        check("level starts neutral", d.level == {"roll": 0.0, "pitch": 0.0})
        time.sleep(0.35)
        got = d.samples_since(0)
        check("samples arrived", len(got) > 3)
        if got:
            s = got[-1]
            check("sample has the cost fields",
                  all(k in s for k in ("angle", "rate", "control", "motor", "roll", "pitch")))
            close("status block stamped roll", 1.5, s["roll"], 1e-6)
            close("status block stamped pitch", -2.5, s["pitch"], 1e-6)
            close("angle is stabilizer.roll", 1.5, s["angle"], 1e-6)
            close("control is controller.roll", 7.0, s["control"], 1e-6)
            close("motor decoded as uint16", 1500.0, s["motor"], 1e-6)
            close("P term is in the sample", 1.0, s["p"], 1e-6)
            close("D term is in the sample", 3.0, s["d"], 1e-6)
            close("rate is finite-differenced to 0 when flat", 0.0, s["rate"], 1e-6)
            check("every sample carries a host timestamp", "t" in s)

            print("\n--- cost over decoded samples ---")
            cost, m = D.cost_of(got, 6.0)
            check("cost is finite", math.isfinite(cost))
            check("no tip recorded", not m["fell"])
            close("angle_std of a constant is 0", 0.0, m["angle_std"], 1e-6)
            close("motor_rms of 1500 is 0", 0.0, m["motor_rms"], 1e-6)
            close("saturation share of 1500 is 0", 0.0, m["sat"], 1e-6)
            close("control effort is |7|", 7.0, m["control_effort"], 1e-6)

        print("\n--- yaw uses stabilizer.yaw as the rate directly ---")
        D.open_tune_blocks(d, "yaw", 50)
        time.sleep(0.35)
        yaw = d.samples_since(0)
        check("yaw samples arrived", len(yaw) > 3)
        if yaw:
            close("yaw angle is stabilizer.yaw", 12.0, yaw[-1]["angle"], 1e-6)
            close("yaw rate IS the angle (no differencing)", 12.0, yaw[-1]["rate"], 1e-6)

        print("\n--- the tip penalty (autotune.py's measure()) ---")
        flat = [{"angle": 0.0, "rate": 0.0, "control": 0.0, "motor": 1500.0,
                 "roll": 0.0, "pitch": 0.0} for _ in range(20)]
        tipped = [dict(f) for f in flat]
        tipped[5]["roll"] = 40.0
        cost, m = D.cost_of(tipped, 10.0)
        check("a tip is flagged", m["fell"])
        # survived = 5/20 * 10 s = 2.5 s -> 100 + 10 * 7.5 = 175
        # sống sót = 5/20 * 10 s = 2.5 s -> 100 + 10 * 7.5 = 175
        close("tip cost is 100 + 10 x seconds lost", 175.0, cost, 1e-6)
        cost, m = D.cost_of(flat, 10.0)
        check("a clean window costs under 1", not m["fell"] and cost < 1.0)

        print("\n--- flight-control commands (port 7) ---")
        # The firmware reads the command from p[1], right after the 0x70
        # header. An extra magic byte there made every command a "bad
        # request" on the real board, EMERGENCY included.
        fw.fc_seen.clear()
        check("ARM is acknowledged", d.arm())
        expect("ARM goes out as <0x70><0x07>", bytes([0x70, 0x07]), fw.fc_seen[-1])
        check("EMERGENCY is acknowledged", d.emergency())
        expect("EMERGENCY goes out as <0x70><0x05>", bytes([0x70, 0x05]), fw.fc_seen[-1])
        check("DISARM is acknowledged", d.disarm())
        check("SAVE is acknowledged", d.save())
        check("LOAD is acknowledged", d.load())
        check("DEFAULTS is acknowledged", d.defaults())
        check("an unknown command reports refused", d._fc(0x7E)[-1] == 1)

        print("\n--- stand thrust stream (port 3) ---")
        expect("40 % is 26214 thrust units", 26214, D.thrust_counts(40))
        expect("thrust is capped at THRUST_MAX_PCT", D.thrust_counts(D.THRUST_MAX_PCT), D.thrust_counts(95))
        fw.thrusts.clear()
        st = D.ThrustStream(d, pct=30, ramp_s=0.5)
        time.sleep(1.6)
        check("the stream starts at zero thrust", fw.thrusts and fw.thrusts[0] == 0)
        # The loop aims at SETPOINT_HZ = 50, but time.sleep() overshoots on some
        # hosts (27 ms for a 20 ms request on this macOS = 37 Hz ceiling), so the
        # floor sits below 1.6 x 50 on purpose: it still catches a halved,
        # stuttering or missing stream without failing on the scheduler.
        # Vòng lặp đặt mục tiêu SETPOINT_HZ = 50, nhưng time.sleep() trễ trên
        # một số máy (27 ms cho yêu cầu 20 ms trên macOS này = trần 37 Hz), nên
        # sàn cố tình đặt dưới 1.6 x 50: vẫn bắt được luồng chậm một nửa, giật
        # cục hay mất luồng mà không fail vì chính bộ lập lịch.
        check("~50 packets a second", 48 <= len(fw.thrusts) <= 95)
        close("it ramps to the target", D.thrust_counts(30), fw.thrusts[-1], 1)
        check("the board armed on thrust-up", fw.arm_events[:1] == ["arm"] and fw.armed)
        st.stop()
        time.sleep(0.2)
        expect("stop() ends at zero thrust", 0, fw.thrusts[-1])
        check("zero thrust disarms the board", not fw.armed)
        check("the thread is gone", not st.running)
    finally:
        d.close()
        fw.close()


# ============================================================ bluetooth links


def test_btlink():
    # The Bluetooth wire rules are the firmware's: the BLE segment state
    # machine is pinned by Esp32_drone/test/bleseg_test.cpp, and the SPP stream
    # resync by btFrameLen()/btPoll(). These tests pin the Python half of the
    # same rules so the three implementations cannot drift apart.
    # Quy tắc dây Bluetooth là của firmware: máy trạng thái đoạn BLE được chốt
    # bởi Esp32_drone/test/bleseg_test.cpp, còn đồng bộ luồng SPP bởi
    # btFrameLen()/btPoll(). Các bài này chốt nửa Python của cùng quy tắc để ba
    # bản triển khai không thể lệch nhau.
    print("\n--- bluetooth links (BLE segments, SPP stream) ---")
    expect("btlink checksum agrees with Drone",
           D.Drone._checksum(b"\x70\x07"), B.crtp_checksum(b"\x70\x07"))

    pay = bytes(range(0xA0, 0xA0 + 30))

    print("\n--- BLE segment packing (PROTOCOL.md §6) ---")
    segs = B.seg_pack(pay[:15], 0)
    expect("a 15-byte packet is one segment", 1, len(segs))
    expect("it is start|pid0|len-1 then the CRTP", bytes([0x80 | 14]) + pay[:15], segs[0])
    segs = B.seg_pack(pay[:24], 1)
    expect("a 24-byte packet is two segments", 2, len(segs))
    expect("first segment: start|pid1|23, 19 bytes", bytes([0x80 | (1 << 5) | 23]) + pay[:19], segs[0])
    expect("second segment: cont|pid1|23, 5 bytes", bytes([(1 << 5) | 23]) + pay[19:24], segs[1])
    segs = B.seg_pack(pay[:20], 2)
    expect("a 20-byte packet is two segments (it cannot fit one)", 2, len(segs))
    expect("its last byte travels alone", bytes([(2 << 5) | 19]) + pay[19:20], segs[1])
    segs = B.seg_pack(pay[:30], 3)
    expect("a 30-byte packet is two segments", 2, len(segs))
    expect("the split is 19 + 11", [20, 12], [len(s) for s in segs])
    for bad in (b"", pay + b"\xEE"):
        try:
            B.seg_pack(bad, 0)
            check("pack refuses %d-byte datagrams" % len(bad), False)
        except ValueError:
            check("pack refuses %d-byte datagrams" % len(bad), True)

    print("\n--- BLE reassembly mirrors bleRxSeg ---")
    for n in (1, 15, 16, 19, 20, 21, 24, 30):
        segs = B.seg_pack(pay[:n], 1)
        ra = B.SegReassembler()
        got = None
        for i, s in enumerate(segs):
            r = ra.feed(s[0], s[1:])
            if i < len(segs) - 1:
                check("%d-byte datagram: segment %d waits" % (n, i + 1), r is None)
            else:
                got = r
        expect("%d-byte datagram reassembles" % n, pay[:n], got)
        check("%d-byte datagram resets the state" % n, not ra.buf)

    ra = B.SegReassembler()
    check("a 20-byte packet is not mistaken for single-segment",
          ra.feed(0x80 | (2 << 5) | 19, pay[:19]) is None)
    expect("its last byte completes it", pay[:20], ra.feed((2 << 5) | 19, pay[19:20]))
    expect("padding beyond the announced length is ignored",
           pay[:4], B.SegReassembler().feed(0x80 | 3, pay[:19]))
    ra = B.SegReassembler()
    ra.feed(0x80 | 23, pay[:19])
    check("a continuation with another pid is refused",
          ra.feed((1 << 5) | 23, pay[19:24]) is None)
    check("and the partial packet is gone", not ra.buf)
    check("a continuation without start is refused",
          B.SegReassembler().feed(23, pay[:5]) is None)
    check("an oversize write is refused",
          B.SegReassembler().feed(0x80 | 30, pay + b"\xEE") is None)
    ra = B.SegReassembler()
    ra.feed(0x80 | 29, pay[:19])
    check("accumulation past CRTP_MAX_SIZE resets",
          ra.feed(29, pay[19:] + b"\xEE") is None and not ra.buf)

    print("\n--- BLE writes never carry a checksum ---")
    commander = bytes([0x30]) + bytes(range(14))       # 15-byte raw CommanderPacket
    w = B.ble_writes(commander, 0)
    expect("a 15-byte commander is one raw write on 0x202",
           [(B.BLE_UUID_CRTP, commander)], w)
    expect("the raw write is exactly the CRTP bytes", 15, len(w[0][1]))
    big = bytes([0x70]) + bytes(range(25))             # 26 bytes -> segmented
    w = B.ble_writes(big, 3)
    expect("a 26-byte packet goes to 0x203 in two segments",
           [B.BLE_UUID_CRTPUP, B.BLE_UUID_CRTPUP], [u for u, _ in w])
    expect("the segments carry exactly the CRTP bytes, checksum-free",
           big, b"".join(d[1:] for _u, d in w))

    print("\n--- SPP stream framing mirrors btFrameLen ---")

    def frame(raw):
        return bytes(raw) + bytes([sum(raw) & 0xFF])   # PROTOCOL.md §1, built
                                                       # here, not via btlink
    f1, f2 = frame(b"\x20\x05\x01"), frame(b"\x20\x06\x02\x03")
    fd = B.StreamDeframer()
    expect("two back-to-back frames come out stripped",
           [b"\x20\x05\x01", b"\x20\x06\x02\x03"], fd.feed(f1 + f2))
    check("nothing is left over", not fd.acc)
    fd = B.StreamDeframer()
    check("a partial frame waits", [] == fd.feed(f1[:2]))
    expect("and completes when the rest arrives", [b"\x20\x05\x01"], fd.feed(f1[2:]))

    junk = bytearray([0x50])                           # never a valid prefix:
    while len(junk) < B.BT_FRAME_MAX:                  # each byte != running sum
        junk.append((sum(junk) + 1) & 0xFF)
    expect("31 bytes with no valid frame ask to drop one for resync",
           -1, B.frame_len(bytes(junk)))
    expect("under 31, the same stream waits for more", 0, B.frame_len(bytes(junk[:30])))

    cmd_raw = bytes([0x30]) + bytes(range(14))
    cmd_f = frame(cmd_raw)
    expect("a commander frame is 16 wire bytes", 16, len(cmd_f))
    expect("it is recognised at its real length", 16, B.frame_len(cmd_f))
    expect("a partial commander waits", 0, B.frame_len(cmd_f[:10]))
    bad = cmd_f[:15] + bytes([(cmd_f[15] + 1) & 0xFF])
    expect("a bad commander checksum drops the head byte, never a shorter frame",
           -1, B.frame_len(bad))
    tricky = bytes([0x30, 0x30]) + bytes(13) + b"\x00"  # an inner l=2 match...
    expect("a commander frame is never split at an inner match",
           -1, B.frame_len(tricky))

    print("\n--- SppLink frames and deframes like the wire ---")
    a, b = socket.socketpair()
    link = B.SppLink("AA:BB:CC:DD:EE:FF", sock=a)
    link.connect()
    try:
        link.send(b"\x70\x07")
        expect("send appends the checksum", b"\x70\x07\x77", b.recv(64))
        b.sendall(frame(b"\x20\x01\x02"))
        expect("recv strips the checksum", b"\x20\x01\x02", link.recv())
        b.sendall(f1 + f2)
        expect("recv returns one frame per call", b"\x20\x05\x01", link.recv())
        expect("and then the next", b"\x20\x06\x02\x03", link.recv())
        check("silence reads as None", link.recv() is None)

        print("\n--- Drone drives a stream link end to end ---")
        def echo_once():
            data = b.recv(256)
            reply = data[:-1]                      # strip the wire checksum
            b.sendall(reply + bytes([sum(reply) & 0xFF]))
        threading.Thread(target=echo_once, daemon=True).start()
        d = D.Drone(link=link, timeout=1.5)
        try:
            expect("a request round-trips over SPP", b"\x03", d.request(2, 1, b"\x03"))
        finally:
            d.close()
    finally:
        try:
            b.close()
        except OSError:
            pass

    print("\n--- a dead link fails as OSError, never a bleak error ---")
    # bleak's BleakError is a bare Exception, so unless the link translates it
    # (the way _write_all does) it sails past every except in the GUI and
    # lands as "Exception in Tkinter callback". These checks pin the contract:
    # anything a dead or broken BLE link raises is an OSError.
    # BleakError của bleak là Exception trần, nên trừ khi link dịch nó (như
    # _write_all làm) nó sẽ lọt qua mọi except trong GUI và rơi thành
    # "Exception in Tkinter callback". Các kiểm này chốt hợp đồng: mọi thứ
    # liên kết BLE chết/lỗi ném ra đều là OSError.

    class BleakShapedError(Exception):
        pass                                  # same shape as bleak.exc.BleakError
                                              # cùng hình dạng với bleak.exc.BleakError

    class DyingClient:
        async def write_gatt_char(self, uuid, data, response=False):
            raise BleakShapedError("Service Discovery has not been performed yet")

    link = B.BleLink("ESP32_Done")
    try:
        link.send(b"\x70\x07")
        check("send before connect raises OSError", False)
    except OSError:
        check("send before connect raises OSError", True)

    link._client = DyingClient()
    link._loop = asyncio.new_event_loop()   # send() dispatches onto a live loop
    threading.Thread(target=link._loop.run_forever, daemon=True).start()
    try:
        link.send(b"\x70\x07")
        check("a bleak write error escapes as OSError", False)
    except OSError as e:
        check("a bleak write error escapes as OSError",
              "BLE link lost" in str(e))
    except BleakShapedError:
        check("a bleak write error escapes as OSError", False)
    check("and the link is marked dead", link._dead is not None)
    link._loop.call_soon_threadsafe(link._loop.stop)
    link._loop = None

    try:
        link.send(b"\x70\x07")
        check("a later send fails fast on the dead state", False)
    except OSError as e:
        expect("a later send fails fast on the dead state",
               "BLE link lost", str(e).split(" (")[0])
    try:
        link.recv()
        check("recv raises the same dead error", False)
    except OSError:
        check("recv raises the same dead error", True)

    link = B.BleLink("ESP32_Done")
    link.close()
    check("close() marks the link dead", link._dead is not None)
    try:
        link.send(b"\x70\x07")
        check("send after close() raises OSError", False)
    except OSError:
        check("send after close() raises OSError", True)


# ============================================================ maths tests


def test_maths():
    print("\n--- to_y / from_y ---")
    close("to_y(0) is 0", 0.0, M.to_y(0.0), 1e-12)
    close("from_y inverts to_y", 7.5, float(M.from_y(M.to_y(7.5))), 1e-9)
    close("a fall is capped", M.to_y(1e9), M.to_y(M.COST_CAP), 1e-12)

    print("\n--- latin hypercube ---")
    rng = np.random.default_rng(3)
    L = M.latin_hypercube(12, 3, rng)
    expect("shape", (12, 3), L.shape)
    check("inside the unit cube", bool(((L >= 0) & (L < 1)).all()))
    check("one sample per stratum on each axis",
          all(len(set((L[:, j] * 12).astype(int))) == 12 for j in range(3)))

    print("\n--- Space round-trip ---")
    names = D.axis_names("roll")
    centre = [2.0, 0.2, 15.0]
    sp = M.Space(names, [0, 1, 2], centre, 2.0)
    expect("three dimensions", 3, len(sp.ids))
    expect("names follow ids", ["pid.roll_kp", "pid.roll_ki", "pid.roll_kd"], sp.names)
    for k in range(3):
        close(f"centre maps to 0.5 (dim {k})", 0.5, float(sp.to_unit(centre)[k]), 1e-9)
        close(f"0.5 maps back to the centre (dim {k})", centre[k],
              sp.to_params([0.5, 0.5, 0.5])[k], 1e-6)
    check("a gain of 0 is rejected", sp.to_unit([0.0, 0.2, 15.0]) is None)
    check("outside the box is rejected", sp.to_unit([2.0, 0.2, 999.0]) is None)
    # the box must never leave the firmware's own clamp
    # hộp không bao giờ được vượt ra ngoài clamp của chính firmware
    for k in range(3):
        lo, hi = sp.lo[k], sp.hi[k]
        check(f"dim {k} box sits inside [P_MIN, P_MAX]",
              math.log(max(D.P_MIN[k], 1e-3 * D.P_MAX[k])) - 1e-9 <= lo
              and hi <= math.log(D.P_MAX[k]) + 1e-9)

    print("\n--- GaussianProcess ---")
    rng = np.random.default_rng(11)
    X = M.latin_hypercube(14, 2, rng)
    truth = lambda x: 3.0 + 8.0 * ((x[0] - 0.35) ** 2 + (x[1] - 0.6) ** 2)   # noqa: E731
    y = np.array([M.to_y(truth(x)) for x in X])
    gp = M.GaussianProcess(X, y)
    check("picked a length scale", gp.ls in M.GaussianProcess.LENGTHS)
    check("picked a noise", gp.noise in M.GaussianProcess.NOISES)
    mean, std = gp.predict(X)
    for j in range(len(X)):
        close(f"GP interpolates training point {j}", y[j], float(mean[j]), 0.35)
    close("uncertainty is small at a training point", 0.0, float(std[0]), 0.35)
    far, farstd = gp.predict(np.array([[0.95, 0.05]]))
    check("uncertainty grows away from the data", float(farstd[0]) > float(std[0]))
    grid = M.latin_hypercube(400, 2, np.random.default_rng(5))
    ei = gp.expected_improvement(grid, float(y.min()))
    best_pred = grid[int(np.argmax(ei))]
    close("EI points near the true minimum (x0)", 0.35, float(best_pred[0]), 0.3)
    close("EI points near the true minimum (x1)", 0.6, float(best_pred[1]), 0.3)

    print("\n--- MlpEnsemble ---")
    rng = np.random.default_rng(21)
    X = M.latin_hypercube(60, 2, rng)
    y = np.array([M.to_y(truth(x)) for x in X])
    net = M.MlpEnsemble(X, y, rng)
    expect("five members", M.MlpEnsemble.MEMBERS, len(net.nets))
    mean, std = net.predict(X[:5])
    err = float(np.abs(mean - y[:5]).mean())
    check(f"the ensemble learns the data (mean |err| {err:.3f})", err < 0.4)
    check("member spread is positive", bool((std > 0).all()))
    far_mean, far_std = net.predict(np.array([[0.02, 0.98]]))
    check("spread is a usable uncertainty estimate", float(far_std[0]) > 0.0)


# ============================================================ optimiser tests

BOWL_MIN = (2.0, 0.2, 15.0)


def bowl(params):
    """A synthetic cost with a known minimum, in the same currency as
    drone.cost_of(): small and positive near the minimum, and a tip-like 100+
    far away so to_y() and the GP see the same landscape they would on hardware."""
    e = sum(((p - m) / m) ** 2 for p, m in zip(params, BOWL_MIN))
    return 0.4 + 60.0 * e


class FakeDrone:
    """Just enough of Drone for GuiTuner.run() to drive a synthetic trial.

    The starting gains sit near the bowl minimum but not on it, and inside the
    search box (centre / 2 .. centre x 2) so the minimum is reachable — a start
    of (1, 1, 1) would put ki=0.2 and kd=15 outside the box and the test could
    not pass however well the optimiser worked.
    """

    START = (1.2, 0.3, 12.0)

    def __init__(self):
        self.values = dict(zip(D.axis_names("roll"), self.START))

    def get_param(self, name):
        return self.values[name]

    def set_param(self, name, value):
        self.values[name] = value
        return value

    def samples_since(self, t0):
        return []


def synthetic(cls):
    """Wrap a tuner so evaluate() scores the bowl instead of flying."""
    class Synthetic(cls):
        def wait_level(self, prompt=False):
            self.check_stop()          # the stop test still has to see the flag
                                       # bài kiểm tra dừng vẫn phải thấy cờ này

        def evaluate(self, stage, params):
            self.check_stop()
            self.params = [min(max(p, D.P_MIN[k]), D.P_MAX[k]) for k, p in enumerate(params)]
            self.tried = list(self.params)
            cost = bowl(self.params) * (1.0 + 0.01 * float(self.rng.standard_normal()))
            self.n += 1
            self.log.writerow([self.n, stage] + [f"{p:.5g}" for p in self.params]
                              + [f"{cost:.3f}", False] + ["0"] * len(M.METRIC_FIELDS))
            self.logf.flush()
            return cost, {"fell": False}
    Synthetic.__name__ = "Synthetic" + cls.__name__
    return Synthetic


TUNERS = [
    ("ml", M.BayesTuner), ("neural", M.NeuralTuner), ("rl", M.PolicyGradientTuner),
    ("cma-es", M.CmaesTuner), ("de", M.DifferentialEvolutionTuner),
    ("pso", M.PsoTuner), ("genetic", M.GeneticTuner), ("nelder-mead", M.NelderMeadTuner),
    ("spsa", M.SpsaTuner), ("twiddle", M.TwiddleTuner), ("random", M.RandomTuner),
]


def test_optimisers():
    print("\n--- every tuner finds the minimum of a synthetic bowl ---")
    print(f"    true minimum kp={BOWL_MIN[0]} ki={BOWL_MIN[1]} kd={BOWL_MIN[2]}")
    tmp = tempfile.mkdtemp(prefix="pidtune-")
    for name, cls in TUNERS:
        path = os.path.join(tmp, f"{name}.csv")
        posts = []
        tuner = synthetic(cls)(FakeDrone(), "roll", [0, 1, 2], 34, 1.0, 2.0, [],
                               path, threading.Event(),
                               lambda k, d: posts.append((k, d)), seed=7)
        tuner.run()
        done = [d for k, d in posts if k == "done"]
        expect(f"{name}: finished cleanly", 1, len(done))
        if not done:
            continue
        expect(f"{name}: reason is 'finished'", "finished", done[0]["reason"])
        best = done[0]["result"]
        for k in range(3):
            tol = 0.25 * BOWL_MIN[k]
            close(f"{name}: {D.gain_name('roll', k)} near the true value",
                  BOWL_MIN[k], best[k], tol)
        trials = [d for k, d in posts if k == "trial"]
        expect(f"{name}: used its trial budget", 34, len(trials))
        check(f"{name}: reported a predicted best",
              trials and trials[-1]["best_pred"] is not None)
        # every trial must record the model window the ModelWindow draws
        # mỗi lần thử phải ghi lại cửa sổ mô hình mà ModelWindow vẽ
        t = trials[-1]
        check(f"{name}: trial report carries the model",
              all(key in t for key in ("gp", "space", "X", "Y", "own", "fell_mask", "best_x")))


def test_history_seeding():
    print("\n--- history seeding ---")
    tmp = tempfile.mkdtemp(prefix="pidtune-")
    path = os.path.join(tmp, "past.csv")
    rows = [
        ["trial", "stage", "pid.roll_kp", "pid.roll_ki", "pid.roll_kd", "cost", "fell"],
        [1, "ml", "2.0", "0.2", "15.0", "0.50", "False"],
        [2, "ml", "2.1", "0.2", "15.0", "0.55", "False"],
        [3, "ml", "2.0", "0.2", "15.0", "120.0", "True"],
        [4, "ml", "9.9", "0.2", "15.0", "0.30", "False"],   # other gains differ
                                                            # các gain khác lệch đi
    ]
    with open(path, "w") as f:
        for r in rows:
            f.write(",".join(str(x) for x in r) + "\n")
    hist = M.read_history([path, os.path.join(tmp, "missing.csv")])
    expect("rows read (a missing file is skipped, not fatal)", 4, len(hist))
    expect("the tip is recorded as a tip", True, hist[2]["fell"])

    posts = []
    tuner = synthetic(M.BayesTuner)(FakeDrone(), "roll", [0], 4, 1.0, 2.0, hist,
                                    os.path.join(tmp, "run.csv"), threading.Event(),
                                    lambda k, d: posts.append((k, d)), seed=7)
    tuner.params = [2.0, 0.2, 15.0]
    sp = M.Space(D.axis_names("roll"), [0], tuner.params, 2.0)
    X, Y = tuner.seed(sp)
    # rows 1 and 3 match the other gains; row 4 has kp 9.9 which is the tuned
    # gain so it is *not* filtered by the "other gains" test; all three land.
    # dòng 1 và 3 khớp các gain khác; dòng 4 có kp 9.9 vốn là gain được tinh
    # chỉnh nên *không* bị lọc bởi phép kiểm "gain khác"; cả ba đều được nhận.
    expect("matching past trials seed the model", 3, len(X))
    check("a tip seeds a high cost", max(Y) > M.to_y(50))


def test_stop_and_errors():
    print("\n--- stop handling ---")
    ev = threading.Event()
    ev.set()
    posts = []
    tmp = tempfile.mkdtemp(prefix="pidtune-")
    tuner = synthetic(M.BayesTuner)(FakeDrone(), "roll", [0], 10, 1.0, 2.0, [],
                                    os.path.join(tmp, "s.csv"), ev,
                                    lambda k, d: posts.append((k, d)), seed=7)
    tuner.run()
    done = [d for k, d in posts if k == "done"][-1]
    expect("a stop reports 'stopped'", "stopped", done["reason"])
    check("the starting gains are reported", done["start"] is not None)


def main():
    for name, fn in [("wire", test_wire), ("link", test_link), ("btlink", test_btlink),
                     ("maths", test_maths),
                     ("optimisers", test_optimisers), ("history", test_history_seeding),
                     ("stop", test_stop_and_errors)]:
        try:
            fn()
        except Exception:
            FAILS.append(name)
            print(f"FAIL {name} raised:")
            traceback.print_exc()
    print("\n" + "=" * 60)
    if FAILS:
        print(f"{len(FAILS)} FAILING: " + ", ".join(FAILS))
        return 1
    print("all tests passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
