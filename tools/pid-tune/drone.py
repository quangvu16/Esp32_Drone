#!/usr/bin/env python3
"""CRTP client for the ESP32 drone — the transport half of the tuning tools.

Stands in for PID-Calibration's `autotune.Robot`, which talked to a balancing
robot over USB serial with its own command bytes. This speaks the same CRTP the
phone apps speak, over UDP: [CRTP bytes ...][uint8 checksum], checksum the low
byte of the sum of all CRTP bytes including the header (PROTOCOL.md section 1).

Wire map, all from PROTOCOL.md:

  port 2  parameters   ch 0 TOC, ch 1 read, ch 2 write
  port 5  logging      ch 0 TOC, ch 1 control, ch 2 data
  port 7  extensions   ch 0 flight control, ch 1 navigation

Nothing here is drone-specific beyond the parameter and log names, so the
tuners in ml_tune.py stay the same shape as the reference's.
"""
import math
import socket
import struct
import threading
import time

# --- framing ---------------------------------------------------------------

CRTP_MAX_SIZE = 30
UDP_DEVICE_PORT = 2390        # drone listens here
                              # drone lắng nghe ở đây
UDP_APP_PORT = 2399           # drone replies here
                              # drone trả lời ở đây

PORT_PARAMS, PORT_LOG, PORT_FLIGHT = 2, 5, 7
CH_TOC, CH_READ, CH_WRITE = 0, 1, 2
LOG_CH_TOC, LOG_CH_CTRL, LOG_CH_DATA = 0, 1, 2

CMD_TOC_ELEMENT, CMD_TOC_INFO = 0, 1

LOG_CREATE, LOG_APPEND, LOG_DELETE, LOG_START, LOG_STOP, LOG_RESET = 0, 1, 2, 3, 4, 5

# flight-control extension (port 7, ch 0), payload led by this magic
# phần mở rộng điều khiển bay (port 7, ch 0), payload mở đầu bởi magic này
FC_MAGIC = 0x70
FC_SET_MODE, FC_TAKEOFF, FC_LAND, FC_SET_HOVER = 0x01, 0x02, 0x03, 0x04
FC_EMERGENCY, FC_SET_HEADING, FC_ARM, FC_DISARM = 0x05, 0x06, 0x07, 0x08
FC_SAVE, FC_DEFAULTS, FC_LOAD = 0x09, 0x0A, 0x0B

# parameter type codes (port 2) — note these differ from the log ones
# mã kiểu tham số (port 2) — lưu ý chúng khác với mã kiểu log
PARAM_TYPES = {
    0x00: ("b", 1), 0x08: ("B", 1),
    0x01: ("<h", 2), 0x09: ("<H", 2),
    0x02: ("<i", 4), 0x0A: ("<I", 4),
    0x03: ("<q", 8), 0x0B: ("<Q", 8),
    0x06: ("<f", 4), 0x07: ("<d", 8),
}
# log type codes (port 5) — deliberately not the same numbering
# mã kiểu log (port 5) — cố ý không dùng chung một cách đánh số
LOG_TYPES = {
    0x01: ("B", 1), 0x02: ("<H", 2), 0x03: ("<I", 4), 0x04: ("b", 1),
    0x05: ("<h", 2), 0x06: ("<i", 4), 0x07: ("<f", 4),
}

# --- what the tuner drives -------------------------------------------------

# One axis at a time, like the apps' axis picker. A gain is addressed as
# (axis, k) with k in 0..2, and named pid.<axis>_<kp|ki|kd>; the name is looked
# up in the parameter TOC at connect time, so a TOC change cannot silently
# write the wrong parameter.
# Một trục mỗi lần, giống trình chọn trục của các app. Một gain được chỉ định
# theo (axis, k) với k trong 0..2, và tên pid.<axis>_<kp|ki|kd>; tên được tra
# trong TOC tham số ngay khi kết nối, nên TOC đổi cũng không thể ghi nhầm
# tham số một cách âm thầm.
AXES = ("roll", "pitch", "yaw")
GAIN_LABELS = ("kp", "ki", "kd")


def gain_name(axis, k):
    return "pid.%s_%s" % (axis, GAIN_LABELS[k])


def axis_names(axis):
    return [gain_name(axis, k) for k in range(3)]


# The firmware clamps every write into the parameter's [lo, hi] (paramClampF),
# so these are the real limits rather than a guess: kp 0..100, ki 0..20,
# kd 0..200 on all three axes. Indexed by k. They are also the outer edge of
# the tuner's search box; Space narrows it to current / range .. x range.
# Firmware kẹp mọi lần ghi vào khoảng [lo, hi] của tham số (paramClampF), nên
# đây là giới hạn thật chứ không phải phỏng đoán: kp 0..100, ki 0..20, kd 0..200
# trên cả ba trục. Đánh chỉ mục theo k. Chúng cũng là mép ngoài của miền tìm
# kiếm của tuner; Space thu hẹp còn current / range .. x range.
P_MIN = (0.0, 0.0, 0.0)
P_MAX = (100.0, 20.0, 200.0)

TIP_DEG = 25.0                # firmware's MAX_TILT_DEG; past this a trial is over
                              # MAX_TILT_DEG của firmware; vượt mức này thì lần thử kết thúc
LEVEL_DEG = 15.0              # both axes inside this counts as upright
                              # hai trục nằm trong khoảng này được coi là đứng thẳng
MOTOR_SAT_HI, MOTOR_SAT_LO = 1990, 1010   # matches the apps' saturation test
                                          # khớp với phép kiểm bão hòa của các app


def tipped(frame):
    """The drone has lost control. On a stand this flags wild oscillation; in the
    air it flags an actual fall. Same role as autotune.py's fallen()."""
    return abs(frame.get("roll", 0)) > TIP_DEG or abs(frame.get("pitch", 0)) > TIP_DEG


fallen = tipped                     # the name ml_tune and the reference use
                                    # tên mà ml_tune và bản tham chiếu dùng


def cost_of(frames, trial_s):
    """One trial's score. The surviving-window terms are AutoTuner.costOf() on both
    apps, so a tuning run means the same thing whichever tool produced it:

        angleStd + 0.1*rateRms + 0.2*motorRms + 0.005*controlEffort
                 + 20*saturatedShare

    A tip scores 100 + 10 x seconds lost (autotune.py's measure()), which keeps
    falls "much worse" in the same currency as everything else so the GP learns
    to avoid them instead of being handed a short, deceptively clean window.

    Returns (cost, metrics).
    """
    if not frames:
        raise RuntimeError("no telemetry received")

    # Score only the surviving window: cut at the first tipped frame, but keep
    # at least 5 frames so the statistics stay defined after a very quick tip.
    # Chỉ chấm điểm trên cửa sổ còn sống: cắt tại frame nghiêng đầu tiên, nhưng
    # giữ ít nhất 5 frame để các thống kê vẫn có nghĩa khi nghiêng quá nhanh.
    bad_at = next((i for i, f in enumerate(frames) if tipped(f)), None)
    ok = frames[:bad_at] if bad_at is not None else frames
    if len(ok) < 5:
        ok = frames[:5]

    angle = [f["angle"] for f in ok]
    mean = sum(angle) / len(angle)
    angle_std = math.sqrt(sum((a - mean) ** 2 for a in angle) / len(angle))
    rate_rms = math.sqrt(sum(f["rate"] ** 2 for f in ok) / len(ok))
    motor_rms = math.sqrt(sum((f["motor"] - 1500.0) ** 2 for f in ok) / len(ok))
    control_effort = sum(abs(f["control"]) for f in ok) / len(ok)
    sat = sum(1 for f in ok
              if f["motor"] >= MOTOR_SAT_HI or f["motor"] <= MOTOR_SAT_LO) / len(ok)

    m = {"fell": bad_at is not None, "n": len(frames), "angle_mean": mean,
         "angle_std": angle_std, "rate_rms": rate_rms, "motor_rms": motor_rms,
         "control_effort": control_effort, "sat": sat}
    if m["fell"]:
        survived = bad_at / max(len(frames), 1) * trial_s
        # Fall penalty: a crash is worse the earlier it happens, so the cost is
        # dominated by survival time rather than by the quality of the flight.
        # Phạt nghiêng: rơi càng sớm càng tệ, nên chi phí bị chi phối bởi thời gian
        # sống sót hơn là chất lượng của chuyến bay.
        m["cost"] = 100 + 10 * (trial_s - survived)
    else:
        m["cost"] = (angle_std + 0.1 * rate_rms + 0.2 * motor_rms
                     + 0.005 * control_effort + 20 * sat)
    return m["cost"], m


def measure(drone, axis, trial_s):
    """One trial with the parameters already applied: settle, measure, score.
    The trial ends early the moment the drone tips."""
    time.sleep(1.5)                       # settle after the parameter change
                                          # chờ ổn định sau khi đổi tham số
    t0 = time.monotonic()
    while time.monotonic() - t0 < trial_s:
        time.sleep(0.1)
        if any(tipped(f) for f in drone.samples_since(t0)):
            break
    return cost_of(drone.samples_since(t0), trial_s)


# --- the link --------------------------------------------------------------


class Toc:
    """CRTP TOC: id -> (group, name, typeCode, size, readOnly)."""

    def __init__(self):
        self.by_id = {}
        self.by_name = {}
        self.crc = 0

    def add(self, i, group, name, type_code, read_only):
        fmt_size = PARAM_TYPES.get(type_code) or LOG_TYPES.get(type_code)
        size = fmt_size[1] if fmt_size else 0
        self.by_id[i] = (group, name, type_code, size, read_only)
        self.by_name[group + "." + name] = i

    def id_of(self, complete_name):
        if complete_name not in self.by_name:
            raise KeyError("not in TOC: " + complete_name)
        return self.by_name[complete_name]


class Drone:
    """CRTP over UDP. Thread-safe; telemetry frames are buffered for measure()."""

    def __init__(self, host="192.168.43.42", device_port=UDP_DEVICE_PORT,
                 app_port=UDP_APP_PORT, timeout=1.0):
        self.host = host
        self.timeout = timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", app_port))
        self.sock.settimeout(0.05)
        self._sendto = (host, device_port)
        self._lock = threading.Lock()
        self._pending = {}            # (port, channel) -> list of raw replies
        self._samples = []            # decoded telemetry, oldest first
        self._frame_lock = threading.Lock()
        self._decoders = {}           # block id -> raw values -> sample dict | None
        self.level = {"roll": 0.0, "pitch": 0.0}   # status block, stamped into samples
        self.param_toc = Toc()
        self.log_toc = Toc()
        self._running = True
        self._rx = threading.Thread(target=self._reader, daemon=True)
        self._rx.start()

    # --- transport ---

    def close(self):
        self._running = False
        try:
            self.sock.close()
        except OSError:
            pass

    @staticmethod
    def _checksum(payload):
        return sum(payload) & 0xFF

    def send(self, port, channel, payload):
        raw = bytes([((port & 0xF) << 4) | (channel & 0xF)]) + bytes(payload)
        pkt = raw + bytes([self._checksum(raw)])
        with self._lock:
            self.sock.sendto(pkt, self._sendto)

    def _reader(self):
        while self._running:
            try:
                data, _ = self.sock.recvfrom(512)
            except socket.timeout:
                continue
            except OSError:
                break
            if len(data) < 2:
                continue
            raw, csum = data[:-1], data[-1]
            if self._checksum(raw) != csum:
                continue                        # corrupt; drop
                                                # hỏng; bỏ qua
            header = raw[0]
            port, channel, payload = header >> 4, header & 0xF, raw[1:]
            if port == PORT_LOG and channel == LOG_CH_DATA:
                self._handle_log_data(payload)
            else:
                with self._lock:
                    self._pending.setdefault((port, channel), []).append(payload)

    def request(self, port, channel, payload, timeout=None, want=None):
        """Send and wait for one reply on (port, channel). `want(payload)` can
        reject a reply and keep waiting (for example a different param id)."""
        deadline = time.monotonic() + (timeout if timeout is not None else self.timeout)
        with self._lock:
            self._pending.pop((port, channel), None)
        self.send(port, channel, payload)
        while time.monotonic() < deadline:
            with self._lock:
                inbox = self._pending.get((port, channel))
                if inbox:
                    reply = inbox.pop(0)
                else:
                    reply = None
            if reply is not None and (want is None or want(reply)):
                return reply
            time.sleep(0.005)
        raise TimeoutError("no reply on port %d ch %d" % (port, channel))

    # --- parameters ---

    def fetch_toc(self, port, toc):
        info = self.request(port, CH_TOC, bytes([CMD_TOC_INFO]))
        if len(info) < 6 or info[0] != CMD_TOC_INFO:
            raise RuntimeError("bad TOC info on port %d" % port)
        n, crc = info[1], struct.unpack("<I", info[2:6])[0]
        toc.by_id.clear()
        toc.by_name.clear()
        toc.crc = crc
        for i in range(n):
            r = self.request(port, CH_TOC, bytes([CMD_TOC_ELEMENT, i]),
                             want=lambda p, i=i: len(p) > 1 and p[0] == CMD_TOC_ELEMENT and p[1] == i)
            body = r[2:]
            z1 = body.find(b"\0")
            z2 = body.find(b"\0", z1 + 1)
            if z1 < 0 or z2 < 0:
                continue
            type_access = body[0]
            toc.add(i, body[1:z1].decode("utf-8", "replace"),
                    body[z1 + 1:z2].decode("utf-8", "replace"),
                    type_access & 0x0F, bool(type_access & 0x40))
        return toc

    def get_param(self, complete_name):
        i = self.param_toc.id_of(complete_name)
        r = self.request(PORT_PARAMS, CH_READ, bytes([i]),
                         want=lambda p: len(p) > 0 and p[0] == i)
        code = self.param_toc.by_id[i][2]
        fmt, size = PARAM_TYPES[code]
        return struct.unpack(fmt, bytes(r[1:1 + size]))[0]

    def set_param(self, complete_name, value):
        i = self.param_toc.id_of(complete_name)
        code = self.param_toc.by_id[i][2]
        fmt, size = PARAM_TYPES[code]
        body = bytes([i]) + struct.pack(fmt, value)
        r = self.request(PORT_PARAMS, CH_WRITE, body,
                         want=lambda p: len(p) > 0 and p[0] == i)
        return struct.unpack(fmt, bytes(r[1:1 + size]))[0]

    # --- logging ---

    def _log_ctrl(self, cmd, args=b"", timeout=None):
        return self.request(PORT_LOG, LOG_CH_CTRL, bytes([cmd]) + args, timeout=timeout)

    def log_create(self, block_id, variables, period_ms):
        """variables: list of complete log names. `period_ms` is rounded to the
        firmware's 10 ms ticks, as the apps do."""
        args = bytes([block_id])
        for name in variables:
            i = self.log_toc.id_of(name)
            args += bytes([self.log_toc.by_id[i][2] & 0x0F, i])
        r = self._log_ctrl(LOG_CREATE, args)
        if not r or r[-1] != 0:
            raise RuntimeError("log CREATE failed, err=%r" % (r[-1] if r else None))
        ticks = max(1, int(round(period_ms / 10.0)))
        r = self._log_ctrl(LOG_START, bytes([block_id, ticks & 0xFF]))
        if not r or r[-1] != 0:
            raise RuntimeError("log START failed, err=%r" % (r[-1] if r else None))

    def log_stop(self, block_id):
        try:
            self._log_ctrl(LOG_STOP, bytes([block_id]))
            self._log_ctrl(LOG_DELETE, bytes([block_id]))
        except TimeoutError:
            pass                                    # already gone; re-acking is idempotent
                                                    # đã biến mất rồi; ack lại là idempotent

    def _handle_log_data(self, payload):
        if len(payload) < 4:
            return
        block = payload[0]
        # timestamp is 24-bit LE, then values; we do not need it for tuning
        # timestamp là 24-bit LE, sau đó là giá trị; ta không cần nó khi tinh chỉnh
        decode = self._decoders.get(block)
        if decode is None:
            return
        sample = decode(payload[4:])
        if sample is not None:
            sample["t"] = time.monotonic()
            with self._frame_lock:
                self._samples.append(sample)

    def samples_since(self, t0):
        """Decoded telemetry samples, oldest first, from t0 onwards. Each is a
        dict of angle / rate / control / motor / roll / pitch plus "t", the host
        monotonic clock it arrived on."""
        with self._frame_lock:
            return [s for s in self._samples if s["t"] >= t0]

    def clear_samples(self):
        with self._frame_lock:
            self._samples.clear()

    # --- flight-control extension ---

    def _fc(self, cmd, args=b""):
        return self.request(PORT_FLIGHT, 0, bytes([FC_MAGIC, cmd]) + args,
                            want=lambda p: len(p) >= 2 and p[0] == FC_MAGIC and p[1] == cmd)

    def arm(self):
        return self._fc(FC_ARM)[-1] == 0

    def disarm(self):
        return self._fc(FC_DISARM)[-1] == 0

    def emergency(self):
        return self._fc(FC_EMERGENCY)[-1] == 0

    def save(self):
        return self._fc(FC_SAVE)[-1] == 0

    def load(self):
        return self._fc(FC_LOAD)[-1] == 0

    def defaults(self):
        return self._fc(FC_DEFAULTS)[-1] == 0


# --- telemetry blocks ------------------------------------------------------

# Two blocks, exactly as TuneActivity.java builds them: a status block carrying
# the level angles the tip check needs, and one per-axis tune block carrying the
# samples cost_of() scores. Values are packed in request order (logSendData),
# so the struct layouts below match TUNE_VARS / STATUS_VARS entry for entry.
# LOG_MAX_PAYLOAD is 25 bytes of values; both blocks fit.
# Hai khối, đúng như TuneActivity.java dựng: một khối status mang các góc nghiêng
# mà phép kiểm nghiêng cần, và mỗi trục một khối tune mang các mẫu mà cost_of()
# chấm điểm. Giá trị được đóng gói theo thứ tự request (logSendData), nên bố cục
# struct bên dưới khớp từng mục với TUNE_VARS / STATUS_VARS. LOG_MAX_PAYLOAD là
# 25 byte giá trị; cả hai khối đều vừa.

STATUS_BLOCK_ID = 0
TUNE_BLOCK_ID = 1
TUNE_PERIOD_MS = 50                    # 20 Hz, the apps' chart rate

STATUS_VARS = ["pm.vbat", "flight.mode", "flight.armed", "flight.auto_state",
               "stabilizer.roll", "stabilizer.pitch"]
STATUS_STRUCT = struct.Struct("<fBBBff")          # 15 B of values

TUNE_VARS = {
    "roll":  ["pid.roll_p", "pid.roll_i", "pid.roll_d",
              "stabilizer.roll", "controller.roll", "motor.m1"],
    "pitch": ["pid.pitch_p", "pid.pitch_i", "pid.pitch_d",
              "stabilizer.pitch", "controller.pitch", "motor.m1"],
    "yaw":   ["pid.yaw_p", "pid.yaw_i", "pid.yaw_d",
              "stabilizer.yaw", "controller.yaw", "motor.m1"],
}
TUNE_STRUCT = struct.Struct("<fffffH")            # 22 B of values


def stop_log_blocks(drone):
    """Remove both blocks so the next open_tune_blocks() can CREATE them again.
    A CREATE over a block with different variables is refused (err 17), so this
    has to succeed first."""
    for block in (TUNE_BLOCK_ID, STATUS_BLOCK_ID):
        drone.log_stop(block)
    drone._decoders.clear()


def open_tune_blocks(drone, axis, period_ms=TUNE_PERIOD_MS):
    """Start the status block and this axis's tune block.

    Decoded samples land in drone.samples_since() as dicts of
    angle / rate / control / motor / roll / pitch.
    """
    if axis not in TUNE_VARS:
        raise ValueError("axis must be one of " + ", ".join(TUNE_VARS))
    stop_log_blocks(drone)
    drone.clear_samples()

    def status_decode(raw):
        if len(raw) < STATUS_STRUCT.size:
            return None
        _vbat, _mode, _armed, _auto, roll, pitch = STATUS_STRUCT.unpack(raw[:STATUS_STRUCT.size])
        drone.level["roll"], drone.level["pitch"] = roll, pitch
        return None                     # not a cost sample
                                        # không phải mẫu chi phí

    prev_angle = [None]

    def tune_decode(raw):
        if len(raw) < TUNE_STRUCT.size:
            return None
        p, i, d, angle, control, motor = TUNE_STRUCT.unpack(raw[:TUNE_STRUCT.size])
        # The drone logs no roll/pitch gyro rates (only stabilizer.yaw is a
        # rate), so roll/pitch rates are finite-differenced from consecutive
        # angle samples. A deliberate, honest substitution — not a bug.
        # Drone không log tốc độ con quay roll/pitch (chỉ stabilizer.yaw là
        # tốc độ), nên tốc độ roll/pitch được sai phân hữu hạn từ các mẫu góc
        # liên tiếp. Một sự thay thế có chủ đích, trung thực — không phải bug.
        if axis == "yaw":
            rate = angle
        else:
            rate = (0.0 if prev_angle[0] is None
                    else (angle - prev_angle[0]) * (1000.0 / period_ms))
        prev_angle[0] = angle
        return {"angle": angle, "rate": rate, "control": control, "motor": motor,
                "p": p, "i": i, "d": d,
                "roll": drone.level["roll"], "pitch": drone.level["pitch"]}

    drone._decoders[STATUS_BLOCK_ID] = status_decode
    drone._decoders[TUNE_BLOCK_ID] = tune_decode
    drone.log_create(STATUS_BLOCK_ID, STATUS_VARS, period_ms)
    drone.log_create(TUNE_BLOCK_ID, TUNE_VARS[axis], period_ms)
