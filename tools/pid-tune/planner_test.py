#!/usr/bin/env python3
"""Unit tests for planner.py — the QGC WPL 110 codec, the mission wire and the
transfer state machine (PROTOCOL.md §12).

The codec pins mirror the Android CodecCheck.java driver (32 checks) and the
iOS PlannerCodecTests: a Mission Planner file must round-trip byte-for-byte and
every reader quirk must behave exactly like Mission Planner's.
Các ghim codec phản chiếu driver CodecCheck.java của Android (32 phép kiểm) và
PlannerCodecTests của iOS: tệp Mission Planner phải đi-về nguyên vẹn từng byte
và mọi điểm lạ của trình đọc phải đúng như của Mission Planner.

  python3 planner_test.py
"""
import struct
import unittest

from planner import (CMD_ACK, CMD_COUNT, CMD_CURRENT, CMD_DO_JUMP, CMD_ITEM,
                     CMD_REQUEST, CMD_REQUEST_LIST, CMD_SET_CURRENT, CMD_START,
                     CMD_TAKEOFF, CMD_WAYPOINT, FRAME_GLOBAL_RELATIVE_ALT,
                     MISSION_ACCEPTED, MISSION_ERROR, MISSION_INVALID,
                     MissionPacket, MissionTransfer, PlannerItem, PlannerMission,
                     _e7, parse_command, command_name_of, to_lat_lon,
                     to_north_east)


class CodecTests(unittest.TestCase):
    """The 32 CodecCheck pins. / 32 ghim CodecCheck."""

    # 1. A Mission-Planner-written file must round-trip byte-for-byte.
    #    Number formats: home lat/lng 7 decimals, row p/lat/lon 8, alt 6.
    # Tệp do Mission Planner viết phải đi-về nguyên vẹn từng byte.
    # Định dạng số: lat/lng của home 7 chữ số thập phân, p/lat/lon của dòng 8,
    # alt 6.
    FILE = ("QGC WPL 110\n"
            "0\t1\t0\t16\t0\t0\t0\t0\t10.7626220\t106.6601720\t30.000000\t1\n"
            "1\t0\t3\t22\t0.00000000\t2.50000000\t0.00000000\t0.00000000\t10.76262200\t106.66017200\t30.000000\t1\n"
            "2\t0\t3\t16\t0.00000000\t2.50000000\t0.00000000\t0.00000000\t10.76270000\t106.66020000\t25.500000\t1\n"
            "3\t0\t3\t177\t1.00000000\t3.00000000\t0.00000000\t0.00000000\t0.00000000\t0.00000000\t0.000000\t1\n")

    def test_round_trip(self):
        mission = PlannerMission.parse(self.FILE)
        self.assertEqual(mission.invalid_lines, [])
        self.assertEqual(mission.size(), 3)
        self.assertEqual(mission.home.lat, 10.7626220)
        self.assertEqual(mission.home.lon, 106.6601720)
        self.assertEqual(mission.home.alt, 30.0)
        self.assertEqual(mission.items[0].command, CMD_TAKEOFF)
        self.assertEqual(mission.items[0].p2, 2.5)
        self.assertEqual(mission.items[2].command, CMD_DO_JUMP)
        self.assertEqual(mission.items[2].p1, 1.0)
        self.assertEqual(mission.items[2].p2, 3.0)
        self.assertEqual(mission.format(), self.FILE)

    # 2. The blank-home quirk: a first row that is not index 0 inserts a blank
    #    home, and the comment/separator rules of the reader.
    # Điểm lạ home trống: dòng đầu không phải index 0 sẽ chèn một home trống,
    # cùng các quy tắc chú thích/dấu phân cách của trình đọc.
    def test_blank_home_quirk(self):
        quirky = ("QGC WPL 110\n"
                  "# a comment line\n"
                  "1, 0, 3, WAYPOINT, 0, 2.5, 0, 0, 1.5, 2.5, 10.0, 1\n"
                  "  2 0 3 16 0 2.5 0 0 3.5 4.5 12.0 1\n")
        mission = PlannerMission.parse(quirky)
        self.assertEqual(mission.invalid_lines, [])
        self.assertEqual(mission.size(), 2)
        self.assertEqual(mission.home.lat, 0)
        self.assertEqual(mission.home.lon, 0)
        self.assertEqual(mission.home.frame, 0)
        # The leading separator of the space row must not shift its fields —
        # RemoveEmptyEntries drops that empty token.
        # Dấu phân cách đầu dòng của dòng cách-spaces không được đẩy lệch các
        # trường — RemoveEmptyEntries bỏ token rỗng đó.
        self.assertEqual(mission.items[0].command, CMD_WAYPOINT)
        self.assertEqual(mission.items[0].lat, 1.5)
        self.assertEqual(mission.items[0].p2, 2.5)
        self.assertEqual(mission.items[1].lat, 3.5)
        self.assertEqual(mission.items[1].alt, 12.0)

    # 3. Command 99 is rewritten to 0 (the legacy waypoint id).
    # Lệnh 99 được viết lại thành 0 (id waypoint kiểu cũ).
    def test_legacy_command_99(self):
        legacy = ("QGC WPL 110\n"
                  "0\t1\t0\t16\t0\t0\t0\t0\t1.0000000\t2.0000000\t3.000000\t1\n"
                  "1\t0\t3\t99\t0\t0\t0\t0\t5.00000000\t6.00000000\t7.000000\t1\n")
        mission = PlannerMission.parse(legacy)
        self.assertEqual(mission.items[0].command, 0)

    # 4. Bad rows are skipped and reported; the rest still loads.
    # Dòng sai bị bỏ qua và được báo cáo; phần còn lại vẫn nạp.
    def test_invalid_line_reported(self):
        bad = ("QGC WPL 110\n"
               "0\t1\t0\t16\t0\t0\t0\t0\t1.0000000\t2.0000000\t3.000000\t1\n"
               "1\t0\t3\tNOT_A_COMMAND\t0\t0\t0\t0\t5.00000000\t6.00000000\t7.000000\t1\n"
               "2\t0\t3\t16\t0\t0\t0\t0\t8.00000000\t9.00000000\t10.000000\t1\n")
        mission = PlannerMission.parse(bad)
        self.assertEqual(len(mission.invalid_lines), 1)
        self.assertEqual(mission.size(), 1)
        self.assertEqual(mission.items[0].lat, 8.0)

    # 5. A file without the QGC WPL header is refused.
    # Tệp thiếu header QGC WPL bị từ chối.
    def test_header_refused(self):
        with self.assertRaises(ValueError):
            PlannerMission.parse("not a mission file\n")

    # 6. Short rows are skipped silently (<= 9 fields), like Mission Planner.
    # Dòng ngắn bị bỏ qua im lặng (<= 9 trường), như Mission Planner.
    def test_short_rows_skipped_silently(self):
        short_rows = ("QGC WPL 110\n"
                      "junk line\n"
                      "0\t1\t0\t16\t0\t0\t0\t0\t1.0000000\t2.0000000\t3.000000\t1\n")
        mission = PlannerMission.parse(short_rows)
        self.assertEqual(mission.invalid_lines, [])
        self.assertEqual(mission.home.lat, 1.0)

    # 7. The writer's fallback home row for a non-finite home.
    # Dòng home dự phòng của trình ghi khi home không hữu hạn.
    def test_fallback_home_row(self):
        mission = PlannerMission()
        mission.home.lat = float("nan")
        self.assertEqual(mission.format(),
                         "QGC WPL 110\n0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1\n")

    # 8. Name table pins (the MAV_CMD enum from common.xml).
    # Ghim bảng tên (enum MAV_CMD từ common.xml).
    def test_command_name_table(self):
        self.assertEqual(parse_command("WAYPOINT"), 16)
        self.assertEqual(parse_command("DO_JUMP"), 177)
        # NAV_SPLINE_WAYPOINT displays as SPLINE_WAYPOINT.
        # NAV_SPLINE_WAYPOINT hiển thị là SPLINE_WAYPOINT.
        self.assertEqual(parse_command("SPLINE_WAYPOINT"), 82)
        # Case-sensitive like Enum.Parse.
        # Phân biệt hoa thường như Enum.Parse.
        self.assertEqual(parse_command("waypoint"), -1)
        self.assertEqual(parse_command("NOPE"), -1)
        # Enum.Parse takes any number, even one with no enum member.
        # Enum.Parse nhận mọi con số, kể cả số không có thành viên enum.
        self.assertEqual(parse_command("31000"), 31000)
        self.assertEqual(command_name_of(12345), "UNKNOWN")
        self.assertEqual(command_name_of(16), "WAYPOINT")

    def test_blank_home_stays_after_failed_rows(self):
        # A failed first row keeps its blank home — and one blank can repeat per
        # failed row, so a file of failures still has rows.
        # Dòng đầu thất bại vẫn giữ home trống — và một home trống có thể lặp
        # theo từng dòng thất bại, nên tệp toàn thất bại vẫn có các dòng.
        text = ("QGC WPL 110\n"
                "1\t0\t3\tNOPE\t0\t0\t0\t0\t5.0\t6.0\t7.0\t1\n"
                "2\t0\t3\tALSO_NOPE\t0\t0\t0\t0\t5.0\t6.0\t7.0\t1\n")
        mission = PlannerMission.parse(text)
        self.assertEqual(len(mission.invalid_lines), 2)
        self.assertEqual(mission.size(), 1)      # the second blank home is a row
        self.assertEqual(mission.home.lat, 0)
        self.assertEqual(mission.items[0].lat, 0)


class WireTests(unittest.TestCase):
    """MissionPacket layouts (PROTOCOL.md §12). / Bố cục MissionPacket."""

    def test_item_wire_layout(self):
        item = PlannerItem.waypoint(10.762622, 106.660172, 25.5)
        item.command = CMD_TAKEOFF
        item.p1, item.p2 = 1.5, 2.5
        wire = MissionPacket.item(item, 3).wire()
        self.assertEqual(len(wire), 25)          # cmd + 24 bytes of args
        self.assertEqual(wire[0], CMD_ITEM)
        seq, cmd, p1, p2, lat_e7, lon_e7, alt = struct.unpack("<HHffiif", wire[1:])
        self.assertEqual((seq, cmd), (3, 22))
        self.assertEqual((p1, p2), (1.5, 2.5))
        self.assertEqual(lat_e7, 107626220)      # degrees × 1e7, MISSION_ITEM_INT
        self.assertEqual(lon_e7, 1066601720)
        self.assertEqual(alt, 25.5)

    def test_item_round_trip_p3_p4_zeroed(self):
        item = PlannerItem.waypoint(1.5, 2.5, 10.0)
        item.p1, item.p3, item.p4 = 7.5, -1.25, 9.0
        back = MissionPacket.parse_item(MissionPacket.item(item, 0).wire())
        self.assertEqual(back.command, CMD_WAYPOINT)
        self.assertEqual((back.p1, back.p2), (7.5, 0.0))
        self.assertEqual((back.p3, back.p4), (0.0, 0.0))   # the §12 wire note
        self.assertEqual((back.lat, back.lon), (1.5, 2.5))
        self.assertEqual(back.alt, 10.0)
        self.assertEqual(back.frame, FRAME_GLOBAL_RELATIVE_ALT)

    def test_e7_half_up(self):
        # Java's Math.round is half-up; Python's round() is half-even. 2^-8 is
        # the exact tie (2^-8 × 1e7 = 39062.5) so the two would disagree here.
        # Math.round của Java làm tròn nửa lên; round() của Python làm tròn nửa
        # chẵn. 2^-8 là mốc nửa chính xác (2^-8 × 1e7 = 39062.5) nên hai cách
        # sẽ khác nhau tại đây.
        self.assertEqual(_e7(0.00390625), 39063)
        self.assertEqual(_e7(-0.00390625), -39062)

    def test_small_parsers(self):
        self.assertEqual(MissionPacket.parse_u16(b"\x34\x12", 0), 0x1234)
        self.assertEqual(MissionPacket.parse_u16(b"\x01", 0), -1)
        self.assertEqual(MissionPacket.parse_u8(b"\x05", 0), 5)
        self.assertEqual(MissionPacket.parse_u8(b"", 0), -1)

    def test_command_packets(self):
        self.assertEqual(MissionPacket.count(2).wire(), bytes([CMD_COUNT, 2, 0]))
        self.assertEqual(MissionPacket.request(1).wire(), bytes([CMD_REQUEST, 1, 0]))
        self.assertEqual(MissionPacket.request_list().wire(), bytes([CMD_REQUEST_LIST]))
        self.assertEqual(MissionPacket.set_current(3).wire(),
                         bytes([CMD_SET_CURRENT, 3, 0]))
        self.assertEqual(MissionPacket.start().wire(), bytes([CMD_START]))


class _FakeTimer:
    """Tk-free root.after stand-in: collect what is scheduled.
    Bản thay root.after không cần Tk: gom các lời hẹn giờ."""

    def __init__(self):
        self._next = 0
        self.pending = {}

    def schedule(self, ms, fn):
        self._next += 1
        self.pending[self._next] = (ms, fn)
        return self._next

    def cancel(self, handle):
        self.pending.pop(handle, None)

    def fire(self):
        due = [fn for _ms, fn in self.pending.values()]
        self.pending.clear()
        for fn in due:
            fn()


class _Recorder:
    def __init__(self):
        self.events = []

    def on_transfer_status(self, message):
        self.events.append(("status", message))

    def on_upload_finished(self, result):
        self.events.append(("upload", result))

    def on_download_finished(self, mission, result):
        self.events.append(("download", mission, result))

    def on_command_finished(self, cmd, ok):
        self.events.append(("cmd", cmd, ok))

    def on_current_item(self, seq):
        self.events.append(("current", seq))


def _make_transfer():
    sent, timer, rec = [], _FakeTimer(), _Recorder()
    transfer = MissionTransfer(rec, timer.schedule, timer.cancel)
    transfer.attach(sent.append)               # every wire lands in sent
    return transfer, sent, timer, rec


class TransferTests(unittest.TestCase):
    """The MISSION_* state machine (PROTOCOL.md §12).
    Máy trạng thái MISSION_* (PROTOCOL.md mục 12)."""

    def test_upload_handshake(self):
        t, sent, timer, rec = _make_transfer()
        a = PlannerItem.waypoint(1.5, 2.5, 10.0)
        b = PlannerItem.waypoint(1.6, 2.6, 11.0)
        t.upload([a, b])
        self.assertEqual(sent[-1], MissionPacket.count(2).wire())
        self.assertEqual(rec.events[0], ("status", "Uploading 0/2"))
        # drone asks for each item, then ACKs — the MAVLink handshake.
        # drone hỏi từng mục, rồi ACK — bắt tay kiểu MAVLink.
        t.on_reply(bytes([CMD_REQUEST]) + struct.pack("<H", 0))
        self.assertEqual(sent[-1], MissionPacket.item(a, 0).wire())
        t.on_reply(bytes([CMD_REQUEST]) + struct.pack("<H", 1))
        self.assertEqual(sent[-1], MissionPacket.item(b, 1).wire())
        t.on_reply(bytes([CMD_ACK, MISSION_ACCEPTED]))
        self.assertIn(("status", "Mission uploaded"), rec.events)
        self.assertIn(("upload", MISSION_ACCEPTED), rec.events)
        self.assertFalse(t.is_busy())

    def test_upload_clear(self):
        # COUNT with 0 items clears the store and ACKs immediately.
        # COUNT với 0 mục xóa kho và ACK ngay.
        t, sent, timer, rec = _make_transfer()
        t.upload([])
        self.assertEqual(sent[-1], MissionPacket.count(0).wire())
        self.assertIn(("status", "Clearing stored mission"), rec.events)
        t.on_reply(bytes([CMD_ACK, MISSION_ACCEPTED]))
        self.assertIn(("upload", MISSION_ACCEPTED), rec.events)

    def test_upload_out_of_range_seq_is_invalid(self):
        t, sent, timer, rec = _make_transfer()
        t.upload([PlannerItem.waypoint(1, 2, 3)])
        t.on_reply(bytes([CMD_REQUEST]) + struct.pack("<H", 7))
        self.assertIn(("upload", MISSION_INVALID), rec.events)
        self.assertFalse(t.is_busy())

    def test_download_handshake(self):
        t, sent, timer, rec = _make_transfer()
        t.download()
        self.assertEqual(sent[-1], MissionPacket.request_list().wire())
        t.on_reply(bytes([CMD_COUNT]) + struct.pack("<H", 2))
        self.assertEqual(sent[-1], MissionPacket.request(0).wire())
        takeoff = PlannerItem.waypoint(10.0, 20.0, 5.0)
        takeoff.command = CMD_TAKEOFF
        t.on_reply(MissionPacket.item(takeoff, 0).wire())
        self.assertEqual(sent[-1], MissionPacket.request(1).wire())
        t.on_reply(MissionPacket.item(PlannerItem.waypoint(11.0, 21.0, 6.0), 1).wire())
        done = [e for e in rec.events if e[0] == "download"]
        self.assertEqual(len(done), 1)
        mission, result = done[0][1], done[0][2]
        self.assertEqual(result, MISSION_ACCEPTED)
        self.assertEqual(mission.size(), 2)
        self.assertEqual(mission.items[0].command, CMD_TAKEOFF)
        self.assertEqual(mission.items[1].lat, 11.0)
        self.assertEqual(mission.items[0].frame, FRAME_GLOBAL_RELATIVE_ALT)
        self.assertFalse(t.is_busy())

    def test_resend_and_give_up(self):
        # The app repeats its last message every 500 ms and gives up after 10
        # unanswered repeats (PROTOCOL.md §12).
        # App lặp thông điệp cuối mỗi 500 ms và bỏ cuộc sau 10 lần lặp không
        # được trả lời (PROTOCOL.md mục 12).
        t, sent, timer, rec = _make_transfer()
        t.upload([PlannerItem.waypoint(1, 2, 3)])
        self.assertEqual(len(sent), 1)
        timer.fire()
        self.assertEqual(sent[0], sent[1])       # the same message again
        for _ in range(MissionTransfer.MAX_TRIES):
            timer.fire()
        self.assertEqual(len(sent), 1 + MissionTransfer.MAX_TRIES)
        self.assertIn(("upload", MISSION_ERROR), rec.events)
        self.assertIn(("status", "No reply from the drone"), rec.events)
        self.assertFalse(t.is_busy())
        self.assertEqual(timer.pending, {})

    def test_set_current_and_start(self):
        t, sent, timer, rec = _make_transfer()
        t.set_current(2)
        self.assertEqual(sent[-1], MissionPacket.set_current(2).wire())
        t.on_reply(bytes([CMD_SET_CURRENT, 0]))
        self.assertIn(("cmd", CMD_SET_CURRENT, True), rec.events)
        self.assertIn(("status", "Accepted"), rec.events)
        t.start_mission()
        self.assertEqual(sent[-1], MissionPacket.start().wire())
        t.on_reply(bytes([CMD_START, 1]))
        self.assertIn(("cmd", CMD_START, False), rec.events)
        self.assertIn(("status", "Refused"), rec.events)

    def test_current_telemetry_any_state(self):
        t, sent, timer, rec = _make_transfer()
        t.on_reply(bytes([CMD_CURRENT]) + struct.pack("<H", 3))
        self.assertEqual(rec.events, [("current", 3)])

    def test_busy_guard(self):
        t, sent, timer, rec = _make_transfer()
        t.upload([PlannerItem.waypoint(1, 2, 3)])
        t.download()                             # refused while busy
        self.assertIn(("status", "Another transfer is still running"), rec.events)
        self.assertIn(("upload", MISSION_ERROR), rec.events)
        self.assertFalse(t.is_busy())

    def test_no_link(self):
        timer, rec = _FakeTimer(), _Recorder()
        t = MissionTransfer(rec, timer.schedule, timer.cancel)
        t.upload([PlannerItem.waypoint(1, 2, 3)])
        self.assertIn(("status", "No drone connected"), rec.events)
        self.assertFalse(t.is_busy())


class FrameTests(unittest.TestCase):
    """The flat-earth launch frame (PROTOCOL.md §12).
    Khung cất cánh đất phẳng (PROTOCOL.md mục 12)."""

    def test_north_east_offset(self):
        north, east = to_north_east(10.0, 106.0, 11.0, 106.0)
        self.assertAlmostEqual(north, 111319.49)
        self.assertAlmostEqual(east, 0.0)

    def test_round_trip(self):
        lat, lon = 10.762622, 106.660172
        north, east = to_north_east(lat, lon, lat + 0.001, lon + 0.002)
        back_lat, back_lon = to_lat_lon(lat, lon, north, east)
        self.assertAlmostEqual(back_lat, lat + 0.001, places=12)
        self.assertAlmostEqual(back_lon, lon + 0.002, places=12)


if __name__ == "__main__":
    unittest.main(verbosity=2)
