#!/usr/bin/env python3
"""Mission Planner's flight plan for the tuner — the QGC WPL 110 codec, the
CRTP mission wire and the transfer handshake (PROTOCOL.md §12).

Python port of the Android app's Plan page (the gpsnav package) and of the iOS
app's PlannerCore — the same Mission Planner semantics, quirks included. Files
produced here are byte-identical to Mission Planner's for the same mission, and
anything Mission Planner loads loads here.

  PlannerItem      one mission item — command, p1..p4, lat, lon, alt, frame
  PlannerMission   read/write .waypoints files (QGC WPL 110)
  MissionPacket    the wire: CRTP port 7 channel 2, header 0x72
  MissionTransfer  the MISSION_* handshake with the 500 ms resend

Bản dịch Python cho trang Plan của app Android (gói gpsnav) và PlannerCore của
app iOS — đúng ngữ nghĩa Mission Planner, kể cả các điểm lạ. Tệp tạo ra ở đây
khớp từng byte với tệp Mission Planner cho cùng nhiệm vụ, và mọi nội dung
Mission Planner nạp được đều nạp được ở đây.

  PlannerItem      một mục nhiệm vụ — lệnh, p1..p4, vĩ độ, kinh độ, độ cao, frame
  PlannerMission   đọc/ghi tệp .waypoints (QGC WPL 110)
  MissionPacket    mặt dây: CRTP cổng 7 kênh 2, header 0x72
  MissionTransfer  bắt tay MISSION_* với việc gửi lại mỗi 500 ms
"""
import math
import re
import struct

# --- MAV_CMD (the MAVLink common dialect) -----------------------------------

# The MAV_CMD enum generated from message_definitions/v1.0/common.xml — the
# same 171-entry table the Android and iOS ports carry (PlannerItem.CMD_IDS /
# CMD_NAMES), parallel arrays in id order. Display strips the names the way
# Mission Planner's C# generator does: MAV_CMD_NAV_WAYPOINT → WAYPOINT,
# MAV_CMD_DO_JUMP → DO_JUMP.
# Enum MAV_CMD sinh từ message_definitions/v1.0/common.xml — đúng bảng 171 mục
# mà bản Android và iOS mang (PlannerItem.CMD_IDS / CMD_NAMES), hai mảng song
# song theo thứ tự id. Cách hiển thị cắt tên như trình sinh C# của Mission
# Planner: MAV_CMD_NAV_WAYPOINT → WAYPOINT, MAV_CMD_DO_JUMP → DO_JUMP.
CMD_IDS = (
    16, 17, 18, 19, 20, 21, 22, 23,
    24, 30, 31, 32, 33, 34, 35, 36,
    80, 81, 82, 84, 85, 92, 93, 94,
    95, 112, 113, 114, 115, 159, 176, 177,
    178, 179, 180, 181, 182, 183, 184, 185,
    186, 187, 188, 189, 190, 191, 192, 193,
    194, 195, 196, 197, 198, 200, 201, 202,
    203, 204, 205, 206, 207, 208, 209, 210,
    211, 212, 213, 214, 220, 221, 222, 223,
    224, 240, 241, 242, 243, 245, 246, 252,
    260, 262, 300, 310, 311, 400, 401, 405,
    406, 410, 420, 500, 510, 511, 512, 519,
    520, 521, 522, 525, 526, 527, 528, 529,
    530, 531, 532, 533, 534, 600, 601, 611,
    1000, 1001, 2000, 2001, 2002, 2003, 2004, 2005,
    2010, 2500, 2501, 2502, 2503, 2504, 2505, 2510,
    2511, 2520, 2600, 2800, 3000, 3001, 4000, 4001,
    4501, 5000, 5001, 5002, 5003, 5004, 5100, 5200,
    5300, 10001, 30001, 30002, 42006, 42600, 43000, 43001,
    43002, 43003, 43004, 31000, 31001, 31002, 31003, 31004,
    31005, 31006, 31007, 31008, 31009, 31010, 31011, 31012,
    31013, 31014, 32000,
)
CMD_NAMES = (
    "WAYPOINT", "LOITER_UNLIM", "LOITER_TURNS", "LOITER_TIME",
    "RETURN_TO_LAUNCH", "LAND", "TAKEOFF", "LAND_LOCAL",
    "TAKEOFF_LOCAL", "CONTINUE_AND_CHANGE_ALT", "LOITER_TO_ALT", "DO_FOLLOW",
    "DO_FOLLOW_REPOSITION", "DO_ORBIT", "DO_FIGURE_EIGHT", "ARC_WAYPOINT",
    "ROI", "PATHPLANNING", "SPLINE_WAYPOINT", "VTOL_TAKEOFF",
    "VTOL_LAND", "GUIDED_ENABLE", "DELAY", "PAYLOAD_PLACE",
    "LAST", "CONDITION_DELAY", "CONDITION_CHANGE_ALT", "CONDITION_DISTANCE",
    "CONDITION_YAW", "CONDITION_LAST", "DO_SET_MODE", "DO_JUMP",
    "DO_CHANGE_SPEED", "DO_SET_HOME", "DO_SET_PARAMETER", "DO_SET_RELAY",
    "DO_REPEAT_RELAY", "DO_SET_SERVO", "DO_REPEAT_SERVO", "DO_FLIGHTTERMINATION",
    "DO_CHANGE_ALTITUDE", "DO_SET_ACTUATOR", "DO_RETURN_PATH_START", "DO_LAND_START",
    "DO_RALLY_LAND", "DO_GO_AROUND", "DO_REPOSITION", "DO_PAUSE_CONTINUE",
    "DO_SET_REVERSE", "DO_SET_ROI_LOCATION", "DO_SET_ROI_WPNEXT_OFFSET", "DO_SET_ROI_NONE",
    "DO_SET_ROI_SYSID", "DO_CONTROL_VIDEO", "DO_SET_ROI", "DO_DIGICAM_CONFIGURE",
    "DO_DIGICAM_CONTROL", "DO_MOUNT_CONFIGURE", "DO_MOUNT_CONTROL", "DO_SET_CAM_TRIGG_DIST",
    "DO_FENCE_ENABLE", "DO_PARACHUTE", "DO_MOTOR_TEST", "DO_INVERTED_FLIGHT",
    "DO_GRIPPER", "DO_AUTOTUNE_ENABLE", "SET_YAW_SPEED", "DO_SET_CAM_TRIGG_INTERVAL",
    "DO_MOUNT_CONTROL_QUAT", "DO_GUIDED_MASTER", "DO_GUIDED_LIMITS", "DO_ENGINE_CONTROL",
    "DO_SET_MISSION_CURRENT", "DO_LAST", "PREFLIGHT_CALIBRATION", "PREFLIGHT_SET_SENSOR_OFFSETS",
    "PREFLIGHT_UAVCAN", "PREFLIGHT_STORAGE", "PREFLIGHT_REBOOT_SHUTDOWN", "OVERRIDE_GOTO",
    "OBLIQUE_SURVEY", "DO_SET_STANDARD_MODE", "MISSION_START", "ACTUATOR_TEST",
    "CONFIGURE_ACTUATOR", "COMPONENT_ARM_DISARM", "RUN_PREARM_CHECKS", "ILLUMINATOR_ON_OFF",
    "DO_ILLUMINATOR_CONFIGURE", "GET_HOME_POSITION", "INJECT_FAILURE", "START_RX_PAIR",
    "GET_MESSAGE_INTERVAL", "SET_MESSAGE_INTERVAL", "REQUEST_MESSAGE", "REQUEST_PROTOCOL_VERSION",
    "REQUEST_AUTOPILOT_CAPABILITIES", "REQUEST_CAMERA_INFORMATION", "REQUEST_CAMERA_SETTINGS", "REQUEST_STORAGE_INFORMATION",
    "STORAGE_FORMAT", "REQUEST_CAMERA_CAPTURE_STATUS", "REQUEST_FLIGHT_INFORMATION", "RESET_CAMERA_SETTINGS",
    "SET_CAMERA_MODE", "SET_CAMERA_ZOOM", "SET_CAMERA_FOCUS", "SET_STORAGE_USAGE",
    "SET_CAMERA_SOURCE", "JUMP_TAG", "DO_JUMP_TAG", "DO_SET_GLOBAL_ORIGIN",
    "DO_GIMBAL_MANAGER_PITCHYAW", "DO_GIMBAL_MANAGER_CONFIGURE", "IMAGE_START_CAPTURE", "IMAGE_STOP_CAPTURE",
    "REQUEST_CAMERA_IMAGE_CAPTURE", "DO_TRIGGER_CONTROL", "CAMERA_TRACK_POINT", "CAMERA_TRACK_RECTANGLE",
    "CAMERA_STOP_TRACKING", "VIDEO_START_CAPTURE", "VIDEO_STOP_CAPTURE", "VIDEO_START_STREAMING",
    "VIDEO_STOP_STREAMING", "REQUEST_VIDEO_STREAM_INFORMATION", "REQUEST_VIDEO_STREAM_STATUS", "LOGGING_START",
    "LOGGING_STOP", "AIRFRAME_CONFIGURATION", "CONTROL_HIGH_LATENCY", "PANORAMA_CREATE",
    "DO_VTOL_TRANSITION", "ARM_AUTHORIZATION_REQUEST", "SET_GUIDED_SUBMODE_STANDARD", "SET_GUIDED_SUBMODE_CIRCLE",
    "CONDITION_GATE", "FENCE_RETURN_POINT", "FENCE_POLYGON_VERTEX_INCLUSION", "FENCE_POLYGON_VERTEX_EXCLUSION",
    "FENCE_CIRCLE_INCLUSION", "FENCE_CIRCLE_EXCLUSION", "RALLY_POINT", "UAVCAN_GET_NODE_INFO",
    "DO_SET_SAFETY_SWITCH_STATE", "DO_ADSB_OUT_IDENT", "PAYLOAD_PREPARE_DEPLOY", "PAYLOAD_CONTROL_DEPLOY",
    "FIXED_MAG_CAL_YAW", "DO_WINCH", "GUIDED_CHANGE_SPEED", "GUIDED_CHANGE_ALTITUDE",
    "GUIDED_CHANGE_HEADING", "EXTERNAL_POSITION_ESTIMATE", "EXTERNAL_WIND_ESTIMATE", "WAYPOINT_USER_1",
    "WAYPOINT_USER_2", "WAYPOINT_USER_3", "WAYPOINT_USER_4", "WAYPOINT_USER_5",
    "SPATIAL_USER_1", "SPATIAL_USER_2", "SPATIAL_USER_3", "SPATIAL_USER_4",
    "SPATIAL_USER_5", "USER_1", "USER_2", "USER_3",
    "USER_4", "USER_5", "CAN_FORWARD",
)
assert len(CMD_IDS) == len(CMD_NAMES)

_NAMES_BY_ID = dict(zip(CMD_IDS, CMD_NAMES))
_IDS_BY_NAME = dict(zip(CMD_NAMES, CMD_IDS))

# The commands the firmware actually flies (PROTOCOL.md §12); the Plan screens'
# command menu lists these.
# Các lệnh firmware thực sự bay (PROTOCOL.md mục 12); menu lệnh của màn hình Plan
# liệt kê các lệnh này.
CMD_WAYPOINT, CMD_RETURN_TO_LAUNCH, CMD_LAND, CMD_TAKEOFF, CMD_DO_JUMP = 16, 20, 21, 22, 177
FLOWN_COMMANDS = ("WAYPOINT", "TAKEOFF", "LAND", "RETURN_TO_LAUNCH", "DO_JUMP")

# MAV_FRAME.GLOBAL — what Mission Planner writes for the home row;
# MAV_FRAME.GLOBAL_RELATIVE_ALT — its default for new rows ("Relative" alt
# mode) and what the firmware's items mean by alt.
# MAV_FRAME.GLOBAL — Mission Planner ghi cho dòng home;
# MAV_FRAME.GLOBAL_RELATIVE_ALT — mặc định của Mission Planner cho dòng mới
# (chế độ độ cao "Relative") và ý nghĩa alt trong mục của firmware.
FRAME_GLOBAL, FRAME_GLOBAL_RELATIVE_ALT = 0, 3

DEFAULT_WAYPOINT_RADIUS = 2.5     # p2 on new waypoints (Mission Planner)
                                  # p2 của waypoint mới (Mission Planner)
DEFAULT_WAYPOINT_ALT = 3.0        # alt on new waypoints
                                  # alt của waypoint mới


def command_name_of(command):
    """The name Mission Planner's grid shows: the MAV_CMD name, else "UNKNOWN".
    Tên mà lưới Mission Planner hiện: tên MAV_CMD, nếu không thì "UNKNOWN"."""
    return _NAMES_BY_ID.get(command, "UNKNOWN")


def parse_command(text):
    """Mission Planner parses the file's command cell with
    Enum.Parse(typeof(MAV_CMD), text, false): a number or the exact enum name,
    case-sensitive. Anything else is an invalid line there — this returns -1
    for the caller to report the same way.
    Mission Planner ô lệnh trong tệp bằng Enum.Parse(typeof(MAV_CMD), text,
    false): một con số hoặc đúng tên enum, phân biệt hoa thường. Nội dung khác
    là dòng không hợp lệ bên đó — hàm này trả -1 để nơi gọi báo như vậy."""
    if text is None:
        return -1
    try:
        # Enum.Parse takes any number, even one with no enum member, and the
        # (ushort) cast rejects what does not fit the storage.
        # Enum.Parse nhận mọi con số, kể cả số không có thành viên enum, còn
        # ép kiểu (ushort) từ chối phần không vừa chỗ lưu.
        value = int(text)
        return value if 0 <= value <= 0xFFFF else -1
    except ValueError:
        pass                                    # not a number — try the names
                                                # không phải số — thử bảng tên
    return _IDS_BY_NAME.get(text, -1)


# --- one mission item -------------------------------------------------------


def _f32(x):
    """Narrow to float32, exactly like Java's float fields / C#'s float.Parse.
    Thu hẹp về float32, đúng như trường float của Java / float.Parse của C#."""
    try:
        return struct.unpack("<f", struct.pack("<f", x))[0]
    except OverflowError:
        return math.copysign(math.inf, x)


class PlannerItem:
    """One mission item in the Mission Planner sense: the same fields the QGC
    WPL 110 file and the MAVLink MISSION_ITEM_INT message carry — command plus
    four parameters, latitude/longitude and altitude (PROTOCOL.md §12).
    Một mục nhiệm vụ theo nghĩa Mission Planner: đúng các trường mà tệp QGC WPL
    110 và thông điệp MAVLink MISSION_ITEM_INT mang — lệnh cộng bốn tham số,
    vĩ độ/kinh độ và độ cao (PROTOCOL.md mục 12)."""

    def __init__(self):
        # MAV_CMD id; the wire and file truth, kept even when no name is known.
        # Số id MAV_CMD; dữ liệu gốc trên đường truyền và tệp, giữ nguyên kể cả
        # khi không biết tên.
        self.command = 0
        self.p1 = 0.0
        self.p2 = 0.0
        self.p3 = 0.0
        self.p4 = 0.0
        # WGS-84 degrees.
        # Độ WGS-84.
        self.lat = 0.0
        self.lon = 0.0
        # Metres, frame-relative as self.frame says.
        # Mét, tương đối theo khung như self.frame quy định.
        self.alt = 0.0
        # MAV_FRAME id; written back verbatim so files round-trip.
        # Số id MAV_FRAME; ghi lại nguyên văn để tệp không mất dữ liệu.
        self.frame = FRAME_GLOBAL_RELATIVE_ALT

    @staticmethod
    def waypoint(lat, lon, alt):
        """A fresh mission row in Mission Planner's default frame, at a position.
        Một dòng nhiệm vụ mới trong khung mặc định của Mission Planner, tại một
        vị trí."""
        item = PlannerItem()
        item.command = CMD_WAYPOINT
        item.lat = lat
        item.lon = lon
        item.alt = alt
        return item

    def command_name(self):
        return command_name_of(self.command)


# --- the QGC WPL 110 codec --------------------------------------------------

WPL_HEADER = "QGC WPL 110"


def _blank_home():
    # Mission Planner's new Locationwp(): every field zero, frame included.
    # new Locationwp() của Mission Planner: mọi trường bằng 0, kể cả frame.
    home = PlannerItem()
    home.frame = 0
    home.command = 0
    return home


class PlannerMission:
    """A mission in the QGC WPL 110 format, read and written with the exact
    semantics of Mission Planner — the reader is WaypointFile.ReadWaypointFile
    and the writer is FlightPlanner.savewaypoints() (PROTOCOL.md §12). Files
    produced here are byte-identical to Mission Planner's for the same mission,
    and anything Mission Planner loads loads here; the quirks below are
    deliberate, not oversights.
    Một nhiệm vụ theo định dạng QGC WPL 110, đọc và ghi với đúng ngữ nghĩa của
    Mission Planner — đầu đọc là WaypointFile.ReadWaypointFile, đầu ghi là
    FlightPlanner.savewaypoints() (PROTOCOL.md mục 12). Tệp tạo ra ở đây khớp
    từng byte với tệp Mission Planner cho cùng nhiệm vụ, và mọi nội dung Mission
    Planner nạp được đều nạp được ở đây; các điểm lạ dưới đây là cố ý, không
    phải thiếu sót.

    Reader rules, copied from Mission Planner:
    - the header line must contain "QGC WPL" or the file is rejected;
    - lines starting with '#' are comments and are skipped;
    - fields split on tab, space or comma, empty fields dropped;
    - a row with 9 or fewer fields is skipped silently;
    - if the first parsed row's index field is not the string "0", a blank home
      row is inserted before it — and the blank is inserted even when that row
      then fails to parse;
    - columns are index, current, frame, command, p1..p4, lat, lng, alt — the
      command is an MAV_CMD name or its number;
    - command 99 is a legacy waypoint id and is rewritten to 0;
    - a row that cannot be parsed is reported and skipped, the rest of the file
      still loads.
    Quy tắc đọc, sao chép từ Mission Planner:
    - dòng đầu phải chứa "QGC WPL" thì tệp mới được chấp nhận;
    - dòng bắt đầu bằng '#' là chú thích và bị bỏ qua;
    - trường phân tách bằng tab, dấu cách hoặc phẩy, trường rỗng bị bỏ;
    - dòng có từ 9 trường trở xuống bị bỏ qua im lặng;
    - nếu ô chỉ số của dòng đầu tiên phân tích được không đúng chuỗi "0", một
      dòng home trống sẽ được chèn vào trước nó — và dòng trống vẫn được chèn
      kể cả khi dòng đó sau đó phân tích thất bại;
    - các cột là chỉ số, current, frame, lệnh, p1..p4, vĩ độ, kinh độ, độ cao —
      lệnh là tên MAV_CMD hoặc số của nó;
    - lệnh 99 là id waypoint kiểu cũ và được viết lại thành 0;
    - dòng không phân tích được sẽ được báo và bỏ qua, phần còn lại của tệp vẫn
      được nạp."""

    def __init__(self):
        # Home is row 0 of a mission file — the reference for every altitude.
        # Home là dòng 0 của tệp nhiệm vụ — gốc tham chiếu cho mọi độ cao.
        self.home = _blank_home()
        # The mission rows, in file order (the file's seq 1..n).
        # Các dòng nhiệm vụ, theo thứ tự tệp (seq 1..n trong tệp).
        self.items = []
        # Lines the reader could not parse, kept so the screen can report them.
        # Các dòng đầu đọc không phân tích được, giữ lại để màn hình có thể báo.
        self.invalid_lines = []

    def size(self):
        return len(self.items)

    def add(self, item):
        self.items.append(item)

    def clear(self):
        self.items.clear()
        self.invalid_lines.clear()

    @classmethod
    def parse(cls, text):
        """Parse a whole file. Raises ValueError only when the header is
        missing — that is Mission Planner's "Invalid Waypoint file". Bad rows
        are collected in invalid_lines and skipped, matching its per-line
        report.
        Phân tích cả tệp. Chỉ ném lỗi ValueError khi thiếu dòng tiêu đề — đó là
        "Invalid Waypoint file" của Mission Planner. Dòng hỏng được gom vào
        invalid_lines và bỏ qua, giống như việc nó báo từng dòng."""
        if text is None:
            raise ValueError("Invalid Waypoint file")
        lines = re.split(r"\r?\n", text)
        if not lines or "QGC WPL" not in lines[0]:
            raise ValueError("Invalid Waypoint file")

        # The reference reads into one list whose row 0 is home. Keep the same
        # shape while parsing — the blank-home quirk only reads naturally that
        # way — and split home from the mission rows at the end.
        # Bản gốc đọc vào một danh sách mà dòng 0 là home. Giữ đúng hình dạng
        # đó khi phân tích — điểm lạ home trống chỉ tự nhiên theo cách đó — rồi
        # tách home khỏi các dòng nhiệm vụ ở cuối.
        mission = cls()
        rows = []
        wp_count = 0
        for line in lines[1:]:
            if line.startswith("#"):
                continue
            # Split(new[] { '\t', ' ', ',' }, RemoveEmptyEntries) — drop the
            # empty tokens a leading/trailing separator makes.
            # Split(new[] { '\t', ' ', ',' }, RemoveEmptyEntries) — loại token
            # rỗng mà dấu phân cách đầu/cuối dòng sinh ra.
            fields = [f for f in re.split(r"[\t ,]+", line) if f]
            if len(fields) <= 9:
                continue
            try:
                # Check to see if the first wp is index 0/home. If it is not
                # index 0, add a blank home point. (Mission Planner's words —
                # the blank is added before the row is parsed and stays when
                # the parse then fails.)
                # Kiểm tra xem waypoint đầu có phải index 0/home không. Nếu
                # không phải index 0, thêm một điểm home trống. (Nguyên văn
                # Mission Planner — dòng trống được thêm trước khi phân tích
                # dòng và ở lại kể cả khi việc phân tích thất bại.)
                if wp_count == 0 and fields[0] != "0":
                    rows.append(_blank_home())

                temp = PlannerItem()
                temp.frame = int(fields[2])
                temp.command = parse_command(fields[3])
                if temp.command < 0:
                    # Enum.Parse threw there → "Line invalid".
                    # Enum.Parse ném lỗi ở đó → "Line invalid".
                    raise ValueError("bad command")
                temp.p1 = _f32(float(fields[4]))
                if temp.command == 99:
                    # A legacy waypoint id: Mission Planner rewrites it to 0.
                    # Id waypoint kiểu cũ: Mission Planner viết lại thành 0.
                    temp.command = 0
                temp.alt = _f32(float(fields[10]))
                temp.lat = float(fields[8])
                temp.lon = float(fields[9])
                temp.p2 = _f32(float(fields[5]))
                temp.p3 = _f32(float(fields[6]))
                temp.p4 = _f32(float(fields[7]))

                rows.append(temp)
                wp_count += 1
            except Exception:
                # Mission Planner shows "Line invalid" and keeps reading.
                # Mission Planner báo "Line invalid" rồi vẫn đọc tiếp.
                mission.invalid_lines.append(line)

        if rows:
            mission._replace_home(rows[0])
            mission.items.extend(rows[1:])
        return mission

    def _replace_home(self, home):
        self.home.frame = home.frame
        self.home.command = home.command
        self.home.p1 = home.p1
        self.home.p2 = home.p2
        self.home.p3 = home.p3
        self.home.p4 = home.p4
        self.home.lat = home.lat
        self.home.lon = home.lon
        self.home.alt = home.alt

    def format(self):
        """Write the file exactly like Mission Planner's savewaypoints(): the
        header, one home row with frame 0 and command 16, then the mission rows
        with seq 1..n. Number formats are its string formats — seven decimals
        for the home lat/lng, eight for the rows' p1..p4/lat/lng, six for every
        altitude, invariant culture (dot decimal).
        Ghi tệp đúng như savewaypoints() của Mission Planner: dòng tiêu đề,
        một dòng home với frame 0 và lệnh 16, rồi các dòng nhiệm vụ đánh số seq
        1..n. Định dạng số là định dạng chuỗi của nó — bảy chữ số thập phân cho
        vĩ/kinh độ home, tám cho p1..p4/vĩ/kinh độ của dòng, sáu cho mọi độ cao,
        văn hóa bất biến (dấu chấm thập phân)."""
        out = WPL_HEADER + "\n"

        # Home row: "0\t1\t0\t16\t0\t0\t0\t0\t<lat>\t<lng>\t<alt>\t1". Mission
        # Planner falls back to an all-zero row when its home fields will not
        # parse; here that is the non-finite check below.
        # Dòng home: "0\t1\t0\t16\t0\t0\t0\t0\t<lat>\t<lng>\t<alt>\t1". Mission
        # Planner chuyển sang dòng toàn số 0 khi ô home không phân tích được; ở
        # đây là phép kiểm không hữu hạn bên dưới.
        home = self.home
        if (math.isfinite(home.lat) and math.isfinite(home.lon)
                and math.isfinite(_f32(home.alt))):
            out += "0\t1\t0\t16\t0\t0\t0\t0\t%s\t%s\t%s\t1\n" % (
                _fmt("%.7f", home.lat), _fmt("%.7f", home.lon),
                _fmt("%.6f", _f32(home.alt)))
        else:
            out += "0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1\n"

        for a, it in enumerate(self.items):
            # seq is the row index + 1; current is always 0 for mission rows.
            # seq là chỉ số dòng + 1; current luôn 0 cho dòng nhiệm vụ.
            out += "%d\t0\t%d\t%d\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t1\n" % (
                a + 1, it.frame, it.command,
                _fmt("%.8f", _f32(it.p1)), _fmt("%.8f", _f32(it.p2)),
                _fmt("%.8f", _f32(it.p3)), _fmt("%.8f", _f32(it.p4)),
                _fmt("%.8f", it.lat), _fmt("%.8f", it.lon),
                _fmt("%.6f", _f32(it.alt)))
        return out


def _fmt(spec, value):
    """%f like Java's String.format(Locale.US): infinities print as
    Infinity / -Infinity / NaN, which float.Parse reads back.
    %f như String.format(Locale.US) của Java: vô cực in ra Infinity / -Infinity
    / NaN, mà float.Parse đọc ngược lại được."""
    if value != value:
        return "NaN"
    if value == math.inf:
        return "Infinity"
    if value == -math.inf:
        return "-Infinity"
    return spec % value


# --- the wire (PROTOCOL.md §12, CRTP port 7 channel 2) ----------------------

# Mission transfer on CRTP port 7, channel 2 — the same MISSION_* handshake
# MAVLink uses, re-packed for the 30-byte CRTP payload. The drone's handler is
# crtpHandleMission(); every command is idempotent so the 500 ms resend can
# repeat it safely. Wire layouts, command byte first, all little-endian:
#   COUNT        0x30  u16 count                      app → drone
#   REQUEST      0x31  u16 seq                        drone → app (upload)
#   ITEM         0x32  u16 seq, u16 command, f32 p1, f32 p2,
#                      i32 lat_e7, i32 lon_e7, f32 alt    both ways
#   ACK          0x33  u8 type                        drone → app
#   REQUEST_LIST 0x34                                 app → drone
#   SET_CURRENT  0x35  u16 seq                        app → drone → <0x35><status>
#   CURRENT      0x36  u16 seq                        drone → app (telemetry)
#   START        0x37                                 app → drone → <0x37><status>
# p3/p4 do not fit the 30-byte CRTP ceiling and are sent as zero — they live
# in the .waypoints file and on the MAVLink bridge (§13) only.
# Truyền nhiệm vụ trên CRTP port 7 kênh 2 — đúng bắt tay MISSION_* của MAVLink,
# đóng gói lại cho gói CRTP 30 byte. Xử lý phía drone là crtpHandleMission();
# mọi lệnh đều lặp được nên việc gửi lại sau 500 ms có thể an toàn lặp lại.
# Bố cục dây, byte lệnh đứng đầu, tất cả little-endian: (xem bảng ở trên).
# p3/p4 không vừa trần 30 byte của CRTP nên gửi bằng 0 — chúng chỉ có trong tệp
# .waypoints và trên cầu nối MAVLink (mục 13).

MISSION_CH = 2                    # the channel of the wire above
                                  # kênh của mặt dây ở trên

CMD_COUNT, CMD_REQUEST, CMD_ITEM, CMD_ACK = 0x30, 0x31, 0x32, 0x33
CMD_REQUEST_LIST, CMD_SET_CURRENT, CMD_CURRENT, CMD_START = 0x34, 0x35, 0x36, 0x37

# MAV_MISSION results (PROTOCOL.md §12).
# Kết quả MAV_MISSION (PROTOCOL.md mục 12).
MISSION_ACCEPTED, MISSION_ERROR, MISSION_NO_SPACE = 0, 1, 4
MISSION_INVALID, MISSION_DENIED = 5, 14


def mission_result_name(result):
    return {MISSION_ACCEPTED: "accepted",
            MISSION_NO_SPACE: "no space on the drone",
            MISSION_INVALID: "invalid mission",
            MISSION_DENIED: "denied"}.get(result, "error %d" % result)


def _e7(value):
    """degrees × 1e7 as int32 — half-up like Java's Math.round; Python's
    round() is half-even and would differ at exact .5 ties.
    độ × 1e7 theo int32 — làm tròn nửa lên như Math.round của Java; round()
    của Python làm tròn nửa chẵn và sẽ khác ở đúng mốc .5."""
    if not math.isfinite(value):
        return 0
    return int(math.floor(value * 1e7 + 0.5))


class MissionPacket:
    """One mission message: command byte + little-endian args (the CRTP payload
    send() wraps with the 0x72 header byte).
    Một thông điệp nhiệm vụ: byte lệnh + tham số little-endian (phần dữ liệu
    CRTP mà send() bọc thêm byte header 0x72)."""

    def __init__(self, cmd, args=b""):
        self.cmd = cmd
        self.args = bytes(args)

    def wire(self):
        return bytes([self.cmd]) + self.args

    # app → drone ------------------------------------------------------------

    @staticmethod
    def count(n):
        """Start an upload of n items (0 clears the stored mission).
        Bắt đầu tải lên n mục (0 = xóa nhiệm vụ đang lưu)."""
        return MissionPacket(CMD_COUNT, struct.pack("<H", n))

    @staticmethod
    def request(seq):
        """Ask for the item at seq (also how an upload pulls the next one).
        Yêu cầu mục tại seq (cũng là cách tải lên xin mục tiếp theo)."""
        return MissionPacket(CMD_REQUEST, struct.pack("<H", seq))

    @staticmethod
    def item(item, seq):
        """One mission item. The lat/lon travel as degrees × 1e7, exactly the
        MISSION_ITEM_INT integer encoding the firmware stores (§12).
        Một mục nhiệm vụ. Vĩ độ/kinh độ truyền theo độ × 1e7, đúng mã hóa số
        nguyên MISSION_ITEM_INT mà firmware lưu (mục 12)."""
        args = struct.pack("<HHff iif".replace(" ", ""),
                           seq & 0xFFFF, item.command & 0xFFFF,
                           _f32(item.p1), _f32(item.p2),
                           _e7(item.lat), _e7(item.lon), _f32(item.alt))
        return MissionPacket(CMD_ITEM, args)

    @staticmethod
    def request_list():
        """Ask the drone how many items it holds, before downloading.
        Hỏi drone đang giữ bao nhiêu mục, trước khi tải về."""
        return MissionPacket(CMD_REQUEST_LIST)

    @staticmethod
    def set_current(seq):
        """Make seq the current item (jumps counters reset, §12).
        Đặt seq thành mục hiện tại (bộ đếm jump được reset, mục 12)."""
        return MissionPacket(CMD_SET_CURRENT, struct.pack("<H", seq))

    @staticmethod
    def start():
        """Fly the stored mission (arms first, like ArduCopter's AUTO).
        Bay nhiệm vụ đã lưu (arm trước, như AUTO của ArduCopter)."""
        return MissionPacket(CMD_START)

    # drone → app ------------------------------------------------------------

    @staticmethod
    def parse_u16(payload, offset):
        """Parse the u16 argument of COUNT / REQUEST / CURRENT payloads.
        Phân tích tham số u16 của payload COUNT / REQUEST / CURRENT."""
        if len(payload) < offset + 2:
            return -1
        return payload[offset] | (payload[offset + 1] << 8)

    @staticmethod
    def parse_u8(payload, offset):
        """Parse the u8 argument of ACK / SET_CURRENT / START payloads.
        Phân tích tham số u8 của payload ACK / SET_CURRENT / START."""
        if len(payload) < offset + 1:
            return -1
        return payload[offset]

    @staticmethod
    def parse_item(payload):
        """Turn a downloaded ITEM payload back into an item (p3/p4 are zero).
        Đưa payload ITEM tải về trở lại thành mục (p3/p4 bằng 0)."""
        if len(payload) < 25:
            raise ValueError("short mission item")
        _seq, command, p1, p2, lat_e7, lon_e7, alt = struct.unpack(
            "<HHff iif".replace(" ", ""), bytes(payload[1:25]))
        item = PlannerItem()
        item.command = command
        item.p1, item.p2 = p1, p2
        item.lat, item.lon = lat_e7 / 1e7, lon_e7 / 1e7
        item.alt = alt
        item.frame = FRAME_GLOBAL_RELATIVE_ALT
        return item


# --- the transfer state machine ---------------------------------------------


class MissionTransfer:
    """The mission-transfer side of PROTOCOL.md §12: runs the MISSION_*
    handshake over CRTP port 7 channel 2 and re-sends its last message every
    500 ms until the expected reply arrives. The drone's handlers are
    idempotent, so those repeats are how a lossy UDP hop stays correct.
    Phía truyền nhiệm vụ của PROTOCOL.md mục 12: chạy bắt tay MISSION_* trên
    CRTP port 7 kênh 2 và gửi lại thông điệp cuối mỗi 500 ms cho tới khi nhận
    được câu trả lời mong đợi. Xử lý phía drone lặp được, nên các lần gửi lại
    đó là cách một đường truyền UDP hay mất gói vẫn đúng.

    Upload:  COUNT n → REQUEST 0 → ITEM 0 → REQUEST 1 → … → ITEM n-1 → ACK 0.
    Download:  REQUEST_LIST → COUNT n → REQUEST 0 → ITEM 0 → … → ITEM n-1.

    The listener (main thread only) takes five callbacks:
    on_transfer_status(message), on_upload_finished(result),
    on_download_finished(mission, result), on_command_finished(cmd, ok) and
    on_current_item(seq). `schedule(ms, fn)` must return a timer handle and
    `cancel(handle)` cancel it (Tk: root.after / root.after_cancel); `attach`
    takes the one-argument sender the wires go out on.
    Trình nghe (chỉ luồng chính) nhận năm callback: on_transfer_status,
    on_upload_finished, on_download_finished, on_command_finished và
    on_current_item. `schedule(ms, fn)` phải trả về một handle hẹn giờ và
    `cancel(handle)` hủy nó (Tk: root.after / root.after_cancel); `attach` nhận
    hàm gửi một tham số mà các gói dây đi ra theo."""

    # PROTOCOL.md §12: the app repeats its last mission message at this rate,
    # and gives up after this many unanswered repeats (5 s of silence).
    # PROTOCOL.md mục 12: app lặp thông điệp nhiệm vụ cuối với nhịp này, và bỏ
    # cuộc sau số lần lặp không được trả lời này (5 giây im lặng).
    RESEND_MS = 500
    MAX_TRIES = 10

    _IDLE, _UPLOAD, _DOWNLOAD, _SIMPLE = 0, 1, 2, 3

    def __init__(self, listener, schedule, cancel):
        self.listener = listener
        self._schedule = schedule
        self._cancel = cancel
        self._send = None
        self._mode = self._IDLE
        self._last_sent = None
        self._tries = 0
        self._timer = None
        self._up_items = []
        self._up_total = 0
        self._down_mission = None
        self._down_total = 0
        self._simple_cmd = 0

    # --- lifecycle ----------------------------------------------------------

    def attach(self, send):
        """Start sending on the link; safe to call again with the same sender.
        Bắt đầu gửi trên link; gọi lại với cùng hàm gửi vẫn an toàn."""
        self._send = send

    def detach(self):
        """Stop the timers and abandon any transfer in flight.
        Dừng các hẹn giờ và hủy mọi phiên truyền đang chạy."""
        self._cancel_timer()
        self._send = None
        self._mode = self._IDLE
        self._last_sent = None

    def is_busy(self):
        return self._mode != self._IDLE

    # --- what the buttons drive --------------------------------------------

    def upload(self, items):
        """Upload the items to the drone's store (an empty list clears it).
        Tải các mục lên kho của drone (danh sách rỗng = xóa)."""
        if self.is_busy():
            self._fail("Another transfer is still running")
            return
        if self._send is None:
            self._fail("No drone connected")
            return
        self._up_items = list(items)
        self._up_total = len(self._up_items)
        self._mode = self._UPLOAD
        self.listener.on_transfer_status("Clearing stored mission"
                                         if self._up_total == 0
                                         else "Uploading 0/%d" % self._up_total)
        self._send_now(MissionPacket.count(self._up_total))

    def download(self):
        """Download whatever the drone holds into a fresh mission.
        Tải về nội dung drone đang giữ thành một nhiệm vụ mới."""
        if self.is_busy():
            self._fail("Another transfer is still running")
            return
        if self._send is None:
            self._fail("No drone connected")
            return
        self._mode = self._DOWNLOAD
        self._down_mission = PlannerMission()
        self.listener.on_transfer_status("Requesting mission list")
        self._send_now(MissionPacket.request_list())

    def set_current(self, seq):
        """Jump to an item; the drone answers CMD_SET_CURRENT + status.
        Nhảy tới một mục; drone trả CMD_SET_CURRENT + status."""
        self._start_simple(MissionPacket.set_current(seq), "Setting current item")

    def start_mission(self):
        """Fly the stored mission; the drone answers CMD_START + status.
        Bay nhiệm vụ đã lưu; drone trả CMD_START + status."""
        self._start_simple(MissionPacket.start(), "Starting mission")

    def _start_simple(self, packet, status):
        if self.is_busy():
            self._fail("Another transfer is still running")
            return
        if self._send is None:
            self._fail("No drone connected")
            return
        self._mode = self._SIMPLE
        self._simple_cmd = packet.cmd
        self.listener.on_transfer_status(status)
        self._send_now(packet)

    # --- state machine, main thread only -----------------------------------

    def _send_now(self, packet):
        self._last_sent = packet
        self._tries = 0
        self._send(packet.wire())
        self._cancel_timer()
        self._timer = self._schedule(self.RESEND_MS, self._resend)

    def _cancel_timer(self):
        if self._timer is not None:
            self._cancel(self._timer)
            self._timer = None

    def _resend(self):
        self._timer = None
        if self._last_sent is None or self._send is None:
            return
        if self._tries >= self.MAX_TRIES:
            # Stop before the repeat becomes its own hazard: the drone's
            # handlers are idempotent, but a dead link will never answer.
            # Dừng trước khi việc lặp tự gây hại: xử lý phía drone lặp được,
            # nhưng link chết sẽ không bao giờ trả lời.
            mode = self._mode
            self._last_sent = None
            self._mode = self._IDLE
            if mode == self._UPLOAD:
                self.listener.on_upload_finished(MISSION_ERROR)
            elif mode == self._DOWNLOAD:
                self.listener.on_download_finished(None, MISSION_ERROR)
            elif mode == self._SIMPLE:
                self.listener.on_command_finished(self._simple_cmd, False)
            self.listener.on_transfer_status("No reply from the drone")
            return
        self._tries += 1
        self._send(self._last_sent.wire())
        self._timer = self._schedule(self.RESEND_MS, self._resend)

    def on_reply(self, payload):
        """One payload from the mission channel, main thread.
        Một phần dữ liệu từ kênh nhiệm vụ, luồng chính."""
        if not payload:
            return
        cmd = payload[0]

        # MISSION_CURRENT telemetry is welcome in any state.
        # Telemetry MISSION_CURRENT được chào đón ở mọi trạng thái.
        if cmd == CMD_CURRENT:
            seq = MissionPacket.parse_u16(payload, 1)
            if seq >= 0:
                self.listener.on_current_item(seq)
            return

        if (self._mode == self._UPLOAD and self._last_sent is not None
                and self._last_sent.cmd == CMD_COUNT):
            if cmd == CMD_REQUEST:
                seq = MissionPacket.parse_u16(payload, 1)
                if seq < 0 or seq >= self._up_total:
                    self._finish_upload(MISSION_INVALID)
                else:
                    self._send_now(MissionPacket.item(self._up_items[seq], seq))
            elif cmd == CMD_ACK:
                # COUNT with 0 items clears the store and ACKs immediately.
                # COUNT với 0 mục xóa kho và ACK ngay.
                self._finish_upload(MissionPacket.parse_u8(payload, 1))
            return

        if (self._mode == self._UPLOAD and self._last_sent is not None
                and self._last_sent.cmd == CMD_ITEM):
            if cmd == CMD_REQUEST:
                seq = MissionPacket.parse_u16(payload, 1)
                if seq < 0 or seq >= self._up_total:
                    self._finish_upload(MISSION_INVALID)
                else:
                    self.listener.on_transfer_status(
                        "Uploading %d/%d" % (seq, self._up_total))
                    self._send_now(MissionPacket.item(self._up_items[seq], seq))
            elif cmd == CMD_ACK:
                typ = MissionPacket.parse_u8(payload, 1)
                if typ == MISSION_ACCEPTED:
                    self.listener.on_transfer_status("Mission uploaded")
                self._finish_upload(typ)
            return

        if (self._mode == self._DOWNLOAD and self._last_sent is not None
                and self._last_sent.cmd == CMD_REQUEST_LIST):
            if cmd == CMD_COUNT:
                self._down_total = MissionPacket.parse_u16(payload, 1)
                if self._down_total <= 0:
                    self._finish_download(MISSION_ACCEPTED)
                else:
                    self.listener.on_transfer_status(
                        "Downloading 0/%d" % self._down_total)
                    self._send_now(MissionPacket.request(0))
            elif cmd == CMD_ACK:
                self._finish_download(MissionPacket.parse_u8(payload, 1))
            return

        if (self._mode == self._DOWNLOAD and self._last_sent is not None
                and self._last_sent.cmd == CMD_REQUEST):
            if cmd == CMD_ITEM and len(payload) >= 25:
                item = MissionPacket.parse_item(payload)
                seq = MissionPacket.parse_u16(payload, 1)
                self._down_mission.add(item)
                if seq + 1 >= self._down_total:
                    self._finish_download(MISSION_ACCEPTED)
                else:
                    self.listener.on_transfer_status(
                        "Downloading %d/%d" % (seq + 1, self._down_total))
                    self._send_now(MissionPacket.request(seq + 1))
            elif cmd == CMD_ACK:
                self._finish_download(MissionPacket.parse_u8(payload, 1))
            return

        if self._mode == self._SIMPLE and cmd == self._simple_cmd:
            ok = MissionPacket.parse_u8(payload, 1) == 0
            self._cancel_timer()
            self._last_sent = None
            self._mode = self._IDLE
            self.listener.on_transfer_status("Accepted" if ok else "Refused")
            self.listener.on_command_finished(self._simple_cmd, ok)

    def _finish_upload(self, result):
        self._cancel_timer()
        self._last_sent = None
        self._mode = self._IDLE
        self._up_items = []
        self.listener.on_upload_finished(result)

    def _finish_download(self, result):
        self._cancel_timer()
        self._last_sent = None
        self._mode = self._IDLE
        mission = self._down_mission
        self._down_mission = None
        self.listener.on_download_finished(mission, result)

    def _fail(self, message):
        self.listener.on_transfer_status(message)
        if self._mode == self._UPLOAD:
            self.listener.on_upload_finished(MISSION_ERROR)
        elif self._mode == self._DOWNLOAD:
            self.listener.on_download_finished(None, MISSION_ERROR)
        elif self._mode == self._SIMPLE:
            self.listener.on_command_finished(self._simple_cmd, False)
        self._mode = self._IDLE
        self._last_sent = None


# --- local frame (PROTOCOL.md §12) ------------------------------------------

# Waypoints are lat/lon and the drone's position is metres in the launch frame,
# converted flat-earth — right to well beyond the flight radius:
#   north = (lat − lat0) · 111319.49
#   east  = (lon − lon0) · 111319.49 · cos(lat0)
# Waypoint là vĩ/kinh độ còn vị trí drone là mét trong khung cất cánh, đổi theo
# mô hình đất phẳng — đúng tới rất xa ngoài bán kính bay: (xem công thức trên).
M_PER_DEG = 111319.49


def to_north_east(ref_lat, ref_lon, lat, lon):
    north = (lat - ref_lat) * M_PER_DEG
    east = (lon - ref_lon) * M_PER_DEG * math.cos(math.radians(ref_lat))
    return north, east


def to_lat_lon(ref_lat, ref_lon, north, east):
    lat = ref_lat + north / M_PER_DEG
    lon = ref_lon + east / (M_PER_DEG * math.cos(math.radians(ref_lat)))
    return lat, lon
