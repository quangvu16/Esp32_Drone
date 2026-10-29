//
//  PlannerCore.swift
//  Crazyflie client
//
//  The Plan screen's models: mission items carrying the full MAV_CMD table,
//  the Mission Planner `.waypoints` (QGC WPL 110) codec, and the MISSION_*
//  transfer handshake of PROTOCOL.md §12 over CRTP port 7 channel 2. This is
//  the Mission Planner Flight Plan port — the same reader/writer semantics as
//  its `WaypointFile.ReadWaypointFile` / `savewaypoints`, quirks included.
//  Mô hình của màn Plan: các mục nhiệm vụ mang bảng MAV_CMD đầy đủ, codec
//  `.waypoints` (QGC WPL 110) của Mission Planner, và bắt tay MISSION_* của
//  PROTOCOL.md mục 12 trên CRTP cổng 7 kênh 2. Đây là bản chuyển thể màn
//  Flight Plan của Mission Planner — cùng ngữ nghĩa đọc/ghi như
//  `WaypointFile.ReadWaypointFile` / `savewaypoints` của nó, kể cả các điểm lạ.
//
//  Reliability model (§2/§12): the app re-sends its last mission message every
//  500 ms until the expected reply arrives, and gives up after 10 unanswered
//  repeats. The firmware's handlers are idempotent, so those repeats are how a
//  lossy UDP hop stays correct.
//  Mô hình tin cậy (mục 2/12): app gửi lại thông điệp nhiệm vụ cuối mỗi 500 ms
//  cho tới khi nhận được câu trả lời mong đợi, và bỏ cuộc sau 10 lần lặp không
//  được trả lời. Xử lý phía firmware lặp được, nên các lần gửi lại đó là cách
//  một đường truyền UDP hay mất gói vẫn đúng.
//

import Foundation

// MARK: - PlannerItem

/// One Mission Planner grid row: a MAV_CMD id plus its parameters.
/// Một dòng lưới của Mission Planner: id MAV_CMD cộng các tham số của nó.
struct PlannerItem: Equatable {

    // The commands the firmware actually flies (PROTOCOL.md §12).
    // Các lệnh mà firmware thực sự bay (PROTOCOL.md mục 12).
    static let cmdWaypoint = 16
    static let cmdReturnToLaunch = 20
    static let cmdLand = 21
    static let cmdTakeoff = 22
    static let cmdDoJump = 177

    // MAV_FRAME.GLOBAL — what Mission Planner writes for the home row.
    // MAV_FRAME.GLOBAL — Mission Planner ghi cho dòng home.
    static let frameGlobal = 0
    // MAV_FRAME.GLOBAL_RELATIVE_ALT — Mission Planner's default for new rows
    // (its "Relative" alt mode) and what the firmware's items mean by alt.
    // MAV_FRAME.GLOBAL_RELATIVE_ALT — mặc định của Mission Planner cho dòng mới
    // (chế độ độ cao "Relative") và ý nghĩa alt trong mục của firmware.
    static let frameGlobalRelativeAlt = 3

    /// MAV_CMD id; the wire and file truth, kept even when no name is known.
    /// Số id MAV_CMD; dữ liệu gốc trên đường truyền và tệp, giữ nguyên kể cả khi không biết tên.
    var command = PlannerItem.cmdWaypoint

    var p1: Float = 0
    var p2: Float = 0
    var p3: Float = 0
    var p4: Float = 0

    /// WGS-84 degrees.
    /// Độ WGS-84.
    var lat: Double = 0
    var lon: Double = 0

    /// Metres, frame-relative as `frame` says.
    /// Mét, tương đối theo khung như `frame` quy định.
    var alt: Float = 0

    /// MAV_FRAME id; written back verbatim so files round-trip.
    /// Số id MAV_FRAME; ghi lại nguyên văn để tệp không mất dữ liệu.
    var frame = PlannerItem.frameGlobalRelativeAlt

    /// A fresh mission row in Mission Planner's default frame, at a position.
    /// Một dòng nhiệm vụ mới trong khung mặc định của Mission Planner, tại một vị trí.
    static func waypoint(_ latitude: Double, _ longitude: Double, alt: Float) -> PlannerItem {
        var item = PlannerItem()
        item.command = cmdWaypoint
        item.lat = latitude
        item.lon = longitude
        item.alt = alt
        return item
    }

    /// The name Mission Planner's grid shows: the MAV_CMD name, else "UNKNOWN".
    /// Tên mà lưới Mission Planner hiện: tên MAV_CMD, nếu không thì "UNKNOWN".
    var commandName: String { PlannerItem.commandNameOf(command) }

    static func commandNameOf(_ id: Int) -> String {
        if let index = cmdIds.firstIndex(of: id) {
            return cmdNames[index]
        }
        return "UNKNOWN"
    }

    /// Mission Planner's `Enum.Parse(MAV_CMD, …, false)` accepts a number or an
    /// exact enum name (case-sensitive). Anything else is a line invalid.
    /// `Enum.Parse(MAV_CMD, …, false)` của Mission Planner chấp nhận một con số
    /// hoặc tên enum chính xác (phân biệt hoa thường). Phần còn lại là dòng sai.
    static func parseCommand(_ text: String) -> Int {
        if let value = Int(text) {
            // Enum.Parse takes any number, even one with no enum member; the
            // (ushort) cast there rejects what does not fit the storage.
            // Enum.Parse nhận mọi con số, kể cả số không có thành viên enum;
            // ép kiểu (ushort) ở đó từ chối phần không vừa chỗ lưu.
            return (0...0xFFFF).contains(value) ? value : -1
        }
        if let index = cmdNames.firstIndex(of: text) {
            return cmdIds[index]
        }
        return -1
    }

    // The MAV_CMD enum of message_definitions/v1.0/common.xml: value-sorted,
    // names stripped of MAV_CMD_ and then of a leading NAV_ (the same short
    // names Mission Planner's grid shows).
    // Enum MAV_CMD của message_definitions/v1.0/common.xml: sắp theo giá trị,
    // tên đã bỏ MAV_CMD_ rồi bỏ NAV_ đầu (cùng tên ngắn mà lưới Mission
    // Planner hiển thị).
    static let cmdIds: [Int] = [
      16, 17, 18, 19, 20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 35, 36, 80, 81, 82, 84, 85, 92,
      93, 94, 95, 112, 113, 114, 115, 159, 176, 177, 178, 179, 180, 181, 182, 183, 184, 185,
      186, 187, 188, 189, 190, 191, 192, 193, 194, 195, 196, 197, 198, 200, 201, 202, 203, 204,
      205, 206, 207, 208, 209, 210, 211, 212, 213, 214, 220, 221, 222, 223, 224, 240, 241, 242,
      243, 245, 246, 252, 260, 262, 300, 310, 311, 400, 401, 405, 406, 410, 420, 500, 510, 511,
      512, 519, 520, 521, 522, 525, 526, 527, 528, 529, 530, 531, 532, 533, 534, 600, 601, 611,
      1000, 1001, 2000, 2001, 2002, 2003, 2004, 2005, 2010, 2500, 2501, 2502, 2503, 2504, 2505,
      2510, 2511, 2520, 2600, 2800, 3000, 3001, 4000, 4001, 4501, 5000, 5001, 5002, 5003, 5004,
      5100, 5200, 5300, 10001, 30001, 30002, 42006, 42600, 43000, 43001, 43002, 43003, 43004,
      31000, 31001, 31002, 31003, 31004, 31005, 31006, 31007, 31008, 31009, 31010, 31011, 31012,
      31013, 31014, 32000,
    ]
    static let cmdNames: [String] = [
      "WAYPOINT", "LOITER_UNLIM", "LOITER_TURNS", "LOITER_TIME", "RETURN_TO_LAUNCH", "LAND",
      "TAKEOFF", "LAND_LOCAL", "TAKEOFF_LOCAL", "CONTINUE_AND_CHANGE_ALT", "LOITER_TO_ALT",
      "DO_FOLLOW", "DO_FOLLOW_REPOSITION", "DO_ORBIT", "DO_FIGURE_EIGHT", "ARC_WAYPOINT", "ROI",
      "PATHPLANNING", "SPLINE_WAYPOINT", "VTOL_TAKEOFF", "VTOL_LAND", "GUIDED_ENABLE", "DELAY",
      "PAYLOAD_PLACE", "LAST", "CONDITION_DELAY", "CONDITION_CHANGE_ALT", "CONDITION_DISTANCE",
      "CONDITION_YAW", "CONDITION_LAST", "DO_SET_MODE", "DO_JUMP", "DO_CHANGE_SPEED",
      "DO_SET_HOME", "DO_SET_PARAMETER", "DO_SET_RELAY", "DO_REPEAT_RELAY", "DO_SET_SERVO",
      "DO_REPEAT_SERVO", "DO_FLIGHTTERMINATION", "DO_CHANGE_ALTITUDE", "DO_SET_ACTUATOR",
      "DO_RETURN_PATH_START", "DO_LAND_START", "DO_RALLY_LAND", "DO_GO_AROUND", "DO_REPOSITION",
      "DO_PAUSE_CONTINUE", "DO_SET_REVERSE", "DO_SET_ROI_LOCATION", "DO_SET_ROI_WPNEXT_OFFSET",
      "DO_SET_ROI_NONE", "DO_SET_ROI_SYSID", "DO_CONTROL_VIDEO", "DO_SET_ROI",
      "DO_DIGICAM_CONFIGURE", "DO_DIGICAM_CONTROL", "DO_MOUNT_CONFIGURE", "DO_MOUNT_CONTROL",
      "DO_SET_CAM_TRIGG_DIST", "DO_FENCE_ENABLE", "DO_PARACHUTE", "DO_MOTOR_TEST",
      "DO_INVERTED_FLIGHT", "DO_GRIPPER", "DO_AUTOTUNE_ENABLE", "SET_YAW_SPEED",
      "DO_SET_CAM_TRIGG_INTERVAL", "DO_MOUNT_CONTROL_QUAT", "DO_GUIDED_MASTER",
      "DO_GUIDED_LIMITS", "DO_ENGINE_CONTROL", "DO_SET_MISSION_CURRENT", "DO_LAST",
      "PREFLIGHT_CALIBRATION", "PREFLIGHT_SET_SENSOR_OFFSETS", "PREFLIGHT_UAVCAN",
      "PREFLIGHT_STORAGE", "PREFLIGHT_REBOOT_SHUTDOWN", "OVERRIDE_GOTO", "OBLIQUE_SURVEY",
      "DO_SET_STANDARD_MODE", "MISSION_START", "ACTUATOR_TEST", "CONFIGURE_ACTUATOR",
      "COMPONENT_ARM_DISARM", "RUN_PREARM_CHECKS", "ILLUMINATOR_ON_OFF",
      "DO_ILLUMINATOR_CONFIGURE", "GET_HOME_POSITION", "INJECT_FAILURE", "START_RX_PAIR",
      "GET_MESSAGE_INTERVAL", "SET_MESSAGE_INTERVAL", "REQUEST_MESSAGE",
      "REQUEST_PROTOCOL_VERSION", "REQUEST_AUTOPILOT_CAPABILITIES",
      "REQUEST_CAMERA_INFORMATION", "REQUEST_CAMERA_SETTINGS", "REQUEST_STORAGE_INFORMATION",
      "STORAGE_FORMAT", "REQUEST_CAMERA_CAPTURE_STATUS", "REQUEST_FLIGHT_INFORMATION",
      "RESET_CAMERA_SETTINGS", "SET_CAMERA_MODE", "SET_CAMERA_ZOOM", "SET_CAMERA_FOCUS",
      "SET_STORAGE_USAGE", "SET_CAMERA_SOURCE", "JUMP_TAG", "DO_JUMP_TAG",
      "DO_SET_GLOBAL_ORIGIN", "DO_GIMBAL_MANAGER_PITCHYAW", "DO_GIMBAL_MANAGER_CONFIGURE",
      "IMAGE_START_CAPTURE", "IMAGE_STOP_CAPTURE", "REQUEST_CAMERA_IMAGE_CAPTURE",
      "DO_TRIGGER_CONTROL", "CAMERA_TRACK_POINT", "CAMERA_TRACK_RECTANGLE",
      "CAMERA_STOP_TRACKING", "VIDEO_START_CAPTURE", "VIDEO_STOP_CAPTURE",
      "VIDEO_START_STREAMING", "VIDEO_STOP_STREAMING", "REQUEST_VIDEO_STREAM_INFORMATION",
      "REQUEST_VIDEO_STREAM_STATUS", "LOGGING_START", "LOGGING_STOP", "AIRFRAME_CONFIGURATION",
      "CONTROL_HIGH_LATENCY", "PANORAMA_CREATE", "DO_VTOL_TRANSITION",
      "ARM_AUTHORIZATION_REQUEST", "SET_GUIDED_SUBMODE_STANDARD", "SET_GUIDED_SUBMODE_CIRCLE",
      "CONDITION_GATE", "FENCE_RETURN_POINT", "FENCE_POLYGON_VERTEX_INCLUSION",
      "FENCE_POLYGON_VERTEX_EXCLUSION", "FENCE_CIRCLE_INCLUSION", "FENCE_CIRCLE_EXCLUSION",
      "RALLY_POINT", "UAVCAN_GET_NODE_INFO", "DO_SET_SAFETY_SWITCH_STATE", "DO_ADSB_OUT_IDENT",
      "PAYLOAD_PREPARE_DEPLOY", "PAYLOAD_CONTROL_DEPLOY", "FIXED_MAG_CAL_YAW", "DO_WINCH",
      "GUIDED_CHANGE_SPEED", "GUIDED_CHANGE_ALTITUDE", "GUIDED_CHANGE_HEADING",
      "EXTERNAL_POSITION_ESTIMATE", "EXTERNAL_WIND_ESTIMATE", "WAYPOINT_USER_1",
      "WAYPOINT_USER_2", "WAYPOINT_USER_3", "WAYPOINT_USER_4", "WAYPOINT_USER_5",
      "SPATIAL_USER_1", "SPATIAL_USER_2", "SPATIAL_USER_3", "SPATIAL_USER_4", "SPATIAL_USER_5",
      "USER_1", "USER_2", "USER_3", "USER_4", "USER_5", "CAN_FORWARD",
    ]
}

// MARK: - PlannerMission

enum PlannerMissionError: Error {
    case invalidFile
}

/// A Mission Planner mission file: row 0 is home, the rest are the grid rows.
/// The codec is a literal port of `WaypointFile.ReadWaypointFile` and
/// `savewaypoints()` — see the per-step notes below.
/// Tệp nhiệm vụ Mission Planner: dòng 0 là home, phần còn lại là các dòng lưới.
/// Codec là bản chuyển thể nguyên văn `WaypointFile.ReadWaypointFile` và
/// `savewaypoints()` — xem ghi chú từng bước bên dưới.
final class PlannerMission {

    static let wplHeader = "QGC WPL 110"

    /// Mutable like Android's `items()` list — the screen edits rows in place.
    /// Có thể sửa trực tiếp như danh sách `items()` bên Android — màn hình sửa
    /// từng dòng tại chỗ.
    var items: [PlannerItem] = []
    private(set) var invalidLines: [String] = []

    /// Home is row 0 of the file. Its frame/command ride along so a save
    /// round-trips, but the writer always stamps frame 0 / command 16 on it.
    /// Home là dòng 0 của tệp. Frame/lệnh của nó được giữ để việc lưu không mất
    /// dữ liệu, nhưng phần ghi luôn dán frame 0 / lệnh 16 cho nó.
    var home = PlannerItem()

    var size: Int { items.count }

    func add(_ item: PlannerItem) {
        items.append(item)
    }

    func clear() {
        items.removeAll()
        invalidLines.removeAll()
    }

    // MARK: reader (Mission Planner's ReadWaypointFile)

    static func parse(_ text: String) throws -> PlannerMission {
        let mission = PlannerMission()
        // Split("\r?\n") — fold the CR of CRLF files away with the LF.
        // Split("\r?\n") — gộp CR của tệp CRLF vào cùng LF.
        var lines = text.components(separatedBy: "\n")
        if lines.last == "" {
            lines.removeLast()
        }
        for i in lines.indices where lines[i].hasSuffix("\r") {
            lines[i].removeLast()
        }
        guard !lines.isEmpty, lines[0].contains("QGC WPL") else {
            throw PlannerMissionError.invalidFile
        }

        // The reference reads into one list whose row 0 is home. Keep the same
        // shape while parsing — the blank-home quirk only reads naturally that
        // way — and split home from the mission rows at the end.
        // Bản gốc đọc vào một danh sách mà dòng 0 là home. Giữ đúng hình dạng đó
        // khi phân tích — điểm lạ home trống chỉ tự nhiên theo cách đó — rồi tách
        // home khỏi các dòng nhiệm vụ ở cuối.
        var rows: [PlannerItem] = []
        var wpCount = 0
        for line in lines.dropFirst() {
            if line.hasPrefix("#") {
                continue
            }
            // Split(new[] { '\t', ' ', ',' }, RemoveEmptyEntries) — Swift's
            // split drops empty subsequences by default, the same rule.
            // Split(new[] { '\t', ' ', ',' }, RemoveEmptyEntries) — split của
            // Swift mặc định bỏ các dãy con rỗng, cùng quy tắc.
            let fields = line.split(whereSeparator: { $0 == "\t" || $0 == " " || $0 == "," })
            if fields.count <= 9 {
                continue
            }
            do {
                // Check to see if the first wp is index 0/home. If it is not
                // index 0, add a blank home point. (Mission Planner's words —
                // the blank is added before the row is parsed and stays when
                // the parse then fails.)
                // Kiểm tra xem waypoint đầu có phải index 0/home không. Nếu không
                // phải index 0, thêm một điểm home trống. (Nguyên văn Mission
                // Planner — dòng trống được thêm trước khi phân tích dòng và ở lại
                // kể cả khi việc phân tích thất bại.)
                if wpCount == 0 && fields[0] != "0" {
                    rows.append(blankHome())
                }

                var temp = PlannerItem()
                temp.frame = try intField(fields[2])
                temp.command = PlannerItem.parseCommand(String(fields[3]))
                if temp.command < 0 {
                    // Enum.Parse threw there → "Line invalid".
                    // Enum.Parse ném lỗi ở đó → "Line invalid".
                    throw PlannerMissionError.invalidFile
                }
                temp.p1 = try floatField(fields[4])
                if temp.command == 99 {
                    // A legacy waypoint id: Mission Planner rewrites it to 0.
                    // Id waypoint kiểu cũ: Mission Planner viết lại thành 0.
                    temp.command = 0
                }
                temp.alt = try floatField(fields[10])
                temp.lat = try doubleField(fields[8])
                temp.lon = try doubleField(fields[9])
                temp.p2 = try floatField(fields[5])
                temp.p3 = try floatField(fields[6])
                temp.p4 = try floatField(fields[7])

                rows.append(temp)
                wpCount += 1
            } catch {
                // Mission Planner shows "Line invalid" and keeps reading.
                // Mission Planner báo "Line invalid" rồi vẫn đọc tiếp.
                mission.invalidLines.append(line)
            }
        }

        if let first = rows.first {
            mission.home = first
            mission.items = Array(rows.dropFirst())
        }
        return mission
    }

    private static func blankHome() -> PlannerItem {
        var home = PlannerItem()
        home.frame = PlannerItem.frameGlobal
        home.command = 0
        return home
    }

    private static func intField(_ s: Substring) throws -> Int {
        guard let v = Int(s) else { throw PlannerMissionError.invalidFile }
        return v
    }

    private static func floatField(_ s: Substring) throws -> Float {
        // (float)(double.Parse(…)) is how the reference reads alt and p2..p4:
        // parse at double precision, then narrow.
        // (float)(double.Parse(…)) là cách bản gốc đọc alt và p2..p4: phân tích
        // ở độ chính xác double rồi thu hẹp.
        guard let v = Double(s) else { throw PlannerMissionError.invalidFile }
        return Float(v)
    }

    private static func doubleField(_ s: Substring) throws -> Double {
        guard let v = Double(s) else { throw PlannerMissionError.invalidFile }
        return v
    }

    // MARK: writer (Mission Planner's savewaypoints)

    /// Write the file exactly like Mission Planner's `savewaypoints()`: the
    /// header, one home row with frame 0 and command 16, then the mission rows
    /// with seq 1..n. Number formats are its string formats — seven decimals
    /// for the home lat/lng, eight for the rows' p1..p4/lat/lng, six for every
    /// altitude, invariant culture (dot decimal).
    /// Ghi tệp đúng như `savewaypoints()` của Mission Planner: dòng tiêu đề,
    /// một dòng home với frame 0 và lệnh 16, rồi các dòng nhiệm vụ đánh số seq
    /// 1..n. Định dạng số là định dạng chuỗi của nó — bảy chữ số thập phân cho
    /// vĩ/kinh độ home, tám cho p1..p4/vĩ/kinh độ của dòng, sáu cho mọi độ cao,
    /// văn hóa bất biến (dấu chấm thập phân).
    func format() -> String {
        var out = PlannerMission.wplHeader + "\n"

        // Home row: "0\t1\t0\t16\t0\t0\t0\t0\t<lat>\t<lng>\t<alt>\t1". Mission
        // Planner falls back to an all-zero row when its home fields will not
        // parse; here that is the non-finite check below.
        // Dòng home: "0\t1\t0\t16\t0\t0\t0\t0\t<lat>\t<lng>\t<alt>\t1". Mission
        // Planner chuyển sang dòng toàn số 0 khi ô home không phân tích được; ở
        // đây là phép kiểm không hữu hạn bên dưới.
        if home.lat.isFinite && home.lon.isFinite && home.alt.isFinite {
            out += String(format: "0\t1\t0\t16\t0\t0\t0\t0\t%.7f\t%.7f\t%.6f\t1",
                          home.lat, home.lon, Double(home.alt))
        } else {
            out += "0\t1\t0\t0\t0\t0\t0\t0\t0\t0\t0\t1"
        }
        out += "\n"

        for (a, item) in items.enumerated() {
            // seq is the row index + 1; current is always 0 for mission rows.
            // seq là chỉ số dòng + 1; current luôn 0 cho dòng nhiệm vụ.
            out += String(format: "%d\t0\t%d\t%d\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.8f\t%.6f\t1",
                          a + 1, item.frame, item.command,
                          Double(item.p1), Double(item.p2), Double(item.p3), Double(item.p4),
                          item.lat, item.lon, Double(item.alt))
            out += "\n"
        }
        return out
    }
}

// MARK: - MissionPacket

/// The CRTP wire of PROTOCOL.md §12. The app payload is `[cmd][args]`; the
/// header byte the wire adds is port 7 channel 2.
/// Đường truyền CRTP của PROTOCOL.md mục 12. Phần dữ liệu của app là
/// `[cmd][args]`; byte header mà đường truyền thêm là cổng 7 kênh 2.
enum MissionPacket {

    static let channel: UInt8 = 2

    static let cmdCount: UInt8 = 0x30
    static let cmdRequest: UInt8 = 0x31
    static let cmdItem: UInt8 = 0x32
    static let cmdAck: UInt8 = 0x33
    static let cmdRequestList: UInt8 = 0x34
    static let cmdSetCurrent: UInt8 = 0x35
    static let cmdCurrent: UInt8 = 0x36
    static let cmdStart: UInt8 = 0x37

    // MAV_MISSION_RESULT values the handshake can carry.
    // Các giá trị MAV_MISSION_RESULT mà bắt tay có thể mang.
    static let missionAccepted = 0
    static let missionError = 1
    static let missionNoSpace = 4
    static let missionInvalid = 5
    static let missionDenied = 14

    static func packet(_ payload: Data) -> CrtpPacket {
        CrtpPacket(port: .generic, channel: channel, payload: payload)
    }

    static func count(_ n: Int) -> CrtpPacket {
        var payload = Data([cmdCount])
        appendU16(&payload, UInt16(n))
        return packet(payload)
    }

    static func request(_ seq: Int) -> CrtpPacket {
        var payload = Data([cmdRequest])
        appendU16(&payload, UInt16(seq))
        return packet(payload)
    }

    /// One ITEM upload: seq u16, command u16, p1 f32, p2 f32, lat_e7 i32,
    /// lon_e7 i32, alt f32. p3/p4 stay home — the 30-byte ceiling of §12; the
    /// full four parameters travel in `.waypoints` files instead.
    /// Một mục ITEM gửi lên: seq u16, lệnh u16, p1 f32, p2 f32, lat_e7 i32,
    /// lon_e7 i32, alt f32. p3/p4 ở nhà — trần 30 byte của mục 12; cả bốn tham
    /// số đầy đủ đi trong tệp `.waypoints`.
    static func item(_ item: PlannerItem, seq: Int) -> CrtpPacket {
        var payload = Data([cmdItem])
        appendU16(&payload, UInt16(seq))
        appendU16(&payload, UInt16(item.command))
        appendF32(&payload, item.p1)
        appendF32(&payload, item.p2)
        appendI32(&payload, e7(item.lat))
        appendI32(&payload, e7(item.lon))
        appendF32(&payload, item.alt)
        return packet(payload)
    }

    static func requestList() -> CrtpPacket {
        packet(Data([cmdRequestList]))
    }

    static func setCurrent(_ seq: Int) -> CrtpPacket {
        var payload = Data([cmdSetCurrent])
        appendU16(&payload, UInt16(seq))
        return packet(payload)
    }

    static func start() -> CrtpPacket {
        packet(Data([cmdStart]))
    }

    /// Degrees × 1e7 as the wire carries them: floor(x + 0.5), the same
    /// half-up rounding the Android client's `Math.round` does.
    /// Độ × 1e7 như đường truyền mang: floor(x + 0.5), cùng làm tròn lên ở nửa
    /// như `Math.round` của client Android.
    private static func e7(_ degrees: Double) -> Int32 {
        Int32((degrees * 1e7 + 0.5).rounded(.down))
    }

    // MARK: readers

    /// Reads `seq u16` at `offset`, or -1 when the payload is too short — the
    /// same sentinel the Android client uses.
    /// Đọc `seq u16` tại `offset`, hoặc -1 khi phần dữ liệu quá ngắn — cùng giá
    /// trị đánh dấu mà client Android dùng.
    static func parseU16(_ payload: Data, _ offset: Int) -> Int {
        let b = [UInt8](payload)
        guard b.count > offset + 1 else { return -1 }
        return Int(b[offset]) | (Int(b[offset + 1]) << 8)
    }

    static func parseU8(_ payload: Data, _ offset: Int) -> Int {
        let b = [UInt8](payload)
        guard b.count > offset else { return -1 }
        return Int(b[offset])
    }

    /// Decodes an ITEM reply: `[cmd][seq u16][command u16][p1][p2][lat_e7][lon_e7][alt]`.
    /// The wire carries no frame — the caller stamps `GLOBAL_RELATIVE_ALT`,
    /// the frame the firmware means by alt.
    /// Giải mã một câu trả lời ITEM: `[cmd][seq u16][lệnh u16][p1][p2][lat_e7][lon_e7][alt]`.
    /// Đường truyền không mang frame — người gọi dán `GLOBAL_RELATIVE_ALT`,
    /// khung mà firmware hiểu cho alt.
    static func parseItem(_ payload: Data) -> PlannerItem? {
        let b = [UInt8](payload)
        guard b.count >= 25 else { return nil }
        var item = PlannerItem()
        item.frame = PlannerItem.frameGlobalRelativeAlt
        item.command = Int(u16At(b, 3))
        item.p1 = f32At(b, 5)
        item.p2 = f32At(b, 9)
        item.lat = Double(i32At(b, 13)) / 1e7
        item.lon = Double(i32At(b, 17)) / 1e7
        item.alt = f32At(b, 21)
        return item
    }

    // MARK: little-endian helpers

    private static func appendU16(_ data: inout Data, _ v: UInt16) {
        withUnsafeBytes(of: v.littleEndian) { data.append(contentsOf: $0) }
    }

    private static func appendI32(_ data: inout Data, _ v: Int32) {
        withUnsafeBytes(of: v.littleEndian) { data.append(contentsOf: $0) }
    }

    private static func appendF32(_ data: inout Data, _ v: Float) {
        withUnsafeBytes(of: v.bitPattern.littleEndian) { data.append(contentsOf: $0) }
    }

    private static func u16At(_ b: [UInt8], _ o: Int) -> UInt16 {
        UInt16(b[o]) | (UInt16(b[o + 1]) << 8)
    }

    private static func i32At(_ b: [UInt8], _ o: Int) -> Int32 {
        Int32(bitPattern: UInt32(b[o]) | (UInt32(b[o + 1]) << 8)
                | (UInt32(b[o + 2]) << 16) | (UInt32(b[o + 3]) << 24))
    }

    private static func f32At(_ b: [UInt8], _ o: Int) -> Float {
        Float(bitPattern: UInt32(b[o]) | (UInt32(b[o + 1]) << 8)
                | (UInt32(b[o + 2]) << 16) | (UInt32(b[o + 3]) << 24))
    }
}

// MARK: - MissionTransfer

/// What the transfer tells the screen while it works.
/// Những gì phiên truyền báo cho màn hình trong khi làm việc.
protocol MissionTransferListener: AnyObject {
    func transferStatus(_ message: String)
    func uploadFinished(_ result: Int)
    func downloadFinished(_ mission: PlannerMission?, _ result: Int)
    /// cmd is one of `cmdSetCurrent` / `cmdStart`; ok is the status byte.
    /// cmd là một trong `cmdSetCurrent` / `cmdStart`; ok là byte status.
    func commandFinished(_ cmd: UInt8, _ ok: Bool)
    /// Async MISSION_CURRENT telemetry from the drone.
    /// Telemetry MISSION_CURRENT bất đồng bộ từ drone.
    func currentItem(_ seq: Int)
}

/// Runs the MISSION_* handshake over CRTP port 7 channel 2 and re-sends its
/// last message every 500 ms until the expected reply arrives.
/// Chạy bắt tay MISSION_* trên CRTP cổng 7 kênh 2 và gửi lại thông điệp cuối
/// mỗi 500 ms cho tới khi nhận được câu trả lời mong đợi.
///
/// Upload:  COUNT n → REQUEST 0 → ITEM 0 → REQUEST 1 → … → ITEM n-1 → ACK 0.
/// Download: REQUEST_LIST → COUNT n → REQUEST 0 → ITEM 0 → … → ITEM n-1.
///
/// All state lives on the main thread; the link's receive queue only posts in.
/// Mọi trạng thái nằm ở luồng chính; hàng đợi nhận của link chỉ chuyển vào.
final class MissionTransfer {

    /// PROTOCOL.md §12: the app repeats its last mission message at this rate.
    /// PROTOCOL.md mục 12: app lặp thông điệp nhiệm vụ cuối với nhịp này.
    static let resendInterval: TimeInterval = 0.5

    /// Give up after this many unanswered repeats (5 s of silence).
    /// Bỏ cuộc sau số lần lặp không được trả lời này (5 giây im lặng).
    static let maxTries = 10

    private enum Mode {
        case idle, upload, download, simple
    }

    private weak var listener: MissionTransferListener?
    private weak var client: CrtpClient?

    private var mode: Mode = .idle
    private var lastSent: CrtpPacket?
    private var tries = 0
    private var resendTimer: Timer?

    private var upItems: [PlannerItem] = []
    private var upTotal = 0

    private var downMission: PlannerMission?
    private var downTotal = 0

    private var simpleCmd: UInt8 = 0

    init(listener: MissionTransferListener) {
        self.listener = listener
    }

    deinit {
        resendTimer?.invalidate()
    }

    /// Start listening on the link; safe to call again with the same link.
    /// Bắt đầu nghe trên link; gọi lại với cùng link vẫn an toàn.
    func attach(_ client: CrtpClient) {
        if self.client === client, client.onMissionPacket != nil {
            return
        }
        detach()
        self.client = client
        client.onMissionPacket = { [weak self] payload in
            DispatchQueue.main.async {
                self?.onReply(payload)
            }
        }
    }

    /// Stop listening and abandon any transfer in flight.
    /// Nghe ngừng và hủy mọi phiên truyền đang chạy.
    func detach() {
        resendTimer?.invalidate()
        resendTimer = nil
        client?.onMissionPacket = nil
        client = nil
        mode = .idle
        lastSent = nil
    }

    var isBusy: Bool { mode != .idle }

    /// Upload the items to the drone's store (an empty list clears it).
    /// Tải các mục lên kho của drone (danh sách rỗng = xóa).
    func upload(_ items: [PlannerItem]) {
        guard startTransfer(.upload, "Uploading 0/\(items.count)") else { return }
        upItems = items
        upTotal = items.count
        if upTotal == 0 {
            listener?.transferStatus("Clearing stored mission")
        }
        send(MissionPacket.count(upTotal))
    }

    /// Download whatever the drone holds into a fresh mission.
    /// Tải về nội dung drone đang giữ thành một nhiệm vụ mới.
    func download() {
        guard startTransfer(.download, "Requesting mission list") else { return }
        downMission = PlannerMission()
        send(MissionPacket.requestList())
    }

    /// Jump to an item; the drone answers `cmdSetCurrent` + status.
    /// Nhảy tới một mục; drone trả `cmdSetCurrent` + status.
    func setCurrent(_ seq: Int) {
        startSimple(MissionPacket.setCurrent(seq), cmd: MissionPacket.cmdSetCurrent,
                    status: "Setting current item")
    }

    /// Fly the stored mission; the drone answers `cmdStart` + status.
    /// Bay nhiệm vụ đã lưu; drone trả `cmdStart` + status.
    func startMission() {
        startSimple(MissionPacket.start(), cmd: MissionPacket.cmdStart,
                    status: "Starting mission")
    }

    private func startTransfer(_ newMode: Mode, _ status: String) -> Bool {
        guard mode == .idle else {
            fail("Another transfer is still running")
            return false
        }
        guard client != nil else {
            fail("No drone connected")
            return false
        }
        mode = newMode
        listener?.transferStatus(status)
        return true
    }

    private func startSimple(_ packet: CrtpPacket, cmd: UInt8, status: String) {
        guard startTransfer(.simple, status) else { return }
        simpleCmd = cmd
        send(packet)
    }

    // MARK: state machine, main thread only

    private func send(_ packet: CrtpPacket) {
        lastSent = packet
        tries = 0
        client?.send(packet)
        resendTimer?.invalidate()
        resendTimer = Timer.scheduledTimer(timeInterval: MissionTransfer.resendInterval,
                                           target: self, selector: #selector(resend),
                                           userInfo: nil, repeats: true)
    }

    @objc private func resend() {
        guard let packet = lastSent, let client = client else { return }
        if tries >= MissionTransfer.maxTries {
            // Stop before the repeat becomes its own hazard: the drone's
            // handlers are idempotent, but a dead link will never answer.
            // Dừng trước khi việc lặp tự gây hại: xử lý phía drone lặp được,
            // nhưng link chết sẽ không bao giờ trả lời.
            let failedMode = mode
            lastSent = nil
            mode = .idle
            switch failedMode {
            case .upload: listener?.uploadFinished(MissionPacket.missionError)
            case .download: listener?.downloadFinished(nil, MissionPacket.missionError)
            case .simple: listener?.commandFinished(simpleCmd, false)
            case .idle: break
            }
            listener?.transferStatus("No reply from the drone")
            return
        }
        tries += 1
        client.send(packet)
    }

    private func onReply(_ payload: Data) {
        guard !payload.isEmpty else { return }
        let cmd = payload[0]

        // MISSION_CURRENT telemetry is welcome in any state.
        // Telemetry MISSION_CURRENT được chào đón ở mọi trạng thái.
        if cmd == MissionPacket.cmdCurrent {
            let seq = MissionPacket.parseU16(payload, 1)
            if seq >= 0 {
                listener?.currentItem(seq)
            }
            return
        }

        switch mode {
        case .upload:
            guard let sent = lastSent else { return }
            let sentCmd = sent.payload.first ?? 0
            if sentCmd == MissionPacket.cmdCount {
                if cmd == MissionPacket.cmdRequest {
                    let seq = MissionPacket.parseU16(payload, 1)
                    if seq < 0 || seq >= upTotal {
                        finishUpload(MissionPacket.missionInvalid)
                    } else {
                        send(MissionPacket.item(upItems[seq], seq: seq))
                    }
                } else if cmd == MissionPacket.cmdAck {
                    // COUNT with 0 items clears the store and ACKs immediately.
                    // COUNT với 0 mục xóa kho và ACK ngay.
                    finishUpload(MissionPacket.parseU8(payload, 1))
                }
            } else if sentCmd == MissionPacket.cmdItem {
                if cmd == MissionPacket.cmdRequest {
                    let seq = MissionPacket.parseU16(payload, 1)
                    if seq < 0 || seq >= upTotal {
                        finishUpload(MissionPacket.missionInvalid)
                    } else {
                        listener?.transferStatus("Uploading \(seq)/\(upTotal)")
                        send(MissionPacket.item(upItems[seq], seq: seq))
                    }
                } else if cmd == MissionPacket.cmdAck {
                    let type = MissionPacket.parseU8(payload, 1)
                    if type == MissionPacket.missionAccepted {
                        listener?.transferStatus("Mission uploaded")
                    }
                    finishUpload(type)
                }
            }

        case .download:
            guard let sent = lastSent else { return }
            let sentCmd = sent.payload.first ?? 0
            if sentCmd == MissionPacket.cmdRequestList {
                if cmd == MissionPacket.cmdCount {
                    downTotal = MissionPacket.parseU16(payload, 1)
                    if downTotal <= 0 {
                        finishDownload(MissionPacket.missionAccepted)
                    } else {
                        listener?.transferStatus("Downloading 0/\(downTotal)")
                        send(MissionPacket.request(0))
                    }
                } else if cmd == MissionPacket.cmdAck {
                    finishDownload(MissionPacket.parseU8(payload, 1))
                }
            } else if sentCmd == MissionPacket.cmdRequest {
                if cmd == MissionPacket.cmdItem {
                    guard let item = MissionPacket.parseItem(payload) else {
                        finishDownload(MissionPacket.missionInvalid)
                        return
                    }
                    let seq = MissionPacket.parseU16(payload, 1)
                    downMission?.add(item)
                    if seq + 1 >= downTotal {
                        finishDownload(MissionPacket.missionAccepted)
                    } else {
                        listener?.transferStatus("Downloading \(seq + 1)/\(downTotal)")
                        send(MissionPacket.request(seq + 1))
                    }
                } else if cmd == MissionPacket.cmdAck {
                    finishDownload(MissionPacket.parseU8(payload, 1))
                }
            }

        case .simple:
            if cmd == simpleCmd {
                let ok = MissionPacket.parseU8(payload, 1) == 0
                stopResend()
                listener?.transferStatus(ok ? "Accepted" : "Refused")
                listener?.commandFinished(simpleCmd, ok)
            }

        case .idle:
            break
        }
    }

    private func finishUpload(_ result: Int) {
        stopResend()
        upItems = []
        listener?.uploadFinished(result)
    }

    private func finishDownload(_ result: Int) {
        stopResend()
        let mission = downMission
        downMission = nil
        listener?.downloadFinished(mission, result)
    }

    private func stopResend() {
        resendTimer?.invalidate()
        resendTimer = nil
        lastSent = nil
        mode = .idle
    }

    private func fail(_ message: String) {
        listener?.transferStatus(message)
        switch mode {
        case .upload: listener?.uploadFinished(MissionPacket.missionError)
        case .download: listener?.downloadFinished(nil, MissionPacket.missionError)
        case .simple: listener?.commandFinished(simpleCmd, false)
        case .idle: break
        }
        mode = .idle
        lastSent = nil
    }
}
