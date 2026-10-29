package se.bitcraze.crazyfliecontrol2.gpsnav;

/**
 * One mission item in the Mission Planner sense: the same fields the QGC WPL
 * 110 file and the MAVLink MISSION_ITEM_INT message carry — command plus four
 * parameters, latitude/longitude and altitude (PROTOCOL.md §12).
 * <p>
 * Một mục nhiệm vụ theo nghĩa Mission Planner: đúng các trường mà tệp QGC WPL
 * 110 và thông điệp MAVLink MISSION_ITEM_INT mang — lệnh cộng bốn tham số,
 * vĩ độ/kinh độ và độ cao (PROTOCOL.md mục 12).
 *
 * <p>The command column is the MAV_CMD enum, exactly as Mission Planner's grid
 * uses it: known ids display the enum name ({@code "WAYPOINT"}), unknown ids
 * display {@code "UNKNOWN"} while keeping their numeric id, and a file may name
 * a command or give its number. The table below is the MAV_CMD enum of the
 * MAVLink common dialect (171 entries), names stripped the way Mission
 * Planner's C# generator strips them (MAV_CMD_NAV_WAYPOINT → WAYPOINT,
 * MAV_CMD_DO_JUMP → DO_JUMP).
 * <p>
 * Cột lệnh là enum MAV_CMD, đúng như lưới của Mission Planner: id đã biết hiện
 * tên enum ({@code "WAYPOINT"}), id không biết hiện {@code "UNKNOWN"} nhưng vẫn
 * giữ số id, và tệp có thể đặt tên lệnh hoặc đưa số. Bảng dưới là enum MAV_CMD
 * của dialect common MAVLink (171 mục), tên được cắt theo cách trình sinh C#
 * của Mission Planner (MAV_CMD_NAV_WAYPOINT → WAYPOINT, MAV_CMD_DO_JUMP → DO_JUMP).
 */
public class PlannerItem {

    // The commands the firmware actually flies (PROTOCOL.md §12).
    // Các lệnh mà firmware thực sự bay (PROTOCOL.md mục 12).
    public static final int CMD_WAYPOINT = 16;
    public static final int CMD_RETURN_TO_LAUNCH = 20;
    public static final int CMD_LAND = 21;
    public static final int CMD_TAKEOFF = 22;
    public static final int CMD_DO_JUMP = 177;

    // MAV_FRAME.GLOBAL — what Mission Planner writes for the home row.
    // MAV_FRAME.GLOBAL — Mission Planner ghi cho dòng home.
    public static final int FRAME_GLOBAL = 0;
    // MAV_FRAME.GLOBAL_RELATIVE_ALT — Mission Planner's default for new rows
    // (its "Relative" alt mode) and what the firmware's items mean by alt.
    // MAV_FRAME.GLOBAL_RELATIVE_ALT — mặc định của Mission Planner cho dòng mới
    // (chế độ độ cao "Relative") và ý nghĩa alt trong mục của firmware.

    public static final int FRAME_GLOBAL_RELATIVE_ALT = 3;

    /** MAV_CMD id; the wire and file truth, kept even when no name is known. */
    /** Số id MAV_CMD; dữ liệu gốc trên đường truyền và tệp, giữ nguyên kể cả khi không biết tên. */
    public int command;

    public float p1;
    public float p2;
    public float p3;
    public float p4;

    /** WGS-84 degrees. */
    /** Độ WGS-84. */
    public double lat;
    public double lon;

    /** Metres, frame-relative as {@link #frame} says. */
    /** Mét, tương đối theo khung như {@link #frame} quy định. */
    public float alt;

    /** MAV_FRAME id; written back verbatim so files round-trip. */
    /** Số id MAV_FRAME; ghi lại nguyên văn để tệp không mất dữ liệu. */
    public int frame;

    public PlannerItem() {
        this.frame = FRAME_GLOBAL_RELATIVE_ALT;
    }

    /** A fresh mission row in Mission Planner's default frame, at a position. */
    /** Một dòng nhiệm vụ mới trong khung mặc định của Mission Planner, tại một vị trí. */
    public static PlannerItem waypoint(double latitude, double longitude, float altitude) {
        PlannerItem item = new PlannerItem();
        item.command = CMD_WAYPOINT;
        item.lat = latitude;
        item.lon = longitude;
        item.alt = altitude;
        return item;
    }

    /** The name Mission Planner's grid shows: the MAV_CMD name, else "UNKNOWN". */
    /** Tên mà lưới Mission Planner hiện: tên MAV_CMD, nếu không thì "UNKNOWN". */
    public String commandName() {
        return commandNameOf(command);
    }

    public static String commandNameOf(int id) {
        for (int i = 0; i < CMD_IDS.length; i++) {
            if (CMD_IDS[i] == id) {
                return CMD_NAMES[i];
            }
        }
        return "UNKNOWN";
    }

    /**
     * Mission Planner parses the file's command cell with
     * {@code Enum.Parse(typeof(MAV_CMD), text, false)}: a number or the exact
     * enum name, case-sensitive. Anything else is an invalid line there — this
     * returns -1 for the caller to report the same way.
     * <p>
     * Mission Planner ô lệnh trong tệp bằng {@code Enum.Parse(typeof(MAV_CMD),
     * text, false)}: một con số hoặc đúng tên enum, phân biệt hoa thường. Nội
     * dung khác là dòng không hợp lệ bên đó — hàm này trả -1 để nơi gọi báo như vậy.
     */
    public static int parseCommand(String text) {
        if (text == null) {
            return -1;
        }
        try {
            // Enum.Parse takes any number, even one with no enum member.
            // Enum.Parse nhận mọi con số, kể cả số không có thành viên enum.
            int value = Integer.parseInt(text);
            // (ushort) cast there rejects what does not fit the storage.
            // Ép kiểu (ushort) ở đó từ chối phần không vừa chỗ lưu.
            return (value >= 0 && value <= 0xFFFF) ? value : -1;
        } catch (NumberFormatException ignored) {
            // Not a number — fall through to the name table.
            // Không phải số — thử tiếp bảng tên.
        }
        for (int i = 0; i < CMD_IDS.length; i++) {
            if (CMD_NAMES[i].equals(text)) {
                return CMD_IDS[i];
            }
        }
        return -1;
    }

    // The MAV_CMD enum of the MAVLink common dialect, generated from
    // message_definitions/v1.0/common.xml. Parallel arrays, the same shape the
    // firmware uses for its crc_extra table.
    // Enum MAV_CMD của dialect common MAVLink, sinh từ
    // message_definitions/v1.0/common.xml. Hai mảng song song, cùng hình dạng
    // với bảng crc_extra của firmware.
  private static final int[] CMD_IDS = {
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
  };
  private static final String[] CMD_NAMES = {
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
  };

    static {
        if (CMD_IDS.length != CMD_NAMES.length) {
            throw new IllegalStateException("MAV_CMD table columns differ in length");
        }
    }
}
