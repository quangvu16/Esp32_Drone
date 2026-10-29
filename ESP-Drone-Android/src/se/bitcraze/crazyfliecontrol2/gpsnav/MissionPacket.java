package se.bitcraze.crazyfliecontrol2.gpsnav;

import java.nio.ByteBuffer;

import se.bitcraze.crazyflie.lib.crtp.CrtpPacket;
import se.bitcraze.crazyflie.lib.crtp.CrtpPort;

/**
 * Mission transfer on CRTP port 7, channel 2 (PROTOCOL.md §12) — the same
 * MISSION_* handshake MAVLink uses, re-packed for the 30-byte CRTP payload.
 * The drone's handler is {@code crtpHandleMission()}; every command is
 * idempotent so the 500 ms resend can repeat it safely.
 * <p>
 * Truyền nhiệm vụ trên CRTP port 7, kênh 2 (PROTOCOL.md mục 12) — đúng bắt tay
 * MISSION_* của MAVLink, đóng gói lại cho gói CRTP 30 byte. Xử lý phía drone
 * là {@code crtpHandleMission()}; mọi lệnh đều lặp được nên việc gửi lại sau
 * 500 ms có thể an toàn lặp lại.
 *
 * <p>Wire layouts, command byte first (all little-endian):
 * <pre>
 *   COUNT        0x30  u16 count                      app → drone
 *   REQUEST      0x31  u16 seq                        drone → app (upload)
 *   ITEM         0x32  u16 seq, u16 command, f32 p1,
 *                      f32 p2, i32 lat_e7, i32 lon_e7,
 *                      f32 alt                         both ways
 *   ACK          0x33  u8 type                        drone → app
 *   REQUEST_LIST 0x34                                 app → drone
 *   SET_CURRENT  0x35  u16 seq                        app → drone
 *                                                    → drone replies 0x35, u8 status
 *   CURRENT      0x36  u16 seq                        drone → app (telemetry)
 *   START        0x37                                 app → drone
 *                                                    → drone replies 0x37, u8 status
 * </pre>
 * p3/p4 do not fit the 30-byte CRTP ceiling and are sent as zero — they live
 * in the .waypoints file and on the MAVLink bridge (§13) only.
 * p3/p4 không vừa trần 30 byte của CRTP nên gửi bằng 0 — chúng chỉ có trong tệp
 * .waypoints và trên cầu nối MAVLink (mục 13).
 */
public class MissionPacket extends CrtpPacket {

    /** Port 7, channel 2 — the firmware's {@code MSN_HDR}. */
    /** Port 7, kênh 2 — {@code MSN_HDR} của firmware. */
    public static final int CHANNEL = 2;

    public static final int CMD_COUNT = 0x30;
    public static final int CMD_REQUEST = 0x31;
    public static final int CMD_ITEM = 0x32;
    public static final int CMD_ACK = 0x33;
    public static final int CMD_REQUEST_LIST = 0x34;
    public static final int CMD_SET_CURRENT = 0x35;
    public static final int CMD_CURRENT = 0x36;
    public static final int CMD_START = 0x37;

    // MAV_MISSION results (PROTOCOL.md §12).
    // Kết quả MAV_MISSION (PROTOCOL.md mục 12).
    public static final int MISSION_ACCEPTED = 0;
    public static final int MISSION_ERROR = 1;
    public static final int MISSION_NO_SPACE = 4;
    public static final int MISSION_INVALID = 5;
    public static final int MISSION_DENIED = 14;

    private final int mCmd;
    private final byte[] mArgs;

    private MissionPacket(int cmd, byte[] args) {
        super(CHANNEL, CrtpPort.COMMANDER_GENERIC);
        this.mCmd = cmd;
        this.mArgs = args;
        setExpectedReply(new byte[] {(byte) cmd});
    }

    /** Start an upload of {@code count} items (0 clears the stored mission). */
    /** Bắt đầu tải lên {@code count} mục (0 = xóa nhiệm vụ đang lưu). */
    public static MissionPacket count(int count) {
        ByteBuffer buffer = ByteBuffer.allocate(2).order(BYTE_ORDER);
        buffer.putShort((short) count);
        return new MissionPacket(CMD_COUNT, buffer.array());
    }

    /** Ask for the item at {@code seq} (also how an upload pulls the next one). */
    /** Yêu cầu mục tại {@code seq} (cũng là cách tải lên xin mục tiếp theo). */
    public static MissionPacket request(int seq) {
        ByteBuffer buffer = ByteBuffer.allocate(2).order(BYTE_ORDER);
        buffer.putShort((short) seq);
        return new MissionPacket(CMD_REQUEST, buffer.array());
    }

    /**
     * One mission item. The lat/lon travel as degrees × 1e7, exactly the
     * MISSION_ITEM_INT integer encoding the firmware stores (§12).
     * <p>
     * Một mục nhiệm vụ. Vĩ độ/kinh độ truyền theo độ × 1e7, đúng mã hóa số
     * nguyên MISSION_ITEM_INT mà firmware lưu (mục 12).
     */
    public static MissionPacket item(PlannerItem item, int seq) {
        ByteBuffer buffer = ByteBuffer.allocate(24).order(BYTE_ORDER);
        buffer.putShort((short) seq);
        buffer.putShort((short) item.command);
        buffer.putFloat(item.p1);
        buffer.putFloat(item.p2);
        buffer.putInt((int) Math.round(item.lat * 1e7));
        buffer.putInt((int) Math.round(item.lon * 1e7));
        buffer.putFloat(item.alt);
        return new MissionPacket(CMD_ITEM, buffer.array());
    }

    /** Ask the drone how many items it holds, before downloading. */
    /** Hỏi drone đang giữ bao nhiêu mục, trước khi tải về. */
    public static MissionPacket requestList() {
        return new MissionPacket(CMD_REQUEST_LIST, new byte[0]);
    }

    /** Make {@code seq} the current item (jumps counters reset, §12). */
    /** Đặt {@code seq} thành mục hiện tại (bộ đếm jump được reset, mục 12). */
    public static MissionPacket setCurrent(int seq) {
        ByteBuffer buffer = ByteBuffer.allocate(2).order(BYTE_ORDER);
        buffer.putShort((short) seq);
        return new MissionPacket(CMD_SET_CURRENT, buffer.array());
    }

    /** Fly the stored mission (arms first, like ArduCopter's AUTO). */
    /** Bay nhiệm vụ đã lưu (arm trước, như AUTO của ArduCopter). */
    public static MissionPacket start() {
        return new MissionPacket(CMD_START, new byte[0]);
    }

    /** The command byte of any mission packet, for matching replies. */
    /** Byte lệnh của mọi gói nhiệm vụ, để so khớp câu trả lời. */
    public int getCmd() {
        return mCmd;
    }

    /** Parse the u16 argument of COUNT / REQUEST / CURRENT payloads. */
    /** Phân tích tham số u16 của payload COUNT / REQUEST / CURRENT. */
    public static int parseU16(byte[] payload, int offset) {
        if (payload.length < offset + 2) {
            return -1;
        }
        return (payload[offset] & 0xFF) | ((payload[offset + 1] & 0xFF) << 8);
    }

    /** Parse the u8 argument of ACK / SET_CURRENT / START payloads. */
    /** Phân tích tham số u8 của payload ACK / SET_CURRENT / START. */
    public static int parseU8(byte[] payload, int offset) {
        if (payload.length < offset + 1) {
            return -1;
        }
        return payload[offset] & 0xFF;
    }

    /** Turn a downloaded ITEM payload back into an item (p3/p4 are zero). */
    /** Đưa payload ITEM tải về trở lại thành mục (p3/p4 bằng 0). */
    public static PlannerItem parseItem(byte[] payload) {
        PlannerItem item = new PlannerItem();
        ByteBuffer buffer = ByteBuffer.wrap(payload).order(BYTE_ORDER);
        buffer.get();                            // the command byte itself
        buffer.getShort();                       // seq — the caller knows it
        item.command = buffer.getShort() & 0xFFFF;
        item.p1 = buffer.getFloat();
        item.p2 = buffer.getFloat();
        item.lat = buffer.getInt() / 1e7;
        item.lon = buffer.getInt() / 1e7;
        item.alt = buffer.getFloat();
        item.frame = PlannerItem.FRAME_GLOBAL_RELATIVE_ALT;
        return item;
    }

    @Override
    protected void serializeData(ByteBuffer buffer) {
        buffer.put((byte) mCmd);
        buffer.put(mArgs);
    }

    @Override
    protected int getDataByteCount() {
        return 1 + mArgs.length;
    }

    @Override
    public String toString() {
        return "MissionPacket: cmd: 0x" + Integer.toHexString(mCmd);
    }
}
