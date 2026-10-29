/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Copyright (C) 2015 Bitcraze AB
 *
 * Crazyflie Nano Quadcopter Client
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 */

package se.bitcraze.crazyflie.lib.crtp;

import java.nio.ByteBuffer;

/**
 * Navigation extension on CRTP port 7, channel 1 (PROTOCOL.md section 6).
 * Phần mở rộng định hướng trên CRTP port 7, channel 1 (PROTOCOL.md mục 6).
 *
 * <p>The phone supplies the target as a north/east offset from the launch point; the drone has no
 * GPS and no compass and only estimates where it is. Offsets are rotated into the launch frame by
 * the firmware, which is defined at {@code ALIGN}:
 * <p>Điện thoại cung cấp mục tiêu dạng độ lệch bắc/đông so với điểm cất cánh; drone không có
 * GPS và la bàn, chỉ ước lượng vị trí của mình. Firmware quay độ lệch vào khung cất cánh,
 * khung này được định nghĩa tại {@code ALIGN}:
 *
 * <pre>
 *   x = n*cos(alpha) + e*sin(alpha)    // +x = where the nose pointed at ALIGN
 *                                       // +x = hướng mũi chỉ tại ALIGN
 *   y = n*sin(alpha) - e*cos(alpha)    // +y = 90 deg LEFT of that
 *                                       // +y = 90 độ BÊN TRÁI hướng đó
 * </pre>
 */
public class NavPacket extends CrtpPacket {

    /** Port 7, channel 1 — the firmware's {@code NAV_HDR}. */
    /** Port 7, channel 1 — {@code NAV_HDR} của firmware. */
    public static final int CHANNEL = 1;

    public static final int CMD_TARGET = 0x20;
    public static final int CMD_RESET = 0x21;
    public static final int CMD_ALIGN = 0x22;
    public static final int CMD_ORIGIN = 0x23;

    private final int mCmd;
    private final byte[] mArgs;

    private NavPacket(int cmd, byte[] args) {
        super(CHANNEL, CrtpPort.COMMANDER_GENERIC);
        this.mCmd = cmd;
        this.mArgs = args;
        setExpectedReply(new byte[] {(byte) cmd});
    }

    /** North/east target offset from the launch point, in metres. */
    /** Độ lệch mục tiêu bắc/đông so với điểm cất cánh, tính bằng mét. */
    public static NavPacket target(float northMeters, float eastMeters) {
        ByteBuffer buffer = ByteBuffer.allocate(8).order(BYTE_ORDER);
        buffer.putFloat(northMeters);
        buffer.putFloat(eastMeters);
        return new NavPacket(CMD_TARGET, buffer.array());
    }

    public static NavPacket reset() {
        return new NavPacket(CMD_RESET, new byte[0]);
    }

    /** Compass bearing of the nose at launch, in degrees. Starts a fresh launch frame. */
    /** Phương hướng la bàn của mũi máy khi cất cánh, tính bằng độ. Bắt đầu một khung cất cánh mới. */
    public static NavPacket align(float alphaDegrees) {
        ByteBuffer buffer = ByteBuffer.allocate(4).order(BYTE_ORDER);
        buffer.putFloat(alphaDegrees);
        return new NavPacket(CMD_ALIGN, buffer.array());
    }

    /**
     * Launch latitude/longitude in WGS-84 degrees — the only bridge between the
     * dead-reckoned local frame and mission/telemetry coordinates (PROTOCOL.md §12).
     * Send it with ALIGN before flying a mission with latitude/longitude items.
     * <p>
     * Vĩ độ/kinh độ điểm cất cánh theo WGS-84 — cầu nối duy nhất giữa khung cục bộ
     * tự dẫn đường và tọa độ nhiệm vụ/telemetry (PROTOCOL.md mục 12). Gửi kèm ALIGN
     * trước khi bay nhiệm vụ có mục vĩ độ/kinh độ.
     */
    public static NavPacket origin(double latitude, double longitude) {
        ByteBuffer buffer = ByteBuffer.allocate(16).order(BYTE_ORDER);
        buffer.putDouble(latitude);
        buffer.putDouble(longitude);
        return new NavPacket(CMD_ORIGIN, buffer.array());
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
        return "NavPacket: cmd: 0x" + Integer.toHexString(mCmd);
    }
}
