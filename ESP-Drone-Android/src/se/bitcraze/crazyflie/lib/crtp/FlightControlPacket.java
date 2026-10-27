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
 * Flight-control extension on CRTP port 7, channel 0 (PROTOCOL.md section 5).
 *
 * <p>The ESP32 firmware answers with {@code <0x70><cmd><status>} where status 0 is OK. Every
 * command is idempotent on the drone side, which matters because {@link
 * se.bitcraze.crazyflie.lib.crazyflie.Crazyflie} retransmits unacknowledged packets every 500 ms.
 */
public class FlightControlPacket extends CrtpPacket {

    /** Port 7, channel 0 — the firmware's {@code FC_HDR}. */
    public static final int CHANNEL = 0;

    public static final int CMD_SET_MODE = 0x01;
    public static final int CMD_TAKEOFF = 0x02;
    public static final int CMD_LAND = 0x03;
    public static final int CMD_SET_HOVER = 0x04;
    public static final int CMD_EMERGENCY = 0x05;
    public static final int CMD_SET_HEADING = 0x06;
    public static final int CMD_ARM = 0x07;
    public static final int CMD_DISARM = 0x08;
    public static final int CMD_SAVE = 0x09;
    public static final int CMD_DEFAULTS = 0x0A;
    public static final int CMD_LOAD = 0x0B;
    public static final int CMD_LINK = 0x10;

    public static final int MODE_MANUAL = 0;
    public static final int MODE_HOLD = 1;
    public static final int MODE_AUTO = 2;
    public static final int MODE_FOLLOW = 3;
    public static final int MODE_RTH = 4;

    /** Hover thrust limits, matching the firmware's HOVER_THRUST_MIN/MAX. */
    public static final int HOVER_THRUST_MIN = 8000;
    public static final int HOVER_THRUST_MAX = 55000;

    private final int mCmd;
    private final byte[] mArgs;

    private FlightControlPacket(int cmd, byte[] args) {
        super(CHANNEL, CrtpPort.COMMANDER_GENERIC);
        this.mCmd = cmd;
        this.mArgs = args;
        // The reply payload starts with the command byte, so that alone is enough to retire the
        // packet from the resend queue regardless of the status byte.
        setExpectedReply(new byte[] {(byte) cmd});
    }

    public static FlightControlPacket setMode(int mode) {
        return new FlightControlPacket(CMD_SET_MODE, new byte[] {(byte) mode});
    }

    public static FlightControlPacket takeoff(int hoverThrust) {
        return new FlightControlPacket(CMD_TAKEOFF, uint16(hoverThrust));
    }

    public static FlightControlPacket land() {
        return new FlightControlPacket(CMD_LAND, new byte[0]);
    }

    public static FlightControlPacket setHover(int hoverThrust) {
        return new FlightControlPacket(CMD_SET_HOVER, uint16(hoverThrust));
    }

    public static FlightControlPacket emergency() {
        return new FlightControlPacket(CMD_EMERGENCY, new byte[0]);
    }

    public static FlightControlPacket setHeading(float degrees) {
        ByteBuffer buffer = ByteBuffer.allocate(4).order(BYTE_ORDER);
        buffer.putFloat(degrees);
        return new FlightControlPacket(CMD_SET_HEADING, buffer.array());
    }

    public static FlightControlPacket arm() {
        return new FlightControlPacket(CMD_ARM, new byte[0]);
    }

    public static FlightControlPacket disarm() {
        return new FlightControlPacket(CMD_DISARM, new byte[0]);
    }

    public static FlightControlPacket save() {
        return new FlightControlPacket(CMD_SAVE, new byte[0]);
    }

    public static FlightControlPacket defaults() {
        return new FlightControlPacket(CMD_DEFAULTS, new byte[0]);
    }

    public static FlightControlPacket load() {
        return new FlightControlPacket(CMD_LOAD, new byte[0]);
    }

    /** Selects the drone's transport profile; the board reboots about 1.5 s later. */
    public static FlightControlPacket link(int mode) {
        return new FlightControlPacket(CMD_LINK, new byte[] {(byte) mode});
    }

    private static byte[] uint16(int value) {
        return new byte[] {(byte) (value & 0xFF), (byte) ((value >> 8) & 0xFF)};
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
        return "FlightControlPacket: cmd: 0x" + Integer.toHexString(mCmd);
    }
}
