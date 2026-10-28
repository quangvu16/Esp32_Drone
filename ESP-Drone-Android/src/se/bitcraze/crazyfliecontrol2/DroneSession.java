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

package se.bitcraze.crazyfliecontrol2;

import se.bitcraze.crazyflie.lib.crazyflie.Crazyflie;

/**
 * Hands the live {@link Crazyflie} to the tuning and GPS screens.
 *
 * <p>Those screens are separate activities but must share the one radio link, so a
 * static holder is the pragmatic fit: the alternative is binding a service just to
 * pass a reference the app already owns. Cleared on disconnect so a stale activity
 * cannot talk to a drone that has gone away.
 * Chuyển {@link Crazyflie} đang chạy cho màn hình tune và GPS.
 *
 * <p>Các màn hình đó là những activity riêng nhưng phải dùng chung một link radio,
 * nên một holder tĩnh là cách thực dụng: giải pháp thay thế là bind một service
 * chỉ để truyền một tham chiếu mà app đã sở hữu. Được xoá khi ngắt kết nối để một
 * activity cũ không thể nói chuyện với drone đã biến mất.
 */
public final class DroneSession {

    private static volatile Crazyflie sCrazyflie;

    private DroneSession() {
    }

    public static void set(Crazyflie crazyflie) {
        sCrazyflie = crazyflie;
    }

    public static void clear() {
        sCrazyflie = null;
    }

    /** The connected Crazyflie, or null when nothing is linked. */
    /** Crazyflie đang kết nối, hoặc null khi chưa có liên kết nào. */
    public static Crazyflie get() {
        return sCrazyflie;
    }

    // ---- Latest flight status, for the shared screen header (redesign stage 3) ----
    // ---- Trạng thái bay mới nhất, dùng cho header màn hình dùng chung (redesign stage 3) ----
    //
    // MainPresenter owns the one "Standard" log block and writes here; the Mission and
    // Tune headers read it, so every screen shows the same battery / mode / ARMED
    // without each one spending a block of the firmware's three.
    // MainPresenter sở hữu block log "Standard" duy nhất và ghi vào đây; header Mission
    // và Tune đọc nó, nên mọi màn hình đều hiển thị cùng pin / mode / ARMED
    // mà không màn hình nào tốn một block trong ba block của firmware.

    private static volatile float sVbat = -1f;
    private static volatile int sMode = -1;
    private static volatile boolean sArmed;
    private static volatile String sLinkQuality;

    public static void setStatus(float vbat, int mode, boolean armed) {
        sVbat = vbat;
        sMode = mode;
        sArmed = armed;
    }

    public static void setBattery(float vbat) {
        sVbat = vbat;
    }

    public static void setLinkQuality(String quality) {
        sLinkQuality = quality;
    }

    public static void clearStatus() {
        sVbat = -1f;
        sMode = -1;
        sArmed = false;
        sLinkQuality = null;
    }

    /** Battery volts, or a negative number when unknown. */
    /** Điện áp pin (vôn), hoặc số âm khi chưa biết. */
    public static float vbat() {
        return sVbat;
    }

    /** Flight mode 0..4, or -1 when no status has arrived yet. */
    /** Chế độ bay 0..4, hoặc -1 khi chưa nhận được trạng thái nào. */
    public static int mode() {
        return sMode;
    }

    public static boolean armed() {
        return sArmed;
    }

    /** Link quality text such as "92%", or null. */
    /** Chất lượng link dạng chuỗi như "92%", hoặc null. */
    public static String linkQuality() {
        return sLinkQuality;
    }
}
