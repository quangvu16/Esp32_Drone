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
    public static Crazyflie get() {
        return sCrazyflie;
    }
}
