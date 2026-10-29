#!/bin/sh
# Runs the host tests against the formulas in the sketch.
#
# The .cpp files copy their math and framing verbatim out of ../Esp32_drone.ino,
# which means they would silently go stale if those lines ever changed. So
# before building anything, check the copied lines are still in the sketch and
# fail loudly if not.
set -e

here=$(cd "$(dirname "$0")" && pwd)
sketch="$here/../Esp32_drone.ino"

if [ ! -f "$sketch" ]; then
    echo "cannot find $sketch" >&2
    exit 1
fi

# Each entry is a line the tests rely on, copied from the sketch. An optional
# second argument names another source file to grep instead — commander_test
# also copies the wire builders out of the Android / iOS apps and the web page.
check() {
    f=${2:-$sketch}
    if ! grep -qF "$1" "$f"; then
        echo "STALE TEST: '$1' is no longer in $f" >&2
        echo "            the tests copy the math and framing verbatim;" >&2
        echo "            re-sync test/*.cpp with the sources before trusting them." >&2
        exit 1
    fi
    echo "ok   $f still has: $1"
}

echo "--- checking the tests are in sync with the sketch ---"
check 'tgt_x = n * ca + e * sa;'
check 'tgt_y = n * sa - e * ca;'
check 'ax_w = fwd * cy - left * sy;'
check 'ay_w = fwd * sy + left * cy;'
check 'body_fwd   =  ux * cy + uy * sy;'
check 'body_left  = -ux * sy + uy * cy;'
check 'NAV_HEADING_SIGN'
# The log-config handler, which logcfg_test.cpp copies.
check 'if (p[i] == 0) break;'
check 'memcmp(b->var_id, ids, n)'
check 'LOG_ERR_EXISTS'

# Mode bounds. Both the CRTP handler and flightSetMode() used to validate
# against MODE_AUTO, which silently rejected FOLLOW and RTH. Grep for the
# correct bound in each so it cannot quietly regress.
echo "--- checking the flight-mode bounds ---"
bounds=$(grep -c 'p\[2\] > MODE_RTH\|mode > MODE_RTH' "$sketch")
if [ "$bounds" -lt 2 ]; then
    echo "STALE: expected 2 mode bounds against MODE_RTH, found $bounds" >&2
    echo "       SET_MODE and flightSetMode() must both allow 0..MODE_RTH," >&2
    echo "       or FOLLOW/RTH get rejected as bad requests." >&2
    exit 1
fi
echo "ok   both mode bounds allow 0..MODE_RTH"

# Self-level sign and SBUS acceptance, copied by level_test / sbus_test.
check 'roll_setpoint  = sp_roll_rate  - roll_level_adjust;'
check 'pitch_setpoint = sp_pitch_rate - pitch_level_adjust;'
check 'if (footer != 0x00 && (footer & 0x0F) != 0x04) return false;'
check 'if (f[23] & 0x08) return false;'
check 'if (f[23] & 0x04) return false;'

# The mission engine math, copied by mission_test.cpp.
echo "--- checking the mission engine ---"
check '*x = (float)(n * ca + e * sa);'
check '*y = (float)(n * sa - e * ca);'
check 'double n = (double)x * ca + (double)y * sa;'
check 'double e = (double)x * sa - (double)y * ca;'
check 'if (target == items[i].seq) return MAV_MISSION_INVALID;'
check 'uint16_t times = (uint16_t)it->p2;'
check 'jump_exec_count[mission_current]++;'
check 'mission_current = (int16_t)target;'

# The MAVLink framing and CRC, copied by mavlink_test.cpp.
echo "--- checking the MAVLink bridge ---"
check 'MAV_CRC_IDS[]'
check 'MAV_CRC_EXTRAS[]'
check 'tmp ^= (uint8_t)(tmp << 4);'
check 'return (uint16_t)((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4));'
check 'while (len > 0 && payload[len - 1] == 0) len--;'
check 'for (size_t k = 1; k < i; k++) crc = mavCrcAccumulate(out[k], crc);'
check 'crc = mavCrcAccumulate(mavCrcExtra(msgid), crc);'
check 'if ((uint16_t)(b[crc_off] | (b[crc_off + 1] << 8)) != crc) return false;'

# The WebSocket handshake and frame codec, copied by ws_test.cpp.
echo "--- checking the WebSocket codec ---"
check 's->h[0] = 0x67452301;'
check 'w[i] = wsRol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);'
check 'static const char WS_B64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";'
check 'out[k++] = WS_B64[(v >> 18) & 0x3F];'
check 'static const char WS_GUID[] = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";'
check 'if (len == 126 || len == 127) return -1;   // jumbo frames are refused'
check 'return b0 & 0x0F;'
check 'out[1] = (uint8_t)len;               // len < 126 always holds here'
check 'memcpy(ws_msg + ws_frag_len, pay, plen);'

# The web page's JS framing (WEB_PAGE). payload.length+1 once let the checksum
# overwrite the last payload byte, and the hold/killAll re-sends went out
# without the 0x0C command prefix that fc() adds — so the firmware refused
# them. Grep all four lines so none can quietly drift apart again.
echo "--- checking the web page framing ---"
check 'var a=new Uint8Array(payload.length+2);'
check 'var body=new Uint8Array([cmd].concat(args||[]));'
check 'send(7,0,new Uint8Array([FC.MOTOR_TEST].concat(motorPacket(1))));'
check 'send(7,0,new Uint8Array([FC.MOTOR_TEST].concat(motorPacket(0))));'

# The BLE segment reassembly, copied by bleseg_test.cpp, and the Bluetooth
# reply routing (SPP and BLE must claim replies the way UDP and the page do).
echo "--- checking the BLE segment reassembly ---"
check 'size_t bleRxSeg(uint8_t ctrl, const uint8_t *pay, size_t n) {'
check 'if (total <= n) {                 // single-segment packet'
check 'if (pid != ble_rx_pid || ble_rx_len == 0) { ble_rx_len = 0; return 0; }'
check 'if (ble_rx_len + n > sizeof(ble_rx_buf))   { ble_rx_len = 0; return 0; }'
check 'size_t done = bleRxSeg(buf[0], buf + 1, n - 1);'
echo "--- checking the Bluetooth reply routing ---"
check 'crtp_reply_via = 2;              // replies follow the SPP phone'
check 'crtp_reply_via = 0;                  // replies follow the BLE phone'
check '(crtp_reply_via == 2 || link_mode == LINK_MODE_BT_SPP)'
check 'case LINK_MODE_BT_DUAL: {'
check 'if (ok && (link_mode == LINK_MODE_WIFI_AP_BLE || link_mode == LINK_MODE_WIFI_STA_BLE))'

# The CommanderPacket wire and the CRTP arming sequence, copied by
# commander_test.cpp — both the firmware's parse/arm logic and the bytes the
# Android, iOS and web clients build. The fly-screen sticks looked dead because
# the arming gate (zero-thrust witness first, battery above 3.30 V) said no,
# not because these lines moved; pin both sides so neither drifts quietly.
echo "--- checking the commander wire and arming sequence ---"
check 'if (len < 15) return;'
check 'memcpy(&roll,  p + 1,  4);'
check 'memcpy(&thrust, p + 13, 2);'
check 'if (!isfinite(roll) || !isfinite(pitch) || !isfinite(yaw)) return;'
check 'if (fabsf(roll) > 180.0f || fabsf(pitch) > 180.0f || fabsf(yaw) > 2000.0f) return;'
check 'if (cmd_thrust <= 500) crtp_thrust_was_low = true;'
check 'if (cmd_thrust > 1000 && crtp_thrust_was_low && armChecksOk("CRTP thrust-up")) {'
check 'if (flight_mode != MODE_MANUAL) return;'
check 'if (!cmd_ok) crtp_thrust_was_low = false;'
check '#define VBAT_CELL_CRIT    3.30f'
check '#define VBAT_ARM_CHECK    1'
apps="$here/../.."
check 'var b=new Uint8Array(14);'
check 'b[12]=t&255;b[13]=(t>>8)&255;'
check 'streamHold=25;'
check 'buffer.putFloat(-mPitch); //invert axis' \
      "$apps/ESP-Drone-Android/src/se/bitcraze/crazyflie/lib/crtp/CommanderPacket.java"
check 'protected static final int MAX_THRUST = 65000;' \
      "$apps/ESP-Drone-Android/src/se/bitcraze/crazyfliecontrol/controller/AbstractController.java"
check 'return mControls.getMinThrust() + (thrust * mControls.getThrustFactor());' \
      "$apps/ESP-Drone-Android/src/se/bitcraze/crazyfliecontrol/controller/AbstractController.java"
check 'return thrust / 100 * MAX_THRUST;' \
      "$apps/ESP-Drone-Android/src/se/bitcraze/crazyfliecontrol/controller/AbstractController.java"
check 'preferences_deadzone_defaultValue">0.2' "$apps/ESP-Drone-Android/res/values/strings.xml"
check 'preferences_minThrust_defaultValue">25' "$apps/ESP-Drone-Android/res/values/strings.xml"
check 'preferences_maxThrust_defaultValue">90' "$apps/ESP-Drone-Android/res/values/strings.xml"
check 'thrust = control * 65535 * (maxThrust/100)' \
      "$apps/ESP-Drone-iOS/Crazyflie client/CrazyFlieCommandHelper.swift"

# The nav accelerometer scale must match the range mpuInit() configures.
# It once said +-2 g (16384) while the sensor ran at +-8 g -- a 4x error.
echo "--- checking accelerometer scale ---"
cfg=$(sed -n 's/.*mpuWrite(MPU_REG_ACCEL_CFG,[[:space:]]*\(0x[0-9A-Fa-f]*\)).*/\1/p' "$sketch" | head -1)
lsb=$(sed -n 's/^#define[[:space:]]*NAV_ACC_LSB_PER_G[[:space:]]*\([0-9][0-9]*\).*/\1/p' "$sketch" | head -1)
case "$cfg" in
    0x00) want=16384 ;; 0x08) want=8192 ;; 0x10) want=4096 ;; 0x18) want=2048 ;;
    *) echo "cannot read MPU_REG_ACCEL_CFG ($cfg)" >&2; exit 1 ;;
esac
if [ "$lsb" != "$want" ]; then
    echo "STALE: ACCEL_CFG $cfg needs NAV_ACC_LSB_PER_G $want, sketch has $lsb" >&2
    exit 1
fi
echo "ok   ACCEL_CFG $cfg matches NAV_ACC_LSB_PER_G $lsb"

echo
echo "--- building ---"
cxx=${CXX:-c++}
out=${TMPDIR:-/tmp}/esp_drone_frame_tests
mkdir -p "$out"

# The yaw test has to be built with the sign the sketch is actually using,
# otherwise it would happily validate a configuration the firmware does not run.
sign=$(sed -n 's/^#define[[:space:]]*NAV_HEADING_SIGN[[:space:]]*\(-\{0,1\}[0-9][0-9]*\).*/\1/p' "$sketch" | head -1)
if [ -z "$sign" ]; then
    echo "could not read NAV_HEADING_SIGN from Esp32_drone.ino" >&2
    exit 1
fi
echo "ok   NAV_HEADING_SIGN = $sign"

"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/frame_test"  "$here/frame_test.cpp"  -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -DHEADING_SIGN="$sign" \
       -o "$out/yawrot_test" "$here/yawrot_test.cpp" -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/logcfg_test" "$here/logcfg_test.cpp" -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/level_test"  "$here/level_test.cpp"  -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/sbus_test"   "$here/sbus_test.cpp"   -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/mission_test" "$here/mission_test.cpp" -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/mavlink_test" "$here/mavlink_test.cpp" -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/ws_test"     "$here/ws_test.cpp"     -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/bleseg_test" "$here/bleseg_test.cpp" -lm
"$cxx" -std=c++17 -Wall -Wextra -O0 -o "$out/commander_test" "$here/commander_test.cpp" -lm

echo
echo "--- frame_test: does the target rotation match the dead reckoning? ---"
"$out/frame_test"
echo
echo "--- yawrot_test: does the body/world rotation run the right way? ---"
"$out/yawrot_test"
echo
echo "--- logcfg_test: log-config parsing and retransmission ---"
"$out/logcfg_test"
echo
echo "--- level_test: does self-level pull back toward the target? ---"
"$out/level_test"
echo
echo "--- sbus_test: are failsafe / lost frames rejected? ---"
"$out/sbus_test"
echo
echo "--- mission_test: conversion, DO_JUMP accounting, jump chains ---"
"$out/mission_test"
echo
echo "--- mavlink_test: framing, crc_extra, golden frames ---"
"$out/mavlink_test"
echo
echo "--- ws_test: handshake accept key, frame codec ---"
"$out/ws_test"
echo
echo "--- bleseg_test: BLE control-byte segment reassembly ---"
"$out/bleseg_test"
echo
echo "--- commander_test: CommanderPacket wire and the CRTP arming sequence ---"
"$out/commander_test"
