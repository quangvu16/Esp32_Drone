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

# Each entry is a line the tests rely on, copied from the sketch.
check() {
    if ! grep -qF "$1" "$sketch"; then
        echo "STALE TEST: '$1' is no longer in Esp32_drone.ino" >&2
        echo "            the tests copy the math and framing verbatim;" >&2
        echo "            re-sync test/*.cpp with the sketch before trusting them." >&2
        exit 1
    fi
    echo "ok   sketch still has: $1"
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
