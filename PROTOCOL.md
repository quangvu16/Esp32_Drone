# ESP32 Drone — CRTP Protocol Reference

Everything the phone apps need to talk to `Esp32_drone/Esp32_drone.ino`.
Byte layouts are pinned to the Bitcraze "crazyflie" Java library that
`ESP-Drone-Android` is built from; where this firmware extends CRTP it says so.

---

## 1. Framing

Every transport carries complete CRTP packets.

| Transport | Wire bytes |
|---|---|
| UDP | `[CRTP ...][uint8 checksum]` |
| Bluetooth Classic SPP | `[CRTP ...][uint8 checksum]` |
| BLE GATT | raw CRTP, wrapped in control-byte segments (§6) |

`checksum` = low byte of the sum of **all** CRTP bytes including the header.

### CRTP header

```
bits 7..4  port
bits 3..2  reserved
bits 1..0  channel
```

| Port | Name | Direction | Purpose |
|---|---|---|---|
| 0 | CONSOLE | drone→phone | human-readable text, UTF-8 |
| 2 | PARAMS | both | TOC + read/write parameters |
| 3 | COMMANDER | phone→drone | flight setpoints |
| 5 | LOGGING | both | TOC + streaming telemetry |
| 7 | GENERIC | both | **extension**: flight control (ch 0) and navigation (ch 1) |

**Ports the firmware ignores:** 4 (MEMORY), 15 (LINK).

---

## 2. TOC protocol (ports 2 and 5, channel 0)

Both the parameter and log TOCs use the same request/reply pair.

```
CMD_TOC_ELEMENT = 0
CMD_TOC_INFO    = 1

info request    : <1>
info reply      : <1><uint8 nItems><uint32 crc LE>
element request : <0><uint8 id>
element reply   : <0><uint8 id><type|access><group\0><name\0>
```

`type|access`: bit 6 (0x40) set = read-only, clear = read/write.
Low nibble = type code.

Element ids are 0-based and dense. A request for a missing id is not answered.

### Parameter type codes (port 2)

| Code | Type | Code | Type |
|---|---|---|---|
| 0x00 | INT8 | 0x08 | UINT8 |
| 0x01 | INT16 | 0x09 | UINT16 |
| 0x02 | INT32 | 0x0A | UINT32 |
| 0x03 | INT64 | 0x0B | UINT64 |
| 0x06 | FLOAT | 0x07 | DOUBLE |

### Log type codes (port 5) — **different numbering**

| Code | Type | Code | Type |
|---|---|---|---|
| 0x01 | UINT8 | 0x05 | INT16 |
| 0x02 | UINT16 | 0x06 | INT32 |
| 0x03 | UINT32 | 0x07 | FLOAT |
| 0x04 | INT8 | | |

---

## 3. Parameters (port 2)

```
CHAN_TOC   = 0    (above)
CHAN_READ  = 1
CHAN_WRITE = 2

read  request : <uint8 id>            reply: <id><value LE>
write request : <id><value LE>        reply: <id><value LE>
```

Values are little-endian, exactly `size` bytes as given by the TOC element.

### Parameter TOC

| id | group | name | type | access |
|---|---|---|---|---|
| 0 | cpu | flash | UINT32 | RO |
| 1 | deck | bcZRanger | UINT8 | RO |
| 2 | ring | neffect | UINT8 | RO |
| 3 | pid | roll_kp | FLOAT | RW |
| 4 | pid | roll_ki | FLOAT | RW |
| 5 | pid | roll_kd | FLOAT | RW |
| 6 | pid | pitch_kp | FLOAT | RW |
| 7 | pid | pitch_ki | FLOAT | RW |
| 8 | pid | pitch_kd | FLOAT | RW |
| 9 | pid | yaw_kp | FLOAT | RW |
| 10 | pid | yaw_ki | FLOAT | RW |
| 11 | pid | yaw_kd | FLOAT | RW |
| 12 | flight | hover_thrust | UINT16 | RW |
| 13 | flight | takeoff_ms | UINT16 | RW |
| 14 | flight | land_ms | UINT16 | RW |
| 15 | nav | kp | FLOAT | RW |

`cpu.flash`, `deck.bcZRanger` and `ring.neffect` report 0 — the board has no
buzzer deck, no zRanger and no LED ring. They exist so the stock app's matching
buttons disable themselves instead of failing.

**Idempotency:** the client retransmits unacknowledged writes every 500 ms.
A repeated write re-applies and re-acks the same value, so it is safe.

---

## 4. Logging (port 5)

```
CHAN_TOC     = 0   (above)
CHAN_SETTINGS= 1
CHAN_LOGDATA = 2

control request : <cmd><id><args...>     reply: <cmd><id><err>
data packet     : <id><uint8 ts0><ts1><ts2><values...>
```

The timestamp is 24-bit little-endian, `d1 | d2<<8 | d3<<16`, in milliseconds
since the block started.

Control commands:

| cmd | Name | args |
|---|---|---|
| 0 | CREATE | `<id><type><tocId>...` |
| 1 | APPEND | `<id><type><tocId>...` |
| 2 | DELETE | `<id>` |
| 3 | START | `<id><period>` |
| 4 | STOP | `<id>` |
| 5 | RESET | — |

`period` is in **10 ms units** (the client divides its ms by 10 before sending).

Error codes (surfaced to the user by `Logg.java`):

| err | Meaning | err | Meaning |
|---|---|---|---|
| 0 | OK | 8 | command not found |
| 2 | no such id | 12 | no memory |
| 7 | too large | 17 | already exists |

**Wire-format trap:** `Logg.create()` builds its request in a fixed 31-byte
`ByteBuffer` and sends the whole backing array (`CrtpPacket.toByteArray()`
serialises untrimmed). So real `<type><tocId>` pairs are followed by zero
padding. Log type codes start at 1, so a zero type byte is unambiguous padding
and must terminate the list.

**Idempotency:** CREATE on an existing identical block re-acks OK; DELETE/STOP
on a missing block re-acks OK. Do not report "already exists" / "no such id"
for a retransmission.

### Log TOC

| id | group | name | type | size |
|---|---|---|---|---|
| 0 | pm | vbat | FLOAT | 4 |
| 1 | stabilizer | roll | FLOAT | 4 |
| 2 | stabilizer | pitch | FLOAT | 4 |
| 3 | stabilizer | yaw | FLOAT | 4 |
| 4 | controller | roll | FLOAT | 4 |
| 5 | controller | pitch | FLOAT | 4 |
| 6 | controller | yaw | FLOAT | 4 |
| 7 | motor | m1 | UINT16 | 2 |
| 8 | motor | m2 | UINT16 | 2 |
| 9 | motor | m3 | UINT16 | 2 |
| 10 | motor | m4 | UINT16 | 2 |
| 11 | flight | mode | UINT8 | 1 |
| 12 | flight | auto_state | UINT8 | 1 |
| 13 | flight | armed | UINT8 | 1 |
| 14 | flight | heading | FLOAT | 4 |
| 15 | nav | pos_x | FLOAT | 4 |
| 16 | nav | pos_y | FLOAT | 4 |
| 17 | nav | vel_x | FLOAT | 4 |
| 18 | nav | vel_y | FLOAT | 4 |
| 19 | nav | tgt_x | FLOAT | 4 |
| 20 | nav | tgt_y | FLOAT | 4 |
| 21 | pid | roll_p | FLOAT | 4 |
| 22 | pid | roll_i | FLOAT | 4 |
| 23 | pid | roll_d | FLOAT | 4 |
| 24 | pid | pitch_p | FLOAT | 4 |
| 25 | pid | pitch_i | FLOAT | 4 |
| 26 | pid | pitch_d | FLOAT | 4 |
| 27 | pid | yaw_p | FLOAT | 4 |
| 28 | pid | yaw_i | FLOAT | 4 |
| 29 | pid | yaw_d | FLOAT | 4 |
| 30 | gyro | roll | FLOAT | 4 |
| 31 | gyro | pitch | FLOAT | 4 |
| 32 | gyro | yaw | FLOAT | 4 |
| 33 | acc | x | FLOAT | 4 |
| 34 | acc | y | FLOAT | 4 |
| 35 | acc | z | FLOAT | 4 |
| 36 | acc | roll | FLOAT | 4 |
| 37 | acc | pitch | FLOAT | 4 |

`gyro.*` are body rates in deg/s; `acc.x/y/z` are in g; `acc.roll/pitch` are
the accelerometer-derived tilt angles in deg, i.e. the raw input to the
complementary filter **before** it is blended with the gyro — a vibration
spike shows up here long before it moves `stabilizer.roll`.

Budget: `<header><id><ts24><values...>` must fit in 30 CRTP bytes, so a data
packet holds at most **25 bytes of values**. Example: 5 FLOAT (20) + 2 UINT16
(4) = 24 fits; all 11 flight/motor variables (36) does not.

Suggested blocks:

- **Status** (25 B): `pm.vbat`, `flight.mode`, `flight.armed`, `flight.auto_state`,
  `stabilizer.roll`, `stabilizer.pitch` → 4+1+1+1+4+4 = 15 B
- **Tune roll**: `pid.roll_p/i/d`, `stabilizer.roll`, `controller.roll`, `motor.m1`
  → 4+4+4+4+4+2 = 22 B
- **Tune pitch**: same with the pitch names
- **Tune yaw**: `pid.yaw_p/i/d`, `stabilizer.yaw`, `controller.yaw`
- **Nav**: `nav.pos_x/pos_y`, `nav.tgt_x/tgt_y`, `flight.heading`, `flight.mode`
  → 4+4+4+4+4+1 = 21 B
- **Bench IMU** (24 B): `gyro.roll`, `gyro.pitch`, `gyro.yaw`, `acc.x`,
  `acc.y`, `acc.z` → 6×4 = 24 B — for spotting motor vibration on the ground

---

## 5. Flight control (port 7, channel 0) — **firmware extension**

```
request : <0x70><cmd><args...>
reply   : <0x70><cmd><status uint8>      status: 0 = OK, 1 = bad request
```

| cmd | Name | args | Effect |
|---|---|---|---|
| 0x01 | SET_MODE | `uint8 mode` | 0 manual, 1 hold, 2 auto, 3 follow, 4 rth |
| 0x02 | TAKEOFF | `uint16 thrust` | arm, start AUTO takeoff ramp to this hover thrust |
| 0x03 | LAND | — | AUTO land ramp, then disarm |
| 0x04 | SET_HOVER | `uint16 thrust` | change hover thrust (8000..55000) |
| 0x05 | EMERGENCY | — | motors off + disarm now, **always honoured** |
| 0x06 | SET_HEADING | `float deg` | heading to lock in hold/auto |
| 0x07 | ARM | — | arm (checked — see **Arming** below) |
| 0x08 | DISARM | — | disarm + motors off |
| 0x09 | SAVE | — | persist gains + flight config to NVS |
| 0x0A | DEFAULTS | — | restore default gains (not saved) |
| 0x0B | LOAD | — | reload gains + flight config from NVS |
| 0x0C | MOTOR_TEST | `uint8 enable` + `uint16 m1..m4` | bench motor test — see below |
| 0x10 | LINK | `uint8 mode` | store transport profile, reboot in ~1.5 s |

`uint16` args are little-endian. `float` args are IEEE-754 little-endian.

### Bench motor test (MOTOR_TEST, 0x0C)

Drives m1..m4 directly with the PID loop bypassed — for checking motor
direction, vibration and prop balance on the bench. Payload:

```
<cmd=0x0C><enable uint8><m1 uint16><m2 uint16><m3 uint16><m4 uint16>
```

Thrust values are µs, clamped to 1000..2000. The client must re-send the full
packet at least every **300 ms** while the test runs: the firmware treats the
gap as link loss and stops the motors. The test also stops immediately on
`enable=0`, EMERGENCY, DISARM, or if the drone arms. Status 1 is returned for
a payload shorter than the `enable` byte, for `enable=1` with a payload
shorter than the four thrusts, or while armed.

Safety: props off or restrained; this path never goes through the mixer or
the arming checks beyond "must be disarmed".

### Modes

| Value | Name | Behaviour |
|---|---|---|
| 0 | MANUAL | pilot commands tilt angles / yaw rate / thrust directly |
| 1 | HOLD | attitude level, heading locked at entry, thrust = hover_thrust |
| 2 | AUTO | takeoff ramp → hold → land ramp → disarm |
| 3 | FOLLOW | tilt toward the phone's GPS-derived target offset |
| 4 | RTH | like FOLLOW but the target is the launch point |

`flight.auto_state` (log id 12): 0 idle, 1 takeoff, 2 hold, 3 land.

**Arming:** in MANUAL the drone auto-arms when thrust rises above 1000 *after*
thrust has been seen at ≤ 500 since the CommanderPacket stream started (a phone
that connects with its throttle already up does not spin the props), and
auto-disarms on thrust ≤ 500. In HOLD/AUTO/FOLLOW/RTH arming is explicit via
TAKEOFF / ARM / DISARM / LAND / EMERGENCY.

Arming checks (every path — SBUS, thrust-up, ARM, TAKEOFF): the filtered
battery must be at or above 3.30 V per cell. `ARM` is also refused (status 1)
unless the motors would start from idle: MANUAL with thrust ≤ 500, or AUTO
before a takeoff. `TAKEOFF` while already climbing or hovering in AUTO is
acked as a no-op instead of restarting the ramp; while flying in any other
mode it is refused. `SAVE` and `LINK` are refused while armed.

**Failsafe:** while armed, motors cut, the drone disarms and the mode returns
to MANUAL when, for 500 ms:
- MANUAL: no **CommanderPacket** arrives and there is no valid SBUS — the
  commander stream is the setpoint, so other traffic does not count;
- HOLD/AUTO/FOLLOW/RTH: no **CRTP packet of any kind** arrives and there is no
  valid SBUS. Keep resending an idempotent command (SET_MODE, LAND) at least
  every 500 ms.

SBUS frames flagged *failsafe* or *frame lost* by the receiver (byte 23) are
ignored, so a receiver in failsafe also runs this timeout out.

**Low battery:** below 3.30 V per cell for 3 s, HOLD/AUTO/FOLLOW/RTH switch to
the AUTO land ramp and refuse SET_MODE until disarmed. In MANUAL and on SBUS
the pilot gets a console warning every 2 s instead.

### Safety note for the UI

This board has **only an MPU6500** — no barometer, no ToF, no GPS, no
magnetometer. So:

- HOLD keeps a **thrust level**, not an altitude. Hover height drifts as the
  battery sags. Say so; do not draw an altitude hold.
- AUTO is a **takeoff → hover → land** sequence, not waypoint flight.
- FOLLOW / RTH use accelerometer dead reckoning. It drifts — metres within a
  minute. Label it approximate.

---

## 6. Navigation (port 7, channel 1) — **firmware extension**

The phone reads its **own** GPS and compass and sends the target as a
north/east offset from the launch point. The drone cannot measure its own
position; it estimates it by integrating its accelerometer.

```
request : <0x71><cmd><args...>
reply   : <0x71><cmd><status uint8>
```

| cmd | Name | args | Effect |
|---|---|---|---|
| 0x20 | TARGET | `float north_m`, `float east_m` | target offset from launch |
| 0x21 | RESET | — | zero the dead-reckoned position |
| 0x22 | ALIGN | `float alpha_deg` | compass bearing of the nose at launch; also zeroes the position and starts a new launch frame |

Send `ALIGN` once when the pilot arms/launches. The drone rotates north/east
offsets into its own frame using that angle, which is the one frame
relationship it cannot measure itself (no magnetometer).

**A target is honoured for 3 s** after the last TARGET packet; after that the
drone stops chasing and holds. Keep sending TARGET while in FOLLOW.

In RTH the firmware forces the target to the launch point `(0, 0)` every tick,
so no TARGET packet is needed — but it is harmless to send one.

### The launch frame

`pos_x`, `pos_y`, `tgt_x`, `tgt_y` all live in one frame, defined at `ALIGN`:

| Axis | Meaning |
|---|---|
| `+x` | where the nose pointed at `ALIGN` — compass bearing `alpha` |
| `+y` | 90° **left** of that — compass bearing `alpha − 90°` |

So a north/east offset `(n, e)` becomes:

```
x = n·cos(alpha) + e·sin(alpha)
y = n·sin(alpha) − e·cos(alpha)
```

The drone's reported position is in exactly this frame (metres), so a phone can
compare it against a waypoint's offset directly and skip the lat/lon round trip.

Three sign constants in the firmware control this frame. Two are settled from
the code and pinned by `Esp32_drone/test/`; one still needs hardware.

| Constant | What it fixes | State |
|---|---|---|
| `NAV_HEADING_SIGN` | which way `heading_deg` turns, i.e. the body↔launch rotation | **−1, derived.** `heading_deg` is a compass bearing (wraps 0..360, and `hold_heading_deg` is assigned straight from the phone's compass `SET_HEADING`). The rotation needs the opposite sense because `+y` is the drone's left. |
| `NAV_FWD_SIGN` | which accelerometer axis is nose-forward | **needs a flight** |
| `NAV_LEFT_SIGN` | which accelerometer axis is to the left | **needs a flight** |

`Esp32_drone/test/run.sh` checks all of it: the target rotation and the dead
reckoning must agree on where a point is, and a target dead along the nose must
produce pure pitch at any yaw. If either fails it names the constant to flip.

What the two remaining ones affect: only the **dead reckoning**, i.e. whether
`pos_*` lands in the frame above. The target rotation is fixed by the geometry
of the compass and does not depend on them. So expect to flip one of the two
after the first flight, and until then treat the drone's reported position as
approximate — which the map UI should say out loud anyway.

---

## 7. Commander packet (port 3, channel 0)

```
[0] 0x30
[1..4]   float roll   (deg, attitude target)
[5..8]   float pitch  (deg; the clients send the NEGATIVE of the UI pitch)
[9..12]  float yaw    (deg/s, rate)
[13..14] uint16 thrust (0..65535)
```

15 CRTP bytes + 1 checksum = 16 on UDP/SPP.

A packet is **dropped entirely** — the drone keeps its previous command — if any
of the three floats is NaN or infinite, if `|roll|` or `|pitch|` exceeds 180°,
or if `|yaw|` exceeds 2000°/s. Values that survive are then clamped to
`|roll|,|pitch| ≤ 25°` and `|yaw rate| ≤ 200°/s`.

That distinction matters when mapping a phone IMU: 300°/s is clamped to 200°/s,
but 2500°/s vanishes silently. Keep the tilt-to-fly mapping well inside ±180°.

Sent continuously (Android 50 Hz, iOS 20 Hz). A 500 ms gap trips the failsafe,
**including while in HOLD/AUTO/FOLLOW/RTH** — keep sending it.

### Tilt-to-fly from the phone IMU

No firmware change is needed. Map the phone's rotation into the same
roll/pitch/yaw fields and send this packet. See §9.

---

## 8. Transports

Selected at boot from NVS key `link` (0..5). Change with `LINK` (§5) or the
serial console `link <n>`; the board reboots to apply it.

| Value | Name | Notes |
|---|---|---|
| 0 | WIFI_AP | default. Drone hosts `ESP-Drone` / `12345678` at **192.168.43.42** |
| 1 | WIFI_STA | drone joins a saved network, mDNS **esp-drone.local** |
| 2 | BT_SPP | Bluetooth Classic serial. No WiFi — they share the radio |
| 3 | BLE | GATT, see below |
| 4 | WIFI_AP_BLE | softAP + BLE together |
| 5 | WIFI_STA_BLE | station + BLE together |

UDP ports: phone→drone **2390**, drone→phone **2399** (the phone binds 2399).

Station credentials: serial `sta <ssid> <pass>`, stored in NVS.

### BLE GATT

Matches the Bitcraze service the stock Android `BleLink` speaks.

```
00000201-1C7F-4F9E-947B-43B7C00A9A08   service
  00000202  CRTP      write   phone → drone, raw CRTP when ≤ 20 bytes
  00000203  CRTPUP    write   phone → drone, control-byte segmented
  00000204  CRTPDOWN  notify  drone → phone, always control-byte framed
```

Control byte:

```
bit7     start (1 = first segment of a datagram)
bits6..5 pid   (0..3, wrapping, must match across segments)
bits4..0 total CRTP length − 1
```

A datagram of N CRTP bytes goes out in `ceil(N/19)` segments of 20 bytes:

```
[ctrl start, len=N-1]  crtp[0 .. 18]
[ctrl cont , pid    ]  crtp[19 .. 37]   ... and so on
```

A datagram is single-segment when its announced length **fits in that
segment** — do not decide this from the length field alone, since a 20-byte
datagram cannot fit in a 20-byte write alongside the control byte.

Note the asymmetry the stock client imposes: it sends short packets raw on
`00000202` but expects **every** drone→phone notification to be control-byte
framed.

---

## 9. Phone sensors

The apps read the phone's own sensors (no SensorServer) and convert them into
CRTP. Two things are consumed:

**IMU → tilt-to-fly.** Map the phone's rotation vector to roll/pitch/yaw and
send a normal CommanderPacket. Suggested mapping (landscape, phone flat):

- roll  = phone roll  (device tilted right → drone rolls right)
- pitch = phone pitch (device tilted away → drone pitches forward)
- yaw   = phone yaw rate about the vertical, scaled to deg/s

With a deadzone and a max-tilt scale, so a flat phone means a level drone.

**GPS → navigation target.** On arming, capture the phone's position as the
*launch point* and its compass heading as `ALIGN`. From then on the phone
computes where the drone should go and sends it as a north/east offset from the
launch point. Send `TARGET` at ~5 Hz while FOLLOW/RTH/mission is active.

Use great-circle math, not a flat-earth approximation — it costs nothing and
stays right at any range:

```
R        = 6371008.8 m
distance = 2*R*asin( sqrt( sin²(dLat/2) + cos(lat1)cos(lat2)sin²(dLon/2) ) )
bearing  = atan2( sin(dLon)cos(lat2),
                  cos(lat1)sin(lat2) − sin(lat1)cos(lat2)cos(dLon) )

north_m  = distance * cos(radians(bearing))
east_m   = distance * sin(radians(bearing))
```

`bearing` is degrees clockwise from true north, wrapped to `[-180, 180)`.
Reverse it to turn the drone's reported position back into a lat/lon.

---

## 10. GPS mission navigation (phone side)

The **phone** runs the mission; the **drone** chases one offset at a time.
That split matters: the drone has no GPS and no compass, so it cannot evaluate a
mission, but it does not need to. Everything below is a phone-side state machine
that ends in a `TARGET` packet.

Model it on `quangvu16/Ackermann_robot_openbot`'s `gpsnav` module
(`robot/linux/gpsnav/`, mirrored by `robot/android/.../gpsNav/` and
`robot/ios/.../GpsNav/GpsNavCore.swift`). `GpsNavCore.swift` is pure Foundation
and `gpsnav/*.py` is pure Python — both port directly. **Take the geo math, the
mission model and the waypoint sequencing. Do not take `guidance.py` /
`GuidanceController`**: that turns a bearing error into differential-drive
wheel commands. The drone already has its own tilt controller (`navSetpoints`,
§6) and only needs to be told where to go.

### Data model

```
Waypoint(lat, lon, name, acceptance_radius_m)     default radius 2.5 m
Mission(home_lat, home_lon, waypoints[])
```

Support add / remove / reorder, total along-track distance, and the Mission
Planner **`.waypoints`** interchange (`QGC WPL 110`): index 0 is home,
subsequent `NAV_WAYPOINT` (cmd 16) rows are waypoints, non-navigation commands
skipped on read. Writing is one tab-separated row per entry. This lets a flight
be planned on a desktop and loaded on the phone.

### Modes

| Phone mode | Drone mode | Target |
|---|---|---|
| `MISSION` | FOLLOW (0x03) | the active waypoint |
| `GUIDED` | FOLLOW (0x03) | one tapped "go to here" point |
| `RTL` | RTH (0x04) | the home position |

Arriving at a waypoint advances to the next; arriving at the last ends the
mission. `GUIDED` and `RTL` are one-shot — arrival disarms them. Hold a pause
flag so the drone stops being sent a new target (it will hold after 3 s).

### Whose position is "current"?

**The drone's**, not the phone's. The phone is the pilot's handset; if the pilot
walks away, a phone-position arrival check would fire while the drone is
nowhere near the waypoint.

The drone reports its dead-reckoned position as `nav.pos_x` / `nav.pos_y`
(log ids 15/16) in the launch frame — metres along and 90° left of where the
nose pointed at `ALIGN`. Convert that back to a lat/lon offset to run the
arrival check, or equivalently compare `nav.pos_*` against the waypoint's
`north_m`/`east_m` directly and skip the round trip. Use the **drone's** numbers
for arrival and progress; use the **phone's** GPS only to define the plan and
the launch frame.

Note the asymmetry this creates in FOLLOW: there the target *is* the phone, so
the phone's GPS is the source of the target and the drone's estimate is still
the source of progress.

### Arrival

Inside `acceptance_radius_m` counts as arrived. Ignore fixes with horizontal
accuracy worse than ~15 m and say so ("GPS accuracy 8.2 m, holding") rather
than navigating on a bad fix.

### Map UI

Draw the plan (home + waypoints + legs), the drone's estimated track, the
active waypoint, and live distance/bearing to it. `MissionMapView.java`
(osmdroid 6.1.20) and `MapTileSources.java` are a working reference. Label the
drone marker **estimated** — it is dead reckoning, not GPS, and it drifts
metres per minute.

### Map annotations (Valetudo port)

Ported from `hypfer/valetudo` 2026.07.0's map annotations (`ValetudoMapAnnotation`):
*things on the map the robot needs to be aware of that might not be reliably
auto-determined*. Valetudo makes its **firmware** behave differently at those
locations; this project has no smart firmware, so the slot is played by the
**phone-side mission runner** above — same idea, same placement, different
executor.

```
MapAnnotation(type, points[])     at least 2 points (Valetudo's rule, kept)
  type 0 KEEPOUT   4-point quad — area the drone must not fly over
  type 1 CORRIDOR  2-point line — the sanctioned gate/gap crossing
  type 2 WIND      4-point quad — unsteady air, arrival needs confirmation
```

Quads are placed axis-aligned from two taps and carry the 4 corners in Valetudo's
ramp order (`p0` NW → `p1` SW → `p2` SE → `p3` NE, so `p0→p1` is the width edge
and `p0→p3` the height edge) — the on-map rotate handle can turn them, and the
containment tests are polygon-based so rotated quads work.

**Runner behavior** (all messages shown verbatim in the apps):

- A waypoint inside a keep-out zone is refused: "inside keep-out zone - move
  the waypoint". A leg that crosses a keep-out with no corridor across it is
  refused too: "leg crosses a keep-out zone with no fly-through corridor".
  `checkMission()` applies both to the whole plan before a mission starts.
- A leg crossing a corridor is routed through the crossing point first (via
  waypoint, 1.5 m acceptance, name "corridor"); on arrival: "corridor crossed
  - continuing", then the leg resumes.
- Entering a keep-out zone mid-flight self-holds ("keep-out zone entered -
  holding") exactly like pause — no more `TARGET` packets. **RTL is exempt**:
  safety to get home outranks the hint. **Resume doubles as pilot override**
  (`zoneOverride`, cleared as soon as the drone leaves the zone so the next
  entry re-holds).
- Arrival with the target or the drone inside a wind zone needs **2 consecutive**
  in-radius ticks ("wind zone - confirm arrival") before it counts.

**WPL codec** — annotations ride along in the `.waypoints` file as `QGC WPL
110` extension rows with command id `31000` (`CMD_MAP_ANNOTATION`), one row per
vertex:

| column | meaning |
|---|---|
| `p1` | annotation type (wire value 0/1/2) |
| `p2` | ordinal within that type, renumbered 0..n-1 on save |
| `p3` | vertex index (rects 0-3, lines 0-1) |
| `x`, `y` | vertex lat/lon |

Annotation rows **must not** count toward the rowIndex 0 = home rule — an
annotation-only file must not invent a home. Desktop planners skip them
(non-navigation commands are already ignored), so the interchange stays
compatible.

**UX** — the editing UI lives on its own screen, the **Indoor Map** page
(see below); the Mission page draws the zones **read-only**, so the runner's
refusals and holds stay explicable without letting the pilot mutate the plan
while flying. Placement is a drag or two taps depending on the type (the mobile
adaptation of Valetudo's drop-at-viewport-centre), and a new shape is **selected
immediately** (Valetudo's add→active) so its editing handles appear at once.
Placement is persisted with the mission, not separately.

**Editing handles** are Valetudo's `LineClientStructure` / `RampClientStructure`
interaction model, 1:1. The active shape wears the `*_zone.svg` icons, each
rotated to the shape's own angle (`drawRotatedIcon`):

| handle | line (CORRIDOR) | quad (KEEPOUT / WIND) |
|---|---|---|
| ✕ delete | at `p0` — tap deletes | at `p0` — tap deletes |
| ✛ move | at `p1` — drags that endpoint | — |
| ↻ rotate | — | at `p1` — rotates all 4 points about the centroid |
| ↖↘ scale | — | at `p2` — projects the pointer onto the `p0→p1` / `p0→p3` axes, clamped to ≥2 m |

Dragging the **body** translates every point. Handle hitboxes swallow taps (only
the ✕ acts on tap); a drag is never confused with a map pan — Android consumes
the touch before `MapEventsOverlay`, iOS lets its editing `UIPanGestureRecognizer`
begin only on a handle/body.

**Snap (Valetudo's `postProcess`)** runs when the drag ends: `p0` (and a quad's
centre) to whole metres, the `p0→p1` angle to 5°, sizes to whole metres, floors
of 1 m (line) / 2 m (quad edge). Both apps snap identically —
`MapAnnotation.snap()` (Android) and `MapAnnotation.snapped()` (iOS).

**Visuals** are ported from Valetudo's map client structures:

- **Corridor** — `ThresholdClientStructure`: thin bar (half-width 4 dp/pt),
  fill `rgba(16,185,129,0.6)`, border `rgb(16,185,129)` 1 dp/pt, and **8 grooves
  parallel to the line** (`moveTo(0,yOffset) → lineTo(length,yOffset)`, step =
  width/9) in `rgba(16,185,129,0.9)`.
- **Keep-out** — `NoGoAreaClientStructure` red `rgb(239,68,68)`.
- **Wind** — `RampClientStructure` chrome: fill 0.15 active / 0.10 inactive,
  black halo `rgba(0,0,0,0.6)` at 7 dp/pt under a 5 dp/pt border that is
  `rgb(6,182,212)` **dashed [8,6] while active** and 0.7-alpha solid otherwise,
  carrying `CurtainClientStructure`'s sine waves (wavelength 24 dp/pt, amplitude
  4, step 2) with **per-end fade over 0.35× the wavelength**
  (`fadeFactor = distToEdge/fadeLength`, smoothstep), black halo under cyan,
  clipped to the quad.

Identical on both apps: `MissionMapView.AnnotationOverlay` +
`AnnotationHandles` (Android) and `MapAnnotationShape`/`MapAnnotationRenderer` +
`ValetudoHandles` (iOS). Ramp chevrons (`RampClientStructure`'s arrow) are
**not** drawn: this project has no ramp type — the equivalent hint is already
encoded semantically as corridor via-routing above.

The iOS side reads the SVG path data verbatim through a small path reader
(`SvgPathReader`: M/L/H/V/C/S/Q/T/A/Z, relative + implicit repeats, arc→cubic);
Android ships the same path strings in `ic_annotation_*.xml` vector drawables
and `AnnotationHandles`.

### Indoor Map page (vacuum-map-card port)

The annotation editor has its own screen — **Indoor Map**, a peer of Mission and
Tune (tab "Indoor") — laid out from
`piotrmachowski/lovelace-xiaomi-vacuum-map-card`'s `render()` order: map with
its selection overlay → **modes bar** (icon + name, one active) → **icons row**
(actions) → **tiles row** (the current selections). This project draws on a real
GPS map, so the card's `calibration_source` is *identity* (lat/lon direct —
`CoordinatesConverter` is never needed) and its coordinate rounding
(`coordinates_rounding: true`, whole metres) is Valetudo's `postProcess` snap.

Each chip is one card `MapMode` with its `selection_type`:

| chip | SelectionType | interaction |
|---|---|---|
| **Select** (default) | — (editing) | tap a shape to select it → Valetudo handles; body-drag translates; map pans elsewhere |
| **Keep-out** | `MANUAL_RECTANGLE` | one-finger drag draws a rubber-band rect (`rgba(16,185,129)` 0.15 fill / 0.9 stroke) → snapped `KEEPOUT` quad on release; pinch still zooms |
| **Wind** | `MANUAL_RECTANGLE` | same drag-rect → `WIND` quad |
| **Corridor** | `MANUAL_PATH` | tap endpoint 1 → tap endpoint 2 → snapped `CORRIDOR` line |
| **Go to** | `MANUAL_POINT` | tap sets the guided target (keep-out vetoes it) + marker; "start GUIDED from the Mission page" |

Placement modes are placement-only: no selection, no waypoint drops. A tap in a
rect mode does nothing; a second finger cancels the rubber band and hands the
gesture back to the map.

The **icons row** (the card's `icons-wrapper`) is Delete selected / Clear all /
Center (drone when known, else the first object). The **tiles row** (the card's
`tiles-wrapper` / `predefined_selections`) is one tile per annotation — "Keep-out
1", "Corridor 1", "Wind 1" per type ordinal, carrying the type's `CustomIcons.tsx`
glyph — plus one per waypoint ("WP 1"); a tile selects and centres (label + icon,
like the card's `label`/`icon` blocks). Empty state: "Draw a zone — pick a mode
above".

The plan is **shared** between the two screens through a small store —
`NavStore.java` / `NavStore` in `GpsNavCore.swift` — holding one `Navigator` and
the pending "Go to" target, so zones drawn on the Indoor Map are the zones the
Mission page flies and vetoes with. The Mission page keeps waypoint long-press
add, tap-for-GUIDED, load/save/clear of the `.waypoints` file and the runner
controls; only its annotation **editing** moved out.

Identical on both apps: `IndoorMapActivity` +
`activity_indoor_map.xml` (Android) and `IndoorMapViewController` (iOS).

---

## 11. Console (port 0, channel 0)

Drone→phone UTF-8 text after the header byte. The iOS app decodes inbound
datagrams as UTF-8 text and can throw on arbitrary binary, so anything that
both apps must tolerate safely should go here as text.

The firmware never sends an empty console packet — the Android resend-queue
matcher indexes into the payload without a bounds check.
