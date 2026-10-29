# Esp32_Drone

Flight-controller firmware and phone apps for the **XW_Z1_V2.0 欣薇_无人机** —
a small ESP32 quadcopter board carrying nothing but an MPU6500.

The firmware speaks the Bitcraze **CRTP** protocol over UDP, Bluetooth Classic
serial and BLE GATT, so the stock Crazyflie-style clients connect to it. Two
apps are included, both extended with flight modes, PID tuning and GPS mission
navigation.

```
Esp32_drone/       Arduino firmware + native regression tests
PROTOCOL.md        the wire protocol, in full
ESP-Drone-Android/ Android client
ESP-Drone-iOS/     iOS client
tools/pid-tune/    desktop PID auto-tuner (Python)
docs/              install guide, pilot's guide, app guide, course slides, diagrams, board schematic
```

**Want to get flying?**

1. **[docs/INSTALL.md](docs/INSTALL.md)** — flash the firmware, install an
   app, connect.
2. **[docs/USER_GUIDE.md](docs/USER_GUIDE.md)** — the pilot's guide: every
   screen, CUT MOTORS, arming, failsafe, low battery, and a first-flight
   checklist. **Tiếng Việt:** [docs/USER_GUIDE.vi.md](docs/USER_GUIDE.vi.md).
3. **[docs/APP_GUIDE.md](docs/APP_GUIDE.md)** — the app reference: every
   screen and control of the Android app, the iOS app and the Python tuner,
   with screen-map screenshots. **Tiếng Việt:** [docs/APP_GUIDE.vi.md](docs/APP_GUIDE.vi.md).

**Tuning PID from a computer?** [docs/PID_TUNER_GUIDE.md](docs/PID_TUNER_GUIDE.md)
— step by step with the desktop tuner in `tools/pid-tune`
(**Tiếng Việt:** [docs/PID_TUNER_GUIDE.vi.md](docs/PID_TUNER_GUIDE.vi.md)).

**New to drones?** [docs/slides](docs/slides/README.md) — a 150-slide course on
how the drone flies, every part on the board, and the firmware line by line
([PDF](docs/slides/ESP32_Drone_Course_EN.pdf) ·
[bản tiếng Việt](docs/slides/ESP32_Drone_Course_VI.pdf)).

Start with **[PROTOCOL.md](PROTOCOL.md)** for the wire format. It is the
authoritative reference for framing, the parameter and log TOCs, the
flight-control and navigation extensions, and the phone-side GPS mission
design. The **[connectivity research](docs/connectivity-research.md)**
compares the four transports and states plainly which claims are verified
and which are still open.

---

## Hardware limits — read this before flying

The board has **only an MPU6500**. No barometer, no time-of-flight, no GPS, no
magnetometer. Everything below follows from that, and the apps say so on screen
rather than pretending otherwise:

- **HOLD keeps a thrust level, not an altitude.** Hover height drifts as the
  battery sags. There is no altitude-hold to draw.
- **AUTO is a takeoff → hover → land sequence**, not waypoint flight.
- **FOLLOW / RTH use accelerometer dead reckoning.** It drifts — metres within a
  minute. The drone's position on the map is *estimated*.

## The board

![ESP32 pinout](docs/esp32-pinout.svg)

Full wiring notes, the quad layout and the prop directions are in
[docs/INSTALL.md](docs/INSTALL.md#pinout).

## Features

**Firmware** — attitude PID on the MPU6500, five flight modes (Manual, Hold,
Auto, Follow, Return-to-home), SBUS and CRTP command input, and the full CRTP
parameter and log TOCs so the stock apps' Settings and telemetry screens work
unmodified. Safety: a 500 ms link failsafe (it also honours the SBUS
receiver's own failsafe flag), arming only from idle and above 3.30 V,
auto-land on a sustained low battery, range-checked parameter writes, and no
flash writes or reboots while armed.

**Both apps** — the same four sections, **Fly · Mission · Tune · Setup**, under
one header that always shows link, battery voltage, mode, **ARMED** and a red
**CUT MOTORS** button in the same place. The Fly screen has one main button
that follows the situation (Takeoff / Land), live roll, pitch, sequence and
flight time, and a More panel for the rest. Also: WiFi (soft-AP and station),
Bluetooth Classic SPP and BLE GATT; PID tuning with live charts and auto-tune;
phone-IMU tilt-to-fly (Android); and GPS mission navigation with waypoints,
Mission Planner `.waypoints` import and return-to-launch.

![Fly screen](docs/images/fly-connected.png)

*Design mockup of the Fly screen; both apps follow this layout. More screens
in the [pilot's guide](docs/USER_GUIDE.md).*

iOS cannot use Bluetooth Classic SPP — iOS only exposes SPP to MFi accessories.
Use WiFi or BLE GATT there.

**Desktop tuner** — [`tools/pid-tune/`](tools/pid-tune/) tunes one axis at a
time over the same CRTP/UDP link, with eleven search algorithms (Gaussian-
process Bayesian optimisation, a neural ensemble, CMA-ES, differential
evolution, PSO and the rest) and a live plot of the stabiliser. It scores a
trial with the same cost model as the phone apps, so a gain set found on the
desktop means the same thing as one found on the phone.
**Spin up** streams level setpoints at a thrust you choose so the motors run on
a tuning stand. Step-by-step: [docs/PID_TUNER_GUIDE.md](docs/PID_TUNER_GUIDE.md).

## Building

**Firmware** — open [Esp32_drone/Esp32_drone.ino](Esp32_drone/Esp32_drone.ino)
in the Arduino IDE. Board *ESP32 Dev Module*, partition scheme **Huge APP**.

**Android** — `cd ESP-Drone-Android && ./gradlew assembleDebug`

**iOS** — `cd ESP-Drone-iOS && pod install`, then open
`Crazyflie client.xcworkspace`. Run *Product → Test* for the unit tests.

## Tests

| Suite | Run | Covers |
|---|---|---|
| `Esp32_drone/test/` | `./run.sh` | 83 checks in five suites: the launch-frame rotation, body↔world yaw rotation, log-config parsing including the client's wire padding, the self-level sign, and SBUS failsafe/lost-frame rejection. `run.sh` also checks the accelerometer scale matches the configured range |
| `Crazyflie clientTests` | *Product → Test* | 24 tests: great-circle geo math, `.waypoints` codec, mission edits, waypoint sequencing, launch frame |
| `tools/pid-tune/selftest.py` | `python selftest.py` | the whole tuner without hardware: wire framing and struct budgets, the link against a fake firmware transcribed from the sketch, the exact port-7 command bytes, the stand-thrust stream (arms on thrust-up, disarms at zero), telemetry decode and the cost model, the GP / Latin hypercube / MLP maths, all eleven optimisers against a synthetic bowl, history seeding, and the error paths |

The firmware suite is deliberately self-guarding: it greps the sketch for the
formula lines it copies and fails loudly if they drift, so the tests cannot pass
while testing something the firmware no longer does.

Two sign constants (`NAV_FWD_SIGN`, `NAV_LEFT_SIGN`) and all PID gains are still
at their defaults — they need a first flight to confirm. See PROTOCOL.md §6.

## Acknowledgements

- [Bitcraze](https://www.bitcraze.io) — the CRTP protocol and the Crazyflie
  Android/iOS clients this firmware is compatible with and these apps derive
  from.
- [quangvu16/PID-Calibration](https://github.com/quangvu16/PID-Calibration) —
  the tuning UI and auto-tune design.
- [quangvu16/Ackermann_robot_openbot](https://github.com/quangvu16/Ackermann_robot_openbot)
  — the GPS mission and waypoint model.

## Licence

[GPL-2.0](LICENSE) for the whole repository.

Note on provenance: `ESP-Drone-Android/` derives from Bitcraze's GPL-2.0
Crazyflie Android client, and `ESP-Drone-iOS/` derives from Bitcraze's Crazyflie
iOS client. Some per-file headers in those trees are Bitcraze's original notices
and are left intact as attribution; the licence that governs this repository as
a whole is GPL-2.0.
