# pid-tune — desktop PID auto-tuning for the ESP32 drone

A tkinter + matplotlib GUI that tunes one drone axis at a time (roll, pitch or
yaw) over the same CRTP/UDP link the phone apps use. Ported from
[`quangvu16/PID-Calibration`](https://github.com/quangvu16/PID-Calibration):
the eleven search algorithms are that project's, unchanged — same grids, same
hyperparameters, same trial loop — so a run here and a run there mean the same
thing. What changed is what a "gain" is and where a trial's score comes from.

Use it when the phone apps' on-screen tuner is not enough: you want a live
plot of the stabiliser, a cost model you can read, or one of the model-based
searches instead of hand-nudging gains.

## Install

Needs Python 3.9+ with `numpy` and `matplotlib`. No scikit-learn, no torch —
the Gaussian process and the neural ensemble are hand-written in `ml_tune.py`.

```sh
python3 -m venv .venv
. .venv/bin/activate
pip install numpy matplotlib
```

## Run

1. Power the drone and join its network — soft-AP **ESP-Drone** / **12345678**,
   drone at `192.168.43.42`, or your own station-mode network.
2. Put the drone on a stand, props clear. The tuner spins m1.
3. `python tuner_gui.py`

```
python tuner_gui.py --host 192.168.43.42 --axis roll
```

| flag | default | meaning |
| --- | --- | --- |
| `--host` | `192.168.43.42` | drone address (soft-AP default; `esp-drone.local` in station mode) |
| `--device-port` | `2390` | UDP port the drone listens on |
| `--app-port` | `2399` | UDP port the drone replies to |
| `--axis` | `roll` | `roll`, `pitch` or `yaw` |
| `--window` | `10` | seconds of telemetry shown in the plots |

If it exits with `no reply from ...`, you are not on the drone's network yet —
the two TOC fetches at startup are the first thing that talks to the board.

## The window

**Link** — address, axis radio, Reconnect. Changing axis tears down both log
blocks and rebuilds them for the new axis.

**Gains** — the three PID values for the selected axis, each with `-` `+` and a
text field you can type into. `Get` reads the board, `Set` writes what is in the
fields, `Save` persists to NVS (`FC_SAVE`), `Load` / `Defaults` reload. Writes
are clamped by the firmware (`paramClampF`) to kp 0–100, ki 0–20, kd 0–200 —
the search box never leaves those bounds either.

**Drone** — ARM / DISARM / EMERGENCY, the telemetry rate (20 Hz by default),
how many seconds the plots show, Pause, and Record CSV.

**Motor test** — bench testing of single motors with the PID loop bypassed
(`MOTOR_TEST`, `0x0C`, PROTOCOL.md §5). A shared **Speed** spinbox (1000–1500 in
steps of 50; 1000 = stopped) plus a **Run** / **Stop** pair for each motor
M1–M4 and a **STOP ALL** button. `Run` spins only that motor at the chosen
speed and returns the other three to 1000; `Stop` returns its motor to 1000
keeping the others' state; **STOP ALL** sends `enable=0`, which ends the test
at once. While any motor is spinning the GUI re-sends the packet every 150 ms,
because the firmware stops the test after 300 ms of silence. Disarmed bench
use only — the firmware refuses the test while armed, and a failed ack is
reported on the status line ("motor test not acked — drone must be disarmed").

**Auto-tune** — method dropdown, which of the three gains to include (uncheck
one to hold it fixed), "learn from past trials", and Trials / Trial s / Range x.
`Start` runs it in a worker thread; `Model` opens the cost-vs-gain window.

Five stacked plots: angle (stabilizer), rate, the P/I/D terms as the firmware
computed them, controller output, and motor m1 with dashed rails at the
saturation limits 1990 / 1010.

## Mission plan

**Mission plan → Open plan window** opens the Mission Planner *Flight Plan*
screen on the laptop — the Python twin of the Android app's Plan page
(`gpsnav/PlannerActivity`) and the iOS Plan screen. Same mission grid, same
`.waypoints` files, same Upload / Download / Start over the drone's Wi-Fi.

- **Plan plot** — the plan in metres about home (PROTOCOL.md §12). Click to
  add a waypoint there. The phone apps veto taps inside keep-out zones
  (NavStore); the PC has no map database, so nothing is vetoed.
- **Grid + row editor** — Command (the five flown commands or any MAV_CMD
  number), P1–P4, Lat, Lng, Alt. Edits commit with **Set row** or when a box
  loses focus, like Mission Planner's grid.
- **Alt / Set alt / Add WP / Up / Down / Del** — the altitude new waypoints
  get (Set alt also writes it to the selected row), and row order.
- **Home lat / lon** — the launch origin. The phone apps take it from the GPS
  fix; here you type it. It becomes the file's home row and the ORIGIN sent
  before Upload or Start (the drone flies a dead-reckoned local frame — the
  origin bridges the plan's degrees and that frame).
- **Load / Save** — `.waypoints`, the QGC WPL 110 format Mission Planner
  writes. `planner.py` ports the reader/writer quirks and all 171 MAV_CMD
  names from Mission Planner's `MissionFile.cs` — including the blank-home
  insertion, the command-99 rewrite and the exact number formats — so files
  round-trip byte-for-byte with Mission Planner.
- **Upload / Download / Start** — the onboard mission (Read / Write / Start
  waypoints, PROTOCOL.md §12, port 7 channel 2). Before Upload or Start the
  window sends the home as the ORIGIN. The transfer re-sends its last message
  every 500 ms and gives up after 10 tries; a running transfer asks before
  the window closes.

## The eleven methods

| method | what it does | notes |
| --- | --- | --- |
| **ML (Bayesian)** | Gaussian process + expected improvement | fewest trials; the default |
| **Neural (deep ensemble)** | 5 small nets learn the cost, then tries the lowest confidence bound | |
| **RL (policy gradient)** | PGPE: a Gaussian policy over the gains, 2 trials per update | |
| **CMA-ES** | evolution strategy that learns step size and gain correlations | |
| **Differential evolution** | population search, DE/rand/1/bin | wants 30+ trials |
| **Particle swarm (PSO)** | swarm search | wants 30+ trials |
| **Genetic algorithm** | selection, BLX-0.3 crossover, mutation, elitism | wants 30+ trials |
| **Nelder-Mead** | downhill simplex | local, no model, noise-sensitive |
| **SPSA** | noisy gradient from 2 trials per step | local |
| **Twiddle** | one gain at a time, winners re-measured | stays near the start |
| **Random (baseline)** | Latin-hypercube samples | what the others must beat |

Every gain is searched **multiplicatively** — `current / range .. current × range`
on a log scale, because a gain acts proportionally — clipped to the firmware's
own clamp. `Range x` is that factor (2.0 by default, 3.0 for Twiddle).

## How a trial is scored

`drone.cost_of()` is the same `AutoTuner.costOf()` the Android and iOS apps use,
so a gain set scored here scores identically on the phone:

```
angleStd + 0.1*rateRms + 0.2*motorRms + 0.005*controlEffort + 20*saturatedShare
```

with m1 centred at 1500 and counted saturated at ≥1990 or ≤1010. A tip (either
axis past 25°) ends the trial early and scores `100 + 10 × seconds lost`, the
reference's fall penalty. The models learn `log(1 + cost)` so a fall is "much
worse" without erasing the differences between good trials.

After a tip the tuner puts the best gains found so far back and waits for the
drone to be righted (both axes within 15°, held for 1.5 s) before continuing.

**The result is not the luckiest single trial.** Among this run's trials that
did not tip, `choose_best` picks the one with the lowest GP-*predicted* cost.
One quiet window with bad gains is noise; the model averages it away.

## Telemetry

Two log blocks, exactly as `TuneActivity.java` builds them:

| block | contents | size |
| --- | --- | --- |
| 0 status | `pm.vbat`, `flight.mode`, `flight.armed`, `flight.auto_state`, `stabilizer.roll`, `stabilizer.pitch` | 15 B |
| 1 tune | `pid.<axis>_p/i/d`, `stabilizer.<axis>`, `controller.<axis>`, `motor.m1` | 22 B |

Both fit the firmware's 25-byte `LOG_MAX_PAYLOAD`, at 20 Hz (50 ms).

One honest substitution: **the firmware logs no roll/pitch gyro rates.** Only
`stabilizer.yaw` is a rate (deg/s). So roll/pitch rate is finite-differenced
from consecutive `stabilizer.*` angle samples at the block rate. On a 20 Hz
block that is coarse — treat the rate term as a relative signal, not a calibrated
measurement. Yaw uses the logged rate directly.

## Trial log and history

Every trial is appended to `ml_tune_log.csv`:

```
trial, stage, gain0, gain1, gain2, cost, fell,
angle_mean, angle_std, rate_rms, motor_rms, control_effort, sat
```

Tick **learn from past trials** and a new run seeds its model with rows from that
log whose *other* gains match the current ones within 10 %. That is how you tune
kp and kd in one run, then ki in the next without starting from nothing — and a
log taken on the roll axis is simply ignored by a pitch run. `Model` plots those
past trials alongside the current run so you can see what the model already knows.

## Safety

- The tuner writes one axis's gains and spins **m1 only**. Props off or the
  drone secured to a stand.
- The **Motor test** panel is for disarmed bench use only: it never arms the
  drone, `STOP ALL` / `Stop` end the run, and the firmware also stops on
  DISARM, EMERGENCY, on arming, or after 300 ms without a packet.
- `EMERGENCY` in the Drone panel is always available and stops the motors.
- Auto-tune runs in a worker thread; `Start` becomes `Stop`, and closing the
  window sets the stop flag, stops both log blocks and closes the socket.
- All gains are untested placeholders as shipped. The tuner will happily find
  the best point *inside the box you give it*; that is not the same as a drone
  that flies. Verify by hand before trusting a result in the air.

## Offline selftest

`selftest.py` checks the whole tool without hardware. It builds a
`FakeFirmware` UDP server transcribed from `Esp32_drone.ino` — TOC replies,
parameter read/write with `paramClampF`, log CREATE/APPEND/START/STOP/DELETE with
the firmware's exact parse — and then runs all eleven optimisers against a
synthetic bowl to confirm each one converges.

```sh
python selftest.py
```

Sections: wire framing and struct budgets, the link against the fake firmware,
telemetry decode and the cost model, the GP / Latin hypercube / MLP maths, all
eleven optimisers, history seeding, and error paths (including that a CREATE
with different variables over a live block is refused with `EXISTS`).

The plan side has its own suite, `planner_test.py`: it pins the `.waypoints`
codec against the Android `CodecCheck.java` driver's 32 checks (byte-exact
round-trip and every MissionPlanner quirk), the `MISSION_*` wire layout, the
upload/download/start handshake against a fake timer, and the local-frame
conversions.

```sh
python planner_test.py
```

## How this differs from the reference

`PID-Calibration` tuned a wheeled balancing robot over USB serial, with eight
search dimensions including a balance-point offset (`angle0`) and a `nudge()`
that pushed the robot to test recovery. The drone has none of that: three gains
per axis, no wheels, and a tip is a fall rather than a wobble. The algorithms
are the reference's verbatim; everything around them is this project's.

## How this differs from the phone apps

Same cost model, same tip threshold (25°, the firmware's `MAX_TILT_DEG`), same
fall penalty, same two telemetry blocks — a gain set scored here scores the same
on the phone. Two deliberate differences:

1. **Roll/pitch rate is finite-differenced**, as above, because there is no
   logged gyro rate to read. The apps derive it the same way.
2. **The trial logs are different shapes.** This writes the three tuned gains
   per row; the Android app writes all nine, so its `seed()` can match a past
   row's untuned gains. Both are read back only by the tool that wrote them.

The phone apps have no equivalent of the `Model` window — on Android the model's
best predicted cost is reported as text after each trial.
