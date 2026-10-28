#!/usr/bin/env python3
"""Live PID tuning with graphs for the ESP32 drone, over the CRTP/UDP link.

The PC counterpart of the phone apps' Tune PID screen, and the port of
PID-Calibration's tuner_gui.py: scrolling charts of everything the firmware
logs, each control-loop term on its own trace so you can see which one does the
work. What changed from the reference is the link (UDP instead of USB serial)
and what is being tuned — one drone axis at a time instead of the balance
robot's eight parameters.

  angle           stabilizer.<axis>, deg (yaw is logged as a rate in deg/s)
  rate            deg/s: stabilizer.yaw directly, finite-differenced otherwise
  PID terms       pid.<axis>_p / _i / _d, the loop's own breakdown
  controller      controller.<axis>, the summed PID output
  motor m1        one motor's command, 1000..2000 us; dashed lines at the rails

Left panel: the axis picker, parameter editors (Enter or Set applies at once,
-/+ step 10 %), Get/Save (NVS)/Load/Defaults, arm/disarm/emergency, the log
rate, statistics of the visible window, and CSV recording.

Auto-tune: tunes the ticked gains while the drone sits armed on a stand
(ml_tune.py), with the method picked in the list (METHODS below):
  ML (Bayesian)   a Gaussian process learns the cost from every trial and picks
                  the most promising next one (fewest trials, default)
  Neural          the same with a deep ensemble of small neural networks
  RL              policy gradient (PGPE) over the gains, one trial per episode
  CMA-ES, DE, PSO, Genetic   evolutionary / swarm search, populations of trials
  Nelder-Mead, SPSA          local direct search / noisy gradient estimate
  Twiddle         coordinate descent: +-step on one gain at a time, a win is
                  re-measured before it is kept
  Random          Latin-hypercube baseline
A second window shows every trial's cost and a Gaussian-process fit of the cost
over the gains (for all but ML only a picture of the trials).

  python3 tuner_gui.py                      # the soft-AP default, 192.168.43.42
  python3 tuner_gui.py --host esp-drone.local --axis pitch --window 20

Spin up (Drone panel) streams level setpoints at the Thrust % so the motors run;
the drone must be held by a stand, or have its props off. Stop, EMERGENCY or
closing the window drops thrust to zero, which disarms the board.
"""
import argparse
import csv
import math
import os
import queue
import threading
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

import matplotlib
import numpy as np
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402
from matplotlib.ticker import FuncFormatter, LogFormatter  # noqa: E402

import ml_tune  # noqa: E402
from drone import (AXES, MOTOR_SAT_HI, MOTOR_SAT_LO, PORT_LOG, PORT_PARAMS,  # noqa: E402
                   THRUST_MAX_PCT, Drone, ThrustStream, axis_names, gain_name,
                   open_tune_blocks, stop_log_blocks)
from ml_tune import from_y, read_history  # noqa: E402

# (subplot title, unit, [(series key, label, colour)])
PLOTS = [
    ("Angle", "deg", [("angle", "stabilizer", "C0")]),
    ("Rate", "deg/s", [("rate", "rate", "C1")]),
    ("PID terms", "", [("p", "P", "C2"), ("i", "I", "C3"), ("d", "D", "C4")]),
    ("Controller", "", [("control", "output", "C5")]),
    ("Motor m1", "", [("motor", "m1", "C9")]),
]
SERIES = ["angle", "rate", "p", "i", "d", "control", "motor", "roll", "pitch"]

HERE = os.path.dirname(os.path.abspath(__file__))
ML_LOG = os.path.join(HERE, "ml_tune_log.csv")      # both methods' trials
# method: (tuner class, default Range x, hint). The searches cover
# current / range .. x range; twiddle steps inside autotune.py's /3..x3.
METHODS = {
    "ML (Bayesian)": (ml_tune.BayesTuner, 2.0,
                      "Gaussian process + expected improvement. Fewest trials; the default."),
    "Neural (deep ensemble)": (ml_tune.NeuralTuner, 2.0,
                               "5 small neural nets learn the cost; tries the lowest confidence bound."),
    "RL (policy gradient)": (ml_tune.PolicyGradientTuner, 2.0,
                             "PGPE: a Gaussian policy over the gains, 2 trials per update."),
    "CMA-ES": (ml_tune.CmaesTuner, 2.0,
               "Evolution strategy that learns the step size and gain correlations."),
    "Differential evolution": (ml_tune.DifferentialEvolutionTuner, 2.0,
                               "Population search, DE/rand/1/bin. Needs 30+ trials."),
    "Particle swarm (PSO)": (ml_tune.PsoTuner, 2.0, "Swarm search. Needs 30+ trials."),
    "Genetic algorithm": (ml_tune.GeneticTuner, 2.0,
                          "Selection, crossover, mutation, elitism. Needs 30+ trials."),
    "Nelder-Mead": (ml_tune.NelderMeadTuner, 2.0,
                    "Downhill simplex: local, no model, sensitive to noise."),
    "SPSA": (ml_tune.SpsaTuner, 2.0,
             "Noisy gradient from 2 trials per step (extremum-seeking style). Local."),
    "Twiddle": (ml_tune.TwiddleTuner, 3.0,
                "One gain at a time, wins re-measured. Simple, stays close to the start."),
    "Random (baseline)": (ml_tune.RandomTuner, 2.0,
                          "Latin-hypercube samples: what the others must beat."),
}
HISTORY_LOGS = [ML_LOG]
GAIN_HINTS = ("proportional", "integral", "derivative")


class ModelWindow:
    """Trial costs and the GP's predicted cost over the tuned gains."""

    def __init__(self, root):
        self.top = tk.Toplevel(root)
        self.top.title("Auto-tune model")
        self.top.protocol("WM_DELETE_WINDOW", self.close)
        self.fig = Figure(figsize=(10, 4), layout="constrained")
        self.ax_cost, self.ax_map = self.fig.subplots(1, 2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.top)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def close(self):
        self.top.destroy()
        self.top = None

    def update(self, d):
        ax = self.ax_cost
        ax.clear()
        costs = from_y(d["Y"][[i for i, o in enumerate(d["own"]) if o]])
        fell = [f for f, o in zip(d["fell_mask"], d["own"]) if o]
        n = range(1, len(costs) + 1)
        ax.plot(n, costs, color="C7", lw=0.8)
        for k, c, f in zip(n, costs, fell):
            ax.plot(k, c, "x" if f else "o", color="C3" if f else "C0", ms=5)
        ok = [c if not f else math.inf for c, f in zip(costs, fell)]
        run_min = [min(ok[:k]) for k in n]
        ax.step(n, run_min, where="post", color="C2", lw=1.2, label="best trial so far")
        if d["best_pred"] is not None:
            ax.axhline(d["best_pred"], color="C2", ls="--", lw=0.8,
                       label="best (mean of 2)" if d["method"] == "twiddle" else "model's best")
        ax.set_yscale("log")
        ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:.3g}"))
        ax.yaxis.set_minor_formatter(LogFormatter(minor_thresholds=(1, 0.4)))  # only when narrow
        ax.set_xlabel("trial", fontsize=8)
        ax.set_ylabel("cost (x = tipped)", fontsize=8)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
        ax.tick_params(labelsize=7)
        self.draw_map(d)
        self.canvas.draw_idle()

    def draw_map(self, d):
        ax, gp, space, X = self.ax_map, d["gp"], d["space"], d["X"]
        ax.clear()
        if gp is None:
            return
        ax.tick_params(labelsize=7)
        base = d["best_x"] if d["best_x"] is not None else X[-1]
        u = np.linspace(0, 1, 60)
        colours = ["C3" if f else ("C0" if o else "C7") for f, o in zip(d["fell_mask"], d["own"])]
        if len(space.ids) == 1:
            mean, std = gp.predict(u[:, None])
            px = space.value(0, u)
            ax.plot(px, from_y(mean), color="C0")
            ax.fill_between(px, from_y(mean - 2 * std), from_y(mean + 2 * std), color="C0", alpha=0.2)
            ax.scatter(space.value(0, X[:, 0]), from_y(d["Y"]), c=colours, s=14, zorder=3)
            ax.set_yscale("log")
            ax.set_xlabel(space.names[0], fontsize=8)
            ax.set_ylabel("predicted cost (band = 2 std)", fontsize=8)
        else:
            # first two tuned values; any others held at the current best
            gx, gy = np.meshgrid(u, u)
            pts = np.tile(base, (gx.size, 1))
            pts[:, 0], pts[:, 1] = gx.ravel(), gy.ravel()
            mean = from_y(gp.predict(pts)[0]).reshape(gx.shape)
            ax.contourf(space.value(0, gx), space.value(1, gy), np.log10(np.clip(mean, 1, None)),
                        levels=20, cmap="viridis")
            ax.scatter(space.value(0, X[:, 0]), space.value(1, X[:, 1]), c=colours, s=14,
                       edgecolors="w", linewidths=0.5)
            if d["best_x"] is not None:
                ax.plot(space.value(0, base[0]), space.value(1, base[1]), "*", color="w", ms=12,
                        markeredgecolor="k")
            ax.set_xlabel(space.names[0], fontsize=8)
            ax.set_ylabel(space.names[1], fontsize=8)
            extra = ", others at best" if len(space.ids) > 2 else ""
            ax.set_title(f"predicted cost (dark = better{extra}); red = tipped, grey = past runs",
                         fontsize=8)
        # every drone gain is multiplicative: both axes logarithmic
        plain = FuncFormatter(lambda v, _: f"{v:.3g}")
        ax.xaxis.set_major_formatter(plain)
        ax.xaxis.set_minor_formatter(plain)
        if len(space.ids) > 1:
            ax.yaxis.set_major_formatter(plain)
            ax.yaxis.set_minor_formatter(plain)


class TunerGui:
    def __init__(self, root, drone, axis, window_s):
        self.root = root
        self.drone = drone
        self.axis = axis
        self.names = axis_names(axis)
        self.window = tk.DoubleVar(value=window_s)
        self.paused = False
        self.rec = None                     # (file, writer, host time of the last row)
        self.params = [None] * 3
        self.ml_thread = None
        self.ml_stop = threading.Event()
        self.ml_queue = queue.Queue()
        self.model_win = None
        self.ml_last = None                 # latest "trial" report, for the model window
        self.stream = None                  # drone.ThrustStream while the motors spin
        root.title(f"Drone Tuner — {axis}")
        root.protocol("WM_DELETE_WINDOW", self.close)

        side = ttk.Frame(root, padding=6)
        side.pack(side=tk.LEFT, fill=tk.Y)
        self._build_link(side)
        self._build_params(side)
        self._build_controls(side)
        self._build_autotune(side)
        self.stats = tk.StringVar(value="")
        ttk.Label(side, textvariable=self.stats, font=("TkFixedFont", 9), justify=tk.LEFT
                  ).pack(anchor=tk.W, pady=(8, 0))
        self.status = tk.StringVar(value="connecting...")
        ttk.Label(side, textvariable=self.status, foreground="#a33", wraplength=280
                  ).pack(side=tk.BOTTOM, anchor=tk.W)

        self.fig = Figure(figsize=(9, 8), layout="constrained")
        self.axes = self.fig.subplots(len(PLOTS), 1, sharex=True)
        self.lines = {}
        for ax, (title, unit, series) in zip(self.axes, PLOTS):
            ax.set_ylabel(f"{title}\n{unit}" if unit else title, fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, alpha=0.3)
            for key, label, colour in series:
                self.lines[key], = ax.plot([], [], color=colour, lw=1.1, label=label)
            ax.legend(loc="upper left", fontsize=7, ncol=len(series))
        self.axes[0].axhline(0, color="k", lw=0.6)
        for y in (MOTOR_SAT_HI, MOTOR_SAT_LO):
            self.axes[-1].axhline(y, color="r", lw=0.6, ls="--")
        self.axes[-1].set_xlabel("s", fontsize=8)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        root.after(300, self.start)

    # ---------------------------------------------------------------- widgets
    def _build_link(self, parent):
        box = ttk.LabelFrame(parent, text="Link", padding=4)
        box.pack(fill=tk.X)
        ttk.Label(box, text=self.drone.host, font=("TkFixedFont", 9)
                  ).grid(row=0, column=0, columnspan=3, sticky=tk.W)
        ttk.Label(box, text="Axis").grid(row=1, column=0, sticky=tk.W)
        self.axis_var = tk.StringVar(value=self.axis)
        for n, name in enumerate(AXES):
            ttk.Radiobutton(box, text=name.capitalize(), value=name, variable=self.axis_var,
                            command=self.switch_axis).grid(row=1, column=1 + n, sticky=tk.W)
        self.link_btn = ttk.Button(box, text="Reconnect", command=self.reconnect)
        self.link_btn.grid(row=2, column=0, columnspan=3, sticky=tk.W, pady=(4, 0))

    def _build_params(self, parent):
        box = ttk.LabelFrame(parent, text="Gains", padding=4)
        box.pack(fill=tk.X, pady=(6, 0))
        self.entries = []
        self.labels = []
        for k in range(3):
            lab = ttk.Label(box, text=self.names[k])
            lab.grid(row=k, column=0, sticky=tk.W)
            self.labels.append(lab)
            var = tk.StringVar()
            e = ttk.Entry(box, textvariable=var, width=9)
            e.grid(row=k, column=1, padx=2)
            e.bind("<Return>", lambda _e, k=k: self.set_param(k))
            ttk.Button(box, text="-", width=2, command=lambda k=k: self.step(k, -1)).grid(row=k, column=2)
            ttk.Button(box, text="+", width=2, command=lambda k=k: self.step(k, +1)).grid(row=k, column=3)
            ttk.Button(box, text="Set", width=4, command=lambda k=k: self.set_param(k)).grid(row=k, column=4)
            self.entries.append(var)
        row = ttk.Frame(box)
        row.grid(row=3, column=0, columnspan=5, pady=(4, 0))
        for text, cmd in (("Get", self.refresh_params), ("Save", self.save),
                          ("Load", self.load), ("Defaults", self.defaults)):
            ttk.Button(row, text=text, width=8, command=cmd).pack(side=tk.LEFT, padx=1)

    def _build_controls(self, parent):
        box = ttk.LabelFrame(parent, text="Drone", padding=4)
        box.pack(fill=tk.X, pady=(6, 0))
        ttk.Button(box, text="ARM", width=8, command=self.arm).grid(row=0, column=0, sticky=tk.W)
        ttk.Button(box, text="DISARM", width=8, command=self.disarm).grid(row=0, column=1, sticky=tk.W)
        ttk.Button(box, text="EMERGENCY", width=11, command=self.emergency).grid(row=0, column=2, sticky=tk.W)
        ttk.Label(box, text="Log Hz").grid(row=1, column=0, sticky=tk.W)
        self.rate = tk.IntVar(value=20)
        sp = ttk.Spinbox(box, from_=1, to=50, width=4, textvariable=self.rate,
                         command=self.set_rate)
        sp.bind("<Return>", lambda _e: self.set_rate())
        sp.grid(row=1, column=1, sticky=tk.W)
        ttk.Label(box, text="Window s").grid(row=2, column=0, sticky=tk.W)
        ttk.Spinbox(box, from_=2, to=40, width=4, textvariable=self.window).grid(row=2, column=1, sticky=tk.W)
        self.pause_btn = ttk.Button(box, text="Pause", command=self.toggle_pause)
        self.pause_btn.grid(row=3, column=0, sticky=tk.W, pady=(4, 0))
        self.rec_btn = ttk.Button(box, text="Record CSV", command=self.toggle_record)
        self.rec_btn.grid(row=3, column=1, columnspan=2, pady=(4, 0), sticky=tk.W)
        # Stand thrust: the firmware spins the motors only while CommanderPackets
        # arrive (drone.ThrustStream). Spin up arms it; Stop / EMERGENCY / closing
        # this window drops thrust to zero, which disarms it.
        ttk.Label(box, text="Thrust %").grid(row=4, column=0, sticky=tk.W, pady=(6, 0))
        self.thrust_pct = tk.DoubleVar(value=30.0)
        tsp = ttk.Spinbox(box, from_=0, to=THRUST_MAX_PCT, increment=1, width=4,
                          textvariable=self.thrust_pct, command=self.set_thrust)
        tsp.bind("<Return>", lambda _e: self.set_thrust())
        tsp.grid(row=4, column=1, sticky=tk.W, pady=(6, 0))
        self.spin_btn = ttk.Button(box, text="Spin up", command=self.toggle_spin)
        self.spin_btn.grid(row=4, column=2, sticky=tk.W, pady=(6, 0))

    def _build_autotune(self, parent):
        box = ttk.LabelFrame(parent, text="Auto-tune", padding=4)
        box.pack(fill=tk.X, pady=(6, 0))
        self.ml_method = tk.StringVar(value=next(iter(METHODS)))
        methods = ttk.Combobox(box, textvariable=self.ml_method, values=list(METHODS),
                               state="readonly", width=24)
        methods.grid(row=0, column=0, columnspan=4, sticky=tk.W)

        def picked(_event=None):
            self.ml_range.set(METHODS[self.ml_method.get()][1])
            self.ml_info.set(METHODS[self.ml_method.get()][2])
        methods.bind("<<ComboboxSelected>>", picked)
        self.ml_use = []
        self.ml_checks = []
        for k in range(3):
            var = tk.BooleanVar(value=True)
            chk = ttk.Checkbutton(box, text=self.names[k], variable=var)
            chk.grid(row=1 + k // 2, column=(k % 2) * 2, columnspan=2, sticky=tk.W)
            self.ml_use.append((k, var))
            self.ml_checks.append(chk)
        self.ml_history = tk.BooleanVar(value=True)
        ttk.Checkbutton(box, text="learn from past trials (logs)", variable=self.ml_history
                        ).grid(row=3, column=0, columnspan=4, sticky=tk.W)
        self.ml_trials = tk.IntVar(value=20)
        self.ml_trial_s = tk.DoubleVar(value=6.0)
        self.ml_range = tk.DoubleVar(value=2.0)
        for n, (text, var, lo, hi, inc) in enumerate((
                ("Trials", self.ml_trials, 5, 80, 1), ("Trial s", self.ml_trial_s, 3, 20, 1),
                ("Range x", self.ml_range, 1.2, 5, 0.5))):
            r, c = 4 + n // 2, (n % 2) * 2
            ttk.Label(box, text=text).grid(row=r, column=c, sticky=tk.W)
            ttk.Spinbox(box, from_=lo, to=hi, increment=inc, width=5, textvariable=var
                        ).grid(row=r, column=c + 1, sticky=tk.W)
        self.ml_btn = ttk.Button(box, text="Start auto-tune", command=self.toggle_autotune)
        self.ml_btn.grid(row=6, column=0, columnspan=2, sticky=tk.W, pady=(4, 0))
        ttk.Button(box, text="Model", command=self.show_model).grid(row=6, column=2, columnspan=2,
                                                                     pady=(4, 0))
        self.ml_info = tk.StringVar(value=METHODS[self.ml_method.get()][2])
        ttk.Label(box, textvariable=self.ml_info, wraplength=280, justify=tk.LEFT
                  ).grid(row=7, column=0, columnspan=4, sticky=tk.W)

    # ---------------------------------------------------------------- drone I/O
    def start(self):
        try:
            self.open_blocks()
            self.refresh_params()
            self.status.set("")
        except (TimeoutError, OSError, RuntimeError, KeyError) as e:
            self.status.set(f"link problem ({e}); check the drone, then press Reconnect")
        self.root.after(100, self.redraw)

    def open_blocks(self):
        open_tune_blocks(self.drone, self.axis, int(1000 / max(1, self.rate.get())))

    def set_rate(self):
        try:
            self.open_blocks()
        except (TimeoutError, OSError, RuntimeError) as e:
            self.status.set(f"log rate: {e}")

    def reconnect(self):
        try:
            self.drone.fetch_toc(PORT_PARAMS, self.drone.param_toc)
            self.drone.fetch_toc(PORT_LOG, self.drone.log_toc)
            self.open_blocks()
            self.refresh_params()
            self.status.set("")
        except (TimeoutError, OSError, RuntimeError) as e:
            self.status.set(f"reconnect: {e}")

    def switch_axis(self):
        axis = self.axis_var.get()
        if axis == self.axis:
            return
        try:
            stop_log_blocks(self.drone)
        except (TimeoutError, OSError, RuntimeError):
            pass
        self.axis = axis
        self.names = axis_names(axis)
        for k in range(3):
            self.labels[k].config(text=self.names[k])
            self.ml_checks[k].config(text=self.names[k])
        self.root.title(f"Drone Tuner — {axis}")
        try:
            self.open_blocks()
            self.refresh_params()
        except (TimeoutError, OSError, RuntimeError, KeyError) as e:
            self.status.set(f"switch axis: {e}")

    def run(self, what, fn):
        try:
            return fn()
        except (TimeoutError, OSError, RuntimeError, KeyError) as e:
            self.status.set(f"{what}: {e}")
            return None

    def refresh_params(self):
        got = self.run("get gains", lambda: [self.drone.get_param(n) for n in self.names])
        if got is None:
            return
        for k, v in enumerate(got):
            self.params[k] = v
            self.entries[k].set(f"{v:.4g}")

    def set_param(self, k, value=None):
        if value is None:
            try:
                value = float(self.entries[k].get())
            except ValueError:
                self.status.set(f"{self.names[k]}: not a number")
                return
        value = max(ml_tune.P_MIN[k], min(ml_tune.P_MAX[k], value))
        applied = self.run(f"set {self.names[k]}",
                           lambda: self.drone.set_param(gain_name(self.axis, k), value))
        if applied is not None:
            self.params[k] = applied
            self.entries[k].set(f"{applied:.4g}")
            self.status.set("")

    def step(self, k, sign):
        try:
            v = float(self.entries[k].get())
        except ValueError:
            return
        v = v * (1 + 0.1 * sign) if v else 0.1 * max(sign, 0)
        self.set_param(k, round(v, 4))

    def command(self, what, fn):
        """Run a port-7 command and say whether the firmware accepted it.
        The firmware answers status 1 when it refuses (ARM not from idle,
        SAVE while armed, low battery), which is not an error to swallow."""
        ok = self.run(what, fn)
        if ok is True:
            self.status.set(f"{what}: OK")
        elif ok is False:
            self.status.set(f"{what}: refused by the drone")
        return ok

    def save(self):
        if messagebox.askyesno("Save", "Write the current gains to NVS?"):
            self.command("save", self.drone.save)

    def load(self):
        if self.command("load", self.drone.load):
            self.refresh_params()

    def defaults(self):
        if messagebox.askyesno("Defaults", "Restore the firmware's default gains?"):
            if self.command("defaults", self.drone.defaults):
                self.refresh_params()

    def toggle_spin(self):
        if self.stream and self.stream.running:
            self.stop_spin()
            self.status.set("motors stopped (thrust 0, disarmed)")
            return
        try:
            pct = float(self.thrust_pct.get())
        except (tk.TclError, ValueError):
            self.status.set("thrust %: not a number")
            return
        if not messagebox.askyesno(
                "Spin up", f"Spin the motors to {pct:.0f} % thrust?\n\n"
                "The drone must be held by a stand (props on) or have its props off. "
                "Stop, EMERGENCY or closing this window stops them."):
            return
        self.stream = ThrustStream(self.drone, pct)
        self.spin_btn.config(text="Stop motors")
        self.status.set(f"spinning up to {pct:.0f} %")

    def set_thrust(self):
        if self.stream and self.stream.running:
            try:
                self.stream.set_pct(float(self.thrust_pct.get()))
            except (tk.TclError, ValueError):
                pass

    def stop_spin(self):
        if self.stream:
            self.stream.stop()
            self.stream = None
        self.spin_btn.config(text="Spin up")

    def arm(self):
        self.command("arm", self.drone.arm)

    def disarm(self):
        self.command("disarm", self.drone.disarm)

    def emergency(self):
        # No confirmation: one click cuts the motors, as CUT MOTORS does on the
        # phone's Tune screen. A dialog here would cost seconds when it matters.
        if self.stream:
            self.stream.target = 0          # zero thrust at once, before the reply
        self.command("emergency", self.drone.emergency)
        self.stop_spin()

    # ---------------------------------------------------------------- auto-tune
    def toggle_autotune(self):
        if self.ml_thread and self.ml_thread.is_alive():
            self.ml_stop.set()
            self.ml_info.set("stopping after this trial...")
            return
        ids = [k for k, var in self.ml_use if var.get()]
        if not ids:
            self.ml_info.set("tick at least one gain to tune")
            return
        try:
            trials, trial_s = int(self.ml_trials.get()), float(self.ml_trial_s.get())
            factor = float(self.ml_range.get())
        except (tk.TclError, ValueError):
            self.ml_info.set("trials / trial s / range: not a number")
            return
        minutes = trials * (trial_s + 5) / 60
        if not (self.stream and self.stream.running) and not messagebox.askyesno(
                "Auto-tune", "The motors are not spinning (Spin up in the Drone panel).\n\n"
                "Without thrust every trial measures a still drone, so the scores mean "
                "nothing. Start anyway?"):
            return
        if not messagebox.askyesno(
                "Auto-tune", "The drone will try parameter sets on its own for about "
                f"{minutes:.0f} min.\n\nOn a stand, props clear, armed. Stop it at any time "
                "and use EMERGENCY if it misbehaves.\n\n"
                f"Tuning {self.axis} over {', '.join(self.names[i] for i in ids)}.\n\nStart?"):
            return
        self.ml_stop = threading.Event()
        tuner_class = METHODS[self.ml_method.get()][0]
        tuner = tuner_class(self.drone, self.axis, ids, trials, trial_s, max(1.1, factor),
                            read_history(HISTORY_LOGS) if self.ml_history.get() else [],
                            ML_LOG, self.ml_stop,
                            lambda kind, data: self.ml_queue.put((kind, data)))
        self.ml_thread = threading.Thread(target=tuner.run, daemon=True)
        self.ml_thread.start()
        self.ml_btn.config(text="Stop auto-tune")
        self.ml_info.set(f"starting ({self.ml_method.get()})")

    def show_model(self):
        if self.model_win is None or self.model_win.top is None:
            self.model_win = ModelWindow(self.root)
            if self.ml_last:
                self.model_win.update(self.ml_last)
        else:
            self.model_win.top.lift()

    def poll_autotune(self):
        while True:
            try:
                kind, d = self.ml_queue.get_nowait()
            except queue.Empty:
                return
            if kind == "status":
                self.ml_info.set(d)
            elif kind == "params":
                for k, v in enumerate(d):
                    self.params[k] = v
                    self.entries[k].set(f"{v:.4g}")
            elif kind == "trial":
                self.ml_last = d
                tried = " ".join(f"{k}={v:.3g}" for k, v in d["tried"].items())
                best = " ".join(f"{k}={v:.3g}" for k, v in d["best"].items())
                pred = "" if d["best_pred"] is None else f" (predicted cost {d['best_pred']:.1f})"
                result = "TIPPED" if d["fell"] else f"cost {d['cost']:.1f}"
                self.ml_info.set(f"trial {d['n']}/{d['total']}: {tried} -> {result}\n"
                                 f"best: {best}{pred}")
                self.show_model()
                self.model_win.update(d)
            elif kind == "done":
                self.ml_btn.config(text="Start auto-tune")
                changed = [f"{n}: {a:.4g} -> {b:.4g}" for n, a, b in zip(self.names,
                           d["start"] or [], d["result"] or []) if abs(a - b) > 1e-6]
                self.ml_info.set(f"{d['reason']}. " + ("; ".join(changed) if changed else "no change"))
                if changed and messagebox.askyesno(
                        "Auto-tune " + d["reason"].split(":")[0],
                        "Applied:\n  " + "\n  ".join(changed) + "\n\nWrite them to NVS?"):
                    self.run("save", self.drone.save)

    # ---------------------------------------------------------------- recording
    def toggle_record(self):
        if self.rec:
            self.rec[0].close()
            self.rec = None
            self.rec_btn.config(text="Record CSV")
            return
        path = filedialog.asksaveasfilename(defaultextension=".csv", initialfile="tuner_log.csv")
        if not path:
            return
        f = open(path, "w", newline="")
        w = csv.writer(f)
        w.writerow(["host_s"] + SERIES + self.names)
        self.rec = [f, w, time.monotonic()]
        self.rec_btn.config(text="Stop recording")

    def write_rows(self):
        f, w, last = self.rec
        new = [s for s in self.drone.samples_since(last)]
        for s in new:
            w.writerow([f"{s['t']:.3f}"] + [s[k] for k in SERIES]
                       + ["" if p is None else f"{p:.5g}" for p in self.params])
        if new:
            self.rec[2] = new[-1]["t"]

    # ---------------------------------------------------------------- plotting
    def toggle_pause(self):
        self.paused = not self.paused
        self.pause_btn.config(text="Resume" if self.paused else "Pause")

    def span(self):
        try:
            return max(1.0, float(self.window.get()))
        except (tk.TclError, ValueError):
            return 10.0

    def visible(self):
        return self.drone.samples_since(time.monotonic() - self.span())

    def redraw(self):
        self.poll_autotune()
        if self.rec:
            self.write_rows()
        frames = self.visible()
        if not self.paused and frames:
            now = time.monotonic()
            xs = [s["t"] - now for s in frames]
            for key, line in self.lines.items():
                line.set_data(xs, [s[key] for s in frames])
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view(scalex=False)
            self.axes[0].set_xlim(-self.span(), 0)
            self.canvas.draw_idle()
            self.update_stats(frames)
        elif not frames:
            self.status.set("no telemetry")
        self.root.after(100, self.redraw)

    def update_stats(self, frames):
        n = len(frames)
        dt = frames[-1]["t"] - frames[0]["t"]
        ang = [s["angle"] for s in frames]
        mean = sum(ang) / n
        std = math.sqrt(sum((x - mean) ** 2 for x in ang) / n)
        rate = math.sqrt(sum(s["rate"] ** 2 for s in frames) / n)
        sat = sum(s["motor"] >= MOTOR_SAT_HI or s["motor"] <= MOTOR_SAT_LO for s in frames) / n
        ctrl = sum(abs(s["control"]) for s in frames) / n
        last = frames[-1]
        tipped_now = abs(last["roll"]) > 15 or abs(last["pitch"]) > 15
        state = "TIPPED" if tipped_now else "level"
        self.stats.set(f"{self.axis}   {state}   {n / dt if dt > 0 else 0:4.1f} Hz\n"
                       f"angle mean {mean:7.2f}  std {std:6.2f}\n"
                       f"rate rms   {rate:7.1f}\n"
                       f"ctrl mean  {ctrl:7.1f}\n"
                       f"motor sat  {sat:7.0%}\n"
                       f"tilt       {last['roll']:5.1f} {last['pitch']:5.1f}")

    def close(self):
        self.ml_stop.set()
        self.stop_spin()
        if self.rec:
            self.rec[0].close()
        try:
            stop_log_blocks(self.drone)
        except (TimeoutError, OSError, RuntimeError):
            pass
        self.drone.close()
        self.root.destroy()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="192.168.43.42",
                    help="drone address (soft-AP default: 192.168.43.42)")
    ap.add_argument("--device-port", type=int, default=2390)
    ap.add_argument("--app-port", type=int, default=2399)
    ap.add_argument("--axis", choices=AXES, default="roll")
    ap.add_argument("--window", type=float, default=10.0, help="seconds shown (default 10)")
    args = ap.parse_args()

    drone = Drone(args.host, args.device_port, args.app_port)
    try:
        drone.fetch_toc(PORT_PARAMS, drone.param_toc)
        drone.fetch_toc(PORT_LOG, drone.log_toc)
    except TimeoutError as e:
        raise SystemExit(f"no reply from {args.host} ({e}); "
                         "join the drone's WiFi first (ESP-Drone / 12345678)")
    root = tk.Tk()
    TunerGui(root, drone, args.axis, args.window)
    root.mainloop()


if __name__ == "__main__":
    main()
