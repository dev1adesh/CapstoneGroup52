#!/usr/bin/env python3
"""
Replicate Simulink flow: actual orientation (sliders) + reference [0,0,0] → PID → IK → T1, T2, T3.

No simulated plant. Sliders set the actual platform orientation (like IMU input to the PID).
Reference is fixed at [0, 0, 0]. PID output → MATLAB-style IK → motor torques.

3D view: platform orientation + wheel spin arrows (T1, T2, T3). Arrow = spin direction.

Run:  python3 tilt_control_ui.py   (from simulation/ik_validation)
  or  python3 simulation/ik_validation/tilt_control_ui.py   (from project root)
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
import tkinter as tk
from tkinter import ttk

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

from ik import ik
from pid import PID3
from remap import remap, REMAP_PITCH_SIGN, REMAP_ROLL_SIGN, REMAP_SWAP
from viz_3d import draw_platform_and_wheels

DT = 0.02  # s
DT_MS = int(DT * 1000)
DEG_RANGE = 30
MAX_T = 2.0
REF = np.array([0.0, 0.0, 0.0])  # fixed reference [roll, pitch, yaw] rad

KP, KI, KD = 50.0, 10.0, 5.0


def deg2rad(d: float) -> float:
    return math.radians(d)


class TiltControlUI:
    def __init__(self) -> None:
        self.root = tk.Tk()
        self.root.title("PID → IK → T1,T2,T3 — 3D platform + wheel spin")
        self.root.geometry("960x520")
        self.root.resizable(True, True)

        self.pid = PID3(kp=KP, ki=KI, kd=KD, dt=DT, out_limit=MAX_T)
        self.running = False
        self.after_id: str | None = None
        self._last_t1 = self._last_t2 = self._last_t3 = 0.0

        self._build_ui()

    def _build_ui(self) -> None:
        main = ttk.Frame(self.root, padding=6)
        main.pack(fill=tk.BOTH, expand=True)

        left = ttk.Frame(main)
        left.pack(side=tk.LEFT, fill=tk.Y, padx=(0, 6))
        right = ttk.Frame(main)
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # ----- Left: controls -----
        help_f = ttk.LabelFrame(left, text="What this does")
        help_f.pack(fill=tk.X, pady=(0, 8))
        help_text = (
            "Sliders = actual platform orientation (IMU input). Ref = [0,0,0].\n"
            "PID → remap → IK → T1,T2,T3. Right: ball + platform + wheels.\n"
            "Orange = τ. Purple = travel (∥ τ). Green = tilt. Remap so τ faces tilt.\n"
            "FRONT/BACK on platform. Axes: X right (Simulink), Y fwd, Z up."
        )
        ttk.Label(help_f, text=help_text, justify=tk.LEFT, padding=(4, 4)).pack(anchor=tk.W, fill=tk.X)

        arrow_f = ttk.LabelFrame(left, text="What the T1–T3 arrows mean")
        arrow_f.pack(fill=tk.X, pady=(0, 8))
        arrow_text = (
            "Each wheel = tilted disc (orientation) + curved arc (spin direction).\n"
            "• Disc shows wheel plane tilted α≈25.7°. Arc + arrow show which way it spins.\n"
            "• +T = one way, -T = opposite. Arc size ∝ |T|. No arc when T = 0.\n"
            "T1 red, T2 green, T3 blue @ 60°, 180°, 300°."
        )
        ttk.Label(arrow_f, text=arrow_text, justify=tk.LEFT, padding=(4, 4)).pack(anchor=tk.W, fill=tk.X)

        act_f = ttk.LabelFrame(left, text="Actual orientation [°] — sliders")
        act_f.pack(fill=tk.X, pady=(0, 6))

        self.actual_vars = []
        self.actual_labels = []
        for i, name in enumerate(["Roll", "Pitch", "Yaw"]):
            row = ttk.Frame(act_f)
            row.pack(fill=tk.X, pady=2)
            ttk.Label(row, text=f"{name}:", width=6, anchor=tk.W).pack(side=tk.LEFT)
            v = tk.DoubleVar(value=0.0)
            self.actual_vars.append(v)
            lb = ttk.Label(row, text="0.0", width=6)

            def _on_slide(val: str, idx: int = i, lbl: ttk.Label = lb):
                try:
                    x = float(val)
                    lbl.config(text=f"{x:.1f}")
                    self.actual_vars[idx].set(x)
                except Exception:
                    pass
                self._refresh_3d()

            s = ttk.Scale(
                row, from_=-DEG_RANGE, to=DEG_RANGE, variable=v,
                orient=tk.HORIZONTAL, length=160,
                command=_on_slide,
            )
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=4)
            lb.pack(side=tk.LEFT)
            self.actual_labels.append(lb)

        ref_f = ttk.Frame(left)
        ref_f.pack(fill=tk.X, pady=(0, 4))
        ttk.Label(ref_f, text="Reference (fixed):", anchor=tk.W).pack(side=tk.LEFT)
        ttk.Label(ref_f, text=" [0, 0, 0] °", font=("", 10, "bold")).pack(side=tk.LEFT, padx=4)

        pid_f = ttk.LabelFrame(left, text="PID (remapped) [N·m]")
        pid_f.pack(fill=tk.X, pady=(0, 6))
        pid_inner = ttk.Frame(pid_f)
        pid_inner.pack(fill=tk.X)
        remap_label = (
            f"Remap: swap={REMAP_SWAP}, roll_sign={REMAP_ROLL_SIGN}, pitch_sign={REMAP_PITCH_SIGN}"
        )
        ttk.Label(pid_f, text=remap_label, justify=tk.LEFT).pack(anchor=tk.W, pady=(2, 0))
        self.pid_labels = []
        for name in ["Roll_T", "Pitch_T", "Yaw_T"]:
            row = ttk.Frame(pid_inner)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=f"{name}:", width=8, anchor=tk.W).pack(side=tk.LEFT)
            lb = ttk.Label(row, text="0.00", width=10)
            lb.pack(side=tk.LEFT)
            self.pid_labels.append(lb)

        mot_f = ttk.LabelFrame(left, text="Motors [N·m] T1, T2, T3")
        mot_f.pack(fill=tk.X, pady=(0, 6))
        mot_inner = ttk.Frame(mot_f)
        mot_inner.pack(fill=tk.X)
        self.mot_labels = []
        for name in ["T1", "T2", "T3"]:
            row = ttk.Frame(mot_inner)
            row.pack(fill=tk.X, pady=1)
            ttk.Label(row, text=f"{name}:", width=6, anchor=tk.W).pack(side=tk.LEFT)
            lb = ttk.Label(row, text="0.00", width=10)
            lb.pack(side=tk.LEFT)
            self.mot_labels.append(lb)

        btn_f = ttk.Frame(left)
        btn_f.pack(fill=tk.X, pady=(8, 0))
        self.btn_start = ttk.Button(btn_f, text="Start", command=self._on_start)
        self.btn_start.pack(side=tk.LEFT, padx=2)
        self.btn_stop = ttk.Button(btn_f, text="Stop", command=self._on_stop, state=tk.DISABLED)
        self.btn_stop.pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_f, text="Reset", command=self._on_reset).pack(side=tk.LEFT, padx=2)

        # ----- Right: 3D -----
        fig = Figure(figsize=(5.5, 4.5), dpi=100)
        self.ax = fig.add_subplot(111, projection="3d")
        self.canvas = FigureCanvasTkAgg(fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._refresh_3d()

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

    def _get_actual_rad(self) -> np.ndarray:
        return np.array([deg2rad(self.actual_vars[i].get()) for i in range(3)])

    def _get_actual_deg(self) -> tuple[float, float, float]:
        return (
            float(self.actual_vars[0].get()),
            float(self.actual_vars[1].get()),
            float(self.actual_vars[2].get()),
        )

    def _refresh_3d(self) -> None:
        r, p, y = self._get_actual_deg()
        draw_platform_and_wheels(self.ax, r, p, y, self._last_t1, self._last_t2, self._last_t3, max_T=MAX_T)
        self.canvas.draw_idle()

    def _tick(self) -> None:
        if not self.running:
            return
        actual = self._get_actual_rad()
        tau = self.pid.step(REF, actual)
        tau = remap(tau)
        t1, t2, t3 = ik(tau[1], tau[0], tau[2], max_T=MAX_T)
        self._last_t1, self._last_t2, self._last_t3 = t1, t2, t3

        for i, lb in enumerate(self.pid_labels):
            lb.config(text=f"{tau[i]:.3f}")
        for i, lb in enumerate(self.mot_labels):
            lb.config(text=f"{[t1, t2, t3][i]:.3f}")

        self._refresh_3d()
        self.after_id = self.root.after(DT_MS, self._tick)

    def _on_start(self) -> None:
        self.running = True
        self.btn_start.config(state=tk.DISABLED)
        self.btn_stop.config(state=tk.NORMAL)
        if not hasattr(self, "_remap_logged"):
            print(
                f"[remap] swap={REMAP_SWAP} roll_sign={REMAP_ROLL_SIGN} pitch_sign={REMAP_PITCH_SIGN}"
            )
            self._remap_logged = True
        self._tick()

    def _on_stop(self) -> None:
        self.running = False
        if self.after_id:
            self.root.after_cancel(self.after_id)
            self.after_id = None
        self.btn_start.config(state=tk.NORMAL)
        self.btn_stop.config(state=tk.DISABLED)

    def _on_reset(self) -> None:
        was_running = self.running
        self._on_stop()
        for v in self.actual_vars:
            v.set(0.0)
        for lb in self.actual_labels:
            lb.config(text="0.0")
        self.pid.reset()
        self._last_t1 = self._last_t2 = self._last_t3 = 0.0
        for lb in self.pid_labels:
            lb.config(text="0.00")
        for lb in self.mot_labels:
            lb.config(text="0.00")
        self._refresh_3d()
        if was_running:
            self._on_start()

    def _on_close(self) -> None:
        self._on_stop()
        self.root.destroy()

    def run(self) -> None:
        self.root.mainloop()


def main() -> int:
    app = TiltControlUI()
    app.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
