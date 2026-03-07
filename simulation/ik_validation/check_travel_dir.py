#!/usr/bin/env python3
"""
Run PID → IK → FK for a fixed actual orientation and print travel direction.

Use this to check if "direction of travel" matches expectation. Convention:
roll = X (bank), pitch = Y (nose up/down), front = world -X.

Configure orientation via constants below or CLI (--roll, --pitch, --yaw in degrees).
Optional: --save <path> to write a 3D figure.

Example:
  python3 check_travel_dir.py
  python3 check_travel_dir.py --roll 10 --pitch 0 --yaw 0
  python3 check_travel_dir.py --pitch 5 --save /tmp/out.png
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from ik import ik, fk
from pid import PID3
from remap import remap, REMAP_PITCH_SIGN, REMAP_ROLL_SIGN, REMAP_SWAP
from viz_3d import (
    FLIP_X,
    compute_resultant_torque_direction,
    compute_tilt_direction,
    compute_travel_direction,
    draw_platform_and_wheels,
)

# --- Config (overridden by CLI) ---
ROLL_DEG = 10.0
PITCH_DEG = 0.0
YAW_DEG = 0.0

DT = 0.02
MAX_T = 2.0
REF = np.array([0.0, 0.0, 0.0])
KP, KI, KD = 50.0, 10.0, 5.0


def main() -> int:
    ap = argparse.ArgumentParser(description="Check travel direction for given orientation")
    ap.add_argument("--roll", type=float, default=ROLL_DEG, help="Roll [deg]")
    ap.add_argument("--pitch", type=float, default=PITCH_DEG, help="Pitch [deg]")
    ap.add_argument("--yaw", type=float, default=YAW_DEG, help="Yaw [deg]")
    ap.add_argument("--save", type=str, default="", help="Save 3D figure to path")
    args = ap.parse_args()

    roll_deg = args.roll
    pitch_deg = args.pitch
    yaw_deg = args.yaw

    actual_rad = np.array(
        [math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)]
    )
    pid = PID3(kp=KP, ki=KI, kd=KD, dt=DT, out_limit=MAX_T)
    tau = pid.step(REF, actual_rad)
    tau = remap(tau)
    t1, t2, t3 = ik(tau[1], tau[0], tau[2], max_T=MAX_T)
    tau_body = fk(t1, t2, t3)
    travel = compute_travel_direction(roll_deg, pitch_deg, yaw_deg, tau_body)
    tau_dir = compute_resultant_torque_direction(roll_deg, pitch_deg, yaw_deg, tau_body)
    tilt = compute_tilt_direction(roll_deg, pitch_deg, yaw_deg)

    print("=== Orientation → PID → IK → Tilt / τ / Force ===\n")
    print(f"  Actual [deg]     roll={roll_deg: .2f}  pitch={pitch_deg: .2f}  yaw={yaw_deg: .2f}")
    print(f"  PID out [N·m]    Roll_T={tau[0]: .4f}  Pitch_T={tau[1]: .4f}  Yaw_T={tau[2]: .4f}")
    print(f"  Motors [N·m]     T1={t1: .4f}  T2={t2: .4f}  T3={t3: .4f}")
    print()
    if tau_dir is not None:
        tx, ty, tz = tau_dir[0], tau_dir[1], tau_dir[2]
        print(f"  Resultant τ (unit) dx={tx: .6f}  dy={ty: .6f}  dz={tz: .6f}")
        if abs(tz) > 0.9:
            s = "+Z" if tz > 0 else "-Z"
            print(f"  → Mostly {s} (vertical)")
        elif abs(ty) > 0.9 and abs(tx) < 0.1:
            s = "+Y" if ty > 0 else "-Y"
            print(f"  → Mostly {s} (forward/back)")
        elif abs(tx) > 0.9 and abs(ty) < 0.1:
            s = "+X (right)" if tx > 0 else "-X (left)"
            print(f"  → Mostly {s}")
        else:
            print(f"  → Diagonal: {tx:+.2f} X, {ty:+.2f} Y, {tz:+.2f} Z")
    else:
        print("  Resultant τ (unit) [zero]")
    print()
    if travel is not None:
        dx, dy = travel[0], travel[1]
        print(f"  Force (unit)     dx={dx: .6f}  dy={dy: .6f}  [travel ∥ τ, same line]")
        print(f"  Frame: X = {'right (Simulink)' if FLIP_X else 'left'},  Y = forward")
        if abs(dx) < 0.01 and dy > 0.9:
            print("  → Mostly +Y (forward)")
        elif abs(dx) < 0.01 and dy < -0.9:
            print("  → Mostly -Y (backward)")
        elif dx > 0.9 and abs(dy) < 0.01:
            print("  → Mostly +X (right)")
        elif dx < -0.9 and abs(dy) < 0.01:
            print("  → Mostly -X (left)")
        else:
            print(f"  → Diagonal: {dx:+.2f} X, {dy:+.2f} Y")
    else:
        print("  Force (unit)     [zero horizontal τ]")
    print()

    # Tilt (green) = reference. Travel ∥ τ (same line); τ vs tilt, Force vs tilt optional.
    if tilt is not None:
        gx, gy = tilt[0], tilt[1]
        print(f"  Tilt (green)     dx={gx: .6f}  dy={gy: .6f}  [reference]")
        if abs(gx) < 0.01 and gy > 0.9:
            print("  → Mostly +Y (forward)")
        elif abs(gx) < 0.01 and gy < -0.9:
            print("  → Mostly -Y (backward)")
        elif gx > 0.9 and abs(gy) < 0.01:
            print("  → Mostly +X (right)")
        elif gx < -0.9 and abs(gy) < 0.01:
            print("  → Mostly -X (left)")
        else:
            print(f"  → Diagonal: {gx:+.2f} X, {gy:+.2f} Y")
        tau_xy = np.array([tau_dir[0], tau_dir[1]]) if tau_dir is not None else None
        n_tau = np.linalg.norm(tau_xy) if tau_xy is not None else 0.0
        if n_tau > 1e-9:
            tau_xy = tau_xy / n_tau
            dot_tau = float(np.dot(tau_xy, tilt))
            if abs(dot_tau) < 0.15:
                s_tau = "perpendicular"
            elif dot_tau > 0.9:
                s_tau = "same"
            elif dot_tau < -0.9:
                s_tau = "opposite"
            else:
                s_tau = "differ"
            print(f"  τ vs tilt:        {s_tau} (dot={dot_tau:+.3f})  [want same or opposite]")
        if travel is not None and n_tau > 1e-9:
            dot_tr = float(np.dot(travel, tau_xy))
            s_tr = "same line" if dot_tr > 0.9 else ("opposite" if dot_tr < -0.9 else "differ")
            print(f"  Travel vs τ:      {s_tr} (dot={dot_tr:+.3f})  [want same line]")
        if travel is not None:
            dot_f = float(np.dot(travel, tilt))
            s_f = "same" if dot_f > 0.9 else ("opposite" if dot_f < -0.9 else "differ")
            print(f"  Force vs tilt:    {s_f} (dot={dot_f:+.3f})")
        print(f"  Remap: swap={REMAP_SWAP} roll_sign={REMAP_ROLL_SIGN:.0f} pitch_sign={REMAP_PITCH_SIGN:.0f}")
    else:
        print("  Tilt (green)     [upright, no tilt]")

    if args.save:
        import matplotlib
        matplotlib.use("Agg")
        from matplotlib.figure import Figure
        fig = Figure(figsize=(6, 5), dpi=100)
        ax = fig.add_subplot(111, projection="3d")
        draw_platform_and_wheels(ax, roll_deg, pitch_deg, yaw_deg, t1, t2, t3, max_T=MAX_T)
        fig.savefig(args.save, bbox_inches="tight")
        print(f"\n  Saved figure → {args.save}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
