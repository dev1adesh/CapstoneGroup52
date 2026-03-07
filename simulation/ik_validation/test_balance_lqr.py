#!/usr/bin/env python3
"""Headless test of LQR balance controller."""
import sys
from pathlib import Path

if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np
from physics_sim import PhysicsSim
from ik import ik
from lqr import compute_lqr

DT = 0.02
MAX_T = 8.0

def main():
    physics = PhysicsSim()
    K = compute_lqr()
    physics.set_orientation(0, np.radians(5), 0)  # pitch 5 deg
    for step in range(300):  # 6 seconds
        x = np.array([
            physics.roll, physics.pitch, physics.yaw,
            physics.omega[0], physics.omega[1], physics.omega[2]
        ])
        x_ref = np.array([0, 0, 0, 0, 0, 0])
        tau = -K @ (x - x_ref)
        t1, t2, t3 = ik(float(tau[1]), float(tau[0]), float(tau[2]), max_T=MAX_T)
        physics.step(t1, t2, t3, DT)
        if step % 50 == 0:
            r, p, y = np.degrees([physics.roll, physics.pitch, physics.yaw])
            print(f"  t={step*DT:.2f}s: roll={r:.2f} pitch={p:.2f} yaw={y:.2f} deg")
    r, p, y = np.degrees([physics.roll, physics.pitch, physics.yaw])
    print(f"\nFinal: roll={r:.2f} pitch={p:.2f} yaw={y:.2f} deg")
    ok = abs(p) < 5 and abs(r) < 10
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1

if __name__ == "__main__":
    sys.exit(main())
