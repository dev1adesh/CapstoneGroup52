#!/usr/bin/env python3
"""
Ball-Bike Inverse Kinematics Validation

Run:  python validate_ik.py   (from this directory)
  or  python simulation/ik_validation/validate_ik.py   (from project root)

Verifies the 3-wheel IK: round-trip recovery, pure-axis behavior, saturation.
Exits with code 0 if all checks pass, 1 otherwise.
"""

from __future__ import annotations

import sys
from pathlib import Path

# Ensure we can import ik when run from project root
_here = Path(__file__).resolve().parent
if str(_here) not in sys.path:
    sys.path.insert(0, str(_here))

import numpy as np
from ik import A, ik, ALPHA_DEG, CA, SA

MaxT = 2.0
A_inv = np.linalg.inv(A)
tol = 1e-10


def main() -> int:
    nfail = 0

    print("========== Ball-Bike IK Validation ==========\n")

    # ----- Test 1: Round-trip (no saturation) -----
    print("--- Test 1: Round-trip (IK -> FK, no saturation) ---")
    R, P, Y = 0.5, 0.3, 0.2
    rpy = np.array([R, P, Y])
    T = A @ rpy
    recovered = A_inv @ T
    err = np.abs(rpy - recovered)
    print(f"  Input    [Roll, Pitch, Yaw] = [{R:.4f}, {P:.4f}, {Y:.4f}]")
    print(f"  Recovered                   = [{recovered[0]:.4f}, {recovered[1]:.4f}, {recovered[2]:.4f}]")
    print(f"  Max error = {err.max():.2e}")
    if err.max() < tol:
        print(f"  PASS: Round-trip error < {tol:.0e}")
    else:
        print("  FAIL: Round-trip error too large.")
        nfail += 1

    # ----- Test 2: Pure Pitch, Roll, Yaw -----
    print("\n--- Test 2: Pure-axis commands ---")
    pure_pitch = A @ np.array([0, 1, 0])
    pure_roll = A @ np.array([1, 0, 0])
    pure_yaw = A @ np.array([0, 0, 1])

    print(f"  Pure Pitch [0,1,0]: T1={pure_pitch[0]:.4f}  T2={pure_pitch[1]:.4f}  T3={pure_pitch[2]:.4f}")
    t1_neg_t3 = np.abs(pure_pitch[0] + pure_pitch[2]) < tol
    t2_zero = np.abs(pure_pitch[1]) < tol
    print(f"    -> Expect T1 = -T3, T2 = 0 (Wheels 1 & 3). T1=-T3? {'yes' if t1_neg_t3 else 'no'}  T2=0? {'yes' if t2_zero else 'no'}")
    if not (t1_neg_t3 and t2_zero):
        nfail += 1

    print(f"  Pure Roll  [1,0,0]: T1={pure_roll[0]:.4f}  T2={pure_roll[1]:.4f}  T3={pure_roll[2]:.4f}")
    t1_eq_t3 = np.abs(pure_roll[0] - pure_roll[2]) < tol
    t2_dominant = np.abs(pure_roll[1]) >= np.abs(pure_roll[0]) * 0.9
    print(f"    -> Expect T1 = T3, T2 dominant (Wheel 2 back). T1=T3? {'yes' if t1_eq_t3 else 'no'}  T2 dominant? {'yes' if t2_dominant else 'no'}")
    if not (t1_eq_t3 and t2_dominant):
        nfail += 1

    print(f"  Pure Yaw   [0,0,1]: T1={pure_yaw[0]:.4f}  T2={pure_yaw[1]:.4f}  T3={pure_yaw[2]:.4f}")
    yaw_equal = np.all(np.abs(np.diff(pure_yaw)) < tol)
    print(f"    -> Expect T1 = T2 = T3. Equal? {'yes' if yaw_equal else 'no'}")
    if not yaw_equal:
        nfail += 1

    # ----- Test 3: ik() vs matrix (no saturation) -----
    print("\n--- Test 3: ik() vs matrix (no saturation) ---")
    R3, P3, Y3 = 0.4, -0.2, 0.1
    T_mat = A @ np.array([R3, P3, Y3])
    t1, t2, t3 = ik(P3, R3, Y3, max_T=999.0)
    T_fcn = np.array([t1, t2, t3])
    print(f"  [Roll,Pitch,Yaw] = [{R3:.2f}, {P3:.2f}, {Y3:.2f}]")
    print(f"  Matrix T: [{T_mat[0]:.4f}, {T_mat[1]:.4f}, {T_mat[2]:.4f}]")
    print(f"  ik() T:   [{T_fcn[0]:.4f}, {T_fcn[1]:.4f}, {T_fcn[2]:.4f}]")
    df = np.abs(T_mat - T_fcn).max()
    if df < tol:
        print("  PASS: ik() matches matrix.")
    else:
        print(f"  FAIL: ik() differs from matrix (max diff = {df:.2e}).")
        nfail += 1

    # ----- Test 4: Saturation -----
    print(f"\n--- Test 4: Saturation (MaxT = {MaxT:.1f} N·m) ---")
    big = np.array([10.0, 10.0, 10.0])
    T_big = A @ big
    t1s, t2s, t3s = ik(big[1], big[0], big[2], max_T=MaxT)
    T_sat = np.array([t1s, t2s, t3s])
    print(f"  Large [R,P,Y] = [10, 10, 10]")
    print(f"  Before sat: [{T_big[0]:.2f}, {T_big[1]:.2f}, {T_big[2]:.2f}]")
    print(f"  After sat:  [{T_sat[0]:.2f}, {T_sat[1]:.2f}, {T_sat[2]:.2f}]")
    within = np.all(np.abs(T_sat) <= MaxT + 1e-9)
    print(f"  All within [-{MaxT:.1f}, {MaxT:.1f}]? {'yes' if within else 'no'}")
    if not within:
        nfail += 1

    # ----- Test 5: Random round-trips (no saturation) -----
    print("\n--- Test 5: Random round-trips (50 samples, no saturation) ---")
    rng = np.random.default_rng(42)
    fail_count = 0
    for _ in range(50):
        rpy = 2 * rng.random(3) - 1
        T = A @ rpy
        rpy_rec = A_inv @ T
        if np.abs(rpy - rpy_rec).max() >= tol:
            fail_count += 1
    if fail_count == 0:
        print("  PASS: All 50 round-trips recovered within 1e-10.")
    else:
        print(f"  FAIL: {fail_count} round-trips had error >= 1e-10.")
        nfail += 1

    # ----- Test 6: Condition number -----
    print("\n--- Test 6: IK matrix conditioning ---")
    c = np.linalg.cond(A)
    print(f"  cond(A) = {c:.2f}")
    if c < 1e3:
        print("  PASS: Well-conditioned (cond < 1000).")
    else:
        print("  WARN: cond >= 1000; numerical sensitivity possible.")

    print("\n========== All validation checks completed ==========")
    if nfail > 0:
        print(f"\n>>> {nfail} test(s) FAILED. IK may be incorrect.")
    else:
        print("\n>>> All tests PASSED. IK appears correct.")
    return 1 if nfail > 0 else 0


if __name__ == "__main__":
    sys.exit(main())
