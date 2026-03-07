# Simulink Wiring: Controller → Plant

This document lists **blocks**, **settings/code**, and **wiring** to connect the validated **PID → Remap → IK** controller to your Simulink plant (ball, wheels, chassis, contact forces).

**Signal flow:** Transform Sensor (orientation) → Demux → Sum (ref − actual) → **3× PID** → **MATLAB Function (remap + IK + saturation)** → **Simulink–PS Converters** → Revolute Joint torque inputs (Wheels 1, 2, 3). **Remap** (swap Roll↔Pitch only), **IK**, **scale** (~20×), and **saturation** (±40 N·m) are all **inside** the MATLAB block; no Gain −1 blocks or cross-wiring.

---

## 1. Block diagram (ASCII)

```
  ORIENTATION (from plant)          REFERENCE           CONTROLLER                    TORQUES TO PLANT
 ──────────────────────────────────────────────────────────────────────────────────────────────────────────

  ┌─────────────────┐   ┌──────────────────┐   ┌─────────────┐
  │ Transform       │   │  PS-Simulink     │   │   Demux     │
  │ Sensor          │──►│  Converter (rad) │──►│ r   p   y   │
  │ (ZYX Euler)     │   └──────────────────┘   └──┬───┬───┬──┘
  └─────────────────┘                             │   │   │
  ┌─────────────┐                                 │   │   │
  │  Constant   │     ref [0;0;0] ────────────────┼───┼───┼───────► (+)
  │  [0; 0; 0]  │         (to each Sum)           │   │   │           │
  └─────────────┘                                 │   │   │           │
                    actual (−)        r     p     y   │   │           │
  Per axis: ref − actual               │     │     │   │   │           │
                ┌─────────┐  ┌─────────┐  ┌─────────┐   │   │           │
                │ Sum     │  │ Sum     │  │ Sum     │◄──┴───┴───────────┘
                │ ref−r   │  │ ref−p   │  │ ref−y   │  (each Sum: + ref, − actual)
                └────┬────┘  └────┬────┘  └────┬────┘
                     ▲            ▲            ▲
                     │            │            │
                    r            p            y   ← from Demux
                     │            │            │
                     ▼            ▼            ▼
                ┌─────────┐  ┌─────────┐  ┌─────────┐
                │ PID     │  │ PID     │  │ PID     │
                │ (roll)  │  │ (pitch) │  │ (yaw)   │
                └────┬────┘  └────┬────┘  └────┬────┘
                     │            │            │
                     │  Roll_T    │  Pitch_T   │  Yaw_T
                     │            │            │
                     └────────────┴────────────┴─────────────┐
                                  │            │             │
                                  ▼            ▼             ▼
                           ┌─────────────────────────────────────┐
                           │  MATLAB Function (remap + IK + sat)  │
                           │  in: Roll_T, Pitch_T, Yaw_T          │
                           │  out: T1, T2, T3 (~20–40, ±40 N·m)   │
                           └────────────────┬────────────────────┘
                                            │
                    ┌───────────────────────┼───────────────────────┐
                    │                       │                       │
                    ▼                       ▼                       ▼
             ┌─────────────┐         ┌─────────────┐         ┌─────────────┐
             │ Simulink–PS │         │ Simulink–PS │         │ Simulink–PS │
             │ (N·m)       │         │ (N·m)       │         │ (N·m)       │
             └──────┬──────┘         └──────┬──────┘         └──────┬──────┘
                    │                       │                       │
                    ▼                       ▼                       ▼
             ┌─────────────┐         ┌─────────────┐         ┌─────────────┐
             │ Revolute    │         │ Revolute    │         │ Revolute    │
             │ Joint       │         │ Joint       │         │ Joint       │
             │ Wheel 1     │         │ Wheel 2     │         │ Wheel 3     │
             └─────────────┘         └─────────────┘         └─────────────┘
                  T1                      T2                      T3
```

**Legend:** Sensor → PS-Simulink → Demux gives actual [r,p,y]. Constant [0;0;0] is ref. **Sum** = ref − actual per axis → **PID** → **MATLAB (remap + IK + sat)** → **Simulink–PS** → **Revolute Joint** torque (W1, W2, W3). Remap (swap only) is inside the MATLAB block.

---

## 2. Blocks and roles

| Block                         | Role                                                                                                                               |
| ----------------------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| **Transform Sensor**          | Measures orientation (World → Chassis). Output: ZYX Euler [rad].                                                                   |
| **PS-Simulink Converter**     | Physical Signal → Simulink. **Units: rad.**                                                                                        |
| **Demux** (1→3)               | Split [roll, pitch, yaw] for PID.                                                                                                  |
| **Constant**                  | Reference [0; 0; 0] (rad).                                                                                                         |
| **Sum**                       | `ref − actual` (ideal: `+` on ref, `−` on actual).                                                                                 |
| **PID** × 3                   | One per axis (roll, pitch, yaw). Output body torques.                                                                              |
| **MATLAB Function**           | **Remap + IK + saturation**: `(Roll_T, Pitch_T, Yaw_T)` → `[T1, T2, T3]`. Remap (swap only) inside; scale ~20×; clamp **±40** N·m. |
| **Simulink–PS Converter** × 3 | Torque to Physical Signal. **Units: N·m.**                                                                                         |
| **Revolute Joint** × 3        | Wheel joints. **Actuation → Torque → Provided by Input.**                                                                          |

---

## 3. Block settings

### Transform Sensor

- **Measurement**: Rotation (Euler).
- **Euler sequence**: **ZYX**.
- **Output**: Euler angles as **Physical Signal** (rad).

### PS-Simulink Converter (sensor)

- **Units**: **rad** (match Python / PID).
- **Output**: 3-element vector `[roll; pitch; yaw]`.

### Demux

- **Number of outputs**: **3**.
- **Output 1** → roll, **Output 2** → pitch, **Output 3** → yaw.

### Constant (reference)

- **Value**: `[0; 0; 0]` (rad). Same size as actual or match your Sum input format.

### Sum

- **List of signs**: `+--` (ref − roll − pitch − yaw) if you use one Sum per axis; or one Sum with ref − actual vector.

### PID × 3 (roll, pitch, yaw)

Use **three separate PID** blocks (roll, pitch, yaw). Same gains for all three. These match the Python validation (`tilt_control_ui`, `check_travel_dir`).

| Parameter                  | Value        | Notes                                                     |
| -------------------------- | ------------ | --------------------------------------------------------- |
| **Proportional (P)**       | **50**       |                                                           |
| **Integral (I)**           | **10**       |                                                           |
| **Derivative (D)**         | **5**        |                                                           |
| **Filter coefficient (N)** | **100**      | For derivative filter; tune if needed.                    |
| **Sample time**            | **0.02** s   | Match fixed-step solver / model rate.                     |
| **Form**                   | **Parallel** | u = Kp·e + Ki·∫e dt + Kd·(de/dt)                          |
| **Output limits**          | _(optional)_ | IK saturates ±40 N·m; PID limits can match or be omitted. |

- **Controller**: **PID**.
- **Form**: **Parallel**.
- **Input**: ref − actual (error) per axis, in **rad**.
- **Output**: body torque (N·m) per axis → **MATLAB (remap + IK)**. Wire **PID roll → Roll_T**, **PID pitch → Pitch_T**, **PID yaw → Yaw_T**.

### MATLAB Function (remap + IK + saturation)

- **Inputs**: `Roll_T`, `Pitch_T`, `Yaw_T` (each scalar) — **direct from PID**.
- **Outputs**: `T1`, `T2`, `T3` (each scalar), scaled and saturated to **±40** N·m **inside** the function.
- **Remap** (inside): **swap** Roll↔Pitch; **no** −1 on roll/pitch (switches pos/neg vs previous tuning) so Simulink roll/pitch directions match. Yaw unchanged.
- **Scale**: ×20 → **~20–40** N·m. **Sign flip** on motor torques so plant “positive” matches controller.

**Code** (paste into the MATLAB Function block):

```matlab
function [T1, T2, T3] = fcn(Roll_T, Pitch_T, Yaw_T)
    % Remap + IK + saturation. Wheel 2 @ 180° → Roll; Wheels 1 & 3 → Pitch.
    % Remap: swap Roll↔Pitch only (switch pos/neg for Simulink). Scale ~20×, sat ±MaxT, sign flip for plant.
    alpha = 25.659;
    ca = cosd(alpha);
    sa = sind(alpha);

    Scale = 20;   % output torques ~20–40 N·m
    MaxT = 40;    % N·m saturation per motor

    % Remap: swap Roll↔Pitch only (switch pos/neg vs earlier -1 on both)
    r_orig = Roll_T;
    p_orig = Pitch_T;
    Roll_T_map  = p_orig;    % swap: use Pitch as Roll
    Pitch_T_map = r_orig;    % swap: use Roll as Pitch
    Yaw_T_map   = Yaw_T;

    % IK (uses mapped body torques), then scale
    t1 = Scale * ((0.5*ca*Roll_T_map) + (0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));
    t2 = Scale * ((-1.0*ca*Roll_T_map) + (0*Pitch_T_map)       + (sa*Yaw_T_map));
    t3 = Scale * ((0.5*ca*Roll_T_map) + (-0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));

    % Saturation (±MaxT), then sign flip for plant
    T1 = -max(min(t1, MaxT), -MaxT);
    T2 = -max(min(t2, MaxT), -MaxT);
    T3 = -max(min(t3, MaxT), -MaxT);
end
```

#### Recommended Scale and MaxT (from physical parameters)

For **chassis 20 kg**, **ball 13 kg, 0.6 m diameter**, **wheels 0.5 kg each, 14 cm diameter**, platform on ball → **Scale = 2**, **MaxT = 15** N·m. The Python `physics_sim` uses these masses; `IK_SCALE_RECOMMENDED` and `IK_MAX_T_RECOMMENDED` match.

**MATLAB code with recommended Scale/MaxT** (remap version; for no-remap, set `Roll_T_map = Roll_T`, `Pitch_T_map = Pitch_T`, `Yaw_T_map = Yaw_T`):

```matlab
function [T1, T2, T3] = fcn(Roll_T, Pitch_T, Yaw_T)
    % Remap + IK + saturation. Chassis 20 kg, ball 13 kg 0.6 m, wheels 0.5 kg 14 cm.
    % Scale = 2, MaxT = 15 N·m. No PID output limits needed.
    alpha = 25.659;
    ca = cosd(alpha);
    sa = sind(alpha);

    Scale = 2;
    MaxT = 15;

    Roll_T_map  = Pitch_T;   % swap (remap)
    Pitch_T_map = Roll_T;
    Yaw_T_map   = Yaw_T;

    t1 = Scale * ((0.5*ca*Roll_T_map) + (0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));
    t2 = Scale * ((-1.0*ca*Roll_T_map) + (0*Pitch_T_map)       + (sa*Yaw_T_map));
    t3 = Scale * ((0.5*ca*Roll_T_map) + (-0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));

    T1 = -max(min(t1, MaxT), -MaxT);
    T2 = -max(min(t2, MaxT), -MaxT);
    T3 = -max(min(t3, MaxT), -MaxT);
end
```

#### Single-wheel-at-a-time (PWM-style; reduce fighting)

Only **one wheel** receives torque each step; the other two are zero. Switch either **round-robin** (1→2→3→1…) or **largest \|T\|** (activate the wheel with biggest commanded torque).

**Round-robin** — cycles 1→2→3 each step. Scale = 2, MaxT = 15. Remap (swap) as above; set `Roll_T_map = Roll_T` etc. for no-remap.

```matlab
function [T1, T2, T3] = fcn(Roll_T, Pitch_T, Yaw_T)
    % Single-wheel-at-a-time, round-robin (PWM-style). Scale=2, MaxT=15. Remap: swap.
    persistent k
    if isempty(k), k = uint32(0); end
    k = k + 1;
    idx = mod(k - 1, 3) + 1;   % 1, 2, 3

    alpha = 25.659;
    ca = cosd(alpha);
    sa = sind(alpha);
    Scale = 2;
    MaxT = 15;

    Roll_T_map  = Pitch_T;
    Pitch_T_map = Roll_T;
    Yaw_T_map   = Yaw_T;

    t1 = Scale * ((0.5*ca*Roll_T_map) + (0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));
    t2 = Scale * ((-1.0*ca*Roll_T_map) + (0*Pitch_T_map)       + (sa*Yaw_T_map));
    t3 = Scale * ((0.5*ca*Roll_T_map) + (-0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));

    t1 = -max(min(t1, MaxT), -MaxT);
    t2 = -max(min(t2, MaxT), -MaxT);
    t3 = -max(min(t3, MaxT), -MaxT);

    T1 = 0; T2 = 0; T3 = 0;
    if idx == 1
        T1 = t1;
    elseif idx == 2
        T2 = t2;
    else
        T3 = t3;
    end
end
```

**Largest \|T\|** — no `persistent` state. Activate only the wheel with largest \|t_i\|.

```matlab
function [T1, T2, T3] = fcn(Roll_T, Pitch_T, Yaw_T)
    % Single-wheel-at-a-time: only largest |T|. Scale=2, MaxT=15. Remap: swap.
    alpha = 25.659;
    ca = cosd(alpha);
    sa = sind(alpha);
    Scale = 2;
    MaxT = 15;

    Roll_T_map  = Pitch_T;
    Pitch_T_map = Roll_T;
    Yaw_T_map   = Yaw_T;

    t1 = Scale * ((0.5*ca*Roll_T_map) + (0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));
    t2 = Scale * ((-1.0*ca*Roll_T_map) + (0*Pitch_T_map)       + (sa*Yaw_T_map));
    t3 = Scale * ((0.5*ca*Roll_T_map) + (-0.866*ca*Pitch_T_map) + (sa*Yaw_T_map));

    t1 = -max(min(t1, MaxT), -MaxT);
    t2 = -max(min(t2, MaxT), -MaxT);
    t3 = -max(min(t3, MaxT), -MaxT);

    [~, idx] = max([abs(t1), abs(t2), abs(t3)]);
    T1 = 0; T2 = 0; T3 = 0;
    if idx == 1
        T1 = t1;
    elseif idx == 2
        T2 = t2;
    else
        T3 = t3;
    end
end
```

Use the same **sample time** for the MATLAB block as your PID (e.g. 0.02 s) so round-robin cycles evenly. For no-remap, set `Roll_T_map = Roll_T`, `Pitch_T_map = Pitch_T`, `Yaw_T_map = Yaw_T` in either version.

### Simulink–PS Converter × 3 (torques)

- **Units**: **N·m**.
- **Output**: Physical Signal → **Revolute Joint** **torque** input **`t`** (or **Actuation → Torque**).

### Revolute Joint × 3 (wheels)

- **Actuation**: **Torque**.
- **Source**: **Provided by Input**.
- **Torque input**: From corresponding Simulink–PS Converter.

---

## 4. Remap (inside MATLAB function)

**Remap** implements: **swap** Roll ↔ Pitch only (no −1 on roll/pitch). Yaw unchanged. This switches positive/negative vs the earlier “swap + −1” tuning so Simulink roll/pitch directions match. It runs **inside** the MATLAB Function block.

**Wiring:** Wire PID outputs **directly** to the MATLAB block:

- **PID Roll_T** → **Roll_T** input
- **PID Pitch_T** → **Pitch_T** input
- **PID Yaw_T** → **Yaw_T** input

The function computes `Roll_T_map = Pitch_T`, `Pitch_T_map = Roll_T`, `Yaw_T_map = Yaw_T`, then runs IK on the mapped values.

---

## 4.1 Balance sim vs Simulink — what’s different?

The **Python balance sim** (`balance_sim_ui.py`) uses a different controller setup so it can balance and show Segway‑like lean→motion. When taking that behavior **back to Simulink**, these are the differences:

|                     | **Balance sim (Python)**                                                                                   | **Simulink (this doc)**                                                                        |
| ------------------- | ---------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| **Remap**           | **None.** Roll error → roll torque, pitch → pitch. Axis‑aligned correction.                                | **Swap** Roll↔Pitch **inside** the MATLAB function.                                            |
| **PID**             | **P = 80**, **I = 12**, **D = 6**. Output limit **8** N·m per axis.                                        | **P = 50**, **I = 10**, **D = 5**. (Optional limits; IK saturates.)                            |
| **MATLAB function** | Not used. Python `ik()` only: same IK formulas, **no** remap, **no** scale, **no** sign flip, `max_T = 8`. | **Remap** (swap) + **IK** + **Scale 20** + **saturation ±40** N·m + **sign flip** on T1,T2,T3. |
| **Wiring**          | PID → `ik(tau[1], tau[0], tau[2])` (Pitch, Roll, Yaw into IK). No remap.                                   | PID Roll→Roll_T, Pitch→Pitch_T, Yaw→Yaw_T → MATLAB (remap+IK+sat) → T1,T2,T3.                  |

**Summary:**

- **PID:** Different gains and limit. Balance sim: **80, 12, 6**, limit **8**. Simulink: **50, 10, 5** (and IK ±40).
- **Wiring:** Same block flow (sensor → Demux → Sum → PID ×3 → MATLAB → T1,T2,T3 → plant). The difference is **whether remap is applied** (Simulink) or not (balance sim).
- **MATLAB function:** Simulink uses **remap + IK + scale + sat + sign flip**. Balance sim uses raw IK only (no remap, no scale, no flip).

**If you want Simulink to behave like the balance sim** (axis‑aligned correction, better chance to balance):

1. **Remove remap** in the MATLAB function: set `Roll_T_map = Roll_T`, `Pitch_T_map = Pitch_T`, `Yaw_T_map = Yaw_T` (no swap).
2. **Try balance‑sim PID gains:** P = 80, I = 12, D = 6. Keep PID output limits or let IK handle saturation.
3. **Keep** Scale, MaxT, and sign flip in MATLAB as‑is for your **plant** (they match your motors and joints). The balance sim’s ±8 N·m is for simplified physics; Simulink uses ±40 for the real rig.

If the plant **diverges or corrects the wrong way** after dropping remap, use **§6.2** (orientation sign / Gain −1 on actual) to fix correction direction instead of re‑adding remap.

---

## 5. End‑to‑end wiring summary

```
Transform Sensor (ZYX Euler, rad)
    → PS-Simulink Converter (rad)
    → Demux [roll, pitch, yaw]

Constant [0;0;0] ──┬─ Sum (ref − actual) ─→ PID (roll)  ──→ Roll_T  ──┐
                   ├─ Sum ─────────────────→ PID (pitch) ──→ Pitch_T ──┼→ MATLAB (remap+IK+sat) → T1,T2,T3
                   └─ Sum ─────────────────→ PID (yaw)   ──→ Yaw_T  ──┘

IK [T1, T2, T3] (scale ~20×, ±40 N·m inside IK) → Simulink–PS (N·m) → Revolute Joint t (Wheel 1, 2, 3)
```

- **Wheel 1** ← T1, **Wheel 2** ← T2, **Wheel 3** ← T3.
- **Wheel layout**: 1 @ 60°, 2 @ 180°, 3 @ 300° (front = gap between 1 and 3).

---

## 6. Get the ball bot balancing (checklist)

Goal: **reference [0, 0, 0]** → PID → remap + IK → wheel torques → plant balances upright. Do these in order.

### 6.1 Wiring and block setup

- **Ref** = `[0; 0; 0]` (rad). **Sum** = `ref − actual` per axis.
- **Transform Sensor** → ZYX Euler [rad] → **PS-Simulink** → **Demux** → actual [r, p, y].
- **PID** (×3) → **MATLAB (remap + IK + sat)** → **Simulink–PS** → **Revolute Joint** torque (W1, W2, W3). Use the MATLAB code from §3 (remap swap-only; optional `Yaw_T_map = -Yaw_T` if yaw direction is wrong).

### 6.2 Orientation sign (sensor)

If the bot **diverges** (tips over faster with control than without) or clearly **corrects the wrong way**:

- Add **Gain −1** on **roll** and/or **pitch** (and **yaw** if needed) **between Demux and Sum** (i.e. flip **actual** before `ref − actual`).
- So you feed **−actual** into the Sum instead of actual. That flips the sign of the error and thus the correction direction.

Try **roll only** first, then **pitch**, then **yaw** if required. Re-run after each change.

### 6.3 Wheel–ball friction (reduce fighting)

- In **wheel–ball** contact blocks, use **μs = 0.4**, **μk = 0.3** (allow slip).
- Keep **stiffness / damping** as-is (e.g. 1e6, 1e4).
- Ball–ground friction can stay as in your model (e.g. 0.7 / 0.5).

Lower μ reduces “wheels fighting” when commanded torques are kinematically inconsistent.

### 6.4 Solver and sample time

- **Fixed-step** solver, step size **0.02** s (or **0.001** s if contact is very stiff).
- **PID sample time** = **0.02** s.
- Ensure the **MATLAB Function** and any **Zero-Order Hold** (e.g. for filters) use the same step where applicable.

### 6.5 Torque scale

- MATLAB uses **Scale = 20**, **MaxT = 40** N·m.
- If the bot **oscillates violently** or the plant is **small / low inertia**: try **Scale = 10**, **MaxT = 20**.
- If it **barely reacts**: try **Scale = 30**, **MaxT = 50** (or slightly increase PID gains first).

### 6.6 PID gains

- Start with **P = 50**, **I = 10**, **D = 5** (parallel form).
- **Sluggish** → increase P (e.g. 80) or I. **Oscillatory / unstable** → decrease P or increase D; reduce integral if needed.

### 6.7 Geometry

- **Wheels**: 1 @ 60°, 2 @ 180°, 3 @ 300°; **α = 25.659°**.
- **Front** = gap between wheels 1 and 3.
- Confirm your Simulink layout matches the IK (wheel positions, tilt α). Mismatches cause strange behavior and fighting.

### 6.8 Initial condition

- Start **nearly upright**: e.g. **Euler [0, 0, 0]** or very small tilt (e.g. 1–2°).
- Avoid large initial tilt until the controller holds small tilt; then test robustness.

### 6.9 Quick debug

- **No control**: Set wheel torques to **0**. Bot should fall over. If not, check gravity and joint setup.
- **With control**: Apply small initial tilt (e.g. 2° roll). Log **orientation** and **T1, T2, T3**.
  - For **forward** tilt, you should see correction that pushes **back** (e.g. τ or wheel torques opposing the tilt).
  - If torques **reinforce** the tilt, flip **orientation sign** as in §6.2.

### 6.10 If it still doesn’t balance

- Recheck **sensor frame** (world vs body) and **Euler order** (ZYX).
- Verify **torque sign** at the Revolute Joints (we use a **sign flip** in MATLAB so “positive” controller torque matches the plant). If the plant convention differs, remove or invert that flip.
- Consider **velocity-based IK** (no-slip Jacobian → desired wheel speeds → inner torque loop) as a next step if torque-based control keeps causing fighting despite the above.

---

## 7. Optional checks

- **Units**: Sensor **rad** → PID; IK output **N·m** → PS **N·m** → Joint torque.
- **Fixed-step solver**: e.g. **0.002–0.02** s; match PID sample time (**0.02** s).
- **Revolute Joint**: **Second-Order Filtering** on the torque input if you see “motion trajectories not achievable” or similar.
- **PID tuning**: **P=50, I=10, D=5** are the validated defaults. Use as a starting point; retune on the Simulink plant if response is sluggish, oscillatory, or unstable.

---

## 8. Wheel–sphere friction (contact)

The wheels contact the main ball via **Sphere–Sphere** (or **Sphere–Solid**) force blocks. If friction is too high (e.g. μs = 0.8, μk = 0.6), the model approximates **no-slip**. When wheel torques conflict (IK remap, geometry, or solver effects), the contact tries to enforce pure rolling and you can get fighting forces, wheels “spinning” against the ball, or stiff contact issues. Allowing **some slip** by **lowering** wheel–sphere friction avoids that and is often more realistic.

**Suggested values (allow a little slip):**

| Parameter               | Value                     | Notes                                                           |
| ----------------------- | ------------------------- | --------------------------------------------------------------- |
| **Static friction μs**  | **0.4**                   | Lower than rubber–rubber (~0.8); slip when demand exceeds grip. |
| **Kinetic friction μk** | **0.3**                   | Slightly below μs.                                              |
| **Friction law**        | **Stick-Slip Continuous** | Standard for contact forces.                                    |
| **Velocity threshold**  | **0.001** m/s             | Or 0.002–0.005 if you want slip to engage a bit earlier.        |

- Use these in the **wheel–ball** contact force block(s) only. Ball–ground friction is separate.
- **Stiffness / damping** (e.g. 1e6 N/m, 1e4 N·s/m): keep as-is; slip comes from **μ**, not from softening contact.
- If the ball still slips too much or too little, try **μs = 0.35, μk = 0.25** (more slip) or **μs = 0.5, μk = 0.4** (less slip).
- If you use `init_omni_chair.m`, set wheel–ball contact to **`contact.mu_static_slip`** and **`contact.mu_kinetic_slip`** (0.4 / 0.3) instead of `contact.mu_static` / `contact.mu_kinetic`.

---

## 9. Debugging: 6-DOF Total Torque (Composite Force/Torque Sensing)

You can use **Total Torque** from the 6-DOF joint’s **Composite Force/Torque Sensing** to inspect the resultant torque on the ball. The block outputs a **3‑element vector** `[τx, τy, τz]` (N·m). After a **Demux (1→3)** you get:

| Output | Component | Meaning                                                          |
| ------ | --------- | ---------------------------------------------------------------- |
| **1**  | **τx**    | Torque about the **X** axis (world frame, usually: X = right).   |
| **2**  | **τy**    | Torque about the **Y** axis (world frame, usually: Y = forward). |
| **3**  | **τz**    | Torque about the **Z** axis (world frame, usually: Z = up).      |

The frame is defined by the **joint’s base** (commonly world). Check the 6-DOF block help to confirm X/Y/Z.

**Why it’s noisy**

Contact forces (wheel–ball, ball–ground) change rapidly with small penetrations and stick–slip. The joint measures **total** torque, so that high‑frequency content shows up as **noise**.

**Reducing noise**

**Low‑pass filter** each of the three torque signals (τx, τy, τz) before scoping or logging. Use one filter block per branch after the Demux.

**Low-Pass Filter block settings**

| Parameter            | Value           | Notes                                                                   |
| -------------------- | --------------- | ----------------------------------------------------------------------- |
| **Cutoff frequency** | **10** Hz       | If the block uses **rad/s** instead of Hz, use **2π×10 ≈ 62.83** rad/s. |
| **Filter order**     | **2**           | 1st order = gentler; 2nd = good trade-off.                              |
| **Sample time**      | **0.02** s      | Match your fixed step (or **–1** to inherit from upstream).             |
| **Filter type**      | **Butterworth** | If the block has a design-method option.                                |

- **Block location**: **Simulink** → **DSP System Toolbox** → **Filtering** → **Lowpass**, or **Simulink Extras** → **Filtering** → **Low-Pass Filter** (dialogs differ slightly).
- **Cutoff**: 10 Hz keeps dynamics below your ~50 Hz control rate and removes most contact chatter. Use **5–20 Hz** if you need more smoothing (5) or less lag (20).

**Suggested wiring**

The DSP **Lowpass Filter** block requires **discrete** inputs. The 6-DOF torque is often **continuous**. Insert a **Zero-Order Hold** to sample it before the filter:

```
6-DOF Joint (Total Torque) → PS-Simulink → Zero-Order Hold (Ts=0.02) → Demux (1→3)
                                                      → [τx] → Low-Pass Filter → Scope 1
                                                      → [τy] → Low-Pass Filter → Scope 2
                                                      → [τz] → Low-Pass Filter → Scope 3
```

- **Zero-Order Hold**: **Simulink → Discrete → Zero-Order Hold**. Set **Sample time** = **0.02** (50 Hz). Place it **after** PS-Simulink and **before** the Demux.
- If **Total Torque** is already a discrete Simulink signal, you can omit the ZOH.

**"Continuous-time inputs are not supported"**

This means the filter is receiving a continuous signal. Add the **Zero-Order Hold** (Ts = 0.02) **before** the filter (or before the Demux). The filter then sees a discrete signal and runs correctly.

This matches the Python **remap** (swap only) and **IK** in `ik.py`, and connects the controller to your plant as intended.
