# Ball-Bike Inverse Kinematics Validation

Python scripts to check that the 3-wheel ball-bike IK (Simulink MATLAB Function block) is correct, and to test **PID → IK → motor torques** with a tilt-control UI.

**Requirements:** `numpy`, `matplotlib`, `scipy` (and `tkinter`, usually bundled with Python)

```bash
pip install numpy matplotlib scipy
```

---

## 1. IK validation

```bash
python3 simulation/ik_validation/validate_ik.py
```

- Exit code **0** = all tests passed, IK appears correct.
- Exit code **1** = at least one test failed.

Tests: round-trip (IK→FK), pure pitch/roll/yaw, `ik()` vs matrix, saturation, random round-trips, matrix conditioning.

---

## 2. Tilt control UI (PID → IK → T1, T2, T3)

```bash
python3 simulation/ik_validation/tilt_control_ui.py
```

Replicates the Simulink flow: **actual** orientation (from sliders, like IMU) and **reference [0, 0, 0]** (fixed) → **PID** → **IK** → **T1, T2, T3**. No simulated plant.

- **Sliders** = actual platform orientation [°] (input to PID).
- **Reference** = fixed [0, 0, 0].
- **PID** output → **remap** (swap/sign flips on Roll_T, Pitch_T) → **IK** → motor torques **T1, T2, T3** (same as Simulink MATLAB function).

**Start** runs the loop; **Stop** pauses; **Reset** zeros sliders and PID. Move sliders while running to see how torques respond for a given "measured" tilt.

**3D view:** Ball below platform, wheels. **Orange** = τ (resultant torque on ball). **Purple** = travel (∥ τ). **Green** = tilt. Remap tuned so **τ faces tilt** (same direction). Travel ∥ τ.

**Arrows:** Green = tilt. Orange = τ. Purple = travel. Check script: τ vs tilt → same/opposite; Travel vs τ → same line.

**Simulink:** Remap = **swap** Roll↔Pitch plus gain **-1** on both before IK (`swap=True`, `roll_sign=-1`, `pitch_sign=-1`).

---

## 3. Check travel direction (batch)

```bash
python3 simulation/ik_validation/check_travel_dir.py [--roll 10] [--pitch 0] [--yaw 0] [--save path.png]
```

Runs **PID → remap → IK → FK** for a **fixed** orientation. Prints actual, τ, T1–T3, **tilt**, **travel**, **τ vs tilt** (want same/opposite), **Travel vs τ** (want same line), and remap config. Optional `--save` writes a 3D figure.

---

## 4. Balance sim (physics)

```bash
python3 simulation/ik_validation/balance_sim_ui.py
```

**Initial** and **desired** orientation sliders; **Start** runs a closed-loop physics simulation that tries to drive the platform from initial to desired orientation.

- **Initial orientation [°]**: Set before **Start**. Physics state is initialized to these roll, pitch, yaw values.
- **Desired orientation [°]**: LQR reference (typically `[0, 0, 0]` for upright).
- **Start**: Initialize physics from initial sliders, then run **LQR → IK → physics** at 50 Hz. Motors apply torques; wheels spin; ball moves; platform orientation updates. A second window shows Simulink-style scope plots (TorqueInput, Roll-Pitch-Yaw, PositionOutput, AngRoll-AngPitch-AngYaw, RollT-PitchT-YawT) for comparison. Use **Lean [°]** sliders during the run to add a person’s weight disturbance and steer.
- **Stop** / **Reset**: Stop the loop; **Reset** also zeros sliders and physics state.

**State panel:** Current orientation [°], wheel angles θ₁, θ₂, θ₃ [°], ball position (x, y) [m], and motor torques T1, T2, T3 [N·m].

**3D view:** Platform, wheels (with rotation marker showing θ), ball, ground, τ and travel arrows. **Lean [°]** sliders (roll, pitch) add a person’s weight as a disturbance: a simple figure and **blue lean arrow** show lean direction. Adjust lean during a run to steer the platform; the controller counteracts it so the system tends to move that way.

---

## 5. Simulink / MATLAB – what changed and what you need to do

**Setup** – Same overall flow: **orientation → PID → (remap) → IK → T1, T2, T3**. The only addition is the **remap** block between PID and IK.

**What changed:**

| Item          | Change                                                                                                     |
| ------------- | ---------------------------------------------------------------------------------------------------------- |
| **IK matrix** | Updated so Wheel 2 (back) → Roll, Wheels 1 & 3 → Pitch. Pure Roll ⇒ T2 active; Pure Pitch ⇒ T1, T3 active. |
| **Remap**     | New block **after** PID, **before** IK: swap Roll↔Pitch, then multiply **both** by **-1**. Yaw unchanged.  |
| **Travel**    | Viz only: travel = horizontal τ (same line). No Simulink change.                                           |

**Do you need to change the MATLAB IK function?** **Yes**, if it still uses the old matrix.

- Update the IK **matrix / formulas** to match [ik.py](simulation/ik_validation/ik.py). Use:
  - `CA = cos(25.659°)`, `SA = sin(25.659°)`
  - `t1 = 0.5*CA*Roll_T + 0.866*CA*Pitch_T + SA*Yaw_T`
  - `t2 = -1.0*CA*Roll_T + 0*Pitch_T + SA*Yaw_T`
  - `t3 = 0.5*CA*Roll_T - 0.866*CA*Pitch_T + SA*Yaw_T`
  - Then saturate each to `[-max_T, max_T]` (e.g. 2.0).
- **Inputs:** `(Pitch_T, Roll_T, Yaw_T)` in that order, or `(Roll_T, Pitch_T, Yaw_T)` and swap internally to match the formulas above (Roll_T and Pitch_T are the **remapped** values).

**Remap in Simulink (between PID and IK):**

1. **Swap** Roll_T and Pitch_T (cross wires).
2. **Gain -1** on the signal that goes to **Roll_T** input of IK.
3. **Gain -1** on the signal that goes to **Pitch_T** input of IK.
4. Yaw_T → IK unchanged.

Result: same behaviour as Python `remap` with `swap=True`, `roll_sign=-1`, `pitch_sign=-1`.

**Full Simulink wiring:** See **[SIMULINK_WIRING.md](SIMULINK_WIRING.md)** for block list, settings (PID, saturation, units), MATLAB IK code, remap wiring, and connection diagram.
