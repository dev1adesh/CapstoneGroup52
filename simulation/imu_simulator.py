#!/usr/bin/env python3
"""
Ball-Wheelchair IMU Response Simulator
=======================================
Mirrors control_helper.cpp PD + IK logic exactly.
Adjust Roll / Pitch / Yaw sliders to see:
  - Per-wheel velocity commands (bar chart + arrows)
  - Predicted ball/platform trajectory over 5 seconds
  - Top-down platform view with velocity arrows

Run:  python3 simulation/imu_simulator.py
"""
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider
from matplotlib.patches import Circle, FancyArrowPatch
import matplotlib.patheffects as pe

# ============================================================
# Parameters — keep in sync with control_helper.cpp
# ============================================================
Kp            = 5.0      # roll/pitch proportional gain
Kd            = 1.0      # (omega = 0 in this sim, Kd not active)
Kp_yaw        = 2.0
IK_MAX_VEL    = 1.5      # per-wheel velocity clamp  [rad/s]
VEL_MAX_LQR   = 1.5
ALPHA_RAD     = np.radians(25.659)
CA            = np.cos(ALPHA_RAD)
SA            = np.sin(ALPHA_RAD)
# Physical layout: M2(front)=90°, M1(bottom-left)=210°, M3(bottom-right)=330°
WHEEL_ANGLES  = np.radians([210.0, 90.0, 330.0])   # node 0 (M1), node 1 (M2), node 2 (M3)
MOTOR_SIGNS   = np.array([1.0, -1.0, 1.0])
LQR_REMAP_SWAP = False   # disabled — wheel angles now match physical layout
INVERT_ROLL    = True

WHEEL_COLORS = ['#e74c3c', '#2ecc71', '#3498db']
WHEEL_NAMES  = ['M1 node0\n(210°, BL)', 'M2 node1\n(90°, Front)', 'M3 node2\n(330°, BR)']

# ============================================================
# Control math  (exact translation of Arduino code)
# ============================================================
def compute(roll_deg, pitch_deg, yaw_deg):
    """Returns (wheel_velocities[3], ball_dx, ball_dy)"""
    roll  = np.radians(roll_deg)
    pitch = np.radians(pitch_deg)
    yaw   = np.radians(yaw_deg)

    if INVERT_ROLL:
        roll = -roll

    # u = -K @ e
    tau_r_raw = -Kp * roll
    tau_p_raw = -Kp * pitch
    tau_yaw   = -Kp_yaw * yaw

    if LQR_REMAP_SWAP:
        tau_r, tau_p = tau_p_raw, tau_r_raw   # swap roll↔pitch before IK
    else:
        tau_r, tau_p = tau_r_raw, tau_p_raw

    vels = []
    for i, a in enumerate(WHEEL_ANGLES):
        t = np.cos(a)*CA*tau_r + np.sin(a)*CA*tau_p + SA*tau_yaw
        t = np.clip(t, -IK_MAX_VEL, IK_MAX_VEL)
        v = np.clip(MOTOR_SIGNS[i] * t, -VEL_MAX_LQR, VEL_MAX_LQR)
        vels.append(v)
    vels = np.array(vels)

    # Forward kinematics: ball movement (each wheel drives ball tangentially)
    # After the remap/sign corrections, "pitch" → forward, "roll" → sideways
    bx = sum(np.cos(WHEEL_ANGLES[i]) * CA * vels[i] for i in range(3))  # sideways
    by = sum(np.sin(WHEEL_ANGLES[i]) * CA * vels[i] for i in range(3))  # forward

    return vels, bx, by


def simulate_trajectory(roll_deg, pitch_deg, yaw_deg, steps=100, dt=0.05):
    """Euler integration: where does the ball go if tilt is held constant?"""
    x, y = 0.0, 0.0
    xs, ys = [x], [y]
    for _ in range(steps):
        _, bx, by = compute(roll_deg, pitch_deg, yaw_deg)
        x += bx * dt
        y += by * dt
        xs.append(x)
        ys.append(y)
    return np.array(xs), np.array(ys)


# ============================================================
# Figure
# ============================================================
BG   = '#1a1a2e'
DARK = '#16213e'

fig = plt.figure(figsize=(15, 8.5), facecolor=BG)
fig.suptitle(
    'Ball-Wheelchair  IMU → Motor Response Simulator',
    color='white', fontsize=14, fontweight='bold', y=0.99
)

gs = gridspec.GridSpec(
    1, 3, figure=fig,
    left=0.05, right=0.97, top=0.93, bottom=0.26,
    wspace=0.38
)
ax_plat = fig.add_subplot(gs[0, 0])
ax_traj = fig.add_subplot(gs[0, 1])
ax_bars = fig.add_subplot(gs[0, 2])

for ax in [ax_plat, ax_traj, ax_bars]:
    ax.set_facecolor(DARK)
    for sp in ax.spines.values():
        sp.set_edgecolor('#333355')
    ax.tick_params(colors='#aaaaaa', labelsize=8)

# ── Panel 1: top-down platform ──────────────────────────────
ax_plat.set_xlim(-2.3, 2.3)
ax_plat.set_ylim(-2.3, 2.3)
ax_plat.set_aspect('equal')
ax_plat.set_title('Top-down: wheel velocity arrows', color='white', fontsize=10)
ax_plat.set_xlabel('← Roll left / Roll right →', color='#888888', fontsize=8)
ax_plat.set_ylabel('← Pitch back / Pitch fwd →', color='#888888', fontsize=8)
ax_plat.axhline(0, color='#252548', lw=0.8)
ax_plat.axvline(0, color='#252548', lw=0.8)

# Platform ring
th = np.linspace(0, 2*np.pi, 300)
ax_plat.plot(np.cos(th), np.sin(th), '--', color='#334455', lw=1.2)

# Ball at centre
ax_plat.add_patch(Circle((0, 0), 0.22, color='#2c3e50', ec='#7f8c8d', lw=1.5, zorder=3))
ax_plat.text(0, 0, 'BALL', color='#7f8c8d', fontsize=6, ha='center', va='center', zorder=4)

# Wheel markers
for i, a in enumerate(WHEEL_ANGLES):
    wx, wy = np.cos(a), np.sin(a)
    ax_plat.add_patch(Circle((wx, wy), 0.13, color=WHEEL_COLORS[i], zorder=5))
    ax_plat.text(wx*1.8, wy*1.8, WHEEL_NAMES[i],
                 color=WHEEL_COLORS[i], fontsize=7.5, ha='center', va='center',
                 fontweight='bold')

# Quivers for wheel velocities (updated each frame)
Q_wheels = [
    ax_plat.quiver(
        np.cos(a), np.sin(a), 0, 0,
        color=WHEEL_COLORS[i],
        scale=4, scale_units='xy',
        width=0.018, headwidth=5, headlength=6,
        zorder=6
    )
    for i, a in enumerate(WHEEL_ANGLES)
]

# Ball direction quiver
Q_ball = ax_plat.quiver(
    0, 0, 0, 0,
    color='yellow', scale=2.5, scale_units='xy',
    width=0.022, headwidth=6, zorder=7
)
ax_plat.text(0, -2.1, '● Yellow arrow = ball movement direction',
             color='yellow', fontsize=7, ha='center')

# ── Panel 2: trajectory ──────────────────────────────────────
ax_traj.set_xlim(-5, 5)
ax_traj.set_ylim(-5, 5)
ax_traj.set_aspect('equal')
ax_traj.set_title('Predicted trajectory  (5 seconds)', color='white', fontsize=10)
ax_traj.set_xlabel('X  (roll direction, metres)', color='#888888', fontsize=8)
ax_traj.set_ylabel('Y  (pitch / forward direction, metres)', color='#888888', fontsize=8)
ax_traj.axhline(0, color='#252548', lw=0.8)
ax_traj.axvline(0, color='#252548', lw=0.8)

# Grid rings
for r in [1, 2, 3, 4]:
    ax_traj.plot(r*np.cos(th), r*np.sin(th), color='#252548', lw=0.5)

traj_line,   = ax_traj.plot([], [], color='#f39c12', lw=2.5, zorder=3)
traj_start,  = ax_traj.plot([0], [0], 'o', color='white',  ms=7,  zorder=5)
traj_end,    = ax_traj.plot([0], [0], '*', color='yellow', ms=13, zorder=5)
traj_time_labels = [
    ax_traj.text(0, 0, f'{t}s', color='#f39c12', fontsize=7, ha='left', va='bottom')
    for t in [1, 2, 3, 4, 5]
]

ax_traj.legend(
    handles=[
        plt.Line2D([0], [0], marker='o', color='white', ms=6, linestyle=''),
        plt.Line2D([0], [0], marker='*', color='yellow', ms=10, linestyle=''),
    ],
    labels=['Start (0s)', 'End (5s)'],
    facecolor=DARK, edgecolor='#444466', labelcolor='white', fontsize=8, loc='upper right'
)

# Tilt direction indicator
tilt_quiver = ax_traj.quiver(0, 0, 0, 0,
                              color='#e74c3c', scale=4, scale_units='xy',
                              width=0.025, headwidth=5, zorder=6)
tilt_text = ax_traj.text(0, 0, '', color='#e74c3c', fontsize=8, ha='center',
                          fontweight='bold')

# ── Panel 3: bar chart ───────────────────────────────────────
ax_bars.set_ylim(-VEL_MAX_LQR * 1.35, VEL_MAX_LQR * 1.35)
ax_bars.set_xticks([0, 1, 2])
ax_bars.set_xticklabels(['M1\n(node0, 60°)', 'M2\n(node1, 180°)', 'M3\n(node2, 300°)'],
                         color='white', fontsize=8)
ax_bars.set_ylabel('Wheel velocity  (rad/s)', color='#aaaaaa', fontsize=9)
ax_bars.set_title('Commanded wheel velocities', color='white', fontsize=10)
ax_bars.axhline(0, color='#555577', lw=1.2)
ax_bars.axhline( VEL_MAX_LQR, color='#444466', lw=0.8, linestyle=':')
ax_bars.axhline(-VEL_MAX_LQR, color='#444466', lw=0.8, linestyle=':')
ax_bars.text(2.45,  VEL_MAX_LQR + 0.05, f'+{VEL_MAX_LQR} max', color='#666688', fontsize=7)
ax_bars.text(2.45, -VEL_MAX_LQR - 0.18, f'{-VEL_MAX_LQR} max', color='#666688', fontsize=7)

bars = ax_bars.bar([0, 1, 2], [0, 0, 0],
                   color=WHEEL_COLORS, width=0.55,
                   edgecolor='white', linewidth=0.6)
bar_texts = [
    ax_bars.text(i, 0.05, '0.00', ha='center', color='white',
                 fontsize=10, fontweight='bold')
    for i in range(3)
]
ax_bars.text(0.99, 0.01, '+ve = CW  /  −ve = CCW',
             transform=ax_bars.transAxes, color='#888888', fontsize=8,
             ha='right', va='bottom')

# Info box (bottom-right of bars panel)
info_box = ax_bars.text(
    0.5, -0.42, '',
    transform=ax_bars.transAxes,
    color='#dddddd', fontsize=8.5, ha='center', va='top',
    fontfamily='monospace',
    bbox=dict(facecolor='#0d0d1a', edgecolor='#333355', boxstyle='round,pad=0.5')
)

# ============================================================
# Sliders
# ============================================================
SC = '#2c2c4e'
ax_sr = plt.axes([0.08, 0.16, 0.84, 0.032], facecolor=SC)
ax_sp = plt.axes([0.08, 0.10, 0.84, 0.032], facecolor=SC)
ax_sy = plt.axes([0.08, 0.04, 0.84, 0.032], facecolor=SC)

s_roll  = Slider(ax_sr, 'Roll  (°)',  -25, 25, valinit=0, color='#c0392b')
s_pitch = Slider(ax_sp, 'Pitch (°)', -25, 25, valinit=0, color='#27ae60')
s_yaw   = Slider(ax_sy, 'Yaw   (°)', -25, 25, valinit=0, color='#2980b9')

for s in [s_roll, s_pitch, s_yaw]:
    s.label.set_color('white')
    s.label.set_fontsize(10)
    s.valtext.set_color('white')

# ============================================================
# Update callback
# ============================================================
DT      = 0.05
STEPS   = 100   # = 5 seconds
T_MARKS = [20, 40, 60, 80, 100]   # indices for 1s, 2s, 3s, 4s, 5s labels

def update(_=None):
    rd, pd, yd = s_roll.val, s_pitch.val, s_yaw.val
    vels, bx, by = compute(rd, pd, yd)

    # ── wheel arrows (tangential direction per wheel) ──
    for i, a in enumerate(WHEEL_ANGLES):
        tx, ty = -np.sin(a), np.cos(a)          # tangential unit vector
        v = vels[i]
        Q_wheels[i].set_UVC(tx * v * 0.55, ty * v * 0.55)

    # ── ball direction arrow ──
    mag = np.hypot(bx, by)
    if mag > 0.02:
        sc = min(mag, 0.75) / mag
        Q_ball.set_UVC(bx * sc, by * sc)
    else:
        Q_ball.set_UVC(0, 0)

    # ── trajectory ──
    xs, ys = simulate_trajectory(rd, pd, yd, steps=STEPS, dt=DT)
    traj_line.set_data(xs, ys)
    traj_end.set_data([xs[-1]], [ys[-1]])
    for j, (idx, lbl) in enumerate(zip(T_MARKS, traj_time_labels)):
        if idx < len(xs):
            lbl.set_position((xs[idx] + 0.1, ys[idx] + 0.1))

    # ── tilt indicator on trajectory panel ──
    tilt_dx = np.sin(np.radians(rd)) * 0.8
    tilt_dy = np.sin(np.radians(pd)) * 0.8
    tilt_quiver.set_UVC(tilt_dx, tilt_dy)
    tilt_text.set_position((tilt_dx * 1.6, tilt_dy * 1.6 + 0.2))
    tilt_text.set_text(f'Tilt\n({rd:+.0f}°R, {pd:+.0f}°P)')

    # ── bars ──
    for i, (bar, txt) in enumerate(zip(bars, bar_texts)):
        v = float(vels[i])
        bar.set_height(abs(v))
        bar.set_y(-abs(v) if v < 0 else 0)
        txt.set_text(f'{v:+.2f}')
        txt.set_y(v + (0.07 if v >= 0 else -0.22))

    # ── info text ──
    ball_dist = np.hypot(xs[-1], ys[-1])
    info_box.set_text(
        f'Roll: {rd:+.1f}°   Pitch: {pd:+.1f}°   Yaw: {yd:+.1f}°\n'
        f'─────────────────────────────\n'
        f'v1: {vels[0]:+.3f}   v2: {vels[1]:+.3f}   v3: {vels[2]:+.3f}  rad/s\n'
        f'Ball dir: dx={bx:+.3f}  dy={by:+.3f}\n'
        f'5s travel: {ball_dist:.2f} m  '
        f'@ {np.degrees(np.arctan2(by, bx)):+.0f}°'
    )

    fig.canvas.draw_idle()


s_roll.on_changed(update)
s_pitch.on_changed(update)
s_yaw.on_changed(update)
update()   # initial draw

plt.show()
