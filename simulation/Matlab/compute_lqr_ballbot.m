%% compute_lqr_ballbot.m
% Compute LQR gains for 3-axis ball-bot balance.
% Aligned with Python simulation/ik_validation/lqr.py (same params and weights).
% Convention: front = -X; state order [roll; pitch; yaw]; roll = X (bank), pitch = Y (nose up/down).
%
% State: x = [roll; pitch; yaw; omega_roll; omega_pitch; omega_yaw]
% Input: u = [tau_roll; tau_pitch; tau_yaw]  (body torques, N·m)
% Control: u = -K*x (or u = -K*(x - x_ref) with x_ref = [roll_d; pitch_d; yaw_d; 0; 0; 0]).
%
% Roll & pitch: gravity coupling (unstable). Linearized:
%   alpha = (1/J)*(tau - B*omega + M*g*H*theta)  --> tips over when tilted
% Yaw: no gravity (rotation about vertical axis)
%
% Run in MATLAB to get K. Use u = -K*x in Simulink (Gain block or MATLAB Function).
% Time-varying variant (H_eff from lean, yaw setpoint): see lqr_gain_from_lean.m.

%% Physical parameters (match updated user setup)
% Mass: platform and person separate (change M_person or M_platform as needed)
M_platform = 50.0;  % kg
M_person   = 30.0;  % kg
M_ASSY     = M_platform + M_person;  % top assembly mass for gravity term [kg]

G = 9.81;           % gravity [m/s^2]
% CoM heights above ball center [m]; combined H_CM is mass-weighted for gravity torque.
% Person: frame at bottom of torso block (rotation about bottom); bottom 0.40 m above ball;
% torso 0.2 x 0.2 x 0.5 m; CoM at center of block; orientation in world frame.
H_platform = 0.2;   % platform CoM above ball center [m]
H_person   = 0.65;  % person CoM: 0.40 (bottom of block) + 0.5/2 [m]
H_CM       = (M_platform * H_platform + M_person * H_person) / M_ASSY;

% Inertia [kg·m^2] about ball center: platform ~0.5x0.4x0.05 m plate; person torso 0.2x0.2x0.5 m (see compute_lqr_lookup.m)
J_platform_roll  = 2.7;
J_platform_pitch = 3.0;
J_platform_yaw   = 3.7;
J_person_roll    = 13.4;
J_person_pitch   = 13.4;
J_person_yaw     = 12.9;
J_roll  = J_platform_roll  + J_person_roll;
J_pitch = J_platform_pitch + J_person_pitch;
J_yaw   = J_platform_yaw   + J_person_yaw;

B_roll = 0.5;       % damping roll [N·m·s]
B_pitch = 0.5;      % damping pitch [N·m·s]
B_yaw = 0.3;        % damping yaw [N·m·s]

%% Linearized A, B matrices
% Roll:  d(roll)/dt = omega_r,  d(omega_r)/dt = (M*g*H/J)*roll - (B/J)*omega_r + (1/J)*tau_r
% Pitch: same
% Yaw:   d(yaw)/dt = omega_y,   d(omega_y)/dt = -(B/J)*omega_y + (1/J)*tau_y   (no gravity)
g_roll = M_ASSY * G * H_CM / J_roll;   % gravity gain roll
g_pitch = M_ASSY * G * H_CM / J_pitch; % gravity gain pitch
% g_yaw = 0  (yaw: no gravity)

A = [0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1;
     g_roll  0       0    -B_roll/J_roll   0              0;
     0       g_pitch 0     0               -B_pitch/J_pitch 0;
     0       0       0     0               0              -B_yaw/J_yaw];

B = [0 0 0;
     0 0 0;
     0 0 0;
     1/J_roll  0         0;
     0         1/J_pitch 0;
     0         0         1/J_yaw];

%% LQR weights
% Q: state cost. Penalize orientation and angular velocity.
% R: input cost. Higher R = less control effort (smoother, less overshoot).
%    Lower R = more aggressive (higher torques). Try 0.01–0.1.
Q = diag([500, 200, 50, 10, 10, 5]);   % [roll, pitch, yaw, omega_r, omega_p, omega_y]
R = 0.05 * eye(3);                     % Try 0.01 (aggressive) to 0.1 (conservative)

%% Compute gain
K = lqr(A, B, Q, R);

%% Display
fprintf('LQR gain K (u = -K*x):\n');
disp(K);
fprintf('State order: [roll; pitch; yaw; omega_roll; omega_pitch; omega_yaw] (front=-X, roll=X, pitch=Y)\n');
fprintf('Input order: [tau_roll; tau_pitch; tau_yaw]\n');
fprintf('In Simulink: use Gain block with -K (or u = -K*x in MATLAB Function).\n');
