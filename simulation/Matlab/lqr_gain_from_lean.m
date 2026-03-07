function u = lqr_gain_from_lean(x, lean)
% LQR gain from person lean — time-varying K for Simulink MATLAB Function block.
% Uses precomputed K lookup table (no lqr() here — codegen-safe). H_eff from
% lean (roll/pitch); yaw setpoint from lean(3). Run compute_lqr_lookup.m to
% build lqr_lookup.mat (e.g. after changing mass/inertia/CoM).
%
% Inputs (rad):
%   x    (6x1) — state [roll; pitch; yaw; omega_roll; omega_pitch; omega_yaw]
%   lean (3x1) — lean(1)=roll, lean(2)=pitch, lean(3)=yaw (desired yaw setpoint)
%
% Output:
%   u    (3x1) — [tau_roll; tau_pitch; tau_yaw]. u = K*(x - x_ref), x_ref = [0;0;lean(3);0;0;0].

%% Load precomputed K table (embedded at compile time; lqr() not supported for code gen)
persistent K_table H_eff_min H_eff_max N
if isempty(K_table)
    data = coder.load('lqr_lookup.mat');
    K_table   = data.K_table;
    H_eff_min = data.H_eff_min;
    H_eff_max = data.H_eff_max;
    N         = data.N;
end

%% Combined CoM height for H_eff (match compute_lqr_ballbot.m)
M_platform = 50.0;
M_person   = 30.0;
M_ASSY     = M_platform + M_person;
% Person: frame at bottom of torso block; bottom 0.40 m above ball; torso 0.2x0.2x0.5 m; CoM at center; world-frame orientation
H_platform = 0.2;   % platform CoM above ball center [m]
H_person   = 0.65;  % person CoM: 0.40 + 0.5/2 [m]
H_CM       = (M_platform * H_platform + M_person * H_person) / M_ASSY;

%% Effective CoM height from lean [rad], clamped to table range
lean_roll  = lean(1);
lean_pitch = lean(2);
lean_yaw   = lean(3);
H_eff = H_CM * cos(lean_roll) * cos(lean_pitch);
H_eff = max(min(H_eff, H_eff_max), H_eff_min);

%% Index into lookup table (1-based, clip to [1, N])
idx = 1 + round((H_eff - H_eff_min) / (H_eff_max - H_eff_min) * (N - 1));
idx = min(max(idx, 1), N);
K = K_table(:, :, idx);

%% Control: u = K*(x - x_ref)
x_ref = [0; 0; lean_yaw; 0; 0; 0];
u = K * (x - x_ref);

end
