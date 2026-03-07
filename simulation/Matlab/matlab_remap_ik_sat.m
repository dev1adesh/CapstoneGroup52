function [T1, T2, T3] = matlab_remap_ik_sat(Roll_T, Pitch_T, Yaw_T)
% MATLAB Function block: Remap + IK + saturation.
% Aligned with Python simulation/ik_validation/ik.py and balance_sim_ui.py.
%
% LQR output order: [tau_roll, tau_pitch, tau_yaw] (roll=X, pitch=Y, front=-X). Pass-through.
%
% Outputs: T1, T2, T3 (N·m). Wheels @ 60°, 180° (-Y), 300° (120° spacing). Scale=1, MaxT=8.
%
% For Simulink: fcn(Roll_T, Pitch_T, Yaw_T)

    alpha = 25.659;
    ca = cosd(alpha);
    sa = sind(alpha);

    Scale = 1;
    MaxT = 8;

    Roll_T_map  = Roll_T;
    Pitch_T_map = Pitch_T;
    Yaw_T_map   = Yaw_T;

    % IK: row i = [cos(θ_i)*ca, sin(θ_i)*ca, sa] for θ = 60°, 180°, 300°
    c1 = cosd(60);  s1 = sind(60);
    c2 = cosd(180); s2 = sind(180);
    c3 = cosd(300); s3 = sind(300);
    t1 = Scale * ((c1*ca*Roll_T_map) + (s1*ca*Pitch_T_map) + (sa*Yaw_T_map));
    t2 = Scale * ((c2*ca*Roll_T_map) + (s2*ca*Pitch_T_map) + (sa*Yaw_T_map));
    t3 = Scale * ((c3*ca*Roll_T_map) + (s3*ca*Pitch_T_map) + (sa*Yaw_T_map));

    % Saturation (no sign flip — match Python)
    T1 = max(min(t1, MaxT), -MaxT);
    T2 = max(min(t2, MaxT), -MaxT);
    T3 = max(min(t3, MaxT), -MaxT);
end
