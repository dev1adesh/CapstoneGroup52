%% Omni-Directional Chair Simulation - Initialization Script
% This script defines all parameters for the Simulink/Simscape model
% Run this before running the simulation
% Project: Wheelchair Basketball Omni-Directional Chair
% Team: Capstone Group 52
% Date: 2025-11-04

clear all;
clc;

fprintf('Initializing Omni-Directional Chair Simulation Parameters...\n');

%% ========== UNIT SYSTEM ==========
% All units in SI: meters, kilograms, seconds, radians

%% ========== MAIN BALL PARAMETERS ==========
main_ball.diameter = 8 * 0.0254;          % 8 inches to meters
main_ball.rad = main_ball.diameter / 2;   % Radius = 0.1016 m
main_ball.density = 1200;                 % kg/m^3 (rubber/foam ball)
main_ball.volume = (4/3) * pi * main_ball.rad^3;
main_ball.mass = main_ball.volume * main_ball.density;
main_ball.inertia = (2/5) * main_ball.mass * main_ball.rad^2; % Sphere inertia

fprintf('  Main Ball: Diameter = %.1f cm, Mass = %.2f kg\n', ...
    main_ball.diameter*100, main_ball.mass);

%% ========== OMNI WHEEL PARAMETERS ==========
omni_wheel.diameter = 2 * 0.0254;         % 2 inches to meters
omni_wheel.rad = omni_wheel.diameter / 2; % Radius = 0.0254 m
omni_wheel.width = 0.02;                  % 20mm wheel width
omni_wheel.density = 2700;                % kg/m^3 (aluminum)
omni_wheel.mass = 0.1;                    % 100g per wheel (estimated)

% Wheel geometry (cylinder)
omni_wheel.volume = pi * omni_wheel.rad^2 * omni_wheel.width;
omni_wheel.inertia = 0.5 * omni_wheel.mass * omni_wheel.rad^2; % Cylinder inertia

fprintf('  Omni Wheels: Diameter = %.1f cm, Mass = %.1f g each\n', ...
    omni_wheel.diameter*100, omni_wheel.mass*1000);

%% ========== WHEEL POSITIONING (3 wheels at 120° spacing) ==========
wheel.num_wheels = 3;
wheel.angle_spacing = 120;                % degrees between wheels

% Distance from ball center to wheel center (adjusted for contact)
wheel.radius_from_center = 0.08;          % 8 cm from center

% Define angles for 3 wheels (in degrees)
wheel1.angle_deg = 0;                     % Front wheel (along +X axis)
wheel2.angle_deg = 120;                   % Left-back wheel
wheel3.angle_deg = 240;                   % Right-back wheel

% Convert to radians
wheel1.angle_rad = deg2rad(wheel1.angle_deg);
wheel2.angle_rad = deg2rad(wheel2.angle_deg);
wheel3.angle_rad = deg2rad(wheel3.angle_deg);

% Calculate X-Y positions (cartesian coordinates)
wheel1.x = wheel.radius_from_center * cos(wheel1.angle_rad);
wheel1.y = wheel.radius_from_center * sin(wheel1.angle_rad);
wheel2.x = wheel.radius_from_center * cos(wheel2.angle_rad);
wheel2.y = wheel.radius_from_center * sin(wheel2.angle_rad);
wheel3.x = wheel.radius_from_center * cos(wheel3.angle_rad);
wheel3.y = wheel.radius_from_center * sin(wheel3.angle_rad);

% All wheels at same height below ball center (slight penetration for contact)
wheel.z_offset = -(main_ball.rad + omni_wheel.rad * 0.85);

fprintf('  Wheel Positions:\n');
fprintf('    Wheel 1 (%.0f°): X=%.3fm, Y=%.3fm\n', wheel1.angle_deg, wheel1.x, wheel1.y);
fprintf('    Wheel 2 (%.0f°): X=%.3fm, Y=%.3fm\n', wheel2.angle_deg, wheel2.x, wheel2.y);
fprintf('    Wheel 3 (%.0f°): X=%.3fm, Y=%.3fm\n', wheel3.angle_deg, wheel3.x, wheel3.y);

%% ========== PLATFORM/FRAME PARAMETERS ==========
platform.length = 0.3;                    % 300mm x 300mm platform
platform.width = 0.3;
platform.height_dim = 0.05;               % 50mm thickness
platform.size = [platform.length, platform.width, platform.height_dim];
platform.mass = 5;                        % 5 kg (aluminum frame + components)
platform.height_above_ball = 0.15;        % Center of platform 150mm above ball center

% Platform inertia (approximated as rectangular prism)
platform.Ixx = (1/12) * platform.mass * (platform.width^2 + platform.height_dim^2);
platform.Iyy = (1/12) * platform.mass * (platform.length^2 + platform.height_dim^2);
platform.Izz = (1/12) * platform.mass * (platform.length^2 + platform.width^2);

fprintf('  Platform: %.0fx%.0f cm, Mass = %.1f kg\n', ...
    platform.length*100, platform.width*100, platform.mass);

%% ========== USER/CHAIR PARAMETERS ==========
user.mass = 70;                           % 70 kg (~154 lbs) - design target
user.height_above_platform = 0.4;         % 400mm (seat to center of mass)
user.total_height = platform.height_above_ball + user.height_above_platform;

% Combined system
system.total_mass = main_ball.mass + (omni_wheel.mass * 3) + platform.mass + user.mass;
system.CoM_height = user.total_height;    % Approximate CoM at user mass center

fprintf('  User Mass: %.1f kg\n', user.mass);
fprintf('  Total System Mass: %.1f kg\n', system.total_mass);
fprintf('  System CoM Height: %.3f m\n', system.CoM_height);

%% ========== CONTACT FORCE PARAMETERS ==========
% Based on rubber-on-rubber contact at small scale
contact.k_stiffness = 1e6;                % Contact stiffness (N/m)
contact.b_damping = 1e4;                  % Contact damping (N/(m/s))
contact.mu_static = 0.8;                  % Static friction coefficient (no-slip)
contact.mu_kinetic = 0.6;                 % Kinetic friction coefficient (no-slip)
% For wheel–ball: if wheels "spin" / torques fight, use lower friction (allow slip):
contact.mu_static_slip = 0.4;             % Static friction when allowing slip (recommended)
contact.mu_kinetic_slip = 0.3;            % Kinetic friction when allowing slip
contact.v_threshold = 0.001;              % Velocity threshold for friction transition (m/s)

% Penetration depth for full damping
contact.pen_full_damp = 1e-4;             % 0.1mm

% Expected normal force per wheel at equilibrium
contact.normal_force_per_wheel = (system.total_mass * 9.81) / 3;

fprintf('  Contact Forces:\n');
fprintf('    Stiffness = %.1e N/m, Damping = %.1e N/(m/s)\n', ...
    contact.k_stiffness, contact.b_damping);
fprintf('    Static friction μs = %.2f, Kinetic μk = %.2f\n', ...
    contact.mu_static, contact.mu_kinetic);
fprintf('    Expected normal force per wheel: %.1f N\n', contact.normal_force_per_wheel);

%% ========== MOTOR PARAMETERS ==========
% Based on 12V DC geared motors from preliminary design
motor.voltage_nominal = 12;               % 12V DC
motor.max_torque = 5;                     % 5 N·m maximum torque (estimate)
motor.max_speed_rpm = 100;                % 100 RPM at output shaft
motor.max_speed_rad = motor.max_speed_rpm * (2*pi/60); % Convert to rad/s
motor.gear_ratio = 10;                    % 10:1 gear reduction
motor.efficiency = 0.85;                  % 85% mechanical efficiency

% Motor constants (approximated)
motor.Kt = 0.5;                           % Torque constant (N·m/A)
motor.Kv = 1000;                          % Velocity constant (RPM/V)
motor.resistance = 1.0;                   % Armature resistance (Ohms)
motor.inductance = 0.001;                 % Armature inductance (H)

fprintf('  Motors: Max Torque = %.1f Nm, Max Speed = %.0f RPM\n', ...
    motor.max_torque, motor.max_speed_rpm);

%% ========== CONTROL SYSTEM PARAMETERS ==========
% PID controller gains for balance control
control.Kp_balance = 50;                  % Proportional gain for balance
control.Ki_balance = 5;                   % Integral gain
control.Kd_balance = 10;                  % Derivative gain

% Velocity control gains
control.Kp_velocity = 2;                  % Proportional gain for velocity tracking
control.Ki_velocity = 0.5;                % Integral gain

% Response time specification
control.response_time_target = 0.1;       % 100ms target response time
control.sample_rate = 100;                % 100 Hz control loop (10ms period)
control.sample_period = 1 / control.sample_rate;

fprintf('  Control System: Sample Rate = %.0f Hz (%.1f ms period)\n', ...
    control.sample_rate, control.sample_period*1000);

%% ========== SIMULATION PARAMETERS ==========
sim.gravity = 9.81;                       % m/s^2 (magnitude)
sim.gravity_vector = [0; 0; -sim.gravity]; % Direction: -Z

sim.time_step_max = 0.001;                % Maximum time step: 1ms
sim.time_step_min = 1e-6;                 % Minimum time step: 1μs
sim.duration = 10;                        % 10 second simulation
sim.solver = 'ode23t';                    % Stiff solver (good for contact dynamics)

% Initial conditions
sim.initial_ball_pos = [0; 0; 0.15];      % Ball starts 15cm above ground
sim.initial_ball_vel = [0; 0; 0];         % Stationary
sim.initial_ball_angle = [0; 0; 0];       % No rotation (Euler angles)
sim.initial_ball_omega = [0; 0; 0];       % No angular velocity

fprintf('  Simulation: Duration = %.1f s, Max Time Step = %.1f ms\n', ...
    sim.duration, sim.time_step_max*1000);

%% ========== GROUND PLANE PARAMETERS ==========
ground.size = [10, 10, 0.01];             % 10m x 10m x 10mm
ground.position = [0, 0, -0.01];          % Just below origin
ground.density = 7850;                    % Steel (doesn't matter, it's fixed)
ground.friction_static = 0.7;             % Ball-to-ground friction
ground.friction_kinetic = 0.5;

%% ========== VISUALIZATION PARAMETERS ==========
vis.show_forces = true;                   % Show contact force vectors
vis.force_scale = 0.01;                   % Scale factor for force visualization
vis.show_traces = true;                   % Show trajectory traces
vis.camera_view = [45, 30];               % Azimuth and elevation for camera

%% ========== DISTURBANCE/TEST PARAMETERS ==========
% For testing stability and response
test.collision_force = 30;                % 30N lateral force (typical collision)
test.collision_duration = 0.1;            % 100ms impulse
test.tilt_angle_max = 15;                 % 15° maximum tilt before e-stop

%% ========== OMNIDIRECTIONAL KINEMATICS MATRIX ==========
% Maps desired [Vx; Vy; Omega_z] to wheel velocities [w1; w2; w3]
% For 3-wheel omnidirectional system with 120° spacing

% Wheel orientation vectors (perpendicular to wheel radius)
wheel1.drive_angle = wheel1.angle_rad + pi/2;  % Perpendicular to radius
wheel2.drive_angle = wheel2.angle_rad + pi/2;
wheel3.drive_angle = wheel3.angle_rad + pi/2;

% Kinematic matrix (3x3)
kinematics.matrix = [
    cos(wheel1.drive_angle), sin(wheel1.drive_angle), wheel.radius_from_center;
    cos(wheel2.drive_angle), sin(wheel2.drive_angle), wheel.radius_from_center;
    cos(wheel3.drive_angle), sin(wheel3.drive_angle), wheel.radius_from_center
];

% Inverse kinematics (velocity command to wheel speeds)
kinematics.inv_matrix = inv(kinematics.matrix);

% Convert wheel angular velocity (rad/s) to platform velocity (m/s)
kinematics.wheel_radius_effective = omni_wheel.rad;

fprintf('\n');
fprintf('Kinematics Matrix (Platform velocity to wheel velocities):\n');
disp(kinematics.matrix);

%% ========== SAFETY LIMITS ==========
safety.max_speed = 2.24;                  % 5 mph = 2.24 m/s (design spec)
safety.max_tilt_angle = 15;               % 15° maximum tilt before warning
safety.max_acceleration = 2.0;            % 2 m/s^2 maximum acceleration
safety.emergency_stop_tilt = 20;          % 20° triggers emergency stop

fprintf('  Safety Limits:\n');
fprintf('    Max Speed: %.2f m/s (%.1f mph)\n', safety.max_speed, safety.max_speed*2.237);
fprintf('    Max Tilt: %.0f°\n', safety.max_tilt_angle);

%% ========== DATA LOGGING CONFIGURATION ==========
log.enable = true;
log.sample_rate = 100;                    % Log at 100 Hz
log.variables = {
    'ball_position', ...
    'ball_velocity', ...
    'ball_orientation', ...
    'wheel1_speed', ...
    'wheel2_speed', ...
    'wheel3_speed', ...
    'platform_tilt', ...
    'contact_forces', ...
    'motor_torques'
};

%% ========== COLORS FOR VISUALIZATION ==========
colors.main_ball = [0.7, 0.7, 0.7];       % Gray
colors.omni_wheel = [0.2, 0.4, 0.8];      % Blue
colors.platform = [0.5, 0.5, 0.5];        % Dark gray
colors.ground = [0.3, 0.6, 0.3];          % Green (court)
colors.user_mass = [0.9, 0.2, 0.2];       % Red

%% ========== PRINT SUMMARY ==========
fprintf('\n');
fprintf('========================================\n');
fprintf('Initialization Complete!\n');
fprintf('========================================\n');
fprintf('Ready to run simulation: OmniChair_Simulation.slx\n');
fprintf('\n');
fprintf('Key System Properties:\n');
fprintf('  Total Mass: %.1f kg\n', system.total_mass);
fprintf('  Ball Radius: %.1f cm\n', main_ball.rad*100);
fprintf('  Wheel Radius: %.1f cm\n', omni_wheel.rad*100);
fprintf('  CoM Height: %.2f m\n', system.CoM_height);
fprintf('  Max Speed: %.2f m/s\n', safety.max_speed);
fprintf('\n');

%% ========== SAVE WORKSPACE ==========
% Optionally save all parameters to a .mat file
% save('omni_chair_params.mat');
% fprintf('Parameters saved to: omni_chair_params.mat\n');

%% ========== HELPER FUNCTIONS ==========

% Calculate required motor torque for given acceleration
function torque = calc_required_torque(mass, radius, accel)
    % T = m * a * r (simplified)
    torque = mass * accel * radius;
end

% Calculate expected wheel speed for given platform velocity
function wheel_speed = calc_wheel_speed(platform_vel, wheel_radius)
    % omega = v / r
    wheel_speed = platform_vel / wheel_radius;
end

fprintf('Type "help init_omni_chair" for parameter descriptions\n');
fprintf('\n');

