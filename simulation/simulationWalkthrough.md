# Omni-Directional Chair Simulation Walkthrough

## Overview

This guide walks you through creating a Simulink/Simscape simulation of an omni-directional chair with 3 omni wheels in contact with a large ball. The simulation uses the Simscape Multibody Contact Forces Library.

**Reference Example:** `Frict3D06BallonBalls` - Large ball on top of 4 smaller spinning balls  
**Your Design:** Large ball (8") with 3 omni wheels underneath, motors drive the wheels, platform sits on top

---

## Prerequisites

### Required MATLAB Toolboxes

1. **Simulink**
2. **Simscape**
3. **Simscape Multibody**
4. **Simscape Multibody Contact Forces Library**

### CAD Files Available

- `/simulation/cadFiles/Ball.STEP` - Main ball geometry
- `/simulation/cadFiles/omni_wheel.STEP` - Omni wheel geometry
- `/simulation/cadFiles/Structure.STEP` - Platform/frame structure

---

## Part 1: Model Setup & Parameters

### Step 1: Create New Simulink Model

1. Open MATLAB
2. Type `simulink` in the command window or click "Simulink" button
3. Click "Blank Model" to create a new model
4. Save the model as `OmniChair_Simulation.slx` in `/simulation/`

### Step 2: Configure Model Workspace

1. In the Simulink model, click **Modeling** tab → **Model Explorer** (or Ctrl+H)
2. In Model Explorer, select your model in the left pane
3. Right-click → **Add** → **MATLAB Variable**
4. In the **Model Workspace** section, change "Type" to **MATLAB Code**
5. Add the following parameters:

```matlab
% All units in SI (meters, kg, seconds)

% ===== BALL PARAMETERS =====
main_ball.rad = 0.1016;           % 8 inches = 0.1016 meters (4 inch radius)
main_ball.density = 1200;         % kg/m^3 (rubber ball)
main_ball.mass = (4/3)*pi*main_ball.rad^3 * main_ball.density;

% ===== OMNI WHEEL PARAMETERS =====
omni_wheel.rad = 0.0254;          % 2 inches = 0.0254 meters (1 inch radius)
omni_wheel.width = 0.02;          % 20mm wheel width
omni_wheel.density = 2700;        % kg/m^3 (aluminum)
omni_wheel.mass = 0.1;            % 100g per wheel

% ===== WHEEL POSITIONING =====
% Three wheels arranged 120° apart around the ball
wheel.angle_spacing = 120;        % degrees between wheels
wheel.radius_from_center = 0.08;  % meters from ball center to wheel contact point

% Wheel positions in X-Y plane (cartesian coordinates)
wheel1.angle = 0;                 % Front wheel
wheel2.angle = 120;               % Left-back wheel
wheel3.angle = 240;               % Right-back wheel

wheel1.x = wheel.radius_from_center * cosd(wheel1.angle);
wheel1.y = wheel.radius_from_center * sind(wheel1.angle);
wheel2.x = wheel.radius_from_center * cosd(wheel2.angle);
wheel2.y = wheel.radius_from_center * sind(wheel2.angle);
wheel3.x = wheel.radius_from_center * cosd(wheel3.angle);
wheel3.y = wheel.radius_from_center * sind(wheel3.angle);

% All wheels at same height below ball center
wheel.z_offset = -(main_ball.rad + omni_wheel.rad*0.9); % Slight overlap for contact

% ===== PLATFORM PARAMETERS =====
platform.mass = 5;                % 5 kg platform
platform.height = 0.15;           % 150mm above ball center
platform.size = [0.3, 0.3, 0.05]; % 300mm x 300mm x 50mm

% ===== USER/CHAIR PARAMETERS =====
user.mass = 70;                   % 70 kg (~154 lbs)
user.height_above_platform = 0.4; % 400mm (seat height)

% ===== CONTACT FORCE PARAMETERS =====
contact.k_stiffness = 1e6;        % Contact stiffness (N/m)
contact.b_damping = 1e4;          % Contact damping (N/(m/s))
contact.mu_static = 0.8;          % Static friction coefficient (rubber on rubber)
contact.mu_kinetic = 0.6;         % Kinetic friction coefficient
contact.v_threshold = 0.001;      % Velocity threshold for friction (m/s)

% ===== MOTOR PARAMETERS =====
motor.max_torque = 5;             % Maximum motor torque (N·m)
motor.max_speed = 100;            % Maximum motor speed (rad/s) ~955 RPM

% ===== SIMULATION PARAMETERS =====
sim.gravity = -9.81;              % m/s^2 (negative Z direction)
sim.time_step = 0.001;            % 1ms time step
sim.duration = 10;                % 10 second simulation
```

6. Click **Save** in Model Explorer

---

## Part 2: Building the Physical Model

### Step 3: Add World Frame (Gravity Reference)

1. In your Simulink model, click **Simscape** tab → **Simscape Multibody** in Library Browser
2. Navigate to: **Simscape Multibody → Frames and Transforms**
3. Drag **World Frame** into your model
4. Place it at the bottom of your model canvas
5. Double-click it and set:
   - **Gravity:** `[0, 0, sim.gravity]` (or just `[0, 0, -9.81]`)

### Step 4: Add Mechanism Configuration

1. From **Simscape Multibody → Utilities**
2. Drag **Mechanism Configuration** block into model
3. Double-click and configure:
   - **Gravity:** `[0, 0, -9.81]`
   - **Solver Type:** `Variable step`
   - **Dynamics:** `Forward Dynamics`
4. Leave this disconnected (it applies globally)

### Step 5: Add Ground Plane (Optional but Recommended)

1. From **Simscape Multibody → Body Elements**
2. Drag **Brick Solid** into model
3. Double-click and configure:
   - **Geometry:** `[10, 10, 0.01]` (large thin ground plane)
   - **Density:** `7850` (steel)
   - **Graphic:** Color = `[0.5, 0.5, 0.5]` (gray)
4. Add a **Rigid Transform** before it:
   - **Translation:** `[0, 0, -0.5]` (position ground below ball)
5. Add a **Weld Joint** to fix ground to World Frame

---

## Part 3: Creating the Main Ball

### Step 6: Add Main Ball Body

#### Option A: Using Built-in Sphere

1. From **Simscape Multibody → Body Elements**
2. Drag **Solid** block → choose **Sphere** geometry
3. Configure:
   - **Radius:** `main_ball.rad`
   - **Density:** `main_ball.density`
   - **Color:** `[0.7, 0.7, 0.7]` (gray)

#### Option B: Using CAD Import (More Realistic)

1. Right-click in model → **Simscape** → **Import from CAD**
2. Select `/simulation/cadFiles/Ball.STEP`
3. Let MATLAB generate the subsystem
4. Configure the geometry parameters to match `main_ball.rad`

### Step 7: Position Main Ball

1. Add **6-DOF Joint** from **Simscape Multibody → Joints**
2. Connect between **World Frame** and **Main Ball**
3. Configure 6-DOF Joint:
   - **Name:** `Ball_6DOF`
   - This allows the ball to move freely in all directions
   - Initial position: `[0, 0, 0.15]` (ball starts 150mm above ground)

---

## Part 4: Creating Omni Wheels

### Step 8: Create First Omni Wheel

1. Add **Solid** block (Cylinder geometry) or import `omni_wheel.STEP`
2. Configure:
   - **Radius:** `omni_wheel.rad`
   - **Length:** `omni_wheel.width`
   - **Density:** `omni_wheel.density`
   - **Color:** `[0.2, 0.4, 0.8]` (blue)

### Step 9: Position First Omni Wheel

1. Add **Rigid Transform** from **Frames and Transforms**
2. Configure:
   - **Translation Method:** `Cartesian`
   - **Translation:** `[wheel1.x, wheel1.y, wheel.z_offset]`
   - **Rotation:** Configure so wheel axis points toward ball center
3. Connect: **World Frame** → **Rigid Transform** → **Omni Wheel 1**

### Step 10: Add Revolute Joint for Wheel Rotation

1. Add **Revolute Joint** from **Joints**
2. Insert between **Rigid Transform** and **Omni Wheel 1**
3. Configure:
   - **Name:** `Wheel1_Joint`
   - **Axis:** Configure perpendicular to the radius toward ball center
   - **Initial Velocity:** `0` rad/s

### Step 11: Add Motor Actuation

1. Add **Simscape → Physical Signals → PS Constant** or **PS Simulink Converter**
2. Add **Joint Actuator** from **Simscape Multibody → Actuators**
3. Connect torque input to motor control signal
4. For now, use a constant torque or sine wave for testing

### Step 12: Duplicate for Wheels 2 and 3

1. Select **Rigid Transform + Revolute Joint + Omni Wheel** subsystem
2. Copy and paste twice
3. Rename to `Wheel2` and `Wheel3`
4. Update **Rigid Transform** translations:
   - Wheel 2: `[wheel2.x, wheel2.y, wheel.z_offset]`
   - Wheel 3: `[wheel3.x, wheel3.y, wheel.z_offset]`
5. Adjust rotation angles: 120° and 240° respectively

---

## Part 5: Adding Contact Forces

### Step 13: Install Contact Forces Library

If not already installed:

1. Go to MATLAB File Exchange
2. Search "Simscape Multibody Contact Forces Library"
3. Download and install
4. OR: Check if it's in your **Simulink Library Browser** under **Simscape Multibody Contact Forces Library**

### Step 14: Add Ball-to-Wheel Contact Force (Wheel 1)

1. Open **Library Browser** → **Simscape Multibody Contact Forces Library** → **3D**
2. Drag **Sphere to Sphere Force** block (if wheels approximated as spheres)
   - OR use **Sphere to Solid Force** for more complex geometries
3. Configure contact parameters:

   - **Sphere B Radius:** `main_ball.rad` (base - the main ball)
   - **Sphere F Radius:** `omni_wheel.rad` (follower - the wheel)
   - **Force Law:** `Linear`
   - **Contact Stiffness (k):** `contact.k_stiffness` (1e6)
   - **Contact Damping (b):** `contact.b_damping` (1e4)
   - **Friction Law:** `Stick-Slip Continuous`
   - **Static Friction (μs):** `contact.mu_static` (0.8)
   - **Kinetic Friction (μk):** `contact.mu_kinetic` (0.6)
   - **Velocity Threshold:** `contact.v_threshold` (0.001)

4. Connect ports:
   - **Base Frame (B):** Connect to the frame port of **Main Ball**
   - **Follower Frame (F):** Connect to the frame port of **Omni Wheel 1**

### Step 15: Add Contact Forces for Wheels 2 and 3

1. Duplicate the **Sphere to Sphere Force** block twice
2. Connect to Wheel 2 and Wheel 3 respectively
3. Keep same parameters

---

## Part 6: Adding Platform and User Mass

### Step 16: Create Platform Body

1. Add **Brick Solid** from **Body Elements**
2. Configure:
   - **Dimensions:** `platform.size`
   - **Density:** Calculate to match `platform.mass`
3. Add **Rigid Transform** to position above ball:
   - **Translation:** `[0, 0, platform.height]`

### Step 17: Connect Platform to Ball

1. Add **Bushing Joint** from **Joints** (allows flexible connection)
2. Configure stiffness and damping for realistic coupling
3. OR use **Weld Joint** for rigid connection

### Step 18: Add User Mass

1. Add **Point Mass** from **Body Elements**
2. Configure:
   - **Mass:** `user.mass`
   - **Position:** `[0, 0, user.height_above_platform]` relative to platform

---

## Part 7: Sensor and Visualization

### Step 19: Add Sensors

1. From **Simscape Multibody → Sensors & Actuators**
2. Add **Transform Sensor** to each component:

   - Main Ball position and velocity
   - Wheel rotations and speeds
   - Platform orientation (roll, pitch, yaw); roll = X (bank), pitch = Y (nose up/down)

3. Add **PS Simulink Converter** to send data to Simulink scopes

### Step 20: Add Visualization Scopes

1. Add **Scope** blocks to visualize:
   - Ball X, Y, Z position over time
   - Ball velocity
   - Wheel angular velocities
   - Platform tilt angles

### Step 21: Configure Mechanics Explorer

1. Click **Simscape** tab → **Mechanics Explorer**
2. This will open a 3D visualization window when simulation runs

---

## Part 8: Adding Control System

### Step 22: Create Motor Controller

1. Create a subsystem called `Motor_Controller`
2. Inputs:
   - Platform tilt (roll, pitch) from IMU; roll = X (bank), pitch = Y (nose)
   - Desired velocity (X, Y direction)
3. Outputs:
   - Torque commands for 3 motors

### Step 23: Implement Basic Control Logic

For initial testing, implement simple control:

```matlab
% In MATLAB Function block or Simulink Function
function [T1, T2, T3] = motor_controller(pitch, roll, vel_x_cmd, vel_y_cmd)
    % Convert tilt to correction velocity
    Kp = 10;  % Proportional gain

    vel_x = vel_x_cmd - Kp * pitch;
    vel_y = vel_y_cmd - Kp * roll;

    % Convert to wheel velocities (3-wheel omnidirectional kinematics)
    % Wheel 1 (0°), Wheel 2 (120°), Wheel 3 (240°)
    w1 = vel_x;
    w2 = -0.5*vel_x + 0.866*vel_y;
    w3 = -0.5*vel_x - 0.866*vel_y;

    % Convert to torques (simple proportional)
    Kt = 1;  % Torque constant
    T1 = Kt * w1;
    T2 = Kt * w2;
    T3 = Kt * w3;
end
```

### Step 24: Connect Controller to Motors

1. Connect **Transform Sensor** outputs to controller inputs
2. Connect controller torque outputs to **Revolute Joint** actuation inputs

---

## Part 9: Simulation Configuration

### Step 25: Configure Solver Settings

1. Click **Modeling** → **Model Settings** (Ctrl+E)
2. **Solver** tab:

   - **Type:** `Variable-step`
   - **Solver:** `ode23t` (stiff/moderate stiffness) or `ode15s` (stiff)
   - **Max step size:** `0.01` (10ms)
   - **Relative tolerance:** `1e-4`

3. **Data Import/Export** tab:
   - Check **Time** and **States**
   - Set **Format:** `Dataset`

### Step 26: Set Simulation Time

1. In the model toolbar, set:
   - **Stop Time:** `sim.duration` or `10` (10 seconds)

---

## Part 10: Testing and Validation

### Step 27: Initial Test - Static Balance

1. Set all motor torques to 0
2. Run simulation
3. Check if ball/platform settles to stable position
4. Ball should rest on the 3 wheels with contact forces balanced

### Step 28: Test - Single Motor Actuation

1. Apply constant torque to Wheel 1 only: `motor.max_torque * 0.5`
2. Keep Wheels 2 and 3 at 0 torque
3. Run simulation
4. Observe: Ball should roll in the direction of Wheel 1

### Step 29: Test - Coordinated Motion

1. Apply equal torque to all 3 wheels in same direction
2. Run simulation
3. Observe: Ball should spin or move linearly

### Step 30: Test - Balancing Control

1. Tilt the platform initial condition: `[0.1, 0, platform.height]` (10cm offset)
2. Enable the motor controller
3. Run simulation
4. Observe: Controller should balance the platform back to center

---

## Part 11: Advanced Enhancements

### Step 31: Import Actual CAD Geometry

1. Use **Simscape Multibody Link** to import STEP files
2. Replace simple geometries with actual CAD models
3. This provides accurate mass, inertia, and visual appearance

### Step 32: Add IMU Sensor Model

1. Create subsystem to simulate MPU-6050 IMU
2. Add noise to accelerometer and gyroscope readings
3. Implement complementary filter for orientation estimation

### Step 33: Add Ground Contact Model

1. Add **Sphere to Plane Force** between main ball and ground
2. This allows the chair to move across the ground surface
3. Configure friction appropriately for indoor court surface

### Step 34: Tune Contact Parameters

Based on real materials:

- **Rubber-on-rubber:** μs ≈ 0.8-1.0, k ≈ 1e5-1e6
- Adjust damping to prevent excessive bouncing
- Use **Data Inspector** to plot contact forces

### Step 35: Add Disturbance/Collision Forces

1. Add **External Force and Torque** block
2. Apply impulse forces to simulate:
   - Player collisions (30-50N lateral force)
   - Sudden movements
3. Test stability and recovery

---

## Debugging Tips

### Common Issues and Solutions

**1. Simulation fails to start / Singular matrix error:**

- Check that all joints are properly constrained
- Ensure World Frame is connected properly
- Verify no redundant constraints (over-constrained system)

**2. System is unstable / explodes:**

- Reduce time step (max step size)
- Increase contact damping
- Check initial conditions (no penetration)
- Use stiffer solver (ode15s)

**3. Contact forces not working:**

- Verify Contact Forces Library is installed
- Check that geometries are close enough initially (< 1mm gap)
- Increase contact stiffness gradually
- Check frame connections (Base and Follower)

**4. Wheels don't grip / ball slips:**

- Increase friction coefficients
- Increase contact normal force (preload)
- Check velocity threshold setting
- Verify wheel rotation axis is correct

**5. Slow simulation:**

- Reduce model complexity (use simple geometries)
- Increase max step size carefully
- Disable unnecessary visualizations
- Use `Accelerator` or `Rapid Accelerator` mode

---

## Next Steps

### Validation

1. Compare simulation results with physics calculations
2. Verify ball velocity matches expected from wheel speeds
3. Check force balance at equilibrium

### Control System Design

1. Implement PID controller for balance
2. Add user input interface (joystick, body tilt)
3. Tune controller gains for performance

### Optimization

1. Perform parameter sweep for optimal contact stiffness
2. Find minimum motor torque requirements
3. Optimize wheel positions for stability

### Physical Prototype Correlation

1. Once hardware is built, compare simulation to real measurements
2. Update model parameters based on actual testing
3. Use model for control system development before hardware testing

---

## Reference Parameters from Example

The `Frict3D06BallonBalls` example uses:

- **Upper ball radius:** 1m
- **Lower ball radius:** 0.2m
- **Contact stiffness:** 1e7 N/m
- **Contact damping:** 1e5 N/(m/s)
- **Static friction:** 0.7
- **Kinetic friction:** 0.5

For your design (scaled down):

- **Main ball radius:** 0.1016m (8" diameter)
- **Wheel radius:** 0.0254m (2" diameter)
- **Contact stiffness:** 1e6 N/m (softer for realistic rubber)
- **Contact damping:** 1e4 N/(m/s)
- **Static friction:** 0.8 (rubber-on-rubber)
- **Kinetic friction:** 0.6

---

## Files to Create

1. **Model file:** `OmniChair_Simulation.slx`
2. **Init script:** `init_omni_chair.m` (contains all parameters)
3. **Controller:** `motor_controller.m` or Simulink Function
4. **Analysis scripts:** `analyze_simulation.m` (post-processing)

---

## Useful MATLAB/Simulink Commands

```matlab
% Load parameters before simulation
run('init_omni_chair.m')

% Run simulation from command line
sim('OmniChair_Simulation')

% Open Mechanics Explorer
smexplore('OmniChair_Simulation')

% Access simulation results
logsout = simOut.logsout;  % If logging configured
ball_pos = logsout.get('Ball_Position').Values;

% Plot results
figure; plot(ball_pos.Time, ball_pos.Data(:,3));
xlabel('Time (s)'); ylabel('Ball Height (m)');
```

---

## Additional Resources

- **MATLAB Simscape Documentation:** [mathworks.com/simscape](https://www.mathworks.com/products/simscape.html)
- **Contact Forces Library Examples:** Check File Exchange
- **Multibody Tutorials:** MATLAB Documentation → Simscape Multibody → Getting Started

---

**Document Created:** 2025-11-04  
**Project:** Omni-Directional Wheelchair Basketball Chair  
**Team:** Capstone Group 52
