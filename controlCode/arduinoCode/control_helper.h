/*
 * Control helper: roll -> motor 1, pitch -> motor 2, yaw -> motor 3.
 * Positive angle (rad) -> positive velocity (CW); negative angle -> negative (CCW).
 */
#ifndef CONTROL_HELPER_H
#define CONTROL_HELPER_H

// Set the zero (neutral) for the control axis [rad]. Call after sensor calibration (legacy).
void control_setAxisZero(float axis_zero_rad);

// Update three motors from roll, pitch, yaw [rad] and their zeros.
// Output: v1 = gain*(roll - roll_zero), v2 = gain*(pitch - pitch_zero), v3 = gain*(yaw - yaw_zero).
// Deadband and saturation applied per axis. Positive angle -> positive (CW) velocity.
void control_updateThree(float roll_rad, float pitch_rad, float yaw_rad,
                         float roll_zero, float pitch_zero, float yaw_zero,
                         float* v1, float* v2, float* v3);

// Legacy: single-axis update (returns one vel_cmd).
float control_update(float axis_rad, float dt_s);

// Get max velocity magnitude (for display).
float control_getVMax(void);

// LQR controller: x = platform state (IMU A), x_ref = user lean (IMU B).
// State order: [roll, pitch, yaw, omega_roll, omega_pitch, omega_yaw] in rad and rad/s.
// Output: velocity commands v1, v2, v3 for motors 1, 2, 3.
// Optional: tau_roll_out, tau_pitch_out, tau_yaw_out (body-axis response torques); pass NULL to skip.
void control_updateLQR(const float x[6], const float x_ref[6],
                       float* v1, float* v2, float* v3,
                       float* tau_roll_out, float* tau_pitch_out, float* tau_yaw_out);

#endif // CONTROL_HELPER_H
