/*
 * Control helper: roll->motor1, pitch->motor2, yaw->motor3. Positive angle -> CW.
 * LQR + IK path: state from IMU A, reference from IMU B -> body torques -> wheel torques -> velocity.
 */
#include "control_helper.h"
#include <Arduino.h>
#include <math.h>

// ----- LQR gain K (3x6): u = -K @ (x - x_ref). From simulation/ik_validation/lqr.py -----
static const float K_LQR[3][6] = {
  { 343.068682f,  0.0f, 0.0f, 14.815250f, 0.0f, 0.0f },
  { 0.0f, 326.183053f, 0.0f, 0.0f, 14.760023f, 0.0f },
  { 0.0f, 0.0f, 31.622777f, 0.0f, 0.0f, 9.798879f },
};

// ----- IK: 3-wheel ball balancer (matches simulation/ik_validation/ik.py) -----
// alpha = 25.659 deg; wheels at 60°, 180°, 300°
static const float ALPHA_RAD = 25.659f * (PI / 180.0f);
static const float CA = cosf(ALPHA_RAD);
static const float SA = sinf(ALPHA_RAD);
static const float C1 = 0.5f;      // cos(60°)
static const float S1 = 0.866025f;  // sin(60°)
static const float C2 = -1.0f;      // cos(180°)
static const float S2 = 0.0f;
static const float C3 = 0.5f;       // cos(300°)
static const float S3 = -0.866025f; // sin(300°)
static const float IK_MAX_TORQUE = 2.0f;

// Remap: swap roll/pitch to match Simulink (remap.py REMAP_SWAP = true)
#define LQR_REMAP_SWAP 1

// Torque to velocity scaling for ODrive (tune for your motors)
static const float K_TAU_TO_VEL = 2.0f;
static const float VEL_MAX_LQR = 10.0f;
// Scale down overall velocity commands (0.0 to 1.0). Reduce to soften response.
static const float VEL_SCALE = 0.25f;

// LQR deadzone: angle errors within ±5° of (0,0,0) are zeroed (platform can't be perfectly level).
static const float LQR_DEADZONE_DEG = 0.0f;
static const float LQR_DEADZONE_RAD = LQR_DEADZONE_DEG * (PI / 180.0f);

static const float DELTA_DEADBAND_RAD   = 0.06f;
static const float DELTA_FULLSCALE_RAD  = 0.60f;

// Gain [vel/rad]: angle error -> velocity. Tune for your system.
static const float K_ROLL  = 1.0f;
static const float K_PITCH = 1.0f;
static const float K_YAW   = 1.0f;
static const float VEL_MAX = 5.0f;  // max |velocity| per motor
static const float DIR_TIME_CONSTANT_S   = 0.25f;
static const float SPEED_TIME_CONSTANT_S = 0.15f;
static const float SINE_PERIOD_S = 2.0f;
static const float SINE_AMPL     = 1.0f;
static const float V_MAX = SINE_AMPL * (TWO_PI / SINE_PERIOD_S);

static float axis_zero_rad = 0.0f;
static float dir_smoothed   = 0.0f;
static float speed_smoothed = 0.0f;

static float lpfToward(float current, float target, float tau_s, float dt_s) {
  if (tau_s <= 0.0f || dt_s < 0.0f) return target;
  float alpha = dt_s / (tau_s + dt_s);
  return current + alpha * (target - current);
}

static void deltaToTargets(float delta_rad, float* dir_target, float* speed_target) {
  if (fabsf(delta_rad) <= DELTA_DEADBAND_RAD) {
    *dir_target = 0.0f;
    *speed_target = 0.0f;
    return;
  }
  *dir_target = (delta_rad > 0.0f) ? 1.0f : -1.0f;
  float mag = fabsf(delta_rad);
  float frac = (mag - DELTA_DEADBAND_RAD) / (DELTA_FULLSCALE_RAD - DELTA_DEADBAND_RAD);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;
  *speed_target = frac;
}

void control_setAxisZero(float axis_zero_rad_in) {
  axis_zero_rad = axis_zero_rad_in;
}

// angle_rad - zero_rad -> velocity; deadband and saturate
static float angleToVel(float angle_rad, float zero_rad, float gain) {
  float delta = angle_rad - zero_rad;
  if (fabsf(delta) <= DELTA_DEADBAND_RAD) return 0.0f;
  float v = gain * delta;
  if (v > VEL_MAX)  v = VEL_MAX;
  if (v < -VEL_MAX) v = -VEL_MAX;
  return v;
}

void control_updateThree(float roll_rad, float pitch_rad, float yaw_rad,
                         float roll_zero, float pitch_zero, float yaw_zero,
                         float* v1, float* v2, float* v3) {
  *v1 = angleToVel(roll_rad,  roll_zero,  K_ROLL);
  *v2 = angleToVel(pitch_rad, pitch_zero, K_PITCH);
  *v3 = angleToVel(yaw_rad,   yaw_zero,   K_YAW);
}

float control_update(float axis_rad, float dt_s) {
  float delta = axis_rad - axis_zero_rad;
  if (dt_s < 0.0f) dt_s = 0.0f;
  if (dt_s > 0.1f) dt_s = 0.1f;

  float dir_target, speed_target;
  deltaToTargets(delta, &dir_target, &speed_target);

  dir_smoothed   = lpfToward(dir_smoothed,   dir_target,   DIR_TIME_CONSTANT_S,   dt_s);
  speed_smoothed = lpfToward(speed_smoothed, speed_target, SPEED_TIME_CONSTANT_S, dt_s);

  return dir_smoothed * speed_smoothed * V_MAX;
}

float control_getVMax(void) {
  return V_MAX;
}

// Body torques (roll, pitch, yaw) -> wheel torques T1, T2, T3. Saturate to ±IK_MAX_TORQUE.
static void ik(float roll_T, float pitch_T, float yaw_T,
               float* T1, float* T2, float* T3) {
  float t1 = (C1 * CA * roll_T) + (S1 * CA * pitch_T) + (SA * yaw_T);
  float t2 = (C2 * CA * roll_T) + (S2 * CA * pitch_T) + (SA * yaw_T);
  float t3 = (C3 * CA * roll_T) + (S3 * CA * pitch_T) + (SA * yaw_T);
  if (t1 > IK_MAX_TORQUE) t1 = IK_MAX_TORQUE;
  if (t1 < -IK_MAX_TORQUE) t1 = -IK_MAX_TORQUE;
  if (t2 > IK_MAX_TORQUE) t2 = IK_MAX_TORQUE;
  if (t2 < -IK_MAX_TORQUE) t2 = -IK_MAX_TORQUE;
  if (t3 > IK_MAX_TORQUE) t3 = IK_MAX_TORQUE;
  if (t3 < -IK_MAX_TORQUE) t3 = -IK_MAX_TORQUE;
  *T1 = t1;
  *T2 = t2;
  *T3 = t3;
}

void control_updateLQR(const float x[6], const float x_ref[6],
                        float* v1, float* v2, float* v3,
                        float* tau_roll_out, float* tau_pitch_out, float* tau_yaw_out) {
  // Error: x - x_ref
  float e[6];
  for (int i = 0; i < 6; i++)
    e[i] = x[i] - x_ref[i];

  // Deadzone around (0,0,0): zero angle errors within ±5° so we don't fight small level errors
  if (fabsf(e[0]) < LQR_DEADZONE_RAD) e[0] = 0.0f;
  if (fabsf(e[1]) < LQR_DEADZONE_RAD) e[1] = 0.0f;
  if (fabsf(e[2]) < LQR_DEADZONE_RAD) e[2] = 0.0f;

  // u = -K @ e  -> body torques [tau_roll, tau_pitch, tau_yaw]
  float tau_roll  = -(K_LQR[0][0]*e[0] + K_LQR[0][1]*e[1] + K_LQR[0][2]*e[2] +
                     K_LQR[0][3]*e[3] + K_LQR[0][4]*e[4] + K_LQR[0][5]*e[5]);
  float tau_pitch = -(K_LQR[1][0]*e[0] + K_LQR[1][1]*e[1] + K_LQR[1][2]*e[2] +
                     K_LQR[1][3]*e[3] + K_LQR[1][4]*e[4] + K_LQR[1][5]*e[5]);
  float tau_yaw   = -(K_LQR[2][0]*e[0] + K_LQR[2][1]*e[1] + K_LQR[2][2]*e[2] +
                     K_LQR[2][3]*e[3] + K_LQR[2][4]*e[4] + K_LQR[2][5]*e[5]);

#if LQR_REMAP_SWAP
  float tmp = tau_roll;
  tau_roll = tau_pitch;
  tau_pitch = tmp;
#endif

  if (tau_roll_out)  *tau_roll_out  = tau_roll;
  if (tau_pitch_out) *tau_pitch_out = tau_pitch;
  if (tau_yaw_out)   *tau_yaw_out   = tau_yaw;

  float T1, T2, T3;
  ik(tau_roll, tau_pitch, tau_yaw, &T1, &T2, &T3);

  float v1_out = K_TAU_TO_VEL * T1;
  float v2_out = K_TAU_TO_VEL * T2;
  float v3_out = K_TAU_TO_VEL * T3;
  if (v1_out > VEL_MAX_LQR) v1_out = VEL_MAX_LQR;
  if (v1_out < -VEL_MAX_LQR) v1_out = -VEL_MAX_LQR;
  if (v2_out > VEL_MAX_LQR) v2_out = VEL_MAX_LQR;
  if (v2_out < -VEL_MAX_LQR) v2_out = -VEL_MAX_LQR;
  if (v3_out > VEL_MAX_LQR) v3_out = VEL_MAX_LQR;
  if (v3_out < -VEL_MAX_LQR) v3_out = -VEL_MAX_LQR;

  v1_out *= VEL_SCALE;
  v2_out *= VEL_SCALE;
  v3_out *= VEL_SCALE;

  *v1 = v1_out;
  *v2 = v2_out;
  *v3 = v3_out;
}
