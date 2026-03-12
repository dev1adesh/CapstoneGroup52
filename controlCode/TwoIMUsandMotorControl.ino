#include <Arduino.h>
#include <Wire.h>

#include "ODriveCAN.h"
#include <FlexCAN_T4.h>
#include "ODriveFlexCAN.hpp"
struct ODriveStatus; // Teensy compile hack

// IMU A (0x4A) - SparkFun BNO08x Cortex library
#include "SparkFun_BNO08x_Arduino_Library.h"
BNO08x imuA;

// IMU B (0x4B) - SparkFun BNO080 legacy library
#include "SparkFun_BNO080_Arduino_Library.h"
BNO080 imuB;

// ===================== USER SETTINGS =====================

// ---- CAN ----
#define CAN_BAUDRATE   250000
#define ODRV0_NODE_ID  0
#define ODRV1_NODE_ID  1
#define ODRV2_NODE_ID  2

// ---- IMU addresses (both on Wire2) ----
#define IMU_A_ADDR 0x4A
#define IMU_B_ADDR 0x4B

// Choose which axis of IMU A controls motion
enum ImuAxis { AXIS_PITCH, AXIS_ROLL, AXIS_YAW };
static constexpr ImuAxis AXIS_FOR_CONTROL = AXIS_PITCH;

// Stop only when near startup neutral
static constexpr float DELTA_DEADBAND_RAD   = 0.06f; // ~3.4 deg
static constexpr float DELTA_FULLSCALE_RAD  = 0.60f; // ~34 deg for full speed

// Smoothing
static constexpr float DIR_TIME_CONSTANT_S   = 0.25f;
static constexpr float SPEED_TIME_CONSTANT_S = 0.15f;

// Define max speed to match old sine peak speed:
static constexpr float SINE_PERIOD_S = 2.0f;
static constexpr float SINE_AMPL     = 1.0f;

// Presence via heartbeat
static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 500;

// Startup calibration time for IMU neutral
static constexpr uint32_t IMU_ZERO_CAL_MS = 1200;

// Old sine peak velocity:
static constexpr float V_MAX = SINE_AMPL * (TWO_PI / SINE_PERIOD_S);

// =========================================================

// -------------------- CAN interface --------------------
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;

// -------------------- ODrive user data --------------------
struct ODriveUserData {
  Heartbeat_msg_t last_heartbeat;
  bool received_heartbeat = false;
  uint32_t last_hb_ms = 0;
  bool configured_velocity_mode = false;
};

ODriveUserData u0, u1, u2;

// -------------------- ODrive instances --------------------
ODriveCAN odrv0(wrap_can_intf(can_intf), ODRV0_NODE_ID);
ODriveCAN odrv1(wrap_can_intf(can_intf), ODRV1_NODE_ID);
ODriveCAN odrv2(wrap_can_intf(can_intf), ODRV2_NODE_ID);
ODriveCAN* odrives[] = { &odrv0, &odrv1, &odrv2 };

// -------------------- Control state --------------------
static float dir_smoothed   = 0.0f;   // -1..+1
static float speed_smoothed = 0.0f;   // 0..1
static uint32_t last_update_ms = 0;

static float last_axis_val = 0.0f;
static float axis_zero = 0.0f;

// For printing IMU A/B euler
static float yawA=0, pitchA=0, rollA=0;
static float yawB=0, pitchB=0, rollB=0;

// -------------------- Forward declarations --------------------
void onCanMessage(const CanMsg& msg);
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data);

// -------------------- CAN setup --------------------
bool setupCan() {
  can_intf.begin();
  can_intf.setBaudRate(CAN_BAUDRATE);
  can_intf.setMaxMB(16);
  can_intf.enableFIFO();
  can_intf.enableFIFOInterrupt();
  can_intf.onReceive(onCanMessage);
  return true;
}

// -------------------- Callbacks --------------------
void onHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  auto* u = static_cast<ODriveUserData*>(user_data);
  u->last_heartbeat = msg;
  u->received_heartbeat = true;
  u->last_hb_ms = millis();
}

void onCanMessage(const CanMsg& msg) {
  for (auto* odrive : odrives) {
    onReceive(msg, *odrive);
  }
}

// -------------------- Helpers --------------------
static bool isPresent(const ODriveUserData& u) {
  if (!u.received_heartbeat) return false;
  return (uint32_t)(millis() - u.last_hb_ms) <= HEARTBEAT_TIMEOUT_MS;
}

static float lpfToward(float current, float target, float tau_s, float dt_s) {
  float alpha = dt_s / (tau_s + dt_s);
  return current + alpha * (target - current);
}

// IMU A (BNO08x) event updates internal state
static bool readImuA_Axis(float& axis_out) {
  if (!imuA.getSensorEvent()) return false;

  float roll  = imuA.getRoll();
  float pitch = imuA.getPitch();
  float yaw   = imuA.getYaw();

  // Save for printing too
  rollA  = roll  * 180.0f / PI;
  pitchA = pitch * 180.0f / PI;
  yawA   = yaw   * 180.0f / PI;

  switch (AXIS_FOR_CONTROL) {
    case AXIS_ROLL:  axis_out = roll;  break;
    case AXIS_PITCH: axis_out = pitch; break;
    case AXIS_YAW:   axis_out = yaw;   break;
  }
  return true;
}

// IMU B (BNO080 legacy) quaternion -> euler for printing
static void quatToEulerDeg_B(float qw, float qx, float qy, float qz,
                             float &yawDeg, float &pitchDeg, float &rollDeg)
{
  float yaw = atan2f(2.0f * (qw*qz + qx*qy),
                     1.0f - 2.0f * (qy*qy + qz*qz));

  float s = 2.0f * (qw*qy - qz*qx);
  if (s >  1.0f) s =  1.0f;
  if (s < -1.0f) s = -1.0f;
  float pitch = asinf(s);

  float roll = atan2f(2.0f * (qw*qx + qy*qz),
                      1.0f - 2.0f * (qx*qx + qy*qy));

  yawDeg   = yaw   * 180.0f / PI;
  pitchDeg = pitch * 180.0f / PI;
  rollDeg  = roll  * 180.0f / PI;
}

// delta -> targets: dir in {-1,0,+1}, speed in [0,1]
static void deltaToTargets(float delta_rad, float& dir_target, float& speed_target) {
  if (fabsf(delta_rad) <= DELTA_DEADBAND_RAD) {
    dir_target = 0.0f;
    speed_target = 0.0f;
    return;
  }

  dir_target = (delta_rad > 0.0f) ? +1.0f : -1.0f;

  float mag = fabsf(delta_rad);
  float frac = (mag - DELTA_DEADBAND_RAD) / (DELTA_FULLSCALE_RAD - DELTA_DEADBAND_RAD);
  if (frac < 0.0f) frac = 0.0f;
  if (frac > 1.0f) frac = 1.0f;

  speed_target = frac;
}

// Ensure CLOSED LOOP + VELOCITY_CONTROL + PASSTHROUGH once per node
static void ensureVelocityMode(ODriveCAN& odrv, ODriveUserData& u) {
  if (!isPresent(u)) return;

  if (u.last_heartbeat.Axis_State != ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv.clearErrors();
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    return;
  }

  if (!u.configured_velocity_mode) {
    odrv.setControllerMode(ODriveControlMode::CONTROL_MODE_VELOCITY_CONTROL,
                           ODriveInputMode::INPUT_MODE_PASSTHROUGH);
    u.configured_velocity_mode = true;
  }
}

static void cmdVelocity(ODriveCAN& odrv, float vel) {
  odrv.setVelocity(vel);
}

// -------------------- IMU Zero Calibration (IMU A) --------------------
static void calibrateImuZero_A() {
  Serial.println("Calibrating IMU A neutral (hold at stop orientation)...");
  uint32_t t0 = millis();
  double sum = 0.0;
  uint32_t count = 0;

  while (millis() - t0 < IMU_ZERO_CAL_MS) {
    float a;
    if (readImuA_Axis(a)) {
      last_axis_val = a;
      sum += a;
      count++;
    }
    delay(5);
  }

  axis_zero = (count > 0) ? (float)(sum / (double)count) : last_axis_val;

  Serial.print("axis_zero (rad) = ");
  Serial.println(axis_zero, 6);
}

// -------------------- Nice printing helpers --------------------
static void printHeader() {
  Serial.println();
  Serial.println("====================================================================================================");
  Serial.println("                 IMU A (0x4A) controls motion (axis tilt -> velocity) | IMU B (0x4B) monitor");
  Serial.println("====================================================================================================");
  Serial.println("   IMU A (deg)                 IMU B (deg)                 Control / Status");
  Serial.println("  Yaw   Pitch   Roll         Yaw   Pitch   Roll          delta(rad)   vel_cmd   n0 n1 n2");
  Serial.println("----------------------------------------------------------------------------------------------------");
}

// fixed-width number printing without printf (stable spacing)
static void printFixed(float v, int width, int prec) {
  char buf[24];
  dtostrf(v, width, prec, buf); // AVR style, but Teensy supports it too
  Serial.print(buf);
}

static void printRow(float delta, float vel_cmd, bool n0, bool n1, bool n2) {
  // IMU A
  printFixed(yawA,   6, 1); Serial.print(" ");
  printFixed(pitchA, 7, 1); Serial.print(" ");
  printFixed(rollA,  7, 1); Serial.print("     ");

  // IMU B
  printFixed(yawB,   6, 1); Serial.print(" ");
  printFixed(pitchB, 7, 1); Serial.print(" ");
  printFixed(rollB,  7, 1); Serial.print("     ");

  // Control
  printFixed(delta,   9, 3); Serial.print("  ");
  printFixed(vel_cmd, 8, 3); Serial.print("   ");

  Serial.print(n0 ? "1" : "0"); Serial.print("  ");
  Serial.print(n1 ? "1" : "0"); Serial.print("  ");
  Serial.println(n2 ? "1" : "0");
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  last_update_ms = millis();

  Serial.println("=== Dual IMU + ODrive velocity control ===");
  Serial.print("V_MAX = ");
  Serial.println(V_MAX, 4);

  // I2C on Wire2
  Wire2.begin();
  Wire2.setClock(400000);
  delay(50);

  // ---- IMU A (BNO08x Cortex) @ 0x4A ----
  Serial.println("Init IMU A (BNO08x) @ 0x4A...");
  if (!imuA.begin(IMU_A_ADDR, Wire2)) {
    Serial.println("IMU A not found at 0x4A");
    while (true) delay(100);
  }
  if (!imuA.enableRotationVector(50)) {
    Serial.println("IMU A: enableRotationVector failed");
    while (true) delay(100);
  }
  delay(100);

  // ---- IMU B (BNO080 legacy) @ 0x4B ----
  Serial.println("Init IMU B (BNO080) @ 0x4B...");
  if (!imuB.begin(IMU_B_ADDR, Wire2)) { // your library expects (addr, wire)
    Serial.println("IMU B not found at 0x4B");
    while (true) delay(100);
  }
  // Many SparkFun BNO080 builds interpret this as ms: 20ms ≈ 50Hz.
  imuB.enableRotationVector(20);
  delay(100);

  // Calibrate neutral for IMU A
  calibrateImuZero_A();

  // ODrive callbacks
  odrv0.onStatus(onHeartbeat, &u0);
  odrv1.onStatus(onHeartbeat, &u1);
  odrv2.onStatus(onHeartbeat, &u2);

  // CAN init
  if (!setupCan()) {
    Serial.println("CAN init failed");
    while (true) {}
  }

  Serial.println("Listening for ODrive heartbeats for 1.5s...");
  uint32_t start = millis();
  while (millis() - start < 1500) {
    pumpEvents(can_intf);
    delay(10);
  }

  Serial.print("Detected nodes: n0="); Serial.print(isPresent(u0));
  Serial.print(" n1="); Serial.print(isPresent(u1));
  Serial.print(" n2="); Serial.println(isPresent(u2));

  printHeader();
}

// -------------------- Loop --------------------
void loop() {
  pumpEvents(can_intf);

  bool n0 = isPresent(u0);
  bool n1 = isPresent(u1);
  bool n2 = isPresent(u2);

  if (n0) ensureVelocityMode(odrv0, u0);
  if (n1) ensureVelocityMode(odrv1, u1);
  if (n2) ensureVelocityMode(odrv2, u2);

  // dt for smoothing
  uint32_t now = millis();
  float dt = 0.001f * (float)(now - last_update_ms);
  last_update_ms = now;
  if (dt < 0.0f) dt = 0.0f;
  if (dt > 0.1f) dt = 0.1f;

  // ---- Read IMU A (control) ----
  float axis_raw;
  if (readImuA_Axis(axis_raw)) last_axis_val = axis_raw;
  else axis_raw = last_axis_val;

  float delta = axis_raw - axis_zero;

  float dir_target = 0.0f, speed_target = 0.0f;
  deltaToTargets(delta, dir_target, speed_target);

  // Smooth targets
  dir_smoothed   = lpfToward(dir_smoothed,   dir_target,   DIR_TIME_CONSTANT_S,   dt);
  speed_smoothed = lpfToward(speed_smoothed, speed_target, SPEED_TIME_CONSTANT_S, dt);

  float vel_cmd = dir_smoothed * speed_smoothed * V_MAX;

  // Node mapping: node0 opposite node1/node2
  if (n0) cmdVelocity(odrv0, +vel_cmd);
  if (n1) cmdVelocity(odrv1, -vel_cmd);
  if (n2) cmdVelocity(odrv2, -vel_cmd);

  // ---- Read IMU B (monitor) ----
  if (imuB.dataAvailable()) {
    float qw = imuB.getQuatReal();
    float qx = imuB.getQuatI();
    float qy = imuB.getQuatJ();
    float qz = imuB.getQuatK();
    quatToEulerDeg_B(qw, qx, qy, qz, yawB, pitchB, rollB);
  }

  // ---- Print nicely at ~5 Hz ----
  static uint32_t last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    printRow(delta, vel_cmd, n0, n1, n2);
  }
}