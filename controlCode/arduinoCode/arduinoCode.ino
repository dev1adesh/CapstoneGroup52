/*
 * Modular dual-IMU + ODrive: Motor 1=roll, Motor 2=pitch, Motor 3=yaw (IMU B).
 * Positive angle -> CW torque; negative angle -> CCW.
 *
 * USE_USER_LEAN_REFERENCE: 1 = LQR reference from IMU B (user lean); 0 = chassis only, ref = upright (0,0,0).
 * INVERT_ROLL: 1 = flip sign of roll (state and ref) so positive roll direction matches hardware.
 * RUN_MOTOR_SPIN_CHECK: 1 = at startup, spin each motor + then - to verify direction vs simulation.
 */
#define USE_USER_LEAN_REFERENCE 0
#define INVERT_ROLL 1
#define RUN_MOTOR_SPIN_CHECK 0

#include "sensor_helper.h"
#include "control_helper.h"
#include "motor_control_helper.h"

static uint32_t last_update_ms = 0;

static void printHeader(void) {
  Serial.println("IMU_A: Roll   Pitch  Yaw   | omega_R   omega_P   omega_Y (rad/s) | tau_R   tau_P   tau_Y  | n0 n1 n2  | T1    T2    T3 (Nm)");
}

static void printRow(float rollA, float pitchA, float yawA, float omega_r, float omega_p, float omega_y, float tau_r, float tau_p, float tau_y, float v1, float v2, float v3) {
  Serial.print(rollA, 2);  Serial.print(" ");
  Serial.print(pitchA, 2); Serial.print(" ");
  Serial.print(yawA, 2);  Serial.print("  | ");
  Serial.print(omega_r, 4); Serial.print(" ");
  Serial.print(omega_p, 4); Serial.print(" ");
  Serial.print(omega_y, 4); Serial.print("  | ");
  Serial.print(tau_r, 3); Serial.print(" ");
  Serial.print(tau_p, 3); Serial.print(" ");
  Serial.print(tau_y, 3); Serial.print("  | ");
  Serial.print(motor_isNodePresent(0) ? "1" : "0"); Serial.print(" ");
  Serial.print(motor_isNodePresent(1) ? "1" : "0"); Serial.print(" ");
  Serial.print(motor_isNodePresent(2) ? "1" : "0"); Serial.print("  | ");
  Serial.print(v1, 3); Serial.print(" ");
  Serial.print(v2, 3); Serial.print(" ");
  Serial.println(v3, 3);
}

void setup() {
  Serial.begin(115200);
  for (int i = 0; i < 30 && !Serial; ++i) delay(100);
  delay(200);

  last_update_ms = millis();
  Serial.println("=== Modular dual IMU + ODrive (sensor -> control -> motor) ===");

  if (!sensor_init()) {
    Serial.println("Sensor init failed (IMU A or B not found)");
    while (true) delay(100);
  }
  Serial.println("IMU A and B OK");

  sensor_calibrateZero();  // hold still: this orientation becomes (0,0,0) for both IMUs
  control_resetIntegrals();
  float rz, pz, yz;
  sensor_getEulerZeroRad(&rz, &pz, &yz);
  Serial.print("IMU B zero (rad) roll="); Serial.print(rz, 4);
  Serial.print(" pitch="); Serial.print(pz, 4);
  Serial.print(" yaw="); Serial.println(yz, 4);
  sensor_getPlatformZeroRad(&rz, &pz, &yz);
  Serial.print("Platform zero (rad) roll="); Serial.print(rz, 4);
  Serial.print(" pitch="); Serial.print(pz, 4);
  Serial.print(" yaw="); Serial.println(yz, 4);

  if (!motor_init()) {
    Serial.println("Motor init failed");
    while (true) delay(100);
  }

  Serial.println("Listening for ODrive heartbeats 1.5s...");
  uint32_t start = millis();
  while (millis() - start < 1500) {
    motor_pumpEvents();
    delay(10);
  }
  Serial.print("Nodes: n0="); Serial.print(motor_isNodePresent(0));
  Serial.print(" n1="); Serial.print(motor_isNodePresent(1));
  Serial.print(" n2="); Serial.println(motor_isNodePresent(2));
  motor_sendTorques(0.0f, 0.0f, 0.0f);  // hold motors at zero until control loop starts

#if RUN_MOTOR_SPIN_CHECK
  {
    const float SPIN_VEL = 1.0f;
    const uint32_t SPIN_MS = 5000;
    const uint32_t PRINT_INTERVAL_MS = 200;
    printHeader();
    struct { const char* msg; float v1; float v2; float v3; } phases[] = {
      { "Setting motor 1 positive",  SPIN_VEL,  0.0f,  0.0f },
      { "Setting motor 1 negative", -SPIN_VEL,  0.0f,  0.0f },
      { "Setting motor 2 positive",  0.0f, SPIN_VEL,  0.0f },
      { "Setting motor 2 negative",  0.0f,-SPIN_VEL,  0.0f },
      { "Setting motor 3 positive",  0.0f,  0.0f, SPIN_VEL },
      { "Setting motor 3 negative",  0.0f,  0.0f,-SPIN_VEL },
    };
    for (int p = 0; p < 6; p++) {
      Serial.println(phases[p].msg);
      uint32_t phase_start = millis();
      uint32_t last_print = 0;
      while (millis() - phase_start < SPIN_MS) {
        motor_pumpEvents();
        float dummy;
        sensor_readImuA(&dummy);
        sensor_readControlAxis(&dummy);
        motor_sendVelocities(phases[p].v1, phases[p].v2, phases[p].v3);
        if (millis() - last_print >= PRINT_INTERVAL_MS) {
          last_print = millis();
          float x[6];
          sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
          float yawA, pitchA, rollA;
          sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
          float pr0, pp0, py0;
          sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
          const float rad2deg = 180.0f / PI;
          float rollA_z  = rollA  - (pr0 * rad2deg);
          float pitchA_z = pitchA - (pp0 * rad2deg);
          float yawA_z   = yawA   - (py0 * rad2deg);
#if INVERT_ROLL
          rollA_z = -rollA_z;
#endif
          printRow(rollA_z, pitchA_z, yawA_z, x[3], x[4], x[5], 0.0f, 0.0f, 0.0f, phases[p].v1, phases[p].v2, phases[p].v3);
        }
        delay(10);
      }
      motor_sendVelocities(0.0f, 0.0f, 0.0f);
      delay(400);
    }
    Serial.println("Motor spin direction check done.");
    delay(500);
  }
#endif

#if USE_USER_LEAN_REFERENCE
  Serial.println("Mode: LQR with user lean (x_ref from IMU B)");
#else
  Serial.println("Mode: LQR chassis only (x_ref = 0, balance upright)");
#endif
#if INVERT_ROLL
  Serial.println("Roll: inverted (sign flipped for controller)");
#else
  Serial.println("Roll: normal sign");
#endif
  printHeader();
}

void loop() {
  motor_pumpEvents();

  last_update_ms = millis();

  float dummy;
  sensor_readImuA(&dummy);           // update IMU A display
  sensor_readControlAxis(&dummy);     // update IMU B (roll, pitch, yaw) and display

  // LQR: state x from IMU A (platform); x_ref = user lean (IMU B) or upright (0) per flag.
  // Subtract zeros so orientation at reset/calibration is (0,0,0).
  float x[6], x_ref[6];
  sensor_getImuA_StateRad(&x[0], &x[1], &x[2], &x[3], &x[4], &x[5]);
  float pr0, pp0, py0;
  sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
  x[0] -= pr0;
  x[1] -= pp0;
  x[2] -= py0;

#if USE_USER_LEAN_REFERENCE
  float rollB, pitchB, yawB;
  sensor_getImuB_EulerRad(&rollB, &pitchB, &yawB);
  float br0, bp0, by0;
  sensor_getEulerZeroRad(&br0, &bp0, &by0);
  x_ref[0] = rollB - br0;
  x_ref[1] = pitchB - bp0;
  x_ref[2] = yawB - by0;
#else
  x_ref[0] = 0.0f;
  x_ref[1] = 0.0f;
  x_ref[2] = 0.0f;
#endif
  x_ref[3] = 0.0f;
  x_ref[4] = 0.0f;
  x_ref[5] = 0.0f;

#if INVERT_ROLL
  x[0] = -x[0];
  x[3] = -x[3];   // omega_roll
  x_ref[0] = -x_ref[0];
#endif

  float t1, t2, t3, tau_roll, tau_pitch, tau_yaw;
  control_updateLQR(x, x_ref, &t1, &t2, &t3, &tau_roll, &tau_pitch, &tau_yaw);

  motor_sendTorques(t1, t2, t3);

  static uint32_t last_print = 0;
  if (millis() - last_print > 200) {
    last_print = millis();
    float yawA, pitchA, rollA;
    sensor_getImuA_EulerDeg(&yawA, &pitchA, &rollA);
    float pr0, pp0, py0;
    sensor_getPlatformZeroRad(&pr0, &pp0, &py0);
    const float rad2deg = 180.0f / PI;
    float rollA_z  = rollA  - (pr0 * rad2deg);
    float pitchA_z = pitchA - (pp0 * rad2deg);
    float yawA_z   = yawA   - (py0 * rad2deg);
#if INVERT_ROLL
    rollA_z = -rollA_z;  // display matches what controller sees
#endif
    printRow(rollA_z, pitchA_z, yawA_z, x[3], x[4], x[5], tau_roll, tau_pitch, tau_yaw, t1, t2, t3);
  }
}
