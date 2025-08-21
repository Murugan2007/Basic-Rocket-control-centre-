#include <Arduino.h>
#include <Wire.h>
#include "config.h"
#include "imu_mpu6050.h"
#include "pid.h"
#include "ibus.h"
#include "motors.h"

static IMU_MPU6050 imu;
static IBUSReceiver ibus;
static Motors motors;

static PIDController pidRoll(
    PID_ROLL_KP,
    PID_ROLL_KI,
    PID_ROLL_KD,
    PID_I_MAX,
    PID_OUT_MAX
);

static PIDController pidPitch(
    PID_PITCH_KP,
    PID_PITCH_KI,
    PID_PITCH_KD,
    PID_I_MAX,
    PID_OUT_MAX
);

static PIDController pidYaw(
    PID_YAW_KP,
    PID_YAW_KI,
    PID_YAW_KD,
    PID_I_MAX,
    PID_OUT_MAX
);

static bool armed = false;
static uint32_t lastLoopMicros = 0;

static inline float mapChannelToRange(uint16_t ch, float outMin, float outMax) {
  ch = constrain(ch, 1000, 2000);
  float t = (float)(ch - 1500) / 500.0f; // -1..+1
  return outMin + (t + 1.0f) * 0.5f * (outMax - outMin);
}

static inline float mapStickCentered(uint16_t ch, float absMax) {
  ch = constrain(ch, 1000, 2000);
  float t = (float)(ch - 1500) / 500.0f; // -1..+1
  return t * absMax;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("ESP32 Drone starting...");

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);

  if (!imu.begin()) {
    Serial.println("IMU init failed! Check wiring.");
    delay(2000);
  }

  imu.calibrate(CALIBRATION_SAMPLES);
  Serial.println("IMU calibrated.");

  ibus.begin(IBUS_UART_NUM, IBUS_RX_PIN, IBUS_TX_PIN, IBUS_BAUD);

  motors.begin();

  lastLoopMicros = micros();
}

void loop() {
  // Maintain control loop frequency
  const uint32_t nowMicros = micros();
  const float dt = (nowMicros - lastLoopMicros) * 1e-6f;
  if (dt < LOOP_PERIOD_S) {
    return;
  }
  lastLoopMicros = nowMicros;

  ibus.update();
  const bool rcAvailable = ibus.isFrameFresh(IBUS_FRESH_TIMEOUT_MS);

  // Read IMU
  imu.update();
  const float angleRollDeg = imu.getRollDeg();
  const float anglePitchDeg = imu.getPitchDeg();
  const float gyroX = imu.getGyroXDegPerSec();
  const float gyroY = imu.getGyroYDegPerSec();
  const float gyroZ = imu.getGyroZDegPerSec();

  // Defaults if no RC
  uint16_t chThrottle = 1000;
  uint16_t chRoll = 1500;
  uint16_t chPitch = 1500;
  uint16_t chYaw = 1500;
  uint16_t chArm = 1000;

  if (rcAvailable) {
    chThrottle = ibus.getChannel(CH_THROTTLE);
    chRoll = ibus.getChannel(CH_ROLL);
    chPitch = ibus.getChannel(CH_PITCH);
    chYaw = ibus.getChannel(CH_YAW);
    chArm = ibus.getChannel(CH_ARM);
  }

  // Arming logic
  const bool wantArm = chArm > 1500 && chThrottle < 1050 && rcAvailable;
  if (wantArm && !armed) {
    pidRoll.reset();
    pidPitch.reset();
    pidYaw.reset();
    imu.zeroAngles(); // set current attitude as level reference
    armed = true;
  } else if ((!rcAvailable) || chArm < 1200) {
    armed = false;
  }

  if (!armed) {
    motors.writeAll(0);
    return;
  }

  // Setpoints
  const float throttle = mapChannelToRange(chThrottle, 0.0f, 1.0f);
  const float rollSetDeg = mapStickCentered(chRoll, ANGLE_SETPOINT_MAX_DEG);
  const float pitchSetDeg = mapStickCentered(chPitch, ANGLE_SETPOINT_MAX_DEG);
  const float yawRateSetDeg = mapStickCentered(chYaw, YAW_RATE_SETPOINT_MAX_DPS);

  // Angle PID for roll/pitch, using gyro as derivative (D on measurement)
  const float rollError = rollSetDeg - angleRollDeg;
  const float pitchError = pitchSetDeg - anglePitchDeg;

  const float rollCtrl = pidRoll.updateWithDerivativeOnMeasurement(
      rollError,
      -gyroX,
      dt
  );

  const float pitchCtrl = pidPitch.updateWithDerivativeOnMeasurement(
      pitchError,
      -gyroY,
      dt
  );

  // Yaw: rate PID on gyro Z
  const float yawRateError = yawRateSetDeg - gyroZ;
  const float yawCtrl = pidYaw.updateWithDerivativeOnMeasurement(
      yawRateError,
      0.0f,
      dt
  );

  // Motor mixing (X configuration)
  motors.mixAndWrite(throttle, rollCtrl, pitchCtrl, yawCtrl);
}

