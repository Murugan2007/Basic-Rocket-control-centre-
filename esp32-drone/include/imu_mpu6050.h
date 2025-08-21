#pragma once

#include <Arduino.h>

class IMU_MPU6050 {
public:
  bool begin();
  void calibrate(uint16_t numSamples);
  void zeroAngles();
  void update();

  float getRollDeg() const { return rollDeg; }
  float getPitchDeg() const { return pitchDeg; }

  float getGyroXDegPerSec() const { return gyroX_dps; }
  float getGyroYDegPerSec() const { return gyroY_dps; }
  float getGyroZDegPerSec() const { return gyroZ_dps; }

private:
  void readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);

  float accelBiasX = 0.0f;
  float accelBiasY = 0.0f;
  float accelBiasZ = 0.0f;
  float gyroBiasX = 0.0f;
  float gyroBiasY = 0.0f;
  float gyroBiasZ = 0.0f;

  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;

  float gyroX_dps = 0.0f;
  float gyroY_dps = 0.0f;
  float gyroZ_dps = 0.0f;
};

