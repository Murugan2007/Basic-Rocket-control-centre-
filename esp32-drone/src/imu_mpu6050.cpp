#include "imu_mpu6050.h"
#include "config.h"
#include <Wire.h>

// MPU6050 registers
#define MPU6050_ADDR 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_GYRO_CONFIG 0x1B

static bool i2cWriteByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  const size_t n = Wire.requestFrom((int)addr, (int)len, (int)true);
  if (n != len) {
    return false;
  }
  for (size_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

bool IMU_MPU6050::begin() {
  delay(50);
  // Wake up
  if (!i2cWriteByte(MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  delay(10);
  // Accel +- 4g (0x08) or +-2g (0x00). Use 2g for better resolution
  if (!i2cWriteByte(MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }
  // Gyro +- 500 dps (0x08) or 250 dps (0x00). Use 500 dps
  if (!i2cWriteByte(MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 0x08)) {
    return false;
  }
  delay(10);
  return true;
}

void IMU_MPU6050::calibrate(uint16_t numSamples) {
  long axSum = 0, aySum = 0, azSum = 0;
  long gxSum = 0, gySum = 0, gzSum = 0;
  for (uint16_t i = 0; i < numSamples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    readRaw(ax, ay, az, gx, gy, gz);
    axSum += ax; aySum += ay; azSum += az;
    gxSum += gx; gySum += gy; gzSum += gz;
    delay(2);
  }
  const float inv = 1.0f / numSamples;
  // Convert to g and dps scale factors
  const float accelScale = 1.0f / 16384.0f; // 2g
  const float gyroScale = 1.0f / 65.5f;     // 500 dps

  accelBiasX = (axSum * inv) * accelScale;
  accelBiasY = (aySum * inv) * accelScale;
  // For Z, at rest we expect +1g. Bias so that az - bias = +1g when level
  accelBiasZ = (azSum * inv) * accelScale - 1.0f;

  gyroBiasX = (gxSum * inv) * gyroScale;
  gyroBiasY = (gySum * inv) * gyroScale;
  gyroBiasZ = (gzSum * inv) * gyroScale;
}

void IMU_MPU6050::zeroAngles() {
  rollDeg = 0.0f;
  pitchDeg = 0.0f;
}

void IMU_MPU6050::readRaw(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  uint8_t buf[14];
  if (!i2cReadBytes(MPU6050_ADDR, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
    ax = ay = az = gx = gy = gz = 0;
    return;
  }
  ax = (int16_t)((buf[0] << 8) | buf[1]);
  ay = (int16_t)((buf[2] << 8) | buf[3]);
  az = (int16_t)((buf[4] << 8) | buf[5]);
  gx = (int16_t)((buf[8] << 8) | buf[9]);
  gy = (int16_t)((buf[10] << 8) | buf[11]);
  gz = (int16_t)((buf[12] << 8) | buf[13]);
}

void IMU_MPU6050::update() {
  int16_t axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw;
  readRaw(axRaw, ayRaw, azRaw, gxRaw, gyRaw, gzRaw);

  const float accelScale = 1.0f / 16384.0f; // 2g
  const float gyroScale = 1.0f / 65.5f;     // 500 dps

  const float ax_g = axRaw * accelScale - accelBiasX;
  const float ay_g = ayRaw * accelScale - accelBiasY;
  const float az_g = azRaw * accelScale - accelBiasZ;

  // Gyro rates in dps minus bias
  gyroX_dps = gxRaw * gyroScale - gyroBiasX;
  gyroY_dps = gyRaw * gyroScale - gyroBiasY;
  gyroZ_dps = gzRaw * gyroScale - gyroBiasZ;

  // dt derived from global loop period for stability
  const float dt = LOOP_PERIOD_S;

  // Integrate gyro into angles
  const float rollGyro = rollDeg + gyroX_dps * dt;
  const float pitchGyro = pitchDeg + gyroY_dps * dt;

  // Compute roll/pitch from accelerometer
  const float rollAcc = atan2f(ay_g, az_g) * 180.0f / PI;
  const float pitchAcc = atan2f(-ax_g, sqrtf(ay_g * ay_g + az_g * az_g)) * 180.0f / PI;

  // Complementary filter
  rollDeg = COMP_ALPHA * rollGyro + (1.0f - COMP_ALPHA) * rollAcc;
  pitchDeg = COMP_ALPHA * pitchGyro + (1.0f - COMP_ALPHA) * pitchAcc;
}

