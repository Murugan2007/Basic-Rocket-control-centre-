#pragma once

class PIDController {
public:
  PIDController(float kp, float ki, float kd, float iMax, float outMax)
      : kp(kp), ki(ki), kd(kd), iMax(iMax), outMax(outMax) {}

  void reset() {
    integral = 0.0f;
    prevError = 0.0f;
    first = true;
  }

  float updateWithDerivativeOnMeasurement(float error, float dMeasurement, float dt) {
    if (dt <= 0.0f) return 0.0f;
    integral += error * dt;
    if (integral > iMax) integral = iMax;
    if (integral < -iMax) integral = -iMax;

    const float pTerm = kp * error;
    const float iTerm = ki * integral;
    const float dTerm = kd * (-dMeasurement); // derivative on measurement

    float out = pTerm + iTerm + dTerm;
    if (out > outMax) out = outMax;
    if (out < -outMax) out = -outMax;
    prevError = error;
    first = false;
    return out;
  }

private:
  float kp, ki, kd;
  float iMax, outMax;
  float integral = 0.0f;
  float prevError = 0.0f;
  bool first = true;
};

