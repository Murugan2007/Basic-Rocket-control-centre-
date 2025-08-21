#pragma once

#include <Arduino.h>

class Motors {
public:
	void begin();
	// throttle: 0..1, controls base power
	// roll/pitch/yaw corrections: -1..+1 typical range
	void mixAndWrite(float throttle, float rollCtrl, float pitchCtrl, float yawCtrl);
	void writeAll(float duty01);

private:
	void setupMotor(uint8_t ledcChannel, uint8_t inAPin, uint8_t inBPin, bool pwmOnA);
	void writeMotor(uint8_t ledcChannel, float dutyNorm);
};

