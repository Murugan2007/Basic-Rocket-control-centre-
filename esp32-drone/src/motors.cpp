#include "motors.h"
#include "config.h"

void Motors::begin() {
	// Configure LEDC PWM
	ledcSetup(M1_LEDC_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
	ledcSetup(M2_LEDC_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
	ledcSetup(M3_LEDC_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);
	ledcSetup(M4_LEDC_CH, LEDC_FREQ_HZ, LEDC_RES_BITS);

	setupMotor(M1_LEDC_CH, M1_INA_PIN, M1_INB_PIN, true);
	setupMotor(M2_LEDC_CH, M2_INA_PIN, M2_INB_PIN, true);
	setupMotor(M3_LEDC_CH, M3_INA_PIN, M3_INB_PIN, true);
	setupMotor(M4_LEDC_CH, M4_INA_PIN, M4_INB_PIN, true);

	writeAll(0.0f);
}

void Motors::setupMotor(uint8_t ledcChannel, uint8_t inAPin, uint8_t inBPin, bool pwmOnA) {
	pinMode(inAPin, OUTPUT);
	pinMode(inBPin, OUTPUT);
	if (pwmOnA) {
		digitalWrite(inBPin, LOW);
		ledcAttachPin(inAPin, ledcChannel);
	} else {
		digitalWrite(inAPin, LOW);
		ledcAttachPin(inBPin, ledcChannel);
	}
}

static inline float constrain01(float v) {
	if (v < 0.0f) return 0.0f;
	if (v > 1.0f) return 1.0f;
	return v;
}

void Motors::writeMotor(uint8_t ledcChannel, float dutyNorm) {
	dutyNorm = constrain01(dutyNorm);
	const uint32_t maxDuty = (1u << LEDC_RES_BITS) - 1u;
	const uint32_t duty = (uint32_t)(dutyNorm * maxDuty);
	ledcWrite(ledcChannel, duty);
}

void Motors::writeAll(float duty01) {
	writeMotor(M1_LEDC_CH, duty01);
	writeMotor(M2_LEDC_CH, duty01);
	writeMotor(M3_LEDC_CH, duty01);
	writeMotor(M4_LEDC_CH, duty01);
}

void Motors::mixAndWrite(float throttle, float rollCtrl, float pitchCtrl, float yawCtrl) {
	// Mix for X quad:
	// M1 Front Left (CCW), M2 Front Right (CW), M3 Rear Right (CCW), M4 Rear Left (CW)
	// Using throttle base plus corrections. yawCtrl assumed positive for CCW torque.
	const float m1 = throttle + pitchCtrl + rollCtrl + yawCtrl;
	const float m2 = throttle + pitchCtrl - rollCtrl - yawCtrl;
	const float m3 = throttle - pitchCtrl - rollCtrl + yawCtrl;
	const float m4 = throttle - pitchCtrl + rollCtrl - yawCtrl;

	writeMotor(M1_LEDC_CH, m1);
	writeMotor(M2_LEDC_CH, m2);
	writeMotor(M3_LEDC_CH, m3);
	writeMotor(M4_LEDC_CH, m4);
}

