#pragma once

#include <Arduino.h>

// I2C pins for ESP32 DevKit v1
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 21
#endif

#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 22
#endif

#ifndef I2C_CLOCK_HZ
#define I2C_CLOCK_HZ 400000
#endif

// Control loop frequency
#define LOOP_HZ 250.0f
#define LOOP_PERIOD_S (1.0f/LOOP_HZ)

// Calibration
#define CALIBRATION_SAMPLES 2000

// Complementary filter
#define COMP_ALPHA 0.98f

// PID gains
#define PID_I_MAX 0.5f
#define PID_OUT_MAX 0.6f

#define PID_ROLL_KP 0.08f
#define PID_ROLL_KI 0.04f
#define PID_ROLL_KD 0.002f

#define PID_PITCH_KP 0.08f
#define PID_PITCH_KI 0.04f
#define PID_PITCH_KD 0.002f

#define PID_YAW_KP 0.12f
#define PID_YAW_KI 0.05f
#define PID_YAW_KD 0.0f

// Setpoint ranges
#define ANGLE_SETPOINT_MAX_DEG 25.0f
#define YAW_RATE_SETPOINT_MAX_DPS 180.0f

// IBUS configuration (FlySky)
#define IBUS_UART_NUM 2
#define IBUS_RX_PIN 16
#define IBUS_TX_PIN 17
#define IBUS_BAUD 115200
#define IBUS_FRESH_TIMEOUT_MS 50

// RC channel mapping (0-based)
#define CH_ROLL 0
#define CH_PITCH 1
#define CH_THROTTLE 2
#define CH_YAW 3
#define CH_ARM 4

// Motors and L9110S pins
// Each motor uses two input pins. We PWM one input and hold the other LOW.

// LEDC PWM config
#define LEDC_FREQ_HZ 20000
#define LEDC_RES_BITS 10

// Motor 1 (Front Left)
// INA is PWM, INB LOW
#define M1_INA_PIN 25
#define M1_INB_PIN 26

// Motor 2 (Front Right)
#define M2_INA_PIN 27
#define M2_INB_PIN 14

// Motor 3 (Rear Right)
#define M3_INA_PIN 12
#define M3_INB_PIN 13

// Motor 4 (Rear Left)
#define M4_INA_PIN 33
#define M4_INB_PIN 32

// Assign LEDC channels (ensure unique 0..15 on ESP32)
#define M1_LEDC_CH 0
#define M2_LEDC_CH 1
#define M3_LEDC_CH 2
#define M4_LEDC_CH 3