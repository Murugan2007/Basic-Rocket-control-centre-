# ESP32 DevKit v1 Micro Quad (716 motors, L9110S, MPU6050)

Minimal flight controller firmware using ESP32 (Arduino framework), MPU6050 IMU, L9110S motor drivers, and 4x 716 brushed motors. RC input via IBUS (FlySky).

## Hardware
- ESP32 DevKit v1
- IMU: MPU6050 (I2C)
- 2x L9110S (each has A/B drivers; total 4 motor channels)
- 4x 716 brushed motors
- Receiver: FlySky IBUS (e.g., FS-iA6B/FS-A8S)

## Wiring
- I2C MPU6050: SDA=GPIO21, SCL=GPIO22, 3V3, GND
- IBUS: signal -> GPIO16 (RX), receiver Vcc per spec, GND common; TX pin GPIO17 unused but configured
- Motors via L9110S (two inputs per motor; one is PWM, the other is LOW):
  - M1 Front Left: INA=GPIO25 (PWM), INB=GPIO26 (LOW)
  - M2 Front Right: INA=GPIO27 (PWM), INB=GPIO14 (LOW)
  - M3 Rear Right: INA=GPIO12 (PWM), INB=GPIO13 (LOW)
  - M4 Rear Left: INA=GPIO33 (PWM), INB=GPIO32 (LOW)

If a motor spins the wrong direction, swap the motor leads or switch which input is PWM vs LOW for that motor in `include/config.h`.

Power: 1S LiPo to L9110S Vcc/GND, and regulated 5V/3.3V to ESP32 as appropriate. Always share ground between ESP32, receiver, and L9110S.

## Build & Upload
```
pio run -t upload -e esp32dev
pio device monitor -b 115200
```

## RC Channels (IBUS)
- CH1 Roll, CH2 Pitch, CH3 Throttle, CH4 Yaw, CH5 Arm

## Features
- Complementary filter for attitude from MPU6050
- PID control: angle for roll/pitch (D on measurement), rate for yaw
- Arming logic: CH5 high and throttle low to arm; failsafe disarm on IBUS timeout

## Tuning
Adjust PID gains and setpoint ranges in `include/config.h`. Start conservative. Verify motor directions and mixer before first hover.

## Notes
- Control loop runs at 250 Hz.
- LEDC PWM at 20 kHz to reduce audible noise on brushed motors.