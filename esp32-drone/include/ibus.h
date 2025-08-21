#pragma once

#include <Arduino.h>

class IBUSReceiver {
public:
  void begin(uint8_t uartNum, int rxPin, int txPin, uint32_t baud);
  void update();
  uint16_t getChannel(uint8_t idx) const;
  bool isFrameFresh(uint32_t maxAgeMs) const;

private:
  void parseByte(uint8_t b);

  HardwareSerial *serial = nullptr;
  uint8_t buf[32];
  uint8_t idx = 0;
  uint16_t channels[14] = {1500};
  uint32_t lastFrameMs = 0;
};

