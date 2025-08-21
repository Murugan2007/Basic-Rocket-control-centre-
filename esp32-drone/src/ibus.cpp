#include "ibus.h"
#include <HardwareSerial.h>

void IBUSReceiver::begin(uint8_t uartNum, int rxPin, int txPin, uint32_t baud) {
  static HardwareSerial SerialIBUS(2);
  serial = &SerialIBUS;
  SerialIBUS.begin(baud, SERIAL_8N1, rxPin, txPin);
}

void IBUSReceiver::update() {
  if (!serial) return;
  while (serial->available()) {
    uint8_t b = (uint8_t)serial->read();
    parseByte(b);
  }
}

uint16_t IBUSReceiver::getChannel(uint8_t i) const {
  if (i >= 14) return 1500;
  return channels[i];
}

bool IBUSReceiver::isFrameFresh(uint32_t maxAgeMs) const {
  return millis() - lastFrameMs <= maxAgeMs;
}

void IBUSReceiver::parseByte(uint8_t b) {
  // Simple frame assembly for IBUS (32 bytes frame)
  if (idx == 0 && b != 0x20) {
    // looking for frame length 0x20
    return;
  }
  buf[idx++] = b;
  if (idx < 32) return;

  // Validate frame basic structure
  // buf[0] length, buf[1] command, channels at buf[2..]

  for (uint8_t ch = 0; ch < 14; ch++) {
    uint8_t lo = buf[2 + ch * 2];
    uint8_t hi = buf[3 + ch * 2];
    uint16_t val = (uint16_t)((hi << 8) | lo);
    channels[ch] = val;
  }
  lastFrameMs = millis();
  idx = 0;
}

