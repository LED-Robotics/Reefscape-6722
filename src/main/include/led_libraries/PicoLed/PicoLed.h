#pragma once

#include <cstdint>
#include <frc/SerialPort.h>
#include <frc/util/Color.h>

#include "commands.h"

using namespace frc;

class PicoLed : public SerialPort {
 public:
  explicit PicoLed(Port port=kOnboard, int baudRate=115200, int dataBits=8, Parity parity=kParity_None, StopBits stopBits=kStopBits_One);
  bool debugBlink();
  bool createStrip(uint8_t pin, uint16_t ledLength);
  bool destroyStrip(uint8_t stripId);
  bool createSlice(uint8_t stripId, uint16_t startLed, uint16_t endLed);
  int writeData(uint8_t* buf, int len);
  bool sendRequest(Request* req);
  // Move these into a strip object for better UX
  // bool idleAnimate(uint8_t stripId);
  // bool fillStrip(uint8_t stripId, Color color);
  // bool fillSegment(uint8_t stripId, Color color, uint16_t startLed, uint16_t endLed);

  PicoLed(PicoLed&&) = default;
  PicoLed& operator=(PicoLed&&) = default;
};
