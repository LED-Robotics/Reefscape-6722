#include "led_libraries/PicoLed/PicoLed.h"
#include "frc/SerialPort.h"
#include "led_libraries/PicoLed/commands.h"
#include <cstdint>

using namespace frc;

PicoLed::PicoLed(Port port, int baudRate, int dataBits, Parity parity, StopBits stopBits) : SerialPort(baudRate, port, dataBits, parity, stopBits) {
  DisableTermination();
  SetWriteBufferMode(kFlushWhenFull);
}

bool PicoLed::debugBlink() {
  uint8_t dummy;
  Request req{DEBUG_BLINK, &dummy, 0};
  return sendRequest(&req);
}

bool PicoLed::createStrip(uint8_t pin, uint16_t ledLength) {
  CreateStripRequest payload{
    pin,
    ledLength
  };
  Request req{CREATE_STRIP, (uint8_t*)&payload, sizeof(payload)};
  return sendRequest(&req);
}

bool PicoLed::destroyStrip(uint8_t stripId) {
  DestroyStripRequest payload{
    stripId
  };
  Request req{DESTROY_STRIP, (uint8_t*)&payload, sizeof(payload)};
  return sendRequest(&req);
}

bool PicoLed::createSlice(uint8_t stripId, uint16_t startLed, uint16_t endLed) {
  CreateSliceRequest payload{
    stripId,
    startLed,
    endLed
  };
  Request req{CREATE_SLICE, (uint8_t*)&payload, sizeof(payload)};
  return sendRequest(&req);
}

int PicoLed::writeData(uint8_t* buf, int len) {
  return Write((char*)buf, len);
}

bool PicoLed::sendRequest(Request* req) {
  // send packet start, command id, payload len, and payload
  int written = 0;
  uint16_t start = PACKET_START;
  written += writeData((uint8_t*)&start, 2);
  written += writeData((uint8_t*)&req->commandId, 2);
  written += writeData((uint8_t*)&req->payloadLen, 2);
  written += writeData((uint8_t*)&req->payload, req->payloadLen);
  Flush();
  return written == req->payloadLen + 6;
}
