#pragma once

#include <cstdint>

#define PACKET_START 0x6769

struct Request {
  uint16_t commandId;
  uint8_t* payload;
  uint16_t payloadLen;
};

#define DEBUG_BLINK 0x69fa

#define LEDS_OFF 0x0eaf
struct LedsOffRequest {
  uint8_t stripId;
};

#define IDLE_ANIMATE 0xa146
struct IdleAnimateRequest {
  uint8_t stripId;
  uint8_t swaps;
  uint8_t startGreen;
};

#define FILL_STRIP 0xc273
struct FillStripRequest {
  uint8_t stripId;
  uint8_t red;
  uint8_t green;
  uint8_t blue;
};

#define CREATE_STRIP 0x8bda
struct CreateStripRequest {
  uint8_t stripPin;
  uint16_t ledLength;
};

#define DESTROY_STRIP 0x10e7
struct DestroyStripRequest {
  uint8_t stripId;
};

#define CREATE_SLICE 0x3138
struct CreateSliceRequest {
  uint8_t stripId;
  uint16_t startLed;
  uint16_t endLed;
};

