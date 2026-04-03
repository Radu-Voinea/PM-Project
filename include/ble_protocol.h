#pragma once

#include <cstdint>

#define RC_SERVICE_UUID      0x00FF
#define RC_COMMAND_CHR_UUID  0xFF01
#define RC_DEVICE_NAME       "RC-Car"

struct __attribute__((packed)) ControlCommand {
    int8_t  x;       // steering  (-100 .. +100)
    int8_t  y;       // throttle  (-100 .. +100)
    uint8_t speed;   // speed mode (0 / 30 / 60 / 100)
    int8_t  pan_x;   // camera pan  rate
    int8_t  pan_y;   // camera tilt rate
};

static_assert(sizeof(ControlCommand) == 5, "ControlCommand must be 5 bytes");
