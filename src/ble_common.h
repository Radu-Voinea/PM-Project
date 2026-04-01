#pragma once

#include <stdint.h>

#define RC_SERVICE_UUID      0x00FF
#define RC_COMMAND_CHR_UUID  0xFF01

#define RC_DEVICE_NAME       "RC-Car"

typedef struct __attribute__((packed)) {
    int8_t  x;
    int8_t  y;
    uint8_t speed;
    int8_t  pan_x;
    int8_t  pan_y;
} rc_command_t;
