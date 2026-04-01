#pragma once

#include <stdint.h>

/* ── BLE service / characteristic UUIDs ──────────────────────────── */
#define RC_SERVICE_UUID      0x00FF
#define RC_COMMAND_CHR_UUID  0xFF01

#define RC_DEVICE_NAME       "RC-Car"

/* ── Command packet sent from remote → car every 20 ms ──────────── *
 *  Total: 5 bytes, packed, no padding.                               */
typedef struct __attribute__((packed)) {
    int8_t  x;        /* steering  -100…+100  (left/right)          */
    int8_t  y;        /* throttle  -100…+100  (fwd/rev)             */
    uint8_t speed;    /* speed cap  0, 30, 60 or 100                */
    int8_t  pan_x;   /* camera L/R servo  -100…+100                */
    int8_t  pan_y;   /* camera U/D servo  -100…+100                */
} rc_command_t;
