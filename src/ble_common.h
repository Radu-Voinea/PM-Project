#pragma once

#include <stdint.h>

/* ── Custom BLE Service / Characteristic UUIDs ───────────────────── */
#define RC_SERVICE_UUID      0x00FF
#define RC_COMMAND_CHR_UUID  0xFF01

/* ── Device name advertised by the car ───────────────────────────── */
#define RC_DEVICE_NAME       "RC-Car"

/* ── Command packet  (remote → car) ──────────────────────────────── */
typedef struct __attribute__((packed)) {
    int8_t  x;      /* -100 (full left)     … +100 (full right)    */
    int8_t  y;      /* -100 (full backward) … +100 (full forward)  */
    uint8_t speed;  /*    0 (stop)          …  100 (full power)    */
} rc_command_t;
