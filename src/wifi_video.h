#pragma once

#include <stdint.h>

/* ── WiFi AP settings (car = AP, remote = STA) ─────────────────── */
#define VIDEO_WIFI_SSID     "RC-Car-Cam"
#define VIDEO_WIFI_PASS     "rccar1234"
#define VIDEO_WIFI_CHANNEL  6

/* ── TCP video stream ───────────────────────────────────────────── */
#define VIDEO_TCP_PORT      5000
#define VIDEO_MAX_FRAME     (40 * 1024)    /* 40 KB max JPEG frame  */
