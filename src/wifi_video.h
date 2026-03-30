#pragma once

#include <stdint.h>

/* ── WiFi AP settings (car = AP, remote = STA) ─────────────────── */
#define VIDEO_WIFI_SSID     "RC-Car-Cam"
#define VIDEO_WIFI_PASS     "rccar1234"
#define VIDEO_WIFI_CHANNEL  11             /* ch11: no BLE adv overlap   */

/* ── UDP video stream ──────────────────────────────────────────── */
#define VIDEO_UDP_PORT      5000
#define VIDEO_MAX_FRAME     (40 * 1024)    /* 40 KB max JPEG frame  */
#define MIN_VALID_JPEG_LEN  1024           /* reject DMA-partial frames */

/* ── Fragment protocol ─────────────────────────────────────────── *
 *  Each UDP packet:  [frame_id:2][frag_idx:1][frag_cnt:1][payload]
 *  Payload per packet ≤ 1400 B  →  no IP fragmentation needed.    */
#define FRAG_HDR_LEN        4
#define FRAG_MAX_PAYLOAD    1400
#define FRAG_PKT_LEN        (FRAG_HDR_LEN + FRAG_MAX_PAYLOAD)
