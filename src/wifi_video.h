#pragma once

#include <stdint.h>

/* ── WiFi AP settings (car = AP, remote = STA) ──────────────────── */
#define VIDEO_WIFI_SSID     "RC-Car-Cam"
#define VIDEO_WIFI_PASS     "rccar1234"
#define VIDEO_WIFI_CHANNEL  11          /* ch11 centre 2462 MHz;
                                           BLE adv ch39 @ 2480 MHz → minimal overlap */

/* ── Endpoints ──────────────────────────────────────────────────── */
#define VIDEO_CAR_IP        "192.168.4.1"
#define VIDEO_REMOTE_IP     "192.168.4.2"   /* first DHCP lease from car AP */
#define VIDEO_UDP_PORT      5000

/* ── Frame limits ───────────────────────────────────────────────── */
#define VIDEO_MAX_FRAME     (40 * 1024)     /* 40 KB max JPEG per frame     */
#define MIN_VALID_JPEG_LEN  1024            /* reject DMA-partial frames    */

/* ── Fragment protocol ──────────────────────────────────────────── *
 *  Each UDP datagram:  [frame_id : 2B][frag_idx : 1B][frag_cnt : 1B][payload]
 *  payload ≤ 1400 B  →  no IP fragmentation needed.                */
#define FRAG_HDR_LEN        4
#define FRAG_MAX_PAYLOAD    1400
