#ifdef BUILD_CAR

#include "camera_stream.h"
#include "wifi_video.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_camera.h"
#include "lwip/sockets.h"

static const char *TAG = "cam_tx";
static bool camera_ok = false;

/* ── OV2640 GPIO mapping ────────────────────────────────────────── *
 *
 *  Module: 18-pin OV2640 with onboard crystal oscillator.
 *
 *   Top row:  GND  SCL  SDA  D0  D2  D4  D6  DCLK  PWDN
 *   Bot row:  3V3  VSYNC HREF RST D1  D3  D5  D7    NC
 *
 *  PWDN is active-HIGH (HIGH = sleep).  The camera driver drives it
 *  LOW during init to wake the sensor.
 *
 *  The module likely has its own oscillator (NC pin on silkscreen),
 *  but we generate XCLK on GPIO 17 anyway — if the pin really is NC
 *  the signal is harmlessly ignored; if it turns out to be XCLK, the
 *  camera gets a clock.  Uses LEDC_CHANNEL_0 / TIMER_1 (motors are
 *  on channels 1-4 / TIMER_0, so no conflict).
 */
#define CAM_PIN_SIOC    GPIO_NUM_7      /* SCL  */
#define CAM_PIN_SIOD    GPIO_NUM_6      /* SDA  */
#define CAM_PIN_D0      GPIO_NUM_5
#define CAM_PIN_D1      GPIO_NUM_11
#define CAM_PIN_D2      GPIO_NUM_4
#define CAM_PIN_D3      GPIO_NUM_12
#define CAM_PIN_D4      GPIO_NUM_16
#define CAM_PIN_D5      GPIO_NUM_13
#define CAM_PIN_D6      GPIO_NUM_2
#define CAM_PIN_D7      GPIO_NUM_14
#define CAM_PIN_PCLK    GPIO_NUM_1      /* DCLK on module silkscreen */
#define CAM_PIN_VSYNC   GPIO_NUM_8
#define CAM_PIN_HREF    GPIO_NUM_9
#define CAM_PIN_PWDN    GPIO_NUM_3
#define CAM_PIN_RESET   GPIO_NUM_10
#define CAM_PIN_XCLK    GPIO_NUM_17     /* wired to NC — generate anyway */

/* ── WiFi Access Point ──────────────────────────────────────────── */

static void wifi_ap_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid           = VIDEO_WIFI_SSID,
            .password       = VIDEO_WIFI_PASS,
            .ssid_len       = (uint8_t)strlen(VIDEO_WIFI_SSID),
            .channel        = VIDEO_WIFI_CHANNEL,
            .authmode       = WIFI_AUTH_WPA2_PSK,
            .max_connection = 2,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP up  SSID=\"%s\"  ch=%d",
             VIDEO_WIFI_SSID, VIDEO_WIFI_CHANNEL);
}

/* ── Camera initialisation ──────────────────────────────────────── */

static void camera_init(void)
{
    camera_config_t cfg = {
        /* ── control pins ── */
        .pin_pwdn     = CAM_PIN_PWDN,
        .pin_reset    = CAM_PIN_RESET,

        /* ── XCLK: generate 20 MHz on GPIO 17 via LEDC ── */
        .pin_xclk     = CAM_PIN_XCLK,
        .xclk_freq_hz = 20000000,
        .ledc_timer   = LEDC_TIMER_1,      /* TIMER_0 is motors          */
        .ledc_channel = LEDC_CHANNEL_0,    /* motors use ch 1-4          */

        /* ── SCCB / I2C ── */
        .pin_sccb_sda = CAM_PIN_SIOD,
        .pin_sccb_scl = CAM_PIN_SIOC,

        /* ── 8-bit parallel data bus ── */
        .pin_d7       = CAM_PIN_D7,
        .pin_d6       = CAM_PIN_D6,
        .pin_d5       = CAM_PIN_D5,
        .pin_d4       = CAM_PIN_D4,
        .pin_d3       = CAM_PIN_D3,
        .pin_d2       = CAM_PIN_D2,
        .pin_d1       = CAM_PIN_D1,
        .pin_d0       = CAM_PIN_D0,

        /* ── sync / clock ── */
        .pin_vsync    = CAM_PIN_VSYNC,
        .pin_href     = CAM_PIN_HREF,
        .pin_pclk     = CAM_PIN_PCLK,

        /* ── frame settings ── */
        .pixel_format = PIXFORMAT_JPEG,
        .frame_size   = FRAMESIZE_QVGA,     /* 320 x 240 */
        .jpeg_quality = 20,                  /* 0-63; higher = smaller frames, fewer UDP fragments */
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_LATEST,
    };

    ESP_LOGI(TAG, "esp_camera_init  (PWDN=GPIO%d, XCLK=GPIO%d) ...",
             CAM_PIN_PWDN, CAM_PIN_XCLK);

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init FAILED: 0x%x", err);
        return;
    }

    camera_ok = true;
    ESP_LOGI(TAG, "Camera ready  (QVGA JPEG q=%d)", cfg.jpeg_quality);

    /* ── Sensor tuning ─────────────────────────────────────────────
     *  Disabling AEC/AGC kills frame output on this module, so keep
     *  auto-exposure ON and steer brightness via ae_level instead.  */
    sensor_t *s = esp_camera_sensor_get();
if (s) {
    /* ── Exposure & Gain ──────────────────────────────────────── */
	s->set_exposure_ctrl(s, 1);
	s->set_aec2(s, 1);
	s->set_ae_level(s, 2);               /* was 1 → bump to 2                */
	s->set_aec_value(s, 600);            /* was 400 → higher seed            */
	s->set_gain_ctrl(s, 1);
	s->set_gainceiling(s, GAINCEILING_16X); /* was 8X → double it            */
	s->set_agc_gain(s, 5);               /* give AGC a non-zero starting point */

    /* ── White Balance ───────────────────────────────────────── */
	s->set_whitebal(s, 1);
	s->set_awb_gain(s, 1);
	s->set_wb_mode(s, 0);                /* was 2 (fluorescent) → AUTO       */
	/*  Fluorescent mode was adding warmth on top of already-warm bulbs      */

    /* ── Image Quality ───────────────────────────────────────── */
    s->set_brightness(s, 1);             /* slight boost for indoor dims     */
    s->set_contrast(s, 1);               /* +1 helps flat indoor lighting    */
    s->set_saturation(s, 0);             /* neutral, avoid oversaturation    */
    s->set_sharpness(s, 1);              /* edge crispness, indoor is blurry */

    /* ── Noise & Lens ────────────────────────────────────────── */
    s->set_denoise(s, 1);                /* ON — indoor = more noise         */
    s->set_lenc(s, 1);                   /* lens correction ON               */
    s->set_raw_gma(s, 1);                /* gamma correction ON              */
    s->set_dcw(s, 1);                    /* downsize EN for cleaner output   */

    /* ── Flip / Mirror (adjust to your mount) ───────────────── */
    s->set_vflip(s, 0);
    s->set_hmirror(s, 0);

    /* ── Banding filter ─────────────────────────────────────── */
    s->set_bpc(s, 1);                    /* bad pixel correction ON          */
    s->set_wpc(s, 1);                    /* white pixel correction ON        */

    ESP_LOGI(TAG, "Sensor: indoor optimised, fluorescent WB, 8X gain ceiling");
}

    /* Let sensor settle after register writes */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Flush warmup frames */
    for (int i = 0; i < 5; i++) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            ESP_LOGI(TAG, "  warmup %d: %u bytes  [%02X %02X]",
                     i, (unsigned)fb->len, fb->buf[0], fb->buf[1]);
            esp_camera_fb_return(fb);
        } else {
            ESP_LOGW(TAG, "  warmup %d: timeout (OK during startup)", i);
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }

}

/* ── UDP streaming task ────────────────────────────────────────── *
 *
 *  Send JPEG frames as manually-fragmented UDP packets.
 *  Each packet ≤ 1404 B (4-byte header + 1400 payload), so no
 *  IP-level fragmentation is needed — much more reliable over WiFi.
 */
static void stream_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL); return;
    }

    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(VIDEO_UDP_PORT),
        .sin_addr.s_addr = inet_addr("192.168.4.2"),
    };

    /* Packet buffer on stack — fits one fragment */
    uint8_t pkt[FRAG_PKT_LEN];

    ESP_LOGI(TAG, "UDP target 192.168.4.2:%d (fragmented)", VIDEO_UDP_PORT);

    /* Wait for remote to connect & get DHCP before blasting frames */
    ESP_LOGI(TAG, "Waiting 8s for remote to connect...");
    vTaskDelay(pdMS_TO_TICKS(8000));

    uint16_t frame_id = 0;
    uint32_t sent = 0;

    for (;;) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

        /* Light validation */
        if (fb->len < MIN_VALID_JPEG_LEN ||
            fb->buf[0] != 0xFF || fb->buf[1] != 0xD8)
        {
            ESP_LOGW(TAG, "bad frame %u B — skip", (unsigned)fb->len);
            esp_camera_fb_return(fb);
            continue;
        }

        /* Fragment and send */
        uint8_t frag_cnt = (uint8_t)((fb->len + FRAG_MAX_PAYLOAD - 1) / FRAG_MAX_PAYLOAD);
        size_t  offset = 0;
        bool    ok = true;

        for (uint8_t i = 0; i < frag_cnt && ok; i++) {
            size_t chunk = fb->len - offset;
            if (chunk > FRAG_MAX_PAYLOAD) chunk = FRAG_MAX_PAYLOAD;

            pkt[0] = (uint8_t)(frame_id >> 8);
            pkt[1] = (uint8_t)(frame_id & 0xFF);
            pkt[2] = i;
            pkt[3] = frag_cnt;
            memcpy(pkt + FRAG_HDR_LEN, fb->buf + offset, chunk);

            int n = -1;
            for (int retry = 0; retry < 3; retry++) {
                n = sendto(sock, pkt, FRAG_HDR_LEN + chunk, 0,
                           (struct sockaddr *)&dest, sizeof(dest));
                if (n > 0) break;
                vTaskDelay(pdMS_TO_TICKS(2));   /* brief back off */
            }
            if (n <= 0) { ok = false; break; }  /* drop frame, move on */
            offset += chunk;

            /* Yield between fragments to let WiFi TX drain */
            if (i < frag_cnt - 1) vTaskDelay(pdMS_TO_TICKS(2));
        }

        esp_camera_fb_return(fb);

        if (ok) {
            sent++;
            if (sent <= 3 || (sent & 0x3F) == 0)
                ESP_LOGI(TAG, "Sent %lu frames (%u B, %u frags)",
                         (unsigned long)sent, (unsigned)offset, frag_cnt);
        }

        frame_id++;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ── Public entry point ─────────────────────────────────────────── */

void camera_stream_init(void)
{
    camera_init();
    wifi_ap_init();

    if (camera_ok) {
        xTaskCreate(stream_task, "cam_tx", 16384, NULL, 4, NULL);
    } else {
        ESP_LOGW(TAG, "No camera — video disabled");
    }
}

#endif /* BUILD_CAR */
