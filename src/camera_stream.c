#ifdef BUILD_CAR

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "esp_camera.h"
#include "camera_stream.h"
#include "wifi_video.h"

/* ── Logging ─────────────────────────────────────────────────────── */
static const char *TAG = "cam_stream";

/* ── OV2640 pin map ──────────────────────────────────────────────── *
 * GPIO3  = PWDN  ← ESP32-S3 strapping pin; PWDN active-HIGH so it  *
 *                   is LOW at boot (camera active) → JTAG source   *
 *                   from eFuse, USB JTAG disabled.  UART flash OK.  *
 * GPIO17 = XCLK  → wired to module "NC" pad.  Driver generates     *
 *                   20 MHz regardless; covers modules that need it. */
#define CAM_PIN_PWDN    3
#define CAM_PIN_RESET   10
#define CAM_PIN_XCLK    17
#define CAM_PIN_SIOD    6
#define CAM_PIN_SIOC    7
#define CAM_PIN_D7      14
#define CAM_PIN_D6      2
#define CAM_PIN_D5      13
#define CAM_PIN_D4      16
#define CAM_PIN_D3      12
#define CAM_PIN_D2      4
#define CAM_PIN_D1      11
#define CAM_PIN_D0      5
#define CAM_PIN_VSYNC   8
#define CAM_PIN_HREF    9
#define CAM_PIN_PCLK    1

/* ── WiFi event group ────────────────────────────────────────────── */
#define WIFI_AP_STARTED_BIT  BIT0
static EventGroupHandle_t s_wifi_evg;

/* ─────────────────────────────────────────────────────────────────── */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_AP_START) {
        xEventGroupSetBits(s_wifi_evg, WIFI_AP_STARTED_BIT);
        ESP_LOGI(TAG, "WiFi AP started  SSID=\"%s\"  ch=%d",
                 VIDEO_WIFI_SSID, VIDEO_WIFI_CHANNEL);
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "Remote station connected");
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGW(TAG, "Remote station disconnected");
    }
}

/* ─────────────────────────────────────────────────────────────────── */
static void wifi_ap_init(void)
{
    s_wifi_evg = xEventGroupCreate();

    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .ap = {
            .ssid           = VIDEO_WIFI_SSID,
            .ssid_len       = sizeof(VIDEO_WIFI_SSID) - 1,
            .password       = VIDEO_WIFI_PASS,
            .channel        = VIDEO_WIFI_CHANNEL,
            .max_connection = 1,
            .authmode       = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Boost TX power for maximum range (84 × 0.25 dBm = 21 dBm) */
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    /* Wait until AP is up */
    xEventGroupWaitBits(s_wifi_evg, WIFI_AP_STARTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);
}

/* ─────────────────────────────────────────────────────────────────── */
static void camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn      = CAM_PIN_PWDN,
        .pin_reset     = CAM_PIN_RESET,
        .pin_xclk      = CAM_PIN_XCLK,
        .pin_sscb_sda  = CAM_PIN_SIOD,
        .pin_sscb_scl  = CAM_PIN_SIOC,
        .pin_d7        = CAM_PIN_D7,
        .pin_d6        = CAM_PIN_D6,
        .pin_d5        = CAM_PIN_D5,
        .pin_d4        = CAM_PIN_D4,
        .pin_d3        = CAM_PIN_D3,
        .pin_d2        = CAM_PIN_D2,
        .pin_d1        = CAM_PIN_D1,
        .pin_d0        = CAM_PIN_D0,
        .pin_vsync     = CAM_PIN_VSYNC,
        .pin_href      = CAM_PIN_HREF,
        .pin_pclk      = CAM_PIN_PCLK,

        /* LEDC_TIMER_1 / LEDC_CHANNEL_0 reserved for XCLK.
         * Motor PWM uses LEDC_TIMER_0 on channels 1-4 → no conflict. */
        .xclk_freq_hz  = 20000000,
        .ledc_timer    = LEDC_TIMER_1,
        .ledc_channel  = LEDC_CHANNEL_0,

        .pixel_format  = PIXFORMAT_JPEG,
        .frame_size    = FRAMESIZE_QVGA,    /* 320×240 matches ILI9341 exactly */
        .jpeg_quality  = 10,                /* 0-63; lower = higher quality    */
        .fb_count      = 2,
        .fb_location   = CAMERA_FB_IN_PSRAM,
        .grab_mode     = CAMERA_GRAB_LATEST,
    };

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Camera initialised");

    /* Fine-tune sensor for quality */
    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_ae_level(s, 0);
        s->set_aec2(s, 0);          /* disable AEC DSP                 */
        s->set_aec_value(s, 150);   /* manual exposure value           */
        s->set_gain_ctrl(s, 0);     /* manual gain                     */
        s->set_agc_gain(s, 2);      /* 2× gain                         */
        s->set_whitebal(s, 1);      /* auto white balance ON           */
        s->set_awb_gain(s, 1);
        s->set_contrast(s, 1);
        s->set_sharpness(s, 2);
        s->set_denoise(s, 1);
        s->set_lenc(s, 1);          /* lens correction                 */
        s->set_dcw(s, 1);           /* downsize check                  */
        s->set_bpc(s, 1);           /* bad pixel correction            */
        s->set_wpc(s, 1);
        s->set_raw_gma(s, 1);
    }
}

/* ─────────────────────────────────────────────────────────────────── */
static void stream_task(void *arg)
{
    /* Open UDP socket pointing at the remote */
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    /* Increase send buffer to reduce drops */
    int sndbuf = 64 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    struct sockaddr_in dest = {
        .sin_family      = AF_INET,
        .sin_port        = htons(VIDEO_UDP_PORT),
        .sin_addr.s_addr = inet_addr(VIDEO_REMOTE_IP),
    };

    /* Small static packet buffer on stack: header + max payload */
    static uint8_t pkt[FRAG_HDR_LEN + FRAG_MAX_PAYLOAD];

    uint16_t frame_id  = 0;
    uint32_t frame_cnt = 0;

    ESP_LOGI(TAG, "Stream task running → %s:%d", VIDEO_REMOTE_IP, VIDEO_UDP_PORT);

    while (true) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        /* Validate JPEG SOI marker */
        if (fb->len < MIN_VALID_JPEG_LEN ||
            fb->buf[0] != 0xFF || fb->buf[1] != 0xD8) {
            esp_camera_fb_return(fb);
            continue;
        }

        if (fb->len > VIDEO_MAX_FRAME) {
            ESP_LOGW(TAG, "Frame too large (%zu B), skipping", fb->len);
            esp_camera_fb_return(fb);
            continue;
        }

        size_t  remaining = fb->len;
        size_t  offset    = 0;
        uint8_t frag_cnt  = (uint8_t)((fb->len + FRAG_MAX_PAYLOAD - 1) / FRAG_MAX_PAYLOAD);

        for (uint8_t idx = 0; idx < frag_cnt; idx++) {
            size_t pay_len = (remaining < FRAG_MAX_PAYLOAD) ? remaining : FRAG_MAX_PAYLOAD;

            pkt[0] = (uint8_t)(frame_id >> 8);
            pkt[1] = (uint8_t)(frame_id & 0xFF);
            pkt[2] = idx;
            pkt[3] = frag_cnt;
            memcpy(pkt + FRAG_HDR_LEN, fb->buf + offset, pay_len);

            sendto(sock, pkt, FRAG_HDR_LEN + pay_len, 0,
                   (struct sockaddr *)&dest, sizeof(dest));

            offset    += pay_len;
            remaining -= pay_len;

            /* Brief yield between fragments to avoid flooding the NIC */
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        esp_camera_fb_return(fb);
        frame_id++;

        if ((++frame_cnt & 0x3F) == 0) {          /* log every 64 frames */
            ESP_LOGI(TAG, "Streamed %"PRIu32" frames", frame_cnt);
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────── */
void camera_stream_init(void)
{
    wifi_ap_init();
    camera_init();
}

void camera_stream_start(void)
{
    ESP_LOGI(TAG, "Waiting 6 s for remote to connect…");
    vTaskDelay(pdMS_TO_TICKS(6000));

    xTaskCreatePinnedToCore(stream_task, "cam_stream",
                            8192, NULL, 5, NULL, 1);
}

#endif /* BUILD_CAR */
