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
        .jpeg_quality = 20,                  /* 0-63, lower = better */
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
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
        s->set_exposure_ctrl(s, 1);     /* auto-exposure ON            */
        s->set_gain_ctrl(s, 1);         /* auto-gain ON                */
        s->set_ae_level(s, 2);          /* AE target: -2 dark … +2 bright */
        s->set_brightness(s, 2);        /* brightness offset: -2 … +2 */
        s->set_whitebal(s, 1);          /* auto white-balance ON       */
        s->set_awb_gain(s, 1);          /* AWB gain ON                 */
        ESP_LOGI(TAG, "Sensor: auto AE/AG/AWB, ae_level=0, brightness=0");
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

/* ── TCP helpers ────────────────────────────────────────────────── */

static bool tcp_send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = buf;
    while (len) {
        int n = send(fd, p, len, 0);
        if (n <= 0) return false;
        p   += n;
        len -= (size_t)n;
    }
    return true;
}

/* ── Streaming task ─────────────────────────────────────────────── *
 *
 *  Protocol (per frame):
 *      [4 bytes — JPEG length, network byte order]
 *      [N bytes — raw JPEG payload]
 */
static void stream_task(void *arg)
{
    int listen_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL); return;
    }

    int opt = 1;
    setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(VIDEO_TCP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(listen_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(listen_fd); vTaskDelete(NULL); return;
    }
    listen(listen_fd, 1);
    ESP_LOGI(TAG, "Listening on :%d", VIDEO_TCP_PORT);

    for (;;) {
        ESP_LOGI(TAG, "Waiting for client ...");
        int client = accept(listen_fd, NULL, NULL);
        if (client < 0) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        int flag = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY,  &flag, sizeof(flag));
        setsockopt(client, SOL_SOCKET,  SO_KEEPALIVE, &flag, sizeof(flag));

        /* Send timeout: if the receiver disappears mid-frame, don't block
         * forever — bail out and wait for a fresh connection. */
        struct timeval snd_tv = { .tv_sec = 5, .tv_usec = 0 };
        setsockopt(client, SOL_SOCKET, SO_SNDTIMEO, &snd_tv, sizeof(snd_tv));

        ESP_LOGI(TAG, "Client connected");

        uint32_t sent = 0;
        bool     ok   = true;

        while (ok) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

            /* Validate JPEG: must start with SOI (FF D8) and end with EOI (FF D9) */
            if (fb->len < 4 ||
                fb->buf[0] != 0xFF || fb->buf[1] != 0xD8 ||
                fb->buf[fb->len - 2] != 0xFF || fb->buf[fb->len - 1] != 0xD9)
            {
                ESP_LOGW(TAG, "bad JPEG frame %u B — skip", (unsigned)fb->len);
                esp_camera_fb_return(fb);
                continue;
            }

            if (sent == 0) {
                ESP_LOGI(TAG, "1st frame: %u B  hdr=%02X %02X %02X %02X",
                         (unsigned)fb->len,
                         fb->buf[0], fb->buf[1],
                         fb->len > 2 ? fb->buf[2] : 0,
                         fb->len > 3 ? fb->buf[3] : 0);
            }

            uint32_t net_len = htonl((uint32_t)fb->len);
            if (!tcp_send_all(client, &net_len, 4) ||
                !tcp_send_all(client, fb->buf, fb->len))
            {
                ok = false;
            }

            esp_camera_fb_return(fb);

            if (ok) {
                sent++;
                if ((sent & 0x3F) == 0)
                    ESP_LOGI(TAG, "Sent %lu frames", (unsigned long)sent);
                taskYIELD();
            }
        }

        close(client);
        ESP_LOGW(TAG, "Client gone after %lu frames", (unsigned long)sent);
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
