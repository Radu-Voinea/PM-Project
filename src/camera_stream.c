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
/* img_converters.h not needed — camera outputs JPEG directly */
#include "lwip/sockets.h"

static const char *TAG = "cam_tx";
static bool camera_ok = false;

/* ── OV2640 GPIO mapping ────────────────────────────────────────── */
#define CAM_SIOC    GPIO_NUM_7
#define CAM_SIOD    GPIO_NUM_6
#define CAM_D0      GPIO_NUM_5
#define CAM_D1      GPIO_NUM_11
#define CAM_D2      GPIO_NUM_4
#define CAM_D3      GPIO_NUM_12
#define CAM_D4      GPIO_NUM_16
#define CAM_D5      GPIO_NUM_13
#define CAM_D6      GPIO_NUM_2
#define CAM_D7      GPIO_NUM_14
#define CAM_PCLK    GPIO_NUM_1
#define CAM_VSYNC   GPIO_NUM_8
#define CAM_HREF    GPIO_NUM_9
#define CAM_PWDN    (-1)       /* PWDN wired to GND on module */
#define CAM_RESET   GPIO_NUM_10

/*
 * XCLK: The OV2640 needs an external master clock (typically 20 MHz).
 * If your camera module has an onboard crystal / oscillator, set this to -1.
 * If it does NOT (most common breakout boards), wire a free GPIO to the
 * module's XCLK pin and set it here (e.g. GPIO_NUM_17).
 *
 * NOTE: The LEDC peripheral generates XCLK.  Motors already use LEDC
 * channels 0-7 on TIMER_0.  If XCLK is enabled, it uses TIMER_1 and
 * one channel — no conflict because the channel is only used internally
 * by the camera driver while XCLK is active, and we assign LEDC_CHANNEL_0
 * here which maps to TIMER_1 (the driver re-configures it).
 */
#define CAM_XCLK    GPIO_NUM_17

/* ── WiFi SoftAP ────────────────────────────────────────────────── */
static void wifi_ap_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    /* create_default may already exist — ignore ESP_ERR_INVALID_STATE */
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(err);
    }

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

    ESP_LOGI(TAG, "WiFi AP started  SSID=\"%s\"  ch=%d",
             VIDEO_WIFI_SSID, VIDEO_WIFI_CHANNEL);
}

/* ── Camera ─────────────────────────────────────────────────────── */
static void camera_init(void)
{
    camera_config_t cfg = {
        .pin_pwdn     = CAM_PWDN,
        .pin_reset    = CAM_RESET,
        .pin_xclk     = CAM_XCLK,
        .pin_sccb_sda = CAM_SIOD,
        .pin_sccb_scl = CAM_SIOC,
        .pin_d7       = CAM_D7,
        .pin_d6       = CAM_D6,
        .pin_d5       = CAM_D5,
        .pin_d4       = CAM_D4,
        .pin_d3       = CAM_D3,
        .pin_d2       = CAM_D2,
        .pin_d1       = CAM_D1,
        .pin_d0       = CAM_D0,
        .pin_vsync    = CAM_VSYNC,
        .pin_href     = CAM_HREF,
        .pin_pclk     = CAM_PCLK,

        .xclk_freq_hz = 20000000,          /* 20 MHz — needed for JPEG */
        .ledc_timer   = LEDC_TIMER_1,
        .ledc_channel = LEDC_CHANNEL_0,

        .pixel_format = PIXFORMAT_JPEG,      /* HW JPEG — sent directly */
        .frame_size   = FRAMESIZE_QVGA,    /* 320 × 240 */
        .jpeg_quality = 12,
        .fb_count     = 2,
        .fb_location  = CAMERA_FB_IN_PSRAM,
        .grab_mode    = CAMERA_GRAB_WHEN_EMPTY,
    };

    ESP_LOGI(TAG, "Calling esp_camera_init ...");
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x — streaming disabled", err);
        return;
    }
    camera_ok = true;
    ESP_LOGI(TAG, "Camera initialised  (QVGA HW-JPEG q=%d)", cfg.jpeg_quality);
}

/* ── Helper: send exactly `len` bytes over TCP ─────────────────── */
static bool tcp_send_all(int fd, const void *buf, size_t len)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (len > 0) {
        int n = send(fd, p, len, 0);
        if (n <= 0) return false;
        p   += n;
        len -= (size_t)n;
    }
    return true;
}

/* ── TCP streaming task (HW JPEG — send camera JPEG directly) ── */
static void stream_task(void *param)
{
    /* Create listening TCP socket */
    int listen_sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(VIDEO_TCP_PORT),
        .sin_addr.s_addr = INADDR_ANY,
    };
    if (bind(listen_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(listen_sock);
        vTaskDelete(NULL);
        return;
    }
    listen(listen_sock, 1);
    ESP_LOGI(TAG, "TCP server listening on port %d", VIDEO_TCP_PORT);

    for (;;) {
        /* Wait for remote to connect */
        ESP_LOGI(TAG, "Waiting for client ...");
        int client = accept(listen_sock, NULL, NULL);
        if (client < 0) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
        ESP_LOGI(TAG, "Client connected — streaming");

        /* Disable Nagle for lower latency */
        int flag = 1;
        setsockopt(client, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        uint32_t frames_ok = 0;
        bool     alive     = true;

        while (alive) {
            camera_fb_t *fb = esp_camera_fb_get();
            if (!fb) { vTaskDelay(pdMS_TO_TICKS(10)); continue; }

            /* Camera outputs JPEG directly — send as-is */
            const uint8_t *jpeg_buf = fb->buf;
            size_t          jpeg_len = fb->len;

            /* Sanity: first frame — log header bytes */
            if (frames_ok == 0) {
                ESP_LOGI(TAG, "First frame: len=%u  hdr=%02X %02X %02X %02X",
                         (unsigned)jpeg_len,
                         jpeg_buf[0], jpeg_buf[1],
                         jpeg_len > 2 ? jpeg_buf[2] : 0,
                         jpeg_len > 3 ? jpeg_buf[3] : 0);
            }

            /* Send: [4-byte length][jpeg data] */
            uint32_t net_len = htonl((uint32_t)jpeg_len);
            if (!tcp_send_all(client, &net_len, 4) ||
                !tcp_send_all(client, jpeg_buf, jpeg_len))
            {
                alive = false;
            }
            esp_camera_fb_return(fb);

            if (alive) {
                frames_ok++;
                if ((frames_ok & 63) == 0)
                    ESP_LOGI(TAG, "Frames streamed: %lu", (unsigned long)frames_ok);
                vTaskDelay(pdMS_TO_TICKS(30));
            }
        }

        close(client);
        ESP_LOGW(TAG, "Client disconnected after %lu frames", (unsigned long)frames_ok);
    }
}

/* ── Public init ────────────────────────────────────────────────── */
void camera_stream_init(void)
{
    ESP_LOGI(TAG, "Initialising camera ...");
    camera_init();

    ESP_LOGI(TAG, "Starting WiFi AP ...");
    wifi_ap_init();

    if (camera_ok) {
        ESP_LOGI(TAG, "Launching stream task (SW JPEG)");
        xTaskCreate(stream_task, "cam_tx", 16384, NULL, 4, NULL);
    } else {
        ESP_LOGW(TAG, "Camera not available — streaming disabled");
    }
}

#endif /* BUILD_CAR */
