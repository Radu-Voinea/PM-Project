#ifdef BUILD_REMOTE

#include "display_stream.h"
#include "wifi_video.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"
#include "lwip/sockets.h"
#include "img_converters.h"             /* jpg2rgb565() from esp32-camera */

static const char *TAG = "disp_rx";

/* ── ILI9341 8080-parallel pin mapping ──────────────────────────── *
 *
 *  LCD_RD must be held HIGH (tie to 3.3 V) — the ESP-IDF i80 bus
 *  is write-only.  Backlight: wire to 3.3 V (or control via GPIO).
 */
#define LCD_D0   GPIO_NUM_12
#define LCD_D1   GPIO_NUM_11
#define LCD_D2   GPIO_NUM_48
#define LCD_D3   GPIO_NUM_21
#define LCD_D4   GPIO_NUM_18
#define LCD_D5   GPIO_NUM_17
#define LCD_D6   GPIO_NUM_14
#define LCD_D7   GPIO_NUM_13
#define LCD_RST  GPIO_NUM_42
#define LCD_CS   GPIO_NUM_41
#define LCD_DC   GPIO_NUM_38            /* data / command (RS)      */
#define LCD_WR   GPIO_NUM_47            /* write strobe             */

#define LCD_W    320
#define LCD_H    240
#define LCD_PX   (LCD_W * LCD_H)        /* total pixels             */
#define LCD_FB   (LCD_PX * 2)           /* frame-buffer bytes       */

/* ── LCD globals ────────────────────────────────────────────────── */

static esp_lcd_panel_io_handle_t lcd_io;
static SemaphoreHandle_t         flush_sem;

static bool IRAM_ATTR on_color_done(esp_lcd_panel_io_handle_t io,
                                    esp_lcd_panel_io_event_data_t *ev,
                                    void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(flush_sem, &woken);
    return woken == pdTRUE;
}

/* ── WiFi Station ───────────────────────────────────────────────── */

static EventGroupHandle_t wifi_eg;
#define WIFI_UP BIT0

static void wifi_cb(void *arg, esp_event_base_t base,
                    int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED) {
            if (id == WIFI_EVENT_STA_DISCONNECTED)
                xEventGroupClearBits(wifi_eg, WIFI_UP);
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_eg, WIFI_UP);
        ESP_LOGI(TAG, "WiFi connected — got IP");
    }
}

static void wifi_sta_init(void)
{
    wifi_eg = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wcfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wcfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_cb, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_cb, NULL);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta = {
        .sta = {
            .ssid     = VIDEO_WIFI_SSID,
            .password = VIDEO_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA — connecting to \"%s\"", VIDEO_WIFI_SSID);
}

/* ── ILI9341 low-level ──────────────────────────────────────────── */

static inline void lcd_cmd(uint8_t cmd, const uint8_t *p, size_t n)
{
    esp_lcd_panel_io_tx_param(lcd_io, cmd, p, n);
}

static void lcd_flush(const uint8_t *rgb565)
{
    uint8_t ca[] = { 0, 0, (LCD_W - 1) >> 8, (LCD_W - 1) & 0xFF };
    lcd_cmd(0x2A, ca, 4);

    uint8_t ra[] = { 0, 0, (LCD_H - 1) >> 8, (LCD_H - 1) & 0xFF };
    lcd_cmd(0x2B, ra, 4);

    esp_lcd_panel_io_tx_color(lcd_io, 0x2C, rgb565, LCD_FB);
    xSemaphoreTake(flush_sem, portMAX_DELAY);
}

static void lcd_init(void)
{
    /* Hard reset */
    gpio_config_t rc = {
        .pin_bit_mask = 1ULL << LCD_RST,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rc);
    gpio_set_level(LCD_RST, 0);  vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST, 1);  vTaskDelay(pdMS_TO_TICKS(120));

    /* Intel 8080 bus */
    esp_lcd_i80_bus_handle_t bus = NULL;
    esp_lcd_i80_bus_config_t bus_cfg = {
        .dc_gpio_num    = LCD_DC,
        .wr_gpio_num    = LCD_WR,
        .clk_src        = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = { LCD_D0, LCD_D1, LCD_D2, LCD_D3,
                            LCD_D4, LCD_D5, LCD_D6, LCD_D7 },
        .bus_width          = 8,
        .max_transfer_bytes = LCD_FB + 64,
        .psram_trans_align  = 64,
        .sram_trans_align   = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_cfg, &bus));

    flush_sem = xSemaphoreCreateBinary();

    esp_lcd_panel_io_i80_config_t io_cfg = {
        .cs_gpio_num       = LCD_CS,
        .pclk_hz           = 8000000,
        .trans_queue_depth  = 4,
        .on_color_trans_done = on_color_done,
        .dc_levels = {
            .dc_idle_level  = 0,
            .dc_cmd_level   = 0,
            .dc_dummy_level = 0,
            .dc_data_level  = 1,
        },
        .lcd_cmd_bits   = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(bus, &io_cfg, &lcd_io));

    /* ── ILI9341 register init ─────────────────────────────────── */
    uint8_t d[16];

    d[0]=0x00; d[1]=0x83; d[2]=0x30;       lcd_cmd(0xCF, d, 3);   /* Pwr Ctrl B   */
    d[0]=0x64; d[1]=0x03; d[2]=0x12; d[3]=0x81;
                                            lcd_cmd(0xED, d, 4);   /* Pwr On Seq   */
    d[0]=0x85; d[1]=0x01; d[2]=0x79;       lcd_cmd(0xE8, d, 3);   /* Drv Timing A */
    d[0]=0x39; d[1]=0x2C; d[2]=0x00; d[3]=0x34; d[4]=0x02;
                                            lcd_cmd(0xCB, d, 5);   /* Pwr Ctrl A   */
    d[0]=0x20;                              lcd_cmd(0xF7, d, 1);   /* Pump Ratio   */
    d[0]=0x00; d[1]=0x00;                   lcd_cmd(0xEA, d, 2);   /* Drv Timing B */
    d[0]=0x26;                              lcd_cmd(0xC0, d, 1);   /* Pwr Ctrl 1   */
    d[0]=0x11;                              lcd_cmd(0xC1, d, 1);   /* Pwr Ctrl 2   */
    d[0]=0x35; d[1]=0x3E;                   lcd_cmd(0xC5, d, 2);   /* VCOM 1       */
    d[0]=0xBE;                              lcd_cmd(0xC7, d, 1);   /* VCOM 2       */

    /* MADCTL: MV=1 (landscape) | BGR=0  (try RGB order)               */
    d[0] = 0x20;                            lcd_cmd(0x36, d, 1);

    d[0] = 0x55;                            lcd_cmd(0x3A, d, 1);   /* 16-bit 565   */
    d[0]=0x00; d[1]=0x1B;                   lcd_cmd(0xB1, d, 2);   /* 70 Hz        */

    d[0]=0x08;                              lcd_cmd(0xF2, d, 1);   /* 3-gamma off  */
    d[0]=0x01;                              lcd_cmd(0x26, d, 1);   /* gamma set 1  */

    { static const uint8_t pg[] = { 0x1F,0x1A,0x18,0x0A,0x0F,0x06,
            0x45,0x87,0x32,0x0A,0x07,0x02,0x07,0x05,0x00 };
      lcd_cmd(0xE0, pg, sizeof(pg)); }

    { static const uint8_t ng[] = { 0x00,0x25,0x27,0x05,0x10,0x09,
            0x3A,0x78,0x4D,0x05,0x18,0x0D,0x38,0x3A,0x1F };
      lcd_cmd(0xE1, ng, sizeof(ng)); }

    lcd_cmd(0x11, NULL, 0);   vTaskDelay(pdMS_TO_TICKS(120));  /* sleep out  */
    lcd_cmd(0x29, NULL, 0);                                     /* display on */

    ESP_LOGI(TAG, "ILI9341 ready  %dx%d landscape", LCD_W, LCD_H);
}

/* ── TCP helpers ────────────────────────────────────────────────── */

static bool tcp_recv_all(int fd, void *buf, size_t len)
{
    uint8_t *p = buf;
    while (len) {
        int n = recv(fd, p, len, 0);
        if (n <= 0) return false;
        p   += n;
        len -= (size_t)n;
    }
    return true;
}

/* ── Video receive / decode / display task ──────────────────────── */

static void display_task(void *arg)
{
    /* Block until WiFi is associated */
    xEventGroupWaitBits(wifi_eg, WIFI_UP, pdFALSE, pdTRUE, portMAX_DELAY);

    /* Buffers in PSRAM */
    uint8_t *jpeg = heap_caps_malloc(VIDEO_MAX_FRAME, MALLOC_CAP_SPIRAM);
    uint8_t *rgb  = heap_caps_malloc(LCD_FB, MALLOC_CAP_SPIRAM);
    if (!jpeg || !rgb) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        vTaskDelete(NULL); return;
    }

    uint32_t frames = 0;

    for (;;) {
        /* ── connect to car ── */
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket() failed");
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }

        struct sockaddr_in srv = {
            .sin_family      = AF_INET,
            .sin_port        = htons(VIDEO_TCP_PORT),
            .sin_addr.s_addr = inet_addr("192.168.4.1"),
        };

        ESP_LOGI(TAG, "Connecting to car ...");
        if (connect(sock, (struct sockaddr *)&srv, sizeof(srv)) < 0) {
            ESP_LOGW(TAG, "TCP connect failed — retry in 1 s");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000)); continue;
        }
        int flag = 1;
        setsockopt(sock, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
        setsockopt(sock, SOL_SOCKET,  SO_KEEPALIVE, &flag, sizeof(flag));

        /* Recv timeout: 10 s gives the camera plenty of slack for the
         * occasional slow frame while still catching a truly dead link. */
        struct timeval rcv_tv = { .tv_sec = 10, .tv_usec = 0 };
        setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &rcv_tv, sizeof(rcv_tv));

        ESP_LOGI(TAG, "Stream connected");

        /* ── frame loop ── */
        bool alive = true;
        while (alive) {
            /* Read 4-byte length header */
            uint32_t net_len = 0;
            if (!tcp_recv_all(sock, &net_len, 4)) { alive = false; break; }
            uint32_t jlen = ntohl(net_len);

            if (jlen == 0 || jlen > VIDEO_MAX_FRAME) {
                ESP_LOGW(TAG, "bad frame len %lu", (unsigned long)jlen);
                alive = false; break;
            }

            /* Read JPEG payload */
            if (!tcp_recv_all(sock, jpeg, jlen)) { alive = false; break; }

            /* Decode JPEG → RGB565 (little-endian output).
             * Mute the JPEG library's internal error log for the duration:
             * the OV2640 occasionally emits internally-corrupt frames that
             * pass SOI/EOI validation but fail the decoder (JDR_FMT1/6).
             * We already handle the failure gracefully below. */
            esp_log_level_set("JPEG", ESP_LOG_NONE);
            bool ok = jpg2rgb565(jpeg, jlen, rgb, JPG_SCALE_NONE);
            esp_log_level_set("JPEG", ESP_LOG_ERROR);

            if (!ok) {
                ESP_LOGW(TAG, "decode failed (%lu B)", (unsigned long)jlen);
                continue;
            }

            /* Byte-swap every pixel: LE → BE for ILI9341 */
            for (size_t i = 0; i < LCD_FB; i += 2) {
                uint8_t t = rgb[i];
                rgb[i]    = rgb[i + 1];
                rgb[i + 1] = t;
            }

            lcd_flush(rgb);
            frames++;

            if (frames <= 3 || (frames & 0x3F) == 0)
                ESP_LOGI(TAG, "displayed %lu", (unsigned long)frames);
        }

        close(sock);
        ESP_LOGW(TAG, "Stream lost — reconnecting");
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* ── Public entry point ─────────────────────────────────────────── */

void display_stream_init(void)
{
    lcd_init();

    /* Show red / green / blue test bars (RGB565 big-endian) */
    {
        uint8_t *tb = heap_caps_malloc(LCD_FB, MALLOC_CAP_SPIRAM);
        if (tb) {
            for (int y = 0; y < LCD_H; y++) {
                for (int x = 0; x < LCD_W; x++) {
                    uint16_t c;
                    if      (x < LCD_W / 3)     c = 0xF800;    /* red   */
                    else if (x < 2 * LCD_W / 3) c = 0x07E0;    /* green */
                    else                         c = 0x001F;    /* blue  */
                    size_t off = (y * LCD_W + x) * 2;
                    tb[off]     = c >> 8;
                    tb[off + 1] = c & 0xFF;
                }
            }
            lcd_flush(tb);
            free(tb);
            ESP_LOGI(TAG, "Test bars — 2 s");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }

    wifi_sta_init();
    xTaskCreate(display_task, "disp_rx", 16384, NULL, 4, NULL);
}

#endif /* BUILD_REMOTE */
