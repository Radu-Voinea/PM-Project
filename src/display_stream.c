#ifdef BUILD_REMOTE

#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_lcd_panel_io.h"

#include "img_converters.h"         /* jpg2rgb565() from esp32-camera  */
#include "display_stream.h"
#include "wifi_video.h"

/* ── Logging ─────────────────────────────────────────────────────── */
static const char *TAG = "disp_stream";

/* ── LCD I80 pin map ─────────────────────────────────────────────── *
 * GPIO46 = LCD_RD  (strapping pin: ROM message print).              *
 *   Idle state = HIGH (RD active-LOW).  GPIO46 HIGH at boot         *
 *   → ROM messages suppressed.  Harmless side-effect.               *
 * GPIO47 = LCD_WR  Idle = HIGH (active-LOW write strobe). Safe.     */
#define LCD_PIN_D0   12
#define LCD_PIN_D1   11
#define LCD_PIN_D2   48
#define LCD_PIN_D3   21
#define LCD_PIN_D4   18
#define LCD_PIN_D5   17
#define LCD_PIN_D6   14
#define LCD_PIN_D7   13
#define LCD_PIN_RST  42
#define LCD_PIN_CS   41
#define LCD_PIN_DC   38   /* RS / D-C */
#define LCD_PIN_WR   47
#define LCD_PIN_RD   46   /* always HIGH; we never read back */

#define LCD_H_RES    320
#define LCD_V_RES    240
#define LCD_CLOCK_HZ (16 * 1000 * 1000)

/* ── ILI9341 command bytes ───────────────────────────────────────── */
#define ILI_SWRESET  0x01
#define ILI_SLPOUT   0x11
#define ILI_MADCTL   0x36
#define ILI_COLMOD   0x3A
#define ILI_FRMCTR1  0xB1
#define ILI_DFUNCTR  0xB6
#define ILI_PWCTR1   0xC0
#define ILI_PWCTR2   0xC1
#define ILI_VMCTR1   0xC5
#define ILI_VMCTR2   0xC7
#define ILI_GAMMASET 0x26
#define ILI_GMCTRP1  0xE0
#define ILI_GMCTRN1  0xE1
#define ILI_NORON    0x13
#define ILI_DISPON   0x29
#define ILI_CASET    0x2A
#define ILI_RASET    0x2B
#define ILI_RAMWR    0x2C

/* ── Module state ────────────────────────────────────────────────── */
static esp_lcd_panel_io_handle_t s_io     = NULL;
static SemaphoreHandle_t         s_tx_sem = NULL;

/* Frame reassembly buffer (in PSRAM) */
static uint8_t  *s_jpeg_buf  = NULL;
static uint16_t *s_rgb_buf   = NULL;

/* ── DMA-complete callback ───────────────────────────────────────── */
static bool IRAM_ATTR on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                          esp_lcd_panel_io_event_data_t *edata,
                                          void *user_ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR((SemaphoreHandle_t)user_ctx, &woken);
    return woken == pdTRUE;
}

/* ─────────────────────────────────────────────────────────────────── */
static void lcd_send_cmd(uint8_t cmd)
{
    esp_lcd_panel_io_tx_param(s_io, cmd, NULL, 0);
}

static void lcd_send_cmd_data(uint8_t cmd, const uint8_t *data, size_t len)
{
    esp_lcd_panel_io_tx_param(s_io, cmd, data, len);
}

/* ─────────────────────────────────────────────────────────────────── */
static void ili9341_init(void)
{
    lcd_send_cmd(ILI_SWRESET);
    vTaskDelay(pdMS_TO_TICKS(120));

    lcd_send_cmd(ILI_SLPOUT);
    vTaskDelay(pdMS_TO_TICKS(5));

    lcd_send_cmd_data(ILI_PWCTR1,  (uint8_t[]){0x23}, 1);
    lcd_send_cmd_data(ILI_PWCTR2,  (uint8_t[]){0x10}, 1);
    lcd_send_cmd_data(ILI_VMCTR1,  (uint8_t[]){0x3E, 0x28}, 2);
    lcd_send_cmd_data(ILI_VMCTR2,  (uint8_t[]){0x86}, 1);

    /* Landscape 320×240, RGB order (bit3=0).
     * If R/B appear swapped on your module, change 0x28 → 0x08 (adds BGR bit). */
    lcd_send_cmd_data(ILI_MADCTL,  (uint8_t[]){0x28}, 1);

    /* 16-bit RGB565 */
    lcd_send_cmd_data(ILI_COLMOD,  (uint8_t[]){0x55}, 1);

    lcd_send_cmd_data(ILI_FRMCTR1, (uint8_t[]){0x00, 0x18}, 2);
    lcd_send_cmd_data(ILI_DFUNCTR, (uint8_t[]){0x08, 0x82, 0x27}, 3);

    lcd_send_cmd_data(ILI_GAMMASET, (uint8_t[]){0x01}, 1);
    lcd_send_cmd_data(ILI_GMCTRP1, (uint8_t[]){
        0x0F,0x31,0x2B,0x0C,0x0E,0x08,0x4E,0xF1,
        0x37,0x07,0x10,0x03,0x0E,0x09,0x00}, 15);
    lcd_send_cmd_data(ILI_GMCTRN1, (uint8_t[]){
        0x00,0x0E,0x14,0x03,0x11,0x07,0x31,0xC1,
        0x48,0x08,0x0F,0x0C,0x31,0x36,0x0F}, 15);

    lcd_send_cmd(ILI_NORON);
    vTaskDelay(pdMS_TO_TICKS(10));
    lcd_send_cmd(ILI_DISPON);
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "ILI9341 initialised (%d×%d)", LCD_H_RES, LCD_V_RES);
}

/* Draw a full-screen RGB565 buffer via DMA ─────────────────────────── */
static void lcd_draw_full(const void *rgb565)
{
    /* Set column window 0..319 */
    uint8_t col[4] = {0x00, 0x00,
                      (uint8_t)((LCD_H_RES - 1) >> 8),
                      (uint8_t)((LCD_H_RES - 1) & 0xFF)};
    lcd_send_cmd_data(ILI_CASET, col, 4);

    /* Set row window 0..239 */
    uint8_t row[4] = {0x00, 0x00,
                      (uint8_t)((LCD_V_RES - 1) >> 8),
                      (uint8_t)((LCD_V_RES - 1) & 0xFF)};
    lcd_send_cmd_data(ILI_RASET, row, 4);

    /* Transfer pixel data; callback gives semaphore when DMA finishes */
    xSemaphoreTake(s_tx_sem, 0);   /* clear any leftover token */
    esp_lcd_panel_io_tx_color(s_io, ILI_RAMWR,
                              rgb565, LCD_H_RES * LCD_V_RES * 2);
    xSemaphoreTake(s_tx_sem, portMAX_DELAY);  /* wait for DMA done */
}

/* ─────────────────────────────────────────────────────────────────── */
static void lcd_hw_init(void)
{
    /* RD pin: tie HIGH permanently (we never read back from the panel) */
    gpio_config_t rd_cfg = {
        .pin_bit_mask = 1ULL << LCD_PIN_RD,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rd_cfg);
    gpio_set_level(LCD_PIN_RD, 1);

    /* Hardware reset */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << LCD_PIN_RST,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(LCD_PIN_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(LCD_PIN_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    /* I80 bus */
    esp_lcd_i80_bus_handle_t bus;
    esp_lcd_i80_bus_config_t bus_cfg = {
        .dc_gpio_num  = LCD_PIN_DC,
        .wr_gpio_num  = LCD_PIN_WR,
        .clk_src      = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = {
            LCD_PIN_D0, LCD_PIN_D1, LCD_PIN_D2, LCD_PIN_D3,
            LCD_PIN_D4, LCD_PIN_D5, LCD_PIN_D6, LCD_PIN_D7,
        },
        .bus_width         = 8,
        .max_transfer_bytes = LCD_H_RES * LCD_V_RES * 2 + 16,
        .psram_trans_align  = 64,
        .sram_trans_align   = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_cfg, &bus));

    /* Panel IO */
    s_tx_sem = xSemaphoreCreateBinary();
    esp_lcd_panel_io_i80_config_t io_cfg = {
        .cs_gpio_num       = LCD_PIN_CS,
        .pclk_hz           = LCD_CLOCK_HZ,
        .trans_queue_depth = 10,
        .on_color_trans_done = on_color_trans_done,
        .user_ctx          = s_tx_sem,
        .lcd_cmd_bits      = 8,
        .lcd_param_bits    = 8,
        .flags = {
            .swap_color_bytes = 1,  /* ILI9341 wants big-endian; ESP is LE */
            .cs_active_high   = 0,
            .pclk_active_neg  = 0,
        },
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(bus, &io_cfg, &s_io));

    /* Allocate framebuffers in PSRAM */
    s_jpeg_buf = (uint8_t  *)heap_caps_malloc(VIDEO_MAX_FRAME,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_rgb_buf  = (uint16_t *)heap_caps_malloc(LCD_H_RES * LCD_V_RES * 2,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_jpeg_buf || !s_rgb_buf) {
        ESP_LOGE(TAG, "PSRAM allocation failed");
        return;
    }

    ili9341_init();
}

/* ── WiFi STA ────────────────────────────────────────────────────── */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static EventGroupHandle_t s_wifi_evg;
static int s_retry_count = 0;
#define MAX_RETRIES 10

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data)
{
    if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_count < MAX_RETRIES) {
            esp_wifi_connect();
            s_retry_count++;
            ESP_LOGW(TAG, "WiFi reconnect attempt %d", s_retry_count);
        } else {
            xEventGroupSetBits(s_wifi_evg, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "WiFi connected, IP=" IPSTR, IP2STR(&e->ip_info.ip));
        s_retry_count = 0;
        xEventGroupSetBits(s_wifi_evg, WIFI_CONNECTED_BIT);
    }
}

static void wifi_sta_init(void)
{
    s_wifi_evg = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_cfg = {
        .sta = {
            .ssid     = VIDEO_WIFI_SSID,
            .password = VIDEO_WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(84));

    EventBits_t bits = xEventGroupWaitBits(s_wifi_evg,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE, pdFALSE, portMAX_DELAY);
    if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "WiFi connection failed after %d retries", MAX_RETRIES);
    }
}

/* ── Video receive task ──────────────────────────────────────────── */
static void video_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "socket() failed");
        vTaskDelete(NULL);
        return;
    }

    /* Increase receive buffer */
    int rcvbuf = 128 * 1024;
    setsockopt(sock, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    struct sockaddr_in bind_addr = {
        .sin_family      = AF_INET,
        .sin_port        = htons(VIDEO_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    if (bind(sock, (struct sockaddr *)&bind_addr, sizeof(bind_addr)) < 0) {
        ESP_LOGE(TAG, "bind() failed");
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Video task listening on UDP:%d", VIDEO_UDP_PORT);

    /* Per-packet receive buffer */
    static uint8_t pkt[FRAG_HDR_LEN + FRAG_MAX_PAYLOAD];

    /* Reassembly state */
    uint16_t cur_frame_id  = 0xFFFF;
    uint8_t  frags_expected = 0;
    uint8_t  frags_got      = 0;
    size_t   frame_len      = 0;

    uint32_t rx_frames = 0;

    while (true) {
        int len = recv(sock, pkt, sizeof(pkt), 0);
        if (len < FRAG_HDR_LEN) continue;

        uint16_t frame_id  = ((uint16_t)pkt[0] << 8) | pkt[1];
        uint8_t  frag_idx  = pkt[2];
        uint8_t  frag_cnt  = pkt[3];
        size_t   pay_len   = len - FRAG_HDR_LEN;

        /* New frame → reset reassembly state */
        if (frame_id != cur_frame_id) {
            cur_frame_id   = frame_id;
            frags_expected = frag_cnt;
            frags_got      = 0;
            frame_len      = 0;
        }

        /* Copy payload into contiguous JPEG buffer */
        size_t offset = (size_t)frag_idx * FRAG_MAX_PAYLOAD;
        if (offset + pay_len <= VIDEO_MAX_FRAME) {
            memcpy(s_jpeg_buf + offset, pkt + FRAG_HDR_LEN, pay_len);
            frags_got++;
            if (frag_idx == frag_cnt - 1) {
                frame_len = offset + pay_len;  /* last fragment sets total size */
            }
        }

        /* All fragments received → decode and display */
        if (frags_got == frags_expected && frame_len >= MIN_VALID_JPEG_LEN) {
            /* Validate JPEG SOI */
            if (s_jpeg_buf[0] == 0xFF && s_jpeg_buf[1] == 0xD8) {
                /* jpg2rgb565 decodes JPEG to RGB565 big-endian byte pairs */
                bool ok = jpg2rgb565(s_jpeg_buf, frame_len,
                                     (uint8_t *)s_rgb_buf, JPG_SCALE_NONE);
                if (ok) {
                    lcd_draw_full(s_rgb_buf);
                    if ((++rx_frames & 0x3F) == 0) {
                        ESP_LOGI(TAG, "Displayed %"PRIu32" frames", rx_frames);
                    }
                }
            }
            frags_got = 0;
        }
    }
}

/* ── Public API ──────────────────────────────────────────────────── */
void display_stream_init(void)
{
    lcd_hw_init();
    wifi_sta_init();
}

void display_stream_start(void)
{
    xTaskCreatePinnedToCore(video_task, "video_rx",
                            8192, NULL, 6, NULL, 0);
}

#endif /* BUILD_REMOTE */
