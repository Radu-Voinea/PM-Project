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
#include "img_converters.h"          /* jpg2rgb565() from esp32-camera */

static const char *TAG = "disp_rx";

/* ── ILI9341 8080-parallel pin mapping ──────────────────────────── *
 *
 *  IMPORTANT — hardware notes
 *  ──────────────────────────
 *  • LCD_RD must be held HIGH (tie to 3.3 V) because the ESP-IDF
 *    i80 bus is write-only.  Do NOT wire RD to the same GPIO as WR.
 *  • Backlight (LED/BL pin): wire to 3.3 V through an appropriate
 *    resistor, or control via a spare GPIO if brightness is needed.
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
#define LCD_RS   GPIO_NUM_38       /* DC / RS  (data / command) */
#define LCD_WR   GPIO_NUM_47       /* write strobe              */
/* LCD_RD — not driven; tie HIGH externally                       */

#define LCD_H_RES  320
#define LCD_V_RES  240

/* ── LCD globals ────────────────────────────────────────────────── */
static esp_lcd_panel_io_handle_t lcd_io;
static SemaphoreHandle_t flush_done;

/* Called by DMA when color transfer completes */
static bool on_color_done(esp_lcd_panel_io_handle_t io,
                          esp_lcd_panel_io_event_data_t *data,
                          void *user_ctx)
{
    BaseType_t wake = pdFALSE;
    xSemaphoreGiveFromISR(flush_done, &wake);
    return (wake == pdTRUE);
}

/* ── WiFi ───────────────────────────────────────────────────────── */
static EventGroupHandle_t wifi_events;
#define WIFI_CONNECTED_BIT  BIT0

static void wifi_event_handler(void *arg, esp_event_base_t base,
                                int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        if (id == WIFI_EVENT_STA_START || id == WIFI_EVENT_STA_DISCONNECTED) {
            if (id == WIFI_EVENT_STA_DISCONNECTED)
                xEventGroupClearBits(wifi_events, WIFI_CONNECTED_BIT);
            esp_wifi_connect();
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(wifi_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi connected — got IP");
    }
}

static void wifi_sta_init(void)
{
    wifi_events = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                               wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                               wifi_event_handler, NULL);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t sta_cfg = {
        .sta = {
            .ssid     = VIDEO_WIFI_SSID,
            .password = VIDEO_WIFI_PASS,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi STA started — connecting to \"%s\"", VIDEO_WIFI_SSID);
}

/* ── ILI9341 init (8080-parallel, landscape 320×240) ────────────── */
static void lcd_cmd(uint8_t cmd, const uint8_t *data, size_t len)
{
    esp_lcd_panel_io_tx_param(lcd_io, cmd, data, len);
}

static void lcd_init(void)
{
    /* Hardware reset */
    gpio_config_t rst_cfg = {
        .pin_bit_mask = 1ULL << LCD_RST,
        .mode         = GPIO_MODE_OUTPUT,
    };
    gpio_config(&rst_cfg);
    gpio_set_level(LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(120));

    /* Create Intel 8080 bus */
    esp_lcd_i80_bus_handle_t bus = NULL;
    esp_lcd_i80_bus_config_t bus_cfg = {
        .dc_gpio_num   = LCD_RS,
        .wr_gpio_num   = LCD_WR,
        .clk_src       = LCD_CLK_SRC_DEFAULT,
        .data_gpio_nums = {
            LCD_D0, LCD_D1, LCD_D2, LCD_D3,
            LCD_D4, LCD_D5, LCD_D6, LCD_D7,
        },
        .bus_width          = 8,
        .max_transfer_bytes = LCD_H_RES * LCD_V_RES * 2 + 64,
        .psram_trans_align  = 64,
        .sram_trans_align   = 4,
    };
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_cfg, &bus));

    /* Panel IO on the bus */
    flush_done = xSemaphoreCreateBinary();

    esp_lcd_panel_io_i80_config_t io_cfg = {
        .cs_gpio_num    = LCD_CS,
        .pclk_hz        = 8 * 1000 * 1000,
        .trans_queue_depth = 4,
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

    /* ── ILI9341 register initialisation ────────────────────────── */
    uint8_t d[16];

    d[0]=0x00; d[1]=0x83; d[2]=0x30;
    lcd_cmd(0xCF, d, 3);                           /* Power Control B   */

    d[0]=0x64; d[1]=0x03; d[2]=0x12; d[3]=0x81;
    lcd_cmd(0xED, d, 4);                           /* Power On Sequence */

    d[0]=0x85; d[1]=0x01; d[2]=0x79;
    lcd_cmd(0xE8, d, 3);                           /* Driver Timing A   */

    d[0]=0x39; d[1]=0x2C; d[2]=0x00; d[3]=0x34; d[4]=0x02;
    lcd_cmd(0xCB, d, 5);                           /* Power Control A   */

    d[0]=0x20;
    lcd_cmd(0xF7, d, 1);                           /* Pump Ratio Ctrl   */

    d[0]=0x00; d[1]=0x00;
    lcd_cmd(0xEA, d, 2);                           /* Driver Timing B   */

    d[0]=0x26;
    lcd_cmd(0xC0, d, 1);                           /* Power Control 1   */

    d[0]=0x11;
    lcd_cmd(0xC1, d, 1);                           /* Power Control 2   */

    d[0]=0x35; d[1]=0x3E;
    lcd_cmd(0xC5, d, 2);                           /* VCOM Control 1    */

    d[0]=0xBE;
    lcd_cmd(0xC7, d, 1);                           /* VCOM Control 2    */

    /* Memory Access Control — landscape + BGR
     * 0x28 = MV=1 (row/col swap) | BGR=1                            */
    d[0]=0x28;
    lcd_cmd(0x36, d, 1);

    /* Pixel format: 16-bit RGB565 */
    d[0]=0x55;
    lcd_cmd(0x3A, d, 1);

    /* Frame rate: 70 Hz */
    d[0]=0x00; d[1]=0x1B;
    lcd_cmd(0xB1, d, 2);

    /* 3-Gamma function disable */
    d[0]=0x08;
    lcd_cmd(0xF2, d, 1);

    /* Gamma curve 1 */
    d[0]=0x01;
    lcd_cmd(0x26, d, 1);

    /* Positive gamma correction */
    {
        const uint8_t pg[] = { 0x1F,0x1A,0x18,0x0A,0x0F,0x06,0x45,0x87,
                                0x32,0x0A,0x07,0x02,0x07,0x05,0x00 };
        lcd_cmd(0xE0, pg, sizeof(pg));
    }
    /* Negative gamma correction */
    {
        const uint8_t ng[] = { 0x00,0x25,0x27,0x05,0x10,0x09,0x3A,0x78,
                                0x4D,0x05,0x18,0x0D,0x38,0x3A,0x1F };
        lcd_cmd(0xE1, ng, sizeof(ng));
    }

    /* Sleep out + display on */
    lcd_cmd(0x11, NULL, 0);
    vTaskDelay(pdMS_TO_TICKS(120));
    lcd_cmd(0x29, NULL, 0);

    ESP_LOGI(TAG, "ILI9341 initialised  (%dx%d landscape)", LCD_H_RES, LCD_V_RES);
}

/* Push a full-screen RGB565 buffer to the display */
static void lcd_flush(const uint8_t *rgb565)
{
    /* Set column address 0 … 319 */
    uint8_t ca[] = { 0, 0, (LCD_H_RES-1)>>8, (LCD_H_RES-1)&0xFF };
    esp_lcd_panel_io_tx_param(lcd_io, 0x2A, ca, 4);

    /* Set row address 0 … 239 */
    uint8_t ra[] = { 0, 0, (LCD_V_RES-1)>>8, (LCD_V_RES-1)&0xFF };
    esp_lcd_panel_io_tx_param(lcd_io, 0x2B, ra, 4);

    /* Start DMA transfer and block until it completes */
    esp_lcd_panel_io_tx_color(lcd_io, 0x2C, rgb565,
                              LCD_H_RES * LCD_V_RES * 2);
    xSemaphoreTake(flush_done, portMAX_DELAY);
}

/* Fill screen with a solid colour (RGB565, big-endian for ILI9341) */
static void lcd_fill(uint16_t colour)
{
    size_t sz = LCD_H_RES * LCD_V_RES * 2;
    uint8_t *buf = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
    if (!buf) return;
    for (size_t i = 0; i < sz; i += 2) {
        buf[i]   = colour >> 8;
        buf[i+1] = colour & 0xFF;
    }
    lcd_flush(buf);
    free(buf);
}

/* ── Helper: receive exactly `len` bytes from TCP ──────────────── */
static bool tcp_recv_all(int fd, void *buf, size_t len)
{
    uint8_t *p = (uint8_t *)buf;
    while (len > 0) {
        int n = recv(fd, p, len, 0);
        if (n <= 0) return false;
        p   += n;
        len -= (size_t)n;
    }
    return true;
}

/* ── Video receive + decode + display task ──────────────────────── */
static void display_task(void *param)
{
    /* Wait for WiFi connection */
    xEventGroupWaitBits(wifi_events, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    /* Allocate buffers in PSRAM */
    uint8_t *jpeg_buf = heap_caps_malloc(VIDEO_MAX_FRAME, MALLOC_CAP_SPIRAM);
    uint8_t *rgb_buf  = heap_caps_malloc(LCD_H_RES * LCD_V_RES * 2,
                                          MALLOC_CAP_SPIRAM);
    if (!jpeg_buf || !rgb_buf) {
        ESP_LOGE(TAG, "PSRAM alloc failed");
        vTaskDelete(NULL);
        return;
    }

    uint32_t frames_ok = 0;

    for (;;) {
        /* Connect to car's TCP video server */
        int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (sock < 0) {
            ESP_LOGE(TAG, "socket() failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        struct sockaddr_in server = {
            .sin_family      = AF_INET,
            .sin_port        = htons(VIDEO_TCP_PORT),
            .sin_addr.s_addr = inet_addr("192.168.4.1"),  /* car AP address */
        };

        ESP_LOGI(TAG, "Connecting to video server ...");
        if (connect(sock, (struct sockaddr *)&server, sizeof(server)) < 0) {
            ESP_LOGW(TAG, "TCP connect failed — retrying in 1 s");
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        ESP_LOGI(TAG, "Connected to video stream");

        bool alive = true;
        while (alive) {
            /* Read 4-byte frame length */
            uint32_t net_len = 0;
            if (!tcp_recv_all(sock, &net_len, 4)) { alive = false; break; }
            uint32_t jpeg_len = ntohl(net_len);

            if (jpeg_len == 0 || jpeg_len > VIDEO_MAX_FRAME) {
                ESP_LOGW(TAG, "Bad frame length: %lu", (unsigned long)jpeg_len);
                alive = false;
                break;
            }

            /* Read JPEG payload */
            if (!tcp_recv_all(sock, jpeg_buf, jpeg_len)) { alive = false; break; }

            /* Log first frame JPEG header for diagnostics */
            if (frames_ok == 0) {
                ESP_LOGI(TAG, "JPEG recv: len=%lu  hdr=%02X %02X %02X %02X %02X %02X",
                         (unsigned long)jpeg_len,
                         jpeg_buf[0], jpeg_buf[1], jpeg_buf[2],
                         jpeg_buf[3], jpeg_buf[4], jpeg_buf[5]);
            }

            /* Decode JPEG → RGB565 */
            bool ok = jpg2rgb565(jpeg_buf, jpeg_len, rgb_buf, JPG_SCALE_NONE);

            if (frames_ok == 0) {
                ESP_LOGI(TAG, "jpg2rgb565 returned %s", ok ? "TRUE" : "FALSE");
                if (ok) {
                    ESP_LOGI(TAG, "RGB565 first 8 bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                             rgb_buf[0], rgb_buf[1], rgb_buf[2], rgb_buf[3],
                             rgb_buf[4], rgb_buf[5], rgb_buf[6], rgb_buf[7]);
                }
            }

            if (ok) {
                /* Byte-swap LE→BE for ILI9341 */
                size_t npx = LCD_H_RES * LCD_V_RES * 2;
                for (size_t i = 0; i < npx; i += 2) {
                    uint8_t t    = rgb_buf[i];
                    rgb_buf[i]   = rgb_buf[i+1];
                    rgb_buf[i+1] = t;
                }
                lcd_flush(rgb_buf);
                frames_ok++;
                if (frames_ok <= 3 || (frames_ok & 63) == 0)
                    ESP_LOGI(TAG, "frames displayed: %lu", (unsigned long)frames_ok);
            } else {
                ESP_LOGW(TAG, "JPEG decode failed  len=%lu", (unsigned long)jpeg_len);
            }
        }

        close(sock);
        ESP_LOGW(TAG, "Stream disconnected — reconnecting");
    }
}

/* ── Public init ────────────────────────────────────────────────── */
void display_stream_init(void)
{
    lcd_init();

    /* Test bars: Red | Green | Blue  (RGB565 big-endian) */
    {
        size_t sz = LCD_H_RES * LCD_V_RES * 2;
        uint8_t *tb = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
        if (tb) {
            for (int y = 0; y < LCD_V_RES; y++) {
                for (int x = 0; x < LCD_H_RES; x++) {
                    uint16_t c;
                    if      (x < LCD_H_RES / 3) c = 0xF800; /* red   */
                    else if (x < 2*LCD_H_RES/3) c = 0x07E0; /* green */
                    else                         c = 0x001F; /* blue  */
                    size_t off = (y * LCD_H_RES + x) * 2;
                    tb[off]   = c >> 8;      /* big-endian */
                    tb[off+1] = c & 0xFF;
                }
            }
            lcd_flush(tb);
            free(tb);
            ESP_LOGI(TAG, "Test bars displayed — 2 s pause");
            vTaskDelay(pdMS_TO_TICKS(2000));
        }
    }
    wifi_sta_init();
    xTaskCreate(display_task, "disp_rx", 8192, NULL, 4, NULL);
}

#endif /* BUILD_REMOTE */
