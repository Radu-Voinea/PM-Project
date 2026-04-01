#ifdef BUILD_REMOTE

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

#include "ble_common.h"
#include "display_stream.h"
#include "remote.h"

static const char *TAG = "remote";

/* ═══════════════════════════════════════════════════════════════════ *
 *  JOYSTICK INPUT                                                     *
 * ═══════════════════════════════════════════════════════════════════ *
 *  Joystick 1 (movement):  Vx=GPIO1 (CH0), Vy=GPIO2 (CH1), SW=GPIO3 *
 *  Joystick 2 (pan):       Vx=GPIO4 (CH3), Vy=GPIO5 (CH4), SW=GPIO6 *
 *  ADC1, 12-bit, Atten DB_12 (-12 dB, full 0-3.3 V range)           */
#define JS1_VX_CH   ADC_CHANNEL_0   /* GPIO1 */
#define JS1_VY_CH   ADC_CHANNEL_1   /* GPIO2 */
#define JS1_SW_GPIO 3
#define JS2_VX_CH   ADC_CHANNEL_3   /* GPIO4 */
#define JS2_VY_CH   ADC_CHANNEL_4   /* GPIO5 */
#define JS2_SW_GPIO 6

#define JS1_DEADZONE  200
#define JS2_DEADZONE  350
#define ADC_CAL_SAMP  64
#define ADC_MIDPOINT  2048

/* ── Speed LED pins ─────────────────────────────────────────────── */
#define LED_SPD1_GPIO  7   /* 30% */
#define LED_SPD2_GPIO  8   /* 60% */
#define LED_SPD3_GPIO  9   /* 100% */
#define LED_NC_GPIO    10  /* not-connected indicator (inverted logic) */

static const uint8_t k_speed_pct[3] = { 30, 60, 100 };

/* ── Module state ───────────────────────────────────────────────── */
static adc_oneshot_unit_handle_t s_adc;
static int s_js1_mid_x = ADC_MIDPOINT;
static int s_js1_mid_y = ADC_MIDPOINT;
static int s_js2_mid_x = ADC_MIDPOINT;
static int s_js2_mid_y = ADC_MIDPOINT;
static uint8_t s_speed_idx = 0;   /* 0=30% 1=60% 2=100% */

static void joystick_init(void)
{
    /* ADC */
    adc_oneshot_unit_init_cfg_t adc_cfg = { .unit_id = ADC_UNIT_1 };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&adc_cfg, &s_adc));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, JS1_VX_CH, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, JS1_VY_CH, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, JS2_VX_CH, &ch_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, JS2_VY_CH, &ch_cfg));

    /* Switch GPIOs with pull-up */
    gpio_config_t sw_cfg = {
        .pin_bit_mask  = (1ULL << JS1_SW_GPIO) | (1ULL << JS2_SW_GPIO),
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&sw_cfg));

    /* Calibrate centre positions (64 samples each) */
    int sum = 0;
    for (int i = 0; i < ADC_CAL_SAMP; i++) {
        int v; adc_oneshot_read(s_adc, JS1_VX_CH, &v); sum += v;
    }
    s_js1_mid_x = sum / ADC_CAL_SAMP;

    sum = 0;
    for (int i = 0; i < ADC_CAL_SAMP; i++) {
        int v; adc_oneshot_read(s_adc, JS1_VY_CH, &v); sum += v;
    }
    s_js1_mid_y = sum / ADC_CAL_SAMP;

    sum = 0;
    for (int i = 0; i < ADC_CAL_SAMP; i++) {
        int v; adc_oneshot_read(s_adc, JS2_VX_CH, &v); sum += v;
    }
    s_js2_mid_x = sum / ADC_CAL_SAMP;

    sum = 0;
    for (int i = 0; i < ADC_CAL_SAMP; i++) {
        int v; adc_oneshot_read(s_adc, JS2_VY_CH, &v); sum += v;
    }
    s_js2_mid_y = sum / ADC_CAL_SAMP;

    ESP_LOGI(TAG, "Joystick calibrated: JS1 mid(%d,%d)  JS2 mid(%d,%d)",
             s_js1_mid_x, s_js1_mid_y, s_js2_mid_x, s_js2_mid_y);
}

/* Map raw ADC reading to -100…+100 with deadzone */
static int8_t js_scale(int raw, int mid, int deadzone)
{
    int delta = raw - mid;
    if (delta > -deadzone && delta < deadzone) return 0;
    /* Scale remaining range to ±100 */
    int range = (delta > 0) ? (4095 - mid - deadzone) : (mid - deadzone);
    if (range <= 0) return 0;
    int scaled = (delta > 0)
        ? (delta - deadzone) * 100 / range
        : (delta + deadzone) * 100 / range;
    if (scaled >  100) scaled =  100;
    if (scaled < -100) scaled = -100;
    return (int8_t)scaled;
}

static void leds_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << LED_SPD1_GPIO) | (1ULL << LED_SPD2_GPIO)
                      | (1ULL << LED_SPD3_GPIO) | (1ULL << LED_NC_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));

    /* Initial state: speed mode 0 (30%), NC LED on (inverted) */
    gpio_set_level(LED_SPD1_GPIO, 1);
    gpio_set_level(LED_SPD2_GPIO, 0);
    gpio_set_level(LED_SPD3_GPIO, 0);
    gpio_set_level(LED_NC_GPIO,   1);  /* inverted: ON = not connected */
}

static void update_speed_leds(uint8_t idx)
{
    gpio_set_level(LED_SPD1_GPIO, idx == 0 ? 1 : 0);
    gpio_set_level(LED_SPD2_GPIO, idx == 1 ? 1 : 0);
    gpio_set_level(LED_SPD3_GPIO, idx == 2 ? 1 : 0);
}

/* ═══════════════════════════════════════════════════════════════════ *
 *  BLE CENTRAL  (NimBLE  Coded PHY  GATT client)                     *
 * ═══════════════════════════════════════════════════════════════════ */
static uint16_t s_conn_handle     = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_cmd_val_handle  = 0;
static uint8_t  s_own_addr_type;

/* Called from joystick task to send a command */
static void ble_send_command(const rc_command_t *cmd)
{
    if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE || s_cmd_val_handle == 0) return;
    ble_gattc_write_no_rsp_flat(s_conn_handle, s_cmd_val_handle,
                                cmd, sizeof(rc_command_t));
}

/* ── GATT discovery callbacks ──────────────────────────────────── */
static int chr_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status != 0) return 0;
    if (!chr) return 0;

    if (ble_uuid_u16(&chr->uuid.u) == RC_COMMAND_CHR_UUID) {
        s_cmd_val_handle = chr->val_handle;
        ESP_LOGI(TAG, "Command chr found, val_handle=%d", s_cmd_val_handle);
    }
    return 0;
}

static int svc_disc_cb(uint16_t conn_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status != 0) return 0;
    if (!svc) return 0;

    if (ble_uuid_u16(&svc->uuid.u) == RC_SERVICE_UUID) {
        ESP_LOGI(TAG, "RC service found, discovering characteristics…");
        ble_gattc_disc_all_chrs(conn_handle,
                                svc->start_handle, svc->end_handle,
                                chr_disc_cb, NULL);
    }
    return 0;
}

/* ── Scan and connect ──────────────────────────────────────────── */
static void start_scan(void);

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_EXT_DISC: {
        const struct ble_gap_ext_disc_desc *d = &event->ext_disc;
        if (d->length_data == 0) break;

        /* Look for our device name in the advertising payload */
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, d->data, d->length_data) != 0) break;

        if (fields.name_len == (sizeof(RC_DEVICE_NAME) - 1) &&
            memcmp(fields.name, RC_DEVICE_NAME, fields.name_len) == 0) {

            ESP_LOGI(TAG, "Found RC-Car, connecting…");
            ble_gap_disc_cancel();

            /* Request Coded PHY connection */
            struct ble_gap_conn_params coded_params = {
                .scan_itvl       = 200,
                .scan_window     = 100,
                .itvl_min        = BLE_GAP_CONN_ITVL_MS(30),
                .itvl_max        = BLE_GAP_CONN_ITVL_MS(50),
                .latency         = 0,
                .supervision_timeout = 500,
                .min_ce_len      = 0,
                .max_ce_len      = 0,
            };
            ble_gap_ext_connect(s_own_addr_type, &d->addr,
                                30000,
                                BLE_GAP_LE_PHY_CODED_MASK,
                                NULL, NULL, &coded_params,
                                gap_event_cb, NULL);
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "BLE connected (conn=%d)", s_conn_handle);
            gpio_set_level(LED_NC_GPIO, 0);  /* connected → NC LED off */
            ble_gattc_disc_all_svcs(s_conn_handle, svc_disc_cb, NULL);
        } else {
            ESP_LOGW(TAG, "Connect failed, rescanning…");
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "BLE disconnected (reason=%d), rescanning…",
                 event->disconnect.reason);
        s_conn_handle    = BLE_HS_CONN_HANDLE_NONE;
        s_cmd_val_handle = 0;
        gpio_set_level(LED_NC_GPIO, 1);  /* disconnected → NC LED on */
        start_scan();
        break;

    default:
        break;
    }
    return 0;
}

static void start_scan(void)
{
    struct ble_gap_ext_disc_params coded = {
        .itvl    = 200,   /* 125 ms */
        .window  = 100,   /*  62.5 ms */
        .passive = 0,
    };
    /* Scan only on Coded PHY (NULL = skip 1M PHY) */
    int rc = ble_gap_ext_disc(s_own_addr_type,
                              0, 0,     /* duration=0 (indefinite), period=0 */
                              0, 0, 0,  /* no filter */
                              NULL, &coded,
                              gap_event_cb, NULL);
    if (rc == 0) ESP_LOGI(TAG, "Scanning for RC-Car on Coded PHY…");
    else         ESP_LOGE(TAG, "ext_disc failed: %d", rc);
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset (reason=%d)", reason);
}

static void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    start_scan();
}

static void nimble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

static void ble_init(void)
{
    ESP_ERROR_CHECK(nimble_port_init());

    ble_hs_cfg.reset_cb        = ble_on_reset;
    ble_hs_cfg.sync_cb         = ble_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gap_device_name_set("RC-Remote");

    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "BLE initialised");
}

/* ═══════════════════════════════════════════════════════════════════ *
 *  JOYSTICK POLL TASK  (20 ms)                                        *
 * ═══════════════════════════════════════════════════════════════════ */
static void joystick_task(void *arg)
{
    static bool prev_sw1 = true;
    static bool prev_sw2 = true;

    while (true) {
        int raw;

        /* Read Joystick 1 */
        adc_oneshot_read(s_adc, JS1_VX_CH, &raw);
        int8_t x = js_scale(raw, s_js1_mid_x, JS1_DEADZONE);

        adc_oneshot_read(s_adc, JS1_VY_CH, &raw);
        int8_t y = js_scale(raw, s_js1_mid_y, JS1_DEADZONE);

        /* Read Joystick 2 (camera pan) */
        adc_oneshot_read(s_adc, JS2_VX_CH, &raw);
        int8_t pan_x = js_scale(raw, s_js2_mid_x, JS2_DEADZONE);

        adc_oneshot_read(s_adc, JS2_VY_CH, &raw);
        int8_t pan_y = js_scale(raw, s_js2_mid_y, JS2_DEADZONE);

        /* Switch 1: cycle speed on falling edge (pull-up → pressed = LOW) */
        bool sw1 = gpio_get_level(JS1_SW_GPIO);
        if (!sw1 && prev_sw1) {
            s_speed_idx = (s_speed_idx + 1) % 3;
            update_speed_leds(s_speed_idx);
            ESP_LOGI(TAG, "Speed: %d%%", k_speed_pct[s_speed_idx]);
        }
        prev_sw1 = sw1;

        /* Switch 2: unused for now, reserved */
        bool sw2 = gpio_get_level(JS2_SW_GPIO);
        (void)sw2;
        prev_sw2 = sw2;

        /* Send BLE command */
        rc_command_t cmd = {
            .x      = x,
            .y      = y,
            .speed  = k_speed_pct[s_speed_idx],
            .pan_x  = pan_x,
            .pan_y  = pan_y,
        };
        ble_send_command(&cmd);

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ═══════════════════════════════════════════════════════════════════ *
 *  ENTRY POINT                                                        *
 * ═══════════════════════════════════════════════════════════════════ */
void remote_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    leds_init();
    joystick_init();

    /* display_stream_init() also connects to the car's WiFi AP */
    display_stream_init();
    display_stream_start();

    ble_init();

    xTaskCreatePinnedToCore(joystick_task, "joystick",
                            4096, NULL, 5, NULL, 1);
}

#endif /* BUILD_REMOTE */
