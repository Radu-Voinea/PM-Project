#ifdef BUILD_REMOTE

#include "remote.h"
#include "ble_common.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

/* NimBLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "rc_remote";

/* ── Joystick ────────────────────────────────────────────────────── */
#define JOY_VRX_CHAN        ADC_CHANNEL_0   /* GPIO 1 */
#define JOY_VRY_CHAN        ADC_CHANNEL_1   /* GPIO 2 */
#define JOY_SW_PIN          GPIO_NUM_3

#define ADC_MAX             4095            /* 12-bit ADC */
#define JOY_DEADZONE        200             /* centre dead-zone (raw counts) */
#define JOY_CAL_SAMPLES     64              /* samples averaged for centre cal */

static int joy_center_x = ADC_MAX / 2;
static int joy_center_y = ADC_MAX / 2;

/* Speed modes cycled by SW button press */
static const uint8_t speed_modes[] = { 30, 60, 100 };
#define SPEED_MODE_COUNT    (sizeof(speed_modes) / sizeof(speed_modes[0]))
static int  speed_mode_idx = 0;
static uint8_t current_speed = 30;

static adc_oneshot_unit_handle_t adc_handle;

/* ── LED GPIOs ───────────────────────────────────────────────────── */
#define LED_NOT_CONNECTED   GPIO_NUM_10
#define LED_SPEED_1         GPIO_NUM_7   /* 30 %  */
#define LED_SPEED_2         GPIO_NUM_8   /* 60 %  */
#define LED_SPEED_3         GPIO_NUM_9   /* 100 % */

static const gpio_num_t speed_leds[] = {
    LED_SPEED_1, LED_SPEED_2, LED_SPEED_3,
};
#define SPEED_LED_COUNT (sizeof(speed_leds) / sizeof(speed_leds[0]))

/* ── BLE state ───────────────────────────────────────────────────── */
static uint16_t conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t cmd_chr_handle = 0;
static volatile bool connected      = false;
static volatile bool chr_discovered  = false;
static uint8_t  own_addr_type;

static int  gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);

/* ── Joystick init ───────────────────────────────────────────────── */
static void joystick_init(void)
{
    /* ADC for VRX / VRY */
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_new_unit(&unit_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(adc_handle, JOY_VRX_CHAN, &chan_cfg);
    adc_oneshot_config_channel(adc_handle, JOY_VRY_CHAN, &chan_cfg);

    /* SW button — active-low with internal pull-up */
    gpio_config_t sw_cfg = {
        .pin_bit_mask  = 1ULL << JOY_SW_PIN,
        .mode          = GPIO_MODE_INPUT,
        .pull_up_en    = GPIO_PULLUP_ENABLE,
        .pull_down_en  = GPIO_PULLDOWN_DISABLE,
        .intr_type     = GPIO_INTR_DISABLE,
    };
    gpio_config(&sw_cfg);
}

/* Calibrate joystick centre from live readings (call at startup, stick untouched) */
static void joystick_calibrate(void)
{
    int32_t sum_x = 0, sum_y = 0;
    for (int i = 0; i < JOY_CAL_SAMPLES; i++) {
        int rx = 0, ry = 0;
        adc_oneshot_read(adc_handle, JOY_VRX_CHAN, &rx);
        adc_oneshot_read(adc_handle, JOY_VRY_CHAN, &ry);
        sum_x += rx;
        sum_y += ry;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    joy_center_x = (int)(sum_x / JOY_CAL_SAMPLES);
    joy_center_y = (int)(sum_y / JOY_CAL_SAMPLES);
    ESP_LOGI(TAG, "Joystick calibrated: center_x=%d  center_y=%d",
             joy_center_x, joy_center_y);
}

/* Map raw ADC to -100…+100 relative to calibrated centre */
static int joy_axis(adc_channel_t ch)
{
    int raw = 0;
    adc_oneshot_read(adc_handle, ch, &raw);

    int center = (ch == JOY_VRX_CHAN) ? joy_center_x : joy_center_y;
    int centered = raw - center;

    if (abs(centered) < JOY_DEADZONE) return 0;

    /* Scale using the actual range on each side of centre */
    int range = (centered > 0) ? (ADC_MAX - center) : center;
    if (range == 0) range = 1;
    return (int)((int32_t)centered * 100 / range);
}

static void gpio_init_leds(void)
{
    const gpio_num_t pins[] = {
        LED_NOT_CONNECTED,
        LED_SPEED_1, LED_SPEED_2, LED_SPEED_3,
    };
    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_config_t cfg = {
            .pin_bit_mask  = 1ULL << pins[i],
            .mode          = GPIO_MODE_OUTPUT,
            .pull_up_en    = GPIO_PULLUP_DISABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
        gpio_set_level(pins[i], 0);
    }
}

static void update_speed_leds(void)
{
    /* Each LED represents one speed mode: 30%→1, 60%→2, 100%→3 */
    int lit = speed_mode_idx + 1;
    for (int i = 0; i < SPEED_LED_COUNT; i++)
        gpio_set_level(speed_leds[i], i < lit ? 1 : 0);
}

static void update_connection_led(void)
{
    gpio_set_level(LED_NOT_CONNECTED, connected ? 0 : 1);
}

/* ── BLE scanning (Coded PHY) ────────────────────────────────────── */
static void start_scan(void)
{
    struct ble_gap_ext_disc_params coded = {
        .passive = 0,
        .itvl    = 0x40,
        .window  = 0x40,
    };

    int rc = ble_gap_ext_disc(own_addr_type,
                              0, 0,         /* duration, period: forever */
                              1,            /* filter duplicates */
                              0,            /* filter policy */
                              0,            /* limited discovery: no */
                              NULL,         /* skip 1 M scanning */
                              &coded,       /* Coded PHY */
                              gap_event_cb, NULL);
    if (rc && rc != BLE_HS_EALREADY)
        ESP_LOGE(TAG, "ext_disc: %d", rc);
    else
        ESP_LOGI(TAG, "Scanning (Coded PHY)");
}

/* ── GATT discovery callbacks ────────────────────────────────────── */
static int on_chr_disc(uint16_t c_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_chr *chr, void *arg)
{
    if (error->status == 0 &&
        ble_uuid_u16(&chr->uuid.u) == RC_COMMAND_CHR_UUID) {
        cmd_chr_handle = chr->val_handle;
        chr_discovered = true;
        ESP_LOGI(TAG, "Characteristic found  handle=%d", cmd_chr_handle);
    }
    return 0;
}

static int on_svc_disc(uint16_t c_handle,
                       const struct ble_gatt_error *error,
                       const struct ble_gatt_svc *svc, void *arg)
{
    if (error->status == 0) {
        ESP_LOGI(TAG, "Service found — discovering chars");
        ble_gattc_disc_all_chrs(c_handle,
                                svc->start_handle, svc->end_handle,
                                on_chr_disc, NULL);
    }
    return 0;
}

/* ── GAP event handler ───────────────────────────────────────────── */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_EXT_DISC: {
        const struct ble_gap_ext_disc_desc *d = &event->ext_disc;

        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, d->data, d->length_data))
            break;

        /* Look for our service UUID in the advertisement */
        bool match = false;
        for (int i = 0; i < fields.num_uuids16; i++) {
            if (ble_uuid_u16(&fields.uuids16[i].u) == RC_SERVICE_UUID) {
                match = true;
                break;
            }
        }
        if (!match) break;

        ESP_LOGI(TAG, "Found RC-Car — connecting");
        ble_gap_disc_cancel();

        struct ble_gap_conn_params cp = {
            .scan_itvl          = 0x40,
            .scan_window        = 0x40,
            .itvl_min           = 24,   /* 30 ms */
            .itvl_max           = 40,   /* 50 ms */
            .latency            = 0,
            .supervision_timeout = 400, /* 4 s   */
            .min_ce_len         = 0,
            .max_ce_len         = 0,
        };

        int rc = ble_gap_ext_connect(own_addr_type,
                                     &d->addr,
                                     10000,  /* 10 s timeout */
                                     BLE_GAP_LE_PHY_CODED_MASK,
                                     NULL, NULL, &cp,
                                     gap_event_cb, NULL);
        if (rc) {
            ESP_LOGE(TAG, "ext_connect: %d", rc);
            start_scan();
        }
        break;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            connected   = true;
            chr_discovered = false;
            update_connection_led();
            ESP_LOGI(TAG, "Connected — discovering services");

            ble_gap_set_prefered_le_phy(conn_handle,
                BLE_GAP_LE_PHY_CODED_MASK,
                BLE_GAP_LE_PHY_CODED_MASK,
                BLE_GAP_LE_PHY_CODED_ANY);

            ble_uuid16_t uuid = BLE_UUID16_INIT(RC_SERVICE_UUID);
            ble_gattc_disc_svc_by_uuid(conn_handle, &uuid.u,
                                       on_svc_disc, NULL);
        } else {
            ESP_LOGW(TAG, "Connect failed: %d", event->connect.status);
            connected = false;
            update_connection_led();
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        connected      = false;
        chr_discovered = false;
        conn_handle    = BLE_HS_CONN_HANDLE_NONE;
        update_connection_led();
        start_scan();
        break;

    default:
        break;
    }
    return 0;
}

/* ── NimBLE host callbacks ───────────────────────────────────────── */
static void on_sync(void)
{
    ble_hs_id_infer_auto(0, &own_addr_type);
    start_scan();
}

static void on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ── Control-loop task ───────────────────────────────────────────── */
static void control_task(void *param)
{
    bool sw_prev = true;   /* idle = high (pull-up) */

    for (;;) {
        /* SW button — cycle speed mode on press (edge-triggered) */
        bool sw = gpio_get_level(JOY_SW_PIN) == 0;
        if (sw && !sw_prev) {
            speed_mode_idx = (speed_mode_idx + 1) % SPEED_MODE_COUNT;
            current_speed  = speed_modes[speed_mode_idx];
            update_speed_leds();
            ESP_LOGI(TAG, "Speed mode %d (%u%%)", speed_mode_idx, current_speed);
        }
        sw_prev = sw;

        /* Read raw ADC values for debugging */
        int raw_x = 0, raw_y = 0;
        adc_oneshot_read(adc_handle, JOY_VRX_CHAN, &raw_x);
        adc_oneshot_read(adc_handle, JOY_VRY_CHAN, &raw_y);
        ESP_LOGI(TAG, "[HW-504 RAW] VRx=%4d  VRy=%4d  SW=%d",
                 raw_x, raw_y, gpio_get_level(JOY_SW_PIN));

        /* Read joystick axes */
        int x = joy_axis(JOY_VRX_CHAN);   /* left/right */
        int y = joy_axis(JOY_VRY_CHAN);   /* forward/backward */

        /* Clamp to -100…+100 */
        if (x >  100) x =  100;
        if (x < -100) x = -100;
        if (y >  100) y =  100;
        if (y < -100) y = -100;

        rc_command_t cmd = { .x = 0, .y = 0, .speed = 0 };

        if (x != 0 || y != 0) {
            cmd.x     = (int8_t)x;
            cmd.y     = (int8_t)y;
            cmd.speed = current_speed;
        }

        /* Send command over BLE */
        if (connected && chr_discovered) {
            ESP_LOGI(TAG, "x=%+4d y=%+4d spd=%3u", cmd.x, cmd.y, cmd.speed);
            ble_gattc_write_no_rsp_flat(conn_handle, cmd_chr_handle,
                                        &cmd, sizeof(cmd));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ── Entry point ─────────────────────────────────────────────────── */
void remote_main(void)
{
    ESP_LOGI(TAG, "RC Remote starting  (speed=%u%%)", current_speed);

    /* NVS – required by NimBLE */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* GPIO / ADC */
    joystick_init();
    joystick_calibrate();
    gpio_init_leds();
    update_speed_leds();
    update_connection_led();

    /* NimBLE */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: 0x%x", ret);
        return;
    }

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    nimble_port_freertos_init(host_task);

    /* Start button-polling / command-sending loop */
    xTaskCreate(control_task, "ctrl", 4096, NULL, 5, NULL);
}

#endif /* BUILD_REMOTE */
