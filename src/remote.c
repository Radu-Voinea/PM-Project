#ifdef BUILD_REMOTE

#include "remote.h"
#include "ble_common.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

/* NimBLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "rc_remote";

/* ── Button GPIOs ────────────────────────────────────────────────── */
#define BUTTON_FORWARD      GPIO_NUM_1
#define BUTTON_BACKWARD     GPIO_NUM_2
#define BUTTON_LEFT         GPIO_NUM_3
#define BUTTON_RIGHT        GPIO_NUM_4
#define BUTTON_SPEED_UP     GPIO_NUM_13
#define BUTTON_SPEED_DOWN   GPIO_NUM_14

#define SPEED_DEFAULT   30
#define SPEED_MIN       10
#define SPEED_MAX       100
#define SPEED_STEP      10

static uint8_t current_speed = SPEED_DEFAULT;

/* ── BLE state ───────────────────────────────────────────────────── */
static uint16_t conn_handle    = BLE_HS_CONN_HANDLE_NONE;
static uint16_t cmd_chr_handle = 0;
static volatile bool connected      = false;
static volatile bool chr_discovered  = false;
static uint8_t  own_addr_type;

static int  gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_scan(void);

/* ── GPIO ────────────────────────────────────────────────────────── */
static void gpio_init_buttons(void)
{
    const gpio_num_t pins[] = {
        BUTTON_FORWARD, BUTTON_BACKWARD,
        BUTTON_LEFT,    BUTTON_RIGHT,
        BUTTON_SPEED_UP, BUTTON_SPEED_DOWN,
    };
    for (int i = 0; i < sizeof(pins) / sizeof(pins[0]); i++) {
        gpio_config_t cfg = {
            .pin_bit_mask  = 1ULL << pins[i],
            .mode          = GPIO_MODE_INPUT,
            .pull_up_en    = GPIO_PULLUP_ENABLE,
            .pull_down_en  = GPIO_PULLDOWN_DISABLE,
            .intr_type     = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
    }
}

static inline bool btn(gpio_num_t pin) { return gpio_get_level(pin) == 0; }

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
            start_scan();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected");
        connected      = false;
        chr_discovered = false;
        conn_handle    = BLE_HS_CONN_HANDLE_NONE;
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
    bool su_prev = false, sd_prev = false;

    for (;;) {
        /* Speed adjustment — edge-triggered */
        bool su = btn(BUTTON_SPEED_UP);
        bool sd = btn(BUTTON_SPEED_DOWN);

        if (su && !su_prev && current_speed < SPEED_MAX)
            current_speed += SPEED_STEP;
        if (sd && !sd_prev && current_speed > SPEED_MIN)
            current_speed -= SPEED_STEP;

        su_prev = su;
        sd_prev = sd;

        /* Direction from buttons */
        bool fwd  = btn(BUTTON_FORWARD);
        bool bwd  = btn(BUTTON_BACKWARD);
        bool left = btn(BUTTON_LEFT);
        bool rght = btn(BUTTON_RIGHT);

        rc_command_t cmd = { .x = 0, .y = 0, .speed = 0 };

        if (fwd || bwd || left || rght) {
            cmd.speed = current_speed;
            if (fwd)  cmd.y =  100;
            if (bwd)  cmd.y = -100;
            if (left)  cmd.x = -100;
            if (rght)  cmd.x =  100;
        }

        /* Send command over BLE */
        if (connected && chr_discovered) {
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

    /* Buttons */
    gpio_init_buttons();

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
