#ifdef BUILD_CAR

#include "car.h"
#include "ble_common.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"

/* NimBLE */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "rc_car";

/* ── Motor GPIO pins (directly from the original layout) ─────────── */
#define FRONT_LEFT_FORWARDS   GPIO_NUM_46 // left-h2-in2  // blue
#define FRONT_LEFT_BACKWARDS  GPIO_NUM_47 // left-h2-in1  // red
#define FRONT_RIGHT_FORWARDS  GPIO_NUM_35 // right-h2-in3 // brown
#define FRONT_RIGHT_BACKWARDS GPIO_NUM_21 // right-h2-in4 // green
#define REAR_RIGHT_BACKWARDS  GPIO_NUM_37 // right-h2-in1 // gray
#define REAR_RIGHT_FORWARDS   GPIO_NUM_36 // right-h2-in2 // blue
#define REAR_LEFT_FORWARDS    GPIO_NUM_38 // left-h2-in4  // brown
#define REAR_LEFT_BACKWARDS   GPIO_NUM_45 // left-h2-in3  // purple

/* ── LEDC / PWM ──────────────────────────────────────────────────── */
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_RES        LEDC_TIMER_10_BIT   /* 0 – 1023 */
#define LEDC_FREQ_HZ    1000
#define MAX_DUTY        ((1 << 10) - 1)     /* 1023 */

typedef struct { gpio_num_t pin; ledc_channel_t ch; } motor_ch_t;

enum {
    M_FL_FWD, M_FL_BWD,
    M_FR_FWD, M_FR_BWD,
    M_BL_FWD, M_BL_BWD,
    M_BR_FWD, M_BR_BWD,
    MOTOR_COUNT
};

static const motor_ch_t motors[MOTOR_COUNT] = {
    [M_FL_FWD] = { FRONT_LEFT_FORWARDS,    LEDC_CHANNEL_0 },
    [M_FL_BWD] = { FRONT_LEFT_BACKWARDS,  LEDC_CHANNEL_1 },
    [M_FR_FWD] = { FRONT_RIGHT_FORWARDS,   LEDC_CHANNEL_2 },
    [M_FR_BWD] = { FRONT_RIGHT_BACKWARDS, LEDC_CHANNEL_3 },
    [M_BL_FWD] = { REAR_LEFT_FORWARDS,  LEDC_CHANNEL_4 },
    [M_BL_BWD] = { REAR_LEFT_BACKWARDS,   LEDC_CHANNEL_5 },
    [M_BR_FWD] = { REAR_RIGHT_FORWARDS, LEDC_CHANNEL_6 },
    [M_BR_BWD] = { REAR_RIGHT_BACKWARDS,  LEDC_CHANNEL_7 },
};

/* ── Motor helpers ───────────────────────────────────────────────── */
static void set_duty(int idx, uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, motors[idx].ch, duty);
    ledc_update_duty(LEDC_MODE, motors[idx].ch);
}

static void ledc_init_motors(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ch = {
            .speed_mode = LEDC_MODE,
            .channel    = motors[i].ch,
            .timer_sel  = LEDC_TIMER,
            .gpio_num   = motors[i].pin,
            .duty       = 0,
            .hpoint     = 0,
        };
        ledc_channel_config(&ch);
    }
}

static void motors_all_stop(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_duty(i, 0);
}

/*  Arcade-style differential drive.
 *  x : -100…+100   (left / right)
 *  y : -100…+100   (backward / forward)
 *  speed : 0…100   (power %)
 */
static void motors_drive(const rc_command_t *cmd)
{
    if (cmd->speed == 0) { motors_all_stop(); return; }

    int left  = (int)cmd->y + (int)cmd->x;
    int right = (int)cmd->y - (int)cmd->x;

    if (left  >  100) left  =  100;
    if (left  < -100) left  = -100;
    if (right >  100) right =  100;
    if (right < -100) right = -100;

    uint32_t l_pwm = (uint32_t)(abs(left)  * cmd->speed * MAX_DUTY / 10000);
    uint32_t r_pwm = (uint32_t)(abs(right) * cmd->speed * MAX_DUTY / 10000);

    /* left side */
    set_duty(M_FL_FWD, left > 0 ? l_pwm : 0);
    set_duty(M_FL_BWD, left < 0 ? l_pwm : 0);
    set_duty(M_BL_FWD, left > 0 ? l_pwm : 0);
    set_duty(M_BL_BWD, left < 0 ? l_pwm : 0);

    /* right side */
    set_duty(M_FR_FWD, right > 0 ? r_pwm : 0);
    set_duty(M_FR_BWD, right < 0 ? r_pwm : 0);
    set_duty(M_BR_FWD, right > 0 ? r_pwm : 0);
    set_duty(M_BR_BWD, right < 0 ? r_pwm : 0);
}

/* ── BLE – GATT server (peripheral) ─────────────────────────────── */
static uint16_t conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t cmd_chr_val_handle;
static uint8_t  own_addr_type;

static int  gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_advertising(void);

/* Write-handler for the command characteristic */
static int command_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        rc_command_t cmd;
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == sizeof(rc_command_t)) {
            os_mbuf_copydata(ctxt->om, 0, sizeof(cmd), &cmd);
            motors_drive(&cmd);
            ESP_LOGI(TAG, "cmd x=%d y=%d spd=%u", cmd.x, cmd.y, cmd.speed);
        }
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(RC_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid       = BLE_UUID16_DECLARE(RC_COMMAND_CHR_UUID),
                .access_cb  = command_chr_access,
                .val_handle = &cmd_chr_val_handle,
                .flags      = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 },
        },
    },
    { 0 },
};

/* ── Extended advertising on Coded PHY ───────────────────────────── */
static void start_advertising(void)
{
    struct ble_gap_ext_adv_params params = {0};
    params.connectable   = 1;
    params.scannable     = 0;
    params.primary_phy   = BLE_HCI_LE_PHY_CODED;
    params.secondary_phy = BLE_HCI_LE_PHY_CODED;
    params.sid           = 0;

    int rc = ble_gap_ext_adv_configure(0, &params, NULL, gap_event_cb, NULL);
    if (rc) { ESP_LOGE(TAG, "ext_adv_configure: %d", rc); return; }

    /* Build AD payload manually into an mbuf */
    struct os_mbuf *data = os_msys_get_pkthdr(BLE_HCI_MAX_EXT_ADV_DATA_LEN, 0);
    if (!data) { ESP_LOGE(TAG, "mbuf alloc failed"); return; }

    /* Flags */
    uint8_t fl[] = { 0x02, BLE_HS_ADV_TYPE_FLAGS,
                     BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP };
    os_mbuf_append(data, fl, sizeof(fl));

    /* Complete Local Name */
    uint8_t nlen = strlen(RC_DEVICE_NAME);
    uint8_t nhdr[] = { (uint8_t)(nlen + 1), BLE_HS_ADV_TYPE_COMP_NAME };
    os_mbuf_append(data, nhdr, sizeof(nhdr));
    os_mbuf_append(data, RC_DEVICE_NAME, nlen);

    /* 16-bit Service UUID list */
    uint16_t svc = RC_SERVICE_UUID;
    uint8_t uhdr[] = { 0x03, BLE_HS_ADV_TYPE_COMP_UUIDS16 };
    os_mbuf_append(data, uhdr, sizeof(uhdr));
    os_mbuf_append(data, &svc, sizeof(svc));

    rc = ble_gap_ext_adv_set_data(0, data);
    if (rc) { ESP_LOGE(TAG, "ext_adv_set_data: %d", rc); return; }

    rc = ble_gap_ext_adv_start(0, 0, 0);
    if (rc) { ESP_LOGE(TAG, "ext_adv_start: %d", rc); return; }

    ESP_LOGI(TAG, "Advertising started (Coded PHY)");
}

/* ── GAP event handler ───────────────────────────────────────────── */
static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected  handle=%d", conn_handle);

            /* Request Coded PHY for this connection */
            ble_gap_set_prefered_le_phy(conn_handle,
                BLE_GAP_LE_PHY_CODED_MASK,
                BLE_GAP_LE_PHY_CODED_MASK,
                BLE_GAP_LE_PHY_CODED_ANY);
        } else {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            start_advertising();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected — motors stopped");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        motors_all_stop();
        start_advertising();
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
    start_advertising();
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

/* ── Entry point ─────────────────────────────────────────────────── */
void car_main(void)
{
    ESP_LOGI(TAG, "RC Car starting");

    /* NVS – required by NimBLE */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* Motors */
    ledc_init_motors();
    motors_all_stop();

    /* NimBLE */
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init: 0x%x", ret);
        return;
    }

    ble_hs_cfg.sync_cb    = on_sync;
    ble_hs_cfg.reset_cb   = on_reset;
    ble_hs_cfg.sm_bonding = 0;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(RC_DEVICE_NAME);

    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    nimble_port_freertos_init(host_task);
}

#endif /* BUILD_CAR */
