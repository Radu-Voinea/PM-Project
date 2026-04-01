#ifdef BUILD_CAR

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "ble_common.h"
#include "camera_stream.h"
#include "car.h"

static const char *TAG = "car";

/* ═══════════════════════════════════════════════════════════════════ *
 *  MOTOR CONTROL  (4-wheel differential drive via LEDC PWM)          *
 * ═══════════════════════════════════════════════════════════════════ *
 *  LEDC_TIMER_0  1 kHz  10-bit                                       *
 *  (Camera XCLK uses TIMER_1/CH_0 → no conflict)                    *
 *                                                                     *
 *  CH1  left-fwd  → GPIO40 (primary), GPIO41 (via GPIO matrix)       *
 *  CH2  left-bck  → GPIO48 (primary), GPIO47 (via GPIO matrix)       *
 *  CH3  right-fwd → GPIO38 (primary), GPIO21 (via GPIO matrix)       *
 *  CH4  right-bck → GPIO43 (primary), GPIO39 (via GPIO matrix)       */
#define MOTOR_LEDC_TIMER   LEDC_TIMER_0
#define MOTOR_LEDC_MODE    LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_RES     LEDC_TIMER_10_BIT
#define MOTOR_LEDC_FREQ    1000
#define MOTOR_MAX_DUTY     1023

typedef struct { ledc_channel_t ch; int gpio_a; int gpio_b; } motor_pin_t;

static const motor_pin_t k_motors[4] = {
    { LEDC_CHANNEL_1, 40, 41 },
    { LEDC_CHANNEL_2, 48, 47 },
    { LEDC_CHANNEL_3, 38, 21 },
    { LEDC_CHANNEL_4, 43, 39 },
};

/* ESP32-S3 LOW_SPEED LEDC signal indices (soc/gpio_sig_map.h) */
static const int k_ledc_sig[] = {
    LEDC_LS_SIG_OUT0_IDX, LEDC_LS_SIG_OUT1_IDX,
    LEDC_LS_SIG_OUT2_IDX, LEDC_LS_SIG_OUT3_IDX,
    LEDC_LS_SIG_OUT4_IDX, LEDC_LS_SIG_OUT5_IDX,
    LEDC_LS_SIG_OUT6_IDX, LEDC_LS_SIG_OUT7_IDX,
};

static void motors_init(void)
{
    ledc_timer_config_t tmr = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .duty_resolution = MOTOR_LEDC_RES,
        .timer_num       = MOTOR_LEDC_TIMER,
        .freq_hz         = MOTOR_LEDC_FREQ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tmr));

    ledc_channel_config_t ch_cfg = {
        .speed_mode = MOTOR_LEDC_MODE,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 0,
        .hpoint     = 0,
    };

    for (int i = 0; i < 4; i++) {
        ch_cfg.channel  = k_motors[i].ch;
        ch_cfg.gpio_num = k_motors[i].gpio_a;
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

        /* Route the same LEDC signal to the second GPIO via the matrix */
        gpio_set_direction(k_motors[i].gpio_b, GPIO_MODE_OUTPUT);
        esp_rom_gpio_connect_out_signal(k_motors[i].gpio_b,
                                        k_ledc_sig[k_motors[i].ch],
                                        false, false);
    }
    ESP_LOGI(TAG, "Motors initialised");
}

static inline int clamp_i(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static void motor_set_pct(ledc_channel_t ch, int pct)
{
    ledc_set_duty(MOTOR_LEDC_MODE, ch, (uint32_t)(pct * MOTOR_MAX_DUTY / 100));
    ledc_update_duty(MOTOR_LEDC_MODE, ch);
}

/* Arcade-mix:  left = y+x,  right = y-x  (-100…+100) */
static void drive_motors(int8_t x, int8_t y, uint8_t speed_pct)
{
    int left  = clamp_i((int)y + (int)x, -100, 100) * speed_pct / 100;
    int right = clamp_i((int)y - (int)x, -100, 100) * speed_pct / 100;

    motor_set_pct(LEDC_CHANNEL_1, left  > 0 ?  left  : 0);
    motor_set_pct(LEDC_CHANNEL_2, left  < 0 ? -left  : 0);
    motor_set_pct(LEDC_CHANNEL_3, right > 0 ?  right : 0);
    motor_set_pct(LEDC_CHANNEL_4, right < 0 ? -right : 0);
}

/* ═══════════════════════════════════════════════════════════════════ *
 *  SERVO CONTROL  (MCPWM  50 Hz  1 MHz tick)                         *
 *  SERVO_UD_PIN = GPIO15   SERVO_LR_PIN = GPIO42                     *
 *  Pulse range 500-2500 µs,  neutral 1500 µs                         *
 * ═══════════════════════════════════════════════════════════════════ */
#define SERVO_UD_PIN  15
#define SERVO_LR_PIN  42
#define SERVO_MIN_US  500
#define SERVO_MAX_US  2500
#define SERVO_NEU_US  1500

static volatile float s_servo_ud = SERVO_NEU_US;
static volatile float s_servo_lr = SERVO_NEU_US;
static volatile float s_target_ud = SERVO_NEU_US;
static volatile float s_target_lr = SERVO_NEU_US;

static mcpwm_cmpr_handle_t s_cmpr_ud = NULL;
static mcpwm_cmpr_handle_t s_cmpr_lr = NULL;

static void servos_init(void)
{
    mcpwm_timer_handle_t timer;
    mcpwm_timer_config_t tmr_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks  = 20000,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&tmr_cfg, &timer));

    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };

    /* ── U/D ── */
    mcpwm_oper_handle_t oper_ud;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper_ud));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_ud, timer));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_ud, &cmp_cfg, &s_cmpr_ud));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmpr_ud, SERVO_NEU_US));

    mcpwm_gen_handle_t gen_ud;
    mcpwm_generator_config_t gen_cfg = { .gen_gpio_num = SERVO_UD_PIN };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_ud, &gen_cfg, &gen_ud));
    mcpwm_generator_set_action_on_timer_event(gen_ud,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_ud,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_cmpr_ud, MCPWM_GEN_ACTION_LOW));

    /* ── L/R ── */
    mcpwm_oper_handle_t oper_lr;
    ESP_ERROR_CHECK(mcpwm_new_operator(&oper_cfg, &oper_lr));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper_lr, timer));
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper_lr, &cmp_cfg, &s_cmpr_lr));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(s_cmpr_lr, SERVO_NEU_US));

    mcpwm_gen_handle_t gen_lr;
    gen_cfg.gen_gpio_num = SERVO_LR_PIN;
    ESP_ERROR_CHECK(mcpwm_new_generator(oper_lr, &gen_cfg, &gen_lr));
    mcpwm_generator_set_action_on_timer_event(gen_lr,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_lr,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       s_cmpr_lr, MCPWM_GEN_ACTION_LOW));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    ESP_LOGI(TAG, "Servos initialised");
}

/* Smooth servo update task (10 ms tick) */
static void servo_task(void *arg)
{
    const float RATE = 0.6f;
    while (true) {
        s_servo_ud += (s_target_ud - s_servo_ud) * RATE;
        s_servo_lr += (s_target_lr - s_servo_lr) * RATE;

        if (s_servo_ud < SERVO_MIN_US) s_servo_ud = SERVO_MIN_US;
        if (s_servo_ud > SERVO_MAX_US) s_servo_ud = SERVO_MAX_US;
        if (s_servo_lr < SERVO_MIN_US) s_servo_lr = SERVO_MIN_US;
        if (s_servo_lr > SERVO_MAX_US) s_servo_lr = SERVO_MAX_US;

        mcpwm_comparator_set_compare_value(s_cmpr_ud, (uint32_t)s_servo_ud);
        mcpwm_comparator_set_compare_value(s_cmpr_lr, (uint32_t)s_servo_lr);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void update_servo_targets(int8_t pan_x, int8_t pan_y)
{
    /* ±100 → ±1000 µs from neutral */
    s_target_lr = (float)SERVO_NEU_US + (float)pan_x * 10.0f;
    s_target_ud = (float)SERVO_NEU_US + (float)pan_y * 10.0f;
}

/* ═══════════════════════════════════════════════════════════════════ *
 *  BLE PERIPHERAL  (NimBLE  Coded PHY  GATT server)                  *
 * ═══════════════════════════════════════════════════════════════════ */
static uint8_t s_own_addr_type;

static int cmd_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                          struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len != sizeof(rc_command_t)) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

    rc_command_t cmd;
    ble_hs_mbuf_to_flat(ctxt->om, &cmd, sizeof(cmd), NULL);

    drive_motors(cmd.x, cmd.y, cmd.speed);
    update_servo_targets(cmd.pan_x, cmd.pan_y);
    return 0;
}

static const struct ble_gatt_svc_def s_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(RC_SERVICE_UUID),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid      = BLE_UUID16_DECLARE(RC_COMMAND_CHR_UUID),
                .access_cb = cmd_chr_access,
                .flags     = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            { 0 }
        },
    },
    { 0 }
};

static void car_ble_start_adv(void);

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ESP_LOGI(TAG, "BLE remote connected (conn=%d)",
                     event->connect.conn_handle);
        } else {
            car_ble_start_adv();
        }
        break;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "BLE disconnected (reason=%d)", event->disconnect.reason);
        drive_motors(0, 0, 0);
        s_target_ud = SERVO_NEU_US;
        s_target_lr = SERVO_NEU_US;
        car_ble_start_adv();
        break;

    default:
        break;
    }
    return 0;
}

static void car_ble_start_adv(void)
{
    struct ble_gap_ext_adv_params params = {
        .connectable   = 1,
        .scannable     = 0,
        .own_addr_type = s_own_addr_type,
        .primary_phy   = BLE_HCI_LE_PHY_CODED,
        .secondary_phy = BLE_HCI_LE_PHY_CODED,
        .sid           = 0,
        .itvl_min      = BLE_GAP_ADV_ITVL_MS(100),
        .itvl_max      = BLE_GAP_ADV_ITVL_MS(200),
    };

    ble_gap_ext_adv_stop(0);
    int rc = ble_gap_ext_adv_configure(0, &params, NULL, gap_event_cb, NULL);
    if (rc != 0) { ESP_LOGE(TAG, "adv_configure: %d", rc); return; }

    struct ble_hs_adv_fields fields = {0};
    fields.flags            = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.name             = (const uint8_t *)RC_DEVICE_NAME;
    fields.name_len         = sizeof(RC_DEVICE_NAME) - 1;
    fields.name_is_complete = 1;

    uint8_t buf[BLE_HS_ADV_MAX_SZ];
    uint8_t buf_sz = sizeof(buf);
    rc = ble_hs_adv_set_fields(&fields, buf, &buf_sz, sizeof(buf));
    if (rc != 0) { ESP_LOGE(TAG, "adv_set_fields: %d", rc); return; }

    struct os_mbuf *data = os_msys_get_pkthdr(buf_sz, 0);
    if (!data) { ESP_LOGE(TAG, "os_msys alloc failed"); return; }
    os_mbuf_append(data, buf, buf_sz);
    ble_gap_ext_adv_set_data(0, data);

    rc = ble_gap_ext_adv_start(0, 0, 0);
    if (rc == 0) ESP_LOGI(TAG, "Advertising on Coded PHY");
    else         ESP_LOGE(TAG, "adv_start: %d", rc);
}

static void ble_on_reset(int reason)
{
    ESP_LOGW(TAG, "BLE reset (reason=%d)", reason);
}

static void ble_on_sync(void)
{
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    car_ble_start_adv();
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
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(s_gatt_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(s_gatt_svcs));
    ble_svc_gap_device_name_set(RC_DEVICE_NAME);

    nimble_port_freertos_init(nimble_host_task);
    ESP_LOGI(TAG, "BLE initialised");
}

/* ═══════════════════════════════════════════════════════════════════ *
 *  ENTRY POINT                                                        *
 * ═══════════════════════════════════════════════════════════════════ */
void car_main(void)
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

    motors_init();
    servos_init();
    xTaskCreatePinnedToCore(servo_task, "servo", 2048, NULL, 4, NULL, 1);

    /* camera_stream_init() also starts the WiFi AP */
    camera_stream_init();
    ble_init();
    camera_stream_start();
}

#endif /* BUILD_CAR */
