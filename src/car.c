#ifdef BUILD_CAR

#include "car.h"
#include "ble_common.h"
#include "camera_stream.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char *TAG = "rc_car";

#define MOTOR_LOG_ENABLE  0

#define FRONT_LEFT_BACKWARDS  GPIO_NUM_40 // left-h2-in1  // red
#define FRONT_LEFT_FORWARDS   GPIO_NUM_41 // left-h2-in2  // blue
#define REAR_LEFT_BACKWARDS   GPIO_NUM_48 // left-h2-in3  // purple
#define REAR_LEFT_FORWARDS    GPIO_NUM_47 // left-h2-in4  // brown
#define REAR_RIGHT_FORWARDS   GPIO_NUM_43 // right-h2-in2 // blue-black
#define REAR_RIGHT_BACKWARDS  GPIO_NUM_39 // right-h2-in1 // gray
#define FRONT_RIGHT_FORWARDS  GPIO_NUM_38 // right-h2-in3 // brown-black
#define FRONT_RIGHT_BACKWARDS GPIO_NUM_21 // right-h2-in4 // green

#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_RES        LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ    1000
#define MAX_DUTY        ((1 << 10) - 1)

#define SERVO_UD_PIN    GPIO_NUM_15         /* up-down servo   */
#define SERVO_LR_PIN    GPIO_NUM_42         /* left-right servo */
#define SERVO_MIN_US     500
#define SERVO_MID_US    1500
#define SERVO_MAX_US    2500

typedef struct { gpio_num_t pin1; gpio_num_t pin2; ledc_channel_t ch; } motor_ch_t;

enum {
    M_LEFT_FWD,
    M_LEFT_BWD,
    M_RIGHT_FWD,
    M_RIGHT_BWD,
    MOTOR_COUNT
};

static const motor_ch_t motors[MOTOR_COUNT] = {
    [M_LEFT_FWD]  = { FRONT_LEFT_FORWARDS,   REAR_LEFT_FORWARDS,    LEDC_CHANNEL_1 },
    [M_LEFT_BWD]  = { FRONT_LEFT_BACKWARDS,  REAR_LEFT_BACKWARDS,   LEDC_CHANNEL_2 },
    [M_RIGHT_FWD] = { FRONT_RIGHT_FORWARDS,  REAR_RIGHT_FORWARDS,   LEDC_CHANNEL_3 },
    [M_RIGHT_BWD] = { FRONT_RIGHT_BACKWARDS, REAR_RIGHT_BACKWARDS,  LEDC_CHANNEL_4 },
};

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
            .gpio_num   = motors[i].pin1,
            .duty       = 0,
            .hpoint     = 0,
        };
        ledc_channel_config(&ch);
        ch.gpio_num = motors[i].pin2;
        ledc_channel_config(&ch);
    }
}

static void motors_all_stop(void)
{
    for (int i = 0; i < MOTOR_COUNT; i++)
        set_duty(i, 0);
}

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

    set_duty(M_LEFT_FWD,  left  > 0 ? l_pwm : 0);
    set_duty(M_LEFT_BWD,  left  < 0 ? l_pwm : 0);
    set_duty(M_RIGHT_FWD, right > 0 ? r_pwm : 0);
    set_duty(M_RIGHT_BWD, right < 0 ? r_pwm : 0);
}

static mcpwm_cmpr_handle_t servo_ud_cmp;
static mcpwm_cmpr_handle_t servo_lr_cmp;

#define SERVO_RATE       0.6f
static volatile int8_t servo_rate_x = 0;
static volatile int8_t servo_rate_y = 0;
static float servo_pos_x = 0.0f;
static float servo_pos_y = 0.0f;

static inline uint32_t servo_angle_to_us(int8_t val)
{
    return (uint32_t)(SERVO_MID_US + (int)val * (SERVO_MAX_US - SERVO_MIN_US) / 200);
}

static inline uint32_t servo_float_to_us(float val)
{
    return (uint32_t)(SERVO_MID_US + val * (SERVO_MAX_US - SERVO_MIN_US) / 200.0f);
}

static void servos_init(void)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_cfg = {
        .group_id      = 0,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,
        .period_ticks  = 20000,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
    };
    mcpwm_new_timer(&timer_cfg, &timer);

    mcpwm_oper_handle_t oper_ud = NULL;
    mcpwm_operator_config_t oper_cfg = { .group_id = 0 };
    mcpwm_new_operator(&oper_cfg, &oper_ud);
    mcpwm_operator_connect_timer(oper_ud, timer);

    mcpwm_comparator_config_t cmp_cfg = { .flags.update_cmp_on_tez = true };
    mcpwm_new_comparator(oper_ud, &cmp_cfg, &servo_ud_cmp);

    mcpwm_gen_handle_t gen_ud = NULL;
    mcpwm_generator_config_t gen_cfg_ud = { .gen_gpio_num = SERVO_UD_PIN };
    mcpwm_new_generator(oper_ud, &gen_cfg_ud, &gen_ud);

    mcpwm_generator_set_action_on_timer_event(gen_ud,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_ud,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, servo_ud_cmp, MCPWM_GEN_ACTION_LOW));

    mcpwm_oper_handle_t oper_lr = NULL;
    mcpwm_new_operator(&oper_cfg, &oper_lr);
    mcpwm_operator_connect_timer(oper_lr, timer);

    mcpwm_new_comparator(oper_lr, &cmp_cfg, &servo_lr_cmp);

    mcpwm_gen_handle_t gen_lr = NULL;
    mcpwm_generator_config_t gen_cfg_lr = { .gen_gpio_num = SERVO_LR_PIN };
    mcpwm_new_generator(oper_lr, &gen_cfg_lr, &gen_lr);

    mcpwm_generator_set_action_on_timer_event(gen_lr,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_lr,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, servo_lr_cmp, MCPWM_GEN_ACTION_LOW));

    mcpwm_comparator_set_compare_value(servo_ud_cmp, SERVO_MID_US);
    mcpwm_comparator_set_compare_value(servo_lr_cmp, SERVO_MID_US);

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    ESP_LOGI(TAG, "Servos initialised (UD=GPIO%d, LR=GPIO%d)",
             SERVO_UD_PIN, SERVO_LR_PIN);
}

static void servos_set_rate(const rc_command_t *cmd)
{
    servo_rate_x = cmd->pan_x;
    servo_rate_y = cmd->pan_y;
}

static void servo_task(void *param)
{
    for (;;) {
        int8_t rx = servo_rate_x;
        int8_t ry = servo_rate_y;

        servo_pos_x += (float)rx * SERVO_RATE / 100.0f;
        servo_pos_y += (float)ry * SERVO_RATE / 100.0f;
        if (servo_pos_x >  100.0f) servo_pos_x =  100.0f;
        if (servo_pos_x < -100.0f) servo_pos_x = -100.0f;
        if (servo_pos_y >  100.0f) servo_pos_y =  100.0f;
        if (servo_pos_y < -100.0f) servo_pos_y = -100.0f;

        mcpwm_comparator_set_compare_value(servo_lr_cmp, servo_float_to_us(servo_pos_x));
        mcpwm_comparator_set_compare_value(servo_ud_cmp, servo_float_to_us(servo_pos_y));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static uint16_t conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static uint16_t cmd_chr_val_handle;
static uint8_t  own_addr_type;

static int  gap_event_cb(struct ble_gap_event *event, void *arg);
static void start_advertising(void);

static int command_chr_access(uint16_t conn_handle, uint16_t attr_handle,
                              struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        rc_command_t cmd;
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len == sizeof(rc_command_t)) {
            os_mbuf_copydata(ctxt->om, 0, sizeof(cmd), &cmd);
            motors_drive(&cmd);
            servos_set_rate(&cmd);
#if MOTOR_LOG_ENABLE
            ESP_LOGI(TAG, "cmd x=%d y=%d spd=%u pan_x=%d pan_y=%d",
                     cmd.x, cmd.y, cmd.speed, cmd.pan_x, cmd.pan_y);
#endif
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

    struct os_mbuf *data = os_msys_get_pkthdr(BLE_HCI_MAX_EXT_ADV_DATA_LEN, 0);
    if (!data) { ESP_LOGE(TAG, "mbuf alloc failed"); return; }

    uint8_t fl[] = { 0x02, BLE_HS_ADV_TYPE_FLAGS,
                     BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP };
    os_mbuf_append(data, fl, sizeof(fl));

    uint8_t nlen = strlen(RC_DEVICE_NAME);
    uint8_t nhdr[] = { (uint8_t)(nlen + 1), BLE_HS_ADV_TYPE_COMP_NAME };
    os_mbuf_append(data, nhdr, sizeof(nhdr));
    os_mbuf_append(data, RC_DEVICE_NAME, nlen);

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

static int gap_event_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "Connected  handle=%d", conn_handle);

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
        servo_rate_x = 0;
        servo_rate_y = 0;
        start_advertising();
        break;

    default:
        break;
    }
    return 0;
}

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

void car_main(void)
{
    ESP_LOGI(TAG, "RC Car starting");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    ledc_init_motors();
    motors_all_stop();

    servos_init();
    xTaskCreate(servo_task, "servo", 2048, NULL, 5, NULL);

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

    camera_stream_init();
}

#endif /* BUILD_CAR */
