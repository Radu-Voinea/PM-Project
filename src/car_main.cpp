#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_camera.h>
#include <esp_wifi.h>
#include <NimBLEDevice.h>

#include "video_protocol.h"
#include "ble_protocol.h"

/* ESP-IDF drivers for motor PWM and servo PWM */
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"

/* ================================================================
 *  Pin Definitions
 * ================================================================ */

// ---- Camera (U2 → U24 OV2640) ----
#define CAM_PIN_PWDN    -1      // Tied to GND on PCB
#define CAM_PIN_RESET   -1      // Tied to GND on PCB
#define CAM_PIN_XCLK    -1      // Onboard oscillator
#define CAM_PIN_SIOD    6       // SDA
#define CAM_PIN_SIOC    7       // SCL
#define CAM_PIN_D0      5
#define CAM_PIN_D1      10
#define CAM_PIN_D2      4
#define CAM_PIN_D3      11
#define CAM_PIN_D4      3       // GPIO3 strapping (JTAG) — input-only after boot, fine for cam data
#define CAM_PIN_D5      12
#define CAM_PIN_D6      2
#define CAM_PIN_D7      13
#define CAM_PIN_VSYNC   8
#define CAM_PIN_HREF    9
#define CAM_PIN_PCLK    1       // DCLK

// ---- Motor H-bridge right (U2 → U18) ----
#define MOTOR_RR_BWD    39      // U18 IN1
#define MOTOR_RR_FWD    40      // U18 IN2
#define MOTOR_FR_FWD    41      // U18 IN3
#define MOTOR_FR_BWD    42      // U18 IN4

// ---- Motor H-bridge left (U2 → U19) ----
#define MOTOR_FL_BWD    47      // U19 IN1
#define MOTOR_FL_FWD    38      // U19 IN2
#define MOTOR_RL_BWD    18      // U19 IN3
#define MOTOR_RL_FWD    17      // U19 IN4

// ---- Servos (U2 → U20/U21) ----
#define SERVO_UD_PIN    15      // U20 SG90 up-down
#define SERVO_LR_PIN    16      // U21 SG90 left-right

// ---- Error LED (U2 → U22/U23) ----
#define LED_ERR_PIN     14      // U22 ERR_R → U23 ERR LED

/* ================================================================
 *  LEDC motor configuration
 * ================================================================ */
#define MOTOR_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER    LEDC_TIMER_0
#define MOTOR_LEDC_RES      LEDC_TIMER_10_BIT
#define MOTOR_LEDC_FREQ     1000
#define MOTOR_MAX_DUTY      ((1 << 10) - 1)     // 1023

// Camera uses a separate LEDC timer to avoid frequency conflict
#define CAM_LEDC_TIMER      LEDC_TIMER_1
#define CAM_LEDC_CHANNEL    LEDC_CHANNEL_0

/* ================================================================
 *  WiFi AP
 * ================================================================ */
static const char *AP_SSID     = "RC-CAR-CAM";
static const char *AP_PASSWORD = "rccar12345";
static const IPAddress REMOTE_IP(192, 168, 4, 2);

/* ================================================================
 *  Motor Control
 * ================================================================ */
enum { M_LEFT_FWD = 0, M_LEFT_BWD, M_RIGHT_FWD, M_RIGHT_BWD, MOTOR_COUNT };

struct MotorChannel {
    gpio_num_t     pin1;   // front wheel
    gpio_num_t     pin2;   // rear  wheel
    ledc_channel_t ch;
};

static const MotorChannel motors[MOTOR_COUNT] = {
    { (gpio_num_t)MOTOR_FL_FWD, (gpio_num_t)MOTOR_RL_FWD, LEDC_CHANNEL_1 },
    { (gpio_num_t)MOTOR_FL_BWD, (gpio_num_t)MOTOR_RL_BWD, LEDC_CHANNEL_2 },
    { (gpio_num_t)MOTOR_FR_FWD, (gpio_num_t)MOTOR_RR_FWD, LEDC_CHANNEL_3 },
    { (gpio_num_t)MOTOR_FR_BWD, (gpio_num_t)MOTOR_RR_BWD, LEDC_CHANNEL_4 },
};

static void setMotorDuty(int idx, uint32_t duty) {
    ledc_set_duty(MOTOR_LEDC_MODE, motors[idx].ch, duty);
    ledc_update_duty(MOTOR_LEDC_MODE, motors[idx].ch);
}

static void motorsStop() {
    for (int i = 0; i < MOTOR_COUNT; i++)
        setMotorDuty(i, 0);
}

static void motorsDrive(const ControlCommand *cmd) {
    if (cmd->speed == 0) { motorsStop(); return; }

    int left  = (int)cmd->y + (int)cmd->x;
    int right = (int)cmd->y - (int)cmd->x;
    left  = constrain(left,  -100, 100);
    right = constrain(right, -100, 100);

    uint32_t l_pwm = (uint32_t)(abs(left)  * cmd->speed * MOTOR_MAX_DUTY / 10000);
    uint32_t r_pwm = (uint32_t)(abs(right) * cmd->speed * MOTOR_MAX_DUTY / 10000);

    setMotorDuty(M_LEFT_FWD,  left  > 0 ? l_pwm : 0);
    setMotorDuty(M_LEFT_BWD,  left  < 0 ? l_pwm : 0);
    setMotorDuty(M_RIGHT_FWD, right > 0 ? r_pwm : 0);
    setMotorDuty(M_RIGHT_BWD, right < 0 ? r_pwm : 0);
}

static void initMotors() {
    ledc_timer_config_t tcfg = {};
    tcfg.speed_mode      = MOTOR_LEDC_MODE;
    tcfg.duty_resolution = MOTOR_LEDC_RES;
    tcfg.timer_num       = MOTOR_LEDC_TIMER;
    tcfg.freq_hz         = MOTOR_LEDC_FREQ;
    tcfg.clk_cfg         = LEDC_AUTO_CLK;
    ledc_timer_config(&tcfg);

    for (int i = 0; i < MOTOR_COUNT; i++) {
        ledc_channel_config_t ccfg = {};
        ccfg.gpio_num   = motors[i].pin1;
        ccfg.speed_mode = MOTOR_LEDC_MODE;
        ccfg.channel    = motors[i].ch;
        ccfg.intr_type  = LEDC_INTR_DISABLE;
        ccfg.timer_sel  = MOTOR_LEDC_TIMER;
        ccfg.duty       = 0;
        ccfg.hpoint     = 0;
        ledc_channel_config(&ccfg);
        // Route same LEDC channel to the paired rear-wheel pin
        ccfg.gpio_num = motors[i].pin2;
        ledc_channel_config(&ccfg);
    }
    Serial.println("Motors initialised");
}

/* ================================================================
 *  Servo Control  (MCPWM — separate peripheral from LEDC)
 * ================================================================ */
static mcpwm_cmpr_handle_t servo_ud_cmp = NULL;
static mcpwm_cmpr_handle_t servo_lr_cmp = NULL;

#define SERVO_RATE   0.6f
#define SERVO_MIN_US 500
#define SERVO_MID_US 1500
#define SERVO_MAX_US 2500

static volatile int8_t servo_rate_x = 0;   // set by BLE callback
static volatile int8_t servo_rate_y = 0;
static float servo_pos_x = 0.0f;
static float servo_pos_y = 0.0f;

static inline uint32_t servoFloatToUs(float pos) {
    return (uint32_t)(SERVO_MID_US + pos * (SERVO_MAX_US - SERVO_MIN_US) / 200.0f);
}

static void initServos() {
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t tcfg = {};
    tcfg.group_id      = 0;
    tcfg.clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tcfg.resolution_hz = 1000000;                       // 1 MHz → 1 us/tick
    tcfg.count_mode    = MCPWM_TIMER_COUNT_MODE_UP;
    tcfg.period_ticks  = 20000;                          // 50 Hz
    mcpwm_new_timer(&tcfg, &timer);

    mcpwm_operator_config_t opcfg = {};
    opcfg.group_id = 0;

    mcpwm_comparator_config_t cmpcfg = {};
    cmpcfg.flags.update_cmp_on_tez = true;

    // --- Up-Down servo (operator 0) ---
    mcpwm_oper_handle_t oper_ud = NULL;
    mcpwm_new_operator(&opcfg, &oper_ud);
    mcpwm_operator_connect_timer(oper_ud, timer);
    mcpwm_new_comparator(oper_ud, &cmpcfg, &servo_ud_cmp);

    mcpwm_gen_handle_t gen_ud = NULL;
    mcpwm_generator_config_t gcfg_ud = {};
    gcfg_ud.gen_gpio_num = SERVO_UD_PIN;
    mcpwm_new_generator(oper_ud, &gcfg_ud, &gen_ud);

    mcpwm_generator_set_action_on_timer_event(gen_ud,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_ud,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       servo_ud_cmp,
                                       MCPWM_GEN_ACTION_LOW));

    // --- Left-Right servo (operator 1) ---
    mcpwm_oper_handle_t oper_lr = NULL;
    mcpwm_new_operator(&opcfg, &oper_lr);
    mcpwm_operator_connect_timer(oper_lr, timer);
    mcpwm_new_comparator(oper_lr, &cmpcfg, &servo_lr_cmp);

    mcpwm_gen_handle_t gen_lr = NULL;
    mcpwm_generator_config_t gcfg_lr = {};
    gcfg_lr.gen_gpio_num = SERVO_LR_PIN;
    mcpwm_new_generator(oper_lr, &gcfg_lr, &gen_lr);

    mcpwm_generator_set_action_on_timer_event(gen_lr,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                     MCPWM_TIMER_EVENT_EMPTY,
                                     MCPWM_GEN_ACTION_HIGH));
    mcpwm_generator_set_action_on_compare_event(gen_lr,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                                       servo_lr_cmp,
                                       MCPWM_GEN_ACTION_LOW));

    mcpwm_comparator_set_compare_value(servo_ud_cmp, SERVO_MID_US);
    mcpwm_comparator_set_compare_value(servo_lr_cmp, SERVO_MID_US);

    mcpwm_timer_enable(timer);
    mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP);

    Serial.printf("Servos initialised (UD=GPIO%d, LR=GPIO%d)\n",
                  SERVO_UD_PIN, SERVO_LR_PIN);
}

// Rate-based servo task — integrates pan rate into position at 100 Hz
static void servoTask(void *) {
    for (;;) {
        int8_t rx = servo_rate_x;
        int8_t ry = servo_rate_y;

        servo_pos_x += (float)rx * SERVO_RATE / 100.0f;
        servo_pos_y += (float)ry * SERVO_RATE / 100.0f;
        servo_pos_x = constrain(servo_pos_x, -100.0f, 100.0f);
        servo_pos_y = constrain(servo_pos_y, -100.0f, 100.0f);

        mcpwm_comparator_set_compare_value(servo_lr_cmp, servoFloatToUs(servo_pos_x));
        mcpwm_comparator_set_compare_value(servo_ud_cmp, servoFloatToUs(servo_pos_y));

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================================================================
 *  Error LED
 * ================================================================ */
static void initErrorLED() {
    pinMode(LED_ERR_PIN, OUTPUT);
    digitalWrite(LED_ERR_PIN, HIGH);    // ON at startup (no BLE connection yet)
}

static void setErrorLED(bool on) {
    digitalWrite(LED_ERR_PIN, on ? HIGH : LOW);
}

/* ================================================================
 *  BLE Server  (NimBLE, Coded PHY long-range advertising)
 * ================================================================ */
static NimBLEServer *pServer = NULL;

class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer *, NimBLEConnInfo &ci) override {
        setErrorLED(false);             // Clear error — remote connected
        Serial.printf("BLE connected  handle=%d\n", ci.getConnHandle());
    }
    void onDisconnect(NimBLEServer *, NimBLEConnInfo &, int reason) override {
        motorsStop();
        servo_rate_x = 0;
        servo_rate_y = 0;
        setErrorLED(true);              // Signal error — connection lost
        Serial.printf("BLE disconnected (reason=%d) — motors stopped\n", reason);
        // Restart advertising
        NimBLEDevice::getAdvertising()->start(0);
    }
};

class CmdCharCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic *pChr, NimBLEConnInfo &) override {
        NimBLEAttValue val = pChr->getValue();
        if (val.size() == sizeof(ControlCommand)) {
            ControlCommand cmd;
            memcpy(&cmd, val.data(), sizeof(cmd));
            motorsDrive(&cmd);
            servo_rate_x = cmd.pan_x;
            servo_rate_y = cmd.pan_y;
        }
    }
};

static void initBLE() {
    NimBLEDevice::init(RC_DEVICE_NAME);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCB());

    NimBLEService *svc = pServer->createService(
        NimBLEUUID((uint16_t)RC_SERVICE_UUID));

    NimBLECharacteristic *chr = svc->createCharacteristic(
        NimBLEUUID((uint16_t)RC_COMMAND_CHR_UUID),
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    chr->setCallbacks(new CmdCharCB());

    svc->start();

    // Extended advertising with Coded PHY for long range
    NimBLEExtAdvertising *pAdv = NimBLEDevice::getAdvertising();
    NimBLEExtAdvertisement adv(BLE_HCI_LE_PHY_CODED, BLE_HCI_LE_PHY_CODED);
    adv.setConnectable(true);
    adv.setName(RC_DEVICE_NAME);
    adv.setCompleteServices16({NimBLEUUID((uint16_t)RC_SERVICE_UUID)});
    pAdv->setInstanceData(0, adv);
    pAdv->start(0);

    Serial.println("BLE advertising started (Coded PHY)");
}

/* ================================================================
 *  Camera  (unchanged sensor config, XCLK moved to GPIO 17,
 *           LEDC timer changed to TIMER_1)
 * ================================================================ */
static AsyncUDP udp;
static uint16_t frame_counter = 0;

static void initCamera() {
    camera_config_t cfg = {};
    cfg.pin_pwdn     = CAM_PIN_PWDN;
    cfg.pin_reset    = CAM_PIN_RESET;
    cfg.pin_xclk     = CAM_PIN_XCLK;
    cfg.pin_sccb_sda = CAM_PIN_SIOD;
    cfg.pin_sccb_scl = CAM_PIN_SIOC;
    cfg.pin_d7       = CAM_PIN_D7;
    cfg.pin_d6       = CAM_PIN_D6;
    cfg.pin_d5       = CAM_PIN_D5;
    cfg.pin_d4       = CAM_PIN_D4;
    cfg.pin_d3       = CAM_PIN_D3;
    cfg.pin_d2       = CAM_PIN_D2;
    cfg.pin_d1       = CAM_PIN_D1;
    cfg.pin_d0       = CAM_PIN_D0;
    cfg.pin_vsync    = CAM_PIN_VSYNC;
    cfg.pin_href     = CAM_PIN_HREF;
    cfg.pin_pclk     = CAM_PIN_PCLK;

    cfg.xclk_freq_hz = 20000000;
    cfg.ledc_timer   = CAM_LEDC_TIMER;     // TIMER_1 — avoids motor TIMER_0
    cfg.ledc_channel = CAM_LEDC_CHANNEL;    // CHANNEL_0
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = FRAMESIZE_QVGA;      // 320x240
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
    cfg.fb_location  = CAMERA_FB_IN_PSRAM;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("Camera init FAILED: 0x%x\n", err);
        return;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_whitebal(s, 1);
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);
        s->set_gain_ctrl(s, 1);
    }
    Serial.println("Camera initialised");
}

static void sendFrame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return;
    if (fb->format != PIXFORMAT_JPEG) { esp_camera_fb_return(fb); return; }

    uint16_t chunk_count = (fb->len + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
    uint8_t pkt[MAX_PACKET_SIZE];

    for (uint16_t i = 0; i < chunk_count; i++) {
        size_t off = (size_t)i * MAX_PAYLOAD_SIZE;
        size_t len = min((size_t)MAX_PAYLOAD_SIZE, fb->len - off);

        VideoPacketHeader *h = (VideoPacketHeader *)pkt;
        h->frame_id     = frame_counter;
        h->chunk_index  = i;
        h->chunk_count  = chunk_count;
        h->payload_size = (uint16_t)len;
        memcpy(pkt + HEADER_SIZE, fb->buf + off, len);
        udp.writeTo(pkt, HEADER_SIZE + len, REMOTE_IP, UDP_PORT);
    }

    frame_counter++;
    esp_camera_fb_return(fb);
}

/* ================================================================
 *  WiFi AP
 * ================================================================ */
static void initWiFiAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 1);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    Serial.printf("AP started  SSID=%s  IP=%s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
}

/* ================================================================
 *  Camera task — pinned to Core 0
 * ================================================================ */
static void cameraTask(void *) {
    initCamera();

    uint32_t fps_t = millis();
    uint16_t fps_n = 0;

    for (;;) {
        if (WiFi.softAPgetStationNum() > 0) {
            sendFrame();
            fps_n++;
            if (millis() - fps_t >= 5000) {
                Serial.printf("TX FPS: %d\n", fps_n / 5);
                fps_n = 0;
                fps_t = millis();
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

/* ================================================================
 *  Arduino entry
 * ================================================================ */
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RC-CAR ===");

    initErrorLED();
    initMotors();
    motorsStop();
    initServos();
    initWiFiAP();
    initBLE();

    // Servo rate-control task on Core 1 (alongside BLE host)
    xTaskCreatePinnedToCore(servoTask, "servo", 2048, NULL, 5, NULL, 1);

    // Camera + video streaming on Core 0 (dedicated to WiFi TX)
    xTaskCreatePinnedToCore(cameraTask, "camera", 8192, NULL, 5, NULL, 0);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
