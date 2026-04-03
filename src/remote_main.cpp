#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_wifi.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include <NimBLEDevice.h>

#include "video_protocol.h"
#include "ble_protocol.h"

/* ================================================================
 *  Pin Definitions
 * ================================================================ */

// ---- Display (U1 → U16 ILI9341, parallel 8-bit) ----
#define LCD_WR_PIN      16
#define LCD_RD_PIN      15
#define LCD_RS_PIN      39      // DC/RS
#define LCD_CS_PIN      40
#define LCD_RST_PIN     41
#define LCD_D0_PIN      13
#define LCD_D1_PIN      12
#define LCD_D2_PIN      42
#define LCD_D3_PIN      47
#define LCD_D4_PIN      38
#define LCD_D5_PIN      18
#define LCD_D6_PIN      17
#define LCD_D7_PIN      14

// ---- Joystick 1 — driving (U1 → U6 HW-504-left) ----
#define JOY1_VRX_PIN    1       // ADC1_CH0
#define JOY1_VRY_PIN    2       // ADC1_CH1
#define JOY1_SW_PIN     4       // ADC1_CH3

// ---- Joystick 2 — camera pan (U1 → U7 HW-504-right) ----
#define JOY2_VRX_PIN    5       // ADC1_CH4
#define JOY2_VRY_PIN    6       // ADC1_CH5
#define JOY2_SW_PIN     7       // ADC1_CH6

// ---- LEDs (U1 → U12-U15 resistors → U8-U11 LEDs) ----
#define LED_SPD1_PIN    8       // SPD01
#define LED_SPD2_PIN    9       // SPD02
#define LED_SPD3_PIN    10      // SPD03
#define LED_CONN_PIN    11      // ERR — ON = disconnected

/* ================================================================
 *  LovyanGFX display (ILI9341, parallel 8-bit)
 * ================================================================ */
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel;
    lgfx::Bus_Parallel8 _bus;

public:
    LGFX() {
        {
            auto cfg       = _bus.config();
            cfg.freq_write = 16000000;
            cfg.pin_wr     = LCD_WR_PIN;
            cfg.pin_rd     = LCD_RD_PIN;
            cfg.pin_rs     = LCD_RS_PIN;
            cfg.pin_d0     = LCD_D0_PIN;
            cfg.pin_d1     = LCD_D1_PIN;
            cfg.pin_d2     = LCD_D2_PIN;
            cfg.pin_d3     = LCD_D3_PIN;
            cfg.pin_d4     = LCD_D4_PIN;
            cfg.pin_d5     = LCD_D5_PIN;
            cfg.pin_d6     = LCD_D6_PIN;
            cfg.pin_d7     = LCD_D7_PIN;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg          = _panel.config();
            cfg.pin_cs        = LCD_CS_PIN;
            cfg.pin_rst       = LCD_RST_PIN;
            cfg.pin_busy      = -1;
            cfg.memory_width  = 240;
            cfg.memory_height = 320;
            cfg.panel_width   = 240;
            cfg.panel_height  = 320;
            cfg.offset_x      = 0;
            cfg.offset_y      = 0;
            cfg.readable      = true;
            cfg.invert        = false;
            cfg.rgb_order     = false;  // BGR — standard ILI9341
            cfg.dlen_16bit    = false;  // 8-bit bus
            _panel.config(cfg);
            setPanel(&_panel);
        }
    }
};

static LGFX display;

// ---- Joystick tuning ----
#define JOY1_DEADZONE   200
#define JOY2_DEADZONE   350
#define JOY_CAL_SAMPLES 64
#define ADC_MAX         4095

// ---- Speed modes ----
static const uint8_t SPEED_MODES[] = { 30, 60, 100 };
#define SPEED_MODE_COUNT (sizeof(SPEED_MODES) / sizeof(SPEED_MODES[0]))
static int      speed_mode_idx = 0;
static uint8_t  current_speed  = 30;

/* ================================================================
 *  WiFi
 * ================================================================ */
static const char *AP_SSID     = "RC-CAR-CAM";
static const char *AP_PASSWORD = "rccar12345";

/* ================================================================
 *  Frame reassembly (unchanged logic from original)
 * ================================================================ */
static uint8_t *frame_buf_recv    = nullptr;
static uint8_t *frame_buf_display = nullptr;

static volatile uint16_t current_frame_id = 0xFFFF;
static volatile uint16_t chunks_received  = 0;
static volatile uint16_t expected_chunks  = 0;
static volatile size_t   assembled_size   = 0;
static volatile bool     frame_ready      = false;
static volatile size_t   ready_frame_size = 0;

static bool chunk_bitmap[128];  // max 100KB / 1400 ≈ 72 chunks

static AsyncUDP udp;

void IRAM_ATTR onPacketReceived(AsyncUDPPacket &packet) {
    if (packet.length() < HEADER_SIZE) return;

    const VideoPacketHeader *hdr = (const VideoPacketHeader *)packet.data();
    const uint8_t *payload = packet.data() + HEADER_SIZE;

    if (hdr->chunk_index >= hdr->chunk_count)            return;
    if (hdr->payload_size > MAX_PAYLOAD_SIZE)             return;
    if (hdr->payload_size + HEADER_SIZE > packet.length()) return;

    if (hdr->frame_id != current_frame_id) {
        current_frame_id = hdr->frame_id;
        chunks_received  = 0;
        expected_chunks  = hdr->chunk_count;
        assembled_size   = 0;
        memset(chunk_bitmap, 0, sizeof(chunk_bitmap));
    }

    if (chunk_bitmap[hdr->chunk_index]) return;

    size_t offset = (size_t)hdr->chunk_index * MAX_PAYLOAD_SIZE;
    memcpy(frame_buf_recv + offset, payload, hdr->payload_size);
    chunk_bitmap[hdr->chunk_index] = true;
    chunks_received++;

    if (hdr->chunk_index == hdr->chunk_count - 1)
        assembled_size = offset + hdr->payload_size;

    if (chunks_received == expected_chunks && assembled_size > 0) {
        if (!frame_ready) {
            uint8_t *tmp      = frame_buf_recv;
            frame_buf_recv    = frame_buf_display;
            frame_buf_display = tmp;
            ready_frame_size  = assembled_size;
            frame_ready       = true;
        }
        current_frame_id = 0xFFFF;
        chunks_received  = 0;
    }
}

/* ================================================================
 *  Joystick
 * ================================================================ */
static int joy1_cx, joy1_cy, joy2_cx, joy2_cy;

static void initJoysticks() {
    pinMode(JOY1_SW_PIN, INPUT_PULLUP);
    pinMode(JOY2_SW_PIN, INPUT_PULLUP);
}

static void calibrateJoysticks() {
    int32_t s1x = 0, s1y = 0, s2x = 0, s2y = 0;
    for (int i = 0; i < JOY_CAL_SAMPLES; i++) {
        s1x += analogRead(JOY1_VRX_PIN);
        s1y += analogRead(JOY1_VRY_PIN);
        s2x += analogRead(JOY2_VRX_PIN);
        s2y += analogRead(JOY2_VRY_PIN);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    joy1_cx = s1x / JOY_CAL_SAMPLES;
    joy1_cy = s1y / JOY_CAL_SAMPLES;
    joy2_cx = s2x / JOY_CAL_SAMPLES;
    joy2_cy = s2y / JOY_CAL_SAMPLES;
    Serial.printf("Joy1 center: %d,%d   Joy2 center: %d,%d\n",
                  joy1_cx, joy1_cy, joy2_cx, joy2_cy);
}

static int readAxis(int pin, int center, int deadzone) {
    int raw      = analogRead(pin);
    int centered = raw - center;
    if (abs(centered) < deadzone) return 0;
    int range = (centered > 0) ? (ADC_MAX - center) : center;
    if (range == 0) range = 1;
    int val = (int)((int32_t)centered * 100 / range);
    return constrain(val, -100, 100);
}

/* ================================================================
 *  LEDs
 * ================================================================ */
static void initLEDs() {
    pinMode(LED_CONN_PIN, OUTPUT);
    pinMode(LED_SPD1_PIN, OUTPUT);
    pinMode(LED_SPD2_PIN, OUTPUT);
    pinMode(LED_SPD3_PIN, OUTPUT);
    digitalWrite(LED_CONN_PIN, HIGH);   // ON = not connected
    digitalWrite(LED_SPD1_PIN, LOW);
    digitalWrite(LED_SPD2_PIN, LOW);
    digitalWrite(LED_SPD3_PIN, LOW);
}

static void updateSpeedLEDs() {
    int lit = speed_mode_idx + 1;
    digitalWrite(LED_SPD1_PIN, lit >= 1 ? HIGH : LOW);
    digitalWrite(LED_SPD2_PIN, lit >= 2 ? HIGH : LOW);
    digitalWrite(LED_SPD3_PIN, lit >= 3 ? HIGH : LOW);
}

static void updateConnectionLED(bool connected) {
    digitalWrite(LED_CONN_PIN, connected ? LOW : HIGH);
}

/* ================================================================
 *  BLE Client  (NimBLE, Coded PHY scanning)
 * ================================================================ */
static NimBLEClient              *pClient   = nullptr;
static NimBLERemoteCharacteristic *pCmdChar  = nullptr;
static volatile bool              bleConnected = false;
static volatile bool              deviceFound  = false;
static volatile bool              needRescan   = false;
static NimBLEAddress              targetAddr;

class ClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient *) override {
        Serial.println("BLE connected to car");
    }
    void onDisconnect(NimBLEClient *, int reason) override {
        bleConnected = false;
        pCmdChar     = nullptr;
        needRescan   = true;
        Serial.printf("BLE disconnected (reason=%d)\n", reason);
    }
};

class ScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice *dev) override {
        if (dev->isAdvertisingService(NimBLEUUID((uint16_t)RC_SERVICE_UUID))) {
            targetAddr  = dev->getAddress();
            deviceFound = true;
            NimBLEDevice::getScan()->stop();
            Serial.println("Found RC-Car");
        }
    }
};

static bool connectToCar() {
    if (!pClient) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new ClientCB());
    }

    if (!pClient->connect(targetAddr)) {
        Serial.println("BLE connect failed");
        return false;
    }

    NimBLERemoteService *svc =
        pClient->getService(NimBLEUUID((uint16_t)RC_SERVICE_UUID));
    if (!svc) {
        Serial.println("Service not found");
        pClient->disconnect();
        return false;
    }

    pCmdChar = svc->getCharacteristic(
        NimBLEUUID((uint16_t)RC_COMMAND_CHR_UUID));
    if (!pCmdChar) {
        Serial.println("Characteristic not found");
        pClient->disconnect();
        return false;
    }

    bleConnected = true;
    return true;
}

static void startBLEScan() {
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->clearResults();
    scan->start(0, false);      // 0 = scan forever until stopped
}

static void initBLE() {
    NimBLEDevice::init("RC-Remote");

    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setScanCallbacks(new ScanCB());
    scan->setActiveScan(true);
    scan->setInterval(256);     // 160 ms
    scan->setWindow(128);       //  80 ms

    startBLEScan();
    Serial.println("BLE scanning started");
}

/* ================================================================
 *  Control task — pinned to Core 1
 *  Reads joysticks, manages speed/LEDs, sends BLE commands at 50 Hz
 * ================================================================ */
static void controlTask(void *) {
    initJoysticks();
    calibrateJoysticks();
    initLEDs();
    updateSpeedLEDs();
    updateConnectionLED(false);
    initBLE();

    bool sw_prev = true;

    for (;;) {
        /* ---- BLE connection management ---- */
        if (needRescan) {
            needRescan = false;
            vTaskDelay(pdMS_TO_TICKS(500));
            startBLEScan();
        }
        if (deviceFound && !bleConnected) {
            deviceFound = false;
            if (!connectToCar()) {
                vTaskDelay(pdMS_TO_TICKS(500));
                startBLEScan();
            }
        }

        /* ---- Speed mode button (Joy1 SW, debounced) ---- */
        bool sw = digitalRead(JOY1_SW_PIN) == LOW;
        if (sw && !sw_prev) {
            speed_mode_idx = (speed_mode_idx + 1) % SPEED_MODE_COUNT;
            current_speed  = SPEED_MODES[speed_mode_idx];
            updateSpeedLEDs();
        }
        sw_prev = sw;

        /* ---- Read joystick axes ---- */
        int x  = readAxis(JOY1_VRX_PIN, joy1_cx, JOY1_DEADZONE);
        int y  = readAxis(JOY1_VRY_PIN, joy1_cy, JOY1_DEADZONE);
        int px = readAxis(JOY2_VRX_PIN, joy2_cx, JOY2_DEADZONE);
        int py = readAxis(JOY2_VRY_PIN, joy2_cy, JOY2_DEADZONE);

        /* ---- Build command ---- */
        ControlCommand cmd = {};
        cmd.pan_x = (int8_t)constrain(px, -100, 100);
        cmd.pan_y = (int8_t)constrain(py, -100, 100);
        if (x != 0 || y != 0) {
            cmd.x     = (int8_t)constrain(x, -100, 100);
            cmd.y     = (int8_t)constrain(y, -100, 100);
            cmd.speed = current_speed;
        }

        /* ---- Send via BLE ---- */
        if (bleConnected && pCmdChar) {
            pCmdChar->writeValue((uint8_t *)&cmd, sizeof(cmd), false);
        }

        /* ---- LEDs ---- */
        updateConnectionLED(bleConnected);

        vTaskDelay(pdMS_TO_TICKS(20));  // 50 Hz
    }
}

/* ================================================================
 *  Video task — pinned to Core 0
 *  Initialises display & WiFi, receives UDP frames, renders JPEG
 * ================================================================ */
static void videoTask(void *) {
    // Display init on the same core that will render
    display.init();
    display.setRotation(1);         // Landscape 320x240
    display.setBrightness(255);
    display.setColorDepth(16);
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE);
    display.setTextSize(2);
    display.drawString("Waiting for", 70, 100);
    display.drawString("video...", 90, 125);
    Serial.println("Display initialised");

    // WiFi STA connect
    WiFi.mode(WIFI_STA);
    WiFi.begin(AP_SSID, AP_PASSWORD);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    display.fillScreen(TFT_BLACK);
    display.drawString("Connecting...", 70, 110);

    while (WiFi.status() != WL_CONNECTED)
        vTaskDelay(pdMS_TO_TICKS(250));

    Serial.printf("WiFi connected  IP=%s\n", WiFi.localIP().toString().c_str());
    display.fillScreen(TFT_BLACK);
    display.drawString("Connected!", 80, 110);

    // Start UDP listener (callback runs in LwIP task on Core 0)
    if (udp.listen(UDP_PORT)) {
        udp.onPacket(onPacketReceived);
        Serial.printf("UDP listening on port %d\n", UDP_PORT);
    }

    uint32_t fps_t = millis();
    uint16_t fps_n = 0;

    for (;;) {
        if (frame_ready) {
            display.startWrite();
            display.drawJpg(frame_buf_display, ready_frame_size,
                            0, 0, 320, 240);
            display.endWrite();
            frame_ready = false;

            fps_n++;
            if (millis() - fps_t >= 5000) {
                Serial.printf("RX FPS: %d\n", fps_n / 5);
                fps_n = 0;
                fps_t = millis();
            }
        }
        vTaskDelay(1);  // yield to other Core-0 tasks
    }
}

/* ================================================================
 *  Arduino entry
 * ================================================================ */
void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RC-CAR Remote ===");

    // PSRAM frame buffers
    frame_buf_recv    = (uint8_t *)ps_malloc(MAX_FRAME_SIZE);
    frame_buf_display = (uint8_t *)ps_malloc(MAX_FRAME_SIZE);
    if (!frame_buf_recv || !frame_buf_display) {
        Serial.println("PSRAM alloc failed!");
        ESP.restart();
    }

    // Video (display + WiFi + UDP) on Core 0
    xTaskCreatePinnedToCore(videoTask, "video", 16384, NULL, 5, NULL, 0);

    // Control (joystick + LEDs + BLE) on Core 1
    xTaskCreatePinnedToCore(controlTask, "control", 8192, NULL, 5, NULL, 1);
}

void loop() {
    vTaskDelay(portMAX_DELAY);
}
