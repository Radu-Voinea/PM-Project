#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_wifi.h>

#define LGFX_USE_V1
#include <LovyanGFX.hpp>

#include "video_protocol.h"

// ----- LovyanGFX display configuration -----
class LGFX : public lgfx::LGFX_Device {
    lgfx::Panel_ILI9341 _panel;
    lgfx::Bus_Parallel8 _bus;

public:
    LGFX() {
        {
            auto cfg       = _bus.config();
            cfg.freq_write = 16000000;  // 16MHz — safe for ILI9341 8080
            cfg.pin_wr     = 47;
            cfg.pin_rd     = 46;
            cfg.pin_rs     = 38;        // DC/RS
            cfg.pin_d0     = 12;
            cfg.pin_d1     = 11;
            cfg.pin_d2     = 48;
            cfg.pin_d3     = 21;
            cfg.pin_d4     = 18;
            cfg.pin_d5     = 17;
            cfg.pin_d6     = 14;
            cfg.pin_d7     = 13;
            _bus.config(cfg);
            _panel.setBus(&_bus);
        }
        {
            auto cfg          = _panel.config();
            cfg.pin_cs        = 41;
            cfg.pin_rst       = 42;
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

// ----- WiFi config -----
static const char *AP_SSID     = "RC-CAR-CAM";
static const char *AP_PASSWORD = "rccar12345";

// ----- Frame reassembly -----
static uint8_t *frame_buf_recv    = nullptr; // Buffer being assembled into
static uint8_t *frame_buf_display = nullptr; // Buffer being displayed from

static volatile uint16_t current_frame_id = 0xFFFF;
static volatile uint16_t chunks_received  = 0;
static volatile uint16_t expected_chunks  = 0;
static volatile size_t   assembled_size   = 0;
static volatile bool     frame_ready      = false;
static volatile size_t   ready_frame_size = 0;

// Max chunks per frame: 100KB / 1400 = 72
static bool chunk_bitmap[128];

AsyncUDP udp;

// FPS tracking
uint32_t fps_timer = 0;
uint16_t fps_count = 0;

void IRAM_ATTR onPacketReceived(AsyncUDPPacket &packet) {
    if (packet.length() < HEADER_SIZE) return;

    const VideoPacketHeader *hdr = (const VideoPacketHeader *)packet.data();
    const uint8_t *payload = packet.data() + HEADER_SIZE;

    // Validate
    if (hdr->chunk_index >= hdr->chunk_count) return;
    if (hdr->payload_size > MAX_PAYLOAD_SIZE) return;
    if (hdr->payload_size + HEADER_SIZE > packet.length()) return;

    // New frame? Drop any partial assembly in progress
    if (hdr->frame_id != current_frame_id) {
        current_frame_id = hdr->frame_id;
        chunks_received  = 0;
        expected_chunks  = hdr->chunk_count;
        assembled_size   = 0;
        memset(chunk_bitmap, 0, sizeof(chunk_bitmap));
    }

    // Skip duplicate chunks
    if (chunk_bitmap[hdr->chunk_index]) return;

    // Place chunk at correct offset
    size_t offset = (size_t)hdr->chunk_index * MAX_PAYLOAD_SIZE;
    memcpy(frame_buf_recv + offset, payload, hdr->payload_size);
    chunk_bitmap[hdr->chunk_index] = true;
    chunks_received++;

    // Track total size from last chunk
    if (hdr->chunk_index == hdr->chunk_count - 1) {
        assembled_size = offset + hdr->payload_size;
    }

    // Frame complete?
    if (chunks_received == expected_chunks && assembled_size > 0) {
        if (!frame_ready) {
            // Swap buffers
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

void initDisplay() {
    display.init();
    display.setRotation(1);       // Landscape: 320x240
    display.setBrightness(255);
    display.setColorDepth(16);    // RGB565
    display.fillScreen(TFT_BLACK);
    display.setTextColor(TFT_WHITE);
    display.setTextSize(2);
    display.drawString("Waiting for", 70, 100);
    display.drawString("video...", 90, 125);
    Serial.println("Display initialized");
}

void initWiFiSTA() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(AP_SSID, AP_PASSWORD);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    Serial.print("Connecting to car AP");
    display.fillScreen(TFT_BLACK);
    display.drawString("Connecting...", 70, 110);

    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(".");
    }
    Serial.printf("\nConnected! IP: %s\n", WiFi.localIP().toString().c_str());

    display.fillScreen(TFT_BLACK);
    display.drawString("Connected!", 80, 100);
    display.drawString("Waiting stream", 50, 125);
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RC-CAR Remote ===");

    // Allocate frame buffers in PSRAM
    frame_buf_recv    = (uint8_t *)ps_malloc(MAX_FRAME_SIZE);
    frame_buf_display = (uint8_t *)ps_malloc(MAX_FRAME_SIZE);
    if (!frame_buf_recv || !frame_buf_display) {
        Serial.println("PSRAM allocation failed!");
        ESP.restart();
    }

    initDisplay();
    initWiFiSTA();

    if (udp.listen(UDP_PORT)) {
        udp.onPacket(onPacketReceived);
        Serial.printf("UDP listening on port %d\n", UDP_PORT);
    }

    fps_timer = millis();
}

void loop() {
    if (frame_ready) {
        display.startWrite();
        display.drawJpg(frame_buf_display, ready_frame_size, 0, 0, 320, 240);
        display.endWrite();
        frame_ready = false;

        fps_count++;
        if (millis() - fps_timer >= 5000) {
            Serial.printf("RX FPS: %d\n", fps_count / 5);
            fps_count = 0;
            fps_timer = millis();
        }
    }
}
