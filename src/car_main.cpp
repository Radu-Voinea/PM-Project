#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <esp_camera.h>
#include <esp_wifi.h>
#include "video_protocol.h"

// ----- Camera pin definitions -----
#define CAM_PIN_PWDN    3
#define CAM_PIN_RESET   10
#define CAM_PIN_XCLK    15  // Dummy — module has onboard oscillator
#define CAM_PIN_SIOD    6   // SDA
#define CAM_PIN_SIOC    7   // SCL
#define CAM_PIN_D0      5
#define CAM_PIN_D1      11
#define CAM_PIN_D2      4
#define CAM_PIN_D3      12
#define CAM_PIN_D4      16
#define CAM_PIN_D5      13
#define CAM_PIN_D6      2
#define CAM_PIN_D7      14
#define CAM_PIN_VSYNC   8
#define CAM_PIN_HREF    9
#define CAM_PIN_PCLK    1

// ----- WiFi AP config -----
static const char *AP_SSID     = "RC-CAR-CAM";
static const char *AP_PASSWORD = "rccar12345";
static const IPAddress REMOTE_IP(192, 168, 4, 2);

// ----- Globals -----
AsyncUDP udp;
uint16_t frame_counter = 0;

// FPS tracking
uint32_t fps_timer = 0;
uint16_t fps_count = 0;

void initCamera() {
    camera_config_t config;
    config.pin_pwdn     = CAM_PIN_PWDN;
    config.pin_reset    = CAM_PIN_RESET;
    config.pin_xclk     = CAM_PIN_XCLK;
    config.pin_sccb_sda = CAM_PIN_SIOD;
    config.pin_sccb_scl = CAM_PIN_SIOC;
    config.pin_d7       = CAM_PIN_D7;
    config.pin_d6       = CAM_PIN_D6;
    config.pin_d5       = CAM_PIN_D5;
    config.pin_d4       = CAM_PIN_D4;
    config.pin_d3       = CAM_PIN_D3;
    config.pin_d2       = CAM_PIN_D2;
    config.pin_d1       = CAM_PIN_D1;
    config.pin_d0       = CAM_PIN_D0;
    config.pin_vsync    = CAM_PIN_VSYNC;
    config.pin_href     = CAM_PIN_HREF;
    config.pin_pclk     = CAM_PIN_PCLK;

    config.xclk_freq_hz = 20000000;
    config.ledc_timer   = LEDC_TIMER_0;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_QVGA;   // 320x240 — matches ILI9341
    config.jpeg_quality = 12;                // 0-63, lower = better quality
    config.fb_count     = 2;                 // Double buffer in PSRAM
    config.fb_location  = CAMERA_FB_IN_PSRAM;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        ESP.restart();
    }

    sensor_t *s = esp_camera_sensor_get();
    s->set_brightness(s, 0);
    s->set_contrast(s, 0);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_gain_ctrl(s, 1);

    Serial.println("Camera initialized");
}

void initWiFiAP() {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD, 1, 0, 1);
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    Serial.printf("AP started — SSID: %s  IP: %s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());
}

void sendFrame() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) return;

    if (fb->format != PIXFORMAT_JPEG) {
        esp_camera_fb_return(fb);
        return;
    }

    uint16_t chunk_count = (fb->len + MAX_PAYLOAD_SIZE - 1) / MAX_PAYLOAD_SIZE;
    uint8_t packet_buf[MAX_PACKET_SIZE];

    for (uint16_t i = 0; i < chunk_count; i++) {
        size_t offset      = (size_t)i * MAX_PAYLOAD_SIZE;
        size_t payload_len = min((size_t)MAX_PAYLOAD_SIZE, fb->len - offset);

        VideoPacketHeader *hdr = (VideoPacketHeader *)packet_buf;
        hdr->frame_id     = frame_counter;
        hdr->chunk_index  = i;
        hdr->chunk_count  = chunk_count;
        hdr->payload_size = (uint16_t)payload_len;

        memcpy(packet_buf + HEADER_SIZE, fb->buf + offset, payload_len);
        udp.writeTo(packet_buf, HEADER_SIZE + payload_len, REMOTE_IP, UDP_PORT);
    }

    frame_counter++;
    esp_camera_fb_return(fb);

    // FPS counter
    fps_count++;
    if (millis() - fps_timer >= 5000) {
        Serial.printf("TX FPS: %d  frame: %u bytes  chunks: %u\n",
                      fps_count / 5, (unsigned)fb->len, chunk_count);
        fps_count = 0;
        fps_timer = millis();
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("\n=== RC-CAR Camera ===");

    initWiFiAP();
    initCamera();

    fps_timer = millis();
}

void loop() {
    if (WiFi.softAPgetStationNum() > 0) {
        sendFrame();
    } else {
        delay(100);
    }
}
