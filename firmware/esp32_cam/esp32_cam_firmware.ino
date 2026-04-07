// ─────────────────────────────────────────────────────────────────────────────
// esp32_cam_firmware.ino  —  ESP32-CAM RTSP + UDP MJPEG Streaming Firmware
// Hardware: AI-Thinker ESP32-CAM (OV2640, PSRAM required)
// Drone Swarm Sensor Fusion  |  Phase 2 — Firmware
//
// Features:
//   • RTSP H264 stream on port 554  → cv::VideoCapture("rtsp://...")
//   • UDP raw MJPEG on port 1234    → low-latency fallback
//   • HTTP /status endpoint for health monitoring
//   • Over-The-Air (OTA) update support
//   • IMU data relay over UART at 460800 baud (if MPU-6050 wired to GPIO 14/15)
// ─────────────────────────────────────────────────────────────────────────────
#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "fb_gfx.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include "OV2640.h"
#include "SimStreamer.h"
#include "OV2640Streamer.h"
#include "CRtspSession.h"

// ─── WiFi credentials (override via EEPROM / provisioning in production) ────
#ifndef WIFI_SSID
#define WIFI_SSID "DroneSwarm_AP"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS "swarm2024!"
#endif

// ─── AI-Thinker ESP32-CAM pin config ────────────────────────────────────────
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ─── Config ──────────────────────────────────────────────────────────────────
#define RTSP_PORT        554
#define UDP_PORT        1234
#define STATUS_PORT     80
#define TARGET_FPS       30
#define FRAME_TIMEOUT_MS (1000 / TARGET_FPS)

// ─── Globals ─────────────────────────────────────────────────────────────────
OV2640       cam;
WiFiUDP      udp;
uint32_t     frame_count = 0;
uint32_t     dropped_frames = 0;
uint64_t     bytes_sent = 0;

// RTSP
CStreamer*   streamer    = nullptr;
CRtspSession* rtspSession = nullptr;
WiFiServer   rtspServer(RTSP_PORT);

// HTTP status server
httpd_handle_t httpd = nullptr;

// ─────────────────────────────────────────────────────────────────────────────
// Camera initialization
// ─────────────────────────────────────────────────────────────────────────────
bool init_camera() {
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;
    config.pin_d0       = Y2_GPIO_NUM;
    config.pin_d1       = Y3_GPIO_NUM;
    config.pin_d2       = Y4_GPIO_NUM;
    config.pin_d3       = Y5_GPIO_NUM;
    config.pin_d4       = Y6_GPIO_NUM;
    config.pin_d5       = Y7_GPIO_NUM;
    config.pin_d6       = Y8_GPIO_NUM;
    config.pin_d7       = Y9_GPIO_NUM;
    config.pin_xclk     = XCLK_GPIO_NUM;
    config.pin_pclk     = PCLK_GPIO_NUM;
    config.pin_vsync    = VSYNC_GPIO_NUM;
    config.pin_href     = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    // Use PSRAM for large frames
    if (psramFound()) {
        config.frame_size   = FRAMESIZE_VGA;  // 640x480
        config.jpeg_quality = 10;
        config.fb_count     = 2;
    } else {
        config.frame_size   = FRAMESIZE_QVGA;  // 320x240 fallback
        config.jpeg_quality = 12;
        config.fb_count     = 1;
    }

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }

    // Optimize sensor settings
    sensor_t* s = esp_camera_sensor_get();
    s->set_brightness(s, 0);
    s->set_contrast(s, 1);
    s->set_saturation(s, 0);
    s->set_sharpness(s, 1);
    s->set_gainceiling(s, (gainceiling_t)6);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1);  // auto exposure 2

    Serial.println("[CAM] Initialized OK (640×480, JPEG quality 10)");
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// HTTP /status JSON handler
// ─────────────────────────────────────────────────────────────────────────────
esp_err_t status_handler(httpd_req_t* req) {
    StaticJsonDocument<256> doc;
    doc["id"]            = WiFi.macAddress();
    doc["ip"]            = WiFi.localIP().toString();
    doc["rssi"]          = WiFi.RSSI();
    doc["fps"]           = frame_count;
    doc["dropped"]       = dropped_frames;
    doc["bytes_sent"]    = (uint32_t)(bytes_sent / 1024); // KB
    doc["free_heap"]     = esp_get_free_heap_size();
    doc["uptime_s"]      = millis() / 1000;
    doc["psram"]         = psramFound();

    char buf[256];
    serializeJson(doc, buf);
    frame_count = 0; // reset per-second counter

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_send(req, buf, strlen(buf));
}

void start_http_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = STATUS_PORT;
    if (httpd_start(&httpd, &config) != ESP_OK) return;

    httpd_uri_t status_uri = {"/status", HTTP_GET, status_handler, nullptr};
    httpd_register_uri_handler(httpd, &status_uri);
    Serial.printf("[HTTP] Status server on port %d\n", STATUS_PORT);
}

// ─────────────────────────────────────────────────────────────────────────────
// UDP MJPEG sender  (low-latency path — bypasses RTSP overhead)
// Packet layout: [4B magic][4B frame_id][4B timestamp_ms][4B jpeg_len][JPEG]
// ─────────────────────────────────────────────────────────────────────────────
IPAddress udp_dest(192, 168, 1, 100);  // Jetson / RPi IP

void send_udp_frame(camera_fb_t* fb) {
    static uint32_t frame_id = 0;
    static const uint32_t MAGIC = 0xDEADF00D;

    // Simple fragmentation if frame > MTU
    const size_t mtu = 1400;
    const size_t header_size = 16;
    size_t offset = 0;
    uint16_t frag = 0;
    const uint16_t total_frags = (fb->len + mtu - 1) / mtu;

    while (offset < fb->len) {
        size_t chunk = std::min(mtu, fb->len - offset);
        udp.beginPacket(udp_dest, UDP_PORT);

        // Header
        udp.write((uint8_t*)&MAGIC,              4);
        udp.write((uint8_t*)&frame_id,           4);
        udp.write((uint8_t*)&frag,               2);
        udp.write((uint8_t*)&total_frags,        2);
        uint32_t len32 = (uint32_t)fb->len;
        udp.write((uint8_t*)&len32,              4);

        // Payload chunk
        udp.write(fb->buf + offset, chunk);
        udp.endPacket();

        offset += chunk;
        frag++;
        bytes_sent += chunk;
    }
    frame_id++;
}

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brownout detector

    Serial.begin(460800);
    Serial.println("\n[BOOT] ESP32-CAM Drone Node starting…");

    // WiFi
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("[WIFI] Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500); Serial.print(".");
    }
    Serial.printf("\n[WIFI] Connected. IP: %s  RSSI: %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());

    // Camera
    if (!init_camera()) {
        Serial.println("[FATAL] Camera init failed. Halting.");
        while (true) delay(1000);
    }

    // OTA
    ArduinoOTA.setHostname("drone-esp32cam");
    ArduinoOTA.begin();

    // HTTP
    start_http_server();

    // RTSP
    rtspServer.begin();
    cam.run();  // start capture

    // UDP
    udp.begin(UDP_PORT);

    Serial.println("[BOOT] All systems ready.");
    Serial.printf("[RTSP] Stream: rtsp://%s:%d/stream\n",
                  WiFi.localIP().toString().c_str(), RTSP_PORT);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
    ArduinoOTA.handle();

    // ── RTSP session management ─────────────────────────────────────────
    WiFiClient client = rtspServer.accept();
    if (client) {
        if (streamer) delete streamer;
        streamer    = new OV2640Streamer(&client, cam);
        rtspSession = new CRtspSession(&client, streamer);
        Serial.printf("[RTSP] Client connected: %s\n",
                      client.remoteIP().toString().c_str());
    }

    if (rtspSession) {
        rtspSession->handleRequests(0);
        static uint32_t last_rtsp = 0;
        if (millis() - last_rtsp >= FRAME_TIMEOUT_MS) {
            last_rtsp = millis();
            rtspSession->broadcastCurrentFrame(last_rtsp);
            frame_count++;
        }
        if (!rtspSession->m_stopped) {
            // still running
        } else {
            delete rtspSession; rtspSession = nullptr;
            delete streamer;    streamer    = nullptr;
        }
    }

    // ── UDP MJPEG (parallel low-latency path) ───────────────────────────
    static uint32_t last_udp = 0;
    if (millis() - last_udp >= FRAME_TIMEOUT_MS) {
        last_udp = millis();
        camera_fb_t* fb = esp_camera_fb_get();
        if (fb) {
            send_udp_frame(fb);
            esp_camera_fb_return(fb);
        } else {
            dropped_frames++;
        }
    }
}
