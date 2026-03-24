#include <esp_now.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "protocol.h"

uint8_t groundStationMAC[] = { 0xD4, 0xD4, 0xDA, 0x5A, 0x5A, 0x74 };

#define RXD1 44
#define TXD1 43

#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 10
#define SIOD_GPIO_NUM 40
#define SIOC_GPIO_NUM 39
#define Y9_GPIO_NUM 48
#define Y8_GPIO_NUM 11
#define Y7_GPIO_NUM 12
#define Y6_GPIO_NUM 14
#define Y5_GPIO_NUM 16
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 17
#define Y2_GPIO_NUM 15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM 47
#define PCLK_GPIO_NUM 13
#define SD_CS 21

esp_now_peer_info_t peerInfo;
int photoCount = 0;
bool continuousCapture = false;
unsigned long lastCaptureTime = 0;
unsigned long captureInterval = 2000;

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.grab_mode = CAMERA_GRAB_LATEST;
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    return false;
  }
  return true;
}

bool initSD() {
  if (!SD.begin(SD_CS)) return false;
  if (SD.cardType() == CARD_NONE) return false;
  return true;
}

bool takePhoto() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return false;
  char fn[32];
  snprintf(fn, sizeof(fn), "/photo_%03d.jpg", photoCount);
  File f = SD.open(fn, FILE_WRITE);
  if (!f) {
    esp_camera_fb_return(fb);
    return false;
  }
  f.write(fb->buf, fb->len);
  f.close();
  Serial.printf("Saved %s\n", fn);
  photoCount++;
  esp_camera_fb_return(fb);
  return true;
}

// Handle bridge-local commands; return true if consumed (don't forward to FC)
bool handleBridgeCommand(const Command &c) {
  char buf[sizeof(c.cmd) + 1];
  memcpy(buf, c.cmd, sizeof(c.cmd));
  buf[sizeof(c.cmd)] = '\0';
  String cmd = String(buf);
  cmd.trim();
  if (cmd == "photo") {
    takePhoto();
    return true;
  }
  if (cmd == "startcam") {
    continuousCapture = true;
    return true;
  }
  if (cmd == "stopcam") {
    continuousCapture = false;
    return true;
  }
  return false;
}

// ── ESP-NOW receive — from GS ESP32 ──────────────────────────────────────────
// GS sends raw structs (no framing) over ESP-NOW; dispatch by size.
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len == sizeof(Command)) {
    Command c;
    memcpy(&c, data, sizeof(c));
    if (!handleBridgeCommand(c))
      Serial.println("Received command");
    sendPacket(Serial1, c);  // tagged frame to FC
  } else if (len == sizeof(PIDCommand)) {
    PIDCommand pc;
    memcpy(&pc, data, sizeof(pc));
    sendPacket(Serial1, pc);  // tagged frame to FC
    Serial.printf("[BR] PIDCmd gain=%d val=%.6f\n", pc.gain_id, pc.value);
  } else {
    Serial.printf("[BR] Unknown recv size %d\n", len);
  }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {}

void setup() {
  Serial.begin(115200);
  Serial1.begin(921600, SERIAL_8N1, RXD1, TXD1);
  initCamera();
  initSD();

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true) {}
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  memcpy(peerInfo.peer_addr, groundStationMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    while (true) {}
  }
  Serial.println("Bridge ready");
}

void loop() {
  // Forward any upstream packets from FC to GS via ESP-NOW
  SensorData sd;
  PIDConfig cfg;
  uint8_t type = receiveAnyPacket(Serial1, &sd, nullptr, nullptr, &cfg);
  if (type == PKT_SENSOR_DATA) {
    esp_now_send(groundStationMAC, (uint8_t *)&sd, sizeof(sd));
  } else if (type == PKT_PID_CONFIG) {
    esp_now_send(groundStationMAC, (uint8_t *)&cfg, sizeof(cfg));
    Serial.println("[BR] PIDConfig → GS");
  }

  if (continuousCapture && (millis() - lastCaptureTime >= captureInterval)) {
    Serial.println("capturing photo...")
    takePhoto();
    lastCaptureTime = millis();
  }
}
