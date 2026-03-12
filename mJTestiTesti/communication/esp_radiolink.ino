// esp32s3 sense - bridge + camera + SD

#include <esp_now.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "protocol.h"

uint8_t groundMAC[] = {0x08, 0xD1, 0xF9, 0xF4, 0xBF, 0x50};

#define RXD1 44
#define TXD1 43

// ── Camera pins (XIAO ESP32S3 Sense) ──
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

// ── SD card pins (SPI) ──
#define SD_CS   21

esp_now_peer_info_t peerInfo;
int photoCount = 0;
bool continuousCapture = false;
unsigned long lastCaptureTime = 0;
unsigned long captureInterval = 2000;  // ms between photos when continuous

// ── Camera init ──
bool initCamera() {
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
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn     = PWDN_GPIO_NUM;
    config.pin_reset    = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;
    config.frame_size   = FRAMESIZE_UXGA;   // 1600x1200
    config.jpeg_quality = 10;               // 1-63, lower = better quality
    config.fb_count     = 1;
    config.grab_mode    = CAMERA_GRAB_LATEST;

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed: 0x%x\n", err);
        return false;
    }
    Serial.println("Camera OK");
    return true;
}

// ── SD card init ──
bool initSD() {
    if (!SD.begin(SD_CS)) {
        Serial.println("SD mount failed!");
        return false;
    }
    uint8_t cardType = SD.cardType();
    if (cardType == CARD_NONE) {
        Serial.println("No SD card!");
        return false;
    }
    Serial.printf("SD card ready, size: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    return true;
}

// ── Take photo and save to SD ──
bool takePhoto() {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("Capture failed!");
        return false;
    }

    // Create filename: /photo_001.jpg, /photo_002.jpg, etc.
    char filename[32];
    snprintf(filename, sizeof(filename), "/photo_%03d.jpg", photoCount);

    File file = SD.open(filename, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file!");
        esp_camera_fb_return(fb);
        return false;
    }

    file.write(fb->buf, fb->len);
    file.close();
    esp_camera_fb_return(fb);

    Serial.printf("Saved: %s (%d bytes)\n", filename, fb->len);
    photoCount++;
    return true;
}

// ── Handle commands that are for the bridge itself ──
bool handleBridgeCommand(const Command &c) {
    String cmd = String(c.cmd);
    cmd.trim();

    if (cmd == "photo") {
        takePhoto();
        return true;  // handled here, don't forward to FC
    }
    else if (cmd == "startcam") {
        continuousCapture = true;
        Serial.println("Continuous capture ON");
        return true;
    }
    else if (cmd == "stopcam") {
        continuousCapture = false;
        Serial.println("Continuous capture OFF");
        return true;
    }
    return false;  // not a bridge command, forward to FC
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len == sizeof(Command)) {
        Command c;
        memcpy(&c, data, sizeof(c));

        Serial.print("Command from ground: ");
        Serial.println(c.cmd);

        // Check if it's a bridge command first
        if (!handleBridgeCommand(c)) {
            // Not a bridge command → forward to FC
            sendPacket(Serial1, c);
        }
    }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("Send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(921600, SERIAL_8N1, RXD1, TXD1);

    // Init camera and SD
    bool camOK = initCamera();
    bool sdOK  = initSD();

    if (camOK && sdOK) {
        Serial.println("Camera + SD ready");
    } else {
        Serial.println("Camera/SD init failed, radio still works");
    }

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        while (true);
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    memcpy(peerInfo.peer_addr, groundMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer!");
        while (true);
    }

    Serial.println("Bridge ready");
}

void loop() {
    // Forward sensor data from FC to ground
    SensorData sd;
    if (receivePacket(Serial1, sd)) {
        esp_now_send(groundMAC, (uint8_t *)&sd, sizeof(sd));
    }

    // Continuous photo capture if enabled
    if (continuousCapture && (millis() - lastCaptureTime >= captureInterval)) {
        takePhoto();
        lastCaptureTime = millis();
    }
}
