/*
 * CanSat Satellite - XIAO ESP32S3 Sense
 * SIMPLE & FAST - ESP-NOW + SD + Camera
 */

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"

// ===== PINS =====
#define SD_CS 21
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM     10
#define SIOD_GPIO_NUM     40
#define SIOC_GPIO_NUM     39
#define Y9_GPIO_NUM       48
#define Y8_GPIO_NUM       11
#define Y7_GPIO_NUM       12
#define Y6_GPIO_NUM       14
#define Y5_GPIO_NUM       16
#define Y4_GPIO_NUM       18
#define Y3_GPIO_NUM       17
#define Y2_GPIO_NUM       15
#define VSYNC_GPIO_NUM    38
#define HREF_GPIO_NUM     47
#define PCLK_GPIO_NUM     13

// Ground station MAC
uint8_t groundStationMAC[] = {0xD4, 0xD4, 0xDA, 0x5A, 0x5A, 0x74};

// ===== DATA =====
typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  uint8_t packetType;
  uint16_t packetId;
  float altitude;
  float temperature;
  float pressure;
  float latitude;
  float longitude;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float batteryVoltage;
  uint8_t droneState;
  uint8_t gpsFixQuality;
  int8_t rssi;
} TelemetryPacket;

typedef struct __attribute__((packed)) {
  uint8_t commandType;
  uint8_t param1;
  uint8_t param2;
  uint32_t timestamp;
} CommandPacket;

TelemetryPacket telemetry;
CommandPacket receivedCommand;

// ===== TIMING =====
uint16_t packetCounter = 0;
uint32_t photoCounter = 0;
unsigned long lastSendTime = 0;
unsigned long lastPhotoTime = 0;

const int SEND_INTERVAL_MS = 200;      // 5 Hz telemetry
const int PHOTO_INTERVAL_MS = 500;     // 2 photos/sec

bool peerAdded = false;
bool sdCardOK = false;
bool cameraOK = false;

File logFile;

// ===== CALLBACKS =====
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    memcpy(&receivedCommand, data, sizeof(receivedCommand));
    if (receivedCommand.commandType == 0x03) {
      telemetry.droneState = receivedCommand.param1;
    }
  }
}

// ===== SIMPLE SETUP FUNCTIONS =====
void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(84);
  
  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, groundStationMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    peerAdded = true;
  }
}

void setupSD() {
  SPI.begin(7, 8, 9, SD_CS);
  if (!SD.begin(SD_CS)) {
    sdCardOK = false;
    return;
  }
  sdCardOK = true;
  
  // Create photos folder if not exists
  if (!SD.exists("/photos")) {
    SD.mkdir("/photos");
  }
  
  // Single log file - append mode (adds to existing data)
  bool newFile = !SD.exists("/flight_log.csv");
  logFile = SD.open("/flight_log.csv", FILE_APPEND);
  
  if (logFile && newFile) {
    logFile.println("t,pkt,alt,temp,pres,lat,lon,ax,ay,az,gx,gy,gz,bat,st,gps");
    logFile.flush();
  }
}

void setupCamera() {
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
  config.jpeg_quality = 6;
  config.fb_count = 2;
  config.grab_mode = CAMERA_GRAB_LATEST;

  cameraOK = (esp_camera_init(&config) == ESP_OK);
}

// ===== MAIN FUNCTIONS =====
void takePhoto() {
  if (!cameraOK || !sdCardOK) return;
  
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) return;
  
  // Save to photos folder with timestamp
  String name = "/photos/" + String(millis()) + ".jpg";
  File f = SD.open(name, FILE_WRITE);
  if (f) {
    f.write(fb->buf, fb->len);
    f.close();
  }
  esp_camera_fb_return(fb);
}

void sendData() {
  telemetry.timestamp = millis();
  telemetry.packetId = packetCounter++;
  
  // TODO: Add real sensor data here
  telemetry.altitude = 0.0;
  telemetry.temperature = 25.0;
  telemetry.pressure = 1013.25;
  telemetry.latitude = 0.0;
  telemetry.longitude = 0.0;
  telemetry.accelX = 0.0;
  telemetry.accelY = 0.0;
  telemetry.accelZ = 9.81;
  telemetry.gyroX = 0.0;
  telemetry.gyroY = 0.0;
  telemetry.gyroZ = 0.0;
  telemetry.batteryVoltage = 3.7;
  telemetry.gpsFixQuality = 0;
  
  // Send via ESP-NOW
  if (peerAdded) {
    esp_now_send(groundStationMAC, (uint8_t *)&telemetry, sizeof(telemetry));
  }
  
  // Log to SD (single file)
  if (sdCardOK && logFile) {
    logFile.printf("%lu,%u,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
      telemetry.timestamp, telemetry.packetId, telemetry.altitude,
      telemetry.temperature, telemetry.pressure, telemetry.latitude,
      telemetry.longitude, telemetry.accelX, telemetry.accelY, telemetry.accelZ,
      telemetry.gyroX, telemetry.gyroY, telemetry.gyroZ,
      telemetry.batteryVoltage, telemetry.droneState, telemetry.gpsFixQuality);
    
    if (packetCounter % 5 == 0) logFile.flush();
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  
  setupESPNow();
  setupSD();
  setupCamera();
  
  Serial.printf("\nCANSAT READY | ESP:%s SD:%s CAM:%s\n", 
    peerAdded ? "OK" : "X", 
    sdCardOK ? "OK" : "X", 
    cameraOK ? "OK" : "X");
}

// ===== LOOP =====
void loop() {
  unsigned long now = millis();
  
  // 5 Hz telemetry
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;
    sendData();
  }
  
  // 2 Hz photos
  if (now - lastPhotoTime >= PHOTO_INTERVAL_MS) {
    lastPhotoTime = now;
    takePhoto();
  }
}
