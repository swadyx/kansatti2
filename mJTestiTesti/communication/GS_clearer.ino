/*
 * CanSat Ground Station - ESP32-WROOM
 * Fast display, real RSSI
 */

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// Satellite MAC
uint8_t satelliteMAC[] = {0x98, 0xA3, 0x16, 0xF8, 0x27, 0x40};

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
CommandPacket commandToSend;

uint32_t packetsReceived = 0;
uint32_t packetsLost = 0;
uint16_t lastPacketId = 0;
unsigned long lastPacketTime = 0;
unsigned long lastDisplayTime = 0;
int8_t lastRssi = 0;

bool peerAdded = false;

const char* getState(uint8_t s) {
  switch(s) {
    case 0: return "LAUNCH";
    case 1: return "ASCENT";
    case 2: return "APOGEE";
    case 3: return "DESCENT";
    case 4: return "DRONE";
    case 5: return "LANDED";
    default: return "???";
  }
}

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  lastPacketTime = millis();
  lastRssi = info->rx_ctrl->rssi;
  
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&telemetry, data, sizeof(telemetry));
    
    if (packetsReceived > 0 && telemetry.packetId > lastPacketId + 1) {
      packetsLost += (telemetry.packetId - lastPacketId - 1);
    }
    lastPacketId = telemetry.packetId;
    packetsReceived++;
  }
}

void printTelemetry() {
  float lossPercent = packetsReceived > 0 ? (float)packetsLost / (packetsReceived + packetsLost) * 100 : 0;
  
  Serial.println("════════════════════════════════════════════════════════════════════════════════════════════════════");
  Serial.printf("PKT:%05d | T:%8lums | %-6s | ALT:%7.1fm | LAT:%10.6f | LON:%11.6f | RSSI:%4ddBm | LOSS:%.1f%%\n",
    telemetry.packetId,
    telemetry.timestamp,
    getState(telemetry.droneState),
    telemetry.altitude,
    telemetry.latitude,
    telemetry.longitude,
    lastRssi,
    lossPercent);
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(84);
  
  if (esp_now_init() != ESP_OK) return;
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, satelliteMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    peerAdded = true;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n=== CANSAT GROUND STATION ===");
  setupESPNow();
  Serial.printf("ESP-NOW: %s\n", peerAdded ? "OK" : "FAIL");
  Serial.println("Waiting for satellite...\n");
}

void loop() {
  // Display at 2 Hz (every 500ms) - easy to read
  if (millis() - lastDisplayTime >= 500) {
    lastDisplayTime = millis();
    if (packetsReceived > 0) {
      printTelemetry();
    }
  }
  
  // Warning if no data
  if (packetsReceived > 0 && millis() - lastPacketTime > 3000) {
    Serial.println("⚠️ NO DATA FOR 3 SECONDS");
    lastPacketTime = millis();
  }
}
