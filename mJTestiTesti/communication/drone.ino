/*
 * CanSat Satellite - ESP32-S3 Sense
 * Compatible with ESP32 Arduino Core 3.3.5+ (ESP-IDF 5.5)
 */

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// Ground station MAC address
uint8_t groundStationMAC[] = {0xD4, 0xD4, 0xDA, 0x5A, 0x5A, 0x74};

// Telemetry data structure
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

// Command structure (from ground to satellite)
typedef struct __attribute__((packed)) {
  uint8_t commandType;
  uint8_t param1;
  uint8_t param2;
  uint32_t timestamp;
} CommandPacket;

TelemetryPacket telemetry;
CommandPacket receivedCommand;

uint16_t packetCounter = 0;
unsigned long lastSendTime = 0;
const int SEND_INTERVAL_MS = 100;

bool peerAdded = false;

// CORRECT CALLBACK for ESP-IDF 5.5 / Arduino Core 3.3.5+
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[TX] Delivery failed");
  }
}

// CORRECT CALLBACK for ESP-IDF 5.5 / Arduino Core 3.3.5+
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(CommandPacket)) {
    memcpy(&receivedCommand, data, sizeof(receivedCommand));
    
    Serial.printf("[RX] Command received: type=%d, param1=%d, param2=%d\n",
                  receivedCommand.commandType,
                  receivedCommand.param1,
                  receivedCommand.param2);
    
    processCommand(receivedCommand);
  }
}

void processCommand(CommandPacket &cmd) {
  switch (cmd.commandType) {
    case 0x01:
      Serial.println("[CMD] Deploy command received!");
      break;
    case 0x02:
      Serial.printf("[CMD] Motor test: motor %d, power %d%%\n", cmd.param1, cmd.param2);
      break;
    case 0x03:
      Serial.printf("[CMD] Set mode: %d\n", cmd.param1);
      break;
    default:
      Serial.println("[CMD] Unknown command");
  }
}

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(84);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init failed!");
    return;
  }
  
  // Register callbacks with correct signatures
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, groundStationMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add peer");
  } else {
    peerAdded = true;
    Serial.println("[OK] Ground station peer added");
  }
}

void updateTelemetry() {
  telemetry.timestamp = millis();
  telemetry.packetType = 0x01;
  telemetry.packetId = packetCounter++;
  
  // Replace with actual sensor readings
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
  telemetry.droneState = 0;
  telemetry.gpsFixQuality = 0;
  telemetry.rssi = 0;
}

void sendTelemetry() {
  if (!peerAdded) return;
  
  esp_err_t result = esp_now_send(groundStationMAC, (uint8_t *)&telemetry, sizeof(telemetry));
  
  if (result == ESP_OK) {
    Serial.printf("[TX] Packet #%d sent\n", telemetry.packetId);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== CanSat Satellite ESP-NOW ===");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  setupESPNow();
  
  Serial.println("[OK] Satellite ready");
}

void loop() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = currentTime;
    updateTelemetry();
    sendTelemetry();
  }
}
