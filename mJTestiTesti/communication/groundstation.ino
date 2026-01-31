/*
 * CanSat Ground Station - ESP32-WROOM-U
 * Compatible with ESP32 Arduino Core 3.3.5+ (ESP-IDF 5.5)
 */

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// ⚠️ REPLACE WITH YOUR SATELLITE MAC ADDRESS!
uint8_t satelliteMAC[] = {0x98, 0xA3, 0x16, 0xF8, 0x27, 0x40};

// Must match satellite structures exactly
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

TelemetryPacket receivedTelemetry;
CommandPacket commandToSend;

uint32_t packetsReceived = 0;
uint32_t packetsLost = 0;
uint16_t lastPacketId = 0;
unsigned long lastPacketTime = 0;

bool peerAdded = false;

const char* getDroneState(uint8_t state) {
  switch (state) {
    case 0: return "LAUNCH";
    case 1: return "ASCENT";
    case 2: return "APOGEE";
    case 3: return "DESCENT";
    case 4: return "DRONE_MODE";
    case 5: return "LANDED";
    default: return "UNKNOWN";
  }
}

// CORRECT CALLBACK for ESP-IDF 5.5+
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.printf("[TX] Command %s\n", 
                status == ESP_NOW_SEND_SUCCESS ? "delivered" : "failed");
}

// CORRECT CALLBACK for ESP-IDF 5.5+
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  lastPacketTime = millis();
  
  if (len == sizeof(TelemetryPacket)) {
    memcpy(&receivedTelemetry, data, sizeof(receivedTelemetry));
    
    // Track packet loss
    if (packetsReceived > 0) {
      uint16_t expectedId = lastPacketId + 1;
      if (receivedTelemetry.packetId > expectedId) {
        packetsLost += (receivedTelemetry.packetId - expectedId);
      }
    }
    lastPacketId = receivedTelemetry.packetId;
    packetsReceived++;
    
    printTelemetry();
  }
}

void printTelemetry() {
  // CSV format for logging
  Serial.printf("TELEM,%lu,%u,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s,%d,%d\n",
    receivedTelemetry.timestamp,
    receivedTelemetry.packetId,
    receivedTelemetry.altitude,
    receivedTelemetry.temperature,
    receivedTelemetry.pressure,
    receivedTelemetry.latitude,
    receivedTelemetry.longitude,
    receivedTelemetry.accelX,
    receivedTelemetry.accelY,
    receivedTelemetry.accelZ,
    receivedTelemetry.gyroX,
    receivedTelemetry.gyroY,
    receivedTelemetry.gyroZ,
    receivedTelemetry.batteryVoltage,
    getDroneState(receivedTelemetry.droneState),
    receivedTelemetry.gpsFixQuality,
    receivedTelemetry.rssi
  );
  
  // Human-readable format every 10 packets
  if (packetsReceived % 10 == 0) {
    Serial.println("\n--- Telemetry Summary ---");
    Serial.printf("Packets: %lu received, %lu lost (%.1f%% loss)\n", 
                  packetsReceived, packetsLost,
                  packetsReceived > 0 ? (float)packetsLost / (packetsReceived + packetsLost) * 100 : 0);
    Serial.printf("Altitude: %.2f m | State: %s | Battery: %.2f V\n",
                  receivedTelemetry.altitude,
                  getDroneState(receivedTelemetry.droneState),
                  receivedTelemetry.batteryVoltage);
    Serial.printf("Position: %.6f, %.6f | RSSI: %d dBm\n",
                  receivedTelemetry.latitude,
                  receivedTelemetry.longitude,
                  receivedTelemetry.rssi);
    Serial.println("-------------------------\n");
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
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, satelliteMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[ERROR] Failed to add peer");
  } else {
    peerAdded = true;
    Serial.println("[OK] Satellite peer added");
  }
}

void sendCommand(uint8_t cmdType, uint8_t p1, uint8_t p2) {
  if (!peerAdded) {
    Serial.println("[ERROR] No peer connected");
    return;
  }
  
  commandToSend.commandType = cmdType;
  commandToSend.param1 = p1;
  commandToSend.param2 = p2;
  commandToSend.timestamp = millis();
  
  esp_now_send(satelliteMAC, (uint8_t *)&commandToSend, sizeof(commandToSend));
  Serial.printf("[TX] Sending command: type=%d, p1=%d, p2=%d\n", cmdType, p1, p2);
}

void processSerialCommands() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    
    if (cmd == "deploy") {
      sendCommand(0x01, 0, 0);
    }
    else if (cmd.startsWith("motor ")) {
      int motor = cmd.substring(6, 7).toInt();
      int power = cmd.substring(8).toInt();
      sendCommand(0x02, motor, power);
    }
    else if (cmd.startsWith("mode ")) {
      int mode = cmd.substring(5).toInt();
      sendCommand(0x03, mode, 0);
    }
    else if (cmd == "status") {
      Serial.printf("\n=== Ground Station Status ===\n");
      Serial.printf("Packets received: %lu\n", packetsReceived);
      Serial.printf("Packets lost: %lu\n", packetsLost);
      Serial.printf("Last packet: %lu ms ago\n", millis() - lastPacketTime);
      Serial.println("=============================\n");
    }
    else if (cmd == "help") {
      Serial.println("\nAvailable commands:");
      Serial.println("  deploy        - Deploy drone");
      Serial.println("  motor <n> <p> - Test motor n at power p%");
      Serial.println("  mode <n>      - Set flight mode");
      Serial.println("  status        - Show connection status");
      Serial.println();
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n=== CanSat Ground Station ===");
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  
  setupESPNow();
  
  Serial.println("[OK] Ground station ready");
  Serial.println("Type 'help' for commands\n");
}

void loop() {
  processSerialCommands();
  
  if (packetsReceived > 0 && (millis() - lastPacketTime > 5000)) {
    Serial.println("[WARNING] No telemetry for 5 seconds");
    lastPacketTime = millis();
  }
  
  delay(10);
}
