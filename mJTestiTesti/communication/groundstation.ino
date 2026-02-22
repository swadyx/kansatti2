/*
 * CanSat Ground Station - ESP32-WROOM
 * Receives telemetry, sends commands & text messages
 */

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"

// Drone MAC
uint8_t droneMAC[] = {0x98, 0xA3, 0x16, 0xF8, 0x27, 0x40};

// ===== MUST MATCH DRONE STRUCTS EXACTLY =====
typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  uint8_t  packetType;
  uint16_t packetId;
  float    altitude;
  float    temperature;
  float    pressure;
  float    latitude;
  float    longitude;
  float    accelX, accelY, accelZ;
  float    gyroX, gyroY, gyroZ;
  float    batteryVoltage;
  uint8_t  droneState;
  uint8_t  gpsFixQuality;
  int8_t   rssi;
  uint32_t photoCount;
  uint8_t  sdOK;
  uint8_t  camOK;
} TelemetryPacket;

typedef struct __attribute__((packed)) {
  uint8_t  packetType;     // 0x10
  uint8_t  commandType;
  uint8_t  param1;
  uint8_t  param2;
  uint32_t timestamp;
} CommandPacket;

TelemetryPacket telem;

uint32_t packetsReceived = 0;
uint32_t packetsLost = 0;
uint16_t lastPacketId = 0;
unsigned long lastPacketTime = 0;
int8_t lastRssi = 0;
bool peerAdded = false;

// ===== STATE NAME =====
const char* stateName(uint8_t s) {
  switch (s) {
    case 0: return "OFF";
    case 1: return "IDLE";
    case 2: return "LAUNCH";
    case 3: return "PAUSED";
    case 4: return "MOTOR_TEST";
    default: return "???";
  }
}

// ===== CALLBACKS =====
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.printf("[TX] %s\n", status == ESP_NOW_SEND_SUCCESS ? "Delivered" : "FAILED");
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  lastPacketTime = millis();
  lastRssi = info->rx_ctrl->rssi;

  if (len == sizeof(TelemetryPacket) && data[4] == 0x01) {
    memcpy(&telem, data, sizeof(telem));

    // Packet loss tracking
    if (packetsReceived > 0 && telem.packetId > lastPacketId + 1) {
      packetsLost += (telem.packetId - lastPacketId - 1);
    }
    lastPacketId = telem.packetId;
    packetsReceived++;

    // Print every 4th packet (0.5 Hz display)
    if (packetsReceived % 4 == 0) {
      printTelemetry();
    }
  }
}

void printTelemetry() {
  float loss = (packetsReceived + packetsLost) > 0 
    ? (float)packetsLost / (packetsReceived + packetsLost) * 100.0 : 0;

  Serial.println("═══════════════════════════════════════════════════════════════════");
  Serial.printf("PKT:%05d | T:%8lums | %-10s | RSSI:%4ddBm | LOSS:%.1f%% | RX:%lu\n",
    telem.packetId, telem.timestamp, stateName(telem.droneState),
    lastRssi, loss, packetsReceived);
  Serial.printf("ALT:%7.1fm | LAT:%11.6f | LON:%11.6f | TEMP:%.1fC | PRES:%.1fhPa\n",
    telem.altitude, telem.latitude, telem.longitude,
    telem.temperature, telem.pressure);
  Serial.printf("ACC: X=%.2f Y=%.2f Z=%.2f | GYRO: X=%.2f Y=%.2f Z=%.2f\n",
    telem.accelX, telem.accelY, telem.accelZ,
    telem.gyroX, telem.gyroY, telem.gyroZ);
  Serial.printf("BATT:%.2fV | GPS:%d | PHOTOS:%lu | SD:%s | CAM:%s\n",
    telem.batteryVoltage, telem.gpsFixQuality, telem.photoCount,
    telem.sdOK ? "OK" : "X", telem.camOK ? "OK" : "X");
}

// ===== SEND COMMAND =====
void sendCommand(uint8_t cmdType, uint8_t p1, uint8_t p2) {
  if (!peerAdded) { Serial.println("[ERR] No peer"); return; }

  CommandPacket cmd;
  cmd.packetType  = 0x10;
  cmd.commandType = cmdType;
  cmd.param1      = p1;
  cmd.param2      = p2;
  cmd.timestamp   = millis();

  esp_now_send(droneMAC, (uint8_t *)&cmd, sizeof(cmd));
  Serial.printf("[TX] Command 0x%02X p1=%d p2=%d\n", cmdType, p1, p2);
}

// ===== SEND TEXT MESSAGE =====
void sendMessage(const char *msg) {
  if (!peerAdded) { Serial.println("[ERR] No peer"); return; }

  // Build packet: [0x20][string bytes]
  int msgLen = strlen(msg);
  if (msgLen > 199) msgLen = 199;

  uint8_t buf[201];
  buf[0] = 0x20;  // packetType = text message
  memcpy(buf + 1, msg, msgLen);
  buf[msgLen + 1] = '\0';

  esp_now_send(droneMAC, buf, msgLen + 2);
  Serial.printf("[TX] Message: \"%s\"\n", msg);
}

// ===== ESP-NOW SETUP =====
void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(84);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, droneMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  peerAdded = (esp_now_add_peer(&peer) == ESP_OK);
  Serial.printf("[ESP-NOW] Peer: %s\n", peerAdded ? "OK" : "FAIL");
}

// ===== SERIAL COMMAND PARSER =====
void processSerial() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  // Commands (start with /)
  if (input.startsWith("/")) {
    String cmd = input.substring(1);

    if (cmd == "on") {
      sendCommand(0x01, 0, 0);
    }
    else if (cmd == "off") {
      sendCommand(0x02, 0, 0);
    }
    else if (cmd == "launch") {
      sendCommand(0x03, 0, 0);
    }
    else if (cmd == "pause") {
      sendCommand(0x04, 0, 0);
    }
    else if (cmd.startsWith("motor ")) {
      // /motor 1 50  →  motor 1 at 50%
      int space = cmd.indexOf(' ', 6);
      if (space > 0) {
        int motor = cmd.substring(6, space).toInt();
        int power = cmd.substring(space + 1).toInt();
        sendCommand(0x05, motor, power);
      } else {
        Serial.println("Usage: /motor <n> <power%>");
      }
    }
    else if (cmd == "status") {
      Serial.printf("\n--- STATUS ---\n");
      Serial.printf("RX: %lu | Lost: %lu | Last: %lums ago\n",
        packetsReceived, packetsLost, millis() - lastPacketTime);
      Serial.printf("--------------\n\n");
    }
    else if (cmd == "help") {
      Serial.println("\n=== COMMANDS (prefix with /) ===");
      Serial.println("  /on              Power on drone");
      Serial.println("  /off             Power off drone");
      Serial.println("  /launch          Launch sequence");
      Serial.println("  /pause           Pause drone");
      Serial.println("  /motor <n> <p>   Test motor n at p%");
      Serial.println("  /status          Connection status");
      Serial.println("  /help            This help");
      Serial.println("\n=== TEXT MESSAGES ===");
      Serial.println("  Just type anything without / prefix");
      Serial.println("  It will print on drone's Serial console\n");
    }
    else {
      Serial.printf("Unknown command: /%s (type /help)\n", cmd.c_str());
    }
  }
  else {
    // Plain text → send as debug message to drone
    sendMessage(input.c_str());
  }
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n============================");
  Serial.println("  CANSAT GROUND STATION v2.0");
  Serial.println("============================");
  Serial.printf("MAC: %s\n", WiFi.macAddress().c_str());

  setupESPNow();

  Serial.println("\nType /help for commands");
  Serial.println("Type anything else to send text to drone\n");
}

// ===== LOOP =====
void loop() {
  processSerial();

  // Warn if no data
  if (packetsReceived > 0 && millis() - lastPacketTime > 5000) {
    Serial.println("[WARN] No telemetry for 5 seconds");
    lastPacketTime = millis();
  }

  delay(10);
}
