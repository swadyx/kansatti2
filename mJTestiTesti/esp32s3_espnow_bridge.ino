// esp32s3_espnow_bridge.ino — ESP32-S3 Sense UART ↔ ESP-NOW Bridge
//
// Sits on the drone. Receives UART binary frames from the Flight Computer (FC)
// over Serial1, forwards them via ESP-NOW to the ground receiver.
// Also receives ESP-NOW packets from the ground and forwards them as UART
// frames to the FC.
//
// Wiring (XIAO ESP32-S3 Sense):
//   RXD1 = GPIO44, TXD1 = GPIO43  (matches existing esps3.ino pin assignment)
//   Baud = 460800 (must match fc_uart_link.ino)
//
// Frame format (shared with fc_uart_link.ino and ground_espnow_receiver.ino):
//   [0xAA] [0x55] [len] [type] [payload...] [XOR checksum]
//   - Header:   2 bytes 0xAA 0x55
//   - Length:   1 byte  (length of type + payload, NOT including header/checksum)
//   - Type:     1 byte  'T' = telemetry (FC→ground), 'C' = command (ground→FC)
//   - Payload:  variable bytes
//   - Checksum: 1 byte XOR of all bytes from len through end of payload
//
// Data flow:
//   FC ──(UART frame)──> [this ESP32-S3] ──(ESP-NOW)──> Ground receiver
//   FC <──(UART frame)── [this ESP32-S3] <──(ESP-NOW)── Ground receiver

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ===================== Configuration =====================

#define RXD1 44
#define TXD1 43
#define FC_BAUD 460800

// WiFi channel for ESP-NOW (must match ground receiver)
static constexpr uint8_t ESPNOW_CHANNEL = 1;

// Ground receiver MAC address — change this to your ground ESP32's MAC.
// Default: broadcast (works for initial testing, any ESP-NOW peer will receive)
static uint8_t GROUND_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Maximum ESP-NOW payload size
static constexpr size_t ESPNOW_MAX_LEN = 250;

// Maximum UART frame payload size
static constexpr size_t FRAME_BUF_LEN = 64;

// ===================== UART parser state machine =====================
enum UartParseState : uint8_t {
  UP_HEADER1,
  UP_HEADER2,
  UP_LEN,
  UP_PAYLOAD,
  UP_CHECKSUM
};

static UartParseState uartState = UP_HEADER1;
static uint8_t uartLen  = 0;
static uint8_t uartIdx  = 0;
static uint8_t uartCsum = 0;
static uint8_t uartBuf[FRAME_BUF_LEN];

// ===================== Frame builder helper =====================
// Build and write a complete frame to Serial1.
// 'data' already contains the type byte at data[0].
static void writeFrameToUart(const uint8_t* data, uint8_t dataLen) {
  uint8_t csum = dataLen;
  for (uint8_t i = 0; i < dataLen; i++) csum ^= data[i];

  Serial1.write(0xAA);
  Serial1.write(0x55);
  Serial1.write(dataLen);
  Serial1.write(data, dataLen);
  Serial1.write(csum);
}

// ===================== ESP-NOW callbacks =====================

// Called when ESP-NOW send completes (used for diagnostics only)
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[BRIDGE] ESP-NOW send FAILED");
  }
}

// Called when an ESP-NOW packet arrives from the ground receiver.
// Wrap the payload in a UART frame and forward to the FC.
static void onDataRecv(const esp_now_recv_info_t* info,
                       const uint8_t* data, int len) {
  if (len <= 0 || len > (int)FRAME_BUF_LEN) {
    Serial.printf("[BRIDGE] ESP-NOW RX: bad len %d\n", len);
    return;
  }

  Serial.printf("[BRIDGE] ESP-NOW RX %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n",
    len,
    info->src_addr[0], info->src_addr[1], info->src_addr[2],
    info->src_addr[3], info->src_addr[4], info->src_addr[5]);

  // Forward to FC as a UART frame (data already contains type + payload)
  writeFrameToUart(data, (uint8_t)len);
}

// ===================== Setup =====================
void setup() {
  Serial.begin(115200);
  Serial.println("[BRIDGE] ESP32-S3 UART↔ESP-NOW bridge starting...");

  // Serial1 to Flight Computer
  Serial1.begin(FC_BAUD, SERIAL_8N1, RXD1, TXD1);
  Serial.printf("[BRIDGE] Serial1 ready at %d baud (RX=%d TX=%d)\n",
    FC_BAUD, RXD1, TXD1);

  // WiFi in station mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  Serial.print("[BRIDGE] MAC: ");
  Serial.println(WiFi.macAddress());

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[BRIDGE] ESP-NOW init FAILED — halting");
    while (true) {}
  }

  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Register ground receiver as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, GROUND_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[BRIDGE] Failed to add ground peer — halting");
    while (true) {}
  }

  Serial.println("[BRIDGE] Ready.");
}

// ===================== Loop =====================
void loop() {
  // --- UART → ESP-NOW (telemetry from FC) ---
  while (Serial1.available()) {
    uint8_t b = (uint8_t)Serial1.read();

    switch (uartState) {
      case UP_HEADER1:
        if (b == 0xAA) uartState = UP_HEADER2;
        break;

      case UP_HEADER2:
        uartState = (b == 0x55) ? UP_LEN : UP_HEADER1;
        break;

      case UP_LEN:
        uartLen  = b;
        uartIdx  = 0;
        uartCsum = b;
        if (uartLen == 0 || uartLen > FRAME_BUF_LEN) {
          uartState = UP_HEADER1;
        } else {
          uartState = UP_PAYLOAD;
        }
        break;

      case UP_PAYLOAD:
        uartBuf[uartIdx++] = b;
        uartCsum ^= b;
        if (uartIdx >= uartLen) {
          uartState = UP_CHECKSUM;
        }
        break;

      case UP_CHECKSUM:
        if (b == uartCsum) {
          // Valid frame from FC — forward payload via ESP-NOW
          // Payload includes type byte at uartBuf[0]
          Serial.printf("[BRIDGE] UART RX type='%c' len=%d → ESP-NOW\n",
            (char)uartBuf[0], uartLen);

          esp_err_t result = esp_now_send(GROUND_MAC, uartBuf, uartLen);
          if (result != ESP_OK) {
            Serial.printf("[BRIDGE] esp_now_send error: %d\n", result);
          }
        } else {
          Serial.println("[BRIDGE] UART frame checksum error — dropped");
        }
        uartState = UP_HEADER1;
        break;
    }
  }
}
