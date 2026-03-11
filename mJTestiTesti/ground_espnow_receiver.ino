// ground_espnow_receiver.ino — Ground ESP32 Receiver
//
// Runs on a second ESP32 (any variant) connected to the ground station PC
// via USB serial. Acts as a transparent bridge between the drone's ESP32-S3
// (via ESP-NOW) and the Python ground station app (via USB serial).
//
// Frame format (shared with fc_uart_link.ino and esp32s3_espnow_bridge.ino):
//   [0xAA] [0x55] [len] [type] [payload...] [XOR checksum]
//   - Header:   2 bytes 0xAA 0x55
//   - Length:   1 byte  (length of type + payload, NOT including header/checksum)
//   - Type:     1 byte  'T' = telemetry (FC→ground), 'C' = command (ground→FC)
//   - Payload:  variable bytes
//   - Checksum: 1 byte XOR of all bytes from len through end of payload
//
// Data flow:
//   Drone ESP32-S3 ──(ESP-NOW)──> [this ground ESP32] ──(USB Serial)──> Python app
//   Drone ESP32-S3 <──(ESP-NOW)── [this ground ESP32] <──(USB Serial)── Python app

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

// ===================== Configuration =====================

// WiFi channel for ESP-NOW (must match esp32s3_espnow_bridge.ino)
static constexpr uint8_t ESPNOW_CHANNEL = 1;

// Drone ESP32-S3 MAC address — change this to the drone bridge MAC.
// Run the bridge sketch and read its MAC from the Serial monitor output,
// then set it here. Default: broadcast for easy initial testing.
static uint8_t DRONE_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// USB serial baud rate to Python ground station app
static constexpr uint32_t USB_BAUD = 115200;

// Maximum frame payload buffer size
static constexpr size_t FRAME_BUF_LEN = 64;

// ===================== Frame builder helper =====================
// Build and write a complete UART frame to Serial (USB).
// 'data' already contains the type byte at data[0].
static void writeFrameToUsb(const uint8_t* data, uint8_t dataLen) {
  uint8_t csum = dataLen;
  for (uint8_t i = 0; i < dataLen; i++) csum ^= data[i];

  Serial.write(0xAA);
  Serial.write(0x55);
  Serial.write(dataLen);
  Serial.write(data, dataLen);
  Serial.write(csum);
}

// ===================== USB serial parser state machine =====================
// Reads command frames from the Python app and forwards via ESP-NOW.
enum UartParseState : uint8_t {
  UP_HEADER1,
  UP_HEADER2,
  UP_LEN,
  UP_PAYLOAD,
  UP_CHECKSUM
};

static UartParseState usbState = UP_HEADER1;
static uint8_t usbLen  = 0;
static uint8_t usbIdx  = 0;
static uint8_t usbCsum = 0;
static uint8_t usbBuf[FRAME_BUF_LEN];

// ===================== ESP-NOW callbacks =====================

// Called when we receive an ESP-NOW packet from the drone ESP32-S3.
// Wrap in a frame and forward to USB (Python app).
static void onDataRecv(const esp_now_recv_info_t* info,
                       const uint8_t* data, int len) {
  if (len <= 0 || len > (int)FRAME_BUF_LEN) {
    // Log to USB would corrupt the framed stream — use a different indicator
    // (could use a separate debug channel if needed)
    return;
  }

  // Forward raw framed telemetry to Python app
  writeFrameToUsb(data, (uint8_t)len);

  // Optional: print RSSI if available (ESP32 API provides it via rssi field)
  // Note: rssi is in the esp_now_recv_info_t struct on ESP-IDF v5+
  // Uncomment below if your ESP-IDF version supports it:
  // Serial2.printf("[GND] RSSI: %d dBm\n", info->rx_ctrl->rssi);
  (void)info;
}

// Called when ESP-NOW send completes
static void onDataSent(const uint8_t* mac, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    // Cannot print to Serial here (it's the framed data channel)
    // Use a separate debug UART if needed
  }
}

// ===================== Setup =====================
void setup() {
  Serial.begin(USB_BAUD);

  // WiFi in station mode (required for ESP-NOW)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    // Fatal — blink LED or halt
    while (true) { delay(100); }
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  // Register drone ESP32-S3 as a peer
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, DRONE_MAC, 6);
  peerInfo.channel = ESPNOW_CHANNEL;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    while (true) { delay(100); }
  }
}

// ===================== Loop =====================
void loop() {
  // --- USB Serial → ESP-NOW (commands from Python app to drone) ---
  while (Serial.available()) {
    uint8_t b = (uint8_t)Serial.read();

    switch (usbState) {
      case UP_HEADER1:
        if (b == 0xAA) usbState = UP_HEADER2;
        break;

      case UP_HEADER2:
        usbState = (b == 0x55) ? UP_LEN : UP_HEADER1;
        break;

      case UP_LEN:
        usbLen  = b;
        usbIdx  = 0;
        usbCsum = b;
        if (usbLen == 0 || usbLen > FRAME_BUF_LEN) {
          usbState = UP_HEADER1;
        } else {
          usbState = UP_PAYLOAD;
        }
        break;

      case UP_PAYLOAD:
        usbBuf[usbIdx++] = b;
        usbCsum ^= b;
        if (usbIdx >= usbLen) {
          usbState = UP_CHECKSUM;
        }
        break;

      case UP_CHECKSUM:
        if (b == usbCsum) {
          // Valid frame from Python app — forward payload via ESP-NOW
          // Payload includes type byte at usbBuf[0]
          esp_now_send(DRONE_MAC, usbBuf, usbLen);
        }
        // Silently drop bad frames (can't print without corrupting stream)
        usbState = UP_HEADER1;
        break;
    }
  }
}
