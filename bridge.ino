#include <esp_now.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "protocol.h"

// ===================== DEBUG =====================
#define BRIDGE_DEBUG 1
#if BRIDGE_DEBUG
  #define DBG_BEGIN(...) Serial.begin(__VA_ARGS__)
  #define DBG_PRINT(...) Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...) Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...) Serial.printf(__VA_ARGS__)
#else
  #define DBG_BEGIN(...)
  #define DBG_PRINT(...)
  #define DBG_PRINTLN(...)
  #define DBG_PRINTF(...)
#endif

// ===================== CONFIG =====================
uint8_t groundStationMAC[] = { 0xD4, 0xD4, 0xDA, 0x5A, 0x5A, 0x74 };
static constexpr uint32_t FC_UART_BAUD = 921600;
#define RXD1 44
#define TXD1 43

#define PWDN_GPIO_NUM  -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM  10
#define SIOD_GPIO_NUM  40
#define SIOC_GPIO_NUM  39
#define Y9_GPIO_NUM    48
#define Y8_GPIO_NUM    11
#define Y7_GPIO_NUM    12
#define Y6_GPIO_NUM    14
#define Y5_GPIO_NUM    16
#define Y4_GPIO_NUM    18
#define Y3_GPIO_NUM    17
#define Y2_GPIO_NUM    15
#define VSYNC_GPIO_NUM 38
#define HREF_GPIO_NUM  47
#define PCLK_GPIO_NUM  13
#define SD_CS          21

static esp_now_peer_info_t peerInfo{};
static Proto::Parser fcParser;
static uint16_t txSeq = 1;
static bool cameraReady = false;
static bool sdReady = false;
static bool espNowReady = false;
static bool continuousCapture = false;
static int photoCount = 0;
static unsigned long lastCaptureTime = 0;
static unsigned long captureIntervalMs = 2000;

static uint16_t nextSeq() {
  return txSeq++;
}

static bool sendEspNowPacket(const Proto::Packet &pkt) {
  if (!espNowReady) return false;
  uint8_t frame[Proto::MAX_FRAME_LEN];
  const int len = Proto::encodePacket(pkt, frame, sizeof(frame));
  if (len <= 0) return false;
  const esp_err_t err = esp_now_send(groundStationMAC, frame, len);
  return err == ESP_OK;
}

static void sendBridgeAckToGs(uint16_t ackedSeq, uint8_t status, uint8_t detail = 0) {
  Proto::AckPayload ack{};
  ack.acked_seq = ackedSeq;
  ack.status = status;
  ack.detail = detail;
  Proto::Packet pkt{};
  if (Proto::initPacket(pkt, Proto::MSG_ACK, Proto::FLAG_IS_RESPONSE, nextSeq(), ack)) {
    sendEspNowPacket(pkt);
  }
}

static void sendBridgeEventToGs(uint8_t eventId, int32_t value = 0) {
  Proto::EventPayload p{};
  p.event_id = eventId;
  p.source = Proto::SRC_BRIDGE;
  p.value = value;
  Proto::Packet pkt{};
  if (Proto::initPacket(pkt, Proto::MSG_EVENT, Proto::FLAG_IMPORTANT, nextSeq(), p)) {
    sendEspNowPacket(pkt);
  }
}

static void sendBridgeInfoToGs(uint8_t code, int32_t value = 0) {
  Proto::InfoPayload p{};
  p.source = Proto::SRC_BRIDGE;
  p.code = code;
  p.value = value;
  Proto::Packet pkt{};
  if (Proto::initPacket(pkt, Proto::MSG_INFO, 0, nextSeq(), p)) {
    sendEspNowPacket(pkt);
  }
}

static bool initCamera() {
  camera_config_t config{};
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

  const esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    DBG_PRINTF("[BR] Camera init failed: 0x%x\n", (unsigned)err);
    return false;
  }
  return true;
}

static bool initSD() {
  if (!SD.begin(SD_CS)) return false;
  if (SD.cardType() == CARD_NONE) return false;
  return true;
}

static bool takePhoto() {
  if (!cameraReady || !sdReady) return false;
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
  esp_camera_fb_return(fb);
  photoCount++;
  DBG_PRINTF("[BR] Saved %s\n", fn);
  return true;
}

static bool handleBridgeCommand(const Proto::Packet &pkt, const Proto::CommandPayload &cmd) {
  switch (cmd.cmd_id) {
    case Proto::CMD_PING:
      sendBridgeAckToGs(pkt.header.seq, Proto::ACK_OK);
      sendBridgeInfoToGs(Proto::INFO_PING_REPLY, 5678);
      return true;

    case Proto::CMD_START_CAMERA:
      continuousCapture = true;
      sendBridgeAckToGs(pkt.header.seq, Proto::ACK_OK);
      sendBridgeEventToGs(Proto::EVT_CAMERA_STARTED, 0);
      return true;

    case Proto::CMD_STOP_CAMERA:
      continuousCapture = false;
      sendBridgeAckToGs(pkt.header.seq, Proto::ACK_OK);
      sendBridgeEventToGs(Proto::EVT_CAMERA_STOPPED, 0);
      return true;

    case Proto::CMD_TAKE_PHOTO: {
      const bool ok = takePhoto();
      sendBridgeAckToGs(pkt.header.seq, ok ? Proto::ACK_OK : Proto::ACK_INTERNAL_ERR);
      if (ok) sendBridgeEventToGs(Proto::EVT_PHOTO_TAKEN, photoCount);
      return true;
    }

    default:
      sendBridgeAckToGs(pkt.header.seq, Proto::ACK_UNKNOWN_CMD);
      return true;
  }
}

static void forwardPacketToFc(const Proto::Packet &pkt) {
  Proto::writePacket(Serial1, pkt);
}

static void forwardPacketToGs(const Proto::Packet &pkt) {
  sendEspNowPacket(pkt);
}

static void handleFcEvent(const Proto::Packet &pkt) {
  Proto::EventPayload evt{};
  if (!Proto::payloadAs(pkt, evt)) return;

  if (evt.event_id == Proto::EVT_LAUNCH_DETECTED) {
    continuousCapture = true;
  } else if (evt.event_id == Proto::EVT_LANDED) {
    continuousCapture = false;
  }
}

static void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  Proto::Packet pkt{};
  if (!Proto::decodePacket(data, (uint16_t)len, pkt)) {
    DBG_PRINTF("[BR] Bad ESP-NOW packet len=%d\n", len);
    return;
  }

  DBG_PRINTF("[BR] RX GS type=%s seq=%u\n", Proto::messageTypeName(pkt.header.type), pkt.header.seq);

  if (pkt.header.type != Proto::MSG_COMMAND) {
    sendBridgeAckToGs(pkt.header.seq, Proto::ACK_UNKNOWN_TYPE);
    return;
  }

  Proto::CommandPayload cmd{};
  if (!Proto::payloadAs(pkt, cmd)) {
    sendBridgeAckToGs(pkt.header.seq, Proto::ACK_BAD_LENGTH);
    return;
  }

  if (cmd.dest == Proto::DEST_BRIDGE) {
    handleBridgeCommand(pkt, cmd);
  } else if (cmd.dest == Proto::DEST_FC || cmd.dest == Proto::DEST_ALL) {
    forwardPacketToFc(pkt);
  } else {
    sendBridgeAckToGs(pkt.header.seq, Proto::ACK_BAD_DEST);
  }
}

static void onEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  DBG_PRINTF("[BR] ESP-NOW TX %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  DBG_BEGIN(115200);
  Serial1.begin(FC_UART_BAUD, SERIAL_8N1, RXD1, TXD1);

  cameraReady = initCamera();
  sdReady = initSD();

  DBG_PRINTF("[BR] Camera init: %s\n", cameraReady ? "OK" : "FAIL");
  DBG_PRINTF("[BR] SD init: %s\n", sdReady ? "OK" : "FAIL");

  WiFi.mode(WIFI_STA);
  if (esp_now_init() == ESP_OK) {
    espNowReady = true;
    esp_now_register_recv_cb(onEspNowRecv);
    esp_now_register_send_cb(onEspNowSent);

    memcpy(peerInfo.peer_addr, groundStationMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      DBG_PRINTLN("[BR] Failed to add GS peer");
      espNowReady = false;
    }
  } else {
    DBG_PRINTLN("[BR] ESP-NOW init failed, continuing anyway");
    espNowReady = false;
  }

  DBG_PRINTF("[BR] ESP-NOW: %s\n", espNowReady ? "OK" : "FAIL");
  DBG_PRINTLN("[BR] ready");

  sendBridgeInfoToGs(Proto::INFO_BRIDGE_BOOT, 0);
  sendBridgeInfoToGs(Proto::INFO_BRIDGE_CAM_STATUS, cameraReady ? 1 : 0);
  sendBridgeInfoToGs(Proto::INFO_BRIDGE_SD_STATUS, sdReady ? 1 : 0);
}

void loop() {
  Proto::Packet pkt{};
  while (Proto::readPacket(Serial1, fcParser, pkt)) {
    DBG_PRINTF("[BR] RX FC type=%s seq=%u\n", Proto::messageTypeName(pkt.header.type), pkt.header.seq);
    if (pkt.header.type == Proto::MSG_EVENT) {
      handleFcEvent(pkt);
    }
    forwardPacketToGs(pkt);
  }

  if (continuousCapture && (millis() - lastCaptureTime >= captureIntervalMs)) {
    lastCaptureTime = millis();
    if (takePhoto()) {
      sendBridgeEventToGs(Proto::EVT_PHOTO_TAKEN, photoCount);
    }
  }
}
