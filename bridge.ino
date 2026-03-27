#include <esp_now.h>
#include <WiFi.h>
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

// ===================== STATE =====================
static esp_now_peer_info_t peerInfo{};
static Proto::Parser fcParser;
static uint16_t txSeq = 1;
static bool espNowReady = false;

// No camera / no SD on this version
static constexpr bool cameraReady = false;
static constexpr bool sdReady = false;

// kept only so protocol behavior stays simple
static bool continuousCapture = false;
static int photoCount = 0;
static unsigned long lastCaptureTime = 0;
static unsigned long captureIntervalMs = 2000;

// ===================== HELPERS =====================
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

// This board version has no camera
static bool takePhoto() {
  return false;
}

static bool handleBridgeCommand(const Proto::Packet &pkt, const Proto::CommandPayload &cmd) {
  switch (cmd.cmd_id) {
    case Proto::CMD_PING:
      sendBridgeAckToGs(pkt.header.seq, Proto::ACK_OK);
      sendBridgeInfoToGs(Proto::INFO_PING_REPLY, 5678);
      return true;

    case Proto::CMD_START_CAMERA:
      continuousCapture = false;
      sendBridgeAckToGs(pkt.header.seq, Proto::ACK_INTERNAL_ERR);
      return true;

    case Proto::CMD_STOP_CAMERA:
      continuousCapture = false;
      sendBridgeAckToGs(pkt.header.seq, Proto::ACK_OK);
      return true;

    case Proto::CMD_TAKE_PHOTO: {
      const bool ok = takePhoto();
      sendBridgeAckToGs(pkt.header.seq, ok ? Proto::ACK_OK : Proto::ACK_INTERNAL_ERR);
      if (ok) {
        sendBridgeEventToGs(Proto::EVT_PHOTO_TAKEN, photoCount);
      }
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

  // Left here so protocol flow stays compatible,
  // but this hardware has no camera.
  if (evt.event_id == Proto::EVT_LAUNCH_DETECTED) {
    continuousCapture = false;
  } else if (evt.event_id == Proto::EVT_LANDED) {
    continuousCapture = false;
  }
}

static void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  (void)info;

  Proto::Packet pkt{};
  if (!Proto::decodePacket(data, (uint16_t)len, pkt)) {
    DBG_PRINTF("[BR] Bad ESP-NOW packet len=%d\n", len);
    return;
  }

  DBG_PRINTF("[BR] RX GS type=%s seq=%u\n",
             Proto::messageTypeName(pkt.header.type),
             pkt.header.seq);

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
  DBG_PRINTF("[BR] ESP-NOW TX %s\n",
             status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
  DBG_BEGIN(115200);
  Serial1.begin(FC_UART_BAUD, SERIAL_8N1, RXD1, TXD1);

  DBG_PRINTLN("[BR] XIAO ESP32S3 bridge build");
  DBG_PRINTLN("[BR] Camera: NOT PRESENT");
  DBG_PRINTLN("[BR] SD: NOT PRESENT");

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
  sendBridgeInfoToGs(Proto::INFO_BRIDGE_CAM_STATUS, 0);
  sendBridgeInfoToGs(Proto::INFO_BRIDGE_SD_STATUS, 0);
}

void loop() {
  Proto::Packet pkt{};

  while (Proto::readPacket(Serial1, fcParser, pkt)) {
    DBG_PRINTF("[BR] RX FC type=%s seq=%u\n",
               Proto::messageTypeName(pkt.header.type),
               pkt.header.seq);

    if (pkt.header.type == Proto::MSG_EVENT) {
      handleFcEvent(pkt);
    }

    forwardPacketToGs(pkt);
  }

  // No camera on this board, so no periodic capture.
  if (continuousCapture && (millis() - lastCaptureTime >= captureIntervalMs)) {
    lastCaptureTime = millis();
  }
}
