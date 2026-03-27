#include <esp_now.h>
#include <WiFi.h>
#include "protocol.h"

// ===================== DEBUG =====================
#define GS_DEBUG 1
#if GS_DEBUG
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

uint8_t bridgeMAC[] = { 0x1C, 0xDB, 0xD4, 0x5B, 0x0C, 0xD8 };
//  1C:DB:D4:5B:0C:D8
static esp_now_peer_info_t peerInfo{};
static uint16_t txSeq = 1;

static uint16_t nextSeq() {
  return txSeq++;
}

static bool sendPacketToBridge(const Proto::Packet &pkt) {
  uint8_t frame[Proto::MAX_FRAME_LEN];
  const int len = Proto::encodePacket(pkt, frame, sizeof(frame));
  if (len <= 0) return false;
  return esp_now_send(bridgeMAC, frame, len) == ESP_OK;
}

static bool sendCommand(uint8_t dest, uint8_t cmdId, int32_t arg0 = 0, int32_t arg1 = 0, uint8_t flags = 0) {
  Proto::CommandPayload c{};
  c.dest = dest;
  c.cmd_id = cmdId;
  c.arg0 = arg0;
  c.arg1 = arg1;

  Proto::Packet pkt{};
  if (!Proto::initPacket(pkt, Proto::MSG_COMMAND, flags, nextSeq(), c)) return false;
  const bool ok = sendPacketToBridge(pkt);

  DBG_PRINTF("[GS] TX cmd=%s dest=%u seq=%u arg0=%ld arg1=%ld\n",
             Proto::commandName(cmdId), dest, pkt.header.seq, (long)arg0, (long)arg1);
  return ok;
}

static void printStatusFlags(uint16_t flags) {
  Serial.printf("Flags     : 0x%04X", flags);
  if (flags == 0) { Serial.println(" (none)"); return; }
  Serial.print(" (");
  bool first = true;
  auto addFlag = [&](const char *name) {
    if (!first) Serial.print(", ");
    Serial.print(name);
    first = false;
  };

  if (flags & Proto::STATUSF_INIT_FAILED)   addFlag("INIT_FAILED");
  if (flags & Proto::STATUSF_FLASH_PRESENT) addFlag("FLASH_PRESENT");
  if (flags & Proto::STATUSF_GPS_FIX)       addFlag("GPS_FIX");
  if (flags & Proto::STATUSF_HAS_TARGET)    addFlag("HAS_TARGET");

  Serial.println(")");
}

static void printStatus(const Proto::StatusPayload &s, uint16_t seq) {
  Serial.println("------------------------------");
  Serial.printf("STATUS seq: %u\n", seq);
  Serial.printf("State     : %s\n", Proto::stateName(s.state));
  Serial.printf("Armed     : %u\n", s.armed);
  Serial.printf("GPS fix   : %u\n", s.gps_fix);
  Serial.printf("LiPo      : %.3f V\n", s.lipo_mV / 1000.0f);
  printStatusFlags(s.flags);
  Serial.println("------------------------------");
}

static void printAck(const Proto::AckPayload &a, uint16_t seq) {
  Serial.printf("ACK seq=%u acked=%u status=%s", seq, a.acked_seq, Proto::ackStatusName(a.status));
  if (a.detail != 0) {
    Serial.printf(" detail=%u", a.detail);
    const char *txt = ackDetailText(a.status, a.detail);
    if (txt) Serial.printf(" (%s)", txt);
  }
  Serial.println();
}

static void printEvent(const Proto::EventPayload &e, uint16_t seq) {
  Serial.printf("EVENT seq=%u name=%s src=%s value=%ld\n",
                seq, Proto::eventName(e.event_id), Proto::sourceName(e.source), (long)e.value);
}

static void printInfo(const Proto::InfoPayload &i, uint16_t seq, const char *label) {
  Serial.printf("%s seq=%u src=%s code=%s value=%ld",
                label, seq, Proto::sourceName(i.source), Proto::infoCodeName(i.code), (long)i.value);
  switch (i.code) {
    case Proto::INFO_TEST_BARO:
      Serial.printf(" (%.1f Pa)", (double)i.value);
      break;
    case Proto::INFO_TEST_THERMISTOR:
      Serial.printf(" (%.2f C)", i.value / 100.0);
      break;
    case Proto::INFO_TEST_LDR:
      Serial.printf(" (%.3f V)", i.value / 1000.0);
      break;
    case Proto::INFO_FLASH_ADDR:
      Serial.printf(" (0x%08lX)", (unsigned long)(uint32_t)i.value);
      break;
    case Proto::INFO_BRIDGE_BOOT:
      Serial.println("BRIDGE boot");
      break;

    case Proto::INFO_BRIDGE_CAM_STATUS:
      Serial.print("Bridge camera: ");
      Serial.println(i.value ? "OK" : "FAIL");
      break;

    case Proto::INFO_BRIDGE_SD_STATUS:
      Serial.print("Bridge SD: ");
      Serial.println(i.value ? "OK" : "FAIL");
      break;
    case Proto::INFO_FC_BOOT:
    case Proto::INFO_PING_REPLY:
    case Proto::INFO_FLASH_NOT_FOUND:
    default:
      break;
  }
  Serial.println();
}

static void printPrimaryMission(const Proto::PrimaryMissionPayload &p, uint16_t seq) {
  Serial.printf("PRIMARY_MISSION seq=%u pressure=%ld Pa temp=%.2f C\n",
                seq, (long)p.pressure_pa, p.temperature_cC / 100.0f);
}

static const char *ackDetailText(uint8_t status, uint8_t detail) {
  if (status == Proto::ACK_REJECTED && detail == 1) return "invalid state";
  if (detail == 0) return nullptr;
  return "detail set";
}

static void printAttitude(const Proto::AttitudePayload &a, uint16_t seq) {
  Serial.printf("ATTITUDE seq=%u roll=%.2f pitch=%.2f yaw=%.2f deg\n",
                seq, a.roll_cdeg / 100.0f, a.pitch_cdeg / 100.0f, a.yaw_cdeg / 100.0f);
}

static void printGps(const Proto::GpsPayload &g, uint16_t seq) {
  Serial.printf("GPS seq=%u fix=%u lat=%.7f lon=%.7f alt=%.2f m\n",
                seq,
                g.fix_type,
                g.lat_e7 / 10000000.0,
                g.lon_e7 / 10000000.0,
                g.alt_cm / 100.0);
}

static void onEspNowRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  (void)info;
  Proto::Packet pkt{};
  if (!Proto::decodePacket(data, (uint16_t)len, pkt)) {
    Serial.printf("[GS] Bad packet len=%d\n", len);
    return;
  }

  switch (pkt.header.type) {
    case Proto::MSG_STATUS: {
      Proto::StatusPayload s{};
      if (Proto::payloadAs(pkt, s)) printStatus(s, pkt.header.seq);
      else Serial.println("[GS] Bad STATUS payload");
      break;
    }
    case Proto::MSG_ACK: {
      Proto::AckPayload a{};
      if (Proto::payloadAs(pkt, a)) printAck(a, pkt.header.seq);
      else Serial.println("[GS] Bad ACK payload");
      break;
    }
    case Proto::MSG_EVENT: {
      Proto::EventPayload e{};
      if (Proto::payloadAs(pkt, e)) printEvent(e, pkt.header.seq);
      else Serial.println("[GS] Bad EVENT payload");
      break;
    }
    case Proto::MSG_INFO: {
      Proto::InfoPayload i{};
      if (Proto::payloadAs(pkt, i)) printInfo(i, pkt.header.seq, "INFO");
      else Serial.println("[GS] Bad INFO payload");
      break;
    }
    case Proto::MSG_ERROR: {
      Proto::ErrorPayload i{};
      if (Proto::payloadAs(pkt, i)) printInfo(*(Proto::InfoPayload*)&i, pkt.header.seq, "ERROR");
      else Serial.println("[GS] Bad ERROR payload");
      break;
    }
    case Proto::MSG_GPS: {
      Proto::GpsPayload g{};
      if (Proto::payloadAs(pkt, g)) printGps(g, pkt.header.seq);
      else Serial.println("[GS] Bad GPS payload");
      break;
    }
    case Proto::MSG_PRIMARY_MISSION: {
      Proto::PrimaryMissionPayload p{};
      if (Proto::payloadAs(pkt, p)) printPrimaryMission(p, pkt.header.seq);
      else Serial.println("[GS] Bad PRIMARY_MISSION payload");
      break;
    }
    case Proto::MSG_ATTITUDE: {
      Proto::AttitudePayload a{};
      if (Proto::payloadAs(pkt, a)) printAttitude(a, pkt.header.seq);
      else Serial.println("[GS] Bad ATTITUDE payload");
      break;
    }
    case Proto::MSG_IMU_RAW: {
      Proto::ImuRawPayload p{};
      if (Proto::payloadAs(pkt, p)) {
        Serial.print("IMU raw: acc[g] ");
        Serial.print(p.ax_g, 3);
        Serial.print(", ");
        Serial.print(p.ay_g, 3);
        Serial.print(", ");
        Serial.print(p.az_g, 3);
        Serial.print(" gyro[dps] ");
        Serial.print(p.gx_dps, 3);
        Serial.print(", ");
        Serial.print(p.gy_dps, 3);
        Serial.print(", ");
        Serial.println(p.gz_dps, 3);
      } else {
        Serial.println("IMU raw: payload decode failed");
      }
      break;
    }
    default:
      Serial.printf("[GS] RX type=%s seq=%u len=%u\n",
                    Proto::messageTypeName(pkt.header.type),
                    pkt.header.seq,
                    pkt.header.payload_len);
      break;
  }
}

static void onEspNowSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  (void)mac_addr;
  DBG_PRINTF("[GS] ESP-NOW TX %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

static void printHelp() {
  Serial.println("Commands:");
  Serial.println("  ping fc");
  Serial.println("  ping bridge");
  Serial.println("  status");
  Serial.println("  startcam");
  Serial.println("  stopcam");
  Serial.println("  photo");
  Serial.println("  state <0..6>   (6 = FAULT)");
  Serial.println("  test_baro");
  Serial.println("  test_imu");
  Serial.println("  test_thermistor");
  Serial.println("  test_ldr");
  Serial.println("  test_gps");
}

static bool handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) return true;

  if (line == "help") {
    printHelp();
    return true;
  }
  if (line == "status") return sendCommand(Proto::DEST_FC, Proto::CMD_STATUS_REQUEST);
  if (line == "startcam") return sendCommand(Proto::DEST_BRIDGE, Proto::CMD_START_CAMERA);
  if (line == "stopcam") return sendCommand(Proto::DEST_BRIDGE, Proto::CMD_STOP_CAMERA);
  if (line == "photo") return sendCommand(Proto::DEST_BRIDGE, Proto::CMD_TAKE_PHOTO);
  if (line == "ping fc") return sendCommand(Proto::DEST_FC, Proto::CMD_PING);
  if (line == "ping bridge") return sendCommand(Proto::DEST_BRIDGE, Proto::CMD_PING);
  if (line == "test_baro") return sendCommand(Proto::DEST_FC, Proto::CMD_TEST_BARO);
  if (line == "test_imu") return sendCommand(Proto::DEST_FC, Proto::CMD_TEST_IMU);
  if (line == "test_thermistor") return sendCommand(Proto::DEST_FC, Proto::CMD_TEST_THERMISTOR);
  if (line == "test_ldr") return sendCommand(Proto::DEST_FC, Proto::CMD_TEST_LDR);
  if (line == "test_gps") return sendCommand(Proto::DEST_FC, Proto::CMD_TEST_GPS);

  if (line.startsWith("state ")) {
    const int value = line.substring(6).toInt();
    return sendCommand(Proto::DEST_FC, Proto::CMD_SET_STATE, value, 0);
  }

  Serial.println("Unknown command. Type 'help'.");
  return false;
}

void setup() {
  DBG_BEGIN(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  delay(100);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[GS] ESP-NOW init failed");
    while (true) {}
  }

  esp_now_register_recv_cb(onEspNowRecv);
  esp_now_register_send_cb(onEspNowSent);

  memcpy(peerInfo.peer_addr, bridgeMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("[GS] Failed to add bridge peer");
    while (true) {}
  }

  Serial.println("[GS] ready");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleSerialLine(line);
  }
}
