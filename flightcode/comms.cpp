#include "comms.h"
#include <Tonttulib.h>
#include "sensors.h"

namespace FcComms {

static HardwareSerial* sSerial = nullptr;
static Proto::Parser sParser;
static uint16_t sTxSeq = 1;
static Callbacks sCallbacks{};

static uint16_t nextSeq() {
  return sTxSeq++;
}

bool stateImpliesArmed(uint8_t state) {
  return !(state == Proto::STATE_IDLE ||
           state == Proto::STATE_GROUND ||
           state == Proto::STATE_FAULT);
}

static void sendAck(uint16_t ackedSeq, uint8_t status, uint8_t detail = 0) {
  if (!sSerial) return;

  Proto::AckPayload ack{};
  ack.acked_seq = ackedSeq;
  ack.status = status;
  ack.detail = detail;
  Proto::sendTyped(*sSerial, Proto::MSG_ACK, Proto::FLAG_IS_RESPONSE, nextSeq(), ack);
}

void sendInfo(uint8_t code, int32_t value) {
  if (!sSerial) return;

  Proto::InfoPayload p{};
  p.source = Proto::SRC_FC;
  p.code = code;
  p.value = value;
  Proto::sendTyped(*sSerial, Proto::MSG_INFO, 0, nextSeq(), p);
}

void sendError(uint8_t code, int32_t value) {
  if (!sSerial) return;

  Proto::ErrorPayload p{};
  p.source = Proto::SRC_FC;
  p.code = code;
  p.value = value;
  Proto::sendTyped(*sSerial, Proto::MSG_ERROR, Proto::FLAG_IMPORTANT, nextSeq(), p);
}

void sendEvent(uint8_t eventId, int32_t value) {
  if (!sSerial) return;

  Proto::EventPayload p{};
  p.event_id = eventId;
  p.source = Proto::SRC_FC;
  p.value = value;
  Proto::sendTyped(*sSerial, Proto::MSG_EVENT, Proto::FLAG_IMPORTANT, nextSeq(), p);
}

void begin(HardwareSerial& serial, uint32_t baud, const Callbacks& callbacks) {
  sSerial = &serial;
  sCallbacks = callbacks;
  sTxSeq = 1;
  serial.begin(baud, SERIAL_8N1);
}

void sendStatus(const StatusData& data) {
  if (!sSerial) return;

  Proto::StatusPayload s{};
  s.state   = data.state;
  s.armed   = data.armed ? 1 : 0;
  s.gps_fix = data.gpsFix ? 1 : 0;
  s.lipo_mV = data.lipoMv;
  s.flags   = data.flags;

  Proto::sendTyped(*sSerial, Proto::MSG_STATUS, 0, nextSeq(), s);
}

void sendImuRaw(const ImuRawData& data) {
  if (!sSerial) return;

  Proto::ImuRawPayload p{};
  p.ax_g = data.ax_g;
  p.ay_g = data.ay_g;
  p.az_g = data.az_g;
  p.gx_dps = data.gx_dps;
  p.gy_dps = data.gy_dps;
  p.gz_dps = data.gz_dps;

  Proto::sendTyped(*sSerial, Proto::MSG_IMU_RAW, 0, nextSeq(), p);
}

void sendBootEvent() {
  sendEvent(Proto::EVT_BOOT, 0);
}

void sendFcBootInfo() {
  sendInfo(Proto::INFO_FC_BOOT, 0);
}

void sendInitError(int32_t errorCode) {
  sendError(1, errorCode);
}

static void sendOneShotBaro(Tonttulib& tLib) {
  const float pressure = Sensors::getPressure();
  sendInfo(Proto::INFO_TEST_BARO,
           (int32_t)(pressure + (pressure >= 0 ? 0.5f : -0.5f)));
}

static void sendOneShotThermistor(Tonttulib& tLib) {
  const float tempC = tLib.thermistor.readCelsius();
  sendInfo(Proto::INFO_TEST_THERMISTOR,
           (int32_t)(tempC * 100.0f + (tempC >= 0 ? 0.5f : -0.5f)));
}

static void sendOneShotLdr(Tonttulib& tLib) {
  const float ldrV = tLib.ldr.readVoltage();
  sendInfo(Proto::INFO_TEST_LDR,
           (int32_t)(ldrV * 1000.0f + (ldrV >= 0 ? 0.5f : -0.5f)));
}

static void sendOneShotGps(Tonttulib& tLib) {
  GpsData g{};
  g.lat_e7 = (int32_t)(tLib.gps.latitude() * 10000000.0);
  g.lon_e7 = (int32_t)(tLib.gps.longitude() * 10000000.0);
  g.alt_cm = (int32_t)(tLib.gps.altitude() * 100.0f + (tLib.gps.altitude() >= 0 ? 0.5f : -0.5f));
  g.sats = 0;
  g.fixType = (uint8_t)tLib.gps.fixType();
  g.groundSpeed_cms = 0;
  Proto::sendTyped(*sSerial, Proto::MSG_GPS, 0, nextSeq(), g);
}

static void sendOneShotImu(Tonttulib& tLib) {
  ImuRawData d{};
  tLib.imu.readAccel(d.ax_g, d.ay_g, d.az_g);
  tLib.imu.readGyro(d.gx_dps, d.gy_dps, d.gz_dps);
  sendImuRaw(d);
}

static bool handleCommand(Tonttulib& tLib, const Proto::Packet& pkt) {
  Proto::CommandPayload cmd{};
  if (!Proto::payloadAs(pkt, cmd)) {
    sendAck(pkt.header.seq, Proto::ACK_BAD_LENGTH);
    return false;
  }

  if (!(cmd.dest == Proto::DEST_FC || cmd.dest == Proto::DEST_ALL)) {
    sendAck(pkt.header.seq, Proto::ACK_BAD_DEST);
    return false;
  }

  switch (cmd.cmd_id) {
    case Proto::CMD_PING:
      sendAck(pkt.header.seq, Proto::ACK_OK);
      sendInfo(Proto::INFO_PING_REPLY, 1);
      return true;

    case Proto::CMD_STATUS_REQUEST:
      sendAck(pkt.header.seq, Proto::ACK_OK);
      return true;

    case Proto::CMD_SET_STATE:
      if (cmd.arg0 < Proto::STATE_IDLE || cmd.arg0 > Proto::STATE_FAULT) {
        sendAck(pkt.header.seq, Proto::ACK_REJECTED, 1);
        return false;
      }
      if (sCallbacks.onSetState) {
        sCallbacks.onSetState((uint8_t)cmd.arg0);
      }
      sendAck(pkt.header.seq, Proto::ACK_OK);
      return true;

    case Proto::CMD_TEST_BARO:
      sendAck(pkt.header.seq, Proto::ACK_OK);
      sendOneShotBaro(tLib);
      return true;

    case Proto::CMD_TEST_IMU:
      sendAck(pkt.header.seq, Proto::ACK_OK);
      sendOneShotImu(tLib);
      return true;

    case Proto::CMD_TEST_THERMISTOR:
      sendAck(pkt.header.seq, Proto::ACK_OK);
      sendOneShotThermistor(tLib);
      return true;

    case Proto::CMD_TEST_LDR:
      sendAck(pkt.header.seq, Proto::ACK_OK);
      sendOneShotLdr(tLib);
      return true;

    case Proto::CMD_TEST_GPS:
      sendAck(pkt.header.seq, Proto::ACK_OK);
      sendOneShotGps(tLib);
      return true;

    default:
      sendAck(pkt.header.seq, Proto::ACK_UNKNOWN_CMD);
      return false;
  }
}

void update(::Tonttulib& tLib) {
  if (!sSerial) return;

  Proto::Packet pkt{};
  while (Proto::readPacket(*sSerial, sParser, pkt)) {
    if (pkt.header.type == Proto::MSG_COMMAND) {
      handleCommand(tLib, pkt);
    }
  }
}

} // namespace FcComms
