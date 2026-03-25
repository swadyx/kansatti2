#pragma once

#include <Arduino.h>
#include "protocol.h"
#include <Tonttulib.h>

namespace FcComms {

struct StatusData {
  uint8_t  state = Proto::STATE_IDLE;
  bool     armed = false;
  bool     gpsFix = false;
  uint16_t lipoMv = 0;
  uint16_t flags = 0;
};

struct PrimaryMissionData {
  int32_t pressurePa = 0;
  int32_t temperature_cC = 0;
};

struct AttitudeData {
  int16_t roll_cdeg = 0;
  int16_t pitch_cdeg = 0;
  int16_t yaw_cdeg = 0;
};

struct GpsData {
  int32_t lat_e7 = 0;
  int32_t lon_e7 = 0;
  int32_t alt_cm = 0;
  uint8_t sats = 0;
  uint8_t fixType = 0;
  int32_t groundSpeed_cms = 0;
};

struct ImuRawData {
  float ax_g = 0.0f;
  float ay_g = 0.0f;
  float az_g = 0.0f;
  float gx_dps = 0.0f;
  float gy_dps = 0.0f;
  float gz_dps = 0.0f;
};

struct Callbacks {
  // High-level state change request from received command
  void (*onSetState)(uint8_t newState) = nullptr;
};

void begin(HardwareSerial& serial, uint32_t baud, const Callbacks& callbacks = {});

// Incoming commands + periodic internal comms housekeeping
void update(::Tonttulib& tLib);

// Outgoing packets: caller passes prepared data
void sendStatus(const StatusData& data);
void sendPrimaryMission(const PrimaryMissionData& data);
void sendAttitude(const AttitudeData& data);
void sendGps(const GpsData& data);
void sendImuRaw(const ImuRawData& data);

void sendBootEvent();
void sendFcBootInfo();
void sendInitError(int32_t errorCode);
void sendEvent(uint8_t eventId, int32_t value = 0);
void sendInfo(uint8_t code, int32_t value);
void sendError(uint8_t code, int32_t value);

bool stateImpliesArmed(uint8_t state);

} // namespace FcComms
