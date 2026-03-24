#pragma once
#include <stdint.h>
#include <string.h>

// ── Framing ──────────────────────────────────────────────────────────────────
// Every packet on the wire:
//   [0xAA] [0x55] [TYPE] [payload bytes...] [XOR checksum over TYPE+payload]
//
// Having TYPE in the frame means a single byte stream can carry multiple
// packet types without parsers consuming each other's bytes.

static const uint8_t SYNC1 = 0xAA;
static const uint8_t SYNC2 = 0x55;

enum PacketType : uint8_t {
    PKT_SENSOR_DATA  = 0x01,
    PKT_COMMAND      = 0x02,
    PKT_PID_COMMAND  = 0x03,
    PKT_PID_CONFIG   = 0x04,
};

#pragma pack(push, 1)

struct SensorData {
    float    pressure;
    int32_t  num;
    char     state[16];
    float    roll, pitch, yaw;
    float    thrust;
    float    lipo_v;
    uint16_t m1, m2, m3, m4;
};

struct Command {
    char cmd[32];
};

// gain_id map:
//  0=AKP_R  1=AKP_P  2=AKI_R  3=AKI_P
//  4=RKP_R  5=RKP_P  6=RKP_Y
//  7=RKI_R  8=RKI_P  9=RKI_Y
// 10=RKD_R 11=RKD_P 12=RKD_Y
// 0xFE = request echo of all gains
// 0xFF = reset integrators
struct PIDCommand {
    uint8_t gain_id;
    float   value;
};

struct PIDConfig {
    float angle_kp_roll,  angle_kp_pitch;
    float angle_ki_roll,  angle_ki_pitch;
    float rate_kp_roll,   rate_kp_pitch,  rate_kp_yaw;
    float rate_ki_roll,   rate_ki_pitch,  rate_ki_yaw;
    float rate_kd_roll,   rate_kd_pitch,  rate_kd_yaw;
};

#pragma pack(pop)

// ── Send ─────────────────────────────────────────────────────────────────────
static uint8_t _cs(uint8_t init, const uint8_t *buf, size_t len) {
    uint8_t cs = init;
    for (size_t i = 0; i < len; i++) cs ^= buf[i];
    return cs;
}

template<typename T>
static void sendPacket(HardwareSerial &serial, PacketType type, const T &payload) {
    const uint8_t *buf = (const uint8_t *)&payload;
    size_t len = sizeof(T);
    serial.write(SYNC1);
    serial.write(SYNC2);
    serial.write((uint8_t)type);
    serial.write(buf, len);
    serial.write(_cs((uint8_t)type, buf, len));
}

// Convenience overloads so call sites don't need to pass the type manually
static inline void sendPacket(HardwareSerial &s, const SensorData &p)  { sendPacket(s, PKT_SENSOR_DATA, p); }
static inline void sendPacket(HardwareSerial &s, const Command &p)      { sendPacket(s, PKT_COMMAND,     p); }
static inline void sendPacket(HardwareSerial &s, const PIDCommand &p)   { sendPacket(s, PKT_PID_COMMAND, p); }
static inline void sendPacket(HardwareSerial &s, const PIDConfig &p)    { sendPacket(s, PKT_PID_CONFIG,  p); }

// ── Receive — single dispatcher ───────────────────────────────────────────────
// Call this once per serial port per loop iteration.
// It returns the type of packet received, or 0 if nothing yet.
// Pass pointers to whichever structs you want populated; unused ones can be nullptr.
static uint8_t receiveAnyPacket(HardwareSerial &serial,
                                SensorData  *sd,
                                Command     *cmd,
                                PIDCommand  *pc,
                                PIDConfig   *cfg)
{
    // Maximum payload size across all types
    static const size_t MAX_PAYLOAD = 64;
    static uint8_t  buf[MAX_PAYLOAD + 1];  // +1 for checksum
    static size_t   pos      = 0;
    static bool     synced   = false;
    static uint8_t  prev     = 0xFF;
    static uint8_t  pkt_type = 0;
    static size_t   expected = 0;

    while (serial.available()) {
        uint8_t b = (uint8_t)serial.read();

        if (!synced) {
            if (prev == SYNC1 && b == SYNC2) {
                synced   = true;
                pos      = 0;
                pkt_type = 0;
                expected = 0;
            }
            prev = b;
            continue;
        }

        // First byte after sync is the type
        if (pkt_type == 0) {
            pkt_type = b;
            switch (pkt_type) {
                case PKT_SENSOR_DATA: expected = sizeof(SensorData);  break;
                case PKT_COMMAND:     expected = sizeof(Command);      break;
                case PKT_PID_COMMAND: expected = sizeof(PIDCommand);   break;
                case PKT_PID_CONFIG:  expected = sizeof(PIDConfig);    break;
                default:
                    synced   = false;   // unknown type, re-sync
                    pkt_type = 0;
                    break;
            }
            continue;
        }

        // Payload + checksum byte
        if (pos < expected + 1) {
            buf[pos++] = b;
        }

        if (pos == expected + 1) {
            synced = false;
            pos    = 0;

            uint8_t expected_cs = _cs(pkt_type, buf, expected);
            if (buf[expected] != expected_cs) {
                pkt_type = 0;
                continue;   // checksum fail, drop packet
            }

            uint8_t type = pkt_type;
            pkt_type = 0;

            switch (type) {
                case PKT_SENSOR_DATA:
                    if (sd)  memcpy(sd,  buf, sizeof(SensorData));  break;
                case PKT_COMMAND:
                    if (cmd) memcpy(cmd, buf, sizeof(Command));      break;
                case PKT_PID_COMMAND:
                    if (pc)  memcpy(pc,  buf, sizeof(PIDCommand));   break;
                case PKT_PID_CONFIG:
                    if (cfg) memcpy(cfg, buf, sizeof(PIDConfig));    break;
            }
            return type;
        }
    }
    return 0;
}

// ── Legacy single-type helper (kept for bridge ESP-NOW forward path) ──────────
// Use receiveAnyPacket instead wherever possible.
template<typename T>
static bool receivePacket(HardwareSerial &serial, T &out) {
    static uint8_t buf[sizeof(T) + 1];
    static size_t  pos    = 0;
    static bool    synced = false;
    static uint8_t prev   = 0xFF;

    while (serial.available()) {
        uint8_t b = (uint8_t)serial.read();
        if (!synced) {
            if (prev == SYNC1 && b == SYNC2) { synced = true; pos = 0; }
            prev = b;
            continue;
        }
        buf[pos++] = b;
        if (pos == sizeof(T) + 1) {
            synced = false; pos = 0;
            if (buf[sizeof(T)] == _cs(0, buf, sizeof(T))) {
                memcpy(&out, buf, sizeof(T));
                return true;
            }
        }
    }
    return false;
}
