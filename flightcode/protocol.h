#pragma once
#include <Arduino.h>
#include <stdint.h>
#include <string.h>

namespace Proto {

static constexpr uint8_t  SYNC1              = 0xAA;
static constexpr uint8_t  SYNC2              = 0x55;
static constexpr uint8_t  VERSION            = 1;
static constexpr uint16_t MAX_PAYLOAD_LEN    = 192;
static constexpr uint16_t MAX_FRAME_LEN      = 2 + 1 + 1 + 1 + 2 + 2 + MAX_PAYLOAD_LEN + 2;
static constexpr uint16_t HEADER_NO_SYNC_LEN = 1 + 1 + 1 + 2 + 2;
static constexpr uint16_t HEADER_LEN         = 2 + HEADER_NO_SYNC_LEN;
static constexpr uint16_t CRC_LEN            = 2;

enum Destination : uint8_t {
    DEST_FC     = 1,
    DEST_BRIDGE = 2,
    DEST_GS     = 3,
    DEST_ALL    = 255,
};

enum MessageType : uint8_t {
    MSG_NONE            = 0,
    MSG_STATUS          = 1,
    MSG_COMMAND         = 2,
    MSG_ACK             = 3,
    MSG_EVENT           = 4,
    MSG_INFO            = 5,
    MSG_ERROR           = 6,
    MSG_GPS             = 7,
    MSG_PRIMARY_MISSION = 8,
    MSG_ATTITUDE        = 9,
    MSG_IMU_RAW = 10
};

enum FlightState : uint8_t {
    STATE_IDLE           = 0,
    STATE_PRELAUNCH      = 1,
    STATE_ROCKET         = 2,
    STATE_FREEFLIGHT     = 3,
    STATE_POWERED_FLIGHT = 4,
    STATE_GROUND         = 5,
    STATE_FAULT          = 6,
};

enum CommandId : uint8_t {
    CMD_NONE             = 0,
    CMD_PING             = 1,
    CMD_STATUS_REQUEST   = 2,
    CMD_START_CAMERA     = 3,
    CMD_STOP_CAMERA      = 4,
    CMD_TAKE_PHOTO       = 5,
    CMD_SET_TELEM_RATE   = 6,
    CMD_SET_STATE        = 7,
    CMD_TEST_BARO        = 8,
    CMD_TEST_IMU         = 9,
    CMD_TEST_THERMISTOR  = 10,
    CMD_TEST_LDR         = 11,
    CMD_TEST_GPS         = 12,
};

enum AckStatus : uint8_t {
    ACK_OK            = 0,
    ACK_BAD_CRC       = 1,
    ACK_BAD_LENGTH    = 2,
    ACK_UNKNOWN_TYPE  = 3,
    ACK_UNKNOWN_CMD   = 4,
    ACK_BAD_DEST      = 5,
    ACK_REJECTED      = 6,
    ACK_BUSY          = 7,
    ACK_INTERNAL_ERR  = 8,
};

enum EventId : uint8_t {
    EVT_NONE                = 0,
    EVT_BOOT                = 1,
    EVT_LAUNCH_DETECTED     = 2,
    EVT_ENTER_FREEFLIGHT    = 3,
    EVT_ENTER_POWERED       = 4,
    EVT_LANDED              = 5,
    EVT_GPS_FIX_ACQUIRED    = 6,
    EVT_GPS_FIX_LOST        = 7,
    EVT_CAMERA_STARTED      = 8,
    EVT_CAMERA_STOPPED      = 9,
    EVT_PHOTO_TAKEN         = 10,
};

enum BoardSource : uint8_t {
    SRC_UNKNOWN = 0,
    SRC_FC      = 1,
    SRC_BRIDGE  = 2,
    SRC_GS      = 3,
};

enum PacketFlags : uint8_t {
    FLAG_ACK_REQUESTED = 0x01,
    FLAG_IS_RESPONSE   = 0x02,
    FLAG_IMPORTANT     = 0x04,
};

enum StatusFlags : uint16_t {
    STATUSF_INIT_FAILED   = 0x0001,
    STATUSF_FLASH_PRESENT = 0x0002,
    STATUSF_GPS_FIX       = 0x0004,
};

enum InfoCode : uint8_t {
    INFO_NONE              = 0,
    INFO_PING_REPLY        = 1,
    INFO_FLASH_ADDR        = 2,
    INFO_FLASH_NOT_FOUND   = 3,
    INFO_CAMERA_READY      = 4,
    INFO_SD_READY          = 5,
    INFO_BRIDGE_BOOT       = 6,
    INFO_FC_BOOT           = 7,
    INFO_TEST_BARO         = 8,
    INFO_TEST_THERMISTOR   = 9,
    INFO_TEST_LDR          = 10,
    INFO_BRIDGE_CAM_STATUS  = 21,
    INFO_BRIDGE_SD_STATUS   = 22
};

#pragma pack(push, 1)

struct Header {
    uint8_t  version;
    uint8_t  type;
    uint8_t  flags;
    uint16_t seq;
    uint16_t payload_len;
};

struct Packet {
    Header   header;
    uint8_t  payload[MAX_PAYLOAD_LEN];
};

struct StatusPayload {
    uint8_t  state;
    uint8_t  armed;
    uint8_t  gps_fix;
    uint8_t  reserved0;
    uint16_t lipo_mV;
    uint16_t flags;
};

struct CommandPayload {
    uint8_t  dest;
    uint8_t  cmd_id;
    int32_t  arg0;
    int32_t  arg1;
};

struct AckPayload {
    uint16_t acked_seq;
    uint8_t  status;
    uint8_t  detail;
};

struct EventPayload {
    uint8_t  event_id;
    uint8_t  source;
    int32_t  value;
};

struct InfoPayload {
    uint8_t  source;
    uint8_t  code;
    int32_t  value;
};

struct ErrorPayload {
    uint8_t  source;
    uint8_t  code;
    int32_t  value;
};

struct GpsPayload {
    int32_t lat_e7;
    int32_t lon_e7;
    int32_t alt_cm;
    uint8_t sats;
    uint8_t fix_type;
    uint16_t ground_speed_cms;
};

struct PrimaryMissionPayload {
    int32_t pressure_pa;
    int32_t temperature_cC;
};

struct AttitudePayload {
    int16_t roll_cdeg;
    int16_t pitch_cdeg;
    int16_t yaw_cdeg;
};

struct ImuRawPayload {
  float ax_g;
  float ay_g;
  float az_g;
  float gx_dps;
  float gy_dps;
  float gz_dps;
};
#pragma pack(pop)



struct Parser {
    enum State : uint8_t { WAIT_SYNC1 = 0, WAIT_SYNC2, READ_HEADER, READ_PAYLOAD, READ_CRC };
    State    state        = WAIT_SYNC1;
    uint8_t  header_buf[HEADER_NO_SYNC_LEN] = {0};
    uint16_t header_pos   = 0;
    Packet   packet       = {};
    uint16_t payload_pos  = 0;
    uint8_t  crc_buf[2]   = {0};
    uint8_t  crc_pos      = 0;
    void reset() { state = WAIT_SYNC1; header_pos = 0; payload_pos = 0; crc_pos = 0; }
};

static inline uint16_t crc16_ccitt_update(uint16_t crc, uint8_t data) {
    crc ^= (uint16_t)data << 8;
    for (uint8_t i = 0; i < 8; ++i) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    return crc;
}
static inline uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; ++i) crc = crc16_ccitt_update(crc, data[i]);
    return crc;
}
static inline void writeU16LE(uint8_t *dst, uint16_t v) { dst[0] = (uint8_t)(v & 0xFF); dst[1] = (uint8_t)((v >> 8) & 0xFF); }
static inline uint16_t readU16LE(const uint8_t *src) { return (uint16_t)src[0] | ((uint16_t)src[1] << 8); }

static inline uint16_t computePacketCrc(const Packet &pkt) {
    uint8_t temp[HEADER_NO_SYNC_LEN + MAX_PAYLOAD_LEN];
    temp[0] = pkt.header.version;
    temp[1] = pkt.header.type;
    temp[2] = pkt.header.flags;
    writeU16LE(&temp[3], pkt.header.seq);
    writeU16LE(&temp[5], pkt.header.payload_len);
    if (pkt.header.payload_len > 0) memcpy(&temp[7], pkt.payload, pkt.header.payload_len);
    return crc16_ccitt(temp, (uint16_t)(HEADER_NO_SYNC_LEN + pkt.header.payload_len));
}

static inline bool initPacketRaw(Packet &pkt, uint8_t type, uint8_t flags, uint16_t seq, const void *payload, uint16_t payload_len) {
    if (payload_len > MAX_PAYLOAD_LEN) return false;
    pkt.header.version = VERSION;
    pkt.header.type = type;
    pkt.header.flags = flags;
    pkt.header.seq = seq;
    pkt.header.payload_len = payload_len;
    if (payload_len > 0 && payload) memcpy(pkt.payload, payload, payload_len);
    return true;
}

template<typename T>
static inline bool initPacket(Packet &pkt, uint8_t type, uint8_t flags, uint16_t seq, const T &payload) {
    return initPacketRaw(pkt, type, flags, seq, &payload, sizeof(T));
}

template<typename T>
static inline bool payloadAs(const Packet &pkt, T &out) {
    if (pkt.header.payload_len != sizeof(T)) return false;
    memcpy(&out, pkt.payload, sizeof(T));
    return true;
}

static inline int encodePacket(const Packet &pkt, uint8_t *out, uint16_t out_cap) {
    const uint16_t total_len = (uint16_t)(HEADER_LEN + pkt.header.payload_len + CRC_LEN);
    if (out_cap < total_len || pkt.header.payload_len > MAX_PAYLOAD_LEN) return -1;
    out[0] = SYNC1; out[1] = SYNC2; out[2] = pkt.header.version; out[3] = pkt.header.type; out[4] = pkt.header.flags;
    writeU16LE(&out[5], pkt.header.seq); writeU16LE(&out[7], pkt.header.payload_len);
    if (pkt.header.payload_len > 0) memcpy(&out[9], pkt.payload, pkt.header.payload_len);
    const uint16_t crc = computePacketCrc(pkt);
    writeU16LE(&out[9 + pkt.header.payload_len], crc);
    return total_len;
}

static inline bool decodePacket(const uint8_t *frame, uint16_t len, Packet &pkt) {
    if (!frame || len < HEADER_LEN + CRC_LEN) return false;
    if (frame[0] != SYNC1 || frame[1] != SYNC2) return false;
    pkt.header.version = frame[2]; pkt.header.type = frame[3]; pkt.header.flags = frame[4];
    pkt.header.seq = readU16LE(&frame[5]); pkt.header.payload_len = readU16LE(&frame[7]);
    if (pkt.header.version != VERSION || pkt.header.payload_len > MAX_PAYLOAD_LEN) return false;
    if (len != HEADER_LEN + pkt.header.payload_len + CRC_LEN) return false;
    if (pkt.header.payload_len > 0) memcpy(pkt.payload, &frame[9], pkt.header.payload_len);
    const uint16_t rx_crc = readU16LE(&frame[9 + pkt.header.payload_len]);
    return rx_crc == computePacketCrc(pkt);
}

static inline bool parseByte(Parser &p, uint8_t b, Packet &out) {
    switch (p.state) {
        case Parser::WAIT_SYNC1:
            if (b == SYNC1) p.state = Parser::WAIT_SYNC2;
            break;
        case Parser::WAIT_SYNC2:
            if (b == SYNC2) { p.state = Parser::READ_HEADER; p.header_pos = 0; }
            else p.state = (b == SYNC1) ? Parser::WAIT_SYNC2 : Parser::WAIT_SYNC1;
            break;
        case Parser::READ_HEADER:
            p.header_buf[p.header_pos++] = b;
            if (p.header_pos == HEADER_NO_SYNC_LEN) {
                p.packet.header.version = p.header_buf[0];
                p.packet.header.type = p.header_buf[1];
                p.packet.header.flags = p.header_buf[2];
                p.packet.header.seq = readU16LE(&p.header_buf[3]);
                p.packet.header.payload_len = readU16LE(&p.header_buf[5]);
                if (p.packet.header.version != VERSION || p.packet.header.payload_len > MAX_PAYLOAD_LEN) { p.reset(); break; }
                p.payload_pos = 0; p.crc_pos = 0;
                p.state = (p.packet.header.payload_len == 0) ? Parser::READ_CRC : Parser::READ_PAYLOAD;
            }
            break;
        case Parser::READ_PAYLOAD:
            p.packet.payload[p.payload_pos++] = b;
            if (p.payload_pos == p.packet.header.payload_len) { p.crc_pos = 0; p.state = Parser::READ_CRC; }
            break;
        case Parser::READ_CRC:
            p.crc_buf[p.crc_pos++] = b;
            if (p.crc_pos == 2) {
                const uint16_t rx_crc = readU16LE(p.crc_buf);
                const bool ok = (rx_crc == computePacketCrc(p.packet));
                if (ok) out = p.packet;
                p.reset();
                return ok;
            }
            break;
    }
    return false;
}

static inline bool readPacket(Stream &serial, Parser &parser, Packet &out) {
    while (serial.available()) if (parseByte(parser, (uint8_t)serial.read(), out)) return true;
    return false;
}
static inline bool writePacket(Stream &serial, const Packet &pkt) {
    uint8_t frame[MAX_FRAME_LEN];
    const int len = encodePacket(pkt, frame, sizeof(frame));
    return (len > 0) && (serial.write(frame, (size_t)len) == (size_t)len);
}

template<typename T>
static inline bool sendTyped(Stream &serial, uint8_t type, uint8_t flags, uint16_t seq, const T &payload) {
    Packet pkt{};
    if (!initPacket(pkt, type, flags, seq, payload)) return false;
    return writePacket(serial, pkt);
}

static inline const char *messageTypeName(uint8_t type) {
    switch (type) {
        case MSG_STATUS: return "STATUS"; case MSG_COMMAND: return "COMMAND"; case MSG_ACK: return "ACK";
        case MSG_EVENT: return "EVENT"; case MSG_INFO: return "INFO"; case MSG_ERROR: return "ERROR";
        case MSG_GPS: return "GPS"; case MSG_PRIMARY_MISSION: return "PRIMARY_MISSION"; case MSG_ATTITUDE: return "ATTITUDE";
        default: return "UNKNOWN";
    }
}
static inline const char *stateName(uint8_t s) {
    switch (s) {
        case STATE_IDLE: return "IDLE"; case STATE_PRELAUNCH: return "PRELAUNCH"; case STATE_ROCKET: return "ROCKET";
        case STATE_FREEFLIGHT: return "FREEFLIGHT"; case STATE_POWERED_FLIGHT: return "POWERED_FLIGHT";
        case STATE_GROUND: return "GROUND"; case STATE_FAULT: return "FAULT"; default: return "UNKNOWN";
    }
}
static inline const char *commandName(uint8_t c) {
    switch (c) {
        case CMD_PING: return "PING"; case CMD_STATUS_REQUEST: return "STATUS_REQUEST"; case CMD_START_CAMERA: return "START_CAMERA";
        case CMD_STOP_CAMERA: return "STOP_CAMERA"; case CMD_TAKE_PHOTO: return "TAKE_PHOTO"; case CMD_SET_TELEM_RATE: return "SET_TELEM_RATE";
        case CMD_SET_STATE: return "SET_STATE"; case CMD_TEST_BARO: return "TEST_BARO"; case CMD_TEST_IMU: return "TEST_IMU"; case CMD_TEST_THERMISTOR: return "TEST_THERMISTOR"; case CMD_TEST_LDR: return "TEST_LDR"; case CMD_TEST_GPS: return "TEST_GPS"; default: return "UNKNOWN";
    }
}
static inline const char *ackStatusName(uint8_t s) {
    switch (s) {
        case ACK_OK: return "OK"; case ACK_BAD_CRC: return "BAD_CRC"; case ACK_BAD_LENGTH: return "BAD_LENGTH";
        case ACK_UNKNOWN_TYPE: return "UNKNOWN_TYPE"; case ACK_UNKNOWN_CMD: return "UNKNOWN_CMD"; case ACK_BAD_DEST: return "BAD_DEST";
        case ACK_REJECTED: return "REJECTED"; case ACK_BUSY: return "BUSY"; case ACK_INTERNAL_ERR: return "INTERNAL_ERR"; default: return "?";
    }
}
static inline const char *sourceName(uint8_t s) {
    switch (s) {
        case SRC_FC: return "FC"; case SRC_BRIDGE: return "BRIDGE"; case SRC_GS: return "GS"; default: return "UNKNOWN";
    }
}
static inline const char *infoCodeName(uint8_t c) {
    switch (c) {
        case INFO_PING_REPLY: return "PING_REPLY"; case INFO_FLASH_ADDR: return "FLASH_ADDR"; case INFO_FLASH_NOT_FOUND: return "FLASH_NOT_FOUND";
        case INFO_CAMERA_READY: return "CAMERA_READY"; case INFO_SD_READY: return "SD_READY"; case INFO_BRIDGE_BOOT: return "BRIDGE_BOOT";
        case INFO_FC_BOOT: return "FC_BOOT"; case INFO_TEST_BARO: return "TEST_BARO"; case INFO_TEST_THERMISTOR: return "TEST_THERMISTOR"; case INFO_TEST_LDR: return "TEST_LDR"; default: return "INFO_UNKNOWN";
    }
}
static inline const char *eventName(uint8_t e) {
    switch (e) {
        case EVT_BOOT: return "BOOT"; case EVT_LAUNCH_DETECTED: return "LAUNCH_DETECTED"; case EVT_ENTER_FREEFLIGHT: return "ENTER_FREEFLIGHT";
        case EVT_ENTER_POWERED: return "ENTER_POWERED"; case EVT_LANDED: return "LANDED"; case EVT_GPS_FIX_ACQUIRED: return "GPS_FIX_ACQUIRED";
        case EVT_GPS_FIX_LOST: return "GPS_FIX_LOST"; case EVT_CAMERA_STARTED: return "CAMERA_STARTED"; case EVT_CAMERA_STOPPED: return "CAMERA_STOPPED";
        case EVT_PHOTO_TAKEN: return "PHOTO_TAKEN"; default: return "EVENT_UNKNOWN";
    }
}
inline FlightState getFlightState(const Proto::Packet& pkt) {
    Proto::StatusPayload sp;
    if (Proto::payloadAs(pkt, sp)) {
        return (Proto::FlightState)sp.state;
    }
    return Proto::STATE_FAULT; // fallback
}


} // namespace Proto
