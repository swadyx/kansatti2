#pragma once
#include <stdint.h>
#include <string.h>

// Sync bytes for framing
static const uint8_t SYNC1 = 0xAA;
static const uint8_t SYNC2 = 0x55;

#pragma pack(push, 1)
struct SensorData {
    float   pressure;
    int32_t num;
    char    state[16];
    float   lat;
    float   lon;
};

struct Command {
    char cmd[32];
};
#pragma pack(pop)

// XOR checksum over payload bytes
static uint8_t calcChecksum(const uint8_t *buf, size_t len) {
    uint8_t cs = 0;
    for (size_t i = 0; i < len; i++) cs ^= buf[i];
    return cs;
}

// Send a packet: [SYNC1][SYNC2][payload...][checksum]
template<typename T>
static void sendPacket(HardwareSerial &serial, const T &payload) {
    const uint8_t *buf = (const uint8_t *)&payload;
    size_t len = sizeof(T);
    serial.write(SYNC1);
    serial.write(SYNC2);
    serial.write(buf, len);
    serial.write(calcChecksum(buf, len));
}

// Receive a packet using a state-machine. Returns true when a valid packet
// has been received and copied into 'out'. Call repeatedly from loop().
// NOTE: static local variables are per template instantiation, so
// receivePacket<SensorData> and receivePacket<Command> maintain independent state.
template<typename T>
static bool receivePacket(HardwareSerial &serial, T &out) {
    static uint8_t  buf[sizeof(T) + 1];  // +1 for checksum byte
    static size_t   pos    = 0;
    static bool     synced = false;
    static uint8_t  prev   = 0xFF;  // init to non-SYNC1 value

    while (serial.available()) {
        uint8_t b = (uint8_t)serial.read();

        if (!synced) {
            if (prev == SYNC1 && b == SYNC2) {
                synced = true;
                pos    = 0;
            }
            prev = b;
            continue;
        }

        buf[pos++] = b;

        if (pos == sizeof(T) + 1) {  // payload + checksum byte
            synced = false;
            pos    = 0;
            uint8_t expected = calcChecksum(buf, sizeof(T));
            if (buf[sizeof(T)] == expected) {
                memcpy(&out, buf, sizeof(T));
                return true;
            }
            // Bad checksum — drop and keep looking
        }
    }
    return false;
}
