#include <Tonttulib.h>
#include "protocol.h"

Tonttulib tLib;

int32_t num = 0;
float lat = 61.2953;
float lon = 23.4539;
String state = "not armed";

void setup() {
    Serial.begin(115200);
    Serial1.begin(921600);

    int status = tLib.init();
    if (status != 1) {
        Serial.print("Init failed: ");
        Serial.println(status);
        while (true);
    }
}

void handleCommand(const Command &c) {
    // Safely construct String in case cmd is not null-terminated
    char buf[sizeof(c.cmd) + 1];
    memcpy(buf, c.cmd, sizeof(c.cmd));
    buf[sizeof(c.cmd)] = '\0';
    String cmd = String(buf);
    cmd.trim();

    if (cmd == "arm") {
        state = "armed";
        Serial.println("ARMED");
    } else if (cmd == "disarm") {
        state = "not armed";
        Serial.println("DISARMED");
    } else if (cmd == "ping") {
        Serial.println("PONG");
    } else {
        Serial.print("Unknown cmd -> ");
        Serial.println(cmd);
    }
}

void loop() {
    // Check for commands from bridge
    Command cmd;
    while (receivePacket(Serial1, cmd)) {
        handleCommand(cmd);
    }

    float pressure = tLib.baro.readPressure();

    // Send sensor data to bridge as binary
    SensorData sd;
    sd.pressure = pressure;
    sd.num      = num;
    strncpy(sd.state, state.c_str(), sizeof(sd.state));
    sd.state[sizeof(sd.state) - 1] = '\0';
    sd.lat = lat;
    sd.lon = lon;
    sendPacket(Serial1, sd);

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print(" State: ");
    Serial.print(state);
    Serial.println();

    tLib.update();
    num++;
    delay(400);
}
