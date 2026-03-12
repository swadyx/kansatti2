#include <Tonttulib.h>

Tonttulib tLib;

int num = 0;
float lat = 61.2953;
float lon = 23.4539;
String state = "not armed";

void setup() {
    Serial.begin(115200);
    Serial1.begin(900000);

    int status = tLib.init();
    if (status != 1) {
        Serial.print("Init failed: ");
        Serial.println(status);
        while (true);
    }
}

void handleCommand(String cmd) {
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
        // REMOVED: don't send non-CSV text on Serial1, it corrupts the data stream
    }
}

void loop() {
    // Check for commands FIRST
    while (Serial1.available()) {
        String cmd = Serial1.readStringUntil('\n');
        handleCommand(cmd);
    }

    float pressure = tLib.baro.readPressure();

    // Send CSV data to bridge
    Serial1.print(pressure);
    Serial1.print(",");
    Serial1.print(num);
    Serial1.print(",");
    Serial1.print(state);
    Serial1.print(",");
    Serial1.print(lat, 6);
    Serial1.print(",");
    Serial1.println(lon, 6);

    Serial.print("Pressure: ");
    Serial.print(pressure);
    Serial.print(" State: ");
    Serial.print(state);
    Serial.println();

    tLib.update();
    num++;
    delay(400);
}
