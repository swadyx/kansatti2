#include <esp_now.h>
#include <WiFi.h>
#include "protocol.h"

uint8_t bridgeMAC[] = { 0x80, 0xB5, 0x4E, 0xF1, 0x22, 0x34 };


esp_now_peer_info_t peerInfo;
SensorData latest;
PIDConfig  latestPID;
bool dataReceived = false;
bool pidReceived  = false;

void printTelemetry() {
    Serial.println("──────────────────────────────");
    Serial.printf("State   : %s\n",        latest.state);
    Serial.printf("Pressure: %.1f hPa\n",  latest.pressure);
    Serial.printf("Roll    : %.2f °\n",    latest.roll);
    Serial.printf("Pitch   : %.2f °\n",    latest.pitch);
    Serial.printf("Yaw     : %.2f °\n",    latest.yaw);
    Serial.printf("Thrust  : %.3f\n",      latest.thrust);
    Serial.printf("LiPo    : %.2f V\n",    latest.lipo_v);
    Serial.printf("Motors  : %d / %d / %d / %d us\n",
        latest.m1, latest.m2, latest.m3, latest.m4);
    Serial.printf("Seq#    : %d\n",        latest.num);
    Serial.println("──────────────────────────────");
}

void printPIDConfig() {
    Serial.println("=== PID CONFIG ===");
    Serial.printf("ANGLE_KP  R:%.5f  P:%.5f\n", latestPID.angle_kp_roll,  latestPID.angle_kp_pitch);
    Serial.printf("ANGLE_KI  R:%.5f  P:%.5f\n", latestPID.angle_ki_roll,  latestPID.angle_ki_pitch);
    Serial.printf("RATE_KP   R:%.6f  P:%.6f  Y:%.6f\n", latestPID.rate_kp_roll,  latestPID.rate_kp_pitch, latestPID.rate_kp_yaw);
    Serial.printf("RATE_KI   R:%.6f  P:%.6f  Y:%.6f\n", latestPID.rate_ki_roll,  latestPID.rate_ki_pitch, latestPID.rate_ki_yaw);
    Serial.printf("RATE_KD   R:%.7f  P:%.7f  Y:%.7f\n", latestPID.rate_kd_roll,  latestPID.rate_kd_pitch, latestPID.rate_kd_yaw);
    Serial.println("==================");
}

// ESP-NOW recv: GS receives SensorData and PIDConfig from bridge
void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len == sizeof(SensorData)) {
        memcpy(&latest, data, sizeof(latest)); dataReceived = true;
    } else if (len == sizeof(PIDConfig)) {
        memcpy(&latestPID, data, sizeof(latestPID)); pidReceived = true;
    } else {
        Serial.printf("Unexpected recv size %d\n", len);
    }
}

void onDataSent(const uint8_t *mac, esp_now_send_status_t status) {}

// Send a raw Command struct to bridge over ESP-NOW (no framing — ESP-NOW is datagram)
void sendCommand(const char *cmdStr) {
    Command c; memset(&c, 0, sizeof(c));
    strncpy(c.cmd, cmdStr, sizeof(c.cmd)-1);
    esp_now_send(bridgeMAC, (uint8_t *)&c, sizeof(c));
    Serial.print("CMD> "); Serial.println(cmdStr);
}

// Send a raw PIDCommand struct to bridge over ESP-NOW
void sendPIDCommand(uint8_t gain_id, float value) {
    PIDCommand pc; pc.gain_id = gain_id; pc.value = value;
    esp_now_send(bridgeMAC, (uint8_t *)&pc, sizeof(pc));
    Serial.printf("PID> gain=%d val=%.6f\n", gain_id, value);
}

void printHelp() {
    Serial.println("Commands: arm disarm thrust+ thrust- ledon ledoff ping photo startcam stopcam");
    Serial.println("PID: pid <id> <value>   (254=query all, 255=reset integrators)");
}

void setup() {
    Serial.begin(115200); delay(100);
    WiFi.mode(WIFI_STA);  delay(100);

    if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init failed!"); while(true); }
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    memcpy(peerInfo.peer_addr, bridgeMAC, 6);
    peerInfo.channel = 0; peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Failed to add peer!"); while(true); }

    Serial.println("Ground station ready.");
    printHelp();
}

void loop() {
    if (dataReceived)  { printTelemetry();  dataReceived = false; }
    if (pidReceived)   { printPIDConfig();  pidReceived  = false; }

    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        if (input.length() == 0) return;

        if (input == "help") {
            printHelp();
        } else if (input.startsWith("pid ")) {
            int s1 = input.indexOf(' ');
            int s2 = input.indexOf(' ', s1+1);
            if (s2 > 0) {
                sendPIDCommand((uint8_t)input.substring(s1+1, s2).toInt(),
                               input.substring(s2+1).toFloat());
            } else {
                sendPIDCommand((uint8_t)input.substring(s1+1).toInt(), 0.0f);
            }
        } else {
            sendCommand(input.c_str());
        }
    }
}