#include <esp_now.h>
#include <WiFi.h>

uint8_t groundMAC[] = {0x08, 0xD1, 0xF9, 0xF4, 0xBF, 0x50};

#define RXD1 44
#define TXD1 43

struct SensorData {
    float pressure;
    int num;
    char state[16];
    float lat;
    float lon;
};

struct Command {
    char cmd[32];
};

esp_now_peer_info_t peerInfo;

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len == sizeof(Command)) {
        Command c;
        memcpy(&c, data, sizeof(c));

        Serial.print("Command from ground: ");
        Serial.println(c.cmd);

        Serial1.println(c.cmd);
    }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("Send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(900000, SERIAL_8N1, RXD1, TXD1);

    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed!");
        while (true);
    }

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onDataRecv);

    memcpy(peerInfo.peer_addr, groundMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer!");
        while (true);
    }

    Serial.println("Bridge ready");
}

void loop() {
    if (Serial1.available()) {
        String line = Serial1.readStringUntil('\n');
        line.trim();

        if (line.length() == 0) return;

        int idx1 = line.indexOf(',');
        int idx2 = line.indexOf(',', idx1 + 1);
        int idx3 = line.indexOf(',', idx2 + 1);
        int idx4 = line.indexOf(',', idx3 + 1);

        if (idx1 < 0 || idx2 < 0 || idx3 < 0 || idx4 < 0) return;

        SensorData data;
        data.pressure = line.substring(0, idx1).toFloat();
        data.num      = line.substring(idx1 + 1, idx2).toInt();
        strncpy(data.state, line.substring(idx2 + 1, idx3).c_str(), sizeof(data.state));
        data.state[sizeof(data.state) - 1] = '\0';
        data.lat      = line.substring(idx3 + 1, idx4).toFloat();
        data.lon      = line.substring(idx4 + 1).toFloat();

        esp_now_send(groundMAC, (uint8_t *)&data, sizeof(data));
    }
}
