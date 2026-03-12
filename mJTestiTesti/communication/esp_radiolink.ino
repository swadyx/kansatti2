#include <esp_now.h>
#include <WiFi.h>
#include "protocol.h"

uint8_t groundMAC[] = {0x08, 0xD1, 0xF9, 0xF4, 0xBF, 0x50};

#define RXD1 44
#define TXD1 43

esp_now_peer_info_t peerInfo;

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    if (len == sizeof(Command)) {
        Command c;
        memcpy(&c, data, sizeof(c));

        Serial.print("Command from ground: ");
        Serial.println(c.cmd);

        sendPacket(Serial1, c);
    }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
    Serial.print("Send: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(921600, SERIAL_8N1, RXD1, TXD1);

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
    SensorData sd;
    if (receivePacket(Serial1, sd)) {
        esp_now_send(groundMAC, (uint8_t *)&sd, sizeof(sd));
    }
}
