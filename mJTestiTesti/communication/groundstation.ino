#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "protocol.h"

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// vasen
#define J1_X 34
#define J1_Y 35
#define SW1 25

// oikia
#define J2_X 32
#define J2_Y 33
#define SW2 26

#define DEADZONE 300
#define CENTER 2048

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint8_t bridgeMAC[] = { 0x98, 0xA3, 0x16, 0xF8, 0x25, 0x40 };

esp_now_peer_info_t peerInfo;

SensorData latest;
bool dataReceived = false;

void onDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
  if (len == sizeof(SensorData)) {
    memcpy(&latest, data, sizeof(latest));
    dataReceived = true;
  }
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Sent to: ");
  for (int i = 0; i < 6; i++) {
    Serial.printf("%02X", info->des_addr[i]);
    if (i < 5) Serial.print(":");
  }
  Serial.print(" -> ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("P: ");
  display.print(latest.pressure, 1);
  display.println(" Pa");

  display.setCursor(0, 12);
  display.print("N: ");
  display.println(latest.num);

  display.setCursor(0, 24);
  display.print("S: ");
  display.println(latest.state);

  display.setCursor(0, 36);
  display.print("Lat: ");
  display.println(latest.lat, 6);

  display.setCursor(0, 48);
  display.print("Lon: ");
  display.println(latest.lon, 6);

  display.display();
}

void sendCommand(const char *cmdStr) {
  Command c;
  memset(&c, 0, sizeof(c));  // zero out the whole struct
  strncpy(c.cmd, cmdStr, sizeof(c.cmd) - 1);
  esp_now_send(bridgeMAC, (uint8_t *)&c, sizeof(c));
  Serial.print("Sent: ");
  Serial.println(cmdStr);
}

void setup() {
  Serial.begin(115200);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed!");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Ground Station");
  display.println("Waiting for data...");
  display.display();

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed!");
    while (true);
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  memcpy(peerInfo.peer_addr, bridgeMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer!");
    while (true);
  }

  Serial.println("Ground station ready");
  Serial.println("Commands: arm, disarm, ping");
  Serial.println("SW1 = arm, SW2 = disarm");
}

void loop() {
// tämmöne juttu controllerille joystick pohjaa -> arm / disarm
/*
  // --- Joystick buttons ---
  static bool sw1Pressed = false;
  static bool sw2Pressed = false;

  if (digitalRead(SW1) == LOW && !sw1Pressed) {
    sw1Pressed = true;
    sendCommand("arm");
  }
  if (digitalRead(SW1) == HIGH) sw1Pressed = false;

  if (digitalRead(SW2) == LOW && !sw2Pressed) {
    sw2Pressed = true;
    sendCommand("disarm");
  }
  if (digitalRead(SW2) == HIGH) sw2Pressed = false;
*/
  // --- Display received data ---
  if (dataReceived) {
    updateDisplay();

    Serial.print("P:");
    Serial.print(latest.pressure);
    Serial.print(" N:");
    Serial.print(latest.num);
    Serial.print(" S:");
    Serial.print(latest.state);
    Serial.print(" Lat:");
    Serial.print(latest.lat, 6);
    Serial.print(" Lon:");
    Serial.println(latest.lon, 6);

    dataReceived = false;
  }

  // --- Serial commands still work too ---
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() > 0) {
      sendCommand(input.c_str());
    }
  }
}
