#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define J1_X 32 //vasen
#define J1_Y 33
#define J2_X 25 // oikea
#define J2_Y 26

#define DEADZONE 300
#define CENTER 2048

String dir(int v, const char* neg, const char* pos) {
  if (v > CENTER + DEADZONE) return pos;
  if (v < CENTER - DEADZONE) return neg;
  return "CENTER";
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  int t = analogRead(J1_Y);
  int y = analogRead(J1_X);
  int p = analogRead(J2_Y);
  int r = analogRead(J2_X);

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("CanSat Controller");
  display.println("-----------------");
  display.println("Throttle: " + dir(t, "DOWN", "UP"));
  display.println("Yaw:      " + dir(y, "LEFT", "RIGHT"));
  display.println("Pitch:    " + dir(p, "BACK", "FWD"));
  display.println("Roll:     " + dir(r, "LEFT", "RIGHT"));
  display.display();

  delay(100);
}
