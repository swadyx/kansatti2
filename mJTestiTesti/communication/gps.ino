// working neo6m code 5hz data with half antenna

#include <TinyGPS++.h>

// --- Configuration ---
// The ESP32 has 3 hardware serial ports. We will use Serial2.
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600

TinyGPSPlus gps;

// Packet to set GPS refresh rate to 5Hz (200ms)
// Class: 0x06, ID: 0x08, Length: 0x06, Rate: 200ms (0x00C8)
const byte SET_5HZ_RATE[] = {
  0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A
};

// If you really want 10Hz, uncomment this (but 9600 baud might struggle)
// const byte SET_10HZ_RATE[] = {
//   0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12
// };

void setup() {
  Serial.begin(115200); // Serial Monitor (Computer)
  
  // Initialize GPS Serial
  Serial2.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  
  Serial.println(F("ESP32 GPS High-Rate Test"));
  Serial.println(F("Configuring GPS for 5Hz updates..."));
  
  // Send the command to change the rate
  // We send it a few times to make sure the GPS catches it
  for(int i=0; i<3; i++){
    Serial2.write(SET_5HZ_RATE, sizeof(SET_5HZ_RATE));
    delay(100);
  }
}

void loop() {
  // Read data from GPS
  while (Serial2.available() > 0) {
    if (gps.encode(Serial2.read())) {
      printFullData();
    }
  }
}

void printFullData() {
  if (gps.location.isValid()) {
    // --- LOCATION ---
    Serial.print(F("LAT: "));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F("  LNG: "));
    Serial.print(gps.location.lng(), 6);
    
    // --- ALTITUDE ---
    Serial.print(F("  ALT: "));
    if (gps.altitude.isValid()) {
      Serial.print(gps.altitude.meters());
      Serial.print(F("m"));
    } else {
      Serial.print(F("INVALID"));
    }

    // --- SPEED ---
    Serial.print(F("  SPEED: "));
    if (gps.speed.isValid()) {
      Serial.print(gps.speed.kmph());
      Serial.print(F("km/h"));
    } else {
      Serial.print(F("INVALID"));
    }

    // --- SATELLITES ---
    Serial.print(F("  SATS: "));
    if (gps.satellites.isValid()) {
      Serial.print(gps.satellites.value());
    } else {
      Serial.print(F("INVALID"));
    }
    
    // --- TIME ---
    Serial.print(F("  TIME: "));
    if (gps.time.isValid()) {
      if (gps.time.hour() < 10) Serial.print(F("0"));
      Serial.print(gps.time.hour());
      Serial.print(F(":"));
      if (gps.time.minute() < 10) Serial.print(F("0"));
      Serial.print(gps.time.minute());
      Serial.print(F(":"));
      if (gps.time.second() < 10) Serial.print(F("0"));
      Serial.print(gps.time.second());
    } else {
      Serial.print(F("INVALID"));
    }

    Serial.println(); // New line
  }
}
