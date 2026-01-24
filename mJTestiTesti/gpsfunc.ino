#include "definitions.h"

void setup_neo6m() {
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // GPS connected to Serial2 (UART2)
}

GPSData get_gps_data() {
  GPSData data;
  data.dataUpdated = false;  // Default to no new data

  // Process all available bytes from Serial2 (the GPS serial port)
  while (Serial2.available()) {
    gps.encode(Serial2.read());
  }

  // Check if new GPS location data has been updated
  if (gps.location.isUpdated()) {
    data.dataUpdated = true;
    data.latitude = gps.location.lat();
    data.longitude = gps.location.lng();
    data.altitude = gps.altitude.meters();
    data.speed = gps.speed.kmph();
    data.satellites = gps.satellites.value();
    data.hour = gps.time.hour();
    data.minute = gps.time.minute();
    data.second = gps.time.second();
  }
  
  return data;
}
