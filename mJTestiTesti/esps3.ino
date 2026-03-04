// saa uartilla pressuren teensyltä

#define RXD1 44
#define TXD1 43

void setup() {
  Serial.begin(115200);
  Serial1.begin(900000, SERIAL_8N1, RXD1, TXD1);
}

void loop() {
  if (Serial1.available()) {
    String line = Serial1.readStringUntil('\n'); // lukee koko rivin
    line.trim();
    Serial.println(line); 
  }
}
