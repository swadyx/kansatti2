#include <Wire.h>

#define BMP388_ADDR     0x76
#define PRESS_XLSB_REG  0x04
#define TEMP_XLSB_REG 0x07
#define STATUS_REG      0x03

struct CalibrationData {
  // Temperature calibration
  double parT1;
  double parT2;
  double parT3;

  // Pressure calibration
  double parP1;
  double parP2;
  double parP3;
  double parP4;
  double parP5;
  double parP6;
  double parP7;
  double parP8;
  double parP9;
  double parP10;
  double parP11;

  float tLin;
};

CalibrationData calib;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(BMP388_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(BMP388_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388_ADDR, (uint8_t)1);
  return Wire.read();
}

bool baroWorks() {
  // register 0x00 is chip id
  if (readRegister(0x00) == 0x50) return true;

  return false;
}

// derived from sections 3.11.1 and 9.1
//https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmp388-ds001.pdf
void readCalibrationData() {

  uint16_t rawT1 = (uint16_t)((readRegister(0x32) << 8) | readRegister(0x31));
  calib.parT1 = rawT1 * 256.0; // division by 2^-8

  uint16_t rawT2 = (uint16_t)((readRegister(0x34) << 8) | readRegister(0x33));
  calib.parT2 = rawT2 / 1073741824.0; // division by 2^30

  calib.parT3 = ((int8_t)readRegister(0x35)) / 281474976710656.0; // division by 2^48

  int16_t rawP1 = (int16_t)((readRegister(0x37) << 8) | readRegister(0x36));
  calib.parP1 = (rawP1 - 16384.0) / 1048576.0; // subtract 2^14 and divide by 2^20

  int16_t rawP2 = (int16_t)((readRegister(0x39) << 8) | readRegister(0x38));
  calib.parP2 = (rawP2 - 16384.0) / 536870912.0; // subtract 2^14 and divide by 2^29

  calib.parP3 = ((int8_t)readRegister(0x3A)) / 4294967296.0; // division by 2^32

  calib.parP4 = ((int8_t)readRegister(0x3B)) / 137438953472.0; // division by 2^37

  uint16_t rawP5 = (uint16_t)((readRegister(0x3D) << 8) | readRegister(0x3C));
  calib.parP5 = rawP5 * 8; // division by 2^-3

  uint16_t rawP6 = (uint16_t)((readRegister(0x3F) << 8) | readRegister(0x3E));
  calib.parP6 = rawP6 / 64.0; // division by 2^6

  calib.parP7 = ((int8_t)readRegister(0x40)) / 256.0; // division by 2^8

  calib.parP8 = ((int8_t)readRegister(0x41)) / 32768.0; // division by 2^15

  int16_t rawP9 = (int16_t)((readRegister(0x43) << 8) | readRegister(0x42));
  calib.parP9 = rawP9 / 281474976710656.0; // division by 2^48

  calib.parP10 = ((int8_t)readRegister(0x44)) / 281474976710656.0; // division by 2^48

  calib.parP11 = ((int8_t)readRegister(0x45)) / 36893488147419103232.0; // division by 2^65
}


void modifyRegister(uint8_t regAddr, uint8_t mask, uint8_t newVal) {
  uint8_t reg = readRegister(regAddr);
  // set all bits, that in mask are 1, to the value in newVal
  writeRegister(regAddr, (reg & ~mask) | (newVal & mask));
}

bool pressureReady() {
  return readRegister(STATUS_REG) & 0x20;  // bit 5 = drdy_press
}

// Wait for pressure data ready
bool tempReady() {
   return readRegister(STATUS_REG) & 0x40;  // bit 6 = drdy_temp
}

// from BMP388_DEV lib
uint32_t readRawPressure() {
  if (!pressureReady()) {
    return 0;
  }

  Wire.beginTransmission(BMP388_ADDR);
  Wire.write(PRESS_XLSB_REG);  // start at XLSB
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388_ADDR, (uint8_t)3);
  if (Wire.available() < 3) return 0; // sanity check

  uint8_t xlsb = Wire.read(); // 0x04
  uint8_t lsb  = Wire.read(); // 0x05
  uint8_t msb  = Wire.read(); // 0x06

  // Assemble 20-bit raw pressure
  int32_t raw = ((int32_t)msb << 16) | ((int32_t)lsb << 8)  | ((int32_t)xlsb);

  return raw;
}

// from BMP388_DEV lib
uint32_t readRawTemperature() {
  if (!tempReady()) {
    return 0;
  }

  Wire.beginTransmission(BMP388_ADDR);
  Wire.write(TEMP_XLSB_REG);  // start at TEMP_XLSB
  Wire.endTransmission(false);
  Wire.requestFrom(BMP388_ADDR, (uint8_t)3);
  if (Wire.available() < 3) return 0;  // sanity check

  uint8_t xlsb = Wire.read(); // 0x07
  uint8_t lsb  = Wire.read(); // 0x08
  uint8_t msb  = Wire.read(); // 0x09

  // Assemble 20-bit raw temperature
  int32_t raw = ((int32_t)msb << 16) | ((int32_t)lsb << 8)  | ((int32_t)xlsb);

  return raw;  // raw 20-bit temperature
}

// from the bosch datasheet section 9.2
float compensateTemperature(uint32_t uncompTemp, struct CalibrationData* calibData) {

  float partialData1;
  float partialData2;

  partialData1 = (float)(uncompTemp - calibData->parT1);
  partialData2 = (float)(partialData1 * calibData->parT2);

  /* Update the compensated temperature in calib structure since this is
   * needed for pressure calculation */
  calibData->tLin = partialData2 + (partialData1 * partialData1) * calibData->parT3;
  /* Returns compensated temperature */

  return calibData->tLin;
}

// from the bosch datasheet section 9.3
float compensatePressure(uint32_t uncompPress, struct CalibrationData* calibData) {

  /* Variable to store the compensated pressure */
  float compPress;

  /* Temporary variables used for compensation */
  float partialData1;
  float partialData2;
  float partialData3;
  float partialData4;
  float partialOut1;
  float partialOut2;

  /* Calibration data */
  partialData1 = calibData->parP6 * calibData->tLin;
  partialData2 = calibData->parP7 * (calibData->tLin * calibData->tLin);
  partialData3 = calibData->parP8 * (calibData->tLin * calibData->tLin * calibData->tLin);
  partialOut1 = calibData->parP5 + partialData1 + partialData2 + partialData3;

  partialData1 = calibData->parP2 * calibData->tLin;
  partialData2 = calibData->parP3 * (calibData->tLin * calibData->tLin);
  partialData3 = calibData->parP4 * (calibData->tLin * calibData->tLin * calibData->tLin);
  partialOut2 = (float)uncompPress * (calibData->parP1 + partialData1 + partialData2 + partialData3);

  partialData1 = (float)uncompPress * (float)uncompPress;
  partialData2 = calibData->parP9 + calibData->parP10 * calibData->tLin;
  partialData3 = partialData1 * partialData2;
  partialData4 = partialData3 + ((float)uncompPress * (float)uncompPress * (float)uncompPress) * calibData->parP11;

  compPress = partialOut1 + partialOut2 + partialData4;

  return compPress;
}

void setup() {
  Serial.begin(115200);

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(A9, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, 1);
  digitalWrite(A9, 1);

  Wire.begin();            // initialize I2C hardware
  Wire.setClock(400000);   // optional, BMP388 supports 400 kHz

  readCalibrationData();

  //PWR_CTRL (0x1B):
  //* Bit 0:                          1
  //* Bit 1:                          1
  //* Bit 5..4:                       11
  modifyRegister(0x1B, 0b00110011, 0b00110011);

  //OSR (0x1C):
  //* Bit 2..0:                      011
  //* Bit 5..3:                      000
  modifyRegister(0x1C, 0b00111111, 0b00000011);

  //ODR (0x1D):
  //* Bit: 4..0:                     00010
  modifyRegister(0x1D, 0b00011111, 0b00000010);

  //IIR 0x1F:
  //* Bit 3..1:                      010
  modifyRegister(0x1F, 0b00001110, 0b00000100);
}

void loop() {
  Serial.println(baroWorks());
  // Serial.println(readRawPressure());
  uint32_t rawTemp = readRawTemperature();
  uint32_t rawPres = readRawPressure();
  // Serial.println(rawTemp);
  Serial.println(compensateTemperature(rawTemp, &calib));
  Serial.println(compensatePressure(rawPres, &calib));
  delay(1000);
}
