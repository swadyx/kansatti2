/*
 * CanSat Drone - XIAO ESP32S3 Sense
 * 
 * Board settings:
 *   Board:    XIAO_ESP32S3
 *   PSRAM:    OPI PSRAM    <-- REQUIRED
 *   USB CDC:  Enabled
 */

#include <esp_now.h>
#include <WiFi.h>
#include "esp_wifi.h"
#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
//kameran jotku paskat source: trust me vro
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39
#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13
//toisen osan macci
uint8_t groundStationMAC[] = {0xD4, 0xD4, 0xDA, 0x5A, 0x5A, 0x74};

//pimeet gpt jutut
enum DroneState : uint8_t {
  STATE_OFF     = 0,
  STATE_IDLE    = 1,   // powered on, waiting
  STATE_LAUNCH  = 2,
  STATE_PAUSED  = 3,
  STATE_MOTOR_TEST = 4
};
// nää lähtee datana
typedef struct __attribute__((packed)) {
  uint32_t timestamp;
  uint8_t  packetType;     // 0x01 = telemetry
  uint16_t packetId;
  float    altitude;
  float    temperature;
  float    pressure;
  float    latitude;
  float    longitude;
  float    accelX, accelY, accelZ;
  float    gyroX, gyroY, gyroZ;
  float    batteryVoltage;
  uint8_t  droneState;
  uint8_t  gpsFixQuality;
  int8_t   rssi;
  uint32_t photoCount;
  uint8_t  sdOK;
  uint8_t  camOK;
} TelemetryPacket;
// full GPT mode on en tiiä mikä tää on
// ===== COMMAND STRUCT (from GS) =====
// packetType 0x10 = command
typedef struct __attribute__((packed)) {
  uint8_t  packetType;     // 0x10
  uint8_t  commandType;
  uint8_t  param1;
  uint8_t  param2;
  uint32_t timestamp;
} CommandPacket;

// ===== MESSAGE PACKET (from GS, free text) =====
// packetType 0x20 = text message
// First byte is packetType, rest is null-terminated string

TelemetryPacket telemetry;

// ===== STATE =====
volatile bool commandPending = false;
volatile bool messagePending = false;
CommandPacket  pendingCommand;
char           pendingMessage[200];

uint16_t packetCounter = 0;
uint32_t photoCounter  = 0;

unsigned long lastSendTime  = 0;
unsigned long lastPhotoTime = 0;

const int SEND_INTERVAL_MS  = 500;   // 2 Hz telemetry
const int PHOTO_INTERVAL_MS = 3000;  // photo every 3s

DroneState currentState = STATE_OFF;
bool sdMounted = false;

// ============================================================
//  CAMERA (time-shared with SD — pins 38/39/40 conflict)
// ============================================================
bool startCamera() {
  camera_config_t cfg = {};
  cfg.ledc_channel = LEDC_CHANNEL_0;
  cfg.ledc_timer   = LEDC_TIMER_0;
  cfg.pin_d0       = Y2_GPIO_NUM;
  cfg.pin_d1       = Y3_GPIO_NUM;
  cfg.pin_d2       = Y4_GPIO_NUM;
  cfg.pin_d3       = Y5_GPIO_NUM;
  cfg.pin_d4       = Y6_GPIO_NUM;
  cfg.pin_d5       = Y7_GPIO_NUM;
  cfg.pin_d6       = Y8_GPIO_NUM;
  cfg.pin_d7       = Y9_GPIO_NUM;
  cfg.pin_xclk     = XCLK_GPIO_NUM;
  cfg.pin_pclk     = PCLK_GPIO_NUM;
  cfg.pin_vsync    = VSYNC_GPIO_NUM;
  cfg.pin_href     = HREF_GPIO_NUM;
  cfg.pin_sccb_sda = SIOD_GPIO_NUM;
  cfg.pin_sccb_scl = SIOC_GPIO_NUM;
  cfg.pin_pwdn     = PWDN_GPIO_NUM;
  cfg.pin_reset    = RESET_GPIO_NUM;
  cfg.xclk_freq_hz = 20000000;
  cfg.pixel_format = PIXFORMAT_JPEG;
  cfg.frame_size   = FRAMESIZE_SVGA;
  cfg.jpeg_quality = 10;
  cfg.fb_count     = 1;
  cfg.fb_location  = CAMERA_FB_IN_PSRAM;
  cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  return (esp_camera_init(&cfg) == ESP_OK);
}

void stopCamera() {
  esp_camera_deinit();
}

// laitetaa sd päälle jos ei nii tulee error siihe
bool mountSD() {
  SD_MMC.setPins(7, 9, 8);
  if (!SD_MMC.begin("/sdcard", true)) return false;
  sdMounted = true;
  return true;
}
// pois päält
void unmountSD() {
  SD_MMC.end();
  sdMounted = false;
}
// espi data sent ja received
void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 1) return;

  uint8_t type = data[0];

  if (type == 0x10 && len == sizeof(CommandPacket)) {
    // Command packet
    memcpy((void *)&pendingCommand, data, sizeof(CommandPacket));
    commandPending = true;
  }
  else if (type == 0x20 && len > 1) {
    // Text message — copy string (skip first byte which is packetType)
    int strLen = len - 1;
    if (strLen >= (int)sizeof(pendingMessage)) strLen = sizeof(pendingMessage) - 1;
    memcpy((void *)pendingMessage, data + 1, strLen);
    pendingMessage[strLen] = '\0';
    messagePending = true;
  }
}

// wifi setuppi
bool peerAdded = false;

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_LR);
  esp_wifi_set_max_tx_power(84);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERR] ESP-NOW init failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, groundStationMAC, 6);
  peer.channel = 1;
  peer.encrypt = false;
  peerAdded = (esp_now_add_peer(&peer) == ESP_OK);
  Serial.printf("[ESP-NOW] Peer: %s\n", peerAdded ? "OK" : "FAIL");
}

// tätä pitää jatkaa händlää ne wifin yli tulevat commandid kute vaik launch pause on off yms
void handleCommand(CommandPacket &cmd) {
  Serial.printf("[CMD] type=0x%02X p1=%d p2=%d\n", cmd.commandType, cmd.param1, cmd.param2);

  switch (cmd.commandType) {
    case 0x01:  // ON
      currentState = STATE_IDLE;
      Serial.println("[CMD] >>> POWER ON — IDLE");
      break;

    case 0x02:  // OFF
      currentState = STATE_OFF;
      Serial.println("[CMD] >>> POWER OFF");
      break;

    case 0x03:  // LAUNCH
      currentState = STATE_LAUNCH;
      Serial.println("[CMD] >>> LAUNCH");
      break;

    case 0x04:  // PAUSE
      currentState = STATE_PAUSED;
      Serial.println("[CMD] >>> PAUSED");
      break;

    case 0x05:  // MOTOR TEST
      currentState = STATE_MOTOR_TEST;
      Serial.printf("[CMD] >>> MOTOR TEST motor=%d power=%d%%\n", cmd.param1, cmd.param2);
      // TODO: actually drive motor here
      break;

    default:
      Serial.printf("[CMD] Unknown 0x%02X\n", cmd.commandType);
      break;
  }
}

// ============================================================
//  TASK 1: SEND TELEMETRY (highest priority)
// ============================================================
void taskSendTelemetry() {
  telemetry.timestamp      = millis();
  telemetry.packetType     = 0x01;
  telemetry.packetId       = packetCounter++;
  telemetry.droneState     = (uint8_t)currentState;
  telemetry.photoCount     = photoCounter; // aika turha tuski tarvitaa amount of kuvat se on len(kuvat on sd)
  telemetry.sdOK           = sdMounted ? 1 : 0; // pliis toimi
  telemetry.camOK          = 1;  // we know it works from test
// tähä kerätää oikeet datat, voi muokata viel
  telemetry.altitude       = 0.0;
  telemetry.temperature    = 25.0;
  telemetry.pressure       = 1013.25;
  telemetry.latitude       = 0.0;
  telemetry.longitude      = 0.0;
  telemetry.accelX         = 0.0;
  telemetry.accelY         = 0.0;
  telemetry.accelZ         = 9.81;
  telemetry.gyroX          = 0.0;
  telemetry.gyroY          = 0.0;
  telemetry.gyroZ          = 0.0;
  telemetry.batteryVoltage = 3.7;
  telemetry.gpsFixQuality  = 0;
  telemetry.rssi           = 0;

  if (peerAdded) {
    esp_now_send(groundStationMAC, (uint8_t *)&telemetry, sizeof(telemetry));
  }
}

// joo täs sanotaa muikku ja otetaa se hall of fame kuva
// Returns pointer + length. Caller must free the buffer.
uint8_t *capturePhoto(size_t &outLen) {
  outLen = 0;

  // Unmount SD first (free pins 38/39/40) koska niil tulee riitaa muuten emt vähä pimee
  if (sdMounted) unmountSD();
  delay(10);

  if (!startCamera()) {
    Serial.println("[CAM] Init failed — skipping");
    return nullptr;
  }

  // Discard first (possibly stale) frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (fb) esp_camera_fb_return(fb);

  // Real capture emt miks näinki mut en väitä vastaan
  fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("[CAM] Capture failed — skipping");
    stopCamera();
    return nullptr;
  }

  // Copy to PSRAM so we can release camera before SD access
  uint8_t *buf = (uint8_t *)ps_malloc(fb->len);
  if (!buf) {
    Serial.println("[CAM] PSRAM alloc failed — skipping");
    esp_camera_fb_return(fb);
    stopCamera();
    return nullptr;
  }
  memcpy(buf, fb->buf, fb->len);
  outLen = fb->len;

  esp_camera_fb_return(fb);
  stopCamera();
  delay(10);
  return buf;
}

//  TASK 3: WRITE TO SD (skip on error)
// voi skippaa kosk joskus erroreita ja ei haluta et stuckkaa siihe
void taskWriteSD(uint8_t *imgBuf, size_t imgLen) {
  if (!mountSD()) {
    Serial.println("[SD] Mount failed — skipping write");
    return;
  }

  // Save photo if we have one
  if (imgBuf && imgLen > 0) {
    if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");

    String name = "/photos/img_" + String(photoCounter) + ".jpg";
    File f = SD_MMC.open(name, FILE_WRITE);
    if (f) {
      f.write(imgBuf, imgLen);
      f.close();
      photoCounter++;
      Serial.printf("[PHOTO] %s (%d bytes)\n", name.c_str(), imgLen);
    } else {
      Serial.println("[SD] Photo file open failed — skipping");
    }
  }

  // Append telemetry CSV row
  bool newFile = !SD_MMC.exists("/flight_log.csv");
  File log = SD_MMC.open("/flight_log.csv", FILE_APPEND);
  if (log) {
    if (newFile) {
      log.println("t,pkt,alt,temp,pres,lat,lon,ax,ay,az,gx,gy,gz,bat,state,gps");
    }
    log.printf("%lu,%u,%.2f,%.2f,%.2f,%.6f,%.6f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d\n",
      telemetry.timestamp, telemetry.packetId, telemetry.altitude,
      telemetry.temperature, telemetry.pressure, telemetry.latitude,
      telemetry.longitude, telemetry.accelX, telemetry.accelY, telemetry.accelZ,
      telemetry.gyroX, telemetry.gyroY, telemetry.gyroZ,
      telemetry.batteryVoltage, telemetry.droneState, telemetry.gpsFixQuality);
    log.close();
  } else {
    Serial.println("[SD] Log file open failed — skipping");
  }
}

//  main setup
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("poks DRoNE v2.71");
  Serial.printf("PSRAM: %s (%d bytes)\n", psramFound() ? "YES" : "NO!", ESP.getPsramSize());
  Serial.printf("MAC:   %s\n", WiFi.macAddress().c_str());

  if (!psramFound()) {
    Serial.println("!!! Enable PSRAM in Arduino IDE: Tools -> PSRAM -> OPI PSRAM !!!");
    while (1) delay(1000);
  }

  setupESPNow();

  // Mount SD and prepare folders
  if (mountSD()) {
    if (!SD_MMC.exists("/photos")) SD_MMC.mkdir("/photos");
    Serial.println("[OK] SD mounted");
  } else {
    Serial.println("[WARN] SD not available");
  }

  currentState = STATE_OFF;
  Serial.println("\n[READY] Waiting for commands...\n");
}


//  LOOP
void loop() {
  unsigned long now = millis();

  // --- Handle incoming command (from callback) ---
  if (commandPending) {
    commandPending = false;
    handleCommand(pendingCommand);
  }

  // --- Handle incoming text message (from callback) ---
  // messaget on debukkausta varte
  if (messagePending) {
    messagePending = false;
    Serial.printf("[MSG FROM GS] %s\n", pendingMessage);
  }

  // --- PRIORITY 1: Send telemetry at 2 Hz ---
  // prioriteetti: lähetä dataa --> ota kuva --> kirjota sd
  if (now - lastSendTime >= SEND_INTERVAL_MS) {
    lastSendTime = now;
    taskSendTelemetry();
  }

  if (currentState == STATE_OFF || currentState == STATE_PAUSED) {
    return;
  }

  if (now - lastPhotoTime >= PHOTO_INTERVAL_MS) {
    lastPhotoTime = now;

    size_t imgLen = 0;
    uint8_t *imgBuf = capturePhoto(imgLen);

    taskWriteSD(imgBuf, imgLen); // kirjotetaa kortille
//emt mitä tää tekee
    if (imgBuf) free(imgBuf);
  }
}
