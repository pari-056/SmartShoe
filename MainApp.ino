#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <time.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <sys/time.h>
#include <WiFiClientSecure.h>
#include <TinyGPSPlus.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// =====================================================
// SMART SAFETY SHOE
// ESP32 + MPU6050 + FSR + SOS BUTTON + BUZZER
// + NEO-6M GPS + FIREBASE REALTIME DATABASE + RENDER ML
//
// Networking: ONE task (netTask) handles all HTTPS.
// Firebase (emergency data) always goes first.
// ML uploads are skipped while an emergency is active.
// =====================================================


// =====================================================
// WIFI  (fill in your password)
// =====================================================

const char* WIFI_SSID = "Pari's S23";
const char* WIFI_PASSWORD = "8144622010";


// =====================================================
// FIREBASE REALTIME DATABASE
// =====================================================

const char* FIREBASE_URL =
  "https://smart-safety-shoe-a6c7e-default-rtdb.asia-southeast1.firebasedatabase.app";


// =====================================================
// RENDER ML  (fill in your NEW API key)
// =====================================================

const char* ML_API_URL = "http://10.63.161.147:8000/predict";
const char* ML_API_KEY = "mysecretkey123";

// Root CA of the Render endpoint (GTS Root R4). No leading spaces.

constexpr uint32_t ML_SAMPLE_PERIOD_US = 20000;   // 50 Hz
constexpr size_t   ML_WINDOW_SIZE = 150;          // 3 seconds
constexpr uint32_t ML_HTTP_TIMEOUT_MS = 6000;
// Send every Nth window (1 = every window, 2 = every ~6 s)
constexpr uint8_t  ML_SEND_EVERY_N_WINDOWS = 1;


// =====================================================
// PINS
// =====================================================

constexpr uint8_t MPU_ADDR = 0x68;

constexpr int SDA_PIN = 25;
constexpr int SCL_PIN = 26;

constexpr int FSR_PIN = 32;
constexpr int BUTTON_PIN = 27;
constexpr int BUZZER_PIN = 23;

// ESP32 RX receives data from GPS TX
constexpr int GPS_RX_PIN = 16;
// ESP32 TX connects to GPS RX
constexpr int GPS_TX_PIN = 17;
constexpr uint32_t GPS_BAUD = 9600;

HardwareSerial GPSSerial(2);
TinyGPSPlus gps;

double latitude = 0.0;
double longitude = 0.0;
bool gpsValid = false;


// =====================================================
// FALL DETECTION THRESHOLDS
// =====================================================

constexpr int FSR_THRESHOLD = 900;

constexpr uint32_t WEIGHT_BEARING_MEMORY = 2000;
constexpr uint32_t PRESSURE_LOSS_TIME = 1000;

constexpr float IMPACT_THRESHOLD = 3.0f;
constexpr float ROTATION_THRESHOLD = 150.0f;

constexpr uint32_t FALL_EVENT_WINDOW = 2000;

constexpr float STILL_ACCEL_MIN = 0.70f;
constexpr float STILL_ACCEL_MAX = 1.30f;
constexpr float STILL_GYRO_THRESHOLD = 30.0f;

constexpr uint32_t STILL_TIME = 1500;
constexpr uint32_t FALL_TIMEOUT = 6000;

constexpr uint32_t BUTTON_DEBOUNCE_TIME = 40;
constexpr uint32_t MPU_RETRY_INTERVAL = 1000;
constexpr uint32_t DISPLAY_INTERVAL = 250;

constexpr uint32_t FIREBASE_INTERVAL = 5000;
uint32_t lastFirebaseUpdate = 0;


// =====================================================
// DATA STRUCTURES (must be defined before use)
// =====================================================

struct MLSample {
  float ax, ay, az, gx, gy, gz;
  uint32_t t;
};

struct MLWindow {
  MLSample samples[ML_WINDOW_SIZE];
  bool locationValid;
  double lat, lon;
};

// Copy a complete JSON snapshot into the queue; never queue a String object.
struct FirebasePayload {
  char json[768];
};

QueueHandle_t firebaseQueue = nullptr;
QueueHandle_t mlQueue = nullptr;


// =====================================================
// MPU SCALE
// =====================================================

constexpr float ACCEL_SCALE = 4096.0f;  // +/-8g
constexpr float GYRO_SCALE = 65.5f;     // +/-500 deg/s


// =====================================================
// SENSOR VALUES
// =====================================================

float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;
float accelMagnitude = 0;
float gyroMagnitude = 0;


// =====================================================
// SYSTEM STATES
// =====================================================

bool mpuReady = false;

bool weightBearing = false;
bool hasWeightBeenDetected = false;

bool pressureLost = false;

bool motionEventDetected = false;
bool possibleFall = false;
bool checkingStillness = false;

bool fallDetected = false;
bool manualSOS = false;
volatile bool emergencyActive = false;   // read by netTask


// =====================================================
// TIMERS
// =====================================================

uint32_t lastWeightBearingTime = 0;
uint32_t pressureLostTime = 0;
uint32_t motionEventTime = 0;
uint32_t possibleFallTime = 0;
uint32_t stillStartTime = 0;
uint32_t lastMPURetry = 0;


// =====================================================
// ML WINDOW COLLECTION (runs in loop(), never in net task)
// =====================================================

MLWindow mlCollect = {};
size_t mlCount = 0;
uint32_t mlNextSampleUs = 0;
uint32_t mlLastSampleMs = 0;
uint32_t mlDroppedWindows = 0;
uint8_t mlWindowCounter = 0;

bool mlCredentialsConfigured() {
  return strlen(ML_API_KEY) > 0;
}

void resetMLWindow() {
  mlCount = 0;
  mlLastSampleMs = 0;
}

void collectMLSample() {

  uint32_t nowMs = millis();

  if (mlCount > 0) {

    uint32_t dt = nowMs - mlLastSampleMs;

    if (dt < 10 || dt > 40) {

      Serial.println(">>> ML: Sampling gap detected");

      resetMLWindow();
    }
  }

  mlLastSampleMs = nowMs;

  MLSample &s = mlCollect.samples[mlCount];

  s.ax = accelX;
  s.ay = accelY;
  s.az = accelZ;

  s.gx = gyroX;
  s.gy = gyroY;
  s.gz = gyroZ;

  // Relative timestamp: 0, 20, 40, ...
  s.t = mlCount * 20;

  mlCount++;

  if (mlCount == ML_WINDOW_SIZE) {
    Serial.println(">>> ML: 150 samples collected successfully");
    mlCollect.locationValid =
      gpsValid &&
      gps.location.isValid() &&
      gps.location.age() < 10000 &&
      isfinite(latitude) &&
      isfinite(longitude) &&
      latitude >= -90 &&
      latitude <= 90 &&
      longitude >= -180 &&
      longitude <= 180;

    mlCollect.lat = latitude;
    mlCollect.lon = longitude;

    mlWindowCounter++;

    if (mlQueue &&
        (mlWindowCounter % ML_SEND_EVERY_N_WINDOWS == 0)) {

      if (uxQueueMessagesWaiting(mlQueue) > 0) {
        mlDroppedWindows++;
      }

      xQueueOverwrite(mlQueue, &mlCollect);
    }

    resetMLWindow();
  }
}

String mlWindowJSON(const MLWindow &w) {
  String json;
  json.reserve(14000);
  json = "{\"samples\":[";
  for (size_t i = 0; i < ML_WINDOW_SIZE; ++i) {
    const MLSample &v = w.samples[i];
    if (i) json += ',';
    json += "{\"ax\":"; json += String(v.ax, 3);
    json += ",\"ay\":"; json += String(v.ay, 3);
    json += ",\"az\":"; json += String(v.az, 3);
    json += ",\"gx\":"; json += String(v.gx, 2);
    json += ",\"gy\":"; json += String(v.gy, 2);
    json += ",\"gz\":"; json += String(v.gz, 2);
    json += ",\"t\":"; json += String(v.t);
    json += '}';
  }
  json += ']';
  if (w.locationValid) {
    json += ",\"location\":{\"latitude\":";
    json += String(w.lat, 6);
    json += ",\"longitude\":";
    json += String(w.lon, 6);
    json += '}';
  }
  json += '}';
  return json;
}


// =====================================================
// NETWORK: ONE TASK, FIREBASE FIRST, ML ONLY WHEN SAFE
// =====================================================

void sendFirebase(const FirebasePayload &p) {
  WiFiClientSecure c;
  c.setInsecure();   // fine for a demo; Firebase certificates rotate
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(5000);

  int code = -100;
  if (http.begin(c, String(FIREBASE_URL) + "/sensorData.json")) {
    http.addHeader("Content-Type", "application/json");
    code = http.PUT(String(p.json));
    http.end();
  }

  Serial.printf(">>> FIREBASE %s: %d\n",
                (code >= 200 && code < 300) ? "UPDATED" : "ERROR", code);
}

void sendML(const MLWindow &w) {

  if (!mlCredentialsConfigured()) {
    static bool warned = false;

    if (!warned) {
      Serial.println(">>> ML: API key not configured");
      warned = true;
    }

    return;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(">>> ML: WiFi disconnected");
    return;
  }

  // Local HTTP connection to MacBook
  WiFiClient client;
  HTTPClient http;

  http.setConnectTimeout(ML_HTTP_TIMEOUT_MS);
  http.setTimeout(ML_HTTP_TIMEOUT_MS);

  if (!http.begin(client, ML_API_URL)) {
    Serial.println(">>> ML: Local server connection failed");
    return;
  }

  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-Key", ML_API_KEY);

  String payload = mlWindowJSON(w);

  Serial.println(">>> ML: Sending 150 samples to local server");

  int code = http.POST(payload);

  if (code == 200) {

    Serial.println(">>> ML: Prediction successful");

    String response = http.getString();

    Serial.println(">>> ML RESPONSE:");
    Serial.println(response);

  } else {

    Serial.printf(">>> ML: HTTP error %d\n", code);

    if (code > 0) {
      Serial.println(http.getString().substring(0, 500));
    }
  }

  http.end();

  Serial.printf(
    ">>> Free heap=%u | Max block=%u\n",
    ESP.getFreeHeap(),
    ESP.getMaxAllocHeap()
  );
}

void netTask(void* parameter) {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t lastRetry = millis();

  FirebasePayload fb;

  MLWindow *win =
    static_cast<MLWindow*>(malloc(sizeof(MLWindow)));

  if (!win) {
    Serial.println(
      ">>> ML: Insufficient heap for window buffer"
    );
  }

  for (;;) {

    // ---------------------------------------------
    // WIFI CONNECTION
    // ---------------------------------------------

    if (WiFi.status() != WL_CONNECTED) {

      if (uint32_t(millis() - lastRetry) >= 10000) {

        lastRetry = millis();

        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
      }

      vTaskDelay(pdMS_TO_TICKS(250));
      continue;
    }

    // ---------------------------------------------
    // PRIORITY 1: FIREBASE
    // ---------------------------------------------

    if (firebaseQueue &&
        xQueueReceive(firebaseQueue, &fb, 0) == pdTRUE) {

      sendFirebase(fb);
    }

    // ---------------------------------------------
    // PRIORITY 2: LOCAL ML PREDICTION
    // ---------------------------------------------

    else if (win && mlQueue && !emergencyActive) {

      if (xQueueReceive(mlQueue, win, 0) == pdTRUE) {

        sendML(*win);

      } else {

        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }

    else {

      vTaskDelay(pdMS_TO_TICKS(20));
    }
  }
}

void startNetworkTask() {
  firebaseQueue = xQueueCreate(1, sizeof(FirebasePayload));
  if (firebaseQueue == nullptr) {
    Serial.println(">>> FIREBASE QUEUE FAILED - local detection continues");
    return;
  }

  mlQueue = xQueueCreate(1, sizeof(MLWindow));
  if (mlQueue == nullptr) {
    Serial.println(">>> ML QUEUE FAILED - Firebase only, local detection continues");
  }

  if (xTaskCreate(netTask, "net", 16384, nullptr, 1, nullptr) != pdPASS) {
    vQueueDelete(firebaseQueue);
    firebaseQueue = nullptr;
    if (mlQueue) {
      vQueueDelete(mlQueue);
      mlQueue = nullptr;
    }
    Serial.println(">>> NET TASK FAILED - local detection continues");
  }
}


// =====================================================
// MPU FUNCTIONS
// =====================================================

bool writeMPURegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission(true) == 0;
}

bool configureMPU() {
  // Wake MPU6050
  if (!writeMPURegister(0x6B, 0x00)) return false;
  delay(50);
  // Accelerometer +/-8g
  if (!writeMPURegister(0x1C, 0x10)) return false;
  // Gyroscope +/-500 degrees/sec
  if (!writeMPURegister(0x1B, 0x08)) return false;
  // Low-pass filter
  if (!writeMPURegister(0x1A, 0x03)) return false;
  delay(50);
  return true;
}

bool recoverMPU() {
  Wire.end();
  if (!Wire.begin(SDA_PIN, SCL_PIN)) return false;
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  return configureMPU();
}

int16_t readMPUWord() {
  uint16_t highByte = static_cast<uint8_t>(Wire.read());
  uint16_t lowByte = static_cast<uint8_t>(Wire.read());
  return static_cast<int16_t>((highByte << 8) | lowByte);
}

bool readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  size_t received = Wire.requestFrom(MPU_ADDR, size_t(14), true);
  if (received != 14 || Wire.available() < 14) return false;

  int16_t ax = readMPUWord();
  int16_t ay = readMPUWord();
  int16_t az = readMPUWord();

  // Ignore temperature
  (void)readMPUWord();

  int16_t gx = readMPUWord();
  int16_t gy = readMPUWord();
  int16_t gz = readMPUWord();

  if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
    return false;
  }

  accelX = ax / ACCEL_SCALE;
  accelY = ay / ACCEL_SCALE;
  accelZ = az / ACCEL_SCALE;

  gyroX = gx / GYRO_SCALE;
  gyroY = gy / GYRO_SCALE;
  gyroZ = gz / GYRO_SCALE;

  accelMagnitude = sqrtf(accelX * accelX + accelY * accelY + accelZ * accelZ);
  gyroMagnitude = sqrtf(gyroX * gyroX + gyroY * gyroY + gyroZ * gyroZ);

  return true;
}


// =====================================================
// GPS
// =====================================================

void readGPS() {
  while (GPSSerial.available() > 0) {
    char c = GPSSerial.read();
    gps.encode(c);
  }

  if (gps.location.isValid()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    gpsValid = true;
  }
}

// =====================================================
// FIREBASE SNAPSHOT (queued for netTask)
// =====================================================

void sendToFirebase(int fsrValue) {

  // Emergency state changes bypass the 1 s throttle so alerts go out at once.
  static bool lastEmergency = false;
  bool changed = (emergencyActive != lastEmergency);

  if (!changed && uint32_t(millis() - lastFirebaseUpdate) < FIREBASE_INTERVAL) {
    return;
  }

  lastEmergency = emergencyActive;
  lastFirebaseUpdate = millis();

  if (firebaseQueue == nullptr) return;

  String json = "{";

  json += "\"accelX\":";
  json += String(accelX, 3);
  json += ",";

  json += "\"accelY\":";
  json += String(accelY, 3);
  json += ",";

  json += "\"accelZ\":";
  json += String(accelZ, 3);
  json += ",";

  json += "\"gyroX\":";
  json += String(gyroX, 3);
  json += ",";

  json += "\"gyroY\":";
  json += String(gyroY, 3);
  json += ",";

  json += "\"gyroZ\":";
  json += String(gyroZ, 3);
  json += ",";

  json += "\"pressure\":";
  json += String(fsrValue);
  json += ",";

  json += "\"latitude\":";
  json += String(latitude, 6);
  json += ",";

  json += "\"longitude\":";
  json += String(longitude, 6);
  json += ",";

  json += "\"gpsValid\":";
  json += gpsValid ? "true" : "false";
  json += ",";

  json += "\"fallDetected\":";
  json += fallDetected ? "true" : "false";
  json += ",";

  json += "\"manualSOS\":";
  json += manualSOS ? "true" : "false";
  json += ",";

  json += "\"emergencyActive\":";
  json += emergencyActive ? "true" : "false";
  json += ",";

  // millis() gives milliseconds since ESP32 startup.
  struct timeval tv;
  gettimeofday(&tv, nullptr);

  uint64_t timestamp = 0;

  if (tv.tv_sec >= 1700000000LL) {
    timestamp =
        (uint64_t)tv.tv_sec * 1000ULL
        + (uint64_t)tv.tv_usec / 1000ULL;
  }

  char timestampBuffer[24];
  snprintf(
      timestampBuffer,
      sizeof(timestampBuffer),
      "%llu",
      (unsigned long long)timestamp
  );

  json += "\"timestamp\":";
  json += timestampBuffer;

  json += "}";

  // Keep the newest pending snapshot if the network is slower than sampling.
  FirebasePayload payload = {};
  if (json.length() >= sizeof(payload.json)) {
    Serial.println(">>> FIREBASE PAYLOAD TOO LARGE");
    return;
  }
  json.toCharArray(payload.json, sizeof(payload.json));
  xQueueOverwrite(firebaseQueue, &payload);
}


// =====================================================
// EMERGENCY OUTPUT
// =====================================================

void updateEmergencyOutput() {
  emergencyActive = fallDetected || manualSOS;
  digitalWrite(BUZZER_PIN, emergencyActive ? HIGH : LOW);
}


// =====================================================
// FALL STATE RESET
// =====================================================

void clearFallCandidate() {
  pressureLost = false;
  motionEventDetected = false;

  possibleFall = false;
  checkingStillness = false;

  pressureLostTime = 0;
  motionEventTime = 0;

  possibleFallTime = 0;
  stillStartTime = 0;
}

void resetDetectionHistory() {
  clearFallCandidate();
  hasWeightBeenDetected = false;
  lastWeightBearingTime = 0;
}


// =====================================================
// RESET / MANUAL SOS / CONFIRM FALL
// =====================================================

void resetEmergencySystem() {
  fallDetected = false;
  manualSOS = false;

  resetDetectionHistory();
  updateEmergencyOutput();

  Serial.println(">>> EMERGENCY RESET");
}

void triggerManualSOS() {
  manualSOS = true;

  resetDetectionHistory();
  updateEmergencyOutput();

  Serial.println(">>> MANUAL SOS - BUZZER ACTIVATED");
}

void confirmFall() {
  fallDetected = true;

  clearFallCandidate();
  updateEmergencyOutput();

  Serial.println(">>> AUTOMATIC FALL CONFIRMED - BUZZER ACTIVATED");
}


// =====================================================
// BUTTON
// =====================================================

void handleButton() {

  static bool lastRawPressed = false;
  static bool stablePressed = false;
  static uint32_t lastChangeTime = 0;

  uint32_t now = millis();

  bool rawPressed = digitalRead(BUTTON_PIN) == LOW;

  if (rawPressed != lastRawPressed) {
    lastRawPressed = rawPressed;
    lastChangeTime = now;
  }

  if (uint32_t(now - lastChangeTime) >= BUTTON_DEBOUNCE_TIME &&
      rawPressed != stablePressed) {

    stablePressed = rawPressed;

    if (stablePressed) {
      if (emergencyActive) {
        resetEmergencySystem();
      } else {
        triggerManualSOS();
      }
    }
  }
}


// =====================================================
// FALL DETECTION
// =====================================================

void detectFall(int fsrValue) {

  uint32_t now = millis();

  weightBearing = fsrValue > FSR_THRESHOLD;

  // ---------------------------------------------------
  // Weight detected
  // ---------------------------------------------------

  if (weightBearing) {

    hasWeightBeenDetected = true;
    lastWeightBearingTime = now;

    if (possibleFall) {

      Serial.println(">>> PRESSURE RETURNED - POSSIBLE FALL CANCELLED");
      clearFallCandidate();

    } else if (pressureLost) {

      pressureLost = false;
      pressureLostTime = 0;

      Serial.println(">>> PRESSURE RESTORED");
    }

  } else if (hasWeightBeenDetected &&
             uint32_t(now - lastWeightBearingTime) > WEIGHT_BEARING_MEMORY) {

    hasWeightBeenDetected = false;
  }

  bool recentlyWeightBearing =
    hasWeightBeenDetected &&
    uint32_t(now - lastWeightBearingTime) <= WEIGHT_BEARING_MEMORY;

  // ---------------------------------------------------
  // Pressure loss
  // ---------------------------------------------------

  if (!weightBearing && recentlyWeightBearing && !pressureLost) {

    pressureLost = true;
    pressureLostTime = now;

    Serial.println(">>> PRESSURE LOST");
  }

  // ---------------------------------------------------
  // Expire old MPU event
  // ---------------------------------------------------

  if (motionEventDetected &&
      !possibleFall &&
      uint32_t(now - motionEventTime) > FALL_EVENT_WINDOW) {

    motionEventDetected = false;
  }

  // ---------------------------------------------------
  // MPU fall-like event
  // ---------------------------------------------------

  bool motion =
    accelMagnitude > IMPACT_THRESHOLD ||
    gyroMagnitude > ROTATION_THRESHOLD;

  if (motion && recentlyWeightBearing && !possibleFall) {

    if (!motionEventDetected) {
      Serial.println(">>> FALL-LIKE MPU EVENT");
    }

    motionEventDetected = true;
    motionEventTime = now;
  }

  // ---------------------------------------------------
  // Sustained pressure loss
  // ---------------------------------------------------

  bool sustainedPressureLoss =
    pressureLost &&
    uint32_t(now - pressureLostTime) >= PRESSURE_LOSS_TIME;

  // ---------------------------------------------------
  // Combine pressure + MPU event
  // ---------------------------------------------------

  if (!possibleFall && motionEventDetected && sustainedPressureLoss) {

    uint32_t motionAge = now - motionEventTime;
    uint32_t pressureAge = now - pressureLostTime;

    uint32_t difference =
      motionAge > pressureAge
        ? motionAge - pressureAge
        : pressureAge - motionAge;

    if (difference <= FALL_EVENT_WINDOW) {

      possibleFall = true;
      possibleFallTime = now;
      checkingStillness = false;

      Serial.println(">>> POSSIBLE FALL - CHECKING STILLNESS");
    }
  }

  // ---------------------------------------------------
  // Stillness verification
  // ---------------------------------------------------

  if (possibleFall) {

    if (uint32_t(now - possibleFallTime) >= FALL_TIMEOUT) {

      clearFallCandidate();

      Serial.println(">>> POSSIBLE FALL TIMED OUT");

      return;
    }

    bool still =
      accelMagnitude >= STILL_ACCEL_MIN &&
      accelMagnitude <= STILL_ACCEL_MAX &&
      gyroMagnitude < STILL_GYRO_THRESHOLD &&
      !weightBearing;

    if (!still) {

      checkingStillness = false;

    } else if (!checkingStillness) {

      checkingStillness = true;
      stillStartTime = now;

    } else if (uint32_t(now - stillStartTime) >= STILL_TIME) {

      confirmFall();
    }

  } else if (pressureLost &&
             !recentlyWeightBearing &&
             !motionEventDetected &&
             uint32_t(now - pressureLostTime) > FALL_EVENT_WINDOW) {

    clearFallCandidate();
  }
}


// =====================================================
// SERIAL MONITOR
// =====================================================

void printDiagnostics(int fsrValue, bool mpuOK) {

  static uint32_t lastDisplay = 0;

  uint32_t now = millis();

  if (uint32_t(now - lastDisplay) < DISPLAY_INTERVAL) {
    return;
  }

  lastDisplay = now;

  Serial.print("FSR=");
  Serial.print(fsrValue);

  Serial.print(" | Pressure=");
  Serial.print(weightBearing ? "YES" : "NO");

  if (mpuOK) {
    Serial.print(" | Accel(g)=");
    Serial.print(accelMagnitude, 2);

    Serial.print(" | Gyro(deg/s)=");
    Serial.print(gyroMagnitude, 2);
  } else {
    Serial.print(" | MPU INVALID - automatic detection unavailable");
  }

  if (gpsValid) {
    Serial.print(" | GPS=");
    Serial.print(latitude, 6);
    Serial.print(",");
    Serial.print(longitude, 6);
  } else {
    Serial.print(" | GPS=NO FIX");
  }

  Serial.print(" | State=");

  if (manualSOS) {
    Serial.print("MANUAL SOS");
  } else if (fallDetected) {
    Serial.print("FALL CONFIRMED");
  } else if (!mpuOK) {
    Serial.print("MPU ERROR");
  } else if (possibleFall) {
    Serial.print("VERIFYING FALL");
  } else if (pressureLost) {
    Serial.print("PRESSURE LOST");
  } else if (!hasWeightBeenDetected) {
    Serial.print("WAITING FOR WEIGHT");
  } else {
    Serial.print("NORMAL");
  }

  Serial.print(" | Buzzer=");
  Serial.print(emergencyActive ? "ON" : "OFF");

  Serial.print(" | WiFi=");
  Serial.println(WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
}


// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("========================================");
  Serial.println("SMART SAFETY SHOE");
  Serial.println("========================================");

  // FSR
  pinMode(FSR_PIN, INPUT);
  analogReadResolution(12);

  // SOS BUTTON
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // BUZZER
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // GPS
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS serial started");
  Serial.println("Waiting for NEO-6M GPS data...");

  // MPU6050
  if (Wire.begin(SDA_PIN, SCL_PIN)) {
    Wire.setClock(100000);
    Wire.setTimeOut(50);
    mpuReady = configureMPU();
  }

  lastMPURetry = millis();

  if (mpuReady) {
    Serial.println("MPU6050 ready");
  } else {
    Serial.println("MPU unavailable - will retry");
  }

  // Network (clock sync is needed for HTTPS certificate validation)
  configTime(0, 0, "pool.ntp.org", "time.google.com");
  startNetworkTask();
  mlNextSampleUs = micros();

  Serial.println();
  Serial.println("System initialization complete.");
  Serial.println("Press SOS button to activate emergency.");
  Serial.println("Press it again to reset emergency.");
  Serial.println("========================================");
}


// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  handleButton();
  readGPS();

  uint32_t nowUs = micros();
  if (int32_t(nowUs - mlNextSampleUs) < 0) {
    delay(1);
    return;
  }

  if (uint32_t(nowUs - mlNextSampleUs) > ML_SAMPLE_PERIOD_US) {
    resetMLWindow();
    mlNextSampleUs = nowUs;
  }
  mlNextSampleUs += ML_SAMPLE_PERIOD_US;

  readGPS();
  handleButton();

  if (!mpuReady &&
      uint32_t(millis() - lastMPURetry) >= MPU_RETRY_INTERVAL) {

    mpuReady = recoverMPU();
    lastMPURetry = millis();

    if (mpuReady) {
      Serial.println(">>> MPU RECOVERED");
    } else {
      Serial.println(">>> MPU RETRY FAILED");
    }
  }

  bool mpuOK = mpuReady && readMPU();

  if (!mpuOK) {

    if (mpuReady) {
      Serial.println(">>> MPU READ FAILED");
      mpuReady = false;
      lastMPURetry = millis();
    }

    resetDetectionHistory();
    resetMLWindow();
  } else {
    collectMLSample();
  }

  int fsrValue = analogRead(FSR_PIN);

  weightBearing = fsrValue > FSR_THRESHOLD;

  handleButton();

  if (mpuOK && !emergencyActive) {
    detectFall(fsrValue);
  }

  updateEmergencyOutput();

  readGPS();

  printDiagnostics(fsrValue, mpuOK);

  sendToFirebase(fsrValue);

  readGPS();
}
