// #include <Wire.h>
// #include <Adafruit_VL53L1X.h>
// #include <TinyGPS++.h>
// #include <DFRobot_C4001.h>
// #include <Adafruit_DRV2605.h>
// #include <WiFi.h>
// #include <HTTPClient.h>
// #include <ArduinoJson.h>
// #include <Adafruit_MAX1704X.h>

// // Configuration
// #define ENABLE_FUEL_GAUGE true   // Set to false to skip fuel gauge initialization

// #define SDA_PIN 6
// #define SCL_PIN 7

// #define XSHUT_PIN_1 4
// #define LED_PIN_1 10

// #define XSHUT_PIN_2 5
// #define LED_PIN_2 11

// #define MMWAVE_RX_PIN 20
// #define MMWAVE_TX_PIN 21
// #define MMWAVE_OUT_PIN 15

// #define GPS_RX_PIN 18
// #define GPS_TX_PIN 19

// const char* ssid = "Verizon-SM-S901U-FA93";
// const char* password = "cacz115/";
// const char* serverURL = "http://10.206.194.152:5000/nav";

// Adafruit_VL53L1X sensor1 = Adafruit_VL53L1X();
// Adafruit_VL53L1X sensor2 = Adafruit_VL53L1X();

// Adafruit_DRV2605 drv1;
// Adafruit_MAX17048 maxlipo;

// HardwareSerial mmWaveSerial(1);
// // Baud rate is 9600 (factory default). Pins passed to constructor so the
// // library's begin() initialises the serial port correctly on ESP32.
// DFRobot_C4001_UART mmWave(&mmWaveSerial, 9600, MMWAVE_RX_PIN, MMWAVE_TX_PIN);

// HardwareSerial gpsSerial(2);
// TinyGPSPlus gps;

// bool mmWaveReady = false;
// bool fuelGaugePresent = false;
// unsigned long lastMotorUpdate1 = 0;
// bool motorState1 = false;
// unsigned long lastLedToggle2 = 0;
// unsigned long lastGPSUpdate = 0;
// unsigned long lastMmWaveUpdate = 0;
// unsigned long lastHTTPSend = 0;
// unsigned long lastBatteryUpdate = 0;
// const unsigned long HTTP_SEND_INTERVAL = 500;
// const unsigned long BATTERY_UPDATE_INTERVAL = 2000;

// int16_t latestDistance1 = -1;
// int16_t latestDistance2 = -1;
// uint8_t latestTargetNum = 0;
// float latestRange = 0.0;
// float latestSpeed = 0.0;
// uint32_t latestEnergy = 0;

// float batteryVoltage = 0.0;
// float batteryPercent = 0.0;
// float batteryChargeRate = 0.0;

// uint32_t measurement_count_1 = 0;
// uint32_t error_count_1 = 0;
// int16_t min_distance_1 = 32767;
// int16_t max_distance_1 = 0;
// uint32_t sum_distance_1 = 0;

// uint32_t measurement_count_2 = 0;
// uint32_t error_count_2 = 0;
// int16_t min_distance_2 = 32767;
// int16_t max_distance_2 = 0;
// uint32_t sum_distance_2 = 0;

// void sendSensorData() {
//   HTTPClient http;
//   http.begin(serverURL);
//   http.addHeader("Content-Type", "application/json");

//   StaticJsonDocument<768> doc;
  
//   JsonObject tof1 = doc.createNestedObject("tof_sensor1");
//   tof1["distance_mm"] = latestDistance1;
//   tof1["distance_cm"] = latestDistance1 / 10.0;
//   tof1["count"] = measurement_count_1;
//   tof1["min"] = min_distance_1;
//   tof1["avg"] = (measurement_count_1 > 0) ? (sum_distance_1 / measurement_count_1) : 0;
//   tof1["max"] = max_distance_1;
  
//   JsonObject tof2 = doc.createNestedObject("tof_sensor2");
//   tof2["distance_mm"] = latestDistance2;
//   tof2["distance_cm"] = latestDistance2 / 10.0;
//   tof2["count"] = measurement_count_2;
//   tof2["min"] = min_distance_2;
//   tof2["avg"] = (measurement_count_2 > 0) ? (sum_distance_2 / measurement_count_2) : 0;
//   tof2["max"] = max_distance_2;
  
//   JsonObject mmwave = doc.createNestedObject("mmwave");
//   mmwave["targets"] = latestTargetNum;
//   mmwave["range_m"] = latestRange;
//   mmwave["speed_ms"] = latestSpeed;
//   mmwave["energy"] = latestEnergy;
  
//   JsonObject gpsData = doc.createNestedObject("gps");
//   gpsData["valid"] = gps.location.isValid();
//   if (gps.location.isValid()) {
//     gpsData["lat"] = gps.location.lat();
//     gpsData["lng"] = gps.location.lng();
//   }
//   if (gps.altitude.isValid()) {
//     gpsData["altitude_m"] = gps.altitude.meters();
//   }
//   if (gps.speed.isValid()) {
//     gpsData["speed_kmh"] = gps.speed.kmph();
//   }
//   gpsData["satellites"] = gps.satellites.value();
  
//   if (fuelGaugePresent) {
//     JsonObject battery = doc.createNestedObject("battery");
//     battery["voltage_v"] = batteryVoltage;
//     battery["percent"] = batteryPercent;
//     battery["charge_rate_percent_hr"] = batteryChargeRate;
//   }
  
//   doc["timestamp"] = millis();

//   String jsonString;
//   serializeJson(doc, jsonString);
  
//   int httpResponseCode = http.POST(jsonString);
  
//   if (httpResponseCode > 0) {
//     Serial.print("[HTTP] POST Response: ");
//     Serial.println(httpResponseCode);
//   } else {
//     Serial.print("[HTTP] POST Error: ");
//     Serial.println(http.errorToString(httpResponseCode));
//   }
  
//   http.end();
// }

// void setup() {
//   Serial.begin(115200);
//   delay(1000);

//   pinMode(LED_PIN_1, OUTPUT);
//   pinMode(LED_PIN_2, OUTPUT);
//   pinMode(XSHUT_PIN_1, OUTPUT);
//   pinMode(XSHUT_PIN_2, OUTPUT);
//   pinMode(MMWAVE_OUT_PIN, INPUT);

//   digitalWrite(XSHUT_PIN_1, LOW);
//   digitalWrite(XSHUT_PIN_2, LOW);
//   delay(10);

//   Serial.println("========================================");
//   Serial.println("   ESP32 Multi-Sensor Test");
//   Serial.println("   TOF + mmWave + GPS");
//   Serial.println("========================================");

//   // Try to recover I2C bus if stuck
//   pinMode(SDA_PIN, OUTPUT);
//   pinMode(SCL_PIN, OUTPUT);
//   for (int i = 0; i < 10; i++) {
//     digitalWrite(SCL_PIN, HIGH);
//     delayMicroseconds(5);
//     digitalWrite(SCL_PIN, LOW);
//     delayMicroseconds(5);
//   }
//   pinMode(SDA_PIN, INPUT_PULLUP);
//   pinMode(SCL_PIN, INPUT_PULLUP);
//   delay(10);

//   Wire.begin(SDA_PIN, SCL_PIN);
//   Wire.setClock(100000);  // Lower speed for stability with multiple devices
//   Serial.println("I2C initialized (100kHz)");

//   // Scan I2C bus
//   Serial.println("Scanning I2C bus...");
//   byte count = 0;
//   bool foundFuelGauge = false;
//   for (byte i = 8; i < 120; i++) {
//     Wire.beginTransmission(i);
//     byte error = Wire.endTransmission();
//     if (error == 0) {
//       Serial.print("  Found device at 0x");
//       if (i < 16) Serial.print("0");
//       Serial.print(i, HEX);
      
//       // Identify known devices
//       if (i == 0x36) {
//         Serial.print(" (MAX17048 Fuel Gauge)");
//         foundFuelGauge = true;
//       }
//       else if (i == 0x29) Serial.print(" (VL53L1X TOF - default addr)");
//       else if (i == 0x30) Serial.print(" (VL53L1X TOF)");
//       else if (i == 0x5A) Serial.print(" (DRV2605L Motor Driver)");
      
//       Serial.println();
//       count++;
//       delay(5);
//     }
//   }
//   Serial.print("Found ");
//   Serial.print(count);
//   Serial.println(" device(s)");
//   if (!foundFuelGauge) {
//     Serial.println("⚠ Note: MAX17048 not detected in scan - will skip initialization");
//   }
//   Serial.println();

//   // Bring up sensor 1 on default address, reassign to 0x30
//   Serial.println("Initializing Sensor 1...");
//   digitalWrite(XSHUT_PIN_1, HIGH);
//   delay(50);
//   Serial.println("Attempting sensor1.begin(0x29)...");
//   if (!sensor1.begin(0x29, &Wire)) {
//     Serial.println("✗ Failed to initialize sensor 1!");
//     while (1) delay(1000);
//   }
//   Serial.println("Setting I2C address to 0x30...");
//   sensor1.VL53L1X_SetI2CAddress(0x30 << 1);
//   delay(50);
//   Serial.println("Attempting sensor1.begin(0x30)...");
//   if (!sensor1.begin(0x30, &Wire)) {
//     Serial.println("✗ Failed to re-initialize sensor 1 at 0x30!");
//     while (1) delay(1000);
//   }
//   Serial.println("✓ VL53L1X Sensor 1 initialized (addr 0x30)");

//   // Bring up sensor 2 on default address 0x29
//   Serial.println("Initializing Sensor 2...");
//   digitalWrite(XSHUT_PIN_2, HIGH);
//   delay(10);
//   if (!sensor2.begin(0x29, &Wire)) {
//     Serial.println("✗ Failed to initialize sensor 2!");
//     while (1) delay(1000);
//   }
//   Serial.println("✓ VL53L1X Sensor 2 initialized (addr 0x29)");

//   Serial.println("Initializing DRV2605L Motor Driver...");
//   if (!drv1.begin(&Wire)) {
//     Serial.println("✗ Failed to initialize DRV2605L 1!");
//     while (1) delay(1000);
//   }
//   drv1.selectLibrary(1);
//   drv1.setMode(DRV2605_MODE_REALTIME);
//   Serial.println("✓ DRV2605L Motor Driver 1 initialized (addr 0x5A)");

// #if ENABLE_FUEL_GAUGE
//   Serial.println("Checking for MAX17048 Fuel Gauge...");
  
//   // Manual I2C check for MAX17048 at 0x36
//   Wire.beginTransmission(0x36);
//   byte fuelGaugeCheck = Wire.endTransmission();
  
//   if (fuelGaugeCheck == 0) {
//     Serial.println("MAX17048 detected at 0x36");
//     delay(100);
    
//     Serial.println("Initializing MAX17048...");
//     if (maxlipo.begin()) {
//       fuelGaugePresent = true;
//       Serial.print("✓ MAX17048 Fuel Gauge initialized | Battery: ");
//       Serial.print(maxlipo.cellPercent(), 1);
//       Serial.print("% | Voltage: ");
//       Serial.print(maxlipo.cellVoltage(), 3);
//       Serial.println("V");
//     } else {
//       Serial.println("⚠ MAX17048 detected but initialization failed");
//       fuelGaugePresent = false;
//     }
//   } else {
//     Serial.println("⚠ MAX17048 Fuel Gauge not found at 0x36 (skipping)");
//     fuelGaugePresent = false;
//   }
// #else
//   Serial.println("⚠ Fuel gauge disabled in configuration (ENABLE_FUEL_GAUGE=false)");
//   fuelGaugePresent = false;
// #endif

//   Serial.println();
//   Serial.println("Initializing TOF sensors for ranging...");
//   sensor1.startRanging();
//   sensor1.setTimingBudget(50);
//   sensor2.startRanging();
//   sensor2.setTimingBudget(50);

//   // mmWave: library begin() opens the serial port using the constructor pins.
//   mmWave.begin();

//   // Switch to speed mode by sending raw UART commands directly.
//   // The library's setSensorMode() hangs because sensorStop() waits for an ACK
//   // ("sensorStop" echo) that the sensor never sends back to us.
//   // We fire the commands without waiting for ACK and let the sensor switch modes.
//   Serial.println("Switching mmWave to speed mode...");
//   while (mmWaveSerial.available()) mmWaveSerial.read();   // flush
//   mmWaveSerial.print("sensorStop");   delay(1500);
//   while (mmWaveSerial.available()) mmWaveSerial.read();
//   mmWaveSerial.print("setRunApp 1");  delay(200);
//   while (mmWaveSerial.available()) mmWaveSerial.read();
//   mmWaveSerial.print("saveConfig");   delay(600);
//   while (mmWaveSerial.available()) mmWaveSerial.read();
//   mmWaveSerial.print("sensorStart");  delay(300);

//   mmWaveReady = true;
//   Serial.println("✓ mmWave sensor ready (speed mode)");

//   gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
//   Serial.println("✓ GPS module initialized");

//   Serial.println("Connecting to WiFi...");
//   WiFi.begin(ssid, password);
//   int wifiAttempts = 0;
//   while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
//     delay(500);
//     Serial.print(".");
//     wifiAttempts++;
//   }
//   if (WiFi.status() == WL_CONNECTED) {
//     Serial.println("\n✓ WiFi connected");
//     Serial.print("IP address: ");
//     Serial.println(WiFi.localIP());
//   } else {
//     Serial.println("\n✗ WiFi connection failed!");
//   }

//   Serial.println("Starting measurements...");
//   Serial.println("========================================");
// }

// void loop() {
//   // --- LED/Motor Control (runs every loop iteration, independent of sensor readings) ---
//   if (latestDistance1 > 0) {  // Only blink if we have a valid reading
//     int blinkInterval1 = map(latestDistance1, 0, 4000, 50, 1000);
//     if (millis() - lastMotorUpdate1 >= blinkInterval1) {
//       motorState1 = !motorState1;
//       digitalWrite(LED_PIN_1, motorState1);
//       drv1.setRealtimeValue(motorState1 ? 127 : 0);
//       lastMotorUpdate1 = millis();
//     }
//   }
  
//   if (latestDistance2 > 0) {  // Only blink if we have a valid reading
//     int blinkInterval2 = map(latestDistance2, 0, 4000, 50, 1000);
//     if (millis() - lastLedToggle2 >= blinkInterval2) {
//       digitalWrite(LED_PIN_2, !digitalRead(LED_PIN_2));
//       lastLedToggle2 = millis();
//     }
//   }

//   // --- TOF Sensor 1 ---
//   if (sensor1.dataReady()) {
//     int16_t distance1 = sensor1.distance();
//     sensor1.clearInterrupt();

//     if (distance1 == -1) {
//       error_count_1++;
//       Serial.print("[S1 ERROR "); Serial.print(error_count_1); Serial.println("] Bad reading");
//     } else {
//       measurement_count_1++;
//       sum_distance_1 += distance1;
//       if (distance1 < min_distance_1) min_distance_1 = distance1;
//       if (distance1 > max_distance_1) max_distance_1 = distance1;
//       int16_t avg_distance_1 = sum_distance_1 / measurement_count_1;
      
//       latestDistance1 = distance1;

//       Serial.print("[S1:"); Serial.print(measurement_count_1); Serial.print("] ");
//       Serial.print(distance1); Serial.print(" mm | ");
//       Serial.print(distance1 / 10.0, 1); Serial.print(" cm");
//       Serial.print(" | Min:"); Serial.print(min_distance_1);
//       Serial.print(" Avg:"); Serial.print(avg_distance_1);
//       Serial.print(" Max:"); Serial.println(max_distance_1);
//     }
//   }

//   // --- TOF Sensor 2 ---
//   if (sensor2.dataReady()) {
//     int16_t distance2 = sensor2.distance();
//     sensor2.clearInterrupt();

//     if (distance2 == -1) {
//       error_count_2++;
//       Serial.print("[S2 ERROR "); Serial.print(error_count_2); Serial.println("] Bad reading");
//     } else {
//       measurement_count_2++;
//       sum_distance_2 += distance2;
//       if (distance2 < min_distance_2) min_distance_2 = distance2;
//       if (distance2 > max_distance_2) max_distance_2 = distance2;
//       int16_t avg_distance_2 = sum_distance_2 / measurement_count_2;
      
//       latestDistance2 = distance2;

//       Serial.print("[S2:"); Serial.print(measurement_count_2); Serial.print("] ");
//       Serial.print(distance2); Serial.print(" mm | ");
//       Serial.print(distance2 / 10.0, 1); Serial.print(" cm");
//       Serial.print(" | Min:"); Serial.print(min_distance_2);
//       Serial.print(" Avg:"); Serial.print(avg_distance_2);
//       Serial.print(" Max:"); Serial.println(max_distance_2);
//     }
//   }

//   // --- mmWave OUT pin (instant hardware signal) ---
//   static bool lastMotionState = false;
//   bool motionDetected = mmWaveReady && digitalRead(MMWAVE_OUT_PIN);
//   if (motionDetected != lastMotionState) {
//     lastMotionState = motionDetected;
//     Serial.print("[mmWave] ");
//     Serial.println(motionDetected ? "Motion DETECTED" : "No motion");
//   }

//   // --- mmWave UART data (every 200 ms) ---
//   if (mmWaveReady && millis() - lastMmWaveUpdate >= 200) {
//     lastMmWaveUpdate = millis();

//     uint8_t targetNum = mmWave.getTargetNumber();
//     if (targetNum > 0) {
//       float range = mmWave.getTargetRange();
//       float speed = mmWave.getTargetSpeed();
//       uint32_t energy = mmWave.getTargetEnergy();
      
//       latestTargetNum = targetNum;
//       latestRange = range;
//       latestSpeed = speed;
//       latestEnergy = energy;

//       Serial.print("[mmWave] Targets: "); Serial.print(targetNum);
//       Serial.print(" | Range: "); Serial.print(range, 2); Serial.print(" m");
//       Serial.print(" | Speed: "); Serial.print(speed, 2); Serial.print(" m/s");
//       Serial.print(speed > 0.05f ? " (approaching)" : speed < -0.05f ? " (receding)" : " (still)");
//       Serial.print(" | Energy: "); Serial.println(energy);
//     } else {
//       // Still report OUT pin state when no UART target
//       static uint32_t lastNoTargetPrint = 0;
//       if (millis() - lastNoTargetPrint >= 1000) {
//         lastNoTargetPrint = millis();
//         Serial.println("[mmWave] No target detected");
//       }
//     }
//   }

//   // --- GPS ---
//   while (gpsSerial.available()) {
//     gps.encode(gpsSerial.read());
//   }

//   if (millis() - lastGPSUpdate >= 5000) {
//     lastGPSUpdate = millis();

//     if (gps.location.isValid() || gps.date.isValid() || gps.time.isValid()) {
//       Serial.println("========== GPS Data ==========");
//       if (gps.location.isValid()) {
//         Serial.print("Position: ");
//         Serial.print(gps.location.lat(), 6); Serial.print(", ");
//         Serial.println(gps.location.lng(), 6);
//       }
//       if (gps.altitude.isValid()) {
//         Serial.print("Altitude: "); Serial.print(gps.altitude.meters(), 1); Serial.println(" m");
//       }
//       if (gps.speed.isValid()) {
//         Serial.print("Speed: "); Serial.print(gps.speed.kmph(), 2); Serial.println(" km/h");
//       }
//       if (gps.date.isValid() && gps.time.isValid()) {
//         Serial.print("DateTime (UTC): ");
//         Serial.print(gps.date.year()); Serial.print("/");
//         if (gps.date.month() < 10) Serial.print("0");
//         Serial.print(gps.date.month()); Serial.print("/");
//         if (gps.date.day() < 10) Serial.print("0");
//         Serial.print(gps.date.day()); Serial.print(" ");
//         if (gps.time.hour() < 10) Serial.print("0");
//         Serial.print(gps.time.hour()); Serial.print(":");
//         if (gps.time.minute() < 10) Serial.print("0");
//         Serial.print(gps.time.minute()); Serial.print(":");
//         if (gps.time.second() < 10) Serial.print("0");
//         Serial.println(gps.time.second());
//       }
//       Serial.print("Satellites: "); Serial.print(gps.satellites.value());
//       Serial.print(" | HDOP: "); Serial.println(gps.hdop.hdop(), 2);
//       Serial.println("==============================");
//     } else {
//       Serial.print("[GPS] Waiting for fix... (Chars: ");
//       Serial.print(gps.charsProcessed());
//       Serial.print(", Sats: ");
//       Serial.print(gps.satellites.value());
//       Serial.println(")");
//     }
//   }

//   // --- Battery Monitor ---
//   if (fuelGaugePresent && millis() - lastBatteryUpdate >= BATTERY_UPDATE_INTERVAL) {
//     lastBatteryUpdate = millis();
    
//     batteryVoltage = maxlipo.cellVoltage();
//     batteryPercent = maxlipo.cellPercent();
//     batteryChargeRate = maxlipo.chargeRate();
    
//     Serial.print("[Battery] ");
//     Serial.print(batteryPercent, 1);
//     Serial.print("% | ");
//     Serial.print(batteryVoltage, 3);
//     Serial.print("V | Rate: ");
//     Serial.print(batteryChargeRate, 2);
//     Serial.println("%/hr");
//   }

//   if (WiFi.status() == WL_CONNECTED && millis() - lastHTTPSend >= HTTP_SEND_INTERVAL) {
//     lastHTTPSend = millis();
//     sendSensorData();
//   }
// }

/*
 * power_system_test.ino
 *
 * Solder verification test for the ESP32-S3 power board.
 * Target: ESP32-S3-WROOM-1-N8  (8 MB flash, NO PSRAM)
 *
 * ── Arduino IDE board settings ──────────────────────────────────────────────
 *   Board            : ESP32S3 Dev Module
 *   Flash Size       : 8MB (64Mb)
 *   Partition Scheme : 8M with spiffs (3MB APP/1.5MB SPIFFS)
 *   PSRAM            : Disabled
 *   CPU Frequency    : 240MHz
 *   Flash Mode       : QIO 80MHz
 *   USB Mode         : Hardware CDC and JTAG   ← native USB via Type-C
 *   Upload Mode      : UART0 / Hardware CDC
 *   Upload Speed     : 921600
 * ────────────────────────────────────────────────────────────────────────────
 *
 * Components under test:
 *   - ESP32-S3-WROOM-1-N8 (boot, serial, RAM, CPU, internal temperature)
 *   - 3.3 V regulator  (implied by successful boot)
 *   - LiPo charger     (implied by battery voltage reading)
 *   - MAX17043/MAX17048 fuel gauge on I2C0 (SDA=GPIO6, SCL=GPIO7, addr 0x36)
 *
 * No external libraries required beyond Wire (built-in).
 * Open Serial Monitor at 115200 baud.
 */

#include <Wire.h>
#include "driver/temperature_sensor.h"   // ESP32-S3 internal temp sensor

// ── Pin definitions (matches schematic I2C0 bus) ─────────────────────────────
#define I2C_SDA       6
#define I2C_SCL       7
#define FUEL_GAUGE_ADDR 0x36

// ── Test result tracking ─────────────────────────────────────────────────────
struct TestResult {
  const char* name;
  bool        passed;
  String      detail;
};

static TestResult results[10];
static uint8_t    resultCount = 0;

static void recordResult(const char* name, bool passed, String detail = "") {
  if (resultCount < 10) {
    results[resultCount++] = {name, passed, detail};
  }
}

// ── Helpers ──────────────────────────────────────────────────────────────────
static void printSeparator(char c = '-', uint8_t len = 50) {
  for (uint8_t i = 0; i < len; i++) Serial.print(c);
  Serial.println();
}

static void printPass(const char* label) {
  Serial.print("  [PASS] "); Serial.println(label);
}

static void printFail(const char* label) {
  Serial.print("  [FAIL] "); Serial.println(label);
}

// ── MAX17043/MAX17048 direct register helpers ─────────────────────────────────
// Returns true and writes raw value if register read succeeded.
static bool fuelReadReg(uint8_t reg, uint16_t &value) {
  Wire.beginTransmission(FUEL_GAUGE_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)FUEL_GAUGE_ADDR, (uint8_t)2) != 2) return false;
  value = ((uint16_t)Wire.read() << 8) | Wire.read();
  return true;
}

// Voltage from VCELL register (0x02): bits [15:4] × 1.25 mV
static float fuelVoltage() {
  uint16_t raw = 0;
  if (!fuelReadReg(0x02, raw)) return -1.0f;
  return (raw >> 4) * 1.25f / 1000.0f;
}

// State-of-charge from SOC register (0x04): high byte = integer %, low byte = 1/256 %
static float fuelSOC() {
  uint16_t raw = 0;
  if (!fuelReadReg(0x04, raw)) return -1.0f;
  return (raw >> 8) + ((raw & 0xFF) / 256.0f);
}

// Version register (0x08) — 0x0011 or 0x0012 for genuine Maxim parts
static uint16_t fuelVersion() {
  uint16_t raw = 0;
  fuelReadReg(0x08, raw);
  return raw;
}

// ── I2C bus recovery (toggle SCL to free a stuck SDA) ────────────────────────
static void recoverI2C() {
  pinMode(I2C_SDA, OUTPUT); digitalWrite(I2C_SDA, HIGH);
  pinMode(I2C_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
    digitalWrite(I2C_SCL, LOW);  delayMicroseconds(5);
  }
  // STOP condition
  digitalWrite(I2C_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(I2C_SCL, HIGH); delayMicroseconds(5);
  digitalWrite(I2C_SDA, HIGH); delayMicroseconds(5);
  pinMode(I2C_SDA, INPUT_PULLUP);
  pinMode(I2C_SCL, INPUT_PULLUP);
  delay(10);
}

// ── Internal temperature sensor ──────────────────────────────────────────────
static temperature_sensor_handle_t tempHandle = NULL;

static bool initTempSensor() {
  temperature_sensor_config_t cfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(10, 80);
  if (temperature_sensor_install(&cfg, &tempHandle) != ESP_OK) return false;
  if (temperature_sensor_enable(tempHandle) != ESP_OK)         return false;
  return true;
}

static float readTempC() {
  float t = -999.0f;
  if (tempHandle) temperature_sensor_get_celsius(tempHandle, &t);
  return t;
}

// ═════════════════════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  delay(1500);   // wait for USB-CDC to enumerate

  printSeparator('=');
  Serial.println("  ESP32-S3 Power Board — Solder Verification Test");
  Serial.println("  Board: ESP32-S3-WROOM-1-N8  (8MB flash, no PSRAM)");
  Serial.println("  Components: Regulator | Charger | Fuel Gauge (MAX17043/48)");
  printSeparator('=');
  Serial.println();

  // ── TEST 1: Serial / USB-CDC ─────────────────────────────────────────────
  Serial.println("[TEST 1] Serial / USB-CDC");
  printPass("Serial output is working — upload succeeded");
  recordResult("Serial/USB-CDC", true, "Upload OK");
  Serial.println();

  // ── TEST 2: CPU, RAM & Flash ─────────────────────────────────────────────
  // N8 variant: 8 MB flash, no PSRAM. Free heap ~320 KB at boot.
  Serial.println("[TEST 2] CPU, RAM & Flash");
  uint32_t cpuMHz    = getCpuFrequencyMhz();
  uint32_t freeHeap  = ESP.getFreeHeap();
  uint32_t flashSzKB = ESP.getFlashChipSize() / 1024;   // KB
  uint32_t psramSz   = ESP.getPsramSize();

  Serial.print("  CPU frequency : "); Serial.print(cpuMHz);     Serial.println(" MHz");
  Serial.print("  Free heap     : "); Serial.print(freeHeap);    Serial.println(" bytes");
  Serial.print("  Flash size    : "); Serial.print(flashSzKB);   Serial.println(" KB");
  Serial.print("  PSRAM size    : "); Serial.print(psramSz);
  Serial.println(psramSz == 0 ? " bytes  (none — correct for N8)" : " bytes");

  bool cpuOk   = (cpuMHz >= 80);
  bool ramOk   = (freeHeap > 100000);
  // N8 has 8 MB (8192 KB) flash; allow ±1 MB for reporting variance
  bool flashOk = (flashSzKB >= 7168 && flashSzKB <= 9216);
  // N8 has NO PSRAM — presence of PSRAM would indicate wrong board config
  bool psramOk = (psramSz == 0);

  if (cpuOk)   printPass("CPU frequency in expected range");
  else         printFail("CPU frequency unexpected");

  if (ramOk)   printPass("Free heap > 100 KB");
  else         printFail("Free heap too low");

  if (flashOk) printPass("Flash ~8 MB detected (N8 variant confirmed)");
  else {
    if (flashSzKB < 7168) printFail("Flash < 7 MB — set Flash Size to 8MB in Arduino IDE");
    else                  printFail("Flash size unexpected — check board selection");
  }

  if (psramOk) printPass("No PSRAM — correct for N8 variant");
  else         printFail("PSRAM detected — set PSRAM to Disabled in Arduino IDE");

  recordResult("CPU/RAM/Flash", cpuOk && ramOk && flashOk && psramOk,
    String(cpuMHz) + "MHz, " + String(freeHeap/1024) + "KB heap, " +
    String(flashSzKB) + "KB flash");
  Serial.println();

  // ── TEST 3: Internal temperature sensor ──────────────────────────────────
  Serial.println("[TEST 3] Internal Temperature Sensor (ESP32-S3)");
  bool tempOk = initTempSensor();
  if (tempOk) {
    delay(100);
    float t = readTempC();
    Serial.print("  Die temperature: "); Serial.print(t, 1); Serial.println(" C");
    bool tempRange = (t > 10.0f && t < 85.0f);
    if (tempRange) printPass("Die temperature in valid range (10–85 C)");
    else           printFail("Die temperature out of range — check power supply");
    recordResult("Internal Temp", tempRange, String(t, 1) + " C");
  } else {
    printFail("Temperature sensor init failed");
    recordResult("Internal Temp", false, "init error");
  }
  Serial.println();

  // ── TEST 4: I2C bus ──────────────────────────────────────────────────────
  Serial.println("[TEST 4] I2C Bus (SDA=GPIO6, SCL=GPIO7)");
  recoverI2C();
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  Serial.println("  I2C initialised at 100 kHz");

  Serial.println("  Scanning I2C addresses 0x08–0x77 ...");
  uint8_t devCount = 0;
  bool foundFuelGauge = false;

  for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
    Wire.beginTransmission(addr);
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
      Serial.print("    0x");
      if (addr < 0x10) Serial.print('0');
      Serial.print(addr, HEX);
      Serial.print("  —  ");
      switch (addr) {
        case 0x36: Serial.print("MAX17043/MAX17048 Fuel Gauge");  foundFuelGauge = true; break;
        case 0x29: Serial.print("VL53L1X ToF (default addr)");    break;
        case 0x30: Serial.print("VL53L1X ToF (reassigned)");      break;
        case 0x5A: Serial.print("DRV2605L Haptic Driver");        break;
        case 0x68: Serial.print("MPU-6050 / DS3231 RTC");         break;
        default:   Serial.print("Unknown device");                break;
      }
      Serial.println();
      devCount++;
      delay(5);
    }
  }

  Serial.print("  Devices found: "); Serial.println(devCount);

  if (devCount > 0) printPass("I2C bus responding");
  else              printFail("No I2C devices found — check SDA/SCL solder joints");

  recordResult("I2C Bus", devCount > 0, String(devCount) + " device(s)");
  Serial.println();

  // ── TEST 5: Fuel gauge detection ─────────────────────────────────────────
  Serial.println("[TEST 5] Fuel Gauge Detection (MAX17043/MAX17048 @ 0x36)");
  if (!foundFuelGauge) {
    printFail("Fuel gauge NOT found at 0x36");
    Serial.println("  Check: VCC→3.3V, GND, SDA→GPIO6, SCL→GPIO7");
    recordResult("Fuel Gauge Detect", false, "not found");
  } else {
    printPass("Fuel gauge found at 0x36");
    uint16_t ver = fuelVersion();
    Serial.print("  VERSION register : 0x"); Serial.println(ver, HEX);
    if (ver == 0x0011 || ver == 0x0012)
      Serial.println("  Genuine Maxim IC confirmed");
    else
      Serial.println("  Non-standard version (clone IC) — direct register reads still work");
    recordResult("Fuel Gauge Detect", true, "ver=0x" + String(ver, HEX));
  }
  Serial.println();

  // ── TEST 6: Fuel gauge readings ──────────────────────────────────────────
  Serial.println("[TEST 6] Battery Voltage & State of Charge");
  if (!foundFuelGauge) {
    Serial.println("  Skipped — fuel gauge not detected");
    recordResult("Battery Readings", false, "skipped");
  } else {
    float voltage = fuelVoltage();
    float soc     = fuelSOC();

    Serial.print("  Voltage : ");
    if (voltage < 0) { Serial.println("READ ERROR"); }
    else { Serial.print(voltage, 3); Serial.println(" V"); }

    Serial.print("  SOC     : ");
    if (soc < 0) { Serial.println("READ ERROR"); }
    else { Serial.print(soc, 1); Serial.println(" %"); }

    bool voltOk = (voltage >= 3.0f && voltage <= 4.35f);
    bool socOk  = (soc >= 0.0f    && soc <= 101.0f);

    if (voltOk) printPass("Battery voltage in LiPo range (3.0–4.35 V)");
    else        printFail("Battery voltage out of range — check battery connection");

    if (socOk)  printPass("SOC reading valid (0–100%)");
    else        printFail("SOC reading invalid");

    recordResult("Battery Readings", voltOk && socOk,
      String(voltage, 3) + "V / " + String(soc, 1) + "%");
  }
  Serial.println();

  // ── TEST 7: 3.3 V regulator (inferred) ───────────────────────────────────
  Serial.println("[TEST 7] 3.3 V Regulator (inferred)");
  Serial.println("  The ESP32 is running → VBAT_SW and 3.3 V rail are present.");
  Serial.println("  If battery voltage above reads ~3.0–4.35 V, regulator input is good.");
  printPass("3.3 V rail present (ESP32 is running)");
  recordResult("3V3 Regulator", true, "inferred from boot");
  Serial.println();

  // ── SUMMARY ──────────────────────────────────────────────────────────────
  printSeparator('=');
  Serial.println("  SOLDER VERIFICATION SUMMARY");
  printSeparator('=');
  uint8_t passed = 0;
  for (uint8_t i = 0; i < resultCount; i++) {
    Serial.print(results[i].passed ? "  [PASS] " : "  [FAIL] ");
    Serial.print(results[i].name);
    if (results[i].detail.length()) {
      Serial.print("  ("); Serial.print(results[i].detail); Serial.print(")");
    }
    Serial.println();
    if (results[i].passed) passed++;
  }
  printSeparator('-');
  Serial.print("  Result: "); Serial.print(passed); Serial.print("/");
  Serial.print(resultCount); Serial.println(" tests passed");
  if (passed == resultCount)
    Serial.println("  *** ALL TESTS PASSED — Board is healthy! ***");
  else
    Serial.println("  *** SOME TESTS FAILED — review FAIL items above ***");
  printSeparator('=');
  Serial.println();
  Serial.println("Entering live monitoring mode (updates every 3 s)...");
  Serial.println("(Reset the board to re-run the full test suite)");
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 3000) return;
  lastPrint = millis();

  float voltage = fuelVoltage();
  float soc     = fuelSOC();
  float tempC   = readTempC();

  Serial.print("[Live] Uptime: ");
  Serial.print(millis() / 1000);
  Serial.print(" s");

  if (voltage >= 0) {
    Serial.print(" | Vbat: ");
    Serial.print(voltage, 3);
    Serial.print(" V");
  }

  if (soc >= 0) {
    Serial.print(" | SOC: ");
    Serial.print(soc, 1);
    Serial.print(" %");
  }

  if (tempC > -99.0f) {
    Serial.print(" | Die: ");
    Serial.print(tempC, 1);
    Serial.print(" C");
  }

  Serial.print(" | Heap: ");
  Serial.print(ESP.getFreeHeap() / 1024);
  Serial.println(" KB");
}
