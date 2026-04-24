#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <TinyGPS++.h>
#include <DFRobot_C4001.h>
#include <Adafruit_DRV2605.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MAX1704X.h>

// Configuration
#define ENABLE_FUEL_GAUGE true   // Set to false to skip fuel gauge initialization
#define ENABLE_HTTP_POST false   // Set to true when your server is running at 10.206.194.152:5000

#define SDA_PIN 6
#define SCL_PIN 7

#define XSHUT_PIN_1 4
#define LED_PIN_1 10

#define XSHUT_PIN_2 5
#define LED_PIN_2 11

#define MMWAVE_RX_PIN 20
#define MMWAVE_TX_PIN 21
#define MMWAVE_OUT_PIN 15

#define GPS_RX_PIN 18
#define GPS_TX_PIN 19

const char* ssid = "Verizon-SM-S901U-FA93";
const char* password = "cacz115/";
const char* serverURL = "http://10.206.194.152:5000/nav";

Adafruit_VL53L1X sensor1 = Adafruit_VL53L1X();
Adafruit_VL53L1X sensor2 = Adafruit_VL53L1X();

Adafruit_DRV2605 drv1;
Adafruit_MAX17048 maxlipo;  // Works for both MAX17043 and MAX17048

HardwareSerial mmWaveSerial(1);
// Baud rate is 9600 (factory default). Pins passed to constructor so the
// library's begin() initialises the serial port correctly on ESP32.
DFRobot_C4001_UART mmWave(&mmWaveSerial, 9600, MMWAVE_RX_PIN, MMWAVE_TX_PIN);

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

bool mmWaveReady = false;
bool fuelGaugePresent = false;
unsigned long lastMotorUpdate1 = 0;
bool motorState1 = false;
unsigned long lastLedToggle2 = 0;
unsigned long lastGPSUpdate = 0;
unsigned long lastMmWaveUpdate = 0;
unsigned long lastHTTPSend = 0;
unsigned long lastBatteryUpdate = 0;
const unsigned long HTTP_SEND_INTERVAL = 500;
const unsigned long BATTERY_UPDATE_INTERVAL = 2000;

int16_t latestDistance1 = -1;
int16_t latestDistance2 = -1;
uint8_t latestTargetNum = 0;
float latestRange = 0.0;
float latestSpeed = 0.0;
uint32_t latestEnergy = 0;

float batteryVoltage = 0.0;
float batteryPercent = 0.0;

uint32_t measurement_count_1 = 0;
uint32_t error_count_1 = 0;
int16_t min_distance_1 = 32767;
int16_t max_distance_1 = 0;
uint32_t sum_distance_1 = 0;

uint32_t measurement_count_2 = 0;
uint32_t error_count_2 = 0;
int16_t min_distance_2 = 32767;
int16_t max_distance_2 = 0;
uint32_t sum_distance_2 = 0;

// Direct register read functions for MAX17043 (for clone chips)
float readBatteryVoltage() {
  Wire.beginTransmission(0x36);
  Wire.write(0x02);  // VCELL register
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(0x36, 2);
    if (Wire.available() >= 2) {
      uint16_t vcell = (Wire.read() << 8) | Wire.read();
      return (vcell >> 4) * 1.25 / 1000.0;  // Convert to volts
    }
  }
  return 0.0;
}

float readBatteryPercent() {
  Wire.beginTransmission(0x36);
  Wire.write(0x04);  // SOC (State of Charge) register
  if (Wire.endTransmission(false) == 0) {
    Wire.requestFrom(0x36, 2);
    if (Wire.available() >= 2) {
      uint16_t soc = (Wire.read() << 8) | Wire.read();
      return (soc >> 8) + ((soc & 0x00FF) / 256.0);  // Percent with fractional part
    }
  }
  return 0.0;
}

void sendSensorData() {
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/json");

  StaticJsonDocument<768> doc;
  
  JsonObject tof1 = doc.createNestedObject("tof_sensor1");
  tof1["distance_mm"] = latestDistance1;
  tof1["distance_cm"] = latestDistance1 / 10.0;
  tof1["count"] = measurement_count_1;
  tof1["min"] = min_distance_1;
  tof1["avg"] = (measurement_count_1 > 0) ? (sum_distance_1 / measurement_count_1) : 0;
  tof1["max"] = max_distance_1;
  
  JsonObject tof2 = doc.createNestedObject("tof_sensor2");
  tof2["distance_mm"] = latestDistance2;
  tof2["distance_cm"] = latestDistance2 / 10.0;
  tof2["count"] = measurement_count_2;
  tof2["min"] = min_distance_2;
  tof2["avg"] = (measurement_count_2 > 0) ? (sum_distance_2 / measurement_count_2) : 0;
  tof2["max"] = max_distance_2;
  
  JsonObject mmwave = doc.createNestedObject("mmwave");
  mmwave["targets"] = latestTargetNum;
  mmwave["range_m"] = latestRange;
  mmwave["speed_ms"] = latestSpeed;
  mmwave["energy"] = latestEnergy;
  
  JsonObject gpsData = doc.createNestedObject("gps");
  gpsData["valid"] = gps.location.isValid();
  if (gps.location.isValid()) {
    gpsData["lat"] = gps.location.lat();
    gpsData["lng"] = gps.location.lng();
  }
  if (gps.altitude.isValid()) {
    gpsData["altitude_m"] = gps.altitude.meters();
  }
  if (gps.speed.isValid()) {
    gpsData["speed_kmh"] = gps.speed.kmph();
  }
  gpsData["satellites"] = gps.satellites.value();
  
  if (fuelGaugePresent) {
    JsonObject battery = doc.createNestedObject("battery");
    battery["voltage_v"] = batteryVoltage;
    battery["percent"] = batteryPercent;
  }
  
  doc["timestamp"] = millis();

  String jsonString;
  serializeJson(doc, jsonString);
  
  int httpResponseCode = http.POST(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.print("[HTTP] POST Response: ");
    Serial.println(httpResponseCode);
  } else {
    Serial.print("[HTTP] POST Error: ");
    Serial.println(http.errorToString(httpResponseCode));
  }
  
  http.end();
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(XSHUT_PIN_1, OUTPUT);
  pinMode(XSHUT_PIN_2, OUTPUT);
  pinMode(MMWAVE_OUT_PIN, INPUT);

  digitalWrite(XSHUT_PIN_1, LOW);
  digitalWrite(XSHUT_PIN_2, LOW);
  delay(10);

  Serial.println("========================================");
  Serial.println("   ESP32 Multi-Sensor Test");
  Serial.println("   TOF + mmWave + GPS");
  Serial.println("========================================");

  // Try to recover I2C bus if stuck
  pinMode(SDA_PIN, OUTPUT);
  pinMode(SCL_PIN, OUTPUT);
  for (int i = 0; i < 10; i++) {
    digitalWrite(SCL_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(SCL_PIN, LOW);
    delayMicroseconds(5);
  }
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);  // Lower speed for stability with multiple devices
  Serial.println("I2C initialized (100kHz)");

  // Scan I2C bus
  Serial.println("Scanning I2C bus...");
  byte count = 0;
  bool foundFuelGauge = false;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("  Found device at 0x");
      if (i < 16) Serial.print("0");
      Serial.print(i, HEX);
      
      // Identify known devices
      if (i == 0x36) {
        Serial.print(" (MAX17043 Fuel Gauge)");
        foundFuelGauge = true;
      }
      else if (i == 0x29) Serial.print(" (VL53L1X TOF - default addr)");
      else if (i == 0x30) Serial.print(" (VL53L1X TOF)");
      else if (i == 0x5A) Serial.print(" (DRV2605L Motor Driver)");
      
      Serial.println();
      count++;
      delay(5);
    }
  }
  Serial.print("Found ");
  Serial.print(count);
  Serial.println(" device(s)");
  if (!foundFuelGauge) {
    Serial.println("⚠ Note: MAX17043 not detected in scan - will skip initialization");
  }
  Serial.println();

  // Bring up sensor 1 on default address, reassign to 0x30
  Serial.println("Initializing Sensor 1...");
  digitalWrite(XSHUT_PIN_1, HIGH);
  delay(50);
  Serial.println("Attempting sensor1.begin(0x29)...");
  if (!sensor1.begin(0x29, &Wire)) {
    Serial.println("✗ Failed to initialize sensor 1!");
    while (1) delay(1000);
  }
  Serial.println("Setting I2C address to 0x30...");
  sensor1.VL53L1X_SetI2CAddress(0x30 << 1);
  delay(50);
  Serial.println("Attempting sensor1.begin(0x30)...");
  if (!sensor1.begin(0x30, &Wire)) {
    Serial.println("✗ Failed to re-initialize sensor 1 at 0x30!");
    while (1) delay(1000);
  }
  Serial.println("✓ VL53L1X Sensor 1 initialized (addr 0x30)");

  // Bring up sensor 2 on default address 0x29
  Serial.println("Initializing Sensor 2...");
  digitalWrite(XSHUT_PIN_2, HIGH);
  delay(10);
  if (!sensor2.begin(0x29, &Wire)) {
    Serial.println("✗ Failed to initialize sensor 2!");
    while (1) delay(1000);
  }
  Serial.println("✓ VL53L1X Sensor 2 initialized (addr 0x29)");

  Serial.println("Initializing DRV2605L Motor Driver...");
  if (!drv1.begin(&Wire)) {
    Serial.println("✗ Failed to initialize DRV2605L 1!");
    while (1) delay(1000);
  }
  drv1.selectLibrary(1);
  drv1.setMode(DRV2605_MODE_REALTIME);
  Serial.println("✓ DRV2605L Motor Driver 1 initialized (addr 0x5A)");

#if ENABLE_FUEL_GAUGE
  Serial.println("Checking for MAX17043 Fuel Gauge...");
  delay(250);  // Let I2C bus settle after previous device inits

  // Retry loop: the chip can take a moment to respond after power-on
  byte fuelGaugeCheck = 1;
  for (int attempt = 0; attempt < 5 && fuelGaugeCheck != 0; attempt++) {
    Wire.beginTransmission(0x36);
    fuelGaugeCheck = Wire.endTransmission();
    if (fuelGaugeCheck != 0) {
      Serial.print("  0x36 not responding (attempt ");
      Serial.print(attempt + 1);
      Serial.println("/5), retrying...");
      delay(200);
    }
  }

  if (fuelGaugeCheck == 0) {
    Serial.println("MAX17043 detected at 0x36");
    delay(100);

    // Read VERSION register (0x08) to confirm chip identity
    Wire.beginTransmission(0x36);
    Wire.write(0x08);  // VERSION register
    byte verError = Wire.endTransmission(false);
    if (verError == 0) {
      Wire.requestFrom(0x36, 2);
      if (Wire.available() >= 2) {
        uint16_t version = (Wire.read() << 8) | Wire.read();
        Serial.print("  Chip Version: 0x");
        Serial.println(version, HEX);
        
        if (version != 0x0011 && version != 0x0012) {
          Serial.println("  ⚠ Non-standard version detected (possibly clone chip)");
          Serial.println("  → Will use direct register access instead of library");
        }
      }
    }

    // Read VCELL register (0x02) to check battery voltage
    Wire.beginTransmission(0x36);
    Wire.write(0x02);  // VCELL register
    byte vcellError = Wire.endTransmission(false);
    if (vcellError == 0) {
      Wire.requestFrom(0x36, 2);
      if (Wire.available() >= 2) {
        uint16_t vcell = (Wire.read() << 8) | Wire.read();
        float voltage = (vcell >> 4) * 1.25 / 1000.0;  // Convert to volts
        Serial.print("  Raw VCELL reading: ");
        Serial.print(voltage, 3);
        Serial.println(" V");
        
        if (voltage < 2.5) {
          Serial.println("  ⚠ WARNING: Battery voltage too low or not connected!");
          fuelGaugePresent = false;
        } else {
          // Battery is connected and readable - we can use direct register access
          fuelGaugePresent = true;
          Serial.println("✓ MAX17043-compatible chip initialized (direct register mode)");
        }
      }
    }
    
    if (!fuelGaugePresent) {
      Serial.println("⚠ Failed to read battery voltage from chip");
    }
  } else {
    Serial.println("⚠ MAX17043 Fuel Gauge not found at 0x36 after 5 attempts");
    Serial.println("  → Check wiring: SDA=6, SCL=7, VCC=3.3V, GND");
    fuelGaugePresent = false;
  }
#else
  Serial.println("⚠ Fuel gauge disabled in configuration (ENABLE_FUEL_GAUGE=false)");
  fuelGaugePresent = false;
#endif

  Serial.println();
  Serial.println("Initializing TOF sensors for ranging...");
  sensor1.startRanging();
  sensor1.setTimingBudget(50);
  sensor2.startRanging();
  sensor2.setTimingBudget(50);

  // mmWave: library begin() opens the serial port using the constructor pins.
  mmWave.begin();

  // Switch to speed mode by sending raw UART commands directly.
  // The library's setSensorMode() hangs because sensorStop() waits for an ACK
  // ("sensorStop" echo) that the sensor never sends back to us.
  // We fire the commands without waiting for ACK and let the sensor switch modes.
  Serial.println("Switching mmWave to speed mode...");
  while (mmWaveSerial.available()) mmWaveSerial.read();   // flush
  mmWaveSerial.print("sensorStop");   delay(1500);
  while (mmWaveSerial.available()) mmWaveSerial.read();
  mmWaveSerial.print("setRunApp 1");  delay(200);
  while (mmWaveSerial.available()) mmWaveSerial.read();
  mmWaveSerial.print("saveConfig");   delay(600);
  while (mmWaveSerial.available()) mmWaveSerial.read();
  mmWaveSerial.print("sensorStart");  delay(300);

  mmWaveReady = true;
  Serial.println("✓ mmWave sensor ready (speed mode)");

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("✓ GPS module initialized");

#if ENABLE_HTTP_POST
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n✓ WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n✗ WiFi connection failed - HTTP will not work");
  }
#else
  Serial.println("ℹ HTTP posting disabled (set ENABLE_HTTP_POST to true to enable)");
#endif

  Serial.println("Starting measurements...");
  Serial.println("========================================");
}

void loop() {
  // --- LED/Motor Control (runs every loop iteration, independent of sensor readings) ---
  if (latestDistance1 > 0) {  // Only blink if we have a valid reading
    int blinkInterval1 = map(latestDistance1, 0, 4000, 50, 1000);
    if (millis() - lastMotorUpdate1 >= blinkInterval1) {
      motorState1 = !motorState1;
      digitalWrite(LED_PIN_1, motorState1);
      drv1.setRealtimeValue(motorState1 ? 127 : 0);
      lastMotorUpdate1 = millis();
    }
  }
  
  if (latestDistance2 > 0) {  // Only blink if we have a valid reading
    int blinkInterval2 = map(latestDistance2, 0, 4000, 50, 1000);
    if (millis() - lastLedToggle2 >= blinkInterval2) {
      digitalWrite(LED_PIN_2, !digitalRead(LED_PIN_2));
      lastLedToggle2 = millis();
    }
  }

  // --- TOF Sensor 1 ---
  if (sensor1.dataReady()) {
    int16_t distance1 = sensor1.distance();
    sensor1.clearInterrupt();

    if (distance1 == -1) {
      error_count_1++;
      Serial.print("[S1 ERROR "); Serial.print(error_count_1); Serial.println("] Bad reading");
    } else {
      measurement_count_1++;
      sum_distance_1 += distance1;
      if (distance1 < min_distance_1) min_distance_1 = distance1;
      if (distance1 > max_distance_1) max_distance_1 = distance1;
      int16_t avg_distance_1 = sum_distance_1 / measurement_count_1;
      
      latestDistance1 = distance1;

      Serial.print("[S1:"); Serial.print(measurement_count_1); Serial.print("] ");
      Serial.print(distance1); Serial.print(" mm | ");
      Serial.print(distance1 / 10.0, 1); Serial.print(" cm");
      Serial.print(" | Min:"); Serial.print(min_distance_1);
      Serial.print(" Avg:"); Serial.print(avg_distance_1);
      Serial.print(" Max:"); Serial.println(max_distance_1);
    }
  }

  // --- TOF Sensor 2 ---
  if (sensor2.dataReady()) {
    int16_t distance2 = sensor2.distance();
    sensor2.clearInterrupt();

    if (distance2 == -1) {
      error_count_2++;
      Serial.print("[S2 ERROR "); Serial.print(error_count_2); Serial.println("] Bad reading");
    } else {
      measurement_count_2++;
      sum_distance_2 += distance2;
      if (distance2 < min_distance_2) min_distance_2 = distance2;
      if (distance2 > max_distance_2) max_distance_2 = distance2;
      int16_t avg_distance_2 = sum_distance_2 / measurement_count_2;
      
      latestDistance2 = distance2;

      Serial.print("[S2:"); Serial.print(measurement_count_2); Serial.print("] ");
      Serial.print(distance2); Serial.print(" mm | ");
      Serial.print(distance2 / 10.0, 1); Serial.print(" cm");
      Serial.print(" | Min:"); Serial.print(min_distance_2);
      Serial.print(" Avg:"); Serial.print(avg_distance_2);
      Serial.print(" Max:"); Serial.println(max_distance_2);
    }
  }

  // --- mmWave OUT pin (instant hardware signal) ---
  static bool lastMotionState = false;
  bool motionDetected = mmWaveReady && digitalRead(MMWAVE_OUT_PIN);
  if (motionDetected != lastMotionState) {
    lastMotionState = motionDetected;
    Serial.print("[mmWave] ");
    Serial.println(motionDetected ? "Motion DETECTED" : "No motion");
  }

  // --- mmWave UART data (every 200 ms) ---
  if (mmWaveReady && millis() - lastMmWaveUpdate >= 200) {
    lastMmWaveUpdate = millis();

    uint8_t targetNum = mmWave.getTargetNumber();
    if (targetNum > 0) {
      float range = mmWave.getTargetRange();
      float speed = mmWave.getTargetSpeed();
      uint32_t energy = mmWave.getTargetEnergy();
      
      latestTargetNum = targetNum;
      latestRange = range;
      latestSpeed = speed;
      latestEnergy = energy;

      Serial.print("[mmWave] Targets: "); Serial.print(targetNum);
      Serial.print(" | Range: "); Serial.print(range, 2); Serial.print(" m");
      Serial.print(" | Speed: "); Serial.print(speed, 2); Serial.print(" m/s");
      Serial.print(speed > 0.05f ? " (approaching)" : speed < -0.05f ? " (receding)" : " (still)");
      Serial.print(" | Energy: "); Serial.println(energy);
    } else {
      // Still report OUT pin state when no UART target
      static uint32_t lastNoTargetPrint = 0;
      if (millis() - lastNoTargetPrint >= 1000) {
        lastNoTargetPrint = millis();
        Serial.println("[mmWave] No target detected");
      }
    }
  }

  // --- GPS ---
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (millis() - lastGPSUpdate >= 5000) {
    lastGPSUpdate = millis();

    if (gps.location.isValid() || gps.date.isValid() || gps.time.isValid()) {
      Serial.println("========== GPS Data ==========");
      if (gps.location.isValid()) {
        Serial.print("Position: ");
        Serial.print(gps.location.lat(), 6); Serial.print(", ");
        Serial.println(gps.location.lng(), 6);
      }
      if (gps.altitude.isValid()) {
        Serial.print("Altitude: "); Serial.print(gps.altitude.meters(), 1); Serial.println(" m");
      }
      if (gps.speed.isValid()) {
        Serial.print("Speed: "); Serial.print(gps.speed.kmph(), 2); Serial.println(" km/h");
      }
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print("DateTime (UTC): ");
        Serial.print(gps.date.year()); Serial.print("/");
        if (gps.date.month() < 10) Serial.print("0");
        Serial.print(gps.date.month()); Serial.print("/");
        if (gps.date.day() < 10) Serial.print("0");
        Serial.print(gps.date.day()); Serial.print(" ");
        if (gps.time.hour() < 10) Serial.print("0");
        Serial.print(gps.time.hour()); Serial.print(":");
        if (gps.time.minute() < 10) Serial.print("0");
        Serial.print(gps.time.minute()); Serial.print(":");
        if (gps.time.second() < 10) Serial.print("0");
        Serial.println(gps.time.second());
      }
      Serial.print("Satellites: "); Serial.print(gps.satellites.value());
      Serial.print(" | HDOP: "); Serial.println(gps.hdop.hdop(), 2);
      Serial.println("==============================");
    } else {
      Serial.print("[GPS] Waiting for fix... (Chars: ");
      Serial.print(gps.charsProcessed());
      Serial.print(", Sats: ");
      Serial.print(gps.satellites.value());
      Serial.println(")");
    }
  }

  // --- Battery Monitor ---
  if (fuelGaugePresent && millis() - lastBatteryUpdate >= BATTERY_UPDATE_INTERVAL) {
    lastBatteryUpdate = millis();
    
    batteryVoltage = readBatteryVoltage();
    batteryPercent = readBatteryPercent();
    
    Serial.print("[Battery] ");
    Serial.print(batteryPercent, 1);
    Serial.print("% | ");
    Serial.print(batteryVoltage, 3);
    Serial.println("V");
  }

#if ENABLE_HTTP_POST
  if (WiFi.status() == WL_CONNECTED && millis() - lastHTTPSend >= HTTP_SEND_INTERVAL) {
    lastHTTPSend = millis();
    sendSensorData();
  }
#endif
}
