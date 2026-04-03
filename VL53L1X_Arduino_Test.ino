#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <TinyGPS++.h>
#include <DFRobot_C4001.h>
#include <Adafruit_DRV2605.h>

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

Adafruit_VL53L1X sensor1 = Adafruit_VL53L1X();
Adafruit_VL53L1X sensor2 = Adafruit_VL53L1X();

Adafruit_DRV2605 drv1;

HardwareSerial mmWaveSerial(1);
// Baud rate is 9600 (factory default). Pins passed to constructor so the
// library's begin() initialises the serial port correctly on ESP32.
DFRobot_C4001_UART mmWave(&mmWaveSerial, 9600, MMWAVE_RX_PIN, MMWAVE_TX_PIN);

HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

bool mmWaveReady = false;
unsigned long lastMotorUpdate1 = 0;
bool motorState1 = false;
unsigned long lastLedToggle2 = 0;
unsigned long lastGPSUpdate = 0;
unsigned long lastMmWaveUpdate = 0;

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

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  // Bring up sensor 1 on default address, reassign to 0x30
  digitalWrite(XSHUT_PIN_1, HIGH);
  delay(10);
  if (!sensor1.begin(0x29, &Wire)) {
    Serial.println("✗ Failed to initialize sensor 1!");
    while (1) delay(1000);
  }
  sensor1.VL53L1X_SetI2CAddress(0x30 << 1);
  delay(50);
  if (!sensor1.begin(0x30, &Wire)) {
    Serial.println("✗ Failed to re-initialize sensor 1 at 0x30!");
    while (1) delay(1000);
  }
  Serial.println("✓ VL53L1X Sensor 1 initialized (addr 0x30)");

  // Bring up sensor 2 on default address 0x29
  digitalWrite(XSHUT_PIN_2, HIGH);
  delay(10);
  if (!sensor2.begin(0x29, &Wire)) {
    Serial.println("✗ Failed to initialize sensor 2!");
    while (1) delay(1000);
  }
  Serial.println("✓ VL53L1X Sensor 2 initialized (addr 0x29)");

  if (!drv1.begin(&Wire)) {
    Serial.println("✗ Failed to initialize DRV2605L 1!");
    while (1) delay(1000);
  }
  drv1.selectLibrary(1);
  drv1.setMode(DRV2605_MODE_REALTIME);
  Serial.println("✓ DRV2605L Motor Driver 1 initialized (addr 0x5A)");

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

  Serial.println("Starting measurements...");
  Serial.println("========================================");
}

void loop() {
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

      int blinkInterval1 = map(distance1, 0, 4000, 50, 1000);
      if (millis() - lastMotorUpdate1 >= blinkInterval1) {
        motorState1 = !motorState1;
        digitalWrite(LED_PIN_1, motorState1);
        drv1.setRealtimeValue(motorState1 ? 127 : 0);
        lastMotorUpdate1 = millis();
      }

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

      int blinkInterval2 = map(distance2, 0, 4000, 50, 1000);
      if (millis() - lastLedToggle2 >= blinkInterval2) {
        digitalWrite(LED_PIN_2, !digitalRead(LED_PIN_2));
        lastLedToggle2 = millis();
      }

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
}
