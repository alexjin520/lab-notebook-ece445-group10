#include <Wire.h>
#include "driver/temperature_sensor.h"   // ESP32-S3 internal temp sensor
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>               // POSIX time — used for NTP Unix timestamp

// ── WiFi & Server configuration ──────────────────────────────────────────────
#define WIFI_SSID       "Verizon-SM-S901U-FA93"
#define WIFI_PASSWORD   "cacz115/"
#define SERVER_URL      "http://10.206.194.152:5000/nav"
#define HTTP_INTERVAL   100    // ms between HTTP POSTs  (10 Hz)

// ── NTP configuration ────────────────────────────────────────────────────────
#define NTP_SERVER      "pool.ntp.org"
#define NTP_GMT_OFFSET  0      // UTC; change to local offset in seconds if needed
#define NTP_DST_OFFSET  0

// ── Pin definitions (matches schematic I2C0 bus) ─────────────────────────────
#define I2C_SDA       7
#define I2C_SCL       8
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

// ── HTTP POST helper — conforms to server /nav JSON schema ──────────────────
//
// Required top-level fields:
//   device_id, module, tof_sensors, mmwave_sensors, imu, timestamp
//
// Fields this board cannot provide are sent as empty objects / nulls so the
// server schema is satisfied without errors.
// Battery / thermal data are appended as extra top-level keys; algorithm.py
// will ignore them while the server still logs them for verification.
static void sendPowerData(float voltage, float soc, float tempC, uint32_t freeHeap) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(SERVER_URL);
  http.setReuse(true);            // keep-alive — avoids TCP handshake every call
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(80);     // TCP connect must succeed within 80 ms
  http.setTimeout(80);            // total response wait within 80 ms

  // Size: ~512 B is ample for empty sensor objects + battery fields
  StaticJsonDocument<512> doc;

  // ── Required identification fields ────────────────────────────────────────
  // timestamp = Unix time in milliseconds (NTP-synced) so the server can
  // compute true end-to-end latency via  latency_ms = server_recv_ms - timestamp
  doc["device_id"] = "esp32_power";
  doc["module"]    = "body";          // power board is part of the body module
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  uint64_t unix_ms = (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL;
  doc["timestamp"]    = unix_ms;           // ms since Unix epoch (NTP)
  doc["uptime_ms"]    = (uint32_t)millis(); // board uptime for cross-check

  // ── Sensor fields: empty — power board has no ToF / mmWave / IMU ──────────
  doc.createNestedObject("tof_sensors");    // {}
  doc.createNestedObject("mmwave_sensors"); // {}
  doc.createNestedObject("imu");            // {}

  // ── Power-system data (extra fields, logged by server, ignored by algo) ───
  JsonObject battery = doc.createNestedObject("battery");
  battery["voltage_v"]  = serialized(String(voltage, 3));
  battery["soc_pct"]    = serialized(String(soc,     1));

  JsonObject system = doc.createNestedObject("system");
  system["die_temp_c"]  = serialized(String(tempC,   1));
  system["free_heap_b"] = freeHeap;
  system["wifi_rssi"]   = WiFi.RSSI();

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  if (code > 0) {
    Serial.print("[HTTP] POST /nav => "); Serial.println(code);
  } else {
    Serial.print("[HTTP] Error: "); Serial.println(http.errorToString(code));
  }
  http.end();
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

  // ── WiFi connection ───────────────────────────────────────────────────────
  Serial.print("Connecting to WiFi: "); Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  int wifiTries = 0;
  while (WiFi.status() != WL_CONNECTED && wifiTries < 20) {
    delay(500); Serial.print("."); wifiTries++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.print("  [PASS] WiFi connected — IP: "); Serial.println(WiFi.localIP());
    Serial.print("  Posting to: "); Serial.println(SERVER_URL);

    // Sync system clock via NTP so timestamps are Unix ms (needed for latency)
    configTime(NTP_GMT_OFFSET, NTP_DST_OFFSET, NTP_SERVER);
    Serial.print("  Syncing NTP time");
    struct tm ti;
    int ntpTries = 0;
    while (!getLocalTime(&ti) && ntpTries++ < 20) {
      delay(500); Serial.print(".");
    }
    if (ntpTries < 20) {
      char tbuf[32];
      strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S UTC", &ti);
      Serial.println();
      Serial.print("  [PASS] NTP synced: "); Serial.println(tbuf);
    } else {
      Serial.println();
      Serial.println("  [WARN] NTP sync failed — timestamps will be 0-based (no latency calc)");
    }
  } else {
    Serial.println();
    Serial.println("  [WARN] WiFi connection failed — HTTP POST disabled");
  }
  Serial.println();
}

// ═════════════════════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();

  // ── HTTP POST at 10 Hz (runs every loop iteration, independent of serial) ─
  static unsigned long lastPost = 0;
  if (now - lastPost >= HTTP_INTERVAL) {
    lastPost = now;
    float voltage = fuelVoltage();
    float soc     = fuelSOC();
    float tempC   = readTempC();
    sendPowerData(voltage, soc, tempC, ESP.getFreeHeap());
  }

  // ── Serial live print every 3 s (purely for monitoring, does not gate POST) ─
  static unsigned long lastPrint = 0;
  if (now - lastPrint < 3000) return;
  lastPrint = now;

  float voltage = fuelVoltage();
  float soc     = fuelSOC();
  float tempC   = readTempC();

  Serial.print("[Live] Uptime: ");
  Serial.print(now / 1000);
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
