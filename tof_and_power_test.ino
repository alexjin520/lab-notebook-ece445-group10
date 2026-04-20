#include <Wire.h>
#include "driver/temperature_sensor.h"   // ESP32-S3 internal temp sensor
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>               // POSIX time — used for NTP Unix timestamp
#include <Adafruit_VL53L1X.h>    // VL53L1X 4m ToF distance sensor
#include <Adafruit_DRV2605.h>    // DRV2605L haptic motor driver

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

// ── TCA9548A I2C Multiplexer ──────────────────────────────────────────────────
#define MUX_ADDR      0x77   // ToF MUX: A0/A1/A2 all tied to 3.3V → 0x77
#define DRV_MUX_ADDR  0x70   // DRV MUX: A0/A1/A2 all GND → 0x70

// ── Haptic thresholds — RTP intensity driven from ToF distance ———————————
#define HAPTIC_CRITICAL_MM   300   // ≤ 300 mm → max buzz  (RTP 127)
#define HAPTIC_WARNING_MM    600   // ≤ 600 mm → medium   (RTP 70)
#define HAPTIC_CAUTION_MM   1000   // ≤ 1000 mm → light    (RTP 30)
                                   //  > 1000 mm → silent   (RTP 0)

// ── VL53L1X ToF sensor descriptor ────────────────────────────────────────────
struct TofSensor {
  const char* name;       // JSON key — matches server schema direction name
  uint8_t     mux_ch;     // TCA9548A output channel (0-7)
  uint8_t     xshut_pin;  // GPIO driving XSHUT (active-LOW reset)
  bool        present;    // true if sensor responded at init
  int16_t     last_mm;    // most recent valid reading (-1 = none yet)
  uint32_t    count;      // valid readings accumulated since boot
  int16_t     min_mm;     // running minimum
  int16_t     max_mm;     // running maximum
  uint32_t    sum_mm;     // running sum (for avg)
  uint32_t    err_count;  // error / -1 readings
};

// One Adafruit driver per sensor slot — all stay at 0x29; MUX isolates them.
static Adafruit_VL53L1X tofDev[8];

//                  name          mux_ch  xshut  present  last   cnt    min    max  sum  err
// XSHUT pin assignments — avoid strapping pins GPIO45 (VDD_SPI sel) and GPIO46 (ROM log)
static TofSensor tofs[8] = {
  { "front",           0,    1,   false,   -1,    0,  32767,    0,   0,   0 },  // F   GPIO1
  { "front_right",     5,    2,   false,   -1,    0,  32767,    0,   0,   0 },  // FR  GPIO2
  { "right",           1,   38,   false,   -1,    0,  32767,    0,   0,   0 },  // R   GPIO38
  { "back_right",      7,   48,   false,   -1,    0,  32767,    0,   0,   0 },  // BR  GPIO48
  { "back",            2,   13,   false,   -1,    0,  32767,    0,   0,   0 },  // B   GPIO13
  { "back_left",       6,   47,   false,   -1,    0,  32767,    0,   0,   0 },  // BL  GPIO47
  { "left",            3,   14,   false,   -1,    0,  32767,    0,   0,   0 },  // L   GPIO14
  { "front_left",      4,   21,   false,   -1,    0,  32767,    0,   0,   0 },  // FL  GPIO21
};

// ── MUX helper ───────────────────────────────────────────────────────────────
// ch 0-7 enables exactly one downstream channel; 0xFF disables all.
static void muxSelect(uint8_t ch) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(ch > 7 ? 0x00 : (uint8_t)(1 << ch));
  Wire.endTransmission();
}

static void muxSelectDrv(uint8_t ch) {
  Wire.beginTransmission(DRV_MUX_ADDR);
  Wire.write(ch > 7 ? 0x00 : (uint8_t)(1 << ch));
  Wire.endTransmission();
}

// ── DRV2605L haptic motor descriptor ───────────────────────────────
struct DrvMotor {
  const char* dir;      // Compass key for JSON: n/ne/e/se/s/sw/w/nw
  uint8_t     mux_ch;   // DRV MUX (0x70) channel
  bool        present;
  uint8_t     rtp;      // current RTP value (0=silent, 127=max)
};

static Adafruit_DRV2605 drvDev[8];

//                dir   mux_ch  present  rtp
static DrvMotor drvs[8] = {
  { "n",   0,  false,  0 },   // front
  { "ne",  1,  false,  0 },   // front_right
  { "e",   2,  false,  0 },   // right
  { "se",  3,  false,  0 },   // back_right
  { "s",   4,  false,  0 },   // back
  { "sw",  5,  false,  0 },   // back_left
  { "w",   6,  false,  0 },   // left
  { "nw",  7,  false,  0 },   // front_left
};

// ── DRV2605L initialisation — called once from setup() ──────────────────
static void initDrvMotors() {
  Wire.beginTransmission(DRV_MUX_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("  [FAIL] DRV MUX not found at 0x70 — all haptic motors skipped");
    return;
  }
  Serial.println("  [PASS] DRV TCA9548A MUX found at 0x70");

  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("    "); Serial.print(drvs[i].dir);
    Serial.print("  (DRV MUX ch "); Serial.print(drvs[i].mux_ch); Serial.print("): ");

    muxSelectDrv(drvs[i].mux_ch);
    delay(5);

    Wire.beginTransmission(0x5A);
    if (Wire.endTransmission() != 0) {
      Serial.println("NOT FOUND");
      drvs[i].present = false;
      continue;
    }
    if (!drvDev[i].begin(&Wire)) {
      Serial.println("INIT FAILED");
      drvs[i].present = false;
      continue;
    }
    drvDev[i].selectLibrary(1);               // ERM library
    drvDev[i].setMode(DRV2605_MODE_REALTIME); // RTP mode via I2C (IN pin floating)
    drvDev[i].setRealtimeValue(0);            // silent at boot
    drvs[i].present = true;
    Serial.println("OK");
  }
  muxSelectDrv(0xFF);

  uint8_t drvFound = 0;
  for (uint8_t i = 0; i < 8; i++) if (drvs[i].present) drvFound++;
  Serial.print("  DRV init summary: ");
  Serial.print(drvFound); Serial.print("/8 motors found  [");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(drvs[i].dir); Serial.print(drvs[i].present ? "=OK" : "=--");
    if (i < 7) Serial.print("  ");
  }
  Serial.println("]");
}

// ── Haptic update — RTP driven from matched ToF reading ─────────────────
// drvs[i] and tofs[i] share the same direction index.
static void updateHaptics() {
  for (uint8_t i = 0; i < 8; i++) {
    if (!drvs[i].present) continue;
    uint8_t target = 0;
    if (tofs[i].present && tofs[i].count > 0 && tofs[i].last_mm >= 0) {
      int16_t d = tofs[i].last_mm;
      if      (d <= HAPTIC_CRITICAL_MM) target = 127;
      else if (d <= HAPTIC_WARNING_MM)  target = 70;
      else if (d <= HAPTIC_CAUTION_MM)  target = 30;
    }
    if (target == drvs[i].rtp) continue;  // no change — skip write
    drvs[i].rtp = target;
    muxSelectDrv(drvs[i].mux_ch);
    delayMicroseconds(200);
    drvDev[i].setRealtimeValue(target);
  }
  muxSelectDrv(0xFF);
}

// ── ToF initialisation — called once from setup() ────────────────────────────
static void initToFSensors() {
  Wire.beginTransmission(MUX_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("  [FAIL] TCA9548A not found at 0x77 — all ToF sensors skipped");
    return;
  }
  Serial.println("  [PASS] TCA9548A MUX found at 0x77");

  // Hold all XSHUT lines LOW to put every sensor in reset before we start
  for (uint8_t i = 0; i < 8; i++) {
    pinMode(tofs[i].xshut_pin, OUTPUT);
    digitalWrite(tofs[i].xshut_pin, LOW);
  }
  delay(10);

  // Bring each sensor up one at a time on its dedicated MUX channel
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print("    "); Serial.print(tofs[i].name);
    Serial.print("  (MUX ch "); Serial.print(tofs[i].mux_ch);
    Serial.print(", XSHUT GPIO "); Serial.print(tofs[i].xshut_pin); Serial.print("): ");

    digitalWrite(tofs[i].xshut_pin, HIGH);  // release reset
    delay(15);
    muxSelect(tofs[i].mux_ch);              // expose sensor on I2C
    delay(5);

    // Quick ACK probe — if nothing responds at 0x29 skip begin() entirely
    Wire.beginTransmission(0x29);
    bool ackOk = (Wire.endTransmission() == 0);

    if (!ackOk || !tofDev[i].begin(0x29, &Wire)) {
      Serial.println("NOT FOUND");
      Serial.print("      → Check: XSHUT GPIO"); Serial.print(tofs[i].xshut_pin);
      Serial.print(" pulled HIGH? MUX ch"); Serial.print(tofs[i].mux_ch);
      Serial.println(" routed? Sensor addr=0x29 on that channel?");
      tofs[i].present = false;
    } else {
      tofDev[i].startRanging();
      tofDev[i].setTimingBudget(50);
      tofs[i].present = true;
      Serial.println("OK");
    }
  }
  muxSelect(0xFF);  // disable all channels after init

  // Init summary
  uint8_t tofFound = 0;
  Serial.print("  ToF init summary: ");
  for (uint8_t i = 0; i < 8; i++) if (tofs[i].present) tofFound++;
  Serial.print(tofFound); Serial.print("/8 sensors found  [");
  for (uint8_t i = 0; i < 8; i++) {
    Serial.print(tofs[i].name); Serial.print(tofs[i].present ? "=OK" : "=--");
    if (i < 7) Serial.print("  ");
  }
  Serial.println("]");
}

// ── ToF polling — non-blocking, call every loop() iteration ──────────────────
static void readToFSensors() {
  for (uint8_t i = 0; i < 8; i++) {
    if (!tofs[i].present) continue;
    muxSelect(tofs[i].mux_ch);
    delayMicroseconds(200);              // brief settling after channel switch
    if (!tofDev[i].dataReady()) continue;
    int16_t d = tofDev[i].distance();
    tofDev[i].clearInterrupt();
    if (d < 0) {
      tofs[i].err_count++;
    } else {
      tofs[i].last_mm = d;
      tofs[i].count++;
      tofs[i].sum_mm += d;
      if (d < tofs[i].min_mm) tofs[i].min_mm = d;
      if (d > tofs[i].max_mm) tofs[i].max_mm = d;
    }
  }
  muxSelect(0xFF);  // disable all channels when done
}

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
// ToF readings are populated from the 8-sensor array (via TCA9548A MUX).
// mmwave and imu are not on this board — sent as empty objects {}.
// Battery / thermal data are extra top-level keys logged by the server.
static void sendPowerData(float voltage, float soc, float tempC, uint32_t freeHeap) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(SERVER_URL);
  http.setReuse(true);            // keep-alive — avoids TCP handshake every call
  http.addHeader("Content-Type", "application/json");
  http.setConnectTimeout(80);     // TCP connect must succeed within 80 ms
  http.setTimeout(80);            // total response wait within 80 ms

  // 1536 B: 8 ToF × ~5 fields + 8 DRV × ~2 fields + battery/system overhead
  StaticJsonDocument<1536> doc;

  // ── Required identification fields ────────────────────────────────────────
  // timestamp = Unix ms (NTP-synced) — server computes latency_ms = recv - timestamp
  doc["device_id"] = "esp32_power";
  doc["module"]    = "body";
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  uint64_t unix_ms = (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL;
  doc["timestamp"] = unix_ms;
  doc["uptime_ms"] = (uint32_t)millis();

  // ── ToF sensor readings (empty object {} = absent or no data yet) ─────────
  JsonObject tofObj = doc.createNestedObject("tof_sensors");
  for (uint8_t i = 0; i < 8; i++) {
    if (!tofs[i].present || tofs[i].count == 0) {
      tofObj.createNestedObject(tofs[i].name);   // {} — absent / no data yet
    } else {
      JsonObject s = tofObj.createNestedObject(tofs[i].name);
      s["distance_mm"] = tofs[i].last_mm;
      s["count"]       = tofs[i].count;
      s["min"]         = tofs[i].min_mm;
      s["avg"]         = (int16_t)(tofs[i].sum_mm / tofs[i].count);
      s["max"]         = tofs[i].max_mm;
    }
  }
  doc.createNestedObject("mmwave_sensors"); // {} — not on this board
  doc.createNestedObject("imu");            // {} — not on this board

  // ── Haptic motor states — current RTP value + zone for each direction ———
  JsonObject hapticObj = doc.createNestedObject("haptic_motors");
  for (uint8_t i = 0; i < 8; i++) {
    if (!drvs[i].present) {
      hapticObj.createNestedObject(drvs[i].dir);  // {} — not populated
    } else {
      JsonObject h = hapticObj.createNestedObject(drvs[i].dir);
      h["rtp"]  = drvs[i].rtp;
      const char* zone = "CLEAR";
      if      (drvs[i].rtp >= 127) zone = "CRITICAL";
      else if (drvs[i].rtp >= 70)  zone = "WARNING";
      else if (drvs[i].rtp >= 30)  zone = "CAUTION";
      h["zone"] = zone;
    }
  }

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
  delay(4000);   // wait for USB-CDC to re-enumerate after reset (Windows needs ~3s)

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
  // N8 has 8 MB (8192 KB) flash; allow ±1 MB for reporting variance.
  // 16384 KB means IDE Flash Size is set to 16 MB — change to 8 MB in Tools menu.
  bool flashOk = (flashSzKB >= 7168 && flashSzKB <= 9216);
  // N8 has NO PSRAM — presence of PSRAM would indicate wrong board config
  bool psramOk = (psramSz == 0);

  if (cpuOk)   printPass("CPU frequency in expected range");
  else         printFail("CPU frequency unexpected");

  if (ramOk)   printPass("Free heap > 100 KB");
  else         printFail("Free heap too low");

  if (flashOk) printPass("Flash ~8 MB detected (N8 variant confirmed)");
  else {
    if (flashSzKB < 7168)  printFail("Flash < 7 MB — set Flash Size to 8MB in Arduino IDE");
    else if (flashSzKB == 16384) printFail("Flash reports 16 MB — change Tools > Flash Size to 8MB in Arduino IDE");
    else                   printFail("Flash size unexpected — check board selection");
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
  Serial.println("[TEST 4] I2C Bus (SDA=GPIO7, SCL=GPIO8)");
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

  // ── TEST 8: TCA9548A + VL53L1X ToF Sensors ───────────────────────────────
  Serial.println("[TEST 8] TCA9548A MUX (0x77) + 8× VL53L1X ToF");
  Serial.println("  F/FR/R/BR → MUX ch 0/5/1/7  XSHUT GPIO 1/2/38/48");
  Serial.println("  B/BL/L/FL → MUX ch 2/6/3/4  XSHUT GPIO 13/47/14/21");
  initToFSensors();
  Serial.println();

  // ── TEST 9: DRV2605L Haptic Motors via TCA9548A MUX ———————————————
  Serial.println("[TEST 9] DRV2605L Haptic Motors (MUX 0x70)");
  Serial.println("  n/ne/e/se → MUX ch 0/1/2/3   (front/front_right/right/back_right)");
  Serial.println("  s/sw/w/nw → MUX ch 4/5/6/7   (back/back_left/left/front_left)");
  Serial.println("  Thresholds: CRITICAL≤300mm(RTP127) WARNING≤600mm(70) CAUTION≤1000mm(30)");
  initDrvMotors();
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

  // ── Poll all 8 ToF sensors (non-blocking — skips if dataReady() is false) ──
  readToFSensors();

  // ── Update haptic motors from latest ToF readings ──────────────────────
  updateHaptics();

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

  // ToF sensor status — one line per present sensor, skip absent ones
  bool anyTof = false;
  for (uint8_t i = 0; i < 8; i++) if (tofs[i].present) { anyTof = true; break; }
  if (anyTof) {
    Serial.print("[ToF]  ");
    for (uint8_t i = 0; i < 8; i++) {
      if (!tofs[i].present) continue;
      Serial.print(tofs[i].name); Serial.print(":");
      if (tofs[i].count == 0) {
        Serial.print("---mm");
      } else {
        Serial.print(tofs[i].last_mm); Serial.print("mm");
      }
      Serial.print("  ");
    }
    Serial.println();
  } else {
    Serial.println("[ToF]  No sensors present");
  }
}
