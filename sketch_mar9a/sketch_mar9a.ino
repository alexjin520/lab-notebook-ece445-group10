#include <Wire.h>
#include "driver/temperature_sensor.h"   // ESP32-S3 internal temp sensor
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <time.h>               // POSIX time — used for NTP Unix timestamp
#include <Adafruit_VL53L1X.h>    // VL53L1X 4m ToF distance sensor
#include <Adafruit_DRV2605.h>    // DRV2605L haptic motor driver
#include "ICM_20948.h"           // SparkFun ICM-20948 9-DoF IMU (Accel/Gyro/Mag)

// ── WiFi & Server configuration ──────────────────────────────────────────────
#define WIFI_SSID       "Verizon-SM-S901U-FA93"
#define WIFI_PASSWORD   "cacz115/"
#define SERVER_URL      "http://10.206.194.152:5000/nav"
#define HTTP_INTERVAL   1000   // ms between HTTP POSTs  (1 Hz — testing)

// ── NTP configuration ────────────────────────────────────────────────────────
#define NTP_SERVER      "pool.ntp.org"
#define NTP_GMT_OFFSET  0      // UTC
#define NTP_DST_OFFSET  0

// ── Pin definitions (matches schematic I2C0 bus) ─────────────────────────────
#define I2C_SDA       7
#define I2C_SCL       8
#define FUEL_GAUGE_ADDR 0x36

// ── TCA9548A I2C Multiplexer ──────────────────────────────────────────────────
#define MUX_ADDR      0x77   // ToF MUX: A0/A1/A2 all tied to 3.3V → 0x77
#define DRV_MUX_ADDR  0x70   // DRV MUX: A0/A1/A2 all GND → 0x70

// ── ICM-20948 9-DoF IMU (SparkFun breakout, J_IMU_Back_Up1) ──────────────────
// Wired on the I2C0 bus (SDA=GPIO7, SCL=GPIO8). AD0 pin tied to GND → addr 0x68.
// Pins 8-11 (ACL/ADA/FSNC/INT) are unused for basic 9-DoF I2C operation.
#define IMU_ADDR      0x68   // ICM-20948 (AD0=0); WHO_AM_I should report 0xEA
static ICM_20948_I2C imuDev;
static bool          imuPresent = false;
static uint32_t      imuReadCount = 0;
static uint32_t      imuErrCount  = 0;

// ── Magnetometer hard-iron offsets (uT) ──────────────────────────────────────
// Replace these with the values printed at the end of runMagCalibration().
// Until calibrated, the heading will be biased — direction trends still work.
static float magOffX = 0.0f;
static float magOffY = 0.0f;
static float magOffZ = 0.0f;

// Set to 1 to run a 20-second magnetometer calibration sweep at boot.
#define IMU_MAG_CALIBRATE_AT_BOOT 0

// ── Computed orientation (filled by computeIMUOrientation) ────────────────────
struct IMUOrient {
  float roll_deg;     // rotation about body X (forward) — positive = right-side down
  float pitch_deg;    // rotation about body Y (right)   — positive = nose up
  float heading_deg;  // tilt-compensated yaw (0 = magnetic North, CW positive)
  float accel_g;      // |accel| in g  (≈1.0 when stationary)
  float gyro_dps;     // |gyro|  in deg/s
  bool  isStill;      // accel ≈ 1g AND gyro nearly zero
  bool  isTurning;    // |gyro_z| above threshold (yaw rotation)
  bool  isFalling;    // |accel| ≪ 1g (free-fall window)
};
static IMUOrient imuOrient = { 0,0,0, 1.0f, 0, true, false, false };

// ── C4001 24 GHz mmWave Radar ────────────────────────────────────────────────
// Two Seeed C4001 sensors; each has a UART interface + a digital OUT pin.
// ESP32 Serial1 → Radar 1,  Serial2 → Radar 2.
// OUT pin: HIGH = human presence detected, LOW = no presence.
#define RADAR1_TX   39    // ESP32 TX1 → C4001 #1 RX  (module pad 32)
#define RADAR1_RX   40    // ESP32 RX1 ← C4001 #1 TX  (module pad 33)
#define RADAR1_OUT  15    // C4001 #1 OUT digital pin   (module pad 8)
#define RADAR2_TX   41    // ESP32 TX2 → C4001 #2 RX  (module pad 34)
#define RADAR2_RX   42    // ESP32 RX2 ← C4001 #2 TX  (module pad 35)
#define RADAR2_OUT  16    // C4001 #2 OUT digital pin   (module pad 9)
#define RADAR_BAUD  115200

// C4001 frame parser state
enum RadarParseState : uint8_t {
  RPS_IDLE,       // waiting for 0x53
  RPS_HDR2,       // waiting for 0x59
  RPS_FUNC,       // function byte
  RPS_CMD,        // command byte
  RPS_LEN_H,      // length high byte
  RPS_LEN_L,      // length low byte
  RPS_DATA,       // collecting data bytes
  RPS_CHECKSUM,   // checksum byte
  RPS_TAIL1,      // 0x54
  RPS_TAIL2,      // 0x43
};

struct RadarSensor {
  const char*    name;
  HardwareSerial* serial;
  uint8_t         txPin;
  uint8_t         rxPin;
  uint8_t         outPin;
  bool            uartOk;       // UART initialised
  bool            presence;     // current OUT-pin reading
  uint32_t        presenceMs;   // millis() when last presence edge went HIGH
  // Parsed UART values (updated by frame parser)
  uint8_t         detState;     // 0=no one, 1=stationary, 2=moving
  uint16_t        distCm;       // target distance in cm (0 = unknown)
  uint8_t         energy;       // motion energy 0-100
  uint32_t        frameCount;   // valid frames decoded
  uint32_t        lastFrameMs;  // millis() of last valid frame
  // Frame parser state machine
  RadarParseState pstate;
  uint8_t         pFunc;
  uint8_t         pCmd;
  uint16_t        pLen;
  uint16_t        pDataIdx;
  uint8_t         pData[32];
  uint8_t         pChecksum;
  uint8_t         pCalcCk;      // running checksum (additive sum mod 256 of func..data)
  // Raw byte capture — first 64 bytes for protocol diagnostics
  uint8_t         rawDump[64];
  uint8_t         rawDumpLen;   // 0-64; stops at 64
};

static RadarSensor radars[2] = {
  { "radar1", &Serial1, RADAR1_TX, RADAR1_RX, RADAR1_OUT,
    false, false, 0,   0, 0, 0, 0, 0,   RPS_IDLE, 0,0,0,0,{},0,0, {}, 0 },
  { "radar2", &Serial2, RADAR2_TX, RADAR2_RX, RADAR2_OUT,
    false, false, 0,   0, 0, 0, 0, 0,   RPS_IDLE, 0,0,0,0,{},0,0, {}, 0 },
};

// Initialise both C4001 sensors
static void initRadars() {
  for (uint8_t i = 0; i < 2; i++) {
    RadarSensor& r = radars[i];
    pinMode(r.outPin, INPUT_PULLDOWN);  // pull-down prevents floating HIGH false positives
    r.serial->begin(RADAR_BAUD, SERIAL_8N1, r.rxPin, r.txPin);
    delay(100);
    // A freshly powered C4001 sends an init frame within ~100 ms.
    // Accept any byte on RX as proof the UART link is alive.
    r.uartOk = (r.serial->available() > 0);
    r.presence = (digitalRead(r.outPin) == HIGH);
    Serial.print("    "); Serial.print(r.name);
    Serial.print("  TX="); Serial.print(r.txPin);
    Serial.print(" RX="); Serial.print(r.rxPin);
    Serial.print(" OUT="); Serial.print(r.outPin); Serial.print(": ");
    if (r.uartOk) {
      Serial.println("UART OK");
    } else {
      // UART byte not yet seen — sensor may still be booting; mark present
      // optimistically so polling continues, but note it.
      Serial.println("UART no init byte yet (sensor may still be booting)");
      r.uartOk = true;   // allow continued polling
    }
  }
}

// Dispatch a fully-decoded C4001 frame into the RadarSensor fields.
// MR24HPC1/C4001 detection report frames use func=0x80:
//   cmd=0x01 → presence          data[0]: 0=no one, 1=presence
//   cmd=0x02 → motion state      data[0]: 0=none, 1=stationary, 2=moving
//   cmd=0x03 → body movement     data[0]: 0-100 energy
//   cmd=0x06 → target distance   data[0..1]: uint16 big-endian, unit = cm
static void radarHandleFrame(RadarSensor& r) {
  r.frameCount++;
  r.lastFrameMs = millis();
  if (r.pFunc != 0x80) return;  // only handle detection frames
  switch (r.pCmd) {
    case 0x01:  // presence
      if (r.pLen >= 1) {
        r.presence = (r.pData[0] != 0x00);
        if (r.presence) r.presenceMs = millis();
      }
      break;
    case 0x02:  // motion state
      if (r.pLen >= 1) r.detState = r.pData[0];
      break;
    case 0x03:  // body movement energy
      if (r.pLen >= 1) r.energy = r.pData[0];
      break;
    case 0x06:  // distance
      if (r.pLen >= 2)
        r.distCm = ((uint16_t)r.pData[0] << 8) | r.pData[1];
      break;
  }
}

// Feed one byte into the C4001 frame state machine.
// Frame: 53 59 [func] [cmd] [len_h] [len_l] [data...] [checksum] 54 43
// Checksum = additive sum of func + cmd + len_h + len_l + data[0..n-1], mod 256
static void radarFeedByte(RadarSensor& r, uint8_t b) {
  // Capture first 64 bytes for diagnostics
  if (r.rawDumpLen < sizeof(r.rawDump)) r.rawDump[r.rawDumpLen++] = b;

  switch (r.pstate) {
    case RPS_IDLE:    if (b == 0x53) r.pstate = RPS_HDR2;   break;
    case RPS_HDR2:    r.pstate = (b == 0x59) ? RPS_FUNC : RPS_IDLE; break;
    case RPS_FUNC:    r.pFunc = b; r.pCalcCk  = b; r.pstate = RPS_CMD;    break;
    case RPS_CMD:     r.pCmd  = b; r.pCalcCk += b; r.pstate = RPS_LEN_H;  break;
    case RPS_LEN_H:   r.pLen  = (uint16_t)b << 8; r.pCalcCk += b; r.pstate = RPS_LEN_L; break;
    case RPS_LEN_L:
      r.pLen |= b; r.pCalcCk += b; r.pDataIdx = 0;
      r.pstate = (r.pLen == 0) ? RPS_CHECKSUM : RPS_DATA;
      break;
    case RPS_DATA:
      r.pCalcCk += b;
      if (r.pDataIdx < sizeof(r.pData)) r.pData[r.pDataIdx] = b;
      r.pDataIdx++;
      if (r.pDataIdx >= r.pLen) r.pstate = RPS_CHECKSUM;
      break;
    case RPS_CHECKSUM:
      r.pChecksum = b;
      r.pstate = RPS_TAIL1;
      break;
    case RPS_TAIL1:   r.pstate = (b == 0x54) ? RPS_TAIL2 : RPS_IDLE; break;
    case RPS_TAIL2:
      if (b == 0x43 && r.pChecksum == (r.pCalcCk & 0xFF))
        radarHandleFrame(r);
      r.pstate = RPS_IDLE;
      break;
    default: r.pstate = RPS_IDLE; break;
  }
}

// Poll OUT pins and parse UART frames — call every loop()
static void pollRadars() {
  for (uint8_t i = 0; i < 2; i++) {
    RadarSensor& r = radars[i];
    if (!r.uartOk) continue;

    // OUT pin — hardware presence line (fallback when UART not parsed yet)
    bool outHigh = (digitalRead(r.outPin) == HIGH);
    if (outHigh && !r.presence) r.presenceMs = millis();
    // Only trust OUT pin if no UART frames have arrived yet
    if (r.frameCount == 0) r.presence = outHigh;

    // Parse all pending UART bytes
    while (r.serial->available()) {
      radarFeedByte(r, (uint8_t)r.serial->read());
    }
  }
}

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
  { "front",           0,   19,   false,   -1,    0,  32767,    0,   0,   0 },  // F   GPIO19  (schematic: XSHUT_F)
  { "front_right",     5,   46,   false,   -1,    0,  32767,    0,   0,   0 },  // FR  GPIO46  (schematic: XSHUT_FR — moved; GPIO16 now RADAR_OUT_2)
  { "right",           1,   38,   false,   -1,    0,  32767,    0,   0,   0 },  // R   GPIO38
  { "back_right",      7,   25,   false,   -1,    0,  32767,    0,   0,   0 },  // BR  GPIO25  (schematic: XSHUT_BR)
  { "back",            2,   21,   false,   -1,    0,  32767,    0,   0,   0 },  // B   GPIO21  (schematic: XSHUT_B)
  { "back_left",       6,   24,   false,   -1,    0,  32767,    0,   0,   0 },  // BL  GPIO24  (schematic: XSHUT_BL)
  { "left",            3,   22,   false,   -1,    0,  32767,    0,   0,   0 },  // L   GPIO22  (schematic: XSHUT_L)
  { "front_left",      4,    3,   false,   -1,    0,  32767,    0,   0,   0 },  // FL  GPIO3   (schematic: XSHUT_FL)
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

// ── ICM-20948 IMU initialisation ─────────────────────────────────────────────
// ad0 = 0 because schematic ties AD0 → GND (address 0x68).
// Default ranges set: accel ±2g, gyro ±250 dps, magnetometer continuous 100 Hz.
static bool initIMU() {
  Wire.beginTransmission(IMU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("  [FAIL] ICM-20948 not ACK at 0x68");
    Serial.println("    → Check: VCC→3V3 (pin 2/7), GND (pin 1/12), SDA→GPIO7 (pin 3),");
    Serial.println("             SCL→GPIO8 (pin 4), AD0→GND (pin 5)");
    return false;
  }
  Serial.println("  [PASS] ICM-20948 ACK at 0x68");

  imuDev.begin(Wire, 0);   // ad0val=false → AD0=0 (0x68)

  if (imuDev.status != ICM_20948_Stat_Ok) {
    Serial.print("  [FAIL] ICM-20948 begin() returned: ");
    Serial.println(imuDev.statusString());
    return false;
  }

  imuDev.swReset();
  delay(50);
  imuDev.sleep(false);
  imuDev.lowPower(false);

  if (imuDev.setSampleMode((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr),
                           ICM_20948_Sample_Mode_Continuous) != ICM_20948_Stat_Ok) {
    Serial.print("  [WARN] setSampleMode: "); Serial.println(imuDev.statusString());
  }

  ICM_20948_fss_t fss;
  fss.a = gpm2;     // ±2 g
  fss.g = dps250;   // ±250 dps
  if (imuDev.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), fss)
        != ICM_20948_Stat_Ok) {
    Serial.print("  [WARN] setFullScale: "); Serial.println(imuDev.statusString());
  }

  if (imuDev.startupMagnetometer() != ICM_20948_Stat_Ok) {
    Serial.print("  [WARN] Magnetometer startup: ");
    Serial.println(imuDev.statusString());
    Serial.println("         (Accel + Gyro will still work.)");
  } else {
    Serial.println("  [PASS] AK09916 magnetometer online");
  }

  return true;
}

// Compass direction (cardinal/intercardinal) for a 0–360° heading
static const char* headingCardinal(float deg) {
  if (deg < 0)   deg += 360.0f;
  if (deg >= 360) deg -= 360.0f;
  static const char* dirs[8] = {"N","NE","E","SE","S","SW","W","NW"};
  return dirs[(int)((deg + 22.5f) / 45.0f) & 0x07];
}

// Derive roll/pitch (from gravity), tilt-compensated heading (from compass),
// and motion-state booleans. Assumes imuDev.getAGMT() was called recently.
static void computeIMUOrientation() {
  if (!imuPresent) return;

  float ax = imuDev.accX();
  float ay = imuDev.accY();
  float az = imuDev.accZ();
  float gx = imuDev.gyrX();
  float gy = imuDev.gyrY();
  float gz = imuDev.gyrZ();
  float mx = imuDev.magX() - magOffX;
  float my = imuDev.magY() - magOffY;
  float mz = imuDev.magZ() - magOffZ;

  float roll  = atan2f(ay, az);
  float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));

  float cR = cosf(roll),  sR = sinf(roll);
  float cP = cosf(pitch), sP = sinf(pitch);
  float mxh =  mx * cP + my * sR * sP + mz * cR * sP;
  float myh =             my * cR     - mz * sR;

  float yaw = atan2f(-myh, mxh);
  if (yaw < 0) yaw += 2.0f * (float)PI;

  imuOrient.roll_deg    = roll  * 180.0f / (float)PI;
  imuOrient.pitch_deg   = pitch * 180.0f / (float)PI;
  imuOrient.heading_deg = yaw   * 180.0f / (float)PI;

  imuOrient.accel_g  = sqrtf(ax * ax + ay * ay + az * az) / 1000.0f;
  imuOrient.gyro_dps = sqrtf(gx * gx + gy * gy + gz * gz);

  imuOrient.isStill   = (fabsf(imuOrient.accel_g - 1.0f) < 0.05f) &&
                        (imuOrient.gyro_dps < 5.0f);
  imuOrient.isTurning = (fabsf(gz) > 30.0f);
  imuOrient.isFalling = (imuOrient.accel_g < 0.4f);
}

// One-time magnetometer hard-iron calibration. See test_without_http.ino for
// usage instructions. Behind IMU_MAG_CALIBRATE_AT_BOOT flag.
static void runMagCalibration(uint16_t durationMs = 20000) {
  if (!imuPresent) return;
  Serial.println();
  Serial.println("=== Magnetometer Calibration ===");
  Serial.print("  Slowly rotate the device through ALL orientations for ");
  Serial.print(durationMs / 1000); Serial.println(" seconds.");

  float mxMin = 1e9f, myMin = 1e9f, mzMin = 1e9f;
  float mxMax = -1e9f, myMax = -1e9f, mzMax = -1e9f;
  uint32_t samples = 0;
  uint32_t start = millis();
  uint32_t lastTick = start;
  while (millis() - start < durationMs) {
    if (imuDev.dataReady()) {
      imuDev.getAGMT();
      if (imuDev.status == ICM_20948_Stat_Ok) {
        float mx = imuDev.magX(), my = imuDev.magY(), mz = imuDev.magZ();
        if (mx < mxMin) mxMin = mx;  if (mx > mxMax) mxMax = mx;
        if (my < myMin) myMin = my;  if (my > myMax) myMax = my;
        if (mz < mzMin) mzMin = mz;  if (mz > mzMax) mzMax = mz;
        samples++;
      }
    }
    if (millis() - lastTick >= 1000) {
      lastTick = millis();
      Serial.print("  t="); Serial.print((millis() - start) / 1000);
      Serial.print("s  samples="); Serial.println(samples);
    }
    delay(10);
  }
  float offX = (mxMax + mxMin) * 0.5f;
  float offY = (myMax + myMin) * 0.5f;
  float offZ = (mzMax + mzMin) * 0.5f;
  Serial.println("=== Calibration complete — paste into source ===");
  Serial.print("  static float magOffX = "); Serial.print(offX, 2); Serial.println("f;");
  Serial.print("  static float magOffY = "); Serial.print(offY, 2); Serial.println("f;");
  Serial.print("  static float magOffZ = "); Serial.print(offZ, 2); Serial.println("f;");
  magOffX = offX; magOffY = offY; magOffZ = offZ;
}

// Refresh all IMU values + derived orientation. Returns true when a fresh
// reading was successfully consumed. Safe to call every loop.
static bool readIMUOnce() {
  if (!imuPresent) return false;
  if (!imuDev.dataReady()) return false;
  imuDev.getAGMT();
  if (imuDev.status != ICM_20948_Stat_Ok) {
    imuErrCount++;
    return false;
  }
  imuReadCount++;
  computeIMUOrientation();
  return true;
}

// One-line human summary for live monitoring.
static void printIMUReadings() {
  if (!imuPresent) {
    Serial.println("[IMU]  not present");
    return;
  }
  Serial.print("[IMU]  A[mg]=");
  Serial.print(imuDev.accX(), 0); Serial.print(",");
  Serial.print(imuDev.accY(), 0); Serial.print(",");
  Serial.print(imuDev.accZ(), 0);
  Serial.print("  G[dps]=");
  Serial.print(imuDev.gyrX(), 1); Serial.print(",");
  Serial.print(imuDev.gyrY(), 1); Serial.print(",");
  Serial.print(imuDev.gyrZ(), 1);
  Serial.print("  M[uT]=");
  Serial.print(imuDev.magX(), 1); Serial.print(",");
  Serial.print(imuDev.magY(), 1); Serial.print(",");
  Serial.print(imuDev.magZ(), 1);
  Serial.print("  T[C]="); Serial.print(imuDev.temp(), 1);
  Serial.print("  reads:"); Serial.print(imuReadCount);
  if (imuErrCount) { Serial.print(" errs:"); Serial.print(imuErrCount); }
  Serial.println();
  Serial.print("[NAV]  roll=");  Serial.print(imuOrient.roll_deg, 1);
  Serial.print("°  pitch=");      Serial.print(imuOrient.pitch_deg, 1);
  Serial.print("°  hdg=");        Serial.print(imuOrient.heading_deg, 1);
  Serial.print("° (");            Serial.print(headingCardinal(imuOrient.heading_deg));
  Serial.print(")  |a|=");        Serial.print(imuOrient.accel_g, 2);
  Serial.print("g  |w|=");        Serial.print(imuOrient.gyro_dps, 1);
  Serial.print("°/s  state=");
  if (imuOrient.isFalling)      Serial.print("FALLING!");
  else if (imuOrient.isTurning) Serial.print("turning");
  else if (imuOrient.isStill)   Serial.print("still");
  else                          Serial.print("moving");
  if (magOffX == 0.0f && magOffY == 0.0f && magOffZ == 0.0f)
    Serial.print("  (mag uncalibrated)");
  Serial.println();
}


// ── HTTP POST helper — conforms to server /nav JSON schema ──────────────────
static void sendNavData(float voltage, float soc, float tempC, uint32_t freeHeap) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(SERVER_URL);
  http.setReuse(true);
  http.addHeader("Content-Type", "application/json");
  // Timeouts (ms): a healthy round-trip is ~30–80 ms (WiFi RTT + server fuse
  // + ~2 KB JSON each way).  An 80 ms ceiling left zero headroom for WiFi
  // retransmits or server fusion of the dual-module packet, producing the
  // "[HTTP] Error: read Timeout" storm.  Per design-doc §2.2.1 R2 the
  // end-to-end latency budget is 200 ms; we leave several × that as the
  // hard abort so a single bad packet doesn't kill the whole stream, but
  // the loop still recovers well before the next POST cycle.
  http.setConnectTimeout(500);   // first TCP handshake / reconnect after roam
  http.setTimeout(800);          // full request+response read budget

  StaticJsonDocument<2048> doc;

  doc["device_id"] = "esp32_power";
  doc["module"]    = "body";
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  uint64_t unix_ms = (uint64_t)tv.tv_sec * 1000ULL + tv.tv_usec / 1000ULL;
  doc["timestamp"] = unix_ms;
  doc["uptime_ms"] = (uint32_t)millis();

  // ToF sensors
  JsonObject tofObj = doc.createNestedObject("tof_sensors");
  for (uint8_t i = 0; i < 8; i++) {
    if (!tofs[i].present || tofs[i].count == 0) {
      tofObj.createNestedObject(tofs[i].name);
    } else {
      JsonObject s = tofObj.createNestedObject(tofs[i].name);
      s["distance_mm"] = tofs[i].last_mm;
      s["count"]       = tofs[i].count;
      s["min"]         = tofs[i].min_mm;
      s["avg"]         = (int16_t)(tofs[i].sum_mm / tofs[i].count);
      s["max"]         = tofs[i].max_mm;
    }
  }

  // mmWave radar sensors
  JsonObject mmwObj = doc.createNestedObject("mmwave_sensors");
  for (uint8_t i = 0; i < 2; i++) {
    RadarSensor& r = radars[i];
    JsonObject rj = mmwObj.createNestedObject(r.name);
    rj["presence"]   = r.presence;
    rj["det_state"]  = r.detState;   // 0=none, 1=stationary, 2=moving
    if (r.frameCount > 0) {
      rj["dist_cm"]  = r.distCm;
      rj["energy"]   = r.energy;
      rj["frames"]   = r.frameCount;
    }
  }

  // ── IMU (ICM-20948) ─────────────────────────────────────────────────────
  // Always emit the object — populated when the IMU is online, empty otherwise.
  // Server side defaults missing fields to 0/false, so a partial drop is safe.
  JsonObject imuObj = doc.createNestedObject("imu");
  if (imuPresent && imuReadCount > 0) {
    // Orientation (degrees) — derived by computeIMUOrientation()
    imuObj["roll_deg"]         = serialized(String(imuOrient.roll_deg,    1));
    imuObj["pitch_deg"]        = serialized(String(imuOrient.pitch_deg,   1));
    imuObj["yaw_deg"]          = serialized(String(imuOrient.heading_deg, 1));
    imuObj["heading_cardinal"] = headingCardinal(imuOrient.heading_deg);

    // Accel — convert mg → m/s² so the schema "accel_z ≈ 9.8 at rest" holds
    const float MG_TO_MPS2 = 9.80665f / 1000.0f;
    imuObj["accel_x"] = serialized(String(imuDev.accX() * MG_TO_MPS2, 2));
    imuObj["accel_y"] = serialized(String(imuDev.accY() * MG_TO_MPS2, 2));
    imuObj["accel_z"] = serialized(String(imuDev.accZ() * MG_TO_MPS2, 2));

    // Gyro (deg/s) — raw scaled
    imuObj["gyro_x"] = serialized(String(imuDev.gyrX(), 1));
    imuObj["gyro_y"] = serialized(String(imuDev.gyrY(), 1));
    imuObj["gyro_z"] = serialized(String(imuDev.gyrZ(), 1));

    // Magnetometer (uT) — raw (server sees the calibration flag separately)
    imuObj["mag_x"] = serialized(String(imuDev.magX(), 1));
    imuObj["mag_y"] = serialized(String(imuDev.magY(), 1));
    imuObj["mag_z"] = serialized(String(imuDev.magZ(), 1));

    imuObj["temp_c"]   = serialized(String(imuDev.temp(), 1));
    imuObj["accel_g"]  = serialized(String(imuOrient.accel_g,  2));
    imuObj["gyro_dps"] = serialized(String(imuOrient.gyro_dps, 1));

    const char* st = "moving";
    if      (imuOrient.isFalling) st = "falling";
    else if (imuOrient.isTurning) st = "turning";
    else if (imuOrient.isStill)   st = "still";
    imuObj["motion_state"]   = st;
    imuObj["is_still"]       = imuOrient.isStill;
    imuObj["is_turning"]     = imuOrient.isTurning;
    imuObj["is_falling"]     = imuOrient.isFalling;
    imuObj["mag_calibrated"] = (magOffX != 0.0f || magOffY != 0.0f || magOffZ != 0.0f);
    imuObj["read_count"]     = imuReadCount;
    imuObj["err_count"]      = imuErrCount;
  }

  // Haptic motor states
  JsonObject hapticObj = doc.createNestedObject("haptic_motors");
  for (uint8_t i = 0; i < 8; i++) {
    if (!drvs[i].present) {
      hapticObj.createNestedObject(drvs[i].dir);
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

  // Power / system data
  JsonObject battery = doc.createNestedObject("battery");
  battery["voltage_v"] = serialized(String(voltage, 3));
  battery["soc_pct"]   = serialized(String(soc,     1));

  JsonObject system = doc.createNestedObject("system");
  system["die_temp_c"]  = serialized(String(tempC, 1));
  system["free_heap_b"] = freeHeap;
  system["wifi_rssi"]   = WiFi.RSSI();

  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  if (code > 0) {
    Serial.print("[HTTP] POST /nav => "); Serial.println(code);

    if (code == 200) {
      String resp = http.getString();

      StaticJsonDocument<2048> resp_doc;
      DeserializationError err = deserializeJson(resp_doc, resp);
      if (err) {
        Serial.print("[HTTP] JSON parse error: "); Serial.println(err.c_str());
      } else {
        // ── Hazard summary ──────────────────────────────────────────────
        int   hazardLevel = resp_doc["hazard"]["level"] | -1;
        const char* hazardLabel = resp_doc["hazard"]["label"] | "?";
        Serial.print("[NAV]  hazard="); Serial.print(hazardLabel);
        Serial.print(" (level="); Serial.print(hazardLevel); Serial.println(")");

        // ── Motor commands from server (print only) ──────────────────────
        JsonArray motors = resp_doc["motors"].as<JsonArray>();
        for (JsonObject m : motors) {
          const char* dir     = m["direction"] | "?";
          bool        active  = m["active"]    | false;
          int         intens  = m["intensity"] | 0;
          const char* zone    = m["zone"]      | "?";
          float       dist_m  = m["distance_m"]| -1.0f;
          Serial.print("[NAV]    motor dir="); Serial.print(dir);
          Serial.print(" active="); Serial.print(active ? "Y" : "N");
          Serial.print(" intensity="); Serial.print(intens);
          Serial.print(" zone="); Serial.print(zone);
          if (dist_m >= 0) { Serial.print(" dist="); Serial.print(dist_m, 3); Serial.print("m"); }
          Serial.println();
        }

        // ── Latency ─────────────────────────────────────────────────────
        float latency = resp_doc["latency_ms"] | -1.0f;
        if (latency >= 0) {
          Serial.print("[NAV]  latency="); Serial.print(latency, 1); Serial.println("ms");
        }
      }
    }
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
  // Flash chip is 16 MB GigaDevice (confirmed by esptool flash-id).
  // Accept 15–17 MB range to allow for reporting variance.
  bool flashOk = (flashSzKB >= 14336 && flashSzKB <= 17408);
  // N8 has NO PSRAM — presence of PSRAM would indicate wrong board config
  bool psramOk = (psramSz == 0);

  if (cpuOk)   printPass("CPU frequency in expected range");
  else         printFail("CPU frequency unexpected");

  if (ramOk)   printPass("Free heap > 100 KB");
  else         printFail("Free heap too low");

  if (flashOk) printPass("Flash ~16 MB detected (GigaDevice chip confirmed)");
  else         printFail("Flash size unexpected — set Tools > Flash Size > 16MB in Arduino IDE");

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
  bool foundIMU       = false;

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
        case 0x68: Serial.print("ICM-20948 9-DoF IMU (AD0=GND)"); foundIMU = true; break;
        case 0x69: Serial.print("ICM-20948 9-DoF IMU (AD0=VCC)"); foundIMU = true; break;
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
  Serial.println("  F/FR/R/BR → MUX ch 0/5/1/7  XSHUT GPIO 19/46/38/25");
  Serial.println("  B/BL/L/FL → MUX ch 2/6/3/4  XSHUT GPIO 21/24/22/3");
  initToFSensors();
  Serial.println();

  // ── TEST 9: DRV2605L Haptic Motors via TCA9548A MUX ———————————————
  Serial.println("[TEST 9] DRV2605L Haptic Motors (MUX 0x70)");
  Serial.println("  n/ne/e/se → MUX ch 0/1/2/3   (front/front_right/right/back_right)");
  Serial.println("  s/sw/w/nw → MUX ch 4/5/6/7   (back/back_left/left/front_left)");
  Serial.println("  NOTE: Motors initialised but RTP kept at 0 (silent) — haptic update disabled.");
  initDrvMotors();
  Serial.println();

  // ── TEST 10: C4001 24 GHz mmWave Radar Sensors ────────────────────────────
  Serial.println("[TEST 10] C4001 24 GHz mmWave Radar Sensors");
  Serial.println("  radar1: TX=GPIO39 RX=GPIO40 OUT=GPIO15  (Serial1)");
  Serial.println("  radar2: TX=GPIO41 RX=GPIO42 OUT=GPIO16  (Serial2)");
  initRadars();
  for (uint8_t i = 0; i < 2; i++) {
    Serial.print("    "); Serial.print(radars[i].name);
    Serial.print(" OUT pin → ");
    Serial.println(radars[i].presence ? "PRESENCE DETECTED" : "no presence");
  }
  Serial.println();

  // ── TEST 11: ICM-20948 9-DoF IMU (J_IMU_Back_Up1) ─────────────────────────
  Serial.println("[TEST 11] ICM-20948 9-DoF IMU @ 0x68 (I2C0 bus, AD0=GND)");
  if (!foundIMU) {
    printFail("ICM-20948 not seen during I2C scan — skipping init");
    Serial.println("  Check: 3V3 (J_IMU pin 2/7), GND (pin 1/12),");
    Serial.println("         SDA→GPIO7 (pin 3), SCL→GPIO8 (pin 4), AD0→GND (pin 5)");
    recordResult("ICM-20948 IMU", false, "not on bus");
  } else {
    imuPresent = initIMU();
    if (imuPresent) {
      printPass("ICM-20948 initialised (Accel + Gyro + Mag online)");
#if IMU_MAG_CALIBRATE_AT_BOOT
      runMagCalibration(20000);
#endif
      delay(20);
      readIMUOnce();
      printIMUReadings();
      recordResult("ICM-20948 IMU", true, "9-DoF online");
    } else {
      printFail("ICM-20948 init failed");
      recordResult("ICM-20948 IMU", false, "init error");
    }
  }
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
      Serial.println("  [WARN] NTP sync failed — timestamps will be 0-based");
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

  // ── Poll mmWave radar OUT pins + drain UART buffers ───────────────────────
  pollRadars();

  // ── Poll the IMU (refreshes raw + derived orientation) ────────────────────
  readIMUOnce();

  // ── Update haptic motors from latest ToF readings ── DISABLED ────────
  // updateHaptics();

  // ── HTTP POST at 10 Hz ────────────────────────────────────────────────────
  static unsigned long lastPost = 0;
  if (now - lastPost >= HTTP_INTERVAL) {
    lastPost = now;
    float voltage = fuelVoltage();
    float soc     = fuelSOC();
    float tempC   = readTempC();
    sendNavData(voltage, soc, tempC, ESP.getFreeHeap());
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

  // Radar presence status with parsed UART data
  for (uint8_t i = 0; i < 2; i++) {
    RadarSensor& r = radars[i];
    Serial.print("[Radar/"); Serial.print(r.name); Serial.print("] ");
    // Presence
    if (r.presence) {
      Serial.print("PRESENT ");
      static const char* stateStr[] = { "(no-motion)", "(stationary)", "(moving)" };
      if (r.detState <= 2) Serial.print(stateStr[r.detState]);
    } else {
      Serial.print("clear");
    }
    // UART-parsed values
    if (r.frameCount > 0) {
      Serial.print(" | dist:");
      if (r.distCm > 0) { Serial.print(r.distCm); Serial.print("cm"); }
      else Serial.print("--cm");
      Serial.print(" energy:"); Serial.print(r.energy);
      Serial.print(" frames:"); Serial.print(r.frameCount);
      Serial.print(" age:"); Serial.print((millis() - r.lastFrameMs)); Serial.print("ms");
    } else if (r.rawDumpLen > 0) {
      // No valid frames yet — dump raw bytes to help diagnose protocol mismatch
      Serial.print(" | UART frames:0  raw[");
      Serial.print(r.rawDumpLen); Serial.print("B]: ");
      for (uint8_t j = 0; j < r.rawDumpLen; j++) {
        if (r.rawDump[j] < 0x10) Serial.print('0');
        Serial.print(r.rawDump[j], HEX); Serial.print(' ');
      }
    } else {
      Serial.print(" | UART frames: 0  no bytes received (check TX/RX wiring)");
    }
    Serial.println();
  }

  // ── ICM-20948 9-DoF IMU: accel/gyro/mag/temp + orientation ───────────────
  printIMUReadings();
}
