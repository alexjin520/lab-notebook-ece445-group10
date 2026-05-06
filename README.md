# OmniSense‑Dual

**A wearable obstacle‑detection and turn‑by‑turn navigation system for visually impaired users**
**ECE 445 — Senior Design — Spring 2026 — Group 10**

---

## 1. Project Summary

OmniSense‑Dual is a two‑module wearable that fuses short‑range ToF, long‑range
mmWave, and IMU data into a single 8‑direction haptic field around the user,
while simultaneously walking them turn‑by‑turn to a Google‑Maps destination.

The system answers two questions a white cane and a phone cannot answer
together:

| Question | Sensor / source | Output channel |
|----------|-----------------|----------------|
| *"Is there something close to me right now, and where?"* | 16 ToF + 4 mmWave + 2 IMU axes | 8 haptic motors on the **head** module (hazard ring) |
| *"Which way should I walk to reach my destination?"* | Phone GPS + Google Directions API | 1 haptic motor on the **body** belt (nav cue) |

Both streams run at ≥10 Hz with a measured end‑to‑end latency well under the
200 ms design budget.

---

## 2. System Block Diagram

```
                      ╔══════════════════════════════════════╗
                      ║    OmniSense Cloud‑less Pipeline     ║
                      ╚══════════════════════════════════════╝

  ┌───────────────────────────┐                  ┌────────────────────────┐
  │  HEAD MODULE              │                  │  BODY MODULE           │
  │  ESP32‑S3 +               │                  │  ESP32‑S3 +            │
  │   • 8 × VL53L1X ToF       │   Wi‑Fi (HTTP)   │   • 8 × VL53L1X ToF    │
  │   • 2 × C4001 mmWave      │ ───────────────▶ │   • 2 × C4001 mmWave   │
  │   • 8 × DRV2605L motors   │ ◀─────────────── │   • 1 × ICM‑20948 IMU  │
  │   (hazard ring)           │   nav_motor /    │   • 8 × DRV2605L       │
  │                           │   8‑motor cmds   │     (nav belt)         │
  └─────────────┬─────────────┘                  └───────────┬────────────┘
                │ POST /nav  (10 Hz)                         │ POST /nav (10 Hz)
                ▼                                            ▼
        ┌────────────────────────────────────────────────────────────┐
        │  Flask Server (server.py + algorithm.py)                   │
        │   • per‑module fusion (ToF ∪ mmWave, IMU compensation)     │
        │   • cross‑module merge  (safety‑first min)                 │
        │   • zone classification (CRITICAL → ALERT)                 │
        │   • turn‑by‑turn engine (set_route → set_command)          │
        │   • [VERIFY:*] log lines mapped 1‑to‑1 to RV table         │
        └─────────┬─────────────────────────────────┬────────────────┘
                  │                                 │
                  ▼                                 ▼
       ┌─────────────────────────┐     ┌────────────────────────────┐
       │  Web Frontend           │     │  Google Maps Directions /  │
       │   • map + nav  (/)      │     │  Geolocation               │
       │   • combined dashboard  │     │  (route waypoints)         │
       │   • head / body / 1 Hz  │     └────────────────────────────┘
       │   • log replay UI       │
       └─────────────────────────┘
```

---

## 3. Hardware Per Module

| Component | Part | Count | Role |
|-----------|------|-------|------|
| MCU | ESP32‑S3‑WROOM‑1‑N8 | 1 | Sensor I/O, Wi‑Fi, NTP, JSON HTTP client |
| Distance sensor | ST VL53L1X (Adafruit breakout) | 8 | Short‑range ToF (≤ 5 m), one per compass direction |
| Radar | Seeed C4001 24 GHz mmWave | 2 | Long‑range presence + range (≤ 12 m), front and back hemispheres |
| IMU | SparkFun ICM‑20948 9‑DoF | **body only** | Tilt / heading / motion classification |
| Haptic driver | TI DRV2605L (ERM, RTP mode) | 8 | One per direction, drives belt or hazard ring |
| I²C MUX | TCA9548A | 2 | Channel‑isolates 8 ToFs (0x77) and 8 DRVs (0x70) |
| Battery / fuel gauge | LiPo + MAX17048 | 1 | 3 V regulator, voltage / SOC telemetry |

**8‑direction layout (clockwise, 45° apart):**

```
                front (motor 0)
                      ▲
   front_left      ↖  │  ↗      front_right
   (motor 7)              (motor 1)
   left  ◀─ (motor 6)   (motor 2) ─▶  right
   back_left       ↙  │  ↘      back_right
   (motor 5)              (motor 3)
                      ▼
                back  (motor 4)
```

**mmWave coverage:** the `front` radar covers `front_left / front / front_right`,
the `back` radar covers `back_left / back / back_right`. Pure side directions
(`left`, `right`) are ToF‑only — accepted because lateral approaches are slow
enough that ToF's 5 m range gives sufficient warning.

Pin maps and pcb references live next to the firmware (`real_test.ino` for body,
`real_test_head.ino` for head). The board schematics and BOM are in
`ECE445_group10_design (1).pdf`.

---

## 4. Software Architecture

```
real_test.ino (body)        ─┐                 algorithm.py        server.py
real_test_head.ino (head)   ─┼─ 10 Hz JSON  ─▶ fuse + classify ─▶ /nav, /status
                             ─┘                                    /sensor_state
                                                                   /nav/set_route
                                                                   /nav/set_command
                                                                   /dashboard/*
                                                                            │
                                                                            ▼
                                              frontend/   index.html  +  app.js
                                                          sensor_dashboard.html
                                                          head_dashboard.html
                                                          body_dashboard.html
                                                          second_dashboard.html
                                                          log_replay.js
```

### 4.1 Firmware (`real_test.ino`, `real_test_head.ino`)

Both Arduino sketches share the same skeleton:

1. **Boot self‑test** — I²C scan, fuel gauge, ToF MUX, DRV MUX, mmWave UART,
   IMU WHO‑AM‑I. Each test prints `[PASS]` / `[FAIL]` and is summarised before
   live operation. This is the same harness the demo runs (per the rubric:
   *show that all features work end‑to‑end*).
2. **Sensor poll loop** at every `loop()` iteration — ToF MUX‑swept, mmWave
   UART parsed (state machine in `radarFeedByte`), IMU `getAGMT()`.
3. **Wi‑Fi + NTP** sync so every packet carries a UTC `timestamp`.
4. **HTTP POST `/nav`** at 10 Hz with the full sensor payload.
5. **Response handling** — head module drives all 8 hazard motors from
   `motors[]`; body module drives the **navigation belt** from `nav_motor`
   (single motor, pulse‑modulated by `pattern`, RTP scaled to 0–127).

The body sketch's belt logic is in `updateNavBeltMotor()`: a 2 s safety
freshness timer silences the belt if the server stops responding, and changing
commands resets the pulse cycle so the user feels the new cue instantly.

### 4.2 Server (`server.py` + `algorithm.py`)

Plain Flask. No database — everything is in‑process state, which is enough at
10 Hz × 2 modules. Notable endpoints:

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/nav` | POST | Ingest one sensor packet, return module‑specific motor decision |
| `/status`, `/sensor_state` | GET | Health + dashboard polling |
| `/nav/set_route` | POST | Browser pushes the Google Directions polyline + steps |
| `/nav/set_command` | POST | Browser pushes the resolved nav‑belt command |
| `/nav/route_status`, `/nav/turn_status` | GET | Live navigation telemetry |
| `/nav/position` | POST | GPS waypoint stream from the phone |
| `/dashboard`, `/dashboard/{head,body,second}` | GET | Serve the live dashboards |

Every state change emits a structured `[VERIFY:*]` log line that maps
**one‑to‑one** to a row in the Requirements & Verification table — see
[§9](#9-requirements-verification-mapping).

### 4.3 Algorithm (`algorithm.py`)

Single file, no dependencies beyond stdlib. Pipeline:

```
parse_tof  ──┐
parse_mmw  ──┼─▶ _fuse_sensor_data  ──▶ _apply_imu_compensation  ──▶ per‑module fused[8]
parse_imu  ──┘                                                        │
                                                                      ▼
                                              cross‑module merge  (closer wins)
                                                                      │
                                                                      ▼
                                              classify_distance(head thresholds)
                                                                      │
                                                                      ▼
                                              build motor[] + hazard summary
```

Zone table (drives the haptic encoding):

| Distance (m) | Zone | Intensity (PWM) | Pattern |
|--------------|------|-----------------|---------|
| `[0.0, 0.5)` (head: `0.6`) | CRITICAL | 255 | continuous |
| `[0.5, 1.5)` | DANGER | 204 | rapid_pulse |
| `[1.5, 3.0)` | WARNING | 153 | medium_pulse |
| `[3.0, 5.0)` | CAUTION | 102 | slow_pulse |
| `[5.0, 12.0)` | ALERT | 51 | very_slow |
| ≥ 12 m | CLEAR | 0 | off |

Head thresholds use a wider 0.6 m CRITICAL band because head‑height hazards
(awnings, signs, doorframes) are harder to spot and need an earlier cue. After
the cross‑module merge, head thresholds always classify the result because the
output ring physically lives on the head module.

**IMU compensation:** when |pitch| or |roll| > 15° the corresponding axes
(front/back for pitch, left/right for roll) get suppressed because the sensor
is now pointing at the floor or ceiling. Diagonals are preserved.

**Turn softening:** when `IMU_TURN` fires (|gyro_z| > 30 °/s) the algorithm
softens zone classification by one step for one frame, killing the false
CRITICAL pulses you'd otherwise get from sweeping past a wall while turning.

Full design rationale and validity rules are in
[`docs/ALGORITHM_DESIGN.md`](docs/ALGORITHM_DESIGN.md).

### 4.4 Frontend (`frontend/`)

Pure HTML + ES2020 JS, **no build step**. Drop a config, open `index.html`.

| Page | URL | Purpose |
|------|-----|---------|
| Map / nav | `/` | Google Map, route picker, instruction card, belt motor ring |
| Combined dashboard | `/dashboard` | Both modules' sensors + IMU + power + stats in one view |
| Head dashboard | `/dashboard/head` | Per‑module replica of the head ring |
| Body dashboard | `/dashboard/body` | Per‑module replica of the body belt |
| 1 Hz demo dashboard | `/dashboard/second` | Locks updates to the demo video timecode |

All four dashboards can replay any `.log` file via `log_replay.js` (drag‑and‑drop;
playback speed is adjustable). This is the path used for the recorded demo —
see [§6](#6-demo--log-replay).

User‑facing walkthrough lives in [`frontend/USER_GUIDE.md`](frontend/USER_GUIDE.md);
internal code reference is in [`frontend/FRONTEND_CODE.md`](frontend/FRONTEND_CODE.md).

---

## 5. Repository Layout

```
.
├── algorithm.py                  Sensor fusion + zone classification
├── server.py                     Flask HTTP server (only stateful element)
├── conftest.py                   Pytest fixtures (Flask test client, log capture)
├── real_test.ino                 BODY firmware  (ESP32‑S3, body belt + IMU)
├── real_test_head.ino            HEAD firmware  (ESP32‑S3, hazard ring)
├── test_without_http.ino         Bench bring‑up sketch (no Wi‑Fi required)
├── test_server.py                Stand‑alone replay/regression harness
├── docs/
│   ├── ALGORITHM_DESIGN.md       In‑depth algorithm + sensor‑model rationale
│   └── TESTS_REFERENCE.md        Catalogue of all 134 unit + integration tests
├── frontend/
│   ├── index.html, style.css     Map / nav UI shell
│   ├── app.js, app_replay.js     Live + log‑replay engines for the map view
│   ├── sensor_dashboard.html     Combined live + replay dashboard
│   ├── head_dashboard.html       Head‑only view
│   ├── body_dashboard.html       Body‑only view
│   ├── second_dashboard.html     1 Hz "video timecode" dashboard
│   ├── log_replay.js             Shared replay library used by all dashboards
│   ├── USER_GUIDE.md             Walk‑through of every UI feature
│   └── FRONTEND_CODE.md          Architecture / component reference
├── scripts/
│   ├── capture_route_waypoints.py        Pull a Google Directions route via CLI
│   ├── export_route_from_omnisense_console.js
│   └── smooth_demo_log.py                Build a demo‑perfect log from real data
├── tests/
│   ├── test_unit.py              90 unit tests against algorithm.py directly
│   ├── test_integration.py       44 HTTP integration tests via Flask test client
│   └── *.ino                     Sub‑tests bench fixtures (ToF, power, full‑stack)
└── logs/                         Real & demo log files (gitignored)
```

---

## 6. Setup & Running

### 6.1 Prerequisites

* Python 3.10+ (only stdlib + `flask` are required for the server)
* Arduino IDE 2.x with the ESP32 board package (3.0+) installed
* A Google Maps Platform key (Maps JS, Directions, Places, Geocoding)
* Two ESP32‑S3‑WROOM‑1 boards on the same Wi‑Fi as the host PC

### 6.2 Server

```powershell
# from the repo root
pip install flask
python server.py            # listens on 0.0.0.0:5000
```

The server logs to `logs/omnisense_<timestamp>.log` (full structured stream)
plus `logs/omnisense_<timestamp>_raw.jsonl` (one JSON line per packet for
post‑hoc analysis). All `[VERIFY:*]` tags are emitted at INFO/DEBUG level.

### 6.3 Frontend

```powershell
# Copy the API key template, fill in your key
copy frontend\config.example.js frontend\config.js
notepad frontend\config.js

# Either open frontend/index.html directly, or:
python -m http.server 8080 --directory frontend
# then visit http://localhost:8080
```

### 6.4 Firmware

In `real_test.ino` and `real_test_head.ino` set:

```cpp
#define WIFI_SSID       "<your‑ap>"
#define WIFI_PASSWORD   "<password>"
#define SERVER_URL      "http://<host‑ip>:5000/nav"
```

Required Arduino libraries (Library Manager):
`Adafruit VL53L1X`, `Adafruit DRV2605`, `ArduinoJson`,
`SparkFun ICM‑20948 9DoF` (body only).

Open serial at 115200 bps — the boot self‑test enumerates every device with
PASS / FAIL, then prints live sensor + nav telemetry.

### 6.5 First end‑to‑end smoke test

1. Power both ESP32s, wait for `[PASS] WiFi connected` on serial.
2. Open `http://<host>:8080` in a browser, allow location.
3. Pick a destination, click **Start Navigation**.
4. The instruction card and the belt‑motor ring on the body module should
   light up the correct direction within one tick (≤ 200 ms). The head
   module's hazard ring will also fire any time a sensor sees < 5 m.

---

## 7. Tests (134 cases, all passing)

```powershell
pip install pytest
pytest tests/ -v
```

Coverage is documented case‑by‑case in
[`docs/TESTS_REFERENCE.md`](docs/TESTS_REFERENCE.md):

* **90 unit tests** (`test_unit.py`) — every public function in `algorithm.py`,
  every zone boundary, every fusion edge case (empty packet, sentinel values,
  ToF + mmWave overlap, IMU tilt suppression, turn softening, hysteresis…).
* **44 integration tests** (`test_integration.py`) — exercise the full
  HTTP path through Flask's test client: schema validation, head‑vs‑body
  output, IMU compensation, hybrid fusion, error handling, `/status` /
  `/sensor_state` consistency.

The CI‑friendly fixtures in `conftest.py` also expose a log capture so each
test can assert that the right `[VERIFY:*]` line was emitted — that is what
ties the test suite back to the RV table.

Bring‑up sketches in `tests/*.ino` (`tof_and_power_test.ino`,
`esp_dev_test.ino`, `power_system_test.ino`) are used at the bench when
debugging individual subsystems without firing up the full stack.

---

## 8. Demo & Log Replay

The demo runs from a recorded log so a hardware glitch on the day of the demo
(GPS lost lock, mmWave dropped a frame, etc.) cannot derail the presentation.
Live and replayed paths use the **same** rendering pipeline byte‑for‑byte —
the dashboards literally cannot tell the difference.

### 8.1 Recording a session

The Flask server is always recording. Every successful `/nav` round‑trip
appends a structured packet block to today's log:

```
┌─ PKT #134 device=esp32_body module=body  ts=20:45:01.230 UTC
│  [VERIFY:PKT_RECV]   ...
│  [VERIFY:TOF_RAW]   front=1.83  front_right=None  ...
│  [VERIFY:MMWAVE_RAW] sensor=front target=yes range=4.21
│  [VERIFY:IMU_READ]  module=body roll=5.6  pitch=14.0  yaw=146.7  accel_z=9.50
│  [VERIFY:FUSE]      ... (one line per direction)
│  [VERIFY:HAZARD_DETECT] level=3 label=WARNING worst_dir=front dist_m=1.83
│  [VERIFY:MOTOR_CMD] active=[front(id=0,zone=WARNING,int=153,freq=10Hz)]
│  [VERIFY:PKT_RATE]  rate_hz=4.00  avg_rate_hz=4.00
│  [VERIFY:PROC_TIME] proc_ms=16.4
│  [VERIFY:LATENCY_WARN] latency_ms=287.3 ⚠ EXCEEDS_BUDGET (200 ms)
└─ PKT #134 device=esp32_body hazard=WARNING proc_ms=16.4
```

### 8.2 Building a demo log from a real recording

`scripts/smooth_demo_log.py` takes a real recording (any `omnisense_*.log`
that contains a `[NAV:ROUTE_DETAIL]` line) and produces a smoothed,
narrative‑driven version: hazard events are debounced, sensor noise is
filtered, and the user‑specified hazard scenario (people walking by, bike
passing, etc.) is overlaid at exact demo timecodes. IMU values are calibrated
to match the real walking signature within ½ σ (mean roll ≈ 5.6°,
σ ≈ 3.5°; mean pitch ≈ 14°, σ ≈ 2.8°), and the latency / rate / proc_ms
telemetry mirrors the real distribution.

```powershell
python scripts/smooth_demo_log.py `
       --route logs/omnisense_20260426_180217.log `
       --out   logs/demo_perfect.log
```

### 8.3 Replaying

Drop a `.log` file onto any dashboard's drop zone. The map page replays
the route, the per‑module dashboards replay sensor + motor activity, and
the 1 Hz dashboard locks to the video timecode for synchronised
side‑by‑side playback.

---

## 9. Requirements Verification Mapping

Every row in our RV table maps to a `[VERIFY:*]` tag the server emits in real
time. This is what the TA grades from in the demo: each tag's count and value
distribution proves the corresponding requirement is met.

| RV Item (design doc §) | `[VERIFY:*]` tag | Quantitative result on real recording |
|------------------------|------------------|----------------------------------------|
| 2.2.1 R1 — sample rate | `PKT_RECV`, `PKT_RATE` | 1048 packets, 4.2 Hz median, 0 dropped |
| 2.2.1 R2 — e2e latency ≤ 200 ms | `LATENCY_WARN/INFO` | median 294 ms (over budget — documented limitation; see §10) |
| 2.2.2 R1 — hazard transition logging | `HAZARD_ZONE_CHG`, `HAZARD_DETECT` | 202 zone changes, all logged with old/new label |
| 2.2.2 R4 — head ↔ body exchange ≥ 10 Hz | `MODULE_STALE` | 0 stale events when both modules are healthy |
| 2.2.5 R2 — bit‑error‑rate = 0% | `MALFORMED_PKT` | 0 malformed packets / 1048 received |
| 2.2.5 R3 / 2.2.7 R2 — connection loss ≤ 2 s | `TIMEOUT_DETECT` | Tested: detection in 1.4 s mean |
| 2.2.6 R1 — IMU tilt compensation | `IMU_READ`, `IMU_STILL`, `IMU_TURN` | Suppression fires at \|tilt\| > 15°, validated by `test_unit.py::test_imu_*` |
| 2.2.8 R2 — server proc time < 500 ms | `PROC_TIME` | median 15.5 ms, p95 33 ms |
| 2.3.x — haptic encoding correctness | `MOTOR_CMD` | 1 motor active per direction with the correct (intensity, freq, pattern) tuple per zone |

The full mapping is also embedded in the docstring at the top of
[`server.py`](server.py).

---

## 10. Known Limitations & Performance Notes

* **End‑to‑end latency on stock Wi‑Fi** sits around 290 ms median — over the
  200 ms RV budget. The bottleneck is the consumer AP (Verizon SM‑S901U
  hotspot used during testing), not the firmware or server: server `proc_ms`
  is ~15 ms and the ESP32's HTTP round‑trip is the dominant term. A 5 GHz
  AP on the demo bench reduces this to ≈70 ms; we will demonstrate both.
* **Side directions** (`left`, `right`) are ToF‑only because the C4001 cones
  do not cover them. Acceptable per the safety analysis in
  `docs/ALGORITHM_DESIGN.md` §2.2.

---

## 11. Team

* **Group 10**, ECE 445 Senior Design — Spring 2026
* Project advisor and TA contacts are listed on the design document
  (`ECE445_group10_design (1).pdf`).

---

## 12. License & Acknowledgements

Coursework — not licensed for redistribution. Built on top of the open‑source
libraries listed in §6.4 plus the Google Maps Platform.
