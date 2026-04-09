# OmniSense-Dual Algorithm Design Document
**ECE 445 Group 10**

---

## 1. System Overview

OmniSense-Dual is a wearable obstacle-detection system for visually impaired
users.  It consists of **two independent modules** worn on the body:

| Module | Placement | Purpose |
|--------|-----------|---------|
| **Head** | Head / helmet | Detects head-height hazards (awnings, signs, doorframes, overhangs) |
| **Body** | Waist / torso | Detects body-level hazards (furniture, walls, people, vehicles) |

Each module runs its own ESP32-S3 microcontroller and transmits sensor packets
to this Flask server over Wi-Fi.  The server fuses the sensor data and returns
a set of 8 haptic motor commands that the ESP32 drives in real-time.

---

## 2. Hardware Configuration Per Module

```
┌─────────────────────────────────────────────────────────┐
│                   Module Hardware                        │
│                                                          │
│   8 × VL53L1X ToF Sensors      (range ≤ 5 m)           │
│   2 × TI mmWave Radars          (range ≤ 12 m)          │
│   1 × MPU-6050 IMU              (tilt / orientation)     │
│   8 × Vibration Motors          (haptic feedback)        │
└─────────────────────────────────────────────────────────┘
```

### 2.1 8-Direction Layout (Clockwise, 45° apart)

```
                    FRONT (Motor 0)
                        ↑
       FRONT-LEFT    ↖     ↗    FRONT-RIGHT
       (Motor 7)                  (Motor 1)

  LEFT ←  (Motor 6)           (Motor 2)  → RIGHT

       BACK-LEFT     ↙     ↘    BACK-RIGHT
       (Motor 5)                  (Motor 3)
                        ↓
                    BACK (Motor 4)
```

Motor index maps 1-to-1 with sensor direction:

| Index | Direction   | Angle | ToF Sensor | mmWave Coverage |
|-------|-------------|-------|------------|-----------------|
| 0     | front       | 0°    | Yes        | front mmWave    |
| 1     | front_right | 45°   | Yes        | front mmWave    |
| 2     | right       | 90°   | Yes        | **none**        |
| 3     | back_right  | 135°  | Yes        | back mmWave     |
| 4     | back        | 180°  | Yes        | back mmWave     |
| 5     | back_left   | 225°  | Yes        | back mmWave     |
| 6     | left        | 270°  | Yes        | **none**        |
| 7     | front_left  | 315°  | Yes        | front mmWave    |

### 2.2 mmWave Sensor Placement

Each module carries **two** mmWave radars with a ≈ ±67.5° azimuth field of
view.  They are mounted on the front and back faces of the module:

```
      Front mmWave (covers front_left, front, front_right)
              ╔═══════╗
       ╔═════╣       ╠═════╗
       ║     ╚═══════╝     ║
       ║    ( module body ) ║
       ║     ╔═══════╗     ║
       ╚═════╣       ╠═════╝
              ╚═══════╝
      Back mmWave  (covers back_left, back, back_right)
```

The **left** and **right** directions are covered by ToF only (up to 5 m).
This is acceptable because lateral approaches are slower and ToF range is
sufficient for timely warnings.

---

## 3. Sensor Specifications

| Sensor      | Model       | Max Range | Accuracy | Update Rate |
|-------------|-------------|-----------|----------|-------------|
| ToF         | VL53L1X     | 5 m       | ±3 %     | 50 Hz       |
| mmWave      | TI IWR6843  | 12 m      | ±5 cm    | 20 Hz       |
| IMU         | MPU-6050    | N/A       | ±0.1°    | 100 Hz      |

---

## 4. Hybrid Sensor Fusion Algorithm

### 4.1 Design Philosophy

Neither sensor alone is sufficient:

- **ToF only**: 5 m range gives roughly 2–3 seconds of warning at walking
  speed – sufficient for close hazards but too short for fast-moving objects
  (cyclists, cars).
- **mmWave only**: 12 m range but lower spatial precision, limited ability
  to discriminate subtle distances below 1 m, and noisier in clutter.

The **hybrid approach** exploits both sensors' strengths:

```
 Distance   0──────────────────────5 m────────────────────12 m
            │                      │                        │
 Sensor     │    ToF  (precise)    │    mmWave  (long)      │
            │   ← primary zone →   │  ← extension zone →   │
```

### 4.2 Fusion Rules (Per Direction)

```
for each direction in [front, front_right, right, back_right,
                       back, back_left, left, front_left]:

    tof_dist  = ToF reading for this direction  (or None if clear / invalid)
    mw_dist   = mmWave range for the sensor covering this direction
                (or None if no target / not covered)

    if   tof_dist is not None  AND  mw_dist is not None:
        effective_distance = min(tof_dist, mw_dist)   # safety-first
        source = "tof+mmwave"

    elif tof_dist is not None:
        effective_distance = tof_dist
        source = "tof"

    elif mw_dist is not None:                          # beyond ToF range
        effective_distance = mw_dist                   # or ToF missed
        source = "mmwave"

    else:
        effective_distance = None                      # direction clear
        source = "none"

    zone = classify(effective_distance)
    motor_command[direction] = vibration_pattern(zone)
```

**Key safety rule:** when both sensors return a distance, the **minimum**
is always used.  We never suppress a hazard because one sensor disagrees.

### 4.3 ToF Validity

A ToF reading is considered valid only if:
- `avg_distance_mm` > 0
- `avg_distance_mm` < 8190 mm  (VL53L1X "no target" sentinel = 8190)
- `avg_distance_mm` ≤ 5000 mm  (beyond max usable range → discard)

The **average** field is preferred over the raw sample to reduce noise.

### 4.4 mmWave Validity

An mmWave reading is considered valid only if:
- `targets` ≥ 1
- `range_m` > 0  and  `range_m` ≤ 12.0 m

---

## 5. Hazard Zone Classification

The effective distance is mapped to one of six hazard zones:

```
  Distance (m)    Zone      Motor Intensity   Frequency   Pattern
  ─────────────────────────────────────────────────────────────────
  [0.0,  0.5)    CRITICAL       255           50 Hz       continuous
  [0.5,  1.5)    DANGER         204           20 Hz       rapid_pulse
  [1.5,  3.0)    WARNING        153           10 Hz       medium_pulse
  [3.0,  5.0)    CAUTION        102            5 Hz       slow_pulse
  [5.0, 12.0)    ALERT           51            2 Hz       very_slow
  [12.0,  ∞)    CLEAR             0            0 Hz       off
```

**Head module exception:** the CRITICAL threshold is widened to **0.6 m**
(vs 0.5 m for body) because head-height obstacles (overhangs, signs, doorframes)
are less visible and require a longer reaction window — the user cannot see
the hazard approaching at head level and may need extra time to duck or turn.

### 5.1 Vibration Encoding

The vibration parameters sent to each motor PWM driver:

```json
{
  "intensity":    153,           // 0–255 PWM duty cycle
  "frequency_hz": 10,            // pulse rate in Hz
  "pattern":      "medium_pulse",// human-readable tag for firmware
  "active":       true           // false → motor off immediately
}
```

---

## 6. IMU Tilt Compensation

When the user bends forward (high pitch angle), the front-facing ToF sensor
points at the ground instead of the horizon, producing spurious sub-1 m
readings that would trigger false CRITICAL alerts.

### 6.1 Suppression Logic

```
if |pitch_deg| > 15°:
    suppress("front")
    suppress("back")

if |roll_deg| > 15°:
    suppress("left")
    suppress("right")

suppress(direction):
    effective_distance[direction] = None
    zone[direction]               = CLEAR
    source[direction]             = "suppressed(tilt)"
```

Only the two axes most aligned with the tilt are suppressed.  Diagonal
directions (front_right, front_left, back_right, back_left) are not
suppressed because a single-axis tilt rarely misaligns them enough to
cause false readings.

### 6.2 Threshold Choice

15° was chosen as a balance between:
- Suppressing false ground-reflection readings (appear below ≈ 12° pitch)
- Not suppressing valid hazard detection during normal walking gait (±8° pitch)

---

## 7. Module Distinction

The `module` field in the request payload routes processing to
module-specific tuning:

| Feature                 | `"head"` module      | `"body"` module   |
|-------------------------|----------------------|-------------------|
| CRITICAL threshold      | < 0.6 m (wider zone) | < 0.5 m           |
| Primary hazard type     | Overhead / head-height | Lateral / torso   |
| Typical mounting height | ~170–190 cm          | ~90–110 cm        |

Future extensions may apply different IMU compensation axes (e.g., head nodding
vs body leaning) or different mmWave sensitivity settings per module.

---

## 8. Data Flow

```
ESP32 HEAD                     Server (algorithm.py)              ESP32 HEAD
┌──────────┐  POST /nav JSON  ┌──────────────────────────────┐  JSON response
│ 8× ToF   │ ──────────────▶ │  1. Parse tof_sensors        │ ──────────────▶
│ 2× mmWave│                 │  2. Parse mmwave_sensors      │  8 motor cmds
│ 1× IMU   │                 │  3. Parse imu                 │
└──────────┘                 │  4. Hybrid fusion             │
                             │  5. IMU tilt compensation     │
ESP32 BODY                   │  6. Generate motor commands   │  ESP32 BODY
┌──────────┐  POST /nav JSON  │  7. Compute hazard summary   │  JSON response
│ 8× ToF   │ ──────────────▶ │  8. Update device state      │ ──────────────▶
│ 2× mmWave│                 └──────────────────────────────┘  8 motor cmds
│ 1× IMU   │
└──────────┘
```

---

## 9. JSON API Reference

### 9.1 Request  `POST /nav`

```json
{
  "device_id":  "esp32_head",
  "module":     "head",

  "tof_sensors": {
    "front":       {"distance_mm": 1200, "avg": 1200, "min": 1100, "max": 1300, "count": 50},
    "front_right": {"distance_mm": 2500, "avg": 2500, "min": 2480, "max": 2520, "count": 50},
    "right":       {"distance_mm":  800, "avg":  800, "min":  790, "max":  810, "count": 50},
    "back_right":  {"distance_mm": 3000, "avg": 3000, "min": 2990, "max": 3010, "count": 50},
    "back":        {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "back_left":   {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "left":        {"distance_mm":  600, "avg":  600, "min":  590, "max":  610, "count": 50},
    "front_left":  {"distance_mm": 1800, "avg": 1800, "min": 1790, "max": 1810, "count": 50}
  },

  "mmwave_sensors": {
    "front": {"targets": 1, "range_m": 7.5, "speed_ms": 0.11, "energy": 10576},
    "back":  {"targets": 0, "range_m": 0.0, "speed_ms": 0.0,  "energy": 0}
  },

  "imu": {
    "roll_deg":  2.5,
    "pitch_deg": -1.2,
    "yaw_deg":   45.0,
    "accel_x":   0.01,
    "accel_y":   0.02,
    "accel_z":   9.8
  },

  "timestamp": 37393
}
```

**Field notes:**
- `module`: **Required**. `"head"` or `"body"`. Any other value is treated as `"body"`.
- `tof_sensors.*`: VL53L1X reading per direction. Use `avg` for noise reduction.
  Value `8190` indicates no target detected (sensor sentinel).
- `mmwave_sensors.{front,back}.targets`: number of detected targets. `0` = clear.
- `imu.accel_z` ≈ 9.81 m/s² at rest (gravity).
- `timestamp`: ESP32 `millis()` value (milliseconds since boot), used for latency measurement.

### 9.2 Response  `POST /nav`

```json
{
  "status":    "ok",
  "device_id": "esp32_head",
  "module":    "head",

  "hazard": {
    "level":  3,
    "label":  "WARNING",
    "alerts": [
      {"direction": "right",      "zone": "WARNING", "distance_m": 1.8, "source": "tof"},
      {"direction": "front",      "zone": "ALERT",   "distance_m": 7.5, "source": "mmwave"},
      {"direction": "front_left", "zone": "ALERT",   "distance_m": 7.5, "source": "mmwave"},
      {"direction": "front_right","zone": "ALERT",   "distance_m": 7.5, "source": "mmwave"}
    ]
  },

  "motors": [
    {"motor_id": 0, "direction": "front",       "intensity": 51,  "frequency_hz": 2,
     "pattern": "very_slow",   "active": true,  "zone": "ALERT",   "distance_m": 7.5, "source": "mmwave"},
    {"motor_id": 1, "direction": "front_right", "intensity": 51,  "frequency_hz": 2,
     "pattern": "very_slow",   "active": true,  "zone": "ALERT",   "distance_m": 7.5, "source": "mmwave"},
    {"motor_id": 2, "direction": "right",       "intensity": 153, "frequency_hz": 10,
     "pattern": "medium_pulse","active": true,  "zone": "WARNING", "distance_m": 1.8, "source": "tof"},
    {"motor_id": 3, "direction": "back_right",  "intensity": 0,   "frequency_hz": 0,
     "pattern": "off",         "active": false, "zone": "CLEAR",   "distance_m": null,"source": "none"},
    {"motor_id": 4, "direction": "back",        "intensity": 0,   "frequency_hz": 0,
     "pattern": "off",         "active": false, "zone": "CLEAR",   "distance_m": null,"source": "none"},
    {"motor_id": 5, "direction": "back_left",   "intensity": 0,   "frequency_hz": 0,
     "pattern": "off",         "active": false, "zone": "CLEAR",   "distance_m": null,"source": "none"},
    {"motor_id": 6, "direction": "left",        "intensity": 0,   "frequency_hz": 0,
     "pattern": "off",         "active": false, "zone": "CLEAR",   "distance_m": null,"source": "none"},
    {"motor_id": 7, "direction": "front_left",  "intensity": 51,  "frequency_hz": 2,
     "pattern": "very_slow",   "active": true,  "zone": "ALERT",   "distance_m": 7.5, "source": "mmwave"}
  ],

  "imu":        {"roll_deg": 2.5, "pitch_deg": -1.2, "yaw_deg": 45.0},
  "latency_ms": 12.4
}
```

**Field notes:**
- `hazard.level`: 0 (CLEAR) → 5 (CRITICAL).  Use this as a quick summary.
- `hazard.alerts`: sorted most-critical first; contains only non-CLEAR directions.
- `motors[i].source`: `"tof"` | `"mmwave"` | `"tof+mmwave"` | `"none"` | `"suppressed(tilt)"`.
- `motors[i].distance_m`: `null` when no obstacle detected or channel suppressed.
- `latency_ms`: server receipt time minus `timestamp`; useful for diagnosing
  Wi-Fi lag.

### 9.3  `GET /status`

```json
{
  "status": "running",
  "devices": {
    "esp32_head": {
      "module":       "head",
      "last_zone":    "WARNING",
      "packet_count": 142,
      "last_seen_ms": 1712678400000
    },
    "esp32_body": {
      "module":       "body",
      "last_zone":    "CLEAR",
      "packet_count": 140,
      "last_seen_ms": 1712678400050
    }
  }
}
```

---

## 10. Algorithm State Machine (Per Direction)

```
              ┌──────────────────────────────────────────────────────┐
  Packet      │            Hybrid Fusion Decision                    │
  arrives     │                                                      │
─────────────▶│  ToF valid?  ──Yes──▶  mmWave covers & detected?   │
              │      │                   ──Yes──▶  min(ToF, mWave)  │
              │      │                   ──No───▶  ToF only         │
              │      No                                              │
              │      │                                              │
              │      └──────▶  mmWave covers & detected?           │
              │                   ──Yes──▶  mmWave only             │
              │                   ──No───▶  CLEAR                   │
              └──────────────────────────────────────────────────────┘
                                    │
                                    ▼
              ┌──────────────────────────────────────────────────────┐
              │              IMU Compensation                        │
              │  |pitch| > 15° → zero out front / back              │
              │  |roll|  > 15° → zero out left / right              │
              └──────────────────────────────────────────────────────┘
                                    │
                                    ▼
              ┌──────────────────────────────────────────────────────┐
              │           Zone Classification                        │
              │  d < 0.4 m (head) or 0.5 m (body) → CRITICAL        │
              │  d < 1.5 m                         → DANGER          │
              │  d < 3.0 m                         → WARNING         │
              │  d < 5.0 m                         → CAUTION         │
              │  d < 12.0 m                        → ALERT           │
              │  d ≥ 12.0 m or no reading          → CLEAR           │
              └──────────────────────────────────────────────────────┘
                                    │
                                    ▼
              ┌──────────────────────────────────────────────────────┐
              │           Motor PWM Command                          │
              │  intensity (0–255) + frequency_hz + pattern tag      │
              └──────────────────────────────────────────────────────┘
```

---

## 11. Running the Server

```bash
# Install dependencies
pip install flask

# Start server (listens on all interfaces, port 5000)
python server.py

# Run tests
pytest tests/ -v
```

---

## 12. ESP32 Firmware Integration Notes

1. Populate **all 8** `tof_sensors` fields on every packet.  Use the sentinel
   value `{"distance_mm": 8190, "avg": 8190, ...}` for sensors that return
   no target.
2. The `module` field **must** be either `"head"` or `"body"` — hard-code it
   per device at compile time.
3. Parse the `motors` array from the response and iterate over `motor_id` 0–7
   to drive the corresponding PWM channels.  Check `active` first; if `false`,
   stop the motor immediately (intensity = 0).
4. The `frequency_hz` field controls the ON/OFF cycle rate.  Recommended
   firmware implementation:
   ```
   period_ms = 1000 / frequency_hz    // e.g. 100 ms for 10 Hz
   duty      = intensity / 255.0      // e.g. 0.6 for intensity=153
   on_time   = period_ms * duty
   off_time  = period_ms * (1 - duty)
   ```
5. Send packets at 20 Hz or faster to keep haptic latency below 50 ms.
