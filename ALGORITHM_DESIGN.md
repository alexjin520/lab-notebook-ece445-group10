# OmniSense-Dual Algorithm Design Document
**ECE 445 Group 10**

---

## 1. System Overview

OmniSense-Dual is a wearable obstacle-detection system for visually impaired
users.  It consists of **two independent modules** worn on the body:

| Module | Placement | Purpose |
|--------|-----------|----------|
| **Head** | Head / helmet | Detects head-height hazards (awnings, signs, doorframes, overhangs) |
| **Body** | Waist / torso | Detects body-level hazards (furniture, walls, people, vehicles) |

Each module runs its own ESP32-S3 microcontroller and transmits sensor packets
to this Flask server over Wi-Fi.  The server **fuses data from both modules
together** into a single combined decision and returns **one set of 8 haptic
motor commands intended for the head module only** — the head is the sole
output device because it is always worn and sits at the most exposed position.

> **Why head-only output?**  Both modules see the same physical space from
> different heights.  Separating the output would require the user to
> interpret two independent vibration streams simultaneously, which is
> cognitively overloading.  A single fused stream on the head conveys the
> worst hazard in any direction regardless of which module detected it.

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

## 7. Dual-Module Sensor Fusion

Because both modules observe the same environment from different heights,
their sensor readings are merged before any motor command is generated.

### 7.1 Per-Module Processing (Parallel)

Each module's sensors are processed independently first:

```
 Head module data        Body module data
       │                       │
  _fuse_sensor_data       _fuse_sensor_data
  (head thresholds)       (body thresholds)
       │                       │
  _apply_imu_compensation _apply_imu_compensation
  (head IMU only)         (body IMU only)
       │                       │
  head_fused[8 dirs]      body_fused[8 dirs]
```

Applying each module's IMU independently ensures that a tilted body cannot
suppress valid head readings, and a tilted head cannot mask a body hazard.

### 7.2 Cross-Module Merge (Safety-First)

For every direction the closer (more dangerous) distance from either module
wins:

```
for each direction:
    dist_head = head_fused[direction].distance_m
    dist_body = body_fused[direction].distance_m

    if both are valid:
        effective = min(dist_head, dist_body)   # safety-first
    elif only head is valid:
        effective = dist_head
    elif only body is valid:
        effective = dist_body
    else:
        effective = None  (CLEAR)

    # Always reclassify with head thresholds because
    # the output motors are mounted on the head.
    zone = classify(effective, module="head")
```

### 7.3 Why Always Head Thresholds After Merge?

| Feature                | `"head"` threshold | `"body"` threshold |
|------------------------|---------------------|--------------------|
| CRITICAL boundary      | < 0.6 m (wider)     | < 0.5 m            |
| Typical mount height   | ~175–190 cm         | ~90–110 cm         |

Once the worst distance per direction is known, the output belongs to the
head motors, so head thresholds define the final zone.  A body-detected
obstacle at 0.55 m will produce CRITICAL on the head motors even though the
body fusion alone would have produced DANGER — this is intentional and
conservative.

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

Each module sends its own packet independently.  The server merges the latest
packet from each module on every call.

**Head module packet:**
```json
{
  "device_id":  "esp32_head",
  "module":     "head",

  "tof_sensors": {
    "front":       {"distance_mm": 450, "avg": 450, "min": 440, "max": 460, "count": 50},
    "front_right": {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "right":       {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "back_right":  {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "back":        {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "back_left":   {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "left":        {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "front_left":  {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50}
  },
  "mmwave_sensors": {
    "front": {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0},
    "back":  {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0}
  },
  "imu": {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
          "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81},
  "timestamp": 37400
}
```

**Body module packet** (sent separately, any order):
```json
{
  "device_id":  "esp32_body",
  "module":     "body",

  "tof_sensors": {
    "front":       {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "front_right": {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "right":       {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "back_right":  {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "back":        {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "back_left":   {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
    "left":        {"distance_mm": 1100, "avg": 1100, "min": 1090, "max": 1110, "count": 50},
    "front_left":  {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50}
  },
  "mmwave_sensors": {
    "front": {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0},
    "back":  {"targets": 1, "range_m": 9.0, "speed_ms": 0.05, "energy": 3200}
  },
  "imu": {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
          "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81},
  "timestamp": 37393
}
```

**Field notes:**
- `module`: **Required**. `"head"` or `"body"`. Any other value is treated as `"body"`.
- `tof_sensors.*`: VL53L1X reading per direction. Use `avg` for noise reduction.
  Value `8190` indicates no target detected (sensor sentinel).
- `mmwave_sensors.{front,back}.targets`: number of detected targets. `0` = clear.
- `imu.accel_z` ≈ 9.81 m/s² at rest (gravity).
- `timestamp`: ESP32 `millis()` value (ms since boot), used for latency measurement.

### 9.2 Response  `POST /nav`

The response is **always** returned to whichever module sent the most recent
packet; `module` is always `"combined"` in the response.

Continuing the example above (body packet arrives after head packet; latest
head data has 0.45 m front obstacle; body data has 1.1 m left obstacle +
back mmWave at 9 m):

```json
{
  "status":    "ok",
  "device_id": "esp32_body",
  "module":    "combined",

  "hazard": {
    "level":  5,
    "label":  "CRITICAL",
    "alerts": [
      {"direction": "front",     "zone": "CRITICAL", "distance_m": 0.45, "source": "tof"},
      {"direction": "left",      "zone": "DANGER",   "distance_m": 1.1,  "source": "tof"},
      {"direction": "back",      "zone": "ALERT",    "distance_m": 9.0,  "source": "mmwave"},
      {"direction": "back_left", "zone": "ALERT",    "distance_m": 9.0,  "source": "mmwave"},
      {"direction": "back_right","zone": "ALERT",    "distance_m": 9.0,  "source": "mmwave"}
    ]
  },

  "motors": [
    {"motor_id": 0, "direction": "front",       "intensity": 255, "frequency_hz": 50,
     "pattern": "continuous",  "active": true,  "zone": "CRITICAL", "distance_m": 0.45, "source": "tof"},
    {"motor_id": 1, "direction": "front_right",  "intensity": 0,   "frequency_hz": 0,
     "pattern": "off",         "active": false, "zone": "CLEAR",    "distance_m": null, "source": "none"},
    {"motor_id": 2, "direction": "right",        "intensity": 0,   "frequency_hz": 0,
     "pattern": "off",         "active": false, "zone": "CLEAR",    "distance_m": null, "source": "none"},
    {"motor_id": 3, "direction": "back_right",   "intensity": 51,  "frequency_hz": 2,
     "pattern": "very_slow",   "active": true,  "zone": "ALERT",   "distance_m": 9.0,  "source": "mmwave"},
    {"motor_id": 4, "direction": "back",         "intensity": 51,  "frequency_hz": 2,
     "pattern": "very_slow",   "active": true,  "zone": "ALERT",   "distance_m": 9.0,  "source": "mmwave"},
    {"motor_id": 5, "direction": "back_left",    "intensity": 51,  "frequency_hz": 2,
     "pattern": "very_slow",   "active": true,  "zone": "ALERT",   "distance_m": 9.0,  "source": "mmwave"},
    {"motor_id": 6, "direction": "left",         "intensity": 204, "frequency_hz": 20,
     "pattern": "rapid_pulse", "active": true,  "zone": "DANGER",  "distance_m": 1.1,  "source": "tof"},
    {"motor_id": 7, "direction": "front_left",   "intensity": 0,   "frequency_hz": 0,
     "pattern": "off",         "active": false, "zone": "CLEAR",   "distance_m": null, "source": "none"}
  ],

  "imu":        {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0},
  "latency_ms": 7.0
}
```

**Field notes:**
- `module`: always `"combined"` — the output is a fusion of both modules.
- `hazard.level`: 0 (CLEAR) → 5 (CRITICAL).  Use as a quick severity summary.
- `hazard.alerts`: sorted most-critical first; contains only non-CLEAR directions.
- `motors[i].source`: `"tof"` | `"mmwave"` | `"tof+mmwave"` | `"none"` | `"suppressed(pitch_tilt)"` | `"suppressed(roll_tilt)"`.
- `motors[i].distance_m`: `null` when no obstacle detected or channel suppressed.
- `imu`: echoes the **head** module's IMU (the output device).
- `latency_ms`: server receipt time minus the latest timestamp from either module.

### 9.3  `GET /status`

```json
{
  "status": "running",
  "devices": {
    "esp32_head": {
      "module":       "head",
      "last_zone":    "CRITICAL",
      "packet_count": 142,
      "last_seen_ms": 1712678400000
    },
    "esp32_body": {
      "module":       "body",
      "last_zone":    "CRITICAL",
      "packet_count": 140,
      "last_seen_ms": 1712678400050
    }
  }
}
```

> Note: `last_zone` in `/status` reflects the combined hazard label from the
> most recent fusion result, not an individual module zone.

---

## 10. Algorithm State Machine

```
  HEAD packet            BODY packet
       │                      │
       ▼                      ▼
┌─────────────┐        ┌─────────────┐
│  Parse ToF  │        │  Parse ToF  │
│  Parse mWave│        │  Parse mWave│
│  Parse IMU  │        │  Parse IMU  │
└──────┬──────┘        └──────┬──────┘
       │                      │
       ▼                      ▼
┌─────────────┐        ┌─────────────┐
│ Hybrid Fuse │        │ Hybrid Fuse │
│ (per-dir    │        │ (per-dir    │
│  min rule)  │        │  min rule)  │
└──────┬──────┘        └──────┬──────┘
       │                      │
       ▼                      ▼
┌─────────────┐        ┌─────────────┐
│ Head IMU    │        │ Body IMU    │
│ compensation│        │ compensation│
│ (head only) │        │ (body only) │
└──────┬──────┘        └──────┬──────┘
       │                      │
       └──────────┬───────────┘
                  ▼
    ┌─────────────────────────┐
    │  Cross-Module Merge     │
    │  for each direction:    │
    │   effective =           │
    │    min(head, body)      │
    │  (safety-first)         │
    └────────────┬────────────┘
                 │
                 ▼
    ┌─────────────────────────┐
    │  Zone Classification    │
    │  (always head thresholds│
    │   — output is head-only)│
    │  d < 0.6 m → CRITICAL   │
    │  d < 1.5 m → DANGER     │
    │  d < 3.0 m → WARNING    │
    │  d < 5.0 m → CAUTION    │
    │  d < 12.0 m → ALERT     │
    │  d ≥ 12.0 m → CLEAR     │
    └────────────┬────────────┘
                 │
                 ▼
    ┌─────────────────────────┐
    │  8 Motor PWM Commands   │
    │  (sent to head only)    │
    └─────────────────────────┘
```

---

## 11. Worked Example — Combined Dual-Module Decision

This section traces a single real-world moment through the entire algorithm
so the combined logic is visible at every step.

### Scenario

The user is walking down a hallway:
- A **doorframe** is 45 cm ahead at head height  (only the head module sees it).
- A **chair** is 1.1 m to the left at waist height  (only the body module sees it).
- A **person** is approaching from behind, 9 m away  (body mmWave detects them).
- The head module is **slightly tilted** forward: `pitch = 20°` (looking down).

### Step 1 — Head Module Parses Its Own Sensors

```
Head ToF readings (after sentinel + range filtering):
  front:       0.45 m   ← doorframe at head height
  all others:  None     ← no target

Head mmWave:
  front:  None (targets = 0)
  back:   None (targets = 0)

Head IMU: pitch = +20°, roll = 0°
```

### Step 2 — Body Module Parses Its Own Sensors

```
Body ToF readings:
  left:        1.10 m   ← chair leg
  all others:  None     ← no target

Body mmWave:
  front:  None (targets = 0)
  back:   range_m = 9.0 m  (person approaching)

Body IMU: pitch = 0°, roll = 0°  (body is upright)
```

### Step 3 — Per-Module Hybrid Sensor Fusion

```
Head fused per direction  (using head thresholds internally):
  front:        dist=0.45 m, source="tof",    zone=CRITICAL
  all others:   dist=None,   source="none",   zone=CLEAR

Body fused per direction  (using body thresholds internally):
  left:         dist=1.10 m, source="tof",    zone=DANGER
  back:         dist=9.0 m,  source="mmwave", zone=ALERT
  back_left:    dist=9.0 m,  source="mmwave", zone=ALERT   ← mmWave cone
  back_right:   dist=9.0 m,  source="mmwave", zone=ALERT   ← mmWave cone
  all others:   dist=None,   source="none",   zone=CLEAR
```

### Step 4 — IMU Compensation (Applied per Module Independently)

```
Head IMU: |pitch| = 20° > 15° threshold
  → suppress head["front"]:   zone=CLEAR, source="suppressed(pitch_tilt)"
  → suppress head["back"]:    (already CLEAR, no change)

Body IMU: |pitch| = 0°, |roll| = 0°  →  no suppression
```

After compensation:
```
Head fused:     front → CLEAR (suppressed!),  all others → CLEAR
Body fused:     left → DANGER, back/back_left/back_right → ALERT
```

> The head was pointing at the ground and would have produced a false
> 45 cm reading from the floor.  Suppression removes it correctly.

### Step 5 — Cross-Module Merge (min per direction, head thresholds)

```
Direction   Head dist   Body dist   Effective   Head zone
─────────────────────────────────────────────────────────
front       None*       None        None        CLEAR        * suppressed
front_right None        None        None        CLEAR
right       None        None        None        CLEAR
back_right  None        9.0 m       9.0 m       ALERT
back        None        9.0 m       9.0 m       ALERT
back_left   None        9.0 m       9.0 m       ALERT
left        None        1.10 m      1.10 m      DANGER
front_left  None        None        None        CLEAR
```

### Step 6 — Final Motor Commands (sent to head module)

```
Motor 0  front       → OFF   (CLEAR)
Motor 1  front_right → OFF   (CLEAR)
Motor 2  right       → OFF   (CLEAR)
Motor 3  back_right  → ON    intensity=51, 2 Hz, very_slow   (ALERT)
Motor 4  back        → ON    intensity=51, 2 Hz, very_slow   (ALERT)
Motor 5  back_left   → ON    intensity=51, 2 Hz, very_slow   (ALERT)
Motor 6  left        → ON    intensity=204, 20 Hz, rapid_pulse (DANGER)
Motor 7  front_left  → OFF   (CLEAR)
```

**Hazard summary:** `level=4, label="DANGER"` (worst non-suppressed zone)

### Key Observations from This Example

| Observation | Why it works |
|---|---|
| Head CRITICAL suppressed by its own pitch tilt | IMU per-module: body's upright IMU doesn't accidentally un-suppress the head |
| Body chair (1.1 m left) reaches the head motors | Cross-module merge: body hazards propagate to head output |
| Back mmWave cone covers 3 directions | Single radar detection → back, back_left, back_right all vibrate |
| Head thresholds apply to the merged result | Final zones always use 0.6 m CRITICAL boundary, not 0.5 m body boundary |

---

## 12. Running the Server

```bash
# Install dependencies
pip install flask

# Start server (listens on all interfaces, port 5000)
python server.py

# Run tests
pytest tests/ -v
```

---

## 13. ESP32 Firmware Integration Notes

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
