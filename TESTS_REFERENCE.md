# Test Suite Reference  |  ECE 445 Group 10

**134 tests total** — 90 unit tests (`tests/test_unit.py`) + 44 integration tests (`tests/test_integration.py`)

Run all tests:
```
pytest tests/ -v
```

---

## Part 1 — Unit Tests  (`tests/test_unit.py`)

Unit tests call algorithm functions directly with no HTTP layer.

---

### Group 1 — `classify_distance`
_Verifies every zone boundary is correct for both body and head modules._

| Test | Input | Expected |
|------|-------|----------|
| `test_critical_lower_bound` | `0.0 m` | `CRITICAL` |
| `test_critical_just_below_threshold` | `0.49 m` | `CRITICAL` |
| `test_danger_at_boundary` | `0.5 m` | `DANGER` |
| `test_danger_mid` | `1.0 m` | `DANGER` |
| `test_warning_at_boundary` | `1.5 m` | `WARNING` |
| `test_warning_mid` | `2.5 m` | `WARNING` |
| `test_caution_at_boundary` | `3.0 m` | `CAUTION` |
| `test_caution_mid` | `4.0 m` | `CAUTION` |
| `test_alert_at_boundary` | `5.0 m` | `ALERT` |
| `test_alert_mid` | `8.5 m` | `ALERT` |
| `test_clear_at_mmwave_max` | `12.0 m` | `CLEAR` |
| `test_clear_beyond_max` | `50.0 m` | `CLEAR` |
| `test_head_critical_tighter_threshold` | `0.55 m, module="body"` | `DANGER`; `0.55 m, module="head"` → `CRITICAL` |
| `test_head_critical_just_below_06` | `0.59 m, module="head"` | `CRITICAL` |
| `test_head_danger_at_or_above_06` | `0.60 m, module="head"` | `DANGER` |

---

### Group 2 — `_parse_tof_sensors`
_Verifies ToF sensor dict parsing, unit conversion (mm → m), sentinel detection, and range clamping._

| Test | Input | Expected |
|------|-------|----------|
| `test_all_directions_present` | All 8 directions at 1000 mm | Dict has all 8 keys |
| `test_valid_reading_converted_to_metres` | `distance_mm=2000, avg=2000` | `2.0 m` |
| `test_prefers_avg_over_distance_mm` | `distance_mm=3000, avg=2500` | `2.5 m` (avg wins) |
| `test_sentinel_value_returns_none` | `avg=8190` (no-target sentinel) | `None` |
| `test_above_tof_max_range_clamped_to_none` | `avg=6000` (6 m > 5 m max) | `None` |
| `test_at_tof_max_range_accepted` | `avg=5000` (exactly 5 m) | `5.0 m` |
| `test_missing_direction_returns_none` | Empty dict `{}` | All 8 directions → `None` |
| `test_zero_or_negative_distance_returns_none` | `avg=0` | `None` |
| `test_partial_directions` | Only `front` present at 800 mm | `front=0.8 m`, all others `None` |

---

### Group 3 — `_parse_mmwave_sensors`
_Verifies mmWave sensor dict parsing, target filtering, and range clamping._

| Test | Input | Expected |
|------|-------|----------|
| `test_no_targets_returns_none` | `targets=0` on both sensors | Both `None` |
| `test_valid_target_parsed` | `front: targets=1, range_m=7.5, speed_ms=0.11` | `front.range_m=7.5`, `front.speed_ms=0.11` |
| `test_beyond_max_range_returns_none` | `range_m=13.0` (> 12 m max) | `None` |
| `test_at_mmwave_max_range_accepted` | `range_m=12.0` (exactly 12 m) | `range_m=12.0` |
| `test_missing_data_returns_none` | Empty dict `{}` | Both sensors `None` |
| `test_both_sensors_active` | `front: range_m=6.0`, `back: range_m=9.0` | Both parsed correctly |

---

### Group 4 — `_parse_imu`
_Verifies IMU dict parsing and default fallback values._

| Test | Input | Expected |
|------|-------|----------|
| `test_all_fields_parsed` | `roll=5.0, pitch=-3.0, yaw=90.0, accel_z=9.5` | All fields match |
| `test_defaults_on_empty` | `{}` | `roll=0, pitch=0, yaw=0, accel_z=9.81` |
| `test_partial_fields` | `pitch=10.0` only | `pitch=10.0`, `roll=0` (default) |

---

### Group 5 — `_fuse_sensor_data` (Hybrid Fusion)
_Verifies all sensor fusion scenarios including ToF-only, mmWave-only, and combined cases._

| Test | Scenario | Expected |
|------|----------|----------|
| `test_all_clear_when_no_sensors` | No readings anywhere | All 8 directions: `CLEAR`, `distance_m=None`, `source="none"` |
| `test_tof_only_close_obstacle` | `front ToF = 0.3 m` | `front: CRITICAL, source="tof"` |
| `test_tof_only_warning_range` | `right ToF = 2.0 m` | `right: WARNING, source="tof"` |
| `test_left_right_not_covered_by_mmwave` | front mmWave = 7 m, no ToF | `left=CLEAR`, `right=CLEAR` (outside cone) |
| `test_mmwave_only_extends_beyond_tof` | front mmWave = 7.5 m, ToF clear | `front/front_right/front_left: ALERT, source="mmwave"` |
| `test_back_mmwave_fills_back_cone` | back mmWave = 9.0 m, ToF clear | `back/back_right/back_left: ALERT, source="mmwave"` |
| `test_mmwave_within_tof_range_acts_as_tof_backup` | front mmWave = 3.0 m, ToF clear | `front: CAUTION, source="mmwave"` |
| `test_hybrid_tof_closer_wins` | ToF=1.0 m + mmWave=6.0 m (front) | `front: DANGER, distance_m=1.0, source="tof+mmwave"` |
| `test_hybrid_mmwave_closer_wins` | ToF=4.5 m + mmWave=2.5 m (front) | `front: WARNING, distance_m=2.5, source="tof+mmwave"` |
| `test_hybrid_equal_distances` | ToF=2.0 m + mmWave=2.0 m | `distance_m=2.0, source="tof+mmwave"` |
| `test_vibration_pattern_matches_zone` | front ToF = 0.4 m (body) | CRITICAL vibration: `intensity=255, freq=50 Hz, active=True` |
| `test_clear_vibration_inactive` | No obstacles | `active=False, intensity=0` |
| `test_head_module_tighter_critical` | front = 0.55 m | `head: CRITICAL`, `body: DANGER` |
| `test_all_8_directions_present_in_fused` | Any input | Fused dict always has all 8 direction keys |

---

### Group 6 — `_apply_imu_compensation`
_Verifies tilt suppression logic and that the input dict is never mutated._

| Test | Input | Expected |
|------|-------|----------|
| `test_no_tilt_no_change` | `pitch=5°` (below threshold) | `front: WARNING` (unchanged) |
| `test_pitch_tilt_suppresses_front` | `pitch=16°`, front obstacle at 2 m | `front: CLEAR`, `source contains "suppressed"` |
| `test_pitch_tilt_suppresses_back` | `pitch=-20°`, back obstacle at 1 m | `back: CLEAR` |
| `test_pitch_tilt_does_not_suppress_sides` | `pitch=16°`, left obstacle | `left: WARNING` (unaffected) |
| `test_roll_tilt_suppresses_left_right` | `roll=16°`, both side obstacles | `left=CLEAR`, `right=CLEAR` |
| `test_roll_tilt_does_not_suppress_front` | `roll=16°`, front obstacle | `front: WARNING` (unaffected) |
| `test_exact_threshold_not_suppressed` | `pitch=15.0°` (exactly threshold) | `front: WARNING` (strict `>`, not `>=`) |
| `test_input_fused_not_mutated` | Call with tilted IMU | Original `fused` dict zone unchanged after call |

---

### Group 7 — `_generate_motor_commands`
_Verifies the motor command list structure, indexing, and value encoding._

| Test | Input | Expected |
|------|-------|----------|
| `test_returns_8_commands` | Any fused dict | List length == 8 |
| `test_motor_ids_are_0_to_7` | Clear fused dict | `motor_id` values = `[0,1,2,3,4,5,6,7]` |
| `test_direction_matches_index` | Clear fused dict | `motors[i].direction == DIRECTIONS[i]` |
| `test_clear_motor_is_inactive` | All CLEAR | All: `active=False`, `intensity=0` |
| `test_critical_motor_max_intensity` | `front: CRITICAL` | `front: intensity=255, frequency_hz=50, pattern="continuous", active=True` |
| `test_distance_rounded_to_3_decimal_places` | `distance_m=1.23456789` | `distance_m=1.235` |
| `test_none_distance_remains_none` | All CLEAR | All `distance_m=null` |

---

### Group 8 — `_compute_hazard`
_Verifies the hazard summary: overall level, label, and sorted alert list._

| Test | Input | Expected |
|------|-------|----------|
| `test_all_clear_level_zero` | All CLEAR | `level=0, label="CLEAR", alerts=[]` |
| `test_single_critical_sets_level_5` | `front: CRITICAL` | `level=5, label="CRITICAL", 1 alert` |
| `test_worst_zone_wins` | `front: WARNING` + `right: CRITICAL` | `level=5, label="CRITICAL"` |
| `test_alerts_sorted_most_critical_first` | `front: CAUTION`, `right: CRITICAL`, `back: WARNING` | Order: `CRITICAL → WARNING → CAUTION` |
| `test_alert_count_excludes_clear_directions` | 2 DANGER + 6 CLEAR | `len(alerts)==2` |

---

### Group 9 — `process_packet` (end-to-end)
_Calls the full algorithm pipeline including parsing, fusion, IMU compensation, and output generation._

| Test | Input | Expected |
|------|-------|----------|
| `test_returns_required_keys` | Valid full packet | Keys: `module, hazard, motors, imu, latency_ms` |
| `test_8_motors_returned` | Any packet | `len(motors)==8` |
| `test_all_clear_packet_level_zero` | All sensors clear | `hazard.level==0` |
| `test_head_module_identified` | `module="head"` | Result `module=="head"` |
| `test_body_module_identified` | `module="body"` | Result `module=="body"` |
| `test_invalid_module_defaults_to_body` | `module="torso"` | Result `module=="body"` |
| `test_critical_front_obstacle` | `front ToF=300 mm` | `hazard="CRITICAL"`, front motor: `intensity=255, active=True` |
| `test_mmwave_alert_beyond_tof` | front mmWave = 8 m | `front/front_right/front_left: ALERT` |
| `test_tof_takes_priority_over_mmwave` | front `ToF=0.5 m`, front `mmWave=8 m` | `zone=DANGER, distance_m=0.5, source="tof+mmwave"` |
| `test_imu_tilt_suppresses_channels` | `front ToF=0.8 m`, `pitch=20°` | front motor: `zone=CLEAR, active=False` |
| `test_state_updated_after_packet` | 2 packets sent | `packet_count` increments 0 → 1 → 2 |
| `test_state_module_recorded` | `module="head"` packet | State `module=="head"` |
| `test_latency_calculation` | `timestamp=1000`, server time=`1100 ms` | `latency_ms≈100` |

---

### Group 10 — `fresh_device_state`
_Verifies initial per-device state shape and independence._

| Test | Expected |
|------|----------|
| `test_required_keys_present` | Keys: `last_seen_ms, packet_count, last_zone, module` |
| `test_initial_packet_count_zero` | `packet_count==0` |
| `test_initial_zone_clear` | `last_zone=="CLEAR"` |
| `test_two_instances_are_independent` | Mutating one state does not affect a second |

---

---

## Part 2 — Integration Tests  (`tests/test_integration.py`)

Integration tests exercise the full Flask pipeline: ESP32 sends an HTTP POST, the server runs the algorithm, and the response JSON is inspected. Every example below shows the actual JSON payload sent and the full expected response shape.

---

### Group 1 — `POST /nav` Schema Validation

These tests verify that the server envelope is always correct regardless of sensor content.

---

#### `test_returns_200_on_valid_packet`

**Module sends:**
```json
{
  "device_id": "esp32_body",
  "module": "body",
  "tof_sensors": {
    "front":       {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50},
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
  "timestamp": 1000
}
```

**Server returns:** HTTP `200 OK`

---

#### `test_each_motor_has_required_fields`

**Module sends:** _(same all-clear body packet as above)_

**Server returns (motors array structure — all 8 required):**
```json
{
  "status": "ok",
  "device_id": "esp32_body",
  "module": "body",
  "hazard": {"level": 0, "label": "CLEAR", "alerts": []},
  "motors": [
    {"motor_id": 0, "direction": "front",       "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    {"motor_id": 1, "direction": "front_right",  "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    {"motor_id": 2, "direction": "right",        "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    {"motor_id": 3, "direction": "back_right",   "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    {"motor_id": 4, "direction": "back",         "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    {"motor_id": 5, "direction": "back_left",    "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    {"motor_id": 6, "direction": "left",         "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    {"motor_id": 7, "direction": "front_left",   "intensity": 0, "frequency_hz": 0, "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"}
  ],
  "imu": {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0},
  "latency_ms": 0.0
}
```

---

#### `test_missing_json_returns_400`

**Module sends:** raw text `"not json"` with `Content-Type: text/plain`

**Server returns:** HTTP `400 Bad Request`
```json
{"status": "error", "message": "invalid or missing JSON"}
```

---

#### `test_empty_json_object_is_accepted`

**Module sends:** `{}`

**Server returns:** HTTP `200 OK` with all motors inactive and `hazard.level=0`

---

### Group 2 — Head Module Scenarios

---

#### `test_head_critical_obstacle_front`

**Head module sends** (obstacle 35 cm directly in front):
```json
{
  "device_id": "esp32_head",
  "module": "head",
  "tof_sensors": {
    "front":       {"distance_mm": 350, "avg": 350, "min": 340, "max": 360, "count": 50},
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
  "timestamp": 1000
}
```

**Server returns:**
```json
{
  "status": "ok",
  "device_id": "esp32_head",
  "module": "head",
  "hazard": {
    "level": 5,
    "label": "CRITICAL",
    "alerts": [
      {"direction": "front", "zone": "CRITICAL", "distance_m": 0.35, "source": "tof"}
    ]
  },
  "motors": [
    {"motor_id": 0, "direction": "front", "intensity": 255, "frequency_hz": 50,
     "pattern": "continuous", "active": true, "zone": "CRITICAL",
     "distance_m": 0.35, "source": "tof"},
    {"motor_id": 1, "direction": "front_right", "intensity": 0, "frequency_hz": 0,
     "pattern": "off", "active": false, "zone": "CLEAR", "distance_m": null, "source": "none"},
    ...
  ],
  "imu": {"roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0},
  "latency_ms": 0.0
}
```
**Verified:** `hazard.label=="CRITICAL"`, `motors[0].intensity==255`, `motors[0].active==true`

---

#### `test_head_tighter_critical_threshold`

**Head module sends** (obstacle at 55 cm — between body CRITICAL (50 cm) and head CRITICAL (60 cm) thresholds):
```json
{
  "device_id": "esp32_head", "module": "head",
  "tof_sensors": {
    "front": {"distance_mm": 550, "avg": 550, "min": 540, "max": 560, "count": 50},
    ... (all others: 8190)
  }, ...
}
```

**Server returns for head module:**
```json
"motors": [
  {"motor_id": 0, "direction": "front", "zone": "CRITICAL", "intensity": 255,
   "frequency_hz": 50, "pattern": "continuous", "active": true, "distance_m": 0.55}
]
```

**Body module sends** (same 55 cm reading, `"module": "body"`):

**Server returns for body module:**
```json
"motors": [
  {"motor_id": 0, "direction": "front", "zone": "DANGER", "intensity": 204,
   "frequency_hz": 20, "pattern": "rapid_pulse", "active": true, "distance_m": 0.55}
]
```

---

#### `test_head_imu_echoed`

**Head module sends** IMU data with non-zero orientation:
```json
{
  "imu": {"roll_deg": 3.5, "pitch_deg": -2.1, "yaw_deg": 90.0,
          "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81}, ...
}
```

**Server returns:**
```json
"imu": {"roll_deg": 3.5, "pitch_deg": -2.1, "yaw_deg": 90.0}
```

---

### Group 3 — Body Module Scenarios

---

#### `test_body_danger_left`

**Body module sends** (obstacle 90 cm to the left):
```json
{
  "device_id": "esp32_body", "module": "body",
  "tof_sensors": {
    "left": {"distance_mm": 900, "avg": 900, "min": 880, "max": 920, "count": 50},
    ... (all others: 8190)
  }, ...
}
```

**Server returns:**
```json
{
  "hazard": {"level": 4, "label": "DANGER",
             "alerts": [{"direction": "left", "zone": "DANGER", "distance_m": 0.9, "source": "tof"}]},
  "motors": [
    ...
    {"motor_id": 6, "direction": "left", "intensity": 204, "frequency_hz": 20,
     "pattern": "rapid_pulse", "active": true, "zone": "DANGER", "distance_m": 0.9}
    ...
  ]
}
```

---

#### `test_body_multiple_obstacles`

**Body module sends** (obstacle 20 cm front + 120 cm back-right):
```json
{
  "device_id": "esp32_body", "module": "body",
  "tof_sensors": {
    "front":      {"distance_mm": 200,  "avg": 200,  "min": 190,  "max": 210,  "count": 50},
    "back_right": {"distance_mm": 1200, "avg": 1200, "min": 1190, "max": 1210, "count": 50},
    ... (all others: 8190)
  }, ...
}
```

**Server returns:**
```json
{
  "hazard": {"level": 5, "label": "CRITICAL",
             "alerts": [
               {"direction": "front",      "zone": "CRITICAL", "distance_m": 0.2,  "source": "tof"},
               {"direction": "back_right", "zone": "DANGER",   "distance_m": 1.2,  "source": "tof"}
             ]},
  "motors": [
    {"motor_id": 0, "direction": "front",      "zone": "CRITICAL", "intensity": 255, "active": true, ...},
    {"motor_id": 3, "direction": "back_right",  "zone": "DANGER",   "intensity": 204, "active": true, ...},
    ... (remaining 6 motors: inactive)
  ]
}
```
**Verified:** `hazard.label=="CRITICAL"`, at least 2 motors active.

---

### Group 4 — Hybrid Sensor Fusion over HTTP

---

#### `test_mmwave_only_detection_beyond_tof`

**Body module sends** (no ToF hits, front mmWave detects at 8 m):
```json
{
  "device_id": "esp32_body", "module": "body",
  "tof_sensors": { ... all 8190 ... },
  "mmwave_sensors": {
    "front": {"targets": 1, "range_m": 8.0, "speed_ms": 0.1, "energy": 7000},
    "back":  {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0}
  }, ...
}
```

**Server returns** (front hemisphere — 3 directions — all ALERT):
```json
{
  "hazard": {"level": 1, "label": "ALERT",
             "alerts": [
               {"direction": "front",       "zone": "ALERT", "distance_m": 8.0, "source": "mmwave"},
               {"direction": "front_right", "zone": "ALERT", "distance_m": 8.0, "source": "mmwave"},
               {"direction": "front_left",  "zone": "ALERT", "distance_m": 8.0, "source": "mmwave"}
             ]},
  "motors": [
    {"motor_id": 0, "direction": "front",       "zone": "ALERT", "intensity": 51, "frequency_hz": 2,
     "pattern": "very_slow", "active": true,  "distance_m": 8.0, "source": "mmwave"},
    {"motor_id": 1, "direction": "front_right", "zone": "ALERT", "intensity": 51, "frequency_hz": 2,
     "pattern": "very_slow", "active": true,  "distance_m": 8.0, "source": "mmwave"},
    {"motor_id": 2, "direction": "right",        "zone": "CLEAR", "intensity": 0,  "active": false, ...},
    {"motor_id": 3, "direction": "back_right",   "zone": "CLEAR", "intensity": 0,  "active": false, ...},
    {"motor_id": 4, "direction": "back",         "zone": "CLEAR", "intensity": 0,  "active": false, ...},
    {"motor_id": 5, "direction": "back_left",    "zone": "CLEAR", "intensity": 0,  "active": false, ...},
    {"motor_id": 6, "direction": "left",         "zone": "CLEAR", "intensity": 0,  "active": false, ...},
    {"motor_id": 7, "direction": "front_left",  "zone": "ALERT", "intensity": 51, "frequency_hz": 2,
     "pattern": "very_slow", "active": true,  "distance_m": 8.0, "source": "mmwave"}
  ]
}
```

---

#### `test_tof_and_mmwave_tof_closer`

**Body module sends** (ToF=1 m and mmWave=7 m both see front — safety-first: min wins):
```json
{
  "tof_sensors": {
    "front": {"distance_mm": 1000, "avg": 1000, "min": 990, "max": 1010, "count": 50},
    ... (all others: 8190)
  },
  "mmwave_sensors": {
    "front": {"targets": 1, "range_m": 7.0, "speed_ms": 0.05, "energy": 5000},
    "back":  {"targets": 0, "range_m": 0.0, "speed_ms": 0.0,  "energy": 0}
  }, ...
}
```

**Server returns** (ToF distance wins):
```json
{
  "hazard": {"level": 4, "label": "DANGER", ...},
  "motors": [
    {"motor_id": 0, "direction": "front", "zone": "DANGER",
     "intensity": 204, "frequency_hz": 20, "pattern": "rapid_pulse",
     "active": true, "distance_m": 1.0, "source": "tof+mmwave"},
    ...
  ]
}
```

---

#### `test_tof_and_mmwave_mmwave_closer`

**Body module sends** (ToF=4 m and mmWave=2 m — mmWave sees a closer hazard):
```json
{
  "tof_sensors": {
    "front": {"distance_mm": 4000, "avg": 4000, "min": 3990, "max": 4010, "count": 50},
    ...
  },
  "mmwave_sensors": {
    "front": {"targets": 1, "range_m": 2.0, "speed_ms": 0.1, "energy": 6000},
    "back":  {"targets": 0, ...}
  }, ...
}
```

**Server returns** (mmWave distance wins):
```json
{
  "hazard": {"level": 3, "label": "WARNING", ...},
  "motors": [
    {"motor_id": 0, "direction": "front", "zone": "WARNING",
     "intensity": 153, "frequency_hz": 10, "pattern": "medium_pulse",
     "active": true, "distance_m": 2.0, "source": "tof+mmwave"},
    ...
  ]
}
```

---

#### `test_side_directions_no_mmwave_coverage`

**Body module sends** (front mmWave active, no ToF):
```json
{
  "tof_sensors": { ... all 8190 ... },
  "mmwave_sensors": {
    "front": {"targets": 1, "range_m": 6.0, "speed_ms": 0.0, "energy": 3000},
    "back":  {"targets": 0, ...}
  }, ...
}
```

**Server returns** (`left` and `right` always CLEAR — outside mmWave cone):
```json
{
  "motors": [
    ...
    {"motor_id": 2, "direction": "right", "zone": "CLEAR", "active": false, ...},
    ...
    {"motor_id": 6, "direction": "left",  "zone": "CLEAR", "active": false, ...},
    ...
  ]
}
```

---

#### `test_both_mmwave_sensors_active`

**Body module sends** (both front=7 m and back=9 m mmWave detections):
```json
{
  "tof_sensors": { ... all 8190 ... },
  "mmwave_sensors": {
    "front": {"targets": 1, "range_m": 7.0, "speed_ms": 0.1, "energy": 5000},
    "back":  {"targets": 1, "range_m": 9.0, "speed_ms": 0.0, "energy": 3000}
  }, ...
}
```

**Server returns** (6 of 8 motors active — front cone + back cone):
```json
{
  "motors": [
    {"motor_id": 0, "direction": "front",       "zone": "ALERT", "active": true, "distance_m": 7.0, "source": "mmwave"},
    {"motor_id": 1, "direction": "front_right",  "zone": "ALERT", "active": true, "distance_m": 7.0, "source": "mmwave"},
    {"motor_id": 2, "direction": "right",        "zone": "CLEAR", "active": false, ...},
    {"motor_id": 3, "direction": "back_right",   "zone": "ALERT", "active": true, "distance_m": 9.0, "source": "mmwave"},
    {"motor_id": 4, "direction": "back",         "zone": "ALERT", "active": true, "distance_m": 9.0, "source": "mmwave"},
    {"motor_id": 5, "direction": "back_left",    "zone": "ALERT", "active": true, "distance_m": 9.0, "source": "mmwave"},
    {"motor_id": 6, "direction": "left",         "zone": "CLEAR", "active": false, ...},
    {"motor_id": 7, "direction": "front_left",   "zone": "ALERT", "active": true, "distance_m": 7.0, "source": "mmwave"}
  ]
}
```

---

### Group 5 — IMU Compensation over HTTP

---

#### `test_pitch_tilt_suppresses_front_reading`

**Body module sends** (obstacle at 60 cm front BUT pitch tilted 25° — module points at ground):
```json
{
  "tof_sensors": {
    "front": {"distance_mm": 600, "avg": 600, "min": 590, "max": 610, "count": 50},
    ... (all others: 8190)
  },
  "imu": {"roll_deg": 0.0, "pitch_deg": 25.0, "yaw_deg": 0.0,
          "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81},
  ...
}
```

**Server returns** (front motor suppressed despite close reading):
```json
{
  "hazard": {"level": 0, "label": "CLEAR", "alerts": []},
  "motors": [
    {"motor_id": 0, "direction": "front", "zone": "CLEAR",
     "intensity": 0, "active": false, "distance_m": null,
     "source": "suppressed(pitch_tilt)"},
    ... (all others also inactive)
  ]
}
```

---

#### `test_roll_tilt_suppresses_left_right`

**Body module sends** (obstacles at 70 cm left and 80 cm right, BUT roll tilted 23°):
```json
{
  "tof_sensors": {
    "left":  {"distance_mm": 700, "avg": 700, "min": 690, "max": 710, "count": 50},
    "right": {"distance_mm": 800, "avg": 800, "min": 790, "max": 810, "count": 50},
    ... (all others: 8190)
  },
  "imu": {"roll_deg": 23.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
          "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81},
  ...
}
```

**Server returns** (left and right motors suppressed):
```json
{
  "hazard": {"level": 0, "label": "CLEAR", "alerts": []},
  "motors": [
    ...
    {"motor_id": 2, "direction": "right", "zone": "CLEAR", "active": false,
     "source": "suppressed(roll_tilt)"},
    ...
    {"motor_id": 6, "direction": "left",  "zone": "CLEAR", "active": false,
     "source": "suppressed(roll_tilt)"},
    ...
  ]
}
```

---

#### `test_diagonal_directions_not_suppressed_by_pitch`

**Body module sends** (obstacle at `front_right` 1 m, pitch tilted 25°):
```json
{
  "tof_sensors": {
    "front_right": {"distance_mm": 1000, "avg": 1000, "min": 990, "max": 1010, "count": 50},
    ... (all others: 8190)
  },
  "imu": {"roll_deg": 0.0, "pitch_deg": 25.0, ...},
  ...
}
```

**Server returns** (`front_right` still active — pitch only suppresses pure front/back):
```json
{
  "motors": [
    {"motor_id": 0, "direction": "front",       "zone": "CLEAR", "active": false,
     "source": "suppressed(pitch_tilt)"},
    {"motor_id": 1, "direction": "front_right",  "zone": "DANGER", "active": true,
     "intensity": 204, "distance_m": 1.0, "source": "tof"},
    ...
  ]
}
```

---

### Group 6 — Edge Cases & Error Handling

---

#### `test_two_separate_devices_independent_state`

**Head module sends** CRITICAL obstacle, then body module sends all-clear:

- HEAD packet: `front ToF = 200 mm` → server registers CRITICAL for `esp32_head`
- BODY packet: all clear → server processes `esp32_body` independently

**Server returns for body packet:**
```json
{"hazard": {"level": 0, "label": "CLEAR", "alerts": []}, ...}
```
Each device has its own state; one does not affect the other.

---

#### `test_full_8_direction_obstacle_all_motors_active`

**Body module sends** (obstacles at 2 m in all 8 directions):
```json
{
  "tof_sensors": {
    "front":       {"distance_mm": 2000, "avg": 2000, ...},
    "front_right": {"distance_mm": 2000, "avg": 2000, ...},
    "right":       {"distance_mm": 2000, "avg": 2000, ...},
    "back_right":  {"distance_mm": 2000, "avg": 2000, ...},
    "back":        {"distance_mm": 2000, "avg": 2000, ...},
    "back_left":   {"distance_mm": 2000, "avg": 2000, ...},
    "left":        {"distance_mm": 2000, "avg": 2000, ...},
    "front_left":  {"distance_mm": 2000, "avg": 2000, ...}
  }, ...
}
```

**Server returns** (all 8 motors active at WARNING):
```json
{
  "hazard": {"level": 3, "label": "WARNING", "alerts": [ ... 8 alerts ... ]},
  "motors": [
    {"motor_id": 0, "direction": "front",       "zone": "WARNING", "intensity": 153, "active": true},
    {"motor_id": 1, "direction": "front_right",  "zone": "WARNING", "intensity": 153, "active": true},
    {"motor_id": 2, "direction": "right",        "zone": "WARNING", "intensity": 153, "active": true},
    {"motor_id": 3, "direction": "back_right",   "zone": "WARNING", "intensity": 153, "active": true},
    {"motor_id": 4, "direction": "back",         "zone": "WARNING", "intensity": 153, "active": true},
    {"motor_id": 5, "direction": "back_left",    "zone": "WARNING", "intensity": 153, "active": true},
    {"motor_id": 6, "direction": "left",         "zone": "WARNING", "intensity": 153, "active": true},
    {"motor_id": 7, "direction": "front_left",   "zone": "WARNING", "intensity": 153, "active": true}
  ]
}
```

---

### Group 7 — `GET /status`

---

#### `test_device_state_shows_module_and_zone`

**Head module sends** one packet (`front ToF = 400 mm` → DANGER for head, since 400 mm < 600 mm head CRITICAL threshold... actually 400 mm < 600 mm so CRITICAL).

**Then `GET /status` returns:**
```json
{
  "status": "running",
  "devices": {
    "esp32_head": {
      "module":       "head",
      "last_zone":    "CRITICAL",
      "packet_count": 1,
      "last_seen_ms": 1712678400000
    }
  }
}
```

---

#### `test_multiple_devices_tracked`

After sending one packet from each module:

**`GET /status` returns:**
```json
{
  "status": "running",
  "devices": {
    "esp32_head": {"module": "head", "last_zone": "CLEAR", "packet_count": 1, ...},
    "esp32_body": {"module": "body", "last_zone": "CLEAR", "packet_count": 1, ...}
  }
}
```
