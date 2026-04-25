"""
OmniSense-Dual  Hybrid Navigation Algorithm  |  ECE 445 Group 10
=================================================================

Hardware per module (head OR body)
-----------------------------------
  • 8 × VL53L1X ToF sensors  (max range ≤ 5 m)   – one per compass direction
  • 2 × TI mmWave radars      (max range ≤ 12 m)  – front hemisphere / back hemisphere
  • 1 × IMU (MPU-6050)                             – tilt / orientation compensation
  • 8 × Vibration motors                           – one per compass direction (1:1 with ToF)

8 Compass Directions (clockwise, index 0-7)
-------------------------------------------
  0 = front      (  0°)   Motor-0
  1 = front_right( 45°)   Motor-1
  2 = right      ( 90°)   Motor-2
  3 = back_right (135°)   Motor-3
  4 = back       (180°)   Motor-4
  5 = back_left  (225°)   Motor-5
  6 = left       (270°)   Motor-6
  7 = front_left (315°)   Motor-7

mmWave Coverage Cones (±67.5° field of view each)
--------------------------------------------------
  front  mmWave → covers front_left, front, front_right
  back   mmWave → covers back_left,  back,  back_right
  (left / right sides are covered only by ToF sensors)

Hybrid Sensor Fusion Strategy
------------------------------
For every direction the algorithm selects the best available distance
using a safety-first (minimum-distance) approach:

  Zone A  0 – 5 m    ToF is the primary, high-accuracy source.
  Zone B  5 – 12 m   mmWave is the only available source; the detected
                     range is applied to every direction inside its cone
                     where ToF reports clear, giving a hemisphere-level
                     warning that sharpens to a single direction once the
                     user moves closer and ToF takes over.
  Overlap             If both sensors have a reading for the same direction,
                     the minimum (closest) is used – we never suppress a
                     hazard because one sensor disagrees.

Vibration Pattern Encoding
--------------------------
  Zone      Distance        Intensity  Freq     Pattern
  CRITICAL  0.0 – 0.5 m     255       50 Hz    continuous
  DANGER    0.5 – 1.5 m     204       20 Hz    rapid_pulse
  WARNING   1.5 – 3.0 m     153       10 Hz    medium_pulse
  CAUTION   3.0 – 5.0 m     102        5 Hz    slow_pulse
  ALERT     5.0 – 12.0 m     51        2 Hz    very_slow
  CLEAR     > 12.0 m           0        0 Hz    off

IMU Tilt Compensation
---------------------
When the module tilts beyond TILT_THRESHOLD_DEG the physical sensor
on the corresponding axis points toward the floor or ceiling rather
than the horizontal plane, producing spurious short-range readings.
Those directions are suppressed (set to CLEAR) to avoid false alarms.
  |pitch| > threshold → suppress front & back channels
  |roll|  > threshold → suppress left & right channels

Module-specific Sensitivity
----------------------------
  head  module: critical threshold widened to 0.6 m  (head-height hazards
                need an earlier warning – awnings, signs, doorframes are
                less visible at head height)
  body  module: standard thresholds

Combined Dual-module Decision
------------------------------
  In the dual-module deployment both head and body packets are fused:
  1. Each module's IMU suppresses its own tilted sensor channels first.
  2. For every direction the closer distance from either module is used
     (safety-first minimum across head and body readings).
  3. Head thresholds classify the merged result because the output motors
     are physically mounted on the head module only.
"""

from __future__ import annotations

import logging
import time
from typing import Any

# Library-style: attach NullHandler so callers control output
logging.getLogger("omnisense").addHandler(logging.NullHandler())
logger = logging.getLogger("omnisense.algorithm")


# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

TOF_MAX_RANGE_M   = 5.0     # metres  – VL53L1X useful range
MMWAVE_MAX_RANGE_M = 12.0   # metres  – TI mmWave useful range
TOF_INVALID_MM    = 8190    # mm      – VL53L1X "no target" sentinel
TILT_THRESHOLD_DEG = 15.0   # degrees – tilt beyond which a channel is suppressed

# 8 compass directions in clockwise order
DIRECTIONS: list[str] = [
    "front",        # 0 –   0°
    "front_right",  # 1 –  45°
    "right",        # 2 –  90°
    "back_right",   # 3 – 135°
    "back",         # 4 – 180°
    "back_left",    # 5 – 225°
    "left",         # 6 – 270°
    "front_left",   # 7 – 315°
]
DIR_INDEX: dict[str, int] = {d: i for i, d in enumerate(DIRECTIONS)}
NUM_DIRECTIONS = len(DIRECTIONS)  # always 8

# mmWave coverage: each radar covers 3 adjacent directions (≈ ±67.5° cone)
MMWAVE_COVERAGE: dict[str, list[str]] = {
    "front": ["front_left", "front", "front_right"],
    "back":  ["back_left",  "back",  "back_right"],
}

# Reverse map: direction → which mmWave sensor covers it (None = sides only)
_DIR_TO_MMWAVE: dict[str, str | None] = {d: None for d in DIRECTIONS}
for _sensor_id, _covered in MMWAVE_COVERAGE.items():
    for _d in _covered:
        _DIR_TO_MMWAVE[_d] = _sensor_id


# ---------------------------------------------------------------------------
# Hazard zone definitions
# ---------------------------------------------------------------------------

class Zone:
    CRITICAL = "CRITICAL"
    DANGER   = "DANGER"
    WARNING  = "WARNING"
    CAUTION  = "CAUTION"
    ALERT    = "ALERT"
    CLEAR    = "CLEAR"


# (upper_bound_m, zone_label)  – checked in order; first match wins
_DEFAULT_THRESHOLDS: list[tuple[float, str]] = [
    (0.50,  Zone.CRITICAL),
    (1.50,  Zone.DANGER),
    (3.00,  Zone.WARNING),
    (5.00,  Zone.CAUTION),
    (12.00, Zone.ALERT),
]

# Head module uses a wider critical threshold (more sensitive):
# head-height hazards (overhangs, signs) are less visible and require
# a longer reaction window, so CRITICAL fires earlier at 0.6 m.
_HEAD_THRESHOLDS: list[tuple[float, str]] = [
    (0.60,  Zone.CRITICAL),   # wider: 0.6 m instead of 0.5 m
    (1.50,  Zone.DANGER),
    (3.00,  Zone.WARNING),
    (5.00,  Zone.CAUTION),
    (12.00, Zone.ALERT),
]

VIBRATION_PATTERNS: dict[str, dict] = {
    Zone.CRITICAL: {"intensity": 255, "frequency_hz": 50, "pattern": "continuous",   "active": True},
    Zone.DANGER:   {"intensity": 204, "frequency_hz": 20, "pattern": "rapid_pulse",  "active": True},
    Zone.WARNING:  {"intensity": 153, "frequency_hz": 10, "pattern": "medium_pulse", "active": True},
    Zone.CAUTION:  {"intensity": 102, "frequency_hz":  5, "pattern": "slow_pulse",   "active": True},
    Zone.ALERT:    {"intensity":  51, "frequency_hz":  2, "pattern": "very_slow",    "active": True},
    Zone.CLEAR:    {"intensity":   0, "frequency_hz":  0, "pattern": "off",          "active": False},
}

HAZARD_LEVEL: dict[str, int] = {
    Zone.CRITICAL: 5,
    Zone.DANGER:   4,
    Zone.WARNING:  3,
    Zone.CAUTION:  2,
    Zone.ALERT:    1,
    Zone.CLEAR:    0,
}


def classify_distance(distance_m: float, module: str = "body") -> str:
    """Return the hazard zone string for a given distance (metres)."""
    thresholds = _HEAD_THRESHOLDS if module == "head" else _DEFAULT_THRESHOLDS
    for upper, zone in thresholds:
        if distance_m < upper:
            return zone
    return Zone.CLEAR


# ---------------------------------------------------------------------------
# Sensor parsing helpers
# ---------------------------------------------------------------------------

def _parse_tof_sensors(tof_data: dict) -> dict[str, float | None]:
    """
    Parse the ``tof_sensors`` sub-object from the ESP32 packet.

    Expected shape::

        "tof_sensors": {
            "front":       {"distance_mm": 1200, "min": 1100, "avg": 1200, "max": 1300, "count": 50},
            "front_right": { ... },
            "right":       { ... },
            "back_right":  { ... },
            "back":        { ... },
            "back_left":   { ... },
            "left":        { ... },
            "front_left":  { ... }
        }

    Returns a dict mapping each direction → distance in metres (or ``None``
    when no valid target is reported).
    """
    result: dict[str, float | None] = {}
    for direction in DIRECTIONS:
        reading = tof_data.get(direction)
        if reading is None:
            result[direction] = None
            continue
        # Use distance_mm (most-recent reading) for accuracy; fall back to avg
        raw_mm = reading.get("distance_mm") or reading.get("avg")
        if raw_mm is None or raw_mm >= TOF_INVALID_MM or raw_mm <= 0:
            result[direction] = None
        else:
            dist_m = raw_mm / 1000.0
            result[direction] = dist_m if dist_m <= TOF_MAX_RANGE_M else None

    logger.debug(
        "[VERIFY:TOF_RAW] %s",
        "  ".join(
            f"{d}={v:.3f}m" if v is not None else f"{d}=None"
            for d, v in result.items()
        ),
    )
    return result


def _parse_mmwave_sensors(mmwave_data: dict) -> dict[str, dict | None]:
    """
    Parse the ``mmwave_sensors`` sub-object from the ESP32 packet.

    Firmware shape (esp32_power / esp32_head)::

        "mmwave_sensors": {
            "front": {
                "presence":  true,       # bool – target detected
                "det_state": 2,          # internal radar state int
                "dist_cm":   312,        # distance in cm (only present when presence=true)
                "energy":    10576,      # signal energy  (only present when presence=true)
                "frames":    4           # frame count    (only present when presence=true)
            },
            "back": { "presence": false, "det_state": 0 }
        }

    Returns a dict mapping sensor id ('front' / 'back') → detection dict
    (or ``None`` when no target is present or reading is out of range).
    """
    result: dict[str, dict | None] = {}
    for sensor_id in ("front", "back"):
        reading = mmwave_data.get(sensor_id)
        if reading is None or not reading.get("presence", False):
            result[sensor_id] = None
            continue
        dist_cm = reading.get("dist_cm")
        if dist_cm is None:
            result[sensor_id] = None
            continue
        range_m = float(dist_cm) / 100.0
        if range_m <= 0.0 or range_m > MMWAVE_MAX_RANGE_M:
            result[sensor_id] = None
        else:
            result[sensor_id] = {
                "range_m":  range_m,
                "speed_ms": 0.0,                          # firmware does not send speed
                "energy":   int(reading.get("energy", 0)),
            }

    for sid in ("front", "back"):
        v = result[sid]
        if v is not None:
            logger.debug(
                "[VERIFY:MMWAVE_RAW] sensor=%-5s target=yes range_m=%.3f energy=%d",
                sid, v["range_m"], v["energy"],
            )
        else:
            logger.debug("[VERIFY:MMWAVE_RAW] sensor=%-5s target=no", sid)
    return result


def _parse_imu(imu_data: dict) -> dict:
    """
    Parse the ``imu`` sub-object.

    Expected shape::

        "imu": {
            "roll_deg":  2.5,
            "pitch_deg": -1.2,
            "yaw_deg":   45.0,
            "accel_x": 0.01, "accel_y": 0.02, "accel_z": 9.8
        }
    """
    return {
        "roll_deg":  float(imu_data.get("roll_deg",  0.0)),
        "pitch_deg": float(imu_data.get("pitch_deg", 0.0)),
        "yaw_deg":   float(imu_data.get("yaw_deg",   0.0)),
        "accel_x":   float(imu_data.get("accel_x",   0.0)),
        "accel_y":   float(imu_data.get("accel_y",   0.0)),
        "accel_z":   float(imu_data.get("accel_z",   9.81)),
    }


# ---------------------------------------------------------------------------
# Hybrid sensor fusion
# ---------------------------------------------------------------------------

def _fuse_sensor_data(
    tof_distances: dict[str, float | None],
    mmwave_detections: dict[str, dict | None],
    module: str = "body",
) -> dict[str, dict]:
    """
    Fuse per-direction ToF readings with mmWave hemisphere detections.

    Fusion rules (safety-first):
    1. **ToF primary** – if ToF has a valid reading (≤ 5 m), it is used.
    2. **mmWave fill**  – if the direction falls inside a mmWave coverage
       cone AND the radar reported a target, the mmWave range is also
       considered:
         - If ToF *also* has a reading → take ``min(tof, mmwave)`` (closest
           obstacle wins – e.g. mmWave may see a far wall while ToF sees a
           nearer person, or vice-versa).
         - If ToF is clear → use mmWave range directly (extends detection
           to 12 m for the front/back hemispheres).
    3. **Neither**      – direction is ``CLEAR``.

    Returns
    -------
    dict mapping direction → ``{distance_m, source, zone, vibration}``
    """
    fused: dict[str, dict] = {}
    for direction in DIRECTIONS:
        tof_dist = tof_distances.get(direction)                     # float | None
        mw_id    = _DIR_TO_MMWAVE.get(direction)                    # str   | None
        mw_det   = mmwave_detections.get(mw_id) if mw_id else None  # dict  | None

        if tof_dist is not None and mw_det is not None:
            # Both sensors have a reading – use whichever is closer
            eff_dist = min(tof_dist, mw_det["range_m"])
            source   = "tof+mmwave"
        elif tof_dist is not None:
            eff_dist = tof_dist
            source   = "tof"
        elif mw_det is not None:
            # mmWave extends coverage beyond ToF range (or ToF missed target)
            eff_dist = mw_det["range_m"]
            source   = "mmwave"
        else:
            eff_dist = None
            source   = "none"

        zone    = classify_distance(eff_dist, module) if eff_dist is not None else Zone.CLEAR
        pattern = VIBRATION_PATTERNS[zone].copy()

        fused[direction] = {
            "distance_m": eff_dist,
            "source":     source,
            "zone":       zone,
            "vibration":  pattern,
        }

    return fused


# ---------------------------------------------------------------------------
# IMU tilt compensation
# ---------------------------------------------------------------------------

def _apply_imu_compensation(
    fused: dict[str, dict],
    imu: dict,
) -> dict[str, dict]:
    """
    Suppress sensor channels where module tilt causes the physical sensor
    to point toward the floor or ceiling instead of the horizon.

    Rules
    -----
    * |pitch| > ``TILT_THRESHOLD_DEG``  →  front & back channels suppressed
    * |roll|  > ``TILT_THRESHOLD_DEG``  →  left  & right channels suppressed

    Suppressed channels are set to ``CLEAR`` to avoid spurious ground/sky
    reflections triggering unwanted haptic feedback.
    """
    pitch = imu["pitch_deg"]
    roll  = imu["roll_deg"]

    result = {k: dict(v) for k, v in fused.items()}  # shallow copy

    if abs(pitch) > TILT_THRESHOLD_DEG:
        logger.info(
            "[VERIFY:IMU_SUPPRESS] axis=pitch pitch_deg=%.2f threshold_deg=%.1f "
            "suppressed=front,back  ← sensor pointing off-horizon, readings discarded",
            pitch, TILT_THRESHOLD_DEG,
        )
        for direction in ("front", "back"):
            result[direction].update({
                "distance_m": None,
                "source":     "suppressed(pitch_tilt)",
                "zone":       Zone.CLEAR,
                "vibration":  VIBRATION_PATTERNS[Zone.CLEAR].copy(),
            })

    if abs(roll) > TILT_THRESHOLD_DEG:
        logger.info(
            "[VERIFY:IMU_SUPPRESS] axis=roll  roll_deg=%.2f  threshold_deg=%.1f "
            "suppressed=left,right  ← sensor pointing off-horizon, readings discarded",
            roll, TILT_THRESHOLD_DEG,
        )
        for direction in ("left", "right"):
            result[direction].update({
                "distance_m": None,
                "source":     "suppressed(roll_tilt)",
                "zone":       Zone.CLEAR,
                "vibration":  VIBRATION_PATTERNS[Zone.CLEAR].copy(),
            })

    return result


# ---------------------------------------------------------------------------
# Motor command generation
# ---------------------------------------------------------------------------

def _generate_motor_commands(fused: dict[str, dict]) -> list[dict]:
    """
    Build the ordered list of 8 motor commands to return to the ESP32.

    Motor index maps 1-to-1 with ``DIRECTIONS``:
      Motor 0 → front, Motor 1 → front_right, … Motor 7 → front_left

    Each command contains everything the ESP32 firmware needs to drive the
    haptic motor PWM:

    .. code-block:: json

        {
            "motor_id":     0,
            "direction":    "front",
            "intensity":    153,
            "frequency_hz": 10,
            "pattern":      "medium_pulse",
            "active":       true,
            "zone":         "WARNING",
            "distance_m":   2.3,
            "source":       "tof"
        }
    """
    commands: list[dict] = []
    for idx, direction in enumerate(DIRECTIONS):
        reading = fused[direction]
        vib     = reading["vibration"]
        dist    = reading["distance_m"]
        commands.append({
            "motor_id":     idx,
            "direction":    direction,
            "intensity":    vib["intensity"],
            "frequency_hz": vib["frequency_hz"],
            "pattern":      vib["pattern"],
            "active":       vib["active"],
            "zone":         reading["zone"],
            "distance_m":   round(dist, 3) if dist is not None else None,
            "source":       reading["source"],
        })
    return commands


# ---------------------------------------------------------------------------
# Hazard summary
# ---------------------------------------------------------------------------

def _compute_hazard(fused: dict[str, dict]) -> dict:
    """
    Derive the overall hazard level and an alert list from all fused readings.

    Returns
    -------
    ::

        {
            "level": 3,          # 0 = CLEAR … 5 = CRITICAL
            "label": "WARNING",
            "alerts": [
                {"direction": "front", "zone": "WARNING",
                 "distance_m": 2.3, "source": "tof"},
                ...
            ]
        }

    Alerts are sorted most-critical first.
    """
    alerts: list[dict] = []
    worst_level = 0
    worst_label = Zone.CLEAR

    for direction, reading in fused.items():
        lvl = HAZARD_LEVEL[reading["zone"]]
        if lvl > 0:
            dist = reading["distance_m"]
            alerts.append({
                "direction":  direction,
                "zone":       reading["zone"],
                "distance_m": round(dist, 3) if dist is not None else None,
                "source":     reading["source"],
            })
        if lvl > worst_level:
            worst_level = lvl
            worst_label = reading["zone"]

    alerts.sort(key=lambda a: -HAZARD_LEVEL[a["zone"]])
    return {"level": worst_level, "label": worst_label, "alerts": alerts}


# ---------------------------------------------------------------------------
# Internal logging helpers  (used by process_packet / process_combined_packet)
# ---------------------------------------------------------------------------

def _log_fused_directions(device_id: str, module: str, fused: dict[str, dict]) -> None:
    """
    Emit one DEBUG line per non-CLEAR direction after fusion+IMU compensation.

    Each line carries enough context to verify 2.2.3 R1 (distance accuracy ≤±5%)
    and 2.2.3 R2 (moving obstacle detection) from the log file.
    """
    for direction, reading in fused.items():
        dist = reading["distance_m"]
        zone = reading["zone"]
        dist_str = f"{dist:.3f}m" if dist is not None else "  None "
        logger.debug(
            "[VERIFY:FUSE] device=%-20s module=%-8s dir=%-12s "
            "eff_m=%7s source=%-14s zone=%s",
            device_id, module, direction, dist_str, reading["source"], zone,
        )


def _log_hazard(device_id: str, module: str, hazard: dict) -> None:
    """
    Emit an INFO line whenever the fused hazard level is above CLEAR.

    Supports verification of:
      • 2.2.2 R1  – obstacle detection recall (presence/absence of HAZARD_DETECT)
      • 2.2.2 R3  – direction classification accuracy (worst_dir field)
      • 2.2.3 R2  – moving obstacle detection rate
    """
    if hazard["level"] == 0:
        logger.debug(
            "[VERIFY:HAZARD_DETECT] device=%-20s module=%-8s level=0 label=CLEAR",
            device_id, module,
        )
        return

    worst = hazard["alerts"][0]
    logger.info(
        "[VERIFY:HAZARD_DETECT] device=%-20s module=%-8s level=%d label=%-8s "
        "worst_dir=%-12s dist_m=%s source=%-14s alert_count=%d",
        device_id, module,
        hazard["level"], hazard["label"],
        worst["direction"],
        f"{worst['distance_m']:.3f}" if worst["distance_m"] is not None else "None",
        worst["source"],
        len(hazard["alerts"]),
    )
    # Log every alert direction so the full confusion-matrix row is captured
    for alert in hazard["alerts"]:
        logger.debug(
            "[VERIFY:HAZARD_ALERT] device=%-20s dir=%-12s zone=%-8s dist_m=%s source=%s",
            device_id, alert["direction"], alert["zone"],
            f"{alert['distance_m']:.3f}" if alert["distance_m"] is not None else "None",
            alert["source"],
        )


def _log_active_motors(device_id: str, motors: list[dict]) -> None:
    """
    Emit one DEBUG line listing every active motor.

    Used to build the confusion matrix for 2.2.1 R3 (cross-activation error <5%):
    compare logged active motors against the commanded direction.
    """
    active = [m for m in motors if m["active"]]
    if not active:
        logger.debug(
            "[VERIFY:MOTOR_CMD] device=%-20s active_count=0 all_motors=off",
            device_id,
        )
        return
    parts = [
        f"{m['direction']}(id={m['motor_id']},zone={m['zone']},"
        f"int={m['intensity']},freq={m['frequency_hz']}Hz)"
        for m in active
    ]
    logger.info(
        "[VERIFY:MOTOR_CMD] device=%-20s active_count=%d active=[%s]",
        device_id, len(active), ", ".join(parts),
    )


# ---------------------------------------------------------------------------
# Public interface (called by server.py)
# ---------------------------------------------------------------------------

def fresh_device_state() -> dict:
    """Return a blank per-device persistent state."""
    return {
        "last_seen_ms":  0.0,
        "packet_count":  0,
        "last_zone":     Zone.CLEAR,
        "module":        "unknown",
    }


def process_packet(data: dict[str, Any], state: dict, now_ms: float) -> dict:
    """
    Main entry point: parse one ESP32 packet and return motor commands.

    Parameters
    ----------
    data    : Parsed JSON payload from the ESP32 (see schema below).
    state   : Per-device mutable state dict kept by the server between calls.
    now_ms  : Current server time in milliseconds (for latency measurement).

    Request payload schema
    ----------------------
    ::

        {
          "device_id":  "esp32_head",
          "module":     "head",            ← "head" or "body"
          "tof_sensors": {
              "front":       {"distance_mm": 1200, "avg": 1200,
                              "min": 1100, "max": 1300, "count": 50},
              "front_right": { ... },
              "right":       { ... },
              "back_right":  { ... },
              "back":        { ... },
              "back_left":   { ... },
              "left":        { ... },
              "front_left":  { ... }
          },
          "mmwave_sensors": {
              "front": {"presence": true,  "det_state": 2, "dist_cm": 312, "energy": 10576, "frames": 4},
              "back":  {"presence": false, "det_state": 0}
          },
          "imu": {
              "roll_deg": 2.5, "pitch_deg": -1.2, "yaw_deg": 45.0,
              "accel_x": 0.01, "accel_y": 0.02,  "accel_z": 9.8
          },
          "timestamp": 37393
        }

    Returns
    -------
    ::

        {
          "module":     "head",
          "hazard":     {"level": 3, "label": "WARNING", "alerts": [...]},
          "motors":     [ {motor_id, direction, intensity,
                           frequency_hz, pattern, active,
                           zone, distance_m, source} × 8 ],
          "imu":        {"roll_deg": 2.5, "pitch_deg": -1.2, "yaw_deg": 45.0},
          "latency_ms": 12.4
        }
    """
    # ── 1. Module identification ─────────────────────────────────────────────
    module    = str(data.get("module", "body")).lower()
    device_id = str(data.get("device_id", "unknown"))
    if module not in ("head", "body"):
        module = "body"  # safe default

    # ── 2. Parse raw sensor blocks ───────────────────────────────────────────
    tof_distances  = _parse_tof_sensors(data.get("tof_sensors", {}))
    mmwave_detects = _parse_mmwave_sensors(data.get("mmwave_sensors", {}))
    imu            = _parse_imu(data.get("imu", {}))

    # [VERIFY:IMU_READ] – continuous yaw log for 2.2.3 orientation drift check
    logger.info(
        "[VERIFY:IMU_READ] device=%-14s module=%-4s "
        "roll=%7.2f  pitch=%7.2f  yaw=%7.2f  accel_z=%.3f",
        device_id, module,
        imu["roll_deg"], imu["pitch_deg"], imu["yaw_deg"], imu["accel_z"],
    )

    # ── 3. Hybrid sensor fusion ──────────────────────────────────────────────
    fused = _fuse_sensor_data(tof_distances, mmwave_detects, module)

    # ── 4. IMU tilt compensation ─────────────────────────────────────────────
    fused = _apply_imu_compensation(fused, imu)

    # [VERIFY:FUSE] – per-direction fusion result for 2.2.3 distance accuracy
    _log_fused_directions(device_id, module, fused)

    # ── 5. Generate outputs ──────────────────────────────────────────────────
    motors = _generate_motor_commands(fused)
    hazard = _compute_hazard(fused)

    # [VERIFY:HAZARD_DETECT] – direction + zone for 2.2.2 classification check
    _log_hazard(device_id, module, hazard)

    # [VERIFY:MOTOR_CMD] – active motors for cross-activation error check (2.2.1)
    _log_active_motors(device_id, motors)

    # ── 6. Update persistent per-device state ────────────────────────────────
    state["last_seen_ms"]  = now_ms
    state["packet_count"] += 1
    state["last_zone"]     = hazard["label"]
    state["module"]        = module

    latency_ms = round(now_ms - int(data.get("timestamp", now_ms)), 1)

    return {
        "module":     module,
        "hazard":     hazard,
        "motors":     motors,
        "imu":        {
            "roll_deg":  imu["roll_deg"],
            "pitch_deg": imu["pitch_deg"],
            "yaw_deg":   imu["yaw_deg"],
        },
        "latency_ms": latency_ms,
    }


def process_combined_packet(
    head_data: dict[str, Any],
    body_data: dict[str, Any],
    state: dict,
    now_ms: float,
) -> dict:
    """
    Fuse sensor data from both the HEAD and BODY modules simultaneously and
    produce a single set of motor commands for the head module.

    Each module's IMU is applied to suppress its own tilted sensor channels
    before cross-module merging, so a tilted body cannot mute valid head
    readings, and vice-versa.  The final zone classification always uses
    head thresholds because the output motors are on the head.

    Parameters
    ----------
    head_data : Latest parsed JSON packet from the head module (or ``{}``).
    body_data : Latest parsed JSON packet from the body module (or ``{}``).
    state     : Mutable combined persistent state dict kept by the server.
    now_ms    : Current server time in milliseconds (for latency measurement).
    """
    head_id = str(head_data.get("device_id", "head"))
    body_id = str(body_data.get("device_id", "body"))

    # ── 1. Parse each module's sensors independently ─────────────────────────
    head_tof    = _parse_tof_sensors(head_data.get("tof_sensors", {}))
    body_tof    = _parse_tof_sensors(body_data.get("tof_sensors", {}))
    head_mmwave = _parse_mmwave_sensors(head_data.get("mmwave_sensors", {}))
    body_mmwave = _parse_mmwave_sensors(body_data.get("mmwave_sensors", {}))
    head_imu    = _parse_imu(head_data.get("imu", {}))
    body_imu    = _parse_imu(body_data.get("imu", {}))

    # [VERIFY:IMU_READ] – log both module IMUs for orientation drift analysis (2.2.3)
    logger.info(
        "[VERIFY:IMU_READ] device=%-14s module=head "
        "roll=%7.2f  pitch=%7.2f  yaw=%7.2f  accel_z=%.3f",
        head_id,
        head_imu["roll_deg"], head_imu["pitch_deg"],
        head_imu["yaw_deg"],  head_imu["accel_z"],
    )
    logger.info(
        "[VERIFY:IMU_READ] device=%-14s module=body "
        "roll=%7.2f  pitch=%7.2f  yaw=%7.2f  accel_z=%.3f",
        body_id,
        body_imu["roll_deg"], body_imu["pitch_deg"],
        body_imu["yaw_deg"],  body_imu["accel_z"],
    )

    # ── 2. Fuse and apply per-module IMU compensation ─────────────────────────
    #   Each module's tilt suppresses only its own readings, keeping the two
    #   modules independent before the cross-module merge.
    head_fused = _fuse_sensor_data(head_tof, head_mmwave, module="head")
    head_fused = _apply_imu_compensation(head_fused, head_imu)

    body_fused = _fuse_sensor_data(body_tof, body_mmwave, module="body")
    body_fused = _apply_imu_compensation(body_fused, body_imu)

    # ── 3. Merge: safety-first min across both modules; head thresholds ───────
    fused = _merge_fused_dicts(head_fused, body_fused)

    # ── 4. Generate outputs ───────────────────────────────────────────────────
    motors = _generate_motor_commands(fused)
    hazard = _compute_hazard(fused)

    # [VERIFY:FUSE] – merged per-direction result for distance accuracy check (2.2.3)
    _log_fused_directions(f"{head_id}+{body_id}", "combined", fused)

    # [VERIFY:HAZARD_DETECT] – direction + zone classification result (2.2.2)
    _log_hazard(f"{head_id}+{body_id}", "combined", hazard)

    # [VERIFY:MOTOR_CMD] – active motor set for cross-activation check (2.2.1)
    _log_active_motors(f"{head_id}+{body_id}", motors)

    # ── 5. Update combined persistent state ──────────────────────────────────
    state["last_seen_ms"]  = now_ms
    state["packet_count"] += 1
    state["last_zone"]     = hazard["label"]
    state["module"]        = "combined"

    # Latency relative to the most recent timestamp from either module
    ts = max(int(head_data.get("timestamp", 0)), int(body_data.get("timestamp", 0)))
    latency_ms = round(now_ms - ts, 1) if ts else 0.0

    return {
        "module":     "combined",
        "hazard":     hazard,
        "motors":     motors,
        "imu":        {
            "roll_deg":  head_imu["roll_deg"],
            "pitch_deg": head_imu["pitch_deg"],
            "yaw_deg":   head_imu["yaw_deg"],
        },
        "latency_ms": latency_ms,
    }


def _merge_fused_dicts(
    fused_a: dict[str, dict],
    fused_b: dict[str, dict],
) -> dict[str, dict]:
    """
    Merge two already-IMU-compensated fused direction dicts.

    For each direction the closer (more dangerous) distance wins.  After
    merging all zones are reclassified with head thresholds because the
    combined motor output is intended for the head module.
    """
    merged: dict[str, dict] = {}
    for direction in DIRECTIONS:
        a, b     = fused_a[direction], fused_b[direction]
        dist_a   = a["distance_m"]
        dist_b   = b["distance_m"]

        if dist_a is not None and dist_b is not None:
            eff_dist = min(dist_a, dist_b)
            source   = a["source"] if dist_a <= dist_b else b["source"]
        elif dist_a is not None:
            eff_dist, source = dist_a, a["source"]
        elif dist_b is not None:
            eff_dist, source = dist_b, b["source"]
        else:
            eff_dist = None
            # Preserve suppression annotation if either module suppressed the channel
            source = a["source"] if "suppressed" in a["source"] else b["source"]

        # Reclassify with head thresholds – output is always for the head motors
        zone    = classify_distance(eff_dist, module="head") if eff_dist is not None else Zone.CLEAR
        pattern = VIBRATION_PATTERNS[zone].copy()
        merged[direction] = {
            "distance_m": eff_dist,
            "source":     source,
            "zone":       zone,
            "vibration":  pattern,
        }
    return merged


def print_decision(
    alerts: list[dict],
    level: int,
    device_id: str,
    pkt_ts: int,
) -> None:
    """Log the per-packet decision summary (replaces earlier print-based debug aid)."""
    _labels = {0: "CLEAR", 1: "ALERT", 2: "CAUTION",
               3: "WARNING", 4: "DANGER", 5: "CRITICAL"}
    label = _labels.get(level, "?")
    if alerts:
        parts = [
            f"{a['direction']}@{a['distance_m']}m({a['zone'][0]})"
            for a in alerts[:4]
        ]
        logger.info(
            "[VERIFY:DECISION] pkt_ts=%8d ms  device=%-14s hazard=%s(%d)  → %s",
            pkt_ts, device_id, label, level, ", ".join(parts),
        )
    else:
        logger.info(
            "[VERIFY:DECISION] pkt_ts=%8d ms  device=%-14s hazard=%s(%d)",
            pkt_ts, device_id, label, level,
        )
