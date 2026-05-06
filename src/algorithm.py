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
# Must match ESP32 firmware gate: reject wrap / harmonics / bogus near-zero.
TOF_MIN_VALID_MM_FW = 40
TOF_MAX_VALID_MM_FW = 4000
TILT_THRESHOLD_DEG = 15.0   # degrees – tilt beyond which a channel WOULD be suppressed
                            # (only used when IMU_TILT_SUPPRESS_ENABLED is True)

# Whether to drop sensor channels whose IMU pitch/roll is past TILT_THRESHOLD_DEG.
# Disabled by default so the dashboard always shows the raw fused readings;
# tilt-aware safety is still represented through the body orientation block
# the dashboard renders separately. Set to True to re-enable the legacy
# behaviour (suppressed(pitch_tilt) / suppressed(roll_tilt) sources).
IMU_TILT_SUPPRESS_ENABLED = False

# ── IMU motion thresholds (ICM-20948) ────────────────────────────────────────
# These mirror the firmware-side computeIMUOrientation() classifiers so the
# server validates them rather than blindly trusting the boolean flag from the
# wire — keeps the system robust if a future firmware widens the thresholds.
TURN_RATE_THRESHOLD_DPS = 30.0   # |gyro_z| above this → user is yawing
FALL_ACCEL_G_LOW        = 0.40   # |accel| below this → free-fall window
FALL_ACCEL_G_HIGH       = 2.50   # |accel| above this → impact spike
STILL_ACCEL_TOL_G       = 0.05   # |accel-1g| below this AND ↓ → stationary
STILL_GYRO_TOL_DPS      = 5.0    # |gyro|     below this AND ↑ → stationary

# When the user is actively turning, briefly soften zone classifications by
# this many "steps" (CRITICAL→DANGER→WARNING→…). Prevents single-frame false
# alarms from sensors sweeping past a wall during a head/body rotation.
TURN_ZONE_SOFTEN_STEPS = 1

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


# ---------------------------------------------------------------------------
# Heading / turn helpers — used by IMU-aware navigation in server.py
# ---------------------------------------------------------------------------

def wrap_180(deg: float) -> float:
    """Wrap an angle to the half-open range (-180, +180]."""
    deg = (deg + 180.0) % 360.0 - 180.0
    # Pythons modulo gives (-180, 180], which is exactly what we want
    return deg


# Maps an 8-direction compass command back to the signed yaw delta the
# command implies (positive = clockwise / right turn, negative = left).
# "front" = no turn, "back" = full 180° (sign is arbitrary so we use +180).
_COMMAND_DELTA_DEG: dict[str, float] = {
    "front":         0.0,
    "front_right":  45.0,
    "right":        90.0,
    "back_right":  135.0,
    "back":        180.0,
    "back_left":  -135.0,
    "left":        -90.0,
    "front_left":  -45.0,
    "stop":          0.0,
}


def relative_bearing_to_command(
    target_world_deg: float,
    user_yaw_deg: float,
) -> tuple[str, float]:
    """
    Translate an absolute compass bearing (e.g. from Google Maps) into a
    body-frame haptic command using the user's current heading.

    Parameters
    ----------
    target_world_deg : Desired compass bearing (0=N, 90=E, 180=S, 270=W).
    user_yaw_deg     : The IMU's tilt-compensated yaw (same convention).

    Returns
    -------
    (command, delta_deg)
        command   : one of front / front_right / right / back_right /
                    back / back_left / left / front_left
        delta_deg : signed turn angle in (-180, +180]; positive = clockwise

    8-sector mapping (45° wide each, ±22.5° around the centre):
        |delta|  ≤ 22.5° → front
                 ≤ 67.5° → front_right (delta>0) / front_left (delta<0)
                 ≤112.5° → right       / left
                 ≤157.5° → back_right  / back_left
                 else    → back
    """
    delta = wrap_180(target_world_deg - user_yaw_deg)
    abs_d = abs(delta)
    if   abs_d <= 22.5:  cmd = "front"
    elif abs_d <= 67.5:  cmd = "front_right" if delta > 0 else "front_left"
    elif abs_d <= 112.5: cmd = "right"       if delta > 0 else "left"
    elif abs_d <= 157.5: cmd = "back_right"  if delta > 0 else "back_left"
    else:                cmd = "back"
    return cmd, delta


def expected_turn_angle_deg(command: str) -> float:
    """
    Magnitude of yaw the user is expected to execute for a turn command.
    Used as the target for turn-confirmation integration.
    """
    return abs(_COMMAND_DELTA_DEG.get(command, 0.0))


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

def _tof_subdict_to_raw_mm(reading: dict | None) -> int | None:
    """
    Extract one usable VL53 distance in millimetres from a single direction
    object, or ``None`` if the firmware signalled "no sample" (``{}``) or the
    values are out of band.

    Prefer ``distance_mm`` when that key is present so we do not latch onto a
    smoothed ``avg`` after the live sample has gone invalid on the wire.
    """
    if not isinstance(reading, dict) or not reading:
        return None

    def _coerce(val: object) -> int | None:
        if val is None:
            return None
        try:
            iv = int(float(val))  # type: ignore[arg-type]
        except (TypeError, ValueError):
            return None
        if iv <= 0 or iv >= TOF_INVALID_MM:
            return None
        if iv < TOF_MIN_VALID_MM_FW or iv > TOF_MAX_VALID_MM_FW:
            return None
        return iv

    if "distance_mm" in reading:
        hit = _coerce(reading.get("distance_mm"))
        if hit is not None:
            return hit
    return _coerce(reading.get("avg"))


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
        if not isinstance(reading, dict):
            result[direction] = None
            continue
        raw_mm = _tof_subdict_to_raw_mm(reading)
        if raw_mm is None:
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

    Minimum schema (legacy / MPU-6050 compatible)::

        "imu": {
            "roll_deg":  2.5,
            "pitch_deg": -1.2,
            "yaw_deg":   45.0,
            "accel_x": 0.01, "accel_y": 0.02, "accel_z": 9.8
        }

    Extended schema (ICM-20948 firmware, v2 onward) — fully optional. Each
    extra field has a safe default so the algorithm degrades gracefully when
    the firmware sends only the legacy block (or nothing at all)::

        "imu": {
            ...legacy fields...,
            "heading_cardinal": "SE",
            "gyro_x":  1.7,  "gyro_y":  7.8,  "gyro_z":  17.2,
            "mag_x": -10.1,  "mag_y":  47.1,  "mag_z": -15.9,
            "temp_c":   32.1,
            "accel_g":  1.16,  "gyro_dps": 19.0,
            "motion_state":   "moving",     # "still"|"moving"|"turning"|"falling"
            "is_still":       false,
            "is_turning":     false,
            "is_falling":     false,
            "mag_calibrated": false,
            "read_count":     12, "err_count": 0
        }
    """
    return {
        # ── Legacy / required ──────────────────────────────────────────────
        "roll_deg":         float(imu_data.get("roll_deg",  0.0)),
        "pitch_deg":        float(imu_data.get("pitch_deg", 0.0)),
        "yaw_deg":          float(imu_data.get("yaw_deg",   0.0)),
        "accel_x":          float(imu_data.get("accel_x",   0.0)),
        "accel_y":          float(imu_data.get("accel_y",   0.0)),
        "accel_z":          float(imu_data.get("accel_z",   9.81)),
        # ── Extended (ICM-20948) — safe defaults for legacy packets ────────
        "heading_cardinal": str(imu_data.get("heading_cardinal", "")),
        "gyro_x":           float(imu_data.get("gyro_x", 0.0)),
        "gyro_y":           float(imu_data.get("gyro_y", 0.0)),
        "gyro_z":           float(imu_data.get("gyro_z", 0.0)),
        "mag_x":            float(imu_data.get("mag_x",  0.0)),
        "mag_y":            float(imu_data.get("mag_y",  0.0)),
        "mag_z":            float(imu_data.get("mag_z",  0.0)),
        "temp_c":           float(imu_data.get("temp_c", 0.0)),
        "accel_g":          float(imu_data.get("accel_g",  1.0)),
        "gyro_dps":         float(imu_data.get("gyro_dps", 0.0)),
        "motion_state":     str(imu_data.get("motion_state", "unknown")),
        "is_still":         bool(imu_data.get("is_still",       False)),
        "is_turning":       bool(imu_data.get("is_turning",     False)),
        "is_falling":       bool(imu_data.get("is_falling",     False)),
        "mag_calibrated":   bool(imu_data.get("mag_calibrated", False)),
        "read_count":       int(imu_data.get("read_count", 0)),
        "err_count":        int(imu_data.get("err_count", 0)),
        # Whether this packet actually carried IMU data at all — used by the
        # response builder to advertise availability to clients.
        "_present":         bool(imu_data),
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
# IMU motion modifiers — turning suppression & fall detection
# ---------------------------------------------------------------------------

# Zones in order of severity (low→high) so we can step "downward" by N levels
# when softening due to a transient turning motion.
_ZONE_LADDER: list[str] = [
    Zone.CLEAR, Zone.ALERT, Zone.CAUTION,
    Zone.WARNING, Zone.DANGER, Zone.CRITICAL,
]


def _soften_zone(zone: str, steps: int) -> str:
    """Drop ``zone`` by ``steps`` rungs on the severity ladder (clamped at CLEAR)."""
    if steps <= 0 or zone not in _ZONE_LADDER:
        return zone
    idx = max(0, _ZONE_LADDER.index(zone) - steps)
    return _ZONE_LADDER[idx]


def _apply_imu_motion_modifiers(
    fused: dict[str, dict],
    imu: dict,
    module: str = "body",
) -> tuple[dict[str, dict], dict]:
    """
    Apply motion-derived adjustments to the fused direction map.

    Two effects are applied (independent):

    1. **Turning suppression** – when the IMU reports the user is actively
       yawing (``|gyro_z| > TURN_RATE_THRESHOLD_DPS``), every direction's
       zone is softened by ``TURN_ZONE_SOFTEN_STEPS``. Rationale: while the
       user rotates their head/body, ToF/radar sensors briefly sweep past
       walls and produce flicker alarms that vanish a frame later. Keeping
       genuine close-range hazards above CRITICAL/DANGER ensures we never
       miss a real obstacle — only WARNING/CAUTION are dampened.

    2. **Fall detection** – combines firmware ``is_falling`` with a
       server-side validation of the accel-magnitude window. Returned as a
       dict alongside the modified fused map so ``_compute_hazard()`` can
       prepend it to the alert list as the highest-priority alert.

    Returns
    -------
    (fused_modified, motion_summary) where motion_summary is::

        {
            "is_turning": bool,
            "is_falling": bool,
            "is_still":   bool,
            "soften_steps": int,                 # how many ladder rungs we dropped
            "fall_alert": dict | None,           # alert payload if falling
        }
    """
    result = {k: dict(v) for k, v in fused.items()}

    # Server-side validation of the firmware booleans. Trust both, but only
    # act on the conjunction so a stuck flag can't dominate. ``accel_g`` is
    # the magnitude already computed on the firmware; we sanity-check it.
    accel_g  = float(imu.get("accel_g",  1.0))
    gyro_dps = float(imu.get("gyro_dps", 0.0))
    gyro_z   = float(imu.get("gyro_z",   0.0))

    is_turning_flag = bool(imu.get("is_turning", False))
    is_falling_flag = bool(imu.get("is_falling", False))
    is_still_flag   = bool(imu.get("is_still",   False))

    is_turning = is_turning_flag and abs(gyro_z) > TURN_RATE_THRESHOLD_DPS
    # Validated fall = firmware flag AND accel-magnitude is in either the
    # free-fall window (|a| < LOW) or an impact-spike window (|a| > HIGH).
    # The firmware only sets is_falling for free-fall, so this is mainly a
    # defence-in-depth check that catches stuck flags.
    is_falling = is_falling_flag and (accel_g < FALL_ACCEL_G_LOW or accel_g > FALL_ACCEL_G_HIGH)
    is_still   = (
        is_still_flag
        and abs(accel_g - 1.0) < STILL_ACCEL_TOL_G
        and gyro_dps < STILL_GYRO_TOL_DPS
    )

    # ── (1) Turning soften ───────────────────────────────────────────────
    soften_steps = 0
    if is_turning:
        soften_steps = TURN_ZONE_SOFTEN_STEPS
        logger.info(
            "[VERIFY:IMU_TURN] device_module=%-4s gyro_z_dps=%+7.2f "
            "threshold_dps=%.1f  soften_steps=%d  ← head/body actively rotating",
            module, gyro_z, TURN_RATE_THRESHOLD_DPS, soften_steps,
        )
        for direction, reading in result.items():
            new_zone = _soften_zone(reading["zone"], soften_steps)
            if new_zone != reading["zone"]:
                reading["zone"]      = new_zone
                reading["vibration"] = VIBRATION_PATTERNS[new_zone].copy()
                # Annotate so dashboards can show "softened by turning"
                reading["source"]    = f"{reading['source']}(turn_soft)"

    # ── (2) Fall detection ───────────────────────────────────────────────
    fall_alert = None
    if is_falling:
        logger.warning(
            "[VERIFY:IMU_FALL] device_module=%-4s accel_g=%.2f "
            "(free-fall <%.2fg or impact >%.2fg)  ← FALL EVENT",
            module, accel_g, FALL_ACCEL_G_LOW, FALL_ACCEL_G_HIGH,
        )
        fall_alert = {
            "direction":  "all",
            "zone":       Zone.CRITICAL,
            "distance_m": 0.0,
            "source":     "imu_fall_detect",
            "kind":       "fall",
            "accel_g":    round(accel_g, 2),
        }

    if is_still:
        logger.debug(
            "[VERIFY:IMU_STILL] device_module=%-4s accel_g=%.2f gyro_dps=%.1f "
            "← stationary; haptics may suppress when CLEAR",
            module, accel_g, gyro_dps,
        )

    return result, {
        "is_turning":   is_turning,
        "is_falling":   is_falling,
        "is_still":     is_still,
        "soften_steps": soften_steps,
        "fall_alert":   fall_alert,
    }


def _empty_motion_summary() -> dict:
    """
    Return a no-motion summary, matching the shape produced by
    ``_apply_imu_motion_modifiers``. Used as a stand-in when a module has
    no IMU (e.g. our current build only has an ICM-20948 on the body), so
    the merge / hazard logic can run unchanged.
    """
    return {
        "is_turning":   False,
        "is_falling":   False,
        "is_still":     False,
        "soften_steps": 0,
        "fall_alert":   None,
    }


def _merge_motion_summaries(head_motion: dict, body_motion: dict) -> dict:
    """
    Combine per-module motion summaries into one for dual-module decisions.

    Logical OR for the booleans (a fall detected on EITHER module is a fall),
    max for the soften level, and the more-severe fall alert wins (head
    preferred when both are present, since head has the alert headband).
    """
    fall_alert = head_motion.get("fall_alert") or body_motion.get("fall_alert")
    return {
        "is_turning":   bool(head_motion["is_turning"] or body_motion["is_turning"]),
        "is_falling":   bool(head_motion["is_falling"] or body_motion["is_falling"]),
        "is_still":     bool(head_motion["is_still"]   and body_motion["is_still"]),
        "soften_steps": max(head_motion["soften_steps"], body_motion["soften_steps"]),
        "fall_alert":   fall_alert,
    }


def _build_imu_response(imu: dict, motion: dict) -> dict:
    """
    Build the ``imu`` block returned to the ESP32 / dashboard.

    Surfaces the legacy fields (so older clients keep working) plus the
    derived motion state and a small "advice" sub-object the firmware can
    use to drive UX decisions (e.g. silent-mode when stationary,
    emergency-pattern when falling) without re-implementing the same
    thresholds locally.
    """
    return {
        # Presence flag — false means the firmware sent no IMU payload (e.g.
        # the head module in our current build has no IMU). Dashboards use
        # this to draw a "(not equipped)" placeholder instead of fake zeros.
        "present":          bool(imu.get("_present", False)),
        # Legacy (always present)
        "roll_deg":         round(imu["roll_deg"],  2),
        "pitch_deg":        round(imu["pitch_deg"], 2),
        "yaw_deg":          round(imu["yaw_deg"],   2),
        # Extended — only meaningful when ICM-20948 firmware is in use, but
        # always echoed back for client convenience (defaults are harmless).
        "heading_cardinal": imu.get("heading_cardinal", ""),
        "accel_x":          round(imu.get("accel_x", 0.0),  2),
        "accel_y":          round(imu.get("accel_y", 0.0),  2),
        "accel_z":          round(imu.get("accel_z", 9.81), 2),
        "gyro_x":           round(imu.get("gyro_x",  0.0),  1),
        "gyro_y":           round(imu.get("gyro_y",  0.0),  1),
        "gyro_z":           round(imu.get("gyro_z",  0.0),  1),
        "mag_x":            round(imu.get("mag_x",   0.0),  1),
        "mag_y":            round(imu.get("mag_y",   0.0),  1),
        "mag_z":            round(imu.get("mag_z",   0.0),  1),
        "accel_g":          round(imu.get("accel_g",  1.0), 2),
        "gyro_dps":         round(imu.get("gyro_dps", 0.0), 1),
        "temp_c":           round(imu.get("temp_c",   0.0), 1),
        "motion_state":     imu.get("motion_state", "unknown"),
        "is_still":         bool(motion.get("is_still",   False)),
        "is_turning":       bool(motion.get("is_turning", False)),
        "is_falling":       bool(motion.get("is_falling", False)),
        "mag_calibrated":   bool(imu.get("mag_calibrated", False)),
        # UX hints derived server-side. The firmware can act on these without
        # knowing the exact threshold constants used here.
        "advice": {
            "silence_haptics": (
                bool(motion.get("is_still", False))
                # Only suggest silencing when nothing is in alert range; the
                # caller already has the hazard dict to apply this gate too.
            ),
            "fall_emergency":  bool(motion.get("is_falling", False)),
            "soften_steps":    int(motion.get("soften_steps", 0)),
        },
    }


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

def _compute_hazard(
    fused: dict[str, dict],
    motion_summary: dict | None = None,
) -> dict:
    """
    Derive the overall hazard level and an alert list from all fused readings.

    Parameters
    ----------
    fused          : direction → reading dict produced by the fusion stage.
    motion_summary : optional dict from ``_apply_imu_motion_modifiers`` —
                     used to inject a top-priority FALL alert when present.

    Returns
    -------
    ::

        {
            "level": 3,          # 0 = CLEAR … 5 = CRITICAL
            "label": "WARNING",
            "alerts": [
                {"direction": "all",   "zone": "CRITICAL", "kind": "fall", ...},
                {"direction": "front", "zone": "WARNING",
                 "distance_m": 2.3, "source": "tof"},
                ...
            ]
        }

    Alerts are sorted most-critical first; fall alerts always rank above
    obstacle alerts at the same severity level.
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
                "kind":       "obstacle",
            })
        if lvl > worst_level:
            worst_level = lvl
            worst_label = reading["zone"]

    # Inject the IMU-derived fall alert (if any). It always pegs hazard to
    # CRITICAL because a falling user is the highest-priority event the
    # system can report.
    fall_alert = None
    if motion_summary is not None:
        fall_alert = motion_summary.get("fall_alert")
    if fall_alert is not None:
        alerts.insert(0, fall_alert)
        worst_level = HAZARD_LEVEL[Zone.CRITICAL]
        worst_label = Zone.CRITICAL

    # Sort by zone severity, then put fall (kind=="fall") first within a tie.
    alerts.sort(key=lambda a: (-HAZARD_LEVEL[a["zone"]], 0 if a.get("kind") == "fall" else 1))
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
    has_imu        = bool(imu.get("_present"))

    # [VERIFY:IMU_READ] – continuous yaw log for 2.2.3 orientation drift check.
    # Skip when this module has no IMU (avoids logging phantom zeros in the
    # head-only deployment).
    if has_imu:
        logger.info(
            "[VERIFY:IMU_READ] device=%-14s module=%-4s "
            "roll=%7.2f  pitch=%7.2f  yaw=%7.2f  accel_z=%.3f",
            device_id, module,
            imu["roll_deg"], imu["pitch_deg"], imu["yaw_deg"], imu["accel_z"],
        )

    # ── 3. Hybrid sensor fusion ──────────────────────────────────────────────
    fused = _fuse_sensor_data(tof_distances, mmwave_detects, module)

    # ── 4a-b. IMU motion modifiers (only if equipped). Tilt-based channel
    #         suppression is disabled by default so the dashboard always sees
    #         every sensor reading; the body orientation panel + the suppress
    #         flag in motion_state still convey tilt awareness.
    if has_imu:
        if IMU_TILT_SUPPRESS_ENABLED:
            fused = _apply_imu_compensation(fused, imu)
        fused, motion = _apply_imu_motion_modifiers(fused, imu, module)
    else:
        motion = _empty_motion_summary()

    # [VERIFY:FUSE] – per-direction fusion result for 2.2.3 distance accuracy
    _log_fused_directions(device_id, module, fused)

    # ── 5. Generate outputs ──────────────────────────────────────────────────
    motors = _generate_motor_commands(fused)
    hazard = _compute_hazard(fused, motion)

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
        "imu":        _build_imu_response(imu, motion),
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

    head_has_imu = bool(head_imu.get("_present"))
    body_has_imu = bool(body_imu.get("_present"))

    # [VERIFY:IMU_READ] – orientation drift log (2.2.3). Skip the line for
    # modules that didn't ship an IMU so the log isn't filled with phantom
    # zeros (current build: ICM-20948 lives on the body only).
    if head_has_imu:
        logger.info(
            "[VERIFY:IMU_READ] device=%-14s module=head "
            "roll=%7.2f  pitch=%7.2f  yaw=%7.2f  accel_z=%.3f",
            head_id,
            head_imu["roll_deg"], head_imu["pitch_deg"],
            head_imu["yaw_deg"],  head_imu["accel_z"],
        )
    if body_has_imu:
        logger.info(
            "[VERIFY:IMU_READ] device=%-14s module=body "
            "roll=%7.2f  pitch=%7.2f  yaw=%7.2f  accel_z=%.3f",
            body_id,
            body_imu["roll_deg"], body_imu["pitch_deg"],
            body_imu["yaw_deg"],  body_imu["accel_z"],
        )

    # ── 2. Fuse and apply per-module IMU compensation ─────────────────────────
    #   Each module's tilt suppresses only its own readings, keeping the two
    #   modules independent before the cross-module merge. A module without
    #   an IMU passes through untouched and contributes a no-motion summary,
    #   so it can never trigger a phantom turn / fall / tilt-suppress.
    head_fused = _fuse_sensor_data(head_tof, head_mmwave, module="head")
    if head_has_imu:
        if IMU_TILT_SUPPRESS_ENABLED:
            head_fused = _apply_imu_compensation(head_fused, head_imu)
        head_fused, head_motion = _apply_imu_motion_modifiers(head_fused, head_imu, "head")
    else:
        head_motion = _empty_motion_summary()

    body_fused = _fuse_sensor_data(body_tof, body_mmwave, module="body")
    if body_has_imu:
        if IMU_TILT_SUPPRESS_ENABLED:
            body_fused = _apply_imu_compensation(body_fused, body_imu)
        body_fused, body_motion = _apply_imu_motion_modifiers(body_fused, body_imu, "body")
    else:
        body_motion = _empty_motion_summary()

    # ── 3. Merge: safety-first min across both modules; head thresholds ───────
    fused = _merge_fused_dicts(head_fused, body_fused)

    # ── 3b. Merge motion summaries — fall on EITHER module pegs hazard ───────
    motion = _merge_motion_summaries(head_motion, body_motion)

    # ── 4. Generate outputs ───────────────────────────────────────────────────
    motors = _generate_motor_commands(fused)
    hazard = _compute_hazard(fused, motion)

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

    # Canonical orientation: prefer the body (the ICM-20948 lives there in
    # the current build), fall back to head when the body packet is missing
    # or has no IMU. Motion summary is the merged one — a fall detected on
    # either module bubbles up here.
    if body_has_imu:
        combined_imu = _build_imu_response(body_imu, motion)
        combined_imu["canonical_module"] = "body"
    elif head_has_imu:
        combined_imu = _build_imu_response(head_imu, motion)
        combined_imu["canonical_module"] = "head"
    else:
        combined_imu = _build_imu_response(body_imu, motion)  # all-zero defaults
        combined_imu["canonical_module"] = "none"

    # Per-module sub-blocks always exist so dashboards can address them by
    # name; the "present" flag inside indicates whether the values are real.
    combined_imu["body"] = _build_imu_response(body_imu, body_motion)
    combined_imu["head"] = _build_imu_response(head_imu, head_motion)
    combined_imu["body_present"] = body_has_imu
    combined_imu["head_present"] = head_has_imu

    return {
        "module":     "combined",
        "hazard":     hazard,
        "motors":     motors,
        "imu":        combined_imu,
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
            # Neither module has a range — do not inherit a misleading "tof" label
            # from whichever dict happened to be chosen last.
            sa, sb = a["source"], b["source"]
            if "suppressed" in sa:
                source = sa
            elif "suppressed" in sb:
                source = sb
            else:
                source = "none"

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
