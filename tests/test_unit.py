"""
Unit tests for algorithm.py  |  ECE 445 Group 10
=================================================

Covers every public and private function in isolation.  Tests are grouped
by functional area:

  1.  classify_distance           – zone boundary correctness
  2.  _parse_tof_sensors          – ToF parsing & sentinel handling
  3.  _parse_mmwave_sensors       – mmWave parsing & out-of-range handling
  4.  _parse_imu                  – IMU parsing & default fallbacks
  5.  _fuse_sensor_data           – hybrid fusion scenarios
  6.  _apply_imu_compensation     – tilt suppression
  7.  _generate_motor_commands    – motor list structure
  8.  _compute_hazard             – hazard level & alert ordering
  9.  process_packet              – end-to-end (head + body modules)
  10. fresh_device_state          – initial state shape
"""

import sys
import os

# Allow imports from the workspace root whether tests run from root or tests/
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import pytest

from algorithm import (
    DIRECTIONS,
    NUM_DIRECTIONS,
    MMWAVE_COVERAGE,
    TOF_MAX_RANGE_M,
    MMWAVE_MAX_RANGE_M,
    TOF_INVALID_MM,
    TILT_THRESHOLD_DEG,
    VIBRATION_PATTERNS,
    HAZARD_LEVEL,
    Zone,
    classify_distance,
    fresh_device_state,
    process_packet,
    _parse_tof_sensors,
    _parse_mmwave_sensors,
    _parse_imu,
    _fuse_sensor_data,
    _apply_imu_compensation,
    _generate_motor_commands,
    _compute_hazard,
)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _empty_tof() -> dict:
    """All 8 ToF sensors reporting no target (clear)."""
    return {d: None for d in DIRECTIONS}


def _all_tof(distance_mm: int) -> dict:
    """All 8 ToF sensors reporting the same distance."""
    return {d: {"distance_mm": distance_mm, "avg": distance_mm,
                "min": distance_mm - 10, "max": distance_mm + 10,
                "count": 50}
            for d in DIRECTIONS}


def _empty_mmwave() -> dict:
    return {"front": None, "back": None}


def _make_mmwave(sensor_id: str, range_m: float) -> dict:
    """Build a **parsed** mmwave dict (output of _parse_mmwave_sensors) for use
    in _fuse_sensor_data tests.  Both sensors default to None (no detection)."""
    base: dict = {"front": None, "back": None}
    base[sensor_id] = {"range_m": range_m, "speed_ms": 0.0, "energy": 5000}
    return base


def _make_mmwave_raw(sensor_id: str, dist_cm: int, energy: int = 5000) -> dict:
    """Build a **raw firmware-format** mmwave_sensors dict for packet-level tests."""
    base = {"front": {"presence": False, "det_state": 0},
            "back":  {"presence": False, "det_state": 0}}
    base[sensor_id] = {"presence": True, "det_state": 2,
                       "dist_cm": dist_cm, "energy": energy, "frames": 4}
    return base


def _all_clear_fused() -> dict:
    """Pre-built fused dict with all directions CLEAR."""
    return {
        d: {
            "distance_m": None,
            "source":     "none",
            "zone":       Zone.CLEAR,
            "vibration":  VIBRATION_PATTERNS[Zone.CLEAR].copy(),
        }
        for d in DIRECTIONS
    }


def _flat_state() -> dict:
    return fresh_device_state()


# ---------------------------------------------------------------------------
# 1. classify_distance
# ---------------------------------------------------------------------------

class TestClassifyDistance:

    def test_critical_lower_bound(self):
        assert classify_distance(0.0) == Zone.CRITICAL

    def test_critical_just_below_threshold(self):
        assert classify_distance(0.49) == Zone.CRITICAL

    def test_danger_at_boundary(self):
        # 0.5 m is the first value that is NOT CRITICAL
        assert classify_distance(0.5) == Zone.DANGER

    def test_danger_mid(self):
        assert classify_distance(1.0) == Zone.DANGER

    def test_warning_at_boundary(self):
        assert classify_distance(1.5) == Zone.WARNING

    def test_warning_mid(self):
        assert classify_distance(2.5) == Zone.WARNING

    def test_caution_at_boundary(self):
        assert classify_distance(3.0) == Zone.CAUTION

    def test_caution_mid(self):
        assert classify_distance(4.0) == Zone.CAUTION

    def test_alert_at_boundary(self):
        assert classify_distance(5.0) == Zone.ALERT

    def test_alert_mid(self):
        assert classify_distance(8.5) == Zone.ALERT

    def test_clear_at_mmwave_max(self):
        assert classify_distance(12.0) == Zone.CLEAR

    def test_clear_beyond_max(self):
        assert classify_distance(50.0) == Zone.CLEAR

    # Module-specific: head has a WIDER CRITICAL threshold (0.6 m vs 0.5 m).
    # Head-height hazards (overhangs, signs) need earlier warning.
    def test_head_critical_tighter_threshold(self):
        # 0.55 m: DANGER for body (>= 0.5m) but CRITICAL for head (< 0.6m)
        assert classify_distance(0.55, module="body") == Zone.DANGER
        assert classify_distance(0.55, module="head") == Zone.CRITICAL

    def test_head_critical_just_below_06(self):
        assert classify_distance(0.59, module="head") == Zone.CRITICAL

    def test_head_danger_at_or_above_06(self):
        assert classify_distance(0.60, module="head") == Zone.DANGER


# ---------------------------------------------------------------------------
# 2. _parse_tof_sensors
# ---------------------------------------------------------------------------

class TestParseTofSensors:

    def test_all_directions_present(self):
        data = _all_tof(1000)
        result = _parse_tof_sensors(data)
        assert set(result.keys()) == set(DIRECTIONS)

    def test_valid_reading_converted_to_metres(self):
        data = {"front": {"distance_mm": 2000, "avg": 2000, "min": 1990,
                           "max": 2010, "count": 50}}
        result = _parse_tof_sensors(data)
        assert result["front"] == pytest.approx(2.0)

    def test_prefers_distance_mm_over_avg(self):
        # distance_mm is the most-recent reading; avg may lag behind
        data = {"front": {"distance_mm": 3000, "avg": 2500,
                           "min": 2480, "max": 2520, "count": 50}}
        result = _parse_tof_sensors(data)
        assert result["front"] == pytest.approx(3.0)

    def test_sentinel_value_returns_none(self):
        data = {"front": {"distance_mm": TOF_INVALID_MM, "avg": TOF_INVALID_MM}}
        result = _parse_tof_sensors(data)
        assert result["front"] is None

    def test_above_tof_max_range_clamped_to_none(self):
        # 6000 mm = 6 m > TOF_MAX_RANGE_M (5 m) → should be discarded
        data = {"front": {"distance_mm": 6000, "avg": 6000}}
        result = _parse_tof_sensors(data)
        assert result["front"] is None

    def test_at_tof_max_range_accepted(self):
        # exactly 5 m = 5000 mm → 5.0 m == TOF_MAX_RANGE_M is still accepted
        data = {"front": {"distance_mm": 5000, "avg": 5000}}
        result = _parse_tof_sensors(data)
        assert result["front"] == pytest.approx(5.0)

    def test_missing_direction_returns_none(self):
        result = _parse_tof_sensors({})
        for d in DIRECTIONS:
            assert result[d] is None

    def test_zero_or_negative_distance_returns_none(self):
        data = {"front": {"distance_mm": 0, "avg": 0}}
        result = _parse_tof_sensors(data)
        assert result["front"] is None

    def test_partial_directions(self):
        data = {"front": {"distance_mm": 800, "avg": 800}}
        result = _parse_tof_sensors(data)
        assert result["front"] == pytest.approx(0.8)
        for d in DIRECTIONS:
            if d != "front":
                assert result[d] is None


# ---------------------------------------------------------------------------
# 3. _parse_mmwave_sensors
# ---------------------------------------------------------------------------

class TestParseMmwaveSensors:

    def test_no_targets_returns_none(self):
        data = {
            "front": {"presence": False, "det_state": 0},
            "back":  {"presence": False, "det_state": 0},
        }
        result = _parse_mmwave_sensors(data)
        assert result["front"] is None
        assert result["back"] is None

    def test_valid_target_parsed(self):
        data = {"front": {"presence": True, "det_state": 2,
                           "dist_cm": 750, "energy": 10576, "frames": 4},
                "back":  {"presence": False, "det_state": 0}}
        result = _parse_mmwave_sensors(data)
        assert result["front"] is not None
        assert result["front"]["range_m"] == pytest.approx(7.5)
        assert result["back"] is None

    def test_beyond_max_range_returns_none(self):
        dist_cm = int((MMWAVE_MAX_RANGE_M + 1.0) * 100)
        data = {"front": {"presence": True, "det_state": 2,
                           "dist_cm": dist_cm, "energy": 100, "frames": 1},
                "back":  {"presence": False, "det_state": 0}}
        result = _parse_mmwave_sensors(data)
        assert result["front"] is None

    def test_at_mmwave_max_range_accepted(self):
        dist_cm = int(MMWAVE_MAX_RANGE_M * 100)
        data = {"front": {"presence": True, "det_state": 2,
                           "dist_cm": dist_cm, "energy": 100, "frames": 1},
                "back":  {"presence": False, "det_state": 0}}
        result = _parse_mmwave_sensors(data)
        assert result["front"] is not None
        assert result["front"]["range_m"] == pytest.approx(MMWAVE_MAX_RANGE_M)

    def test_missing_data_returns_none(self):
        result = _parse_mmwave_sensors({})
        assert result["front"] is None
        assert result["back"] is None

    def test_both_sensors_active(self):
        data = {
            "front": {"presence": True, "det_state": 2,
                       "dist_cm": 600, "energy": 8000, "frames": 4},
            "back":  {"presence": True, "det_state": 2,
                       "dist_cm": 900, "energy": 3000, "frames": 2},
        }
        result = _parse_mmwave_sensors(data)
        assert result["front"]["range_m"] == pytest.approx(6.0)
        assert result["back"]["range_m"]  == pytest.approx(9.0)


# ---------------------------------------------------------------------------
# 4. _parse_imu
# ---------------------------------------------------------------------------

class TestParseImu:

    def test_all_fields_parsed(self):
        data = {"roll_deg": 5.0, "pitch_deg": -3.0, "yaw_deg": 90.0,
                "accel_x": 0.1, "accel_y": 0.2, "accel_z": 9.5}
        result = _parse_imu(data)
        assert result["roll_deg"]  == pytest.approx(5.0)
        assert result["pitch_deg"] == pytest.approx(-3.0)
        assert result["yaw_deg"]   == pytest.approx(90.0)
        assert result["accel_z"]   == pytest.approx(9.5)

    def test_defaults_on_empty(self):
        result = _parse_imu({})
        assert result["roll_deg"]  == pytest.approx(0.0)
        assert result["pitch_deg"] == pytest.approx(0.0)
        assert result["yaw_deg"]   == pytest.approx(0.0)
        assert result["accel_z"]   == pytest.approx(9.81)

    def test_partial_fields(self):
        result = _parse_imu({"pitch_deg": 10.0})
        assert result["pitch_deg"] == pytest.approx(10.0)
        assert result["roll_deg"]  == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# 5. _fuse_sensor_data – hybrid fusion scenarios
# ---------------------------------------------------------------------------

class TestFuseSensorData:

    def test_all_clear_when_no_sensors(self):
        fused = _fuse_sensor_data(_empty_tof(), _empty_mmwave())
        for d in DIRECTIONS:
            assert fused[d]["zone"]       == Zone.CLEAR
            assert fused[d]["distance_m"] is None
            assert fused[d]["source"]     == "none"

    # ── ToF-only scenarios ────────────────────────────────────────────────

    def test_tof_only_close_obstacle(self):
        tof = _empty_tof()
        tof["front"] = 0.3  # 30 cm
        fused = _fuse_sensor_data(tof, _empty_mmwave())
        assert fused["front"]["zone"]       == Zone.CRITICAL
        assert fused["front"]["source"]     == "tof"
        assert fused["front"]["distance_m"] == pytest.approx(0.3)

    def test_tof_only_warning_range(self):
        tof = _empty_tof()
        tof["right"] = 2.0
        fused = _fuse_sensor_data(tof, _empty_mmwave())
        assert fused["right"]["zone"]   == Zone.WARNING
        assert fused["right"]["source"] == "tof"

    def test_left_right_not_covered_by_mmwave(self):
        # left/right are outside mmWave coverage cones; only ToF applies
        tof = _empty_tof()
        mmwave = _make_mmwave("front", 7.0)
        fused = _fuse_sensor_data(tof, mmwave)
        assert fused["left"]["zone"]  == Zone.CLEAR  # no ToF, no mmWave coverage
        assert fused["right"]["zone"] == Zone.CLEAR

    # ── mmWave-only scenarios (ToF clear, beyond 5 m) ─────────────────────

    def test_mmwave_only_extends_beyond_tof(self):
        """mmWave fills in ALERT zone when ToF shows clear (> 5 m range)."""
        fused = _fuse_sensor_data(_empty_tof(), _make_mmwave("front", 7.5))
        # All 3 front-cone directions should be ALERT
        for d in MMWAVE_COVERAGE["front"]:
            assert fused[d]["zone"]   == Zone.ALERT, f"expected ALERT for {d}"
            assert fused[d]["source"] == "mmwave"
            assert fused[d]["distance_m"] == pytest.approx(7.5)
        # Back and side directions remain clear
        assert fused["back"]["zone"]  == Zone.CLEAR
        assert fused["left"]["zone"]  == Zone.CLEAR
        assert fused["right"]["zone"] == Zone.CLEAR

    def test_back_mmwave_fills_back_cone(self):
        fused = _fuse_sensor_data(_empty_tof(), _make_mmwave("back", 9.0))
        for d in MMWAVE_COVERAGE["back"]:
            assert fused[d]["zone"]   == Zone.ALERT, f"expected ALERT for {d}"
            assert fused[d]["source"] == "mmwave"
        # Front and side directions remain clear
        assert fused["front"]["zone"] == Zone.CLEAR
        assert fused["left"]["zone"]  == Zone.CLEAR

    def test_mmwave_within_tof_range_acts_as_tof_backup(self):
        """mmWave at 3 m while ToF shows nothing → mmWave distance used."""
        fused = _fuse_sensor_data(_empty_tof(), _make_mmwave("front", 3.0))
        assert fused["front"]["zone"]       == Zone.CAUTION  # 3.0 < 5.0
        assert fused["front"]["source"]     == "mmwave"
        assert fused["front"]["distance_m"] == pytest.approx(3.0)

    # ── Hybrid fusion: both sensors active ───────────────────────────────

    def test_hybrid_tof_closer_wins(self):
        """When ToF is closer than mmWave, ToF distance is used."""
        tof = _empty_tof()
        tof["front"] = 1.0          # ToF: 1 m  (DANGER)
        mmwave = _make_mmwave("front", 6.0)  # mmWave: 6 m (ALERT)
        fused = _fuse_sensor_data(tof, mmwave)
        assert fused["front"]["zone"]       == Zone.DANGER
        assert fused["front"]["distance_m"] == pytest.approx(1.0)
        assert fused["front"]["source"]     == "tof+mmwave"

    def test_hybrid_mmwave_closer_wins(self):
        """When mmWave reports a shorter distance than ToF, mmWave wins."""
        tof = _empty_tof()
        tof["front"] = 4.5          # ToF: 4.5 m (CAUTION)
        mmwave = _make_mmwave("front", 2.5)  # mmWave: 2.5 m (WARNING)
        fused = _fuse_sensor_data(tof, mmwave)
        assert fused["front"]["zone"]       == Zone.WARNING
        assert fused["front"]["distance_m"] == pytest.approx(2.5)
        assert fused["front"]["source"]     == "tof+mmwave"

    def test_hybrid_equal_distances(self):
        """Equal ToF and mmWave distances → same effective value."""
        tof = _empty_tof()
        tof["front"] = 2.0
        mmwave = _make_mmwave("front", 2.0)
        fused = _fuse_sensor_data(tof, mmwave)
        assert fused["front"]["distance_m"] == pytest.approx(2.0)
        assert fused["front"]["source"]     == "tof+mmwave"

    def test_vibration_pattern_matches_zone(self):
        tof = _empty_tof()
        tof["front"] = 0.4
        fused = _fuse_sensor_data(tof, _empty_mmwave(), module="body")
        vib = fused["front"]["vibration"]
        assert vib["intensity"]    == VIBRATION_PATTERNS[Zone.CRITICAL]["intensity"]
        assert vib["frequency_hz"] == VIBRATION_PATTERNS[Zone.CRITICAL]["frequency_hz"]
        assert vib["active"]       is True

    def test_clear_vibration_inactive(self):
        fused = _fuse_sensor_data(_empty_tof(), _empty_mmwave())
        vib = fused["front"]["vibration"]
        assert vib["active"]    is False
        assert vib["intensity"] == 0

    # ── Module-specific head thresholds ──────────────────────────────────

    def test_head_module_tighter_critical(self):
        """0.55 m = CRITICAL for head (< 0.6 m), DANGER for body (>= 0.5 m)."""
        tof = _empty_tof()
        tof["front"] = 0.55
        fused_head = _fuse_sensor_data(tof, _empty_mmwave(), module="head")
        fused_body = _fuse_sensor_data(tof, _empty_mmwave(), module="body")
        assert fused_head["front"]["zone"] == Zone.CRITICAL
        assert fused_body["front"]["zone"] == Zone.DANGER

    # ── 8-direction completeness ──────────────────────────────────────────

    def test_all_8_directions_present_in_fused(self):
        fused = _fuse_sensor_data(_empty_tof(), _empty_mmwave())
        assert set(fused.keys()) == set(DIRECTIONS)


# ---------------------------------------------------------------------------
# 6. _apply_imu_compensation
# ---------------------------------------------------------------------------

class TestApplyImuCompensation:

    def _imu(self, roll=0.0, pitch=0.0, yaw=0.0):
        return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
                "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81}

    def _fused_with_obstacle(self, direction: str, dist_m: float) -> dict:
        fused = _all_clear_fused()
        zone    = Zone.WARNING
        fused[direction].update({
            "distance_m": dist_m,
            "source":     "tof",
            "zone":       zone,
            "vibration":  VIBRATION_PATTERNS[zone].copy(),
        })
        return fused

    def test_no_tilt_no_change(self):
        fused = self._fused_with_obstacle("front", 2.0)
        result = _apply_imu_compensation(fused, self._imu(pitch=5.0))
        assert result["front"]["zone"] == Zone.WARNING  # not suppressed

    def test_pitch_tilt_suppresses_front(self):
        fused = self._fused_with_obstacle("front", 2.0)
        result = _apply_imu_compensation(fused, self._imu(pitch=TILT_THRESHOLD_DEG + 1))
        assert result["front"]["zone"]       == Zone.CLEAR
        assert result["front"]["distance_m"] is None
        assert "suppressed" in result["front"]["source"]

    def test_pitch_tilt_suppresses_back(self):
        fused = self._fused_with_obstacle("back", 1.0)
        result = _apply_imu_compensation(fused, self._imu(pitch=-(TILT_THRESHOLD_DEG + 5)))
        assert result["back"]["zone"] == Zone.CLEAR

    def test_pitch_tilt_does_not_suppress_sides(self):
        fused = self._fused_with_obstacle("left", 2.0)
        result = _apply_imu_compensation(fused, self._imu(pitch=TILT_THRESHOLD_DEG + 1))
        assert result["left"]["zone"] == Zone.WARNING  # left should be unaffected

    def test_roll_tilt_suppresses_left_right(self):
        fused = self._fused_with_obstacle("left", 2.0)
        result = _apply_imu_compensation(fused, self._imu(roll=TILT_THRESHOLD_DEG + 1))
        assert result["left"]["zone"]  == Zone.CLEAR
        assert result["right"]["zone"] == Zone.CLEAR

    def test_roll_tilt_does_not_suppress_front(self):
        fused = self._fused_with_obstacle("front", 2.0)
        result = _apply_imu_compensation(fused, self._imu(roll=TILT_THRESHOLD_DEG + 1))
        assert result["front"]["zone"] == Zone.WARNING

    def test_exact_threshold_not_suppressed(self):
        """At exactly the threshold angle suppression should NOT trigger."""
        fused = self._fused_with_obstacle("front", 2.0)
        result = _apply_imu_compensation(fused, self._imu(pitch=TILT_THRESHOLD_DEG))
        # uses strict >, so == threshold should pass through
        assert result["front"]["zone"] == Zone.WARNING

    def test_input_fused_not_mutated(self):
        """_apply_imu_compensation must not modify the input dict."""
        fused = self._fused_with_obstacle("front", 2.0)
        original_zone = fused["front"]["zone"]
        _apply_imu_compensation(fused, self._imu(pitch=TILT_THRESHOLD_DEG + 5))
        assert fused["front"]["zone"] == original_zone  # unchanged


# ---------------------------------------------------------------------------
# 7. _generate_motor_commands
# ---------------------------------------------------------------------------

class TestGenerateMotorCommands:

    def test_returns_8_commands(self):
        fused = _all_clear_fused()
        cmds = _generate_motor_commands(fused)
        assert len(cmds) == NUM_DIRECTIONS  # must be exactly 8

    def test_motor_ids_are_0_to_7(self):
        fused = _all_clear_fused()
        cmds = _generate_motor_commands(fused)
        ids = [c["motor_id"] for c in cmds]
        assert ids == list(range(NUM_DIRECTIONS))

    def test_direction_matches_index(self):
        fused = _all_clear_fused()
        cmds = _generate_motor_commands(fused)
        for i, d in enumerate(DIRECTIONS):
            assert cmds[i]["direction"] == d

    def test_clear_motor_is_inactive(self):
        fused = _all_clear_fused()
        cmds = _generate_motor_commands(fused)
        for cmd in cmds:
            assert cmd["active"]    is False
            assert cmd["intensity"] == 0

    def test_critical_motor_max_intensity(self):
        fused = _all_clear_fused()
        fused["front"].update({
            "distance_m": 0.2,
            "source":     "tof",
            "zone":       Zone.CRITICAL,
            "vibration":  VIBRATION_PATTERNS[Zone.CRITICAL].copy(),
        })
        cmds = _generate_motor_commands(fused)
        front_cmd = next(c for c in cmds if c["direction"] == "front")
        assert front_cmd["intensity"]    == 255
        assert front_cmd["frequency_hz"] == 50
        assert front_cmd["pattern"]      == "continuous"
        assert front_cmd["active"]       is True

    def test_distance_rounded_to_3_decimal_places(self):
        fused = _all_clear_fused()
        fused["front"].update({
            "distance_m": 1.23456789,
            "source":     "tof",
            "zone":       Zone.WARNING,
            "vibration":  VIBRATION_PATTERNS[Zone.WARNING].copy(),
        })
        cmds = _generate_motor_commands(fused)
        front_cmd = next(c for c in cmds if c["direction"] == "front")
        assert front_cmd["distance_m"] == pytest.approx(1.235)

    def test_none_distance_remains_none(self):
        fused = _all_clear_fused()
        cmds = _generate_motor_commands(fused)
        for cmd in cmds:
            assert cmd["distance_m"] is None


# ---------------------------------------------------------------------------
# 8. _compute_hazard
# ---------------------------------------------------------------------------

class TestComputeHazard:

    def test_all_clear_level_zero(self):
        fused = _all_clear_fused()
        hazard = _compute_hazard(fused)
        assert hazard["level"]  == 0
        assert hazard["label"]  == Zone.CLEAR
        assert hazard["alerts"] == []

    def test_single_critical_sets_level_5(self):
        fused = _all_clear_fused()
        fused["front"].update({
            "distance_m": 0.2,
            "source":     "tof",
            "zone":       Zone.CRITICAL,
            "vibration":  VIBRATION_PATTERNS[Zone.CRITICAL].copy(),
        })
        hazard = _compute_hazard(fused)
        assert hazard["level"] == 5
        assert hazard["label"] == Zone.CRITICAL
        assert len(hazard["alerts"]) == 1
        assert hazard["alerts"][0]["direction"] == "front"

    def test_worst_zone_wins(self):
        fused = _all_clear_fused()
        fused["front"].update({
            "distance_m": 2.0, "source": "tof",
            "zone": Zone.WARNING,
            "vibration": VIBRATION_PATTERNS[Zone.WARNING].copy(),
        })
        fused["right"].update({
            "distance_m": 0.3, "source": "tof",
            "zone": Zone.CRITICAL,
            "vibration": VIBRATION_PATTERNS[Zone.CRITICAL].copy(),
        })
        hazard = _compute_hazard(fused)
        assert hazard["level"] == HAZARD_LEVEL[Zone.CRITICAL]
        assert hazard["label"] == Zone.CRITICAL

    def test_alerts_sorted_most_critical_first(self):
        fused = _all_clear_fused()
        fused["front"].update({
            "distance_m": 4.0, "source": "tof",
            "zone": Zone.CAUTION,
            "vibration": VIBRATION_PATTERNS[Zone.CAUTION].copy(),
        })
        fused["right"].update({
            "distance_m": 0.4, "source": "tof",
            "zone": Zone.CRITICAL,
            "vibration": VIBRATION_PATTERNS[Zone.CRITICAL].copy(),
        })
        fused["back"].update({
            "distance_m": 2.0, "source": "tof",
            "zone": Zone.WARNING,
            "vibration": VIBRATION_PATTERNS[Zone.WARNING].copy(),
        })
        hazard = _compute_hazard(fused)
        zones = [a["zone"] for a in hazard["alerts"]]
        assert zones[0] == Zone.CRITICAL   # most severe first
        assert zones[-1] == Zone.CAUTION   # least severe last

    def test_alert_count_excludes_clear_directions(self):
        fused = _all_clear_fused()
        # Only 2 directions active
        for d in ("front", "left"):
            fused[d].update({
                "distance_m": 1.0, "source": "tof",
                "zone": Zone.DANGER,
                "vibration": VIBRATION_PATTERNS[Zone.DANGER].copy(),
            })
        hazard = _compute_hazard(fused)
        assert len(hazard["alerts"]) == 2


# ---------------------------------------------------------------------------
# 9. process_packet – end-to-end
# ---------------------------------------------------------------------------

def _make_full_packet(module: str = "body", **overrides) -> dict:
    """Build a minimal but complete sensor packet."""
    pkt = {
        "device_id":  f"esp32_{module}",
        "module":     module,
        "tof_sensors": {
            d: {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50}
            for d in DIRECTIONS
        },
        "mmwave_sensors": {
            "front": {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0},
            "back":  {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0},
        },
        "imu": {
            "roll_deg": 0.0, "pitch_deg": 0.0, "yaw_deg": 0.0,
            "accel_x": 0.0, "accel_y": 0.0,   "accel_z": 9.81,
        },
        "timestamp": 1000,
    }
    pkt.update(overrides)
    return pkt


class TestProcessPacket:

    def test_returns_required_keys(self):
        result = process_packet(_make_full_packet(), _flat_state(), 1100.0)
        for key in ("module", "hazard", "motors", "imu", "latency_ms"):
            assert key in result

    def test_8_motors_returned(self):
        result = process_packet(_make_full_packet(), _flat_state(), 1100.0)
        assert len(result["motors"]) == NUM_DIRECTIONS

    def test_all_clear_packet_level_zero(self):
        result = process_packet(_make_full_packet(), _flat_state(), 1100.0)
        assert result["hazard"]["level"] == 0

    def test_head_module_identified(self):
        result = process_packet(_make_full_packet(module="head"), _flat_state(), 1100.0)
        assert result["module"] == "head"

    def test_body_module_identified(self):
        result = process_packet(_make_full_packet(module="body"), _flat_state(), 1100.0)
        assert result["module"] == "body"

    def test_invalid_module_defaults_to_body(self):
        pkt = _make_full_packet(module="torso")  # unknown value
        result = process_packet(pkt, _flat_state(), 1100.0)
        assert result["module"] == "body"

    def test_critical_front_obstacle(self):
        pkt = _make_full_packet()
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 300, "avg": 300, "min": 290, "max": 310, "count": 50
        }
        result = process_packet(pkt, _flat_state(), 1100.0)
        assert result["hazard"]["label"] == Zone.CRITICAL
        motor_front = next(m for m in result["motors"] if m["direction"] == "front")
        assert motor_front["zone"]      == Zone.CRITICAL
        assert motor_front["intensity"] == 255
        assert motor_front["active"]    is True

    def test_mmwave_alert_beyond_tof(self):
        pkt = _make_full_packet()
        pkt["mmwave_sensors"] = _make_mmwave_raw("front", dist_cm=800, energy=6000)
        result = process_packet(pkt, _flat_state(), 1100.0)
        # The three front-cone directions should be ALERT
        for d in MMWAVE_COVERAGE["front"]:
            motor = next(m for m in result["motors"] if m["direction"] == d)
            assert motor["zone"] == Zone.ALERT, f"Expected ALERT for {d}"

    def test_tof_takes_priority_over_mmwave(self):
        pkt = _make_full_packet()
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 500, "avg": 500, "min": 490, "max": 510, "count": 50
        }
        pkt["mmwave_sensors"] = _make_mmwave_raw("front", dist_cm=800, energy=6000)
        result = process_packet(pkt, _flat_state(), 1100.0)
        motor_front = next(m for m in result["motors"] if m["direction"] == "front")
        # ToF reads 0.5 m (DANGER); mmWave reads 8 m (ALERT) – min = 0.5 m wins
        assert motor_front["zone"]       == Zone.DANGER
        assert motor_front["distance_m"] == pytest.approx(0.5)
        assert motor_front["source"]     == "tof+mmwave"

    def test_imu_tilt_suppresses_channels(self):
        pkt = _make_full_packet()
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 800, "avg": 800, "min": 780, "max": 820, "count": 50
        }
        pkt["imu"]["pitch_deg"] = TILT_THRESHOLD_DEG + 5.0  # strong pitch tilt
        result = process_packet(pkt, _flat_state(), 1100.0)
        motor_front = next(m for m in result["motors"] if m["direction"] == "front")
        assert motor_front["zone"]   == Zone.CLEAR
        assert motor_front["active"] is False

    def test_state_updated_after_packet(self):
        state = _flat_state()
        assert state["packet_count"] == 0
        process_packet(_make_full_packet(), state, 1100.0)
        assert state["packet_count"] == 1
        process_packet(_make_full_packet(), state, 1200.0)
        assert state["packet_count"] == 2

    def test_state_module_recorded(self):
        state = _flat_state()
        process_packet(_make_full_packet(module="head"), state, 1100.0)
        assert state["module"] == "head"

    def test_latency_calculation(self):
        pkt = _make_full_packet()
        pkt["timestamp"] = 1000
        result = process_packet(pkt, _flat_state(), 1100.0)  # 100 ms later
        assert result["latency_ms"] == pytest.approx(100.0, abs=1.0)


# ---------------------------------------------------------------------------
# 10. fresh_device_state
# ---------------------------------------------------------------------------

class TestFreshDeviceState:

    def test_required_keys_present(self):
        state = fresh_device_state()
        for key in ("last_seen_ms", "packet_count", "last_zone", "module"):
            assert key in state

    def test_initial_packet_count_zero(self):
        assert fresh_device_state()["packet_count"] == 0

    def test_initial_zone_clear(self):
        assert fresh_device_state()["last_zone"] == Zone.CLEAR

    def test_two_instances_are_independent(self):
        s1 = fresh_device_state()
        s2 = fresh_device_state()
        s1["packet_count"] = 99
        assert s2["packet_count"] == 0
