"""
Integration tests for server.py  |  ECE 445 Group 10
=====================================================

Tests the full HTTP request → algorithm → JSON response pipeline using
Flask's built-in test client.  No real network is required.

Groups
------
  1. POST /nav – structural / schema validation
  2. POST /nav – head module scenarios
  3. POST /nav – body module scenarios
  4. POST /nav – hybrid sensor fusion over HTTP
  5. POST /nav – IMU compensation over HTTP
  6. POST /nav – edge-cases & error handling
  7. GET  /status
"""

import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import json
import pytest

from server import app, device_state
from algorithm import (
    DIRECTIONS,
    NUM_DIRECTIONS,
    MMWAVE_COVERAGE,
    TILT_THRESHOLD_DEG,
    Zone,
)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def client():
    """Flask test client with a clean device-state between tests."""
    app.config["TESTING"] = True
    device_state.clear()
    with app.test_client() as c:
        yield c


# ---------------------------------------------------------------------------
# Packet builders
# ---------------------------------------------------------------------------

def _all_tof_clear() -> dict:
    return {
        d: {"distance_mm": 8190, "avg": 8190, "min": 8190, "max": 8190, "count": 50}
        for d in DIRECTIONS
    }


def _all_mmwave_clear() -> dict:
    return {
        "front": {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0},
        "back":  {"targets": 0, "range_m": 0.0, "speed_ms": 0.0, "energy": 0},
    }


def _flat_imu(roll=0.0, pitch=0.0, yaw=0.0) -> dict:
    return {"roll_deg": roll, "pitch_deg": pitch, "yaw_deg": yaw,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81}


def _base_packet(module: str = "body", timestamp: int = 1000) -> dict:
    """Minimal valid packet – all sensors clear."""
    return {
        "device_id":      f"esp32_{module}",
        "module":         module,
        "tof_sensors":    _all_tof_clear(),
        "mmwave_sensors": _all_mmwave_clear(),
        "imu":            _flat_imu(),
        "timestamp":      timestamp,
    }


def _post_nav(client, payload: dict):
    return client.post(
        "/nav",
        data=json.dumps(payload),
        content_type="application/json",
    )


# ---------------------------------------------------------------------------
# 1. POST /nav – structural / schema validation
# ---------------------------------------------------------------------------

class TestNavStructure:

    def test_returns_200_on_valid_packet(self, client):
        resp = _post_nav(client, _base_packet())
        assert resp.status_code == 200

    def test_status_ok_in_response(self, client):
        data = _post_nav(client, _base_packet()).get_json()
        assert data["status"] == "ok"

    def test_device_id_echoed(self, client):
        data = _post_nav(client, _base_packet("head")).get_json()
        assert data["device_id"] == "esp32_head"

    def test_module_field_present(self, client):
        data = _post_nav(client, _base_packet("body")).get_json()
        assert "module" in data

    def test_hazard_block_present(self, client):
        data = _post_nav(client, _base_packet()).get_json()
        hazard = data["hazard"]
        assert "level"  in hazard
        assert "label"  in hazard
        assert "alerts" in hazard

    def test_motors_block_is_list_of_8(self, client):
        data = _post_nav(client, _base_packet()).get_json()
        assert isinstance(data["motors"], list)
        assert len(data["motors"]) == NUM_DIRECTIONS

    def test_each_motor_has_required_fields(self, client):
        data = _post_nav(client, _base_packet()).get_json()
        required = {"motor_id", "direction", "intensity", "frequency_hz",
                    "pattern", "active", "zone", "distance_m", "source"}
        for motor in data["motors"]:
            assert required <= set(motor.keys()), f"Motor {motor['motor_id']} missing keys"

    def test_motor_ids_are_0_to_7(self, client):
        data = _post_nav(client, _base_packet()).get_json()
        ids = [m["motor_id"] for m in data["motors"]]
        assert ids == list(range(NUM_DIRECTIONS))

    def test_imu_block_present(self, client):
        data = _post_nav(client, _base_packet()).get_json()
        imu = data["imu"]
        assert "roll_deg"  in imu
        assert "pitch_deg" in imu
        assert "yaw_deg"   in imu

    def test_latency_ms_present(self, client):
        data = _post_nav(client, _base_packet()).get_json()
        assert "latency_ms" in data

    def test_missing_json_returns_400(self, client):
        resp = client.post("/nav", data="not json", content_type="text/plain")
        assert resp.status_code == 400

    def test_empty_json_object_is_accepted(self, client):
        # {} is valid JSON – all sensors absent means all CLEAR
        resp = _post_nav(client, {})
        assert resp.status_code == 200
        assert resp.get_json()["status"] == "ok"

    def test_empty_body_returns_400(self, client):
        resp = client.post("/nav", data="", content_type="application/json")
        assert resp.status_code == 400


# ---------------------------------------------------------------------------
# 2. POST /nav – head module
# ---------------------------------------------------------------------------

class TestNavHeadModule:

    def test_module_is_head(self, client):
        data = _post_nav(client, _base_packet("head")).get_json()
        assert data["module"] == "head"

    def test_head_all_clear(self, client):
        data = _post_nav(client, _base_packet("head")).get_json()
        assert data["hazard"]["level"] == 0

    def test_head_critical_obstacle_front(self, client):
        pkt = _base_packet("head")
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 350, "avg": 350, "min": 340, "max": 360, "count": 50
        }
        data = _post_nav(client, pkt).get_json()
        assert data["hazard"]["label"] == Zone.CRITICAL
        m = next(m for m in data["motors"] if m["direction"] == "front")
        assert m["zone"]      == Zone.CRITICAL
        assert m["intensity"] == 255
        assert m["active"]    is True

    def test_head_tighter_critical_threshold(self, client):
        """0.55 m → CRITICAL for head (< 0.6 m threshold) but DANGER for body (>= 0.5 m)."""
        pkt_head = _base_packet("head")
        pkt_body = _base_packet("body")
        for pkt in (pkt_head, pkt_body):
            pkt["tof_sensors"]["front"] = {
                "distance_mm": 550, "avg": 550, "min": 540, "max": 560, "count": 50
            }

        data_head = _post_nav(client, pkt_head).get_json()
        data_body = _post_nav(client, pkt_body).get_json()

        m_head = next(m for m in data_head["motors"] if m["direction"] == "front")
        m_body = next(m for m in data_body["motors"] if m["direction"] == "front")

        assert m_head["zone"] == Zone.CRITICAL
        assert m_body["zone"] == Zone.DANGER

    def test_head_warning_side_obstacle(self, client):
        pkt = _base_packet("head")
        pkt["tof_sensors"]["right"] = {
            "distance_mm": 2200, "avg": 2200, "min": 2180, "max": 2220, "count": 50
        }
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "right")
        assert m["zone"] == Zone.WARNING

    def test_head_imu_echoed(self, client):
        pkt = _base_packet("head")
        pkt["imu"] = {"roll_deg": 3.5, "pitch_deg": -2.1, "yaw_deg": 90.0,
                      "accel_x": 0.0, "accel_y": 0.0, "accel_z": 9.81}
        data = _post_nav(client, pkt).get_json()
        assert data["imu"]["roll_deg"]  == pytest.approx(3.5)
        assert data["imu"]["pitch_deg"] == pytest.approx(-2.1)
        assert data["imu"]["yaw_deg"]   == pytest.approx(90.0)


# ---------------------------------------------------------------------------
# 3. POST /nav – body module
# ---------------------------------------------------------------------------

class TestNavBodyModule:

    def test_module_is_body(self, client):
        data = _post_nav(client, _base_packet("body")).get_json()
        assert data["module"] == "body"

    def test_body_all_clear(self, client):
        data = _post_nav(client, _base_packet("body")).get_json()
        assert data["hazard"]["level"] == 0
        for m in data["motors"]:
            assert m["active"]    is False
            assert m["intensity"] == 0

    def test_body_danger_left(self, client):
        pkt = _base_packet("body")
        pkt["tof_sensors"]["left"] = {
            "distance_mm": 900, "avg": 900, "min": 880, "max": 920, "count": 50
        }
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "left")
        assert m["zone"]   == Zone.DANGER
        assert m["active"] is True

    def test_body_multiple_obstacles(self, client):
        pkt = _base_packet("body")
        pkt["tof_sensors"]["front"]      = {"distance_mm": 200, "avg": 200, "min": 190, "max": 210, "count": 50}
        pkt["tof_sensors"]["back_right"] = {"distance_mm": 1200, "avg": 1200, "min": 1190, "max": 1210, "count": 50}
        data = _post_nav(client, pkt).get_json()
        assert data["hazard"]["label"] == Zone.CRITICAL
        active_motors = [m for m in data["motors"] if m["active"]]
        assert len(active_motors) >= 2

    def test_invalid_module_name_defaults_to_body(self, client):
        pkt = _base_packet("body")
        pkt["module"] = "waist"   # not a recognized value
        data = _post_nav(client, pkt).get_json()
        assert data["module"] == "body"


# ---------------------------------------------------------------------------
# 4. POST /nav – hybrid sensor fusion over HTTP
# ---------------------------------------------------------------------------

class TestNavHybridFusion:

    def test_mmwave_only_detection_beyond_tof(self, client):
        """mmWave front at 8 m → front hemisphere motors get ALERT."""
        pkt = _base_packet("body")
        pkt["mmwave_sensors"]["front"] = {
            "targets": 1, "range_m": 8.0, "speed_ms": 0.1, "energy": 7000
        }
        data = _post_nav(client, pkt).get_json()
        for d in MMWAVE_COVERAGE["front"]:
            m = next(m for m in data["motors"] if m["direction"] == d)
            assert m["zone"]   == Zone.ALERT,  f"Expected ALERT for {d}"
            assert m["source"] == "mmwave"

    def test_mmwave_back_detection(self, client):
        pkt = _base_packet("body")
        pkt["mmwave_sensors"]["back"] = {
            "targets": 1, "range_m": 10.0, "speed_ms": 0.0, "energy": 4000
        }
        data = _post_nav(client, pkt).get_json()
        for d in MMWAVE_COVERAGE["back"]:
            m = next(m for m in data["motors"] if m["direction"] == d)
            assert m["zone"] == Zone.ALERT, f"Expected ALERT for {d}"

    def test_tof_and_mmwave_tof_closer(self, client):
        """ToF reads 1 m; mmWave reads 7 m for same direction → DANGER (min)."""
        pkt = _base_packet("body")
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 1000, "avg": 1000, "min": 990, "max": 1010, "count": 50
        }
        pkt["mmwave_sensors"]["front"] = {
            "targets": 1, "range_m": 7.0, "speed_ms": 0.05, "energy": 5000
        }
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "front")
        assert m["zone"]       == Zone.DANGER   # 1 m = DANGER wins over 7 m = ALERT
        assert m["distance_m"] == pytest.approx(1.0)
        assert m["source"]     == "tof+mmwave"

    def test_tof_and_mmwave_mmwave_closer(self, client):
        """ToF reads 4 m; mmWave reads 2 m → WARNING (min = mmWave)."""
        pkt = _base_packet("body")
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 4000, "avg": 4000, "min": 3990, "max": 4010, "count": 50
        }
        pkt["mmwave_sensors"]["front"] = {
            "targets": 1, "range_m": 2.0, "speed_ms": 0.1, "energy": 6000
        }
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "front")
        assert m["zone"]       == Zone.WARNING   # 2 m = WARNING
        assert m["distance_m"] == pytest.approx(2.0)

    def test_side_directions_no_mmwave_coverage(self, client):
        """Left and right motors must NOT be influenced by front/back mmWave."""
        pkt = _base_packet("body")
        pkt["mmwave_sensors"]["front"] = {
            "targets": 1, "range_m": 6.0, "speed_ms": 0.0, "energy": 3000
        }
        data = _post_nav(client, pkt).get_json()
        for direction in ("left", "right"):
            m = next(m for m in data["motors"] if m["direction"] == direction)
            assert m["zone"] == Zone.CLEAR, f"{direction} should be clear"

    def test_mmwave_at_tof_max_range_boundary(self, client):
        """mmWave at exactly 5 m → ALERT (CAUTION threshold is < 5.0 m, strict)."""
        pkt = _base_packet("body")
        pkt["mmwave_sensors"]["front"] = {
            "targets": 1, "range_m": 5.0, "speed_ms": 0.0, "energy": 2000
        }
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "front")
        # classify_distance(5.0): not < 5.0 → falls through to ALERT (< 12.0)
        assert m["zone"]   == Zone.ALERT
        assert m["active"] is True

    def test_both_mmwave_sensors_active(self, client):
        pkt = _base_packet("body")
        pkt["mmwave_sensors"]["front"] = {
            "targets": 1, "range_m": 7.0, "speed_ms": 0.1, "energy": 5000
        }
        pkt["mmwave_sensors"]["back"] = {
            "targets": 1, "range_m": 9.0, "speed_ms": 0.0, "energy": 3000
        }
        data = _post_nav(client, pkt).get_json()
        front_m = next(m for m in data["motors"] if m["direction"] == "front")
        back_m  = next(m for m in data["motors"] if m["direction"] == "back")
        assert front_m["zone"] == Zone.ALERT
        assert back_m["zone"]  == Zone.ALERT


# ---------------------------------------------------------------------------
# 5. POST /nav – IMU compensation over HTTP
# ---------------------------------------------------------------------------

class TestNavImuCompensation:

    def test_pitch_tilt_suppresses_front_reading(self, client):
        """A large pitch angle should suppress a close front obstacle."""
        pkt = _base_packet("body")
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 600, "avg": 600, "min": 590, "max": 610, "count": 50
        }
        pkt["imu"]["pitch_deg"] = TILT_THRESHOLD_DEG + 10.0
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "front")
        assert m["zone"]   == Zone.CLEAR
        assert m["active"] is False

    def test_pitch_tilt_suppresses_back_reading(self, client):
        pkt = _base_packet("body")
        pkt["tof_sensors"]["back"] = {
            "distance_mm": 1000, "avg": 1000, "min": 990, "max": 1010, "count": 50
        }
        pkt["imu"]["pitch_deg"] = -(TILT_THRESHOLD_DEG + 5.0)
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "back")
        assert m["zone"] == Zone.CLEAR

    def test_roll_tilt_suppresses_left_right(self, client):
        pkt = _base_packet("body")
        pkt["tof_sensors"]["left"]  = {"distance_mm": 700, "avg": 700, "min": 690, "max": 710, "count": 50}
        pkt["tof_sensors"]["right"] = {"distance_mm": 800, "avg": 800, "min": 790, "max": 810, "count": 50}
        pkt["imu"]["roll_deg"] = TILT_THRESHOLD_DEG + 8.0
        data = _post_nav(client, pkt).get_json()
        for direction in ("left", "right"):
            m = next(m for m in data["motors"] if m["direction"] == direction)
            assert m["zone"] == Zone.CLEAR, f"{direction} should be suppressed"

    def test_small_tilt_does_not_suppress(self, client):
        pkt = _base_packet("body")
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 800, "avg": 800, "min": 790, "max": 810, "count": 50
        }
        pkt["imu"]["pitch_deg"] = TILT_THRESHOLD_DEG - 3.0  # below threshold
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "front")
        assert m["zone"] != Zone.CLEAR   # should still warn

    def test_diagonal_directions_not_suppressed_by_pitch(self, client):
        """front_right / front_left are NOT suppressed by pitch (only front/back are)."""
        pkt = _base_packet("body")
        pkt["tof_sensors"]["front_right"] = {
            "distance_mm": 1000, "avg": 1000, "min": 990, "max": 1010, "count": 50
        }
        pkt["imu"]["pitch_deg"] = TILT_THRESHOLD_DEG + 10.0
        data = _post_nav(client, pkt).get_json()
        m = next(m for m in data["motors"] if m["direction"] == "front_right")
        assert m["zone"] != Zone.CLEAR   # diagonal should still be active


# ---------------------------------------------------------------------------
# 6. POST /nav – edge-cases & error handling
# ---------------------------------------------------------------------------

class TestNavEdgeCases:

    def test_empty_json_object_all_clear(self, client):
        # An empty JSON object is valid – all sensors absent → all CLEAR
        resp = _post_nav(client, {})
        assert resp.status_code == 200
        data = resp.get_json()
        assert data["hazard"]["level"] == 0

    def test_missing_module_defaults_to_body(self, client):
        pkt = _base_packet("body")
        del pkt["module"]
        data = _post_nav(client, pkt).get_json()
        assert data["module"] == "body"

    def test_missing_tof_sensors_all_clear(self, client):
        pkt = _base_packet()
        del pkt["tof_sensors"]
        data = _post_nav(client, pkt).get_json()
        assert data["hazard"]["level"] == 0

    def test_missing_mmwave_sensors_still_processes(self, client):
        pkt = _base_packet()
        del pkt["mmwave_sensors"]
        data = _post_nav(client, pkt).get_json()
        assert data["status"] == "ok"

    def test_missing_imu_still_processes(self, client):
        pkt = _base_packet()
        del pkt["imu"]
        data = _post_nav(client, pkt).get_json()
        assert data["status"] == "ok"

    def test_two_separate_devices_independent_state(self, client):
        """Packets from two devices must not share state."""
        pkt_head = _base_packet("head")
        pkt_head["tof_sensors"]["front"] = {
            "distance_mm": 200, "avg": 200, "min": 190, "max": 210, "count": 50
        }
        pkt_body = _base_packet("body")

        _post_nav(client, pkt_head)
        data_body = _post_nav(client, pkt_body).get_json()

        # body module should still be clear, unaffected by head packet
        assert data_body["hazard"]["level"] == 0

    def test_consecutive_packets_increment_server_state(self, client):
        for _ in range(3):
            _post_nav(client, _base_packet("head"))

        resp = client.get("/status")
        state_info = resp.get_json()["devices"].get("esp32_head", {})
        assert state_info.get("packet_count", 0) == 3

    def test_full_8_direction_obstacle_all_motors_active(self, client):
        """Every direction has an obstacle → all 8 motors active."""
        pkt = _base_packet("body")
        for d in DIRECTIONS:
            pkt["tof_sensors"][d] = {
                "distance_mm": 2000, "avg": 2000, "min": 1990, "max": 2010, "count": 50
            }
        data = _post_nav(client, pkt).get_json()
        for m in data["motors"]:
            assert m["active"] is True, f"Motor {m['direction']} should be active"


# ---------------------------------------------------------------------------
# 7. GET /status
# ---------------------------------------------------------------------------

class TestStatus:

    def test_status_returns_200(self, client):
        resp = client.get("/status")
        assert resp.status_code == 200

    def test_status_running(self, client):
        data = client.get("/status").get_json()
        assert data["status"] == "running"

    def test_devices_key_present(self, client):
        data = client.get("/status").get_json()
        assert "devices" in data

    def test_device_appears_after_nav_request(self, client):
        _post_nav(client, _base_packet("head"))
        data = client.get("/status").get_json()
        assert "esp32_head" in data["devices"]

    def test_device_state_shows_module_and_zone(self, client):
        pkt = _base_packet("head")
        pkt["tof_sensors"]["front"] = {
            "distance_mm": 400, "avg": 400, "min": 390, "max": 410, "count": 50
        }
        _post_nav(client, pkt)
        data  = client.get("/status").get_json()
        entry = data["devices"]["esp32_head"]
        assert entry["module"]       == "head"
        assert entry["packet_count"] == 1

    def test_multiple_devices_tracked(self, client):
        _post_nav(client, _base_packet("head"))
        _post_nav(client, _base_packet("body"))
        data = client.get("/status").get_json()
        assert "esp32_head" in data["devices"]
        assert "esp32_body" in data["devices"]
