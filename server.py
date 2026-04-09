"""
OmniSense-Dual  Flask Server  |  ECE 445 Group 10
==================================================

Thin HTTP layer: receives ESP32-S3 sensor packets from either the HEAD
or BODY module, delegates all processing to algorithm.py, and returns
per-motor haptic commands as JSON.

Endpoints
---------
  POST /nav     Ingest one sensor packet; returns hazard summary + 8 motor commands.
  GET  /status  Health-check; lists all connected device IDs and their state.

Request JSON schema (POST /nav)
--------------------------------
  {
    "device_id":  "esp32_head",
    "module":     "head",           ← REQUIRED: "head" or "body"
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
        "front": {"targets": 1, "range_m": 7.5,
                  "speed_ms": 0.11, "energy": 10576},
        "back":  {"targets": 0, "range_m": 0.0,
                  "speed_ms": 0.0,  "energy": 0}
    },
    "imu": {
        "roll_deg": 2.5, "pitch_deg": -1.2, "yaw_deg": 45.0,
        "accel_x": 0.01, "accel_y": 0.02,  "accel_z": 9.8
    },
    "timestamp": 37393
  }

Response JSON schema
---------------------
  {
    "status":     "ok",
    "device_id":  "esp32_head",
    "module":     "head",
    "hazard":     {"level": 3, "label": "WARNING", "alerts": [...]},
    "motors": [
        {"motor_id": 0, "direction": "front",      "intensity": 153,
         "frequency_hz": 10, "pattern": "medium_pulse",
         "active": true,  "zone": "WARNING",
         "distance_m": 2.3, "source": "tof"},
        ...   (8 entries total, one per direction)
    ],
    "imu":        {"roll_deg": 2.5, "pitch_deg": -1.2, "yaw_deg": 45.0},
    "latency_ms": 12.4
  }
"""

from flask import Flask, request, jsonify
from collections import defaultdict
import time

from algorithm import process_packet, print_decision, fresh_device_state

app = Flask(__name__)

# Per-device persistent state (keyed by device_id string, auto-initialised)
device_state: defaultdict = defaultdict(fresh_device_state)


@app.route("/nav", methods=["POST"])
def nav():
    data = request.get_json(silent=True)
    if data is None:
        return jsonify({"status": "error", "message": "invalid or missing JSON"}), 400

    device_id = data.get("device_id", "esp32")
    pkt_ts    = int(data.get("timestamp", 0))
    now       = time.time() * 1000.0

    result = process_packet(data, device_state[device_id], now)

    print_decision(
        result["hazard"]["alerts"],
        result["hazard"]["level"],
        device_id,
        pkt_ts,
    )

    return jsonify({"status": "ok", "device_id": device_id, **result})


@app.route("/status", methods=["GET"])
def status():
    active = {
        dev_id: {
            "module":       state.get("module", "unknown"),
            "last_zone":    state.get("last_zone", "CLEAR"),
            "packet_count": state.get("packet_count", 0),
            "last_seen_ms": state.get("last_seen_ms", 0),
        }
        for dev_id, state in device_state.items()
    }
    return jsonify({"status": "running", "devices": active})


if __name__ == "__main__":
    print("OmniSense-Dual server  http://0.0.0.0:5000")
    app.run(host="0.0.0.0", port=5000, debug=True)
