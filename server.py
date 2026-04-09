"""
OmniSense-Dual  Flask Server  |  ECE 445 Group 10
==================================================

Thin HTTP layer: receives ESP32-S3 sensor packets from either the HEAD
or BODY module, delegates all processing to algorithm.py, and returns
per-motor haptic commands as JSON.

Endpoints
---------
  POST /nav         Ingest one sensor packet; returns hazard summary + 8 motor commands.
  GET  /status      Health-check; lists all connected device IDs and their state.
  GET  /log_summary Per-device statistics snapshot for requirement verification.

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

Logging
-------
All log lines tagged [VERIFY:TAG] map directly to design-document requirements:

  TAG                Requirement verified
  ─────────────────  ────────────────────────────────────────────────────────
  PKT_RECV           2.2.1 R1 – packet timestamps for sampling-rate analysis
  PKT_RATE           2.2.1 R1 / 2.2.5 R1 – inter-packet rate ≥10 Hz check
  PROC_TIME          2.2.8 R2 – server processing latency < 500 ms
  LATENCY_WARN       2.2.1 R2 – end-to-end latency ≤ 200 ms budget check
  TIMEOUT_DETECT     2.2.5 R3 / 2.2.7 R2 – connection-loss detection ≤ 2 s
  MODULE_STALE       2.2.2 R4 – head/body packet exchange continuity ≥ 10 Hz
  MALFORMED_PKT      2.2.5 R2 – command bit-error-rate = 0 %
  HAZARD_ZONE_CHG    2.2.2 R1 – hazard state transition for detection-recall log

  (Additional tags IMU_READ, FUSE, HAZARD_DETECT, MOTOR_CMD are emitted by
   algorithm.py and share the same log file.)
"""

from __future__ import annotations

import logging
import logging.handlers
import os
import time
from collections import defaultdict
from typing import Any

from flask import Flask, request, jsonify
from flask_cors import CORS

from algorithm import process_combined_packet, print_decision, fresh_device_state

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

_LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")

# Thresholds for warning-level log triggers (mirrors design doc values)
_LATENCY_WARN_MS      = 200.0   # 2.2.1 R2 / 2.2.8 R2 combined budget
_PROC_TIME_WARN_MS    = 100.0   # server processing share of the 200 ms budget
_TIMEOUT_THRESHOLD_MS = 2000.0  # 2.2.5 R3 / 2.2.7 R2 – connection-loss window
_RATE_MIN_HZ          = 9.0     # 2.2.5 R1 – 10 Hz ± 10 % lower bound
_MODULE_STALE_MS      = 200.0   # 2.2.2 R4 – head/body exchange must be ≥ 10 Hz


def _setup_logging(log_dir: str = _LOG_DIR) -> None:
    """
    Configure two handlers on the root 'omnisense' logger:

      • Console  – INFO level, compact one-line format (terminal monitoring)
      • File     – DEBUG level, full timestamped format (offline analysis)

    The file is rotated at 10 MB and up to 10 backups are kept so a 5-minute
    continuous test at 10 Hz never fills disk.

    Call this once from ``if __name__ == "__main__"`` (or from tests that
    explicitly want file output).  During unit/integration testing the
    NullHandler set in algorithm.py keeps output silent unless this is called.
    """
    os.makedirs(log_dir, exist_ok=True)

    ts    = time.strftime("%Y%m%d_%H%M%S")
    fpath = os.path.join(log_dir, f"omnisense_{ts}.log")

    root = logging.getLogger("omnisense")
    root.setLevel(logging.DEBUG)

    # Remove any handlers already attached (idempotent re-init)
    root.handlers.clear()

    # ── Console handler ───────────────────────────────────────────────────────
    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)-8s %(message)s",
        datefmt="%H:%M:%S",
    ))
    root.addHandler(ch)

    # ── Rotating file handler ─────────────────────────────────────────────────
    fh = logging.handlers.RotatingFileHandler(
        fpath, maxBytes=10 * 1024 * 1024, backupCount=10, encoding="utf-8",
    )
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-8s %(name)s  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    root.addHandler(fh)

    logging.getLogger("omnisense.server").info(
        "[VERIFY:SESSION_START] log_file=%s latency_warn_ms=%.0f "
        "timeout_threshold_ms=%.0f rate_min_hz=%.1f",
        fpath, _LATENCY_WARN_MS, _TIMEOUT_THRESHOLD_MS, _RATE_MIN_HZ,
    )


logger = logging.getLogger("omnisense.server")

# ---------------------------------------------------------------------------
# Flask application
# ---------------------------------------------------------------------------

app = Flask(__name__)
CORS(app)  # Allow cross-origin requests from the frontend

# Per-device persistent state (keyed by device_id string, auto-initialised)
device_state: defaultdict = defaultdict(fresh_device_state)

# Latest raw packet from each module type; both are merged on every /nav call
latest_module_data: dict = {"head": {}, "body": {}}

# Persistent state for the combined dual-module fusion result
combined_state: dict = fresh_device_state()

# ---------------------------------------------------------------------------
# Per-device runtime statistics  (separate from device_state so tests that
# reset device_state don't lose timing data; cleared manually if needed)
# ---------------------------------------------------------------------------

#  _device_timing[device_id] = {
#      "last_recv_ms": float,   ← wall-clock ms of most recent packet arrival
#      "first_recv_ms": float,  ← wall-clock ms of very first packet
#      "interval_sum_ms": float,
#      "interval_count": int,
#      "interval_min_ms": float,
#      "interval_max_ms": float,
#  }
_device_timing: dict[str, dict[str, Any]] = {}

#  _device_stats[device_id] = {
#      "hazard_counts": {zone: int},   ← frequency each zone was the worst
#      "suppression_count": int,       ← tracked via IMU_SUPPRESS events (future)
#      "malformed_count": int,         ← bad JSON packets from this device
#      "proc_time_sum_ms": float,
#      "proc_time_count": int,
#      "proc_time_max_ms": float,
#  }
_device_stats: dict[str, dict[str, Any]] = {}

_global_malformed_count: int = 0


def _ensure_stats(device_id: str) -> None:
    """Lazily initialise per-device stats dicts."""
    if device_id not in _device_stats:
        _device_stats[device_id] = {
            "hazard_counts":    defaultdict(int),
            "suppression_count": 0,
            "malformed_count":   0,
            "proc_time_sum_ms":  0.0,
            "proc_time_count":   0,
            "proc_time_max_ms":  0.0,
        }


def _update_timing(device_id: str, now_ms: float) -> float | None:
    """
    Update inter-packet timing for a device.

    Returns the interval in ms since the last packet, or None for the first
    packet from this device.
    """
    if device_id not in _device_timing:
        _device_timing[device_id] = {
            "last_recv_ms":    now_ms,
            "first_recv_ms":   now_ms,
            "interval_sum_ms": 0.0,
            "interval_count":  0,
            "interval_min_ms": float("inf"),
            "interval_max_ms": 0.0,
        }
        return None

    t = _device_timing[device_id]
    interval_ms = now_ms - t["last_recv_ms"]
    t["last_recv_ms"]     = now_ms
    t["interval_sum_ms"] += interval_ms
    t["interval_count"]  += 1
    t["interval_min_ms"]  = min(t["interval_min_ms"], interval_ms)
    t["interval_max_ms"]  = max(t["interval_max_ms"], interval_ms)
    return interval_ms


# ---------------------------------------------------------------------------
# Routes
# ---------------------------------------------------------------------------

@app.route("/nav", methods=["POST"])
def nav() -> tuple:
    global _global_malformed_count

    # ── 0. Parse request ─────────────────────────────────────────────────────
    data = request.get_json(silent=True)
    if data is None:
        _global_malformed_count += 1
        # [VERIFY:MALFORMED_PKT] – contributes to 2.2.5 R2 bit-error-rate check
        logger.warning(
            "[VERIFY:MALFORMED_PKT] reason=invalid_or_missing_JSON "
            "total_malformed=%d",
            _global_malformed_count,
        )
        return jsonify({"status": "error", "message": "invalid or missing JSON"}), 400

    now_ms    = time.time() * 1000.0
    t_start   = time.perf_counter()

    device_id = str(data.get("device_id", "esp32"))
    pkt_ts    = int(data.get("timestamp", 0))
    module    = str(data.get("module", "body")).lower()
    if module not in ("head", "body"):
        module = "body"

    _ensure_stats(device_id)

    # ── 1. Packet-receive log  [VERIFY:PKT_RECV] ─────────────────────────────
    # Supports 2.2.1 R1 (sampling frequency / packet loss) and
    # 2.2.5 R1 (≥10 Hz packet rate) verification.
    pkt_count = device_state[device_id]["packet_count"] + 1  # pre-increment preview
    logger.info(
        "[VERIFY:PKT_RECV] device=%-14s module=%-4s pkt_ts=%8d "
        "server_ms=%.1f count=%d",
        device_id, module, pkt_ts, now_ms, pkt_count,
    )

    # ── 2. Inter-packet rate log  [VERIFY:PKT_RATE] ──────────────────────────
    # Supports 2.2.5 R1 (sensor packets ≥10 Hz) and
    # 2.2.2 R4 (head/body exchange ≥10 Hz) verification.
    interval_ms = _update_timing(device_id, now_ms)
    if interval_ms is not None:
        rate_hz = 1000.0 / interval_ms if interval_ms > 0 else float("inf")
        t       = _device_timing[device_id]
        avg_interval = (
            t["interval_sum_ms"] / t["interval_count"]
            if t["interval_count"] else interval_ms
        )
        avg_rate_hz = 1000.0 / avg_interval if avg_interval > 0 else float("inf")

        if rate_hz < _RATE_MIN_HZ:
            logger.warning(
                "[VERIFY:PKT_RATE] device=%-14s module=%-4s "
                "interval_ms=%.1f  rate_hz=%.2f  avg_rate_hz=%.2f  "
                "⚠ BELOW_MINIMUM (%.1f Hz)",
                device_id, module, interval_ms, rate_hz, avg_rate_hz, _RATE_MIN_HZ,
            )
        else:
            logger.info(
                "[VERIFY:PKT_RATE] device=%-14s module=%-4s "
                "interval_ms=%.1f  rate_hz=%.2f  avg_rate_hz=%.2f",
                device_id, module, interval_ms, rate_hz, avg_rate_hz,
            )

    # ── 3. Stale-module warning  [VERIFY:MODULE_STALE] ───────────────────────
    # Supports 2.2.2 R4: head and body must exchange at ≥10 Hz continuously.
    # Warn when the *other* module's last packet is older than _MODULE_STALE_MS.
    other_module = "body" if module == "head" else "head"
    other_data   = latest_module_data[other_module]
    if other_data:
        other_ts = other_data.get("_server_recv_ms", 0.0)
        other_idle_ms = now_ms - other_ts if other_ts else float("inf")
        if other_idle_ms > _MODULE_STALE_MS:
            logger.warning(
                "[VERIFY:MODULE_STALE] other_module=%-4s idle_ms=%.1f  "
                "threshold_ms=%.0f  ← fusion using stale %s data",
                other_module, other_idle_ms, _MODULE_STALE_MS, other_module,
            )

    # ── 4. Cache this module's packet and run fusion ──────────────────────────
    data["_server_recv_ms"] = now_ms          # internal timestamp for staleness check
    latest_module_data[module] = data

    result = process_combined_packet(
        latest_module_data["head"],
        latest_module_data["body"],
        combined_state,
        now_ms,
    )

    # ── 5. Measure server processing time  [VERIFY:PROC_TIME] ────────────────
    # Supports 2.2.8 R2 (end-to-end server latency <500 ms) and the
    # 2.2.1 R2 latency budget of ≤200 ms total.
    proc_ms = (time.perf_counter() - t_start) * 1000.0
    stats   = _device_stats[device_id]
    stats["proc_time_sum_ms"]  += proc_ms
    stats["proc_time_count"]   += 1
    stats["proc_time_max_ms"]   = max(stats["proc_time_max_ms"], proc_ms)

    if proc_ms > _PROC_TIME_WARN_MS:
        logger.warning(
            "[VERIFY:PROC_TIME] device=%-14s proc_ms=%.2f  "
            "⚠ EXCEEDS_BUDGET (%.0f ms)",
            device_id, proc_ms, _PROC_TIME_WARN_MS,
        )
    else:
        logger.debug(
            "[VERIFY:PROC_TIME] device=%-14s proc_ms=%.2f",
            device_id, proc_ms,
        )

    # ── 6. End-to-end latency warning  [VERIFY:LATENCY_WARN] ─────────────────
    # The algorithm already computes latency_ms = server_now - pkt_timestamp.
    # NOTE: this is only meaningful when the ESP32 uses NTP-synchronised Unix
    # timestamps.  Log it for reference; oscilloscope verification is still
    # required for the hardware-to-motor timing path (2.2.1 R2, 2.2.2 R2).
    latency_ms = result.get("latency_ms", 0.0)
    if latency_ms > _LATENCY_WARN_MS:
        logger.warning(
            "[VERIFY:LATENCY_WARN] device=%-14s latency_ms=%.1f  "
            "⚠ EXCEEDS_BUDGET (%.0f ms)  "
            "note=requires_NTP_sync_for_accuracy",
            device_id, latency_ms, _LATENCY_WARN_MS,
        )

    # ── 7. Connection-timeout detection  [VERIFY:TIMEOUT_DETECT] ─────────────
    # Supports 2.2.5 R3 (≤2 s loss detection) and 2.2.7 R2 (≤2 s status update).
    # After updating state, re-scan ALL known devices for idle ones.
    # This happens on every packet, which is the server's opportunity to detect
    # a peer that has gone silent.
    for dev_id, t in _device_timing.items():
        idle_ms = now_ms - t["last_recv_ms"]
        if idle_ms > _TIMEOUT_THRESHOLD_MS:
            logger.warning(
                "[VERIFY:TIMEOUT_DETECT] device=%-14s idle_ms=%.1f  "
                "threshold_ms=%.0f  ← connection assumed lost",
                dev_id, idle_ms, _TIMEOUT_THRESHOLD_MS,
            )

    # ── 8. Hazard zone-change log  [VERIFY:HAZARD_ZONE_CHG] ──────────────────
    # Tracks transitions between hazard states for detection-recall analysis
    # (2.2.2 R1) and the confusion matrix (2.2.2 R3).
    prev_zone = device_state[device_id].get("last_zone", "CLEAR")
    new_zone  = result["hazard"]["label"]
    if new_zone != prev_zone:
        logger.info(
            "[VERIFY:HAZARD_ZONE_CHG] device=%-14s %s → %s  "
            "alert_count=%d",
            device_id, prev_zone, new_zone,
            len(result["hazard"]["alerts"]),
        )
    stats["hazard_counts"][new_zone] += 1

    # ── 9. Update per-device state for /status tracking ───────────────────────
    dev = device_state[device_id]
    dev["module"]        = module
    dev["last_seen_ms"]  = now_ms
    dev["packet_count"] += 1
    dev["last_zone"]     = new_zone

    # Emit the concise decision line (uses algorithm logger)
    print_decision(
        result["hazard"]["alerts"],
        result["hazard"]["level"],
        device_id,
        pkt_ts,
    )

    return jsonify({"status": "ok", "device_id": device_id, **result})


@app.route("/status", methods=["GET"])
def status() -> tuple:
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


@app.route("/log_summary", methods=["GET"])
def log_summary() -> tuple:
    """
    Return a per-device statistics snapshot for requirement verification.

    Designed to be called after a test run (e.g. after 5 minutes of packets)
    to produce the tables required by the design-document verification columns:

      • Packet count / rate table          → 2.2.1 R1, 2.2.5 R1
      • Processing latency statistics      → 2.2.8 R2
      • Hazard zone frequency table        → 2.2.2 R1
      • Inter-packet interval statistics   → 2.2.5 R1
    """
    now_ms  = time.time() * 1000.0
    summary: dict[str, Any] = {
        "global_malformed_packets": _global_malformed_count,
        "devices": {},
    }

    for dev_id in set(list(device_state.keys()) + list(_device_timing.keys())):
        state = device_state.get(dev_id, {})
        t     = _device_timing.get(dev_id, {})
        stats = _device_stats.get(dev_id, {})

        pkt_count    = state.get("packet_count", 0)
        first_ms     = t.get("first_recv_ms", now_ms)
        last_ms      = t.get("last_recv_ms", now_ms)
        elapsed_s    = (last_ms - first_ms) / 1000.0 if last_ms > first_ms else 0.0
        avg_rate_hz  = pkt_count / elapsed_s if elapsed_s > 0 else 0.0

        iv_count = t.get("interval_count", 0)
        avg_iv   = (t["interval_sum_ms"] / iv_count) if iv_count else None
        min_iv   = t.get("interval_min_ms")
        max_iv   = t.get("interval_max_ms")
        if min_iv == float("inf"):
            min_iv = None

        pt_count = stats.get("proc_time_count", 0)
        avg_pt   = (stats["proc_time_sum_ms"] / pt_count) if pt_count else None

        idle_ms = now_ms - last_ms if last_ms else None

        summary["devices"][dev_id] = {
            # 2.2.1 R1 / 2.2.5 R1 – packet-rate verification
            "packet_count":           pkt_count,
            "elapsed_s":              round(elapsed_s, 2),
            "avg_rate_hz":            round(avg_rate_hz, 2),
            "interval_avg_ms":        round(avg_iv, 2) if avg_iv is not None else None,
            "interval_min_ms":        round(min_iv, 2) if min_iv is not None else None,
            "interval_max_ms":        round(max_iv, 2) if max_iv is not None else None,
            # 2.2.8 R2 – server processing latency
            "proc_time_avg_ms":       round(avg_pt, 3) if avg_pt is not None else None,
            "proc_time_max_ms":       round(stats.get("proc_time_max_ms", 0), 3),
            # 2.2.2 R1 / 2.2.2 R3 – hazard detection frequency
            "hazard_zone_counts":     dict(stats.get("hazard_counts", {})),
            # 2.2.5 R3 / 2.2.7 R2 – connection status
            "idle_since_ms":          round(idle_ms, 1) if idle_ms is not None else None,
            "last_zone":              state.get("last_zone", "CLEAR"),
            "module":                 state.get("module", "unknown"),
        }

    return jsonify(summary)


# ===========================================================================
# Navigation state  (Phone/PC app writes; Body ESP32 polls)
# ===========================================================================

_NAV_DIRECTIONS = [
    "front", "front_right", "right", "back_right",
    "back",  "back_left",   "left",  "front_left", "stop",
]

_NAV_MOTOR_MAP: dict[str, int] = {
    "front": 0, "front_right": 1, "right": 2, "back_right": 3,
    "back":  4, "back_left":   5, "left":  6, "front_left":  7, "stop": -1,
}

_NAV_INTENSITY: dict[str, int] = {
    "front": 102, "front_right": 153, "right": 153, "back_right": 153,
    "back":  204, "back_left":   153, "left":  153, "front_left":  153, "stop": 0,
}

_NAV_PATTERN: dict[str, str] = {
    "front": "slow_pulse", "front_right": "medium_pulse", "right": "medium_pulse",
    "back_right": "medium_pulse", "back": "rapid_pulse", "back_left": "medium_pulse",
    "left": "medium_pulse", "front_left": "medium_pulse", "stop": "off",
}

nav_state: dict[str, Any] = {
    "command":     "stop",
    "motor_id":    -1,
    "intensity":   0,
    "pattern":     "off",
    "instruction": "Navigation stopped",
    "distance_m":  None,
    "phase":       "idle",    # idle | prepare | execute
    "active":      False,
    "updated_at":  0.0,
}

route_state: dict[str, Any] = {
    "origin":           None,
    "destination":      None,
    "dest_name":        "",
    "steps":            [],
    "step_index":       0,
    "total_distance_m": 0,
    "total_duration_s": 0,
    "active":           False,
    "updated_at":       0.0,
}

position_state: dict[str, Any] = {
    "lat":        None,
    "lng":        None,
    "heading":    None,
    "updated_at": 0.0,
}

nav_logger = logging.getLogger("omnisense.nav")


# ---------------------------------------------------------------------------
# Navigation endpoints
# ---------------------------------------------------------------------------

@app.route("/nav/set_command", methods=["POST"])
def nav_set_command():
    """
    Phone/PC app pushes the resolved navigation motor command.
    The body ESP32 will poll /nav/get_command to receive it.

    Body::

        {
          "command":     "left",          <- front/right/left/back/... or stop
          "motor_id":    6,
          "intensity":   153,
          "pattern":     "medium_pulse",
          "instruction": "Turn left onto Green St",
          "distance_m":  45.0,
          "phase":       "execute"        <- "prepare" or "execute"
        }
    """
    data = request.get_json(silent=True) or {}
    command = str(data.get("command", "stop")).lower()
    if command not in _NAV_DIRECTIONS:
        command = "stop"

    nav_state["command"]     = command
    nav_state["motor_id"]    = _NAV_MOTOR_MAP[command]
    nav_state["intensity"]   = int(data.get("intensity",  _NAV_INTENSITY.get(command, 0)))
    nav_state["pattern"]     = str(data.get("pattern",    _NAV_PATTERN.get(command, "off")))
    nav_state["instruction"] = str(data.get("instruction", ""))
    nav_state["distance_m"]  = data.get("distance_m")
    nav_state["phase"]       = str(data.get("phase", "execute"))
    nav_state["active"]      = command != "stop"
    nav_state["updated_at"]  = time.time()

    nav_logger.info(
        "[NAV:COMMAND] cmd=%s motor=%d intensity=%d dist=%s phase=%s instr=%r",
        command, nav_state["motor_id"], nav_state["intensity"],
        nav_state["distance_m"], nav_state["phase"], nav_state["instruction"],
    )
    return jsonify({"status": "ok", **nav_state})


@app.route("/nav/get_command", methods=["GET"])
def nav_get_command():
    """
    Body ESP32 polls this endpoint (≥10 Hz) to receive the current navigation
    command and activate the corresponding belt motor.

    Response::

        {
          "status":      "ok",
          "command":     "left",
          "motor_id":    6,
          "intensity":   153,
          "pattern":     "medium_pulse",
          "instruction": "Turn left onto Green St",
          "distance_m":  45.0,
          "phase":       "execute",
          "active":      true,
          "updated_at":  1712689423.5
        }
    """
    return jsonify({"status": "ok", **nav_state})


@app.route("/nav/set_route", methods=["POST"])
def nav_set_route():
    """
    Phone app pushes computed route metadata (from Google Maps Directions API)
    for server-side tracking and logging.
    """
    data = request.get_json(silent=True) or {}
    route_state["origin"]           = data.get("origin")
    route_state["destination"]      = data.get("destination")
    route_state["dest_name"]        = str(data.get("dest_name", ""))
    route_state["steps"]            = data.get("steps", [])
    route_state["step_index"]       = 0
    route_state["total_distance_m"] = float(data.get("total_distance_m", 0))
    route_state["total_duration_s"] = float(data.get("total_duration_s", 0))
    route_state["active"]           = bool(data.get("active", True))
    route_state["updated_at"]       = time.time()

    nav_logger.info(
        "[NAV:ROUTE_SET] dest=%r total_m=%.0f steps=%d",
        route_state["dest_name"], route_state["total_distance_m"],
        len(route_state["steps"]),
    )

    # Reset nav command to straight ahead when a new route starts
    nav_state.update({
        "command": "front", "motor_id": 0,
        "intensity": _NAV_INTENSITY["front"], "pattern": _NAV_PATTERN["front"],
        "instruction": "Head toward destination",
        "distance_m": route_state["total_distance_m"],
        "phase": "prepare", "active": True, "updated_at": time.time(),
    })
    return jsonify({"status": "ok", "route": route_state})


@app.route("/nav/route_status", methods=["GET"])
def nav_route_status():
    """Returns current route metadata and the live navigation motor command."""
    return jsonify({"status": "ok", "route": route_state, "nav": nav_state})


@app.route("/nav/position", methods=["POST"])
def nav_position():
    """
    Phone or ESP32 reports current GPS position + compass heading.
    Server records the position and returns the current nav command so the
    caller can drive the belt motor without a separate poll.

    Body::  {"lat": 40.11, "lng": -88.22, "heading": 270.0}
    """
    data = request.get_json(silent=True) or {}
    position_state["lat"]        = data.get("lat")
    position_state["lng"]        = data.get("lng")
    position_state["heading"]    = data.get("heading")
    position_state["updated_at"] = time.time()
    return jsonify({"status": "ok", "position": position_state, "nav": nav_state})


@app.route("/nav/stop", methods=["POST"])
def nav_stop():
    """Stop active navigation and silence belt motors."""
    nav_state.update({
        "command": "stop", "motor_id": -1, "intensity": 0, "pattern": "off",
        "instruction": "Navigation stopped", "distance_m": None,
        "phase": "idle", "active": False, "updated_at": time.time(),
    })
    route_state["active"]    = False
    route_state["updated_at"] = time.time()
    nav_logger.info("[NAV:STOP] navigation stopped by app")
    return jsonify({"status": "ok"})


if __name__ == "__main__":
    _setup_logging()
    logging.getLogger("omnisense.server").info(
        "OmniSense-Dual server starting  http://0.0.0.0:5000"
    )
    app.run(host="0.0.0.0", port=5000, debug=True)
