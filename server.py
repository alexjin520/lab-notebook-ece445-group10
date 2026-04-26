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
The `/nav` response is **module-specific** — each ESP32 only receives the
haptic decision for the channel it owns (design-doc §1.2 / §2.5):

  • HEAD module (hazard headband) – receives the full 8-motor hazard array:
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

  • BODY module (navigation belt) – receives the single navigation motor
    decision pushed by the Phone/PC app via /nav/set_command:
    {
      "status":     "ok",
      "device_id":  "esp32_body",
      "module":     "body",
      "server_recv_ms": 1712689423500.0,
      "hazard":     {"level": 1, "label": "CAUTION", "alerts": [...]},
      "nav_motor": {
          "command":     "left",            ← front/right/left/back/... or stop
          "motor_id":    6,                 ← 0–7 belt index, -1 for stop
          "intensity":   153,
          "pattern":     "medium_pulse",
          "phase":       "execute",         ← "prepare" or "execute"
          "active":      true,
          "instruction": "Turn left onto Green St",
          "distance_m":  45.0,
          "updated_at":  1712689420.1
      }
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

import json
import logging
import logging.handlers
import os
import time
from collections import defaultdict
import datetime
from typing import Any

import os
from flask import Flask, request, jsonify, send_from_directory
from flask_cors import CORS

from algorithm import (
    process_combined_packet,
    print_decision,
    fresh_device_state,
    wrap_180,
    relative_bearing_to_command,
    expected_turn_angle_deg,
)

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
    json_fpath = os.path.join(log_dir, f"omnisense_{ts}_raw.jsonl")

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

    # ── Raw JSON packet log (one JSON object per line) ────────────────────────
    # Every packet received by /nav is written here verbatim so you can
    # diff exactly what the firmware sent against what the server parsed.
    jh = logging.handlers.RotatingFileHandler(
        json_fpath, maxBytes=10 * 1024 * 1024, backupCount=5, encoding="utf-8",
    )
    jh.setLevel(logging.DEBUG)
    jh.setFormatter(logging.Formatter("%(message)s"))   # raw message only
    json_logger = logging.getLogger("omnisense.raw_json")
    json_logger.propagate = False   # don't echo to console / main log
    json_logger.setLevel(logging.DEBUG)
    json_logger.handlers.clear()
    json_logger.addHandler(jh)

    logging.getLogger("omnisense.server").info(
        "[VERIFY:SESSION_START] log_file=%s json_log=%s latency_warn_ms=%.0f "
        "timeout_threshold_ms=%.0f rate_min_hz=%.1f",
        fpath, json_fpath, _LATENCY_WARN_MS, _TIMEOUT_THRESHOLD_MS, _RATE_MIN_HZ,
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

# Latest fully-processed sensor snapshot exposed via GET /sensor_state
_latest_sensor_state: dict[str, Any] = {}

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

    # ── Raw JSON dump (goes only to omnisense_{ts}_raw.jsonl) ─────────────────
    logging.getLogger("omnisense.raw_json").debug(
        '{"server_ts": "%s", "body": %s}',
        time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        json.dumps(data, separators=(",", ":")),
    )

    now_ms    = time.time() * 1000.0
    t_start   = time.perf_counter()

    device_id = str(data.get("device_id", "esp32"))
    pkt_ts    = int(data.get("timestamp", 0))
    uptime_ms = data.get("uptime_ms")          # top-level uptime since ESP32 boot
    module    = str(data.get("module", "body")).lower()
    if module not in ("head", "body"):
        module = "body"

    _ensure_stats(device_id)

    # ── Optional power / system telemetry ────────────────────────────────────
    battery = data.get("battery") or {}
    system  = data.get("system")  or {}

    # ── 1. Packet-receive log  [VERIFY:PKT_RECV] ─────────────────────────────
    # Supports 2.2.1 R1 (sampling frequency / packet loss) and
    # 2.2.5 R1 (≥10 Hz packet rate) verification.
    _UNIX_EPOCH_MIN_MS = 1_577_836_800_000   # Jan 1 2020 — distinguishes epoch vs uptime
    pkt_count = device_state[device_id]["packet_count"] + 1  # pre-increment preview
    if pkt_ts > _UNIX_EPOCH_MIN_MS:
        ts_label = datetime.datetime.utcfromtimestamp(pkt_ts / 1000.0).strftime("%H:%M:%S.%f")[:-3] + " UTC"
    else:
        ts_label = f"{pkt_ts} ms (uptime)"
    logger.info(
        "┌─ PKT #%-4d  device=%-14s  module=%-4s  ts=%s",
        pkt_count, device_id, module, ts_label,
    )
    logger.info(
        "│  [VERIFY:PKT_RECV]   device=%-14s module=%-4s pkt_ts=%d "
        "server_ms=%.1f count=%d",
        device_id, module, pkt_ts, now_ms, pkt_count,
    )

    # ── 1b. Power / system telemetry  [VERIFY:POWER] ─────────────────────────
    # Logs ESP32 battery and system health fields when provided in the packet.
    if battery or system or uptime_ms is not None:
        uptime_s_str = f"{uptime_ms / 1000.0:.1f}" if uptime_ms is not None else "—"
        logger.info(
            "│  [VERIFY:POWER]      device=%-14s  "
            "voltage_v=%-6s  soc_pct=%-5s  temp_c=%-5s  "
            "heap_b=%-7s  rssi=%-4s  uptime_s=%s",
            device_id,
            battery.get("voltage_v", "—"),
            battery.get("soc_pct",   "—"),
            system.get("die_temp_c",  "—"),
            system.get("free_heap_b", "—"),
            system.get("wifi_rssi",   "—"),
            uptime_s_str,
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
                "│  [VERIFY:PKT_RATE]   device=%-14s module=%-4s "
                "interval_ms=%.1f  rate_hz=%.2f  avg_rate_hz=%.2f  "
                "⚠ BELOW_MINIMUM (%.1f Hz)",
                device_id, module, interval_ms, rate_hz, avg_rate_hz, _RATE_MIN_HZ,
            )
        else:
            logger.info(
                "│  [VERIFY:PKT_RATE]   device=%-14s module=%-4s "
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
                "│  [VERIFY:MODULE_STALE] other_module=%-4s idle_ms=%.1f  "
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
            "│  [VERIFY:PROC_TIME]  device=%-14s proc_ms=%.2f  "
            "⚠ EXCEEDS_BUDGET (%.0f ms)",
            device_id, proc_ms, _PROC_TIME_WARN_MS,
        )
    else:
        logger.debug(
            "│  [VERIFY:PROC_TIME]  device=%-14s proc_ms=%.2f",
            device_id, proc_ms,
        )

    # ── 6. End-to-end latency warning  [VERIFY:LATENCY_WARN] ─────────────────
    # ── 6. End-to-end latency check  [VERIFY:LATENCY_WARN] ──────────────────
    # pkt_ts is the ESP32 uptime (ms since boot), NOT a Unix epoch timestamp,
    # unless the firmware uses NTP.  We detect this by checking whether pkt_ts
    # looks like a Unix epoch value (> Jan 1 2020 = 1_577_836_800_000 ms).
    # If it is clearly an uptime value we skip the bogus subtraction and log a
    # one-time advisory instead of a wall of false warnings.
    latency_ms = result.get("latency_ms", 0.0)
    if pkt_ts > _UNIX_EPOCH_MIN_MS:
        # Firmware is sending real Unix timestamps — check the latency budget.
        if latency_ms > _LATENCY_WARN_MS:
            logger.warning(
                "│  [VERIFY:LATENCY_WARN] device=%-14s latency_ms=%.1f  "
                "⚠ EXCEEDS_BUDGET (%.0f ms)",
                device_id, latency_ms, _LATENCY_WARN_MS,
            )
        else:
            logger.debug(
                "│  [VERIFY:LATENCY_WARN] device=%-14s latency_ms=%.1f  OK",
                device_id, latency_ms,
            )
    else:
        # pkt_ts is ESP32 uptime — latency cannot be computed server-side.
        # Log once per device when it first appears, then go silent.
        if device_state[device_id].get("packet_count", 0) <= 1:
            logger.info(
                "│  [VERIFY:LATENCY_INFO] device=%-14s pkt_ts=%d ms (uptime)  "
                "latency_ms N/A — ESP32 not NTP-synced; "
                "use oscilloscope for hardware timing verification",
                device_id, pkt_ts,
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
                "│  [VERIFY:TIMEOUT_DETECT] device=%-14s idle_ms=%.1f  "
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
            "│  [VERIFY:HAZARD_ZONE_CHG] device=%-14s %s → %s  "
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

    # ── 10. Update live sensor state snapshot for /sensor_state ─────────────
    head_pkt = latest_module_data.get("head", {})
    body_pkt = latest_module_data.get("body", {})

    def _tof_to_m(tof_dict: dict) -> dict:
        """Convert raw tof_sensors dict → {direction: dist_m | None}."""
        out = {}
        for d, v in tof_dict.items():
            if not isinstance(v, dict):
                out[d] = None; continue
            raw = v.get("avg") or v.get("distance_mm")
            out[d] = None if (raw is None or raw >= 8190 or raw <= 0) else round(raw / 1000.0, 4)
        return out

    t = _device_timing.get(device_id, {})
    avg_interval = (t.get("interval_sum_ms", 0) / t["interval_count"]
                    if t.get("interval_count") else None)
    rate_hz = round(1000.0 / avg_interval, 2) if avg_interval else None

    # Power telemetry: the body module owns the LiPo + fuel gauge, so it is
    # the preferred source.  When the body hasn't (yet) connected — e.g.
    # head-only bring-up — fall back to whichever packet has battery data,
    # so the dashboard still shows live numbers instead of all "—".
    body_batt = body_pkt.get("battery") or {}
    body_sys  = body_pkt.get("system")  or {}
    head_batt = head_pkt.get("battery") or {}
    head_sys  = head_pkt.get("system")  or {}

    if body_batt or body_sys or body_pkt.get("uptime_ms") is not None:
        power_src   = "body"
        power_batt  = body_batt
        power_sys   = body_sys
        power_uptime_ms = body_pkt.get("uptime_ms", 0) or 0
    elif head_batt or head_sys or head_pkt.get("uptime_ms") is not None:
        power_src   = "head"
        power_batt  = head_batt
        power_sys   = head_sys
        power_uptime_ms = head_pkt.get("uptime_ms", 0) or 0
    else:
        power_src   = "none"
        power_batt  = {}
        power_sys   = {}
        power_uptime_ms = 0

    # IMU-aware navigation hook: feed every body IMU sample into the
    # orientation cache + active-turn tracker. This is what makes
    # body-frame turn commands and turn-confirmation work.
    body_imu_payload = body_pkt.get("imu") or {}
    if body_imu_payload:
        _ingest_body_imu(body_imu_payload, now_ms / 1000.0)

    _latest_sensor_state.update({
        "device_id":   device_id,
        "module":      module,
        "pkt_count":   device_state[device_id]["packet_count"] + 1,
        "timestamp":   now_ms,
        "tof_body":    _tof_to_m(body_pkt.get("tof_sensors", {})),
        "tof_head":    _tof_to_m(head_pkt.get("tof_sensors", {})),
        "mmwave_body": body_pkt.get("mmwave_sensors", {}),
        "mmwave_head": head_pkt.get("mmwave_sensors", {}),
        # Raw IMU block as the firmware sent it. Empty dict when the module
        # has no IMU, so the dashboard can render a "(not equipped)" tile.
        "imu_body":    body_pkt.get("imu", {}),
        "imu_head":    head_pkt.get("imu", {}),
        # IMU-aware navigation state — body orientation cache + active turn.
        # Repeating it on every /sensor_state poll means the combined
        # dashboard doesn't need a second /nav/turn_status request.
        "orientation": dict(body_orientation_state),
        "turn_status": _turn_status_payload(),
        "fused": {
            m["direction"]: {
                "dist_m":  m["distance_m"],
                "zone":    m["zone"],
                "source":  m["source"],
            }
            for m in result["motors"]
        },
        "motors":      result["motors"],
        "hazard":      result["hazard"],
        "imu_combined": result["imu"],
        "power": {
            "source":    power_src,                          # "body" | "head" | "none"
            "voltage_v": power_batt.get("voltage_v"),
            "soc_pct":   power_batt.get("soc_pct"),
            "temp_c":    power_sys.get("die_temp_c"),
            "heap_b":    power_sys.get("free_heap_b"),
            "rssi":      power_sys.get("wifi_rssi"),
            "uptime_s":  power_uptime_ms / 1000.0,
        },
        "pkt_rate_hz": rate_hz,
        "proc_ms":     round(proc_ms, 2),
        "latency_ms":  round(latency_ms, 1),
    })

    # Emit the concise decision line (uses algorithm logger)
    print_decision(
        result["hazard"]["alerts"],
        result["hazard"]["level"],
        device_id,
        pkt_ts,
    )
    logger.info(
        "└─ PKT #%-4d  device=%-14s  hazard=%-8s  proc_ms=%.2f",
        pkt_count, device_id, new_zone, proc_ms,
    )

    # Channel separation (design doc 1.2 / 2.5):
    #   • HEAD  module → 8 hazard motor commands (head wears the alert headband)
    #   • BODY  module → 1 navigation motor command (waist wears the nav belt)
    # Each module receives ONLY the haptics it owns, plus the shared hazard
    # summary so both can log the same fused state.
    if module == "head":
        return jsonify({
            "status":         "ok",
            "device_id":      device_id,
            "module":         "head",
            "server_recv_ms": now_ms,   # ← ESP32: compute RTT as millis()-pkt_ts_sent, latency≈RTT/2
            **result,
        })
    else:
        # Body module: return current navigation motor decision from nav_state
        # (pushed by the Phone/PC app via /nav/set_command).  The body firmware
        # drives the belt motor matching nav_motor.motor_id at the given
        # intensity / pattern.  Hazard summary is still included for logging
        # but hazard haptics belong to the head module only.
        nav_motor = {
            "command":     nav_state["command"],
            "motor_id":    nav_state["motor_id"],
            "intensity":   nav_state["intensity"],
            "pattern":     nav_state["pattern"],
            "phase":       nav_state["phase"],
            "active":      nav_state["active"],
            "instruction": nav_state["instruction"],
            "distance_m":  nav_state["distance_m"],
            "updated_at":  nav_state["updated_at"],
        }

        nav_logger.debug(
            "│  [NAV:BODY_DELIVER] device=%-14s cmd=%s motor=%d intensity=%d "
            "pattern=%s active=%s",
            device_id, nav_motor["command"], nav_motor["motor_id"],
            nav_motor["intensity"], nav_motor["pattern"], nav_motor["active"],
        )

        return jsonify({
            "status":         "ok",
            "device_id":      device_id,
            "module":         module,
            "server_recv_ms": now_ms,
            "hazard":         result["hazard"],   # hazard summary for body logging
            "nav_motor":      nav_motor,          # navigation belt motor decision
            "turn_status":    _turn_status_payload(),  # live turn-confirmation
        })


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


@app.route("/sensor_state", methods=["GET"])
def sensor_state() -> tuple:
    """
    Returns the latest fully-processed sensor snapshot for the dashboard.

    Includes raw ToF/mmWave/IMU readings from both modules, the fused
    per-direction obstacle map, all 8 motor commands, hazard summary,
    power telemetry, and packet statistics.
    """
    if not _latest_sensor_state:
        return jsonify({"status": "waiting", "message": "No packets received yet"}), 204
    return jsonify({"status": "ok", **_latest_sensor_state})


# ---------------------------------------------------------------------------
# Per-module snapshot  (drives the head-only and body-only dashboards)
# ---------------------------------------------------------------------------

_MODULE_OFFLINE_MS = 2000   # treat module as offline after 2s of silence


def _tof_dict_to_metres(tof_dict: dict) -> dict:
    """Convert raw tof_sensors dict → {direction: dist_m | None}.

    Mirrors the same filter used by the combined /sensor_state endpoint:
    drop sentinel/over-range values (>= 8190 mm or <= 0).
    """
    out = {}
    for d, v in tof_dict.items():
        if not isinstance(v, dict):
            out[d] = None
            continue
        raw = v.get("avg") or v.get("distance_mm")
        out[d] = (None if (raw is None or raw >= 8190 or raw <= 0)
                  else round(raw / 1000.0, 4))
    return out


def _per_module_snapshot(module: str) -> dict | None:
    """
    Build a per-module sensor-state snapshot for /sensor_state/<module>.

    Returns ``None`` if no packet has ever been received from that module.
    The payload is intentionally focused on a single module so the per-module
    dashboards (head_dashboard.html / body_dashboard.html) can render their
    page without filtering out the other module's noise.
    """
    if module not in ("head", "body"):
        return None

    pkt = latest_module_data.get(module) or {}
    if not pkt:
        return None

    device_id = pkt.get("device_id")
    if not device_id:
        # Fall back: scan device_state for any device whose .module matches.
        for did, st in device_state.items():
            if st.get("module") == module:
                device_id = did
                break

    state = device_state.get(device_id, {}) if device_id else {}
    t     = _device_timing.get(device_id, {}) if device_id else {}
    stats = _device_stats.get(device_id, {}) if device_id else {}

    now_ms       = time.time() * 1000.0
    last_seen_ms = state.get("last_seen_ms", 0) or t.get("last_recv_ms", 0)
    idle_ms      = (now_ms - last_seen_ms) if last_seen_ms else None
    online       = idle_ms is not None and idle_ms < _MODULE_OFFLINE_MS

    iv_count = t.get("interval_count", 0)
    avg_iv   = (t["interval_sum_ms"] / iv_count) if iv_count else None
    rate_hz  = round(1000.0 / avg_iv, 2) if avg_iv else None
    min_iv   = t.get("interval_min_ms")
    max_iv   = t.get("interval_max_ms")
    if min_iv == float("inf"):
        min_iv = None

    pt_count = stats.get("proc_time_count", 0)
    avg_pt   = (stats["proc_time_sum_ms"] / pt_count) if pt_count else None
    max_pt   = stats.get("proc_time_max_ms", 0.0)

    snap: dict[str, Any] = {
        "module":           module,
        "device_id":        device_id,
        "online":           online,
        "last_seen_ms":     last_seen_ms or None,
        "idle_since_ms":    round(idle_ms, 1) if idle_ms is not None else None,
        "packet_count":     state.get("packet_count", 0),
        "pkt_rate_hz":      rate_hz,
        "interval_avg_ms":  round(avg_iv, 2) if avg_iv is not None else None,
        "interval_min_ms":  round(min_iv, 2) if min_iv is not None else None,
        "interval_max_ms":  round(max_iv, 2) if max_iv is not None else None,
        # proc_ms / latency_ms reflect the most recent packet across all
        # devices, but in practice the module-of-interest is also the most
        # recent producer at ~10 Hz, so the value tracks closely.
        "proc_ms":          _latest_sensor_state.get("proc_ms"),
        "proc_avg_ms":      round(avg_pt, 3) if avg_pt is not None else None,
        "proc_max_ms":      round(max_pt, 3) if max_pt else None,
        "latency_ms":       _latest_sensor_state.get("latency_ms"),
        "uptime_s":         (pkt.get("uptime_ms", 0) or 0) / 1000.0,
        "tof":              _tof_dict_to_metres(pkt.get("tof_sensors", {})),
        "tof_raw":          pkt.get("tof_sensors", {}),
        "mmwave":           pkt.get("mmwave_sensors", {}),
        "imu":              pkt.get("imu", {}),
        "battery":          pkt.get("battery", {}),
        "system":           pkt.get("system", {}),
        "last_zone":        state.get("last_zone", "CLEAR"),
        # Shared fused/hazard summary so the per-module page can show what the
        # algorithm decided after merging with the other module.
        "hazard":           _latest_sensor_state.get("hazard", {}),
        "fused":            _latest_sensor_state.get("fused", {}),
        "hazard_zone_counts": dict(stats.get("hazard_counts", {})),
        "timestamp":        now_ms,
    }

    if module == "head":
        # Head wears the 8-motor hazard ring – expose the server-commanded
        # set so the page shows exactly what the head ESP32 was told to do.
        snap["motors"] = _latest_sensor_state.get("motors", [])
    else:
        # Body wears the single-motor navigation belt.  Reflect the latest
        # nav_state so the page shows the active turn cue / instruction.
        snap["nav_motor"] = {
            "command":     nav_state["command"],
            "motor_id":    nav_state["motor_id"],
            "intensity":   nav_state["intensity"],
            "pattern":     nav_state["pattern"],
            "phase":       nav_state["phase"],
            "active":      nav_state["active"],
            "instruction": nav_state["instruction"],
            "distance_m":  nav_state["distance_m"],
            "updated_at":  nav_state["updated_at"],
        }
        # IMU-aware navigation state — body owns the only IMU, so the body
        # dashboard exposes the orientation cache and live turn confirmation
        # without an extra /nav/turn_status round-trip.
        snap["orientation"] = dict(body_orientation_state)
        snap["turn_status"] = _turn_status_payload()

    return snap


@app.route("/sensor_state/<module>", methods=["GET"])
def sensor_state_module(module: str) -> tuple:
    """
    Return the latest sensor snapshot for a single module ("head" | "body").

    Used by the per-module dashboards (head_dashboard.html, body_dashboard.html)
    so each page can poll only its own module's data without mixing in the
    other module's sensors.
    """
    if module not in ("head", "body"):
        return jsonify({
            "status":  "error",
            "message": f"unknown module '{module}', expected 'head' or 'body'",
        }), 404
    snap = _per_module_snapshot(module)
    if snap is None:
        return jsonify({
            "status":  "waiting",
            "module":  module,
            "message": f"No packets received from {module} module yet",
        }), 204
    return jsonify({"status": "ok", **snap})


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

# ---------------------------------------------------------------------------
# Body IMU orientation cache — populated whenever a body /nav packet arrives
# carrying ICM-20948 data. Used to translate phone-supplied absolute bearings
# into body-frame haptic commands and to confirm turns.
# ---------------------------------------------------------------------------
body_orientation_state: dict[str, Any] = {
    "yaw_deg":          None,    # tilt-compensated heading (0=N, CW)
    "gyro_z_dps":       0.0,     # current yaw rate
    "is_turning":       False,   # firmware classifier
    "is_falling":       False,
    "mag_calibrated":   False,
    "updated_at":       0.0,     # epoch seconds
}

# Active turn-confirmation state. None when no turn is being tracked.
# Started by /nav/set_command, advanced by every body /nav packet, cleared by
# completion / timeout / explicit /nav/stop.
turn_state: dict[str, Any] = {
    "active":          False,
    "command":         "stop",        # body-frame direction issued
    "expected_deg":    0.0,           # absolute angle the user must turn
    "signed_target":   0.0,           # signed delta (+CW / -CCW)
    "start_yaw":       None,          # body yaw at command issue
    "progress_deg":    0.0,           # accumulated yaw rotation since start
    "completion_pct":  0.0,           # progress_deg / expected_deg, clamped 0–1
    "completed":       False,
    "started_at":      0.0,
    "updated_at":      0.0,
    "timeout_s":       30.0,          # auto-give-up window
    "last_yaw":        None,          # internal: previous yaw sample
}

# Tolerance: a turn is confirmed when the user has rotated this fraction of
# the expected angle. 0.85 leaves room for the user not perfectly nailing
# 90°/180° while still being unambiguous.
_TURN_COMPLETE_FRACTION = 0.85

nav_logger = logging.getLogger("omnisense.nav")


# ---------------------------------------------------------------------------
# IMU-aware navigation helpers
# ---------------------------------------------------------------------------
# These are invoked from the /nav handler (every body packet) and from the
# /nav/set_command handler (when the phone issues a turn). They turn raw IMU
# telemetry into:
#   1. A live yaw cache used to translate phone-supplied compass bearings
#      into the correct body-frame haptic command.
#   2. A turn-progress tracker that integrates the user's actual rotation
#      so the server can confirm the turn happened and auto-advance the
#      Google-Maps step index.
# ---------------------------------------------------------------------------

def _ingest_body_imu(imu: dict, now_s: float) -> None:
    """Update body_orientation_state and advance any active turn from a
    body-side IMU payload. Called once per /nav body packet.

    Yaw is required; gyro_z is optional (used as a fallback / sanity check).
    Silently no-ops if no usable yaw is present (e.g. firmware not reporting
    IMU yet, or the magnetometer is uncalibrated and yaw is None).
    """
    yaw = imu.get("yaw_deg")
    if yaw is None:
        # No magnetometer fix yet — keep last known yaw, do not advance turn.
        body_orientation_state["gyro_z_dps"]     = float(imu.get("gyro_z", 0.0) or 0.0)
        body_orientation_state["is_turning"]     = bool(imu.get("is_turning", False))
        body_orientation_state["is_falling"]     = bool(imu.get("is_falling", False))
        body_orientation_state["mag_calibrated"] = bool(imu.get("mag_calibrated", False))
        body_orientation_state["updated_at"]     = now_s
        return

    yaw = float(yaw)
    prev_yaw = body_orientation_state["yaw_deg"]

    body_orientation_state["yaw_deg"]        = yaw
    body_orientation_state["gyro_z_dps"]     = float(imu.get("gyro_z", 0.0) or 0.0)
    body_orientation_state["is_turning"]     = bool(imu.get("is_turning", False))
    body_orientation_state["is_falling"]     = bool(imu.get("is_falling", False))
    body_orientation_state["mag_calibrated"] = bool(imu.get("mag_calibrated", False))
    body_orientation_state["updated_at"]     = now_s

    if turn_state["active"] and not turn_state["completed"]:
        _advance_turn_progress(yaw, prev_yaw, now_s)


def _advance_turn_progress(yaw: float, prev_yaw: float | None, now_s: float) -> None:
    """Integrate yaw rotation into turn_state["progress_deg"]. Each /nav body
    packet contributes the wrapped delta between the previous and current yaw
    samples; we accumulate the *signed* delta in the direction of the
    expected turn.

    When |progress| reaches _TURN_COMPLETE_FRACTION × expected_deg, the turn
    is marked complete, route_state.step_index is auto-advanced, and the
    nav command is reset to "front".
    """
    expected   = float(turn_state["expected_deg"])
    sign_target = 1.0 if turn_state["signed_target"] >= 0 else -1.0

    if prev_yaw is None:
        # First sample after command issue — nothing to integrate yet.
        turn_state["last_yaw"]   = yaw
        turn_state["updated_at"] = now_s
        return

    # Wrapped angle delta in (-180, +180]; positive = clockwise rotation.
    delta = wrap_180(yaw - (turn_state["last_yaw"] or prev_yaw))

    # Project onto the expected rotation direction so counter-rotation
    # subtracts. For "front" / "stop" (expected==0) we just measure |progress|
    # for telemetry purposes but never auto-complete.
    progress = float(turn_state["progress_deg"]) + delta * sign_target
    turn_state["progress_deg"] = progress
    turn_state["last_yaw"]     = yaw
    turn_state["updated_at"]   = now_s

    if expected > 1e-3:
        turn_state["completion_pct"] = max(0.0, min(1.0, progress / expected))

    # Auto-confirm the turn once the user has executed _TURN_COMPLETE_FRACTION
    # of the requested angle. We require progress in the *correct* direction
    # to count, hence the ≥ check (not abs) — pivoting the wrong way will
    # never satisfy this.
    if expected > 1e-3 and progress >= expected * _TURN_COMPLETE_FRACTION:
        _on_turn_completed(now_s)
        return

    # Timeout: if the user hasn't executed the turn within turn_state.timeout_s
    # we leave nav_state alone (the belt keeps buzzing) but mark the turn
    # inactive so the phone can show a nudge / re-issue.
    if (now_s - turn_state["started_at"]) > float(turn_state["timeout_s"]):
        turn_state["active"]    = False
        turn_state["completed"] = False
        nav_logger.warning(
            "[NAV:TURN_TIMEOUT] command=%s expected=%.0f° progress=%.0f° "
            "elapsed=%.1fs — user did not complete the turn in time",
            turn_state["command"], expected, progress,
            now_s - turn_state["started_at"],
        )


def _on_turn_completed(now_s: float) -> None:
    """Mark the active turn complete, advance the route step, and reset the
    belt motor to "front" (straight). The phone will pick up the new
    instruction on its next /nav/route_status poll, or via /nav/turn_status.
    """
    turn_state["active"]     = False
    turn_state["completed"]  = True
    turn_state["updated_at"] = now_s

    if route_state.get("active") and route_state.get("steps"):
        route_state["step_index"] = min(
            int(route_state["step_index"]) + 1,
            max(0, len(route_state["steps"]) - 1),
        )
        route_state["updated_at"] = time.time()

    nav_logger.info(
        "[NAV:TURN_DONE] command=%s expected=%.0f° actual=%.0f° "
        "step_index→%d",
        turn_state["command"], turn_state["expected_deg"],
        turn_state["progress_deg"], route_state.get("step_index", 0),
    )

    # Reset the haptic command to straight-ahead; phone pushes the next turn
    # via /nav/set_command when the user nears the next step.
    nav_state.update({
        "command":     "front",
        "motor_id":    _NAV_MOTOR_MAP["front"],
        "intensity":   _NAV_INTENSITY["front"],
        "pattern":     _NAV_PATTERN["front"],
        "instruction": "Turn complete — continue straight",
        "phase":       "execute",
        "active":      True,
        "updated_at":  time.time(),
    })


def _start_turn_tracking(command: str) -> None:
    """Snapshot the body yaw at the moment a turn command is issued.

    No-op for non-turn commands (front / stop). For valid turn commands we
    seed turn_state with the expected angle and direction so subsequent /nav
    body packets can integrate progress.
    """
    expected_abs    = expected_turn_angle_deg(command)
    signed_target   = 0.0 if command == "front" else (
        180.0 if command == "back" else
        # _COMMAND_DELTA_DEG isn't exported, but we re-derive the sign from the
        # command name — *_right is positive (CW), *_left is negative (CCW).
        +expected_abs if command.endswith("right") else -expected_abs
    )

    if expected_abs < 1e-3 or command in ("front", "stop"):
        # Reset / no tracking needed.
        turn_state.update({
            "active":         False,
            "command":        command,
            "expected_deg":   0.0,
            "signed_target":  0.0,
            "start_yaw":      body_orientation_state.get("yaw_deg"),
            "progress_deg":   0.0,
            "completion_pct": 0.0,
            "completed":      command == "stop",
            "started_at":     time.time(),
            "updated_at":     time.time(),
            "last_yaw":       body_orientation_state.get("yaw_deg"),
        })
        return

    turn_state.update({
        "active":         True,
        "command":        command,
        "expected_deg":   expected_abs,
        "signed_target":  signed_target,
        "start_yaw":      body_orientation_state.get("yaw_deg"),
        "progress_deg":   0.0,
        "completion_pct": 0.0,
        "completed":      False,
        "started_at":     time.time(),
        "updated_at":     time.time(),
        "last_yaw":       body_orientation_state.get("yaw_deg"),
    })

    nav_logger.info(
        "[NAV:TURN_START] command=%s expected=%.0f° start_yaw=%s",
        command, expected_abs,
        f"{turn_state['start_yaw']:.1f}°" if turn_state["start_yaw"] is not None else "?",
    )


def _turn_status_payload() -> dict:
    """Compact, JSON-safe view of turn_state for HTTP responses."""
    return {
        "active":         bool(turn_state["active"]),
        "completed":      bool(turn_state["completed"]),
        "command":        turn_state["command"],
        "expected_deg":   round(float(turn_state["expected_deg"]), 1),
        "progress_deg":   round(float(turn_state["progress_deg"]), 1),
        "completion_pct": round(float(turn_state["completion_pct"]), 3),
        "start_yaw":      (None if turn_state["start_yaw"] is None
                           else round(float(turn_state["start_yaw"]), 1)),
        "current_yaw":    (None if body_orientation_state["yaw_deg"] is None
                           else round(float(body_orientation_state["yaw_deg"]), 1)),
        "elapsed_s":      round(max(0.0, time.time() - float(turn_state["started_at"])), 2),
        "timeout_s":      float(turn_state["timeout_s"]),
        "mag_calibrated": bool(body_orientation_state["mag_calibrated"]),
    }


# ---------------------------------------------------------------------------
# Navigation endpoints
# ---------------------------------------------------------------------------

@app.route("/nav/set_command", methods=["POST"])
def nav_set_command():
    """
    Phone/PC app pushes the resolved navigation motor command.
    The body ESP32 will poll /nav/get_command to receive it.

    Two ways to specify the direction (in priority order):

    1.  IMU-aware (preferred) — supply the *absolute* compass bearing the
        user must travel. The server reads the latest body IMU yaw and
        translates the bearing into the right body-frame command.

            {
              "target_bearing_deg": 90.0,        # 0=N, 90=E, 180=S, 270=W
              "instruction": "Turn right onto Wright St",
              "distance_m":  45.0,
              "phase":       "execute"
            }

    2.  Direct command — caller already knows which belt motor to fire
        (legacy / manual control):

            {
              "command":     "left",             # front/right/left/back/...
              "motor_id":    6,
              "intensity":   153,
              "pattern":     "medium_pulse",
              "instruction": "Turn left onto Green St",
              "distance_m":  45.0,
              "phase":       "execute"
            }

    Whenever a turn command (anything but front/stop) is issued, the server
    snapshots the current body yaw and starts a turn-confirmation tracker.
    Subsequent body /nav packets advance the tracker; once the user has
    rotated ~85% of the expected angle the route step auto-advances and the
    belt drops back to "front".
    """
    data = request.get_json(silent=True) or {}

    target_bearing = data.get("target_bearing_deg")
    resolved_via_imu = False
    delta_deg: float | None = None

    if target_bearing is not None:
        # IMU-aware path: rotate the world bearing into the user body frame.
        try:
            target_bearing = float(target_bearing)
        except (TypeError, ValueError):
            return jsonify({
                "status":  "error",
                "message": "target_bearing_deg must be a number (0–360)",
            }), 400

        user_yaw = body_orientation_state.get("yaw_deg")
        if user_yaw is None:
            # No yaw fix yet — the firmware hasn't sent IMU data, or the
            # magnetometer is uncalibrated. Fall back to the phone's own
            # heading if it pushed one via /nav/position.
            user_yaw = position_state.get("heading")

        if user_yaw is None:
            return jsonify({
                "status":  "error",
                "message": ("no body yaw available yet — calibrate the IMU "
                            "or POST /nav/position first"),
            }), 409

        command, delta_deg = relative_bearing_to_command(
            target_bearing, float(user_yaw)
        )
        resolved_via_imu = True

        nav_logger.info(
            "[NAV:RESOLVE_BEARING] target=%.1f° yaw=%.1f° → command=%s "
            "delta=%+.1f°",
            target_bearing, float(user_yaw), command, delta_deg,
        )
    else:
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

    # Kick off / reset turn confirmation. For "front"/"stop" this just clears
    # any stale tracker; for an actual turn it snapshots the start yaw.
    _start_turn_tracking(command)

    nav_logger.info(
        "[NAV:COMMAND] cmd=%s motor=%d intensity=%d dist=%s phase=%s instr=%r%s",
        command, nav_state["motor_id"], nav_state["intensity"],
        nav_state["distance_m"], nav_state["phase"], nav_state["instruction"],
        f" (resolved_via_imu, delta={delta_deg:+.1f}°)" if resolved_via_imu else "",
    )

    return jsonify({
        "status":           "ok",
        "resolved_via_imu": resolved_via_imu,
        "delta_deg":        (None if delta_deg is None else round(delta_deg, 1)),
        "turn":             _turn_status_payload(),
        **nav_state,
    })


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


@app.route("/nav/turn_status", methods=["GET"])
def nav_turn_status():
    """
    Phone polls this endpoint to know whether the user has executed the
    most recently issued turn (for auto-advancing the on-screen route step
    and for re-prompting if the user is hesitating).

    Response::

        {
          "status": "ok",
          "turn": {
            "active":         true,        # turn currently being tracked
            "completed":      false,       # user has rotated through the angle
            "command":        "left",
            "expected_deg":    90.0,
            "progress_deg":    62.4,       # signed; ≥ expected*0.85 = done
            "completion_pct":  0.69,
            "start_yaw":     180.0,
            "current_yaw":   118.4,
            "elapsed_s":       3.2,
            "timeout_s":      30.0,
            "mag_calibrated": true
          },
          "orientation": {
            "yaw_deg":       118.4,
            "gyro_z_dps":   -27.5,
            "is_turning":    true,
            "is_falling":    false,
            "mag_calibrated": true,
            "updated_at":   1712689424.1
          }
        }
    """
    return jsonify({
        "status":      "ok",
        "turn":        _turn_status_payload(),
        "orientation": dict(body_orientation_state),
    })


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
    # Clear any in-flight turn tracking so the next nav session starts clean.
    _start_turn_tracking("stop")
    nav_logger.info("[NAV:STOP] navigation stopped by app")
    return jsonify({"status": "ok"})


_FRONTEND_DIR = os.path.join(os.path.dirname(__file__), "frontend")


@app.route("/dashboard")
def dashboard():
    """Serve the combined (head + body) sensor dashboard UI."""
    return send_from_directory(_FRONTEND_DIR, "sensor_dashboard.html")


@app.route("/dashboard/head")
def dashboard_head():
    """Serve the head-only dashboard.

    Polls /sensor_state/head and renders the 8-motor hazard ring along
    with head-side ToF / mmWave / IMU / battery telemetry.
    """
    return send_from_directory(_FRONTEND_DIR, "head_dashboard.html")


@app.route("/dashboard/body")
def dashboard_body():
    """Serve the body-only dashboard.

    Polls /sensor_state/body and renders the navigation belt motor along
    with body-side ToF / mmWave / IMU / battery telemetry.
    """
    return send_from_directory(_FRONTEND_DIR, "body_dashboard.html")


@app.route("/frontend/<path:filename>")
def frontend_static(filename):
    """Serve any other frontend asset (CSS, JS, etc.) under /frontend/."""
    return send_from_directory(_FRONTEND_DIR, filename)


if __name__ == "__main__":
    _setup_logging()
    logging.getLogger("omnisense.server").info(
        "OmniSense-Dual server starting  http://0.0.0.0:5000"
    )
    app.run(host="0.0.0.0", port=5000, debug=True)
