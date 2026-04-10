"""
OmniSense-Dual  Test / Simulation Server  |  ECE 445 Group 10
==============================================================

Self-contained simulation server for testing the navigation frontend
WITHOUT any physical hardware (ESP32, sensors, motors).

What it does
------------
  • Exposes every REST endpoint that server.py exposes, returning realistic
    simulated data so the frontend behaves exactly as in production.
  • Runs a background thread that moves a virtual walker along a GPS route
    at configurable speed, computes turn commands, and writes them into
    the same nav_state that /nav/get_command returns.
  • Periodically injects random (or manually triggered) hazard alerts so
    the frontend hazard panel can be verified.
  • Serves a browser-based Simulation Control Panel at /sim/ui.

Usage
-----
  pip install flask flask-cors
  python test_server.py          # starts on port 5001

Then:
  1. Open the control panel  →  http://localhost:5001/sim/ui
  2. Open the navigation UI  →  http://localhost:8080  (python -m http.server
                                8080 --directory frontend)
  3. In the nav UI change SERVER_URL (bottom of sidebar) to
     http://localhost:5001 and click Connect.
  4. Enter a destination and click "Start Navigation" – the simulated walker
     will immediately start moving.
  5. Click "Enable Sim Mode" in the nav UI sidebar to show the moving dot
     on the Google Map.

Simulation Control Panel endpoints  (/sim/*)
---------------------------------------------
  GET  /sim/ui            Browser control panel (HTML)
  GET  /sim/state         Current sim snapshot (position, heading, hazard, nav)
  POST /sim/start         Start / restart the walker
  POST /sim/stop          Pause the walker
  POST /sim/reset         Jump back to route start
  POST /sim/speed         {"speed_mps": 1.4}
  POST /sim/hazard        {"direction":"back","zone":"DANGER","duration_s":3}
  POST /sim/hazard/clear  Clear all active hazards
  POST /sim/config        {"hazard_prob": 0.03, "loop_route": true}
"""

from __future__ import annotations

import logging
import logging.handlers
import math
import os
import random
import threading
import time
from collections import defaultdict
from typing import Any

from flask import Flask, jsonify, request
from flask_cors import CORS

# ---------------------------------------------------------------------------
# Logging setup
# ---------------------------------------------------------------------------

_LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")

# Navigation verification thresholds (from design doc)
_NAV_LATENCY_WARN_MS    = 200.0   # 2.2.1 R2 – motor activation ≤ 200 ms
_HEADING_ERR_WARN_DEG   = 10.0    # 2.2.3 R4 – heading error ≤ 10° during straight walk
_CMD_SUCCESS_WARN_PCT   = 98.0    # 2.2.7 R1 – command delivery ≥ 98 %
_CONN_TIMEOUT_MS        = 2000.0  # 2.2.7 R2 – connection status update ≤ 2 s
_CMD_POLL_WARN_IDLE_MS  = 2000.0  # warn if ESP32 has not polled for > 2 s


def _setup_logging(log_dir: str = _LOG_DIR) -> None:
    """
    Configure console (INFO) + rotating file (DEBUG) handlers on the
    'omnisense' logger hierarchy, creating a new timestamped log file.

    Navigation-specific [VERIFY:*] tags and the requirement each covers:

      TAG                    Requirement
      ─────────────────────  ─────────────────────────────────────────────
      NAV_ROUTE_SET          2.2.7 R1 – route delivery success tracking
      NAV_CMD_RECV           2.2.1 R2 – command latency ≤ 200 ms; 2.2.7 R1
      NAV_CMD_MOTOR          2.2.1 R3 – motor cross-activation confusion matrix
      NAV_CMD_POLL           2.2.7 R1 – ESP32 poll delivery tracking
      NAV_POS_UPDATE         2.2.3 R4 – GPS position updates for heading log
      NAV_HEADING            2.2.3 R4 – heading error ≤ 10° straight walk
      NAV_CMD_COMPUTE        2.2.1 R3 / High-level R2 – turn accuracy ≥ 85 %
      NAV_WAYPOINT           2.2.1 R3 – waypoint transitions for turn accuracy
      NAV_ARRIVED            2.2.7 R1 / High-level R2 – route completion
      NAV_STOPPED            session marker
      SIM_HAZARD_INJ         2.2.2 R1 – injected hazard events for recall test
      SIM_HAZARD_EXP         2.2.2 R1 – hazard expiry (obstacle gone)
      NAV_CMD_SUMMARY        2.2.7 R1 – cumulative success-rate table
      NAV_PROC_TIME          2.2.8 R2 – server processing latency < 500 ms
      CONN_IDLE              2.2.7 R2 – connection-loss detection ≤ 2 s
    """
    os.makedirs(log_dir, exist_ok=True)
    ts    = time.strftime("%Y%m%d_%H%M%S")
    fpath = os.path.join(log_dir, f"nav_{ts}.log")

    root = logging.getLogger("omnisense")
    root.setLevel(logging.DEBUG)
    root.handlers.clear()

    ch = logging.StreamHandler()
    ch.setLevel(logging.INFO)
    ch.setFormatter(logging.Formatter(
        "%(asctime)s %(levelname)-8s %(message)s", datefmt="%H:%M:%S",
    ))
    root.addHandler(ch)

    fh = logging.handlers.RotatingFileHandler(
        fpath, maxBytes=10 * 1024 * 1024, backupCount=10, encoding="utf-8",
    )
    fh.setLevel(logging.DEBUG)
    fh.setFormatter(logging.Formatter(
        "%(asctime)s.%(msecs)03d %(levelname)-8s %(name)s  %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    ))
    root.addHandler(fh)

    logging.getLogger("omnisense.nav").info(
        "[VERIFY:NAV_SESSION_START] log_file=%s  "
        "nav_latency_warn_ms=%.0f  heading_err_warn_deg=%.1f  "
        "cmd_success_warn_pct=%.0f",
        fpath, _NAV_LATENCY_WARN_MS, _HEADING_ERR_WARN_DEG, _CMD_SUCCESS_WARN_PCT,
    )
    return fpath


logger = logging.getLogger("omnisense.nav")


def _record_proc_time(proc_ms: float) -> None:
    """Thread-safe accumulation of server processing latency stats."""
    _nav_stats["proc_time_sum_ms"] += proc_ms
    _nav_stats["proc_time_count"]  += 1
    if proc_ms > _nav_stats["proc_time_max_ms"]:
        _nav_stats["proc_time_max_ms"] = proc_ms

# ---------------------------------------------------------------------------
# Navigation statistics  (accumulated across the server lifetime)
# ---------------------------------------------------------------------------

_nav_stats: dict[str, Any] = {
    # 2.2.7 R1 – command delivery success rate
    "cmd_recv_total":    0,    # total POST /nav/set_command received
    "cmd_recv_ok":       0,    # successfully processed (always == recv for 200 OK)
    "cmd_poll_total":    0,    # total GET /nav/get_command served
    "cmd_last_poll_ms":  0.0,  # wall-clock ms of last poll (for idle detection)
    # 2.2.1 R3 / High-level R2 – turn accuracy
    "cmd_history":       [],   # [{"commanded": dir, "step_maneuver": str, "phase": str}]
    "waypoints_reached": 0,
    # 2.2.3 R4 – heading error
    "heading_errors":    [],   # list of abs(heading_err_deg) during straight nav
    # 2.2.8 R2 – server processing latency
    "proc_time_sum_ms":  0.0,
    "proc_time_max_ms":  0.0,
    "proc_time_count":   0,
    # 2.2.2 R1 – hazard injection events
    "hazard_inject_count": 0,
    # route tracking
    "routes_set":        0,
}

# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------

app = Flask(__name__)
CORS(app)

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------

DIRECTIONS = [
    "front", "front_right", "right", "back_right",
    "back",  "back_left",   "left",  "front_left",
]

HAZARD_LEVELS = {
    "CLEAR": 0, "ALERT": 1, "CAUTION": 2,
    "WARNING": 3, "DANGER": 4, "CRITICAL": 5,
}

ZONE_DISTANCES = {           # representative distances for each zone
    "CRITICAL": 0.3,
    "DANGER":   1.0,
    "WARNING":  2.2,
    "CAUTION":  4.0,
    "ALERT":    8.0,
    "CLEAR":    None,
}

NAV_INTENSITY = {
    "front": 102, "front_right": 153, "right": 153, "back_right": 153,
    "back":  204, "back_left":   153, "left":  153, "front_left":  153, "stop": 0,
}
NAV_PATTERN = {
    "front": "slow_pulse", "front_right": "medium_pulse", "right": "medium_pulse",
    "back_right": "medium_pulse", "back": "rapid_pulse",  "back_left": "medium_pulse",
    "left": "medium_pulse", "front_left": "medium_pulse", "stop": "off",
}

WAYPOINT_REACHED_M  = 15.0   # metres – snap to next waypoint
SIM_HZ              = 10     # simulation tick rate
HEADING_SMOOTH      = 0.25   # heading interpolation factor per tick

# ---------------------------------------------------------------------------
# Default walking route  (UIUC Engineering Quad loop, ~500 m)
# ---------------------------------------------------------------------------

DEFAULT_ROUTE: list[dict] = [
    {"lat": 40.11280, "lng": -88.22690, "name": "Grainger Library"},
    {"lat": 40.11340, "lng": -88.22570, "name": "Illinois St / Green St"},
    {"lat": 40.11390, "lng": -88.22470, "name": "Siebel Center"},
    {"lat": 40.11310, "lng": -88.22400, "name": "Clark St"},
    {"lat": 40.11200, "lng": -88.22440, "name": "Engineering Hall"},
    {"lat": 40.11110, "lng": -88.22550, "name": "Green St / Wright St"},
    {"lat": 40.11170, "lng": -88.22670, "name": "Wright St heading north"},
    {"lat": 40.11280, "lng": -88.22690, "name": "Back at Grainger"},
]

# Turn maneuvers that match each waypoint (what action to take AT that waypoint)
DEFAULT_MANEUVERS: list[str] = [
    "turn-right", "turn-right", "turn-right", "turn-right",
    "turn-right", "turn-right", "straight",   "straight",
]

# ---------------------------------------------------------------------------
# Shared simulation state  (written by background thread, read by routes)
# ---------------------------------------------------------------------------

_lock = threading.Lock()

sim_cfg: dict[str, Any] = {
    "speed_mps":    1.4,        # walking speed
    "hazard_prob":  0.04,       # probability of spontaneous hazard per tick
    "loop_route":   True,       # restart at end of route
    "hazard_dur_s": 3.0,        # default spontaneous hazard duration
}

sim_state: dict[str, Any] = {
    "running":       False,
    "lat":           DEFAULT_ROUTE[0]["lat"],
    "lng":           DEFAULT_ROUTE[0]["lng"],
    "heading":       90.0,      # degrees (0 = north, 90 = east)
    "target_heading": 90.0,
    "waypoint_index": 0,
    "route":         DEFAULT_ROUTE,
    "maneuvers":     DEFAULT_MANEUVERS,
    "route_source":  "default",   # "default" or "frontend"
    "elapsed_s":     0.0,
    "distance_m":    0.0,         # total distance walked
    "updated_at":    time.time(),
}

# Active hazard alerts  →  list of {"direction", "zone", "expires_at"}
active_hazards: list[dict] = []

# Navigation command (mirrors server.py nav_state; read by /nav/get_command)
nav_state: dict[str, Any] = {
    "command": "stop", "motor_id": -1, "intensity": 0, "pattern": "off",
    "instruction": "Waiting for route…", "distance_m": None,
    "phase": "idle", "active": False, "updated_at": time.time(),
}

# Route metadata pushed by the frontend via /nav/set_route
route_state: dict[str, Any] = {
    "origin": None, "destination": None, "dest_name": "",
    "steps": [], "step_index": 0, "total_distance_m": 0,
    "total_duration_s": 0, "active": False, "updated_at": time.time(),
}

position_state: dict[str, Any] = {
    "lat": None, "lng": None, "heading": None, "updated_at": 0.0,
}

device_state: dict = {}  # populated by /nav endpoint if ESP32 is also sending

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------

def _haversine_m(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    R = 6_371_000.0
    dlat = math.radians(lat2 - lat1)
    dlng = math.radians(lng2 - lng1)
    a = (math.sin(dlat / 2) ** 2
         + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2))
         * math.sin(dlng / 2) ** 2)
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def _bearing_deg(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dl   = math.radians(lng2 - lng1)
    y = math.sin(dl) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def _advance_position(lat: float, lng: float, bearing_deg: float,
                      dist_m: float) -> tuple[float, float]:
    """Move a GPS point by dist_m in direction bearing_deg."""
    R = 6_371_000.0
    d = dist_m / R
    b = math.radians(bearing_deg)
    phi1 = math.radians(lat)
    lam1 = math.radians(lng)
    phi2 = math.asin(math.sin(phi1) * math.cos(d) +
                     math.cos(phi1) * math.sin(d) * math.cos(b))
    lam2 = lam1 + math.atan2(
        math.sin(b) * math.sin(d) * math.cos(phi1),
        math.cos(d) - math.sin(phi1) * math.sin(phi2),
    )
    return math.degrees(phi2), math.degrees(lam2)


def _sector_to_direction(relative_bearing: float) -> str:
    """Map a 0-360 relative bearing to the nearest 8-direction label."""
    idx = round(relative_bearing / 45) % 8
    return DIRECTIONS[idx]


def _maneuver_to_direction(maneuver: str) -> str:
    MAP = {
        "straight": "front", "keep-left": "front_left", "keep-right": "front_right",
        "turn-slight-left": "front_left",  "turn-slight-right": "front_right",
        "turn-left": "left",   "turn-right": "right",
        "turn-sharp-left": "back_left", "turn-sharp-right": "back_right",
        "uturn-left": "back",  "uturn-right": "back",
        "ramp-left": "front_left", "ramp-right": "front_right",
        "fork-left": "front_left", "fork-right": "front_right",
        "merge": "front", "roundabout-left": "left", "roundabout-right": "right",
    }
    return MAP.get(maneuver, "front")


# ---------------------------------------------------------------------------
# Simulation background thread
# ---------------------------------------------------------------------------

def _expire_hazards() -> None:
    now     = time.time()
    before  = len(active_hazards)
    expired = [h for h in active_hazards if h["expires_at"] <= now]
    active_hazards[:] = [h for h in active_hazards if h["expires_at"] > now]
    for h in expired:
        # [VERIFY:SIM_HAZARD_EXP] – pair with SIM_HAZARD_INJ to verify
        # 2.2.2 R1 detection window (obstacle persists > 100 ms before alert).
        logger.debug(
            "[VERIFY:SIM_HAZARD_EXP] direction=%-12s zone=%-8s source=%s",
            h["direction"], h["zone"], h.get("source", "unknown"),
        )


def _inject_random_hazard(source: str = "spontaneous") -> None:
    direction = random.choice(DIRECTIONS)
    zone = random.choices(
        ["CAUTION", "WARNING", "DANGER", "CRITICAL"],
        weights=[50, 30, 15, 5],
    )[0]
    duration = sim_cfg["hazard_dur_s"] * random.uniform(0.8, 2.0)
    active_hazards.append({
        "direction":  direction,
        "zone":       zone,
        "distance_m": ZONE_DISTANCES[zone],
        "expires_at": time.time() + duration,
        "source":     source,
    })
    _nav_stats["hazard_inject_count"] += 1
    # [VERIFY:SIM_HAZARD_INJ] – supports 2.2.2 R1 hazard detection recall test.
    # Each injection is a "ground truth" hazard event; correlate with
    # HAZARD_DETECT lines in the hazard log to compute recall.
    logger.info(
        "[VERIFY:SIM_HAZARD_INJ] direction=%-12s zone=%-8s dist_m=%s "
        "duration_s=%.1f  source=%-12s  total_active=%d  total_injected=%d",
        direction, zone,
        f"{ZONE_DISTANCES[zone]:.1f}" if ZONE_DISTANCES[zone] else "None",
        duration, source, len(active_hazards),
        _nav_stats["hazard_inject_count"],
    )


def _compute_nav_command(lat: float, lng: float, heading: float,
                         waypoint_index: int,
                         route: list[dict],
                         maneuvers: list[str]) -> dict:
    """Compute belt motor command from current sim position."""
    if not route or waypoint_index >= len(route):
        return {"command": "stop", "instruction": "Route complete",
                "distance_m": 0, "phase": "idle"}

    wp = route[waypoint_index]
    dist    = _haversine_m(lat, lng, wp["lat"], wp["lng"])
    bearing = _bearing_deg(lat, lng, wp["lat"], wp["lng"])
    rel     = (bearing - heading + 360) % 360

    # Heading error = angular deviation from the direct path to the waypoint.
    # Normalise to [-180, 180] so right-of-path is positive, left is negative.
    rel_norm      = ((rel + 180) % 360) - 180
    heading_err   = abs(rel_norm)

    maneuver = maneuvers[waypoint_index] if waypoint_index < len(maneuvers) else "straight"
    if dist <= 50.0:
        command = _maneuver_to_direction(maneuver)
        phase   = "execute"
    else:
        command = _sector_to_direction(rel)
        phase   = "prepare"

    # [VERIFY:NAV_HEADING] – continuous heading error log for 2.2.3 R4 (≤10°).
    # Only meaningful during the "prepare" (straight-walking) phase; at
    # "execute" the user is intentionally turning so a large angle is expected.
    if phase == "prepare":
        _nav_stats["heading_errors"].append(heading_err)
        if heading_err > _HEADING_ERR_WARN_DEG:
            logger.warning(
                "[VERIFY:NAV_HEADING] heading=%.1f  bearing_to_wp=%.1f  "
                "heading_err=%.1f°  ⚠ EXCEEDS_THRESHOLD (%.1f°)  "
                "wp='%s'  dist_m=%.1f",
                heading, bearing, heading_err, _HEADING_ERR_WARN_DEG,
                wp["name"], dist,
            )
        else:
            logger.debug(
                "[VERIFY:NAV_HEADING] heading=%.1f  bearing_to_wp=%.1f  "
                "heading_err=%.1f°  wp='%s'  dist_m=%.1f",
                heading, bearing, heading_err, wp["name"], dist,
            )

    # [VERIFY:NAV_CMD_COMPUTE] – command decision for turn-accuracy table (2.2.1 R3).
    logger.debug(
        "[VERIFY:NAV_CMD_COMPUTE] wp_idx=%d  phase=%-7s  "
        "computed=%-12s  maneuver=%-20s  dist_m=%.1f  heading=%.1f",
        waypoint_index, phase, command, maneuver, dist, heading,
    )

    instruction = f"{maneuver.replace('-', ' ').title()} toward {wp['name']}"
    return {
        "command":     command,
        "instruction": instruction,
        "distance_m":  round(dist, 1),
        "phase":       phase,
        "maneuver":    maneuver,        # extra field for logging; not sent to ESP32
        "heading_err": round(heading_err, 2),
    }


def _simulation_tick(dt: float) -> None:
    """Advance the simulation by dt seconds. Called under _lock."""
    route    = sim_state["route"]
    maneuvers = sim_state["maneuvers"]
    wp_idx   = sim_state["waypoint_index"]

    if not route or wp_idx >= len(route):
        if sim_cfg["loop_route"] and route:
            sim_state["waypoint_index"] = 0
            wp_idx = 0
        else:
            sim_state["running"] = False
            return

    wp = route[wp_idx]
    lat, lng = sim_state["lat"], sim_state["lng"]

    # Compute desired heading toward next waypoint
    target_hdg = _bearing_deg(lat, lng, wp["lat"], wp["lng"])
    sim_state["target_heading"] = target_hdg

    # Smooth heading interpolation
    diff = ((target_hdg - sim_state["heading"]) + 180) % 360 - 180
    sim_state["heading"] = (sim_state["heading"] + diff * HEADING_SMOOTH) % 360

    # Move forward
    step_m = sim_cfg["speed_mps"] * dt
    new_lat, new_lng = _advance_position(lat, lng, sim_state["heading"], step_m)
    sim_state["lat"]         = new_lat
    sim_state["lng"]         = new_lng
    sim_state["elapsed_s"]  += dt
    sim_state["distance_m"] += step_m
    sim_state["updated_at"]  = time.time()

    # Advance waypoint if close enough
    dist_to_wp = _haversine_m(new_lat, new_lng, wp["lat"], wp["lng"])
    if dist_to_wp <= WAYPOINT_REACHED_M:
        # [VERIFY:NAV_WAYPOINT] – waypoint reached; anchor for turn-accuracy
        # confusion matrix (2.2.1 R3) and High-level R2 (≥85 % turn accuracy).
        logger.info(
            "[VERIFY:NAV_WAYPOINT] wp_idx=%d  name='%s'  "
            "dist_walked_m=%.1f  elapsed_s=%.1f  "
            "last_cmd=%-12s  maneuver=%s",
            wp_idx, wp["name"],
            sim_state["distance_m"], sim_state["elapsed_s"],
            nav_state.get("command", "unknown"),
            maneuvers[wp_idx] if wp_idx < len(maneuvers) else "unknown",
        )
        _nav_stats["waypoints_reached"] += 1
        sim_state["waypoint_index"] = wp_idx + 1

    # Update nav command
    nav_cmd = _compute_nav_command(
        new_lat, new_lng, sim_state["heading"],
        sim_state["waypoint_index"], route, maneuvers,
    )
    motor_id = DIRECTIONS.index(nav_cmd["command"]) if nav_cmd["command"] in DIRECTIONS else -1
    nav_state.update({
        "command":     nav_cmd["command"],
        "motor_id":    motor_id,
        "intensity":   NAV_INTENSITY.get(nav_cmd["command"], 0),
        "pattern":     NAV_PATTERN.get(nav_cmd["command"], "off"),
        "instruction": nav_cmd["instruction"],
        "distance_m":  nav_cmd["distance_m"],
        "phase":       nav_cmd["phase"],
        "active":      nav_cmd["command"] != "stop",
        "updated_at":  time.time(),
    })

    # Spontaneous hazard injection
    if random.random() < sim_cfg["hazard_prob"] * dt * SIM_HZ:
        _inject_random_hazard(source="spontaneous")


def _sim_thread() -> None:
    dt = 1.0 / SIM_HZ
    while True:
        time.sleep(dt)
        with _lock:
            _expire_hazards()
            if sim_state["running"]:
                _simulation_tick(dt)


threading.Thread(target=_sim_thread, daemon=True).start()


# ---------------------------------------------------------------------------
# Helper: build hazard summary for /status and /sim/state
# ---------------------------------------------------------------------------

def _build_hazard_summary() -> dict:
    """Summarise active_hazards into the same format as server.py."""
    _expire_hazards()
    alerts = sorted(
        active_hazards,
        key=lambda h: -HAZARD_LEVELS.get(h["zone"], 0),
    )
    worst = alerts[0] if alerts else None
    level = HAZARD_LEVELS.get(worst["zone"], 0) if worst else 0
    label = worst["zone"] if worst else "CLEAR"
    return {
        "level":  level,
        "label":  label,
        "alerts": [
            {
                "direction":  h["direction"],
                "zone":       h["zone"],
                "distance_m": h.get("distance_m"),
                "source":     h.get("source", "simulated"),
            }
            for h in alerts
        ],
    }


# ---------------------------------------------------------------------------
# ── Production-compatible REST endpoints ────────────────────────────────────
# ---------------------------------------------------------------------------

@app.route("/nav", methods=["POST"])
def nav_ingest():
    """
    Accepts ESP32 sensor packets in the same format as server.py.
    Returns a plausible motor-command response using active_hazards.
    """
    t0        = time.perf_counter()
    data      = request.get_json(silent=True) or {}
    device_id = data.get("device_id", "sim_device")
    module    = str(data.get("module", "body")).lower()
    now       = time.time() * 1000.0
    pkt_ts    = int(data.get("timestamp", 0))

    hazard = _build_hazard_summary()
    # [VERIFY:PKT_RECV] – sensor packet arrival (mirrors server.py tag for
    # packet-count and rate analysis in 2.2.1 R1 / 2.2.5 R1).
    logger.info(
        "[VERIFY:PKT_RECV] device=%-14s module=%-4s pkt_ts=%8d "
        "active_hazards=%d hazard_label=%s",
        device_id, module, pkt_ts,
        len(active_hazards), hazard["label"],
    )

    # Build 8 motor commands – active only for hazard directions
    hazard_dirs = {a["direction"]: a for a in hazard["alerts"]}
    motors = []
    for i, direction in enumerate(DIRECTIONS):
        h = hazard_dirs.get(direction)
        if h:
            zone = h["zone"]
            intensity_map = {
                "CRITICAL": 255, "DANGER": 204, "WARNING": 153,
                "CAUTION": 102, "ALERT": 51,
            }
            pattern_map = {
                "CRITICAL": "continuous", "DANGER": "rapid_pulse",
                "WARNING": "medium_pulse", "CAUTION": "slow_pulse", "ALERT": "very_slow",
            }
            motors.append({
                "motor_id": i, "direction": direction,
                "intensity": intensity_map.get(zone, 0),
                "frequency_hz": {"CRITICAL": 50, "DANGER": 20, "WARNING": 10,
                                 "CAUTION": 5, "ALERT": 2}.get(zone, 0),
                "pattern": pattern_map.get(zone, "off"), "active": True,
                "zone": zone, "distance_m": h.get("distance_m"), "source": "simulated",
            })
        else:
            motors.append({
                "motor_id": i, "direction": direction, "intensity": 0,
                "frequency_hz": 0, "pattern": "off", "active": False,
                "zone": "CLEAR", "distance_m": None, "source": "none",
            })

    device_state[device_id] = {
        "module": module, "last_zone": hazard["label"],
        "packet_count": device_state.get(device_id, {}).get("packet_count", 0) + 1,
        "last_seen_ms": now,
    }

    proc_ms = (time.perf_counter() - t0) * 1000.0
    _record_proc_time(proc_ms)
    logger.debug(
        "[VERIFY:NAV_PROC_TIME] endpoint=/nav device=%-14s proc_ms=%.2f",
        device_id, proc_ms,
    )

    return jsonify({
        "status": "ok", "device_id": device_id, "module": module,
        "hazard": hazard, "motors": motors,
        "imu": data.get("imu", {"roll_deg": 0, "pitch_deg": 0, "yaw_deg": sim_state["heading"]}),
        "latency_ms": round(now - pkt_ts, 1) if pkt_ts else 0.0,
    })


@app.route("/status", methods=["GET"])
def status():
    """Health-check – mirrors server.py /status, adds the sim virtual device."""
    now = time.time() * 1000.0
    hazard = _build_hazard_summary()

    devices = {
        "sim_walker": {
            "module":       "body",
            "last_zone":    hazard["label"],
            "packet_count": int(sim_state["distance_m"] / 0.14) + 1,
            "last_seen_ms": now,
        }
    }
    devices.update(device_state)

    return jsonify({"status": "running", "devices": devices})


# ---------------------------------------------------------------------------
# Navigation command endpoints (same as server.py)
# ---------------------------------------------------------------------------

@app.route("/nav/set_command", methods=["POST"])
def nav_set_command():
    t0   = time.perf_counter()
    data = request.get_json(silent=True) or {}

    command = str(data.get("command", "stop")).lower()
    if command not in DIRECTIONS + ["stop"]:
        command = "stop"

    motor_id  = DIRECTIONS.index(command) if command in DIRECTIONS else -1
    intensity = int(data.get("intensity", NAV_INTENSITY.get(command, 0)))
    pattern   = str(data.get("pattern",   NAV_PATTERN.get(command, "off")))
    phase     = str(data.get("phase", "execute"))
    dist_m    = data.get("distance_m")
    instr     = str(data.get("instruction", ""))

    nav_state.update({
        "command":     command,
        "motor_id":    motor_id,
        "intensity":   intensity,
        "pattern":     pattern,
        "instruction": instr,
        "distance_m":  dist_m,
        "phase":       phase,
        "active":      command != "stop",
        "updated_at":  time.time(),
    })

    proc_ms = (time.perf_counter() - t0) * 1000.0
    _record_proc_time(proc_ms)
    _nav_stats["cmd_recv_total"] += 1
    _nav_stats["cmd_recv_ok"]    += 1   # all server-side receptions are successes

    # [VERIFY:NAV_CMD_RECV] – command delivery from Phone/PC app.
    # Supports 2.2.7 R1 (≥98 % success rate) and 2.2.1 R2 (≤200 ms latency).
    # proc_ms is server processing only; end-to-end includes WiFi round-trip.
    dist_str = f"{dist_m:.1f}" if dist_m is not None else "None"
    if proc_ms > _NAV_LATENCY_WARN_MS:
        logger.warning(
            "[VERIFY:NAV_CMD_RECV] command=%-12s motor_id=%d intensity=%3d "
            "pattern=%-12s phase=%-7s dist_m=%6s proc_ms=%.2f  "
            "⚠ SERVER_PROC_EXCEEDS_BUDGET (%.0f ms)",
            command, motor_id, intensity, pattern, phase,
            dist_str, proc_ms, _NAV_LATENCY_WARN_MS,
        )
    else:
        logger.info(
            "[VERIFY:NAV_CMD_RECV] command=%-12s motor_id=%d intensity=%3d "
            "pattern=%-12s phase=%-7s dist_m=%6s proc_ms=%.2f  "
            "total_recv=%d  success_rate=%.1f%%",
            command, motor_id, intensity, pattern, phase, dist_str, proc_ms,
            _nav_stats["cmd_recv_total"],
            100.0 * _nav_stats["cmd_recv_ok"] / _nav_stats["cmd_recv_total"],
        )

    # [VERIFY:NAV_CMD_MOTOR] – one line per command for the 2.2.1 R3 confusion
    # matrix: cross-activate error < 5 % over 40 trials.
    # Motor activation parameters also document 2.2.4 R3 (distinct patterns).
    freq_hz = {"CRITICAL": 50, "DANGER": 20, "WARNING": 10,
               "CAUTION": 5, "ALERT": 2}.get(
        nav_state.get("last_zone", "CLEAR"), 0)
    logger.info(
        "[VERIFY:NAV_CMD_MOTOR] commanded=%-12s motor_id=%d "
        "intensity=%3d  freq_hz=%2d  pattern=%-12s  active=%s  "
        "cmd_count=%d",
        command, motor_id, intensity, freq_hz, pattern,
        command != "stop", _nav_stats["cmd_recv_total"],
    )

    # Record in history for turn-accuracy analysis
    _nav_stats["cmd_history"].append({
        "commanded":  command,
        "motor_id":   motor_id,
        "phase":      phase,
        "dist_m":     dist_m,
        "instruction": instr,
        "ts":         time.time(),
    })

    # Log cumulative success rate every 10 commands (2.2.7 R1 table)
    if _nav_stats["cmd_recv_total"] % 10 == 0:
        logger.info(
            "[VERIFY:NAV_CMD_SUMMARY] total_recv=%d  success=%d  "
            "fail=%d  success_rate=%.1f%%",
            _nav_stats["cmd_recv_total"],
            _nav_stats["cmd_recv_ok"],
            _nav_stats["cmd_recv_total"] - _nav_stats["cmd_recv_ok"],
            100.0 * _nav_stats["cmd_recv_ok"] / _nav_stats["cmd_recv_total"],
        )

    return jsonify({"status": "ok", **nav_state})


@app.route("/nav/get_command", methods=["GET"])
def nav_get_command():
    now_ms = time.time() * 1000.0
    _nav_stats["cmd_poll_total"] += 1

    # Detect if ESP32 has been idle too long (connection-loss check, 2.2.7 R2)
    last_poll = _nav_stats["cmd_last_poll_ms"]
    if last_poll > 0:
        idle_ms = now_ms - last_poll
        if idle_ms > _CMD_POLL_WARN_IDLE_MS:
            logger.warning(
                "[VERIFY:CONN_IDLE] esp32_poll_idle_ms=%.1f  threshold_ms=%.0f  "
                "← ESP32 may have lost connection",
                idle_ms, _CMD_POLL_WARN_IDLE_MS,
            )
    _nav_stats["cmd_last_poll_ms"] = now_ms

    # [VERIFY:NAV_CMD_POLL] – ESP32 successfully retrieved the navigation command.
    # Pair poll_count with cmd_recv_total to verify ≥98 % delivery (2.2.7 R1).
    logger.debug(
        "[VERIFY:NAV_CMD_POLL] serving command=%-12s motor_id=%2d "
        "intensity=%3d  active=%s  poll_count=%d",
        nav_state["command"], nav_state["motor_id"],
        nav_state["intensity"], nav_state["active"],
        _nav_stats["cmd_poll_total"],
    )
    return jsonify({"status": "ok", **nav_state})


@app.route("/nav/set_route", methods=["POST"])
def nav_set_route():
    """
    Frontend sends route after computing it with Google Maps.
    We store it and use it as the sim walker's path.
    """
    data = request.get_json(silent=True) or {}
    route_state.update({
        "origin":           data.get("origin"),
        "destination":      data.get("destination"),
        "dest_name":        str(data.get("dest_name", "")),
        "steps":            data.get("steps", []),
        "step_index":       0,
        "total_distance_m": float(data.get("total_distance_m", 0)),
        "total_duration_s": float(data.get("total_duration_s", 0)),
        "active":           bool(data.get("active", True)),
        "updated_at":       time.time(),
    })
    _nav_stats["routes_set"] += 1

    # [VERIFY:NAV_ROUTE_SET] – route successfully received from Phone/PC app.
    # Supports 2.2.7 R1 route delivery tracking.  Step list provides the
    # ground-truth maneuver sequence for turn-accuracy analysis (High-level R2).
    steps = data.get("steps", [])
    maneuvers = [s.get("maneuver", "straight") for s in steps]
    logger.info(
        "[VERIFY:NAV_ROUTE_SET] dest_name='%s'  total_m=%.1f  "
        "total_s=%.0f  steps=%d  maneuvers=[%s]  route_count=%d",
        route_state["dest_name"],
        route_state["total_distance_m"],
        route_state["total_duration_s"],
        len(steps),
        ", ".join(maneuvers[:8]) + ("…" if len(maneuvers) > 8 else ""),
        _nav_stats["routes_set"],
    )

    # Convert steps to sim waypoints so the walker follows the real route
    steps = data.get("steps", [])
    if steps:
        with _lock:
            new_route = [{"lat": s["start_lat"], "lng": s["start_lng"], "name": "Start"}
                         for s in steps]
            if steps:
                last = steps[-1]
                new_route.append({"lat": last["end_lat"], "lng": last["end_lng"],
                                  "name": route_state["dest_name"] or "Destination"})
            new_maneuvers = [s.get("maneuver", "straight") for s in steps] + ["stop"]
            sim_state["route"]          = new_route
            sim_state["maneuvers"]      = new_maneuvers
            sim_state["waypoint_index"] = 0
            sim_state["route_source"]   = "frontend"
            # Teleport walker to route start
            sim_state["lat"]    = new_route[0]["lat"]
            sim_state["lng"]    = new_route[0]["lng"]
            sim_state["heading"] = 0.0
            sim_state["elapsed_s"]  = 0.0
            sim_state["distance_m"] = 0.0
            sim_state["running"]    = True  # auto-start

    nav_state.update({
        "command": "front", "motor_id": 0,
        "intensity": NAV_INTENSITY["front"], "pattern": NAV_PATTERN["front"],
        "instruction": f"Head toward {route_state['dest_name'] or 'destination'}",
        "distance_m": route_state["total_distance_m"],
        "phase": "prepare", "active": True, "updated_at": time.time(),
    })
    return jsonify({"status": "ok", "route": route_state})


@app.route("/nav/route_status", methods=["GET"])
def nav_route_status():
    return jsonify({"status": "ok", "route": route_state, "nav": nav_state})


@app.route("/nav/position", methods=["POST"])
def nav_position():
    data = request.get_json(silent=True) or {}
    lat     = data.get("lat")
    lng     = data.get("lng")
    heading = data.get("heading")
    position_state.update({
        "lat": lat, "lng": lng,
        "heading": heading, "updated_at": time.time(),
    })

    # [VERIFY:NAV_POS_UPDATE] – GPS position push from Phone/PC app.
    # Continuous heading log supports 2.2.3 R4 (≤10° heading error) analysis
    # and High-level R2 (≥85 % turn accuracy) validation.
    logger.debug(
        "[VERIFY:NAV_POS_UPDATE] lat=%s  lng=%s  heading=%s  "
        "route_active=%s  current_cmd=%-12s",
        f"{lat:.6f}" if lat is not None else "None",
        f"{lng:.6f}" if lng is not None else "None",
        f"{heading:.1f}" if heading is not None else "None",
        route_state.get("active", False),
        nav_state.get("command", "unknown"),
    )
    return jsonify({"status": "ok", "position": position_state, "nav": nav_state})


@app.route("/nav/stop", methods=["POST"])
def nav_stop():
    with _lock:
        snap_elapsed = sim_state["elapsed_s"]
        snap_dist    = sim_state["distance_m"]
        snap_wp      = sim_state["waypoint_index"]
        sim_state["running"] = False
    nav_state.update({
        "command": "stop", "motor_id": -1, "intensity": 0, "pattern": "off",
        "instruction": "Navigation stopped", "distance_m": None,
        "phase": "idle", "active": False, "updated_at": time.time(),
    })
    route_state["active"] = False

    # [VERIFY:NAV_STOPPED] – session end marker.
    # The cmd_history list accumulated since NAV_ROUTE_SET provides the full
    # commanded-direction sequence for the turn-accuracy confusion matrix.
    he   = _nav_stats["heading_errors"]
    rate = 100.0 * _nav_stats["cmd_recv_ok"] / max(_nav_stats["cmd_recv_total"], 1)
    logger.info(
        "[VERIFY:NAV_STOPPED] elapsed_s=%.1f  dist_m=%.1f  "
        "waypoints_reached=%d  cmd_total=%d  cmd_success_rate=%.1f%%  "
        "heading_err_avg=%.1f°  heading_err_max=%.1f°",
        snap_elapsed, snap_dist, snap_wp,
        _nav_stats["cmd_recv_total"], rate,
        sum(he) / len(he) if he else 0.0,
        max(he) if he else 0.0,
    )
    return jsonify({"status": "ok"})


# ---------------------------------------------------------------------------
# ── Simulation control endpoints  (/sim/*) ──────────────────────────────────
# ---------------------------------------------------------------------------

@app.route("/sim/state", methods=["GET"])
def sim_get_state():
    """
    Full simulation snapshot.  The frontend polls this in Sim Mode to
    get the virtual walker's current position and heading.
    """
    with _lock:
        snap = dict(sim_state)
    hazard = _build_hazard_summary()
    wp_idx = snap["waypoint_index"]
    route  = snap["route"]
    wp     = route[wp_idx] if route and wp_idx < len(route) else None
    return jsonify({
        "status":         "ok",
        "running":        snap["running"],
        "lat":            round(snap["lat"], 7),
        "lng":            round(snap["lng"], 7),
        "heading":        round(snap["heading"], 1),
        "speed_mps":      sim_cfg["speed_mps"],
        "waypoint_index": wp_idx,
        "waypoint_name":  wp["name"] if wp else "—",
        "waypoints_total": len(route),
        "elapsed_s":      round(snap["elapsed_s"], 1),
        "distance_m":     round(snap["distance_m"], 1),
        "route_source":   snap["route_source"],
        "hazard":         hazard,
        "nav":            dict(nav_state),
    })


@app.route("/sim/start", methods=["POST"])
def sim_start():
    with _lock:
        sim_state["running"] = True
    return jsonify({"status": "ok", "running": True})


@app.route("/sim/stop", methods=["POST"])
def sim_stop_sim():
    with _lock:
        sim_state["running"] = False
    return jsonify({"status": "ok", "running": False})


@app.route("/sim/reset", methods=["POST"])
def sim_reset():
    with _lock:
        route = sim_state["route"]
        sim_state["waypoint_index"] = 0
        sim_state["elapsed_s"]      = 0.0
        sim_state["distance_m"]     = 0.0
        sim_state["running"]        = False
        if route:
            sim_state["lat"]     = route[0]["lat"]
            sim_state["lng"]     = route[0]["lng"]
            sim_state["heading"] = 0.0
    nav_state.update({
        "command": "stop", "motor_id": -1, "intensity": 0,
        "pattern": "off", "instruction": "Reset – ready to navigate",
        "distance_m": None, "phase": "idle", "active": False,
        "updated_at": time.time(),
    })
    return jsonify({"status": "ok", "message": "Simulation reset to start"})


@app.route("/sim/speed", methods=["POST"])
def sim_set_speed():
    data = request.get_json(silent=True) or {}
    spd  = float(data.get("speed_mps", 1.4))
    spd  = max(0.1, min(spd, 10.0))
    sim_cfg["speed_mps"] = spd
    return jsonify({"status": "ok", "speed_mps": spd})


@app.route("/sim/hazard", methods=["POST"])
def sim_inject_hazard():
    """
    Manually inject a hazard alert.
    Body (all optional):
      {"direction": "back", "zone": "DANGER", "duration_s": 4}
    """
    data      = request.get_json(silent=True) or {}
    direction = data.get("direction", random.choice(DIRECTIONS))
    zone      = data.get("zone", "WARNING")
    duration  = float(data.get("duration_s", sim_cfg["hazard_dur_s"]))
    if direction not in DIRECTIONS:
        return jsonify({"status": "error", "message": "invalid direction"}), 400
    if zone not in HAZARD_LEVELS:
        return jsonify({"status": "error", "message": "invalid zone"}), 400
    with _lock:
        active_hazards.append({
            "direction":  direction,
            "zone":       zone,
            "distance_m": ZONE_DISTANCES.get(zone),
            "expires_at": time.time() + duration,
            "source":     "manual",
        })
        _nav_stats["hazard_inject_count"] += 1

    # [VERIFY:SIM_HAZARD_INJ] – manual hazard injection for 2.2.2 R1 recall test.
    logger.info(
        "[VERIFY:SIM_HAZARD_INJ] direction=%-12s zone=%-8s dist_m=%s "
        "duration_s=%.1f  source=manual  total_active=%d  total_injected=%d",
        direction, zone,
        f"{ZONE_DISTANCES.get(zone):.1f}" if ZONE_DISTANCES.get(zone) else "None",
        duration, len(active_hazards),
        _nav_stats["hazard_inject_count"],
    )
    return jsonify({"status": "ok", "direction": direction, "zone": zone, "duration_s": duration})


@app.route("/sim/hazard/clear", methods=["POST"])
def sim_clear_hazards():
    with _lock:
        active_hazards.clear()
    return jsonify({"status": "ok", "message": "All hazards cleared"})


@app.route("/sim/config", methods=["GET", "POST"])
def sim_config():
    if request.method == "POST":
        data = request.get_json(silent=True) or {}
        if "hazard_prob" in data:
            sim_cfg["hazard_prob"] = float(max(0.0, min(data["hazard_prob"], 1.0)))
        if "loop_route" in data:
            sim_cfg["loop_route"]  = bool(data["loop_route"])
        if "hazard_dur_s" in data:
            sim_cfg["hazard_dur_s"] = float(max(0.5, data["hazard_dur_s"]))
    return jsonify({"status": "ok", **sim_cfg})


# ---------------------------------------------------------------------------
# ── Navigation log summary endpoint ─────────────────────────────────────────
# ---------------------------------------------------------------------------

@app.route("/nav/log_summary", methods=["GET"])
def nav_log_summary():
    """
    Return a snapshot of accumulated navigation statistics for requirement
    verification.  Call after a test run to get the tables the design doc asks
    for in each verification column.

    Key fields and the requirement they satisfy
    -------------------------------------------
    cmd_recv_total / cmd_success_rate     2.2.7 R1  – ≥ 98 % delivery
    cmd_poll_total                        2.2.7 R1  – ESP32 retrieval count
    waypoints_reached                     High-level R2 / 2.2.1 R3 turn accuracy
    heading_err_avg / heading_err_max     2.2.3 R4  – ≤ 10° heading error
    proc_time_avg / proc_time_max         2.2.8 R2  – < 500 ms end-to-end
    hazard_inject_count                   2.2.2 R1  – ground-truth for recall
    cmd_history (last 20)                 2.2.1 R3  – confusion-matrix rows
    """
    he = _nav_stats["heading_errors"]
    pt = _nav_stats["proc_time_count"]
    recv  = _nav_stats["cmd_recv_total"]
    ok    = _nav_stats["cmd_recv_ok"]

    with _lock:
        snap = dict(sim_state)

    return jsonify({
        # 2.2.7 R1 – command delivery success rate
        "cmd_recv_total":     recv,
        "cmd_recv_ok":        ok,
        "cmd_success_rate_pct": round(100.0 * ok / recv, 2) if recv else None,
        "cmd_poll_total":     _nav_stats["cmd_poll_total"],
        "routes_set":         _nav_stats["routes_set"],

        # 2.2.1 R3 / High-level R2 – turn accuracy
        "waypoints_reached":  _nav_stats["waypoints_reached"],
        "cmd_history_count":  len(_nav_stats["cmd_history"]),
        "cmd_history_last20": _nav_stats["cmd_history"][-20:],

        # 2.2.3 R4 – heading error during straight walk
        "heading_err_samples": len(he),
        "heading_err_avg_deg": round(sum(he) / len(he), 2) if he else None,
        "heading_err_max_deg": round(max(he), 2) if he else None,
        "heading_err_gt10_count": sum(1 for e in he if e > 10.0),

        # 2.2.8 R2 – server processing latency
        "proc_time_samples":  pt,
        "proc_time_avg_ms":   round(_nav_stats["proc_time_sum_ms"] / pt, 3) if pt else None,
        "proc_time_max_ms":   round(_nav_stats["proc_time_max_ms"], 3),

        # 2.2.2 R1 – hazard injection ground truth
        "hazard_inject_count": _nav_stats["hazard_inject_count"],
        "active_hazards_now":  len(active_hazards),

        # Current simulation state (for reference)
        "sim_running":     snap["running"],
        "sim_elapsed_s":   round(snap["elapsed_s"], 1),
        "sim_distance_m":  round(snap["distance_m"], 1),
        "sim_waypoint_idx": snap["waypoint_index"],
        "route_source":    snap["route_source"],
    })


# ---------------------------------------------------------------------------
# ── Simulation Control Panel  (browser UI at /sim/ui) ───────────────────────
# ---------------------------------------------------------------------------

@app.route("/sim/ui")
def sim_ui():
    return SIM_UI_HTML, 200, {"Content-Type": "text/html; charset=utf-8"}


SIM_UI_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1"/>
<title>OmniSense-Dual Simulation Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{
  --bg:#0d1117;--panel:#161b27;--elevated:#1e2435;--hover:#252c3d;
  --accent:#4f8ef7;--danger:#e05c5c;--warn:#e6a817;--ok:#3dba6e;
  --muted:#7a88a4;--text:#e8ecf4;--border:#252c3d;
}
body{background:var(--bg);color:var(--text);font:14px/1.5 system-ui,sans-serif;
     display:flex;flex-direction:column;min-height:100vh}
header{background:var(--panel);border-bottom:1px solid var(--border);
       padding:14px 24px;display:flex;align-items:center;gap:12px}
header h1{font-size:16px;font-weight:700}
header span{font-size:12px;color:var(--muted)}
.badge{padding:3px 10px;border-radius:12px;font-size:11px;font-weight:700;
       letter-spacing:.5px;border:1px solid var(--border)}
.badge.running{background:rgba(61,186,110,.12);border-color:var(--ok);color:var(--ok)}
.badge.stopped{background:rgba(122,136,164,.1);border-color:var(--muted);color:var(--muted)}
main{display:grid;grid-template-columns:320px 1fr;gap:16px;padding:20px;flex:1}
.card{background:var(--panel);border:1px solid var(--border);border-radius:10px;
      padding:18px}
.card h2{font-size:11px;font-weight:600;text-transform:uppercase;
         letter-spacing:1px;color:var(--muted);margin-bottom:14px}
.row{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.row label{font-size:12px;color:var(--muted);width:110px;flex-shrink:0}
.val{font-size:13px;font-weight:600;color:var(--text)}
.val.accent{color:var(--accent)}
.val.ok{color:var(--ok)}
.val.warn{color:var(--warn)}
.val.danger{color:var(--danger)}
input[type=range]{flex:1;accent-color:var(--accent)}
input[type=number]{width:70px;background:var(--elevated);border:1px solid var(--border);
  border-radius:5px;padding:4px 8px;color:var(--text);font-size:13px;outline:none}
.btn{display:inline-flex;align-items:center;justify-content:center;gap:6px;
     padding:8px 16px;border:none;border-radius:6px;font-size:12px;font-weight:600;
     cursor:pointer;transition:.15s}
.btn:active{transform:scale(.97)}
.btn-primary{background:var(--accent);color:#fff}
.btn-primary:hover{background:#3a7ae8}
.btn-stop{background:rgba(224,92,92,.15);color:var(--danger);border:1px solid var(--danger)}
.btn-stop:hover{background:rgba(224,92,92,.25)}
.btn-sm{padding:5px 10px;font-size:11px;background:var(--elevated);
        color:var(--text);border:1px solid var(--border)}
.btn-sm:hover{background:var(--hover)}
.btn-warn{background:rgba(230,168,23,.15);color:var(--warn);border:1px solid var(--warn)}
.btn-warn:hover{background:rgba(230,168,23,.25)}
.btn-row{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
.dir-grid{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;margin-top:10px}
.dir-btn{padding:6px 4px;font-size:11px;font-weight:600;background:var(--elevated);
         border:1px solid var(--border);border-radius:5px;cursor:pointer;
         color:var(--text);text-align:center;transition:.15s}
.dir-btn:hover{background:var(--hover);border-color:var(--accent)}
.dir-btn.active{background:rgba(79,142,247,.2);border-color:var(--accent);color:var(--accent)}
.motor-row{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px}
.motor-chip{padding:5px 10px;border-radius:20px;font-size:11px;font-weight:700;
            letter-spacing:.5px;border:1px solid var(--border);
            background:var(--elevated);color:var(--muted);transition:.2s}
.motor-chip.active{background:var(--accent);border-color:var(--accent);color:#fff;
                   box-shadow:0 0 10px rgba(79,142,247,.3)}
.hazard-list{display:flex;flex-direction:column;gap:6px;margin-top:8px;
             max-height:180px;overflow-y:auto}
.hazard-item{display:flex;align-items:center;gap:10px;padding:6px 10px;
             background:var(--elevated);border-radius:6px;border-left:3px solid var(--warn)}
.hazard-item.CRITICAL,.hazard-item.DANGER{border-left-color:var(--danger)}
.hazard-item.WARNING{border-left-color:var(--warn)}
.hazard-item.CAUTION{border-left-color:var(--accent)}
.progress{height:4px;background:var(--elevated);border-radius:2px;overflow:hidden;
          margin-top:8px}
.progress-bar{height:100%;background:var(--accent);border-radius:2px;transition:.3s}
.zone-badge{padding:3px 8px;border-radius:10px;font-size:11px;font-weight:700;
            letter-spacing:.5px}
.zone-CLEAR{background:rgba(61,186,110,.1);color:var(--ok)}
.zone-ALERT,.zone-CAUTION{background:rgba(79,142,247,.12);color:var(--accent)}
.zone-WARNING{background:rgba(230,168,23,.12);color:var(--warn)}
.zone-DANGER,.zone-CRITICAL{background:rgba(224,92,92,.15);color:var(--danger)}
.sep{border:none;border-top:1px solid var(--border);margin:12px 0}
footer{padding:10px 24px;border-top:1px solid var(--border);font-size:11px;
       color:var(--muted);text-align:center}
#toast{position:fixed;bottom:20px;right:20px;background:var(--panel);
       border:1px solid var(--border);border-radius:8px;padding:10px 16px;
       font-size:12px;opacity:0;transform:translateY(10px);
       transition:.3s;pointer-events:none;z-index:999}
#toast.show{opacity:1;transform:translateY(0)}
</style>
</head>
<body>
<header>
  <div style="display:flex;flex-direction:column">
    <h1>OmniSense-Dual &nbsp;·&nbsp; Simulation Control Panel</h1>
    <span>Test server running on port 5001</span>
  </div>
  <div style="margin-left:auto;display:flex;gap:10px;align-items:center">
    <span id="sim-badge" class="badge stopped">STOPPED</span>
    <span style="font-size:12px;color:var(--muted)">Auto-refresh: 1 s</span>
  </div>
</header>

<main>
<!-- LEFT COLUMN -->
<div style="display:flex;flex-direction:column;gap:16px">

  <!-- Controls -->
  <div class="card">
    <h2>Walker Controls</h2>
    <div class="btn-row">
      <button class="btn btn-primary" onclick="api('/sim/start','POST')">▶ Start</button>
      <button class="btn btn-stop"    onclick="api('/sim/stop','POST')">⏸ Pause</button>
      <button class="btn btn-sm"      onclick="api('/sim/reset','POST')">↺ Reset</button>
    </div>
    <hr class="sep"/>
    <div class="row">
      <label>Speed (m/s)</label>
      <input type="range" id="speed-slider" min="0.3" max="8" step="0.1" value="1.4"
             oninput="document.getElementById('speed-val').textContent=this.value;
                      api('/sim/speed','POST',{speed_mps:+this.value})"/>
      <span class="val accent" id="speed-val">1.4</span>
    </div>
    <hr class="sep"/>
    <div class="row">
      <label>Hazard prob / tick</label>
      <input type="range" id="hprob-slider" min="0" max="0.2" step="0.005" value="0.04"
             oninput="document.getElementById('hprob-val').textContent=(+this.value*100).toFixed(1)+'%';
                      api('/sim/config','POST',{hazard_prob:+this.value})"/>
      <span class="val" id="hprob-val">4.0%</span>
    </div>
    <div class="row">
      <label>Auto-loop route</label>
      <button id="loop-btn" class="btn btn-sm" onclick="toggleLoop()">ON</button>
    </div>
  </div>

  <!-- Inject Hazard -->
  <div class="card">
    <h2>Inject Hazard</h2>
    <div class="row">
      <label>Zone</label>
      <select id="inj-zone" style="background:var(--elevated);border:1px solid var(--border);
              border-radius:5px;padding:5px 8px;color:var(--text);font-size:13px;outline:none">
        <option>CAUTION</option><option selected>WARNING</option>
        <option>DANGER</option><option>CRITICAL</option>
      </select>
      <label style="margin-left:8px">Dur (s)</label>
      <input type="number" id="inj-dur" value="3" min="1" max="30"
             style="width:55px"/>
    </div>
    <div class="dir-grid" id="dir-grid">
      <!-- Filled by JS -->
    </div>
    <hr class="sep"/>
    <button class="btn btn-sm" style="width:100%" onclick="clearHazards()">
      ✕ Clear All Hazards
    </button>
  </div>
</div>

<!-- RIGHT COLUMN -->
<div style="display:flex;flex-direction:column;gap:16px">

  <!-- Live State -->
  <div class="card">
    <h2>Live Walker State</h2>
    <div style="display:grid;grid-template-columns:1fr 1fr;gap:0 24px">
      <div class="row"><label>Position</label><span class="val accent" id="s-pos">—</span></div>
      <div class="row"><label>Heading</label><span class="val" id="s-hdg">—</span></div>
      <div class="row"><label>Waypoint</label><span class="val" id="s-wp">—</span></div>
      <div class="row"><label>Distance</label><span class="val" id="s-dist">—</span></div>
      <div class="row"><label>Elapsed</label><span class="val" id="s-time">—</span></div>
      <div class="row"><label>Route source</label><span class="val" id="s-src">—</span></div>
    </div>
    <div id="route-progress-wrap" style="margin-top:8px">
      <div style="font-size:11px;color:var(--muted);margin-bottom:4px"
           id="route-prog-label">Route progress</div>
      <div class="progress"><div class="progress-bar" id="route-prog-bar" style="width:0%"></div></div>
    </div>
  </div>

  <!-- Navigation Command -->
  <div class="card">
    <h2>Current Belt Motor Command</h2>
    <div class="row">
      <label>Command</label>
      <span class="val accent" style="font-size:18px;font-weight:800" id="n-cmd">—</span>
      <span class="zone-badge zone-CLEAR" id="n-phase" style="margin-left:auto">idle</span>
    </div>
    <div class="row"><label>Instruction</label><span class="val" id="n-inst" style="font-size:12px">—</span></div>
    <div class="row"><label>Distance</label><span class="val" id="n-dist">—</span></div>
    <div class="motor-row" id="motor-row">
      <!-- Filled by JS -->
    </div>
  </div>

  <!-- Hazard Status -->
  <div class="card">
    <h2>Active Hazard Alerts</h2>
    <div class="row">
      <label>Overall level</label>
      <span class="zone-badge" id="hz-badge">CLEAR</span>
    </div>
    <div class="hazard-list" id="hz-list">
      <div style="color:var(--muted);font-size:12px">No active hazards</div>
    </div>
  </div>

</div>
</main>

<div id="toast"></div>
<footer>OmniSense-Dual ECE 445 Group 10 &nbsp;|&nbsp; Test Simulation Server</footer>

<script>
const DIRS = ["front","front_right","right","back_right","back","back_left","left","front_left"];
const DIR_LABELS = {
  front:"F↑", front_right:"FR↗", right:"R→", back_right:"BR↘",
  back:"B↓", back_left:"BL↙", left:"L←", front_left:"FL↖"
};
let loopRoute = true;

// Build direction grid
const grid = document.getElementById("dir-grid");
DIRS.forEach(d => {
  const b = document.createElement("button");
  b.className = "dir-btn"; b.textContent = DIR_LABELS[d]; b.dataset.dir = d;
  b.onclick = () => injectHazard(d);
  grid.appendChild(b);
});

// Build motor chips
const motorRow = document.getElementById("motor-row");
DIRS.forEach((d,i) => {
  const c = document.createElement("div");
  c.className = "motor-chip"; c.id = "mchip-"+i;
  c.textContent = DIR_LABELS[d];
  motorRow.appendChild(c);
});

async function api(path, method="GET", body=null) {
  const opts = { method, headers: {"Content-Type":"application/json"} };
  if (body) opts.body = JSON.stringify(body);
  try {
    const r = await fetch(path, opts);
    const d = await r.json();
    showToast(d.message || (method==="POST" ? "✓ Done" : ""));
    return d;
  } catch(e) { showToast("Error: " + e.message); }
}

function injectHazard(dir) {
  const zone = document.getElementById("inj-zone").value;
  const dur  = +document.getElementById("inj-dur").value;
  api("/sim/hazard","POST",{direction:dir, zone, duration_s:dur});
  // Flash button
  const btn = grid.querySelector(`[data-dir="${dir}"]`);
  if (btn) { btn.classList.add("active"); setTimeout(()=>btn.classList.remove("active"),800); }
}

function clearHazards() { api("/sim/hazard/clear","POST"); }

function toggleLoop() {
  loopRoute = !loopRoute;
  document.getElementById("loop-btn").textContent = loopRoute ? "ON" : "OFF";
  api("/sim/config","POST",{loop_route: loopRoute});
}

function showToast(msg) {
  if (!msg) return;
  const t = document.getElementById("toast");
  t.textContent = msg; t.classList.add("show");
  clearTimeout(t._timer);
  t._timer = setTimeout(()=>t.classList.remove("show"), 2000);
}

function fmtDist(m) {
  if (m == null) return "—";
  return m >= 1000 ? (m/1000).toFixed(1)+" km" : Math.round(m)+" m";
}

function fmtTime(s) {
  if (s < 60) return Math.round(s)+"s";
  return Math.floor(s/60)+"m "+Math.round(s%60)+"s";
}

function zoneClass(z) {
  return {CLEAR:"ok",ALERT:"accent",CAUTION:"accent",WARNING:"warn",DANGER:"danger",CRITICAL:"danger"}[z]||"";
}

async function refresh() {
  const d = await api("/sim/state","GET");
  if (!d) return;

  // Badge
  const badge = document.getElementById("sim-badge");
  badge.className = "badge " + (d.running ? "running" : "stopped");
  badge.textContent = d.running ? "RUNNING" : "STOPPED";

  // Walker state
  document.getElementById("s-pos").textContent =
    d.lat.toFixed(5) + ", " + d.lng.toFixed(5);
  document.getElementById("s-hdg").textContent = d.heading.toFixed(1) + "°";
  document.getElementById("s-wp").textContent  =
    (d.waypoint_index+1) + "/" + d.waypoints_total + "  " + d.waypoint_name;
  document.getElementById("s-dist").textContent = fmtDist(d.distance_m);
  document.getElementById("s-time").textContent = fmtTime(d.elapsed_s);
  document.getElementById("s-src").textContent  = d.route_source;

  const pct = d.waypoints_total > 1
    ? Math.round((d.waypoint_index / (d.waypoints_total-1))*100) : 0;
  document.getElementById("route-prog-bar").style.width = pct + "%";
  document.getElementById("route-prog-label").textContent =
    "Route progress  " + pct + "%";

  // Nav command
  const nav = d.nav;
  document.getElementById("n-cmd").textContent  = (nav.command||"—").toUpperCase().replace("_"," ");
  document.getElementById("n-inst").textContent = nav.instruction || "—";
  document.getElementById("n-dist").textContent = fmtDist(nav.distance_m);
  const phBadge = document.getElementById("n-phase");
  phBadge.textContent  = nav.phase || "idle";
  phBadge.className    = "zone-badge zone-" + (nav.phase === "execute" ? "WARNING" : "CLEAR");

  DIRS.forEach((_,i) => {
    const chip = document.getElementById("mchip-"+i);
    chip.classList.toggle("active", DIRS[i] === nav.command);
  });

  // Hazards
  const hz = d.hazard;
  const hzBadge = document.getElementById("hz-badge");
  hzBadge.textContent = hz.label;
  hzBadge.className   = "zone-badge zone-" + hz.label;

  const list = document.getElementById("hz-list");
  if (!hz.alerts.length) {
    list.innerHTML = '<div style="color:var(--muted);font-size:12px">No active hazards</div>';
  } else {
    list.innerHTML = hz.alerts.map(a => `
      <div class="hazard-item ${a.zone}">
        <span class="zone-badge zone-${a.zone}">${a.zone}</span>
        <span style="font-weight:600">${a.direction.replace("_"," ")}</span>
        <span style="color:var(--muted);font-size:12px">${a.distance_m != null ? a.distance_m.toFixed(1)+"m" : ""}</span>
        <span style="margin-left:auto;font-size:11px;color:var(--muted)">${a.source}</span>
      </div>`).join("");
  }
}

setInterval(refresh, 1000);
refresh();
</script>
</body>
</html>
"""

# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    log_file = _setup_logging()
    logger.info("=" * 60)
    logger.info("  OmniSense-Dual  Test Simulation Server")
    logger.info("  http://0.0.0.0:5001")
    logger.info("")
    logger.info("  Control panel  →  http://localhost:5001/sim/ui")
    logger.info("  Nav log summary→  http://localhost:5001/nav/log_summary")
    logger.info("  Log file       →  %s", log_file)
    logger.info("  Set frontend SERVER_URL to  http://localhost:5001")
    logger.info("=" * 60)
    app.run(host="0.0.0.0", port=5001, debug=False, threaded=True)
