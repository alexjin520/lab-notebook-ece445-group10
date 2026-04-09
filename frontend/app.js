/**
 * OmniSense-Dual  Navigation App  |  ECE 445 Group 10
 * =====================================================
 *
 * Flow:
 *  1.  User selects origin (auto-detected) + destination.
 *  2.  Google Maps Directions Service computes the walking route.
 *  3.  App tracks user position (browser Geolocation API or simulated).
 *  4.  For every position update:
 *        a. Advance through route steps based on proximity.
 *        b. Compute required turn direction (one of 8 compass sectors).
 *        c. POST the direction command to  /nav/set_command  on the Flask server.
 *        d. The body ESP32 polls  /nav/get_command  to activate the belt motor.
 *  5.  Also polls  /status  every 2 s to display live hazard alerts.
 *
 * Belt motor direction mapping (matches algorithm.py DIRECTIONS):
 *   0 front  1 front_right  2 right  3 back_right
 *   4 back   5 back_left    6 left   7 front_left
 */

"use strict";

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

const DIRECTIONS = [
  "front", "front_right", "right", "back_right",
  "back",  "back_left",   "left",  "front_left",
];

/** Degrees each motor covers (centred on its sector). */
const MOTOR_SECTOR_DEG = 360 / 8; // 45°

/** Distance threshold to consider a waypoint reached (metres). */
const WAYPOINT_REACHED_M = 15;

/** Distance to start showing the upcoming maneuver (metres). */
const PREPARE_DISTANCE_M = 50;

/** Server poll interval for hazard status (ms). */
const HAZARD_POLL_MS = 2000;

/** Position update interval when simulating (ms). */
const SIM_INTERVAL_MS = 1000;

/** Intensity for navigation commands (0-255). */
const NAV_INTENSITY = {
  front: 102, front_right: 153, right: 153, back_right: 153,
  back:  204, back_left:   153, left: 153,  front_left:  153,
  stop:    0,
};

const NAV_PATTERN = {
  front: "slow_pulse", front_right: "medium_pulse", right: "medium_pulse",
  back_right: "medium_pulse", back: "rapid_pulse", back_left: "medium_pulse",
  left: "medium_pulse", front_left: "medium_pulse", stop: "off",
};

/** Arrow SVG paths keyed by direction. */
const ARROW_SVG = {
  front: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M14 22V8"/><path d="M8 13l6-6 6 6"/></svg>`,
  front_right: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M8 20L20 8"/><path d="M12 8h8v8"/></svg>`,
  right: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M6 14h16"/><path d="M15 8l6 6-6 6"/></svg>`,
  back_right: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M8 8l12 12"/><path d="M12 20h8v-8"/></svg>`,
  back: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M14 6v16"/><path d="M8 17l6 6 6-6"/></svg>`,
  back_left: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M20 8L8 20"/><path d="M8 12v8h8"/></svg>`,
  left: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M22 14H6"/><path d="M13 8l-6 6 6 6"/></svg>`,
  front_left: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round" stroke-linejoin="round">
    <path d="M20 20L8 8"/><path d="M8 12V8h4 m0 0 0 4"/></svg>`,
  stop: `<svg viewBox="0 0 28 28" fill="none" stroke="currentColor" stroke-width="2.5" stroke-linecap="round">
    <rect x="8" y="8" width="12" height="12" rx="2"/></svg>`,
};

/** Maps Google Maps maneuver strings to our 8-direction command. */
const MANEUVER_MAP = {
  "straight":           "front",
  "keep-left":          "front_left",
  "keep-right":         "front_right",
  "turn-slight-left":   "front_left",
  "turn-slight-right":  "front_right",
  "turn-left":          "left",
  "turn-right":         "right",
  "turn-sharp-left":    "back_left",
  "turn-sharp-right":   "back_right",
  "uturn-left":         "back",
  "uturn-right":        "back",
  "ramp-left":          "front_left",
  "ramp-right":         "front_right",
  "fork-left":          "front_left",
  "fork-right":         "front_right",
  "merge":              "front",
  "roundabout-left":    "left",
  "roundabout-right":   "right",
  "ferry":              "front",
  "ferry-train":        "front",
};

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

const state = {
  map: null,
  directionsRenderer: null,
  directionsService: null,
  geocoder: null,
  placesAutocomplete: null,

  userMarker: null,
  headingMarker: null,

  currentPos: null,   // {lat, lng}
  currentHeading: 0,  // degrees (0 = north)
  watchId: null,

  route: null,        // full Google Directions result
  steps: [],          // parsed step list [{instruction, maneuver, distance_m, end_lat, end_lng}]
  stepIndex: 0,
  routeActive: false,

  simTimer: null,     // for simulated position mode
  simPathPoints: [],  // LatLng array from route polyline
  simPointIndex: 0,

  serverUrl: (window.OMNISENSE_CONFIG || {}).SERVER_URL || "http://localhost:5000",
  connected: false,
  hazardPollTimer: null,
  navSendTimer: null,

  lastCommand: null,
};

// ---------------------------------------------------------------------------
// DOM refs
// ---------------------------------------------------------------------------

const $ = id => document.getElementById(id);
const originInput     = $("origin-input");
const destInput       = $("dest-input");
const routeBtn        = $("route-btn");
const stopBtn         = $("stop-btn");
const locateBtn       = $("locate-btn");
const instructionCard = $("instruction-card");
const instructionArrow= $("instruction-arrow");
const instructionMain = $("instruction-main");
const instructionDist = $("instruction-dist");
const motorRing       = $("motor-ring");
const motorLabelRing  = $("motor-label-ring");
const motorBadge      = $("motor-command-badge");
const stepsSection    = $("steps-section");
const stepsList       = $("steps-list");
const hazardBadge     = $("hazard-badge");
const hazardLabel     = $("hazard-label");
const hazardAlerts    = $("hazard-alerts");
const serverUrlInput  = $("server-url");
const connBtn         = $("conn-btn");
const connDot         = $("conn-dot");
const connStatus      = $("conn-status");
const headingCompass  = $("heading-compass");
const centerBtn       = $("center-btn");
const tiltBtn         = $("tilt-btn");
const compassNeedle   = $("compass-needle");

// ---------------------------------------------------------------------------
// Motor ring initialisation
// ---------------------------------------------------------------------------

(function buildMotorRing() {
  const r = 72; // ring radius in px (ring wrapper is 180px, centre = 90)
  DIRECTIONS.forEach((dir, i) => {
    const angleDeg  = i * 45 - 90; // -90 so front is at top
    const angleRad  = angleDeg * Math.PI / 180;
    const cx = 90 + r * Math.cos(angleRad);
    const cy = 90 + r * Math.sin(angleRad);

    // Dot
    const dot = document.createElement("div");
    dot.className = "motor-dot";
    dot.id = `motor-dot-${i}`;
    dot.style.left = `${cx}px`;
    dot.style.top  = `${cy}px`;
    motorRing.appendChild(dot);

    // Label (slightly further out)
    const lr = 92;
    const lx = 90 + lr * Math.cos(angleRad);
    const ly = 90 + lr * Math.sin(angleRad);
    const lbl = document.createElement("div");
    lbl.className = "motor-dir-label";
    lbl.style.left = `${lx}px`;
    lbl.style.top  = `${ly}px`;
    // short labels
    lbl.textContent = {
      front:"F", front_right:"FR", right:"R", back_right:"BR",
      back:"B", back_left:"BL", left:"L", front_left:"FL"
    }[dir];
    motorLabelRing.appendChild(lbl);
  });
})();

function setActiveMotor(command) {
  DIRECTIONS.forEach((_, i) => {
    const dot = $(`motor-dot-${i}`);
    if (dot) { dot.classList.remove("active", "active-stop"); }
  });
  const idx = DIRECTIONS.indexOf(command);
  if (idx >= 0) {
    const dot = $(`motor-dot-${idx}`);
    if (dot) { dot.classList.add("active"); }
  }
  if (command === "stop") {
    // dim all with a stop colour
    DIRECTIONS.forEach((_, i) => {
      const dot = $(`motor-dot-${i}`);
      if (dot) dot.classList.add("active-stop");
    });
  }
  motorBadge.textContent = command.toUpperCase().replace("_", " ");
}

function setInstructionArrow(command) {
  instructionArrow.innerHTML = ARROW_SVG[command] || ARROW_SVG.front;
}

// ---------------------------------------------------------------------------
// Map initialisation (called by Google Maps loader)
// ---------------------------------------------------------------------------

window.initMap = function () {
  state.map = new google.maps.Map($("map"), {
    center: { lat: 40.1105, lng: -88.2272 }, // UIUC default
    zoom: 16,
    mapTypeId: "roadmap",
    styles: DARK_MAP_STYLES,
    disableDefaultUI: true,
    zoomControl: true,
    zoomControlOptions: { position: google.maps.ControlPosition.RIGHT_BOTTOM },
    fullscreenControl: false,
    streetViewControl: false,
    mapTypeControl: false,
  });

  state.directionsRenderer = new google.maps.DirectionsRenderer({
    map: state.map,
    suppressMarkers: true,
    polylineOptions: {
      strokeColor: "#4f8ef7",
      strokeWeight: 5,
      strokeOpacity: 0.9,
    },
  });

  state.directionsService = new google.maps.DirectionsService();
  state.geocoder = new google.maps.Geocoder();

  // Autocomplete for destination
  const ac = new google.maps.places.Autocomplete(destInput, {
    types: ["geocode", "establishment"],
  });
  ac.addListener("place_changed", () => {
    const place = ac.getPlace();
    if (place.geometry) {
      routeBtn.disabled = false;
      routeBtn.textContent = ""; // cleared; re-add icon + text
      routeBtn.innerHTML = `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 12h18M13 6l6 6-6 6"/></svg> Start Navigation`;
    }
  });
  state.placesAutocomplete = ac;

  destInput.addEventListener("input", () => {
    if (destInput.value.trim()) routeBtn.disabled = false;
  });

  // Draw user marker
  createUserMarker({ lat: 40.1105, lng: -88.2272 });

  // Center/3D buttons
  centerBtn.addEventListener("click", () => {
    if (state.currentPos) state.map.panTo(state.currentPos);
  });
  tiltBtn.addEventListener("click", () => {
    const t = state.map.getTilt();
    if (t > 0) { state.map.setTilt(0); tiltBtn.textContent = "3D"; }
    else        { state.map.setTilt(45); tiltBtn.textContent = "2D"; }
  });

  // Try to get user location immediately
  requestGeolocation();
};

function createUserMarker(pos) {
  if (state.userMarker) state.userMarker.setMap(null);
  state.userMarker = new google.maps.Marker({
    position: pos,
    map: state.map,
    icon: {
      path: google.maps.SymbolPath.CIRCLE,
      scale: 10,
      fillColor: "#4f8ef7",
      fillOpacity: 1,
      strokeColor: "#fff",
      strokeWeight: 2,
    },
    title: "Your Location",
    zIndex: 100,
  });
}

function updateUserMarker(pos, heading) {
  if (!state.map) return;
  const ll = new google.maps.LatLng(pos.lat, pos.lng);
  if (!state.userMarker) {
    createUserMarker(pos);
  } else {
    state.userMarker.setPosition(ll);
  }
  // Update compass needle
  if (compassNeedle) {
    compassNeedle.setAttribute("transform", `rotate(${heading}, 28, 28)`);
  }
}

// ---------------------------------------------------------------------------
// Geolocation
// ---------------------------------------------------------------------------

function requestGeolocation() {
  if (!navigator.geolocation) {
    setOriginText("Geolocation not supported");
    return;
  }
  navigator.geolocation.getCurrentPosition(
    pos => {
      const { latitude: lat, longitude: lng, heading } = pos.coords;
      handlePositionUpdate(lat, lng, heading || 0);
      setOriginText(`${lat.toFixed(5)}, ${lng.toFixed(5)}`);
      // Reverse geocode for friendly name
      if (state.geocoder) {
        state.geocoder.geocode({ location: { lat, lng } }, (res, status) => {
          if (status === "OK" && res[0]) {
            setOriginText(res[0].formatted_address.split(",").slice(0, 2).join(", "));
          }
        });
      }
    },
    err => {
      console.warn("Geolocation error:", err.message);
      setOriginText("Location unavailable");
    },
    { enableHighAccuracy: true, timeout: 8000 }
  );
}

function startWatchingPosition() {
  if (state.watchId !== null) return;
  if (!navigator.geolocation) return;
  state.watchId = navigator.geolocation.watchPosition(
    pos => {
      const { latitude: lat, longitude: lng, heading } = pos.coords;
      handlePositionUpdate(lat, lng, heading || state.currentHeading);
    },
    err => console.warn("Watch position error:", err.message),
    { enableHighAccuracy: true, maximumAge: 1000 }
  );
}

function stopWatchingPosition() {
  if (state.watchId !== null) {
    navigator.geolocation.clearWatch(state.watchId);
    state.watchId = null;
  }
}

function setOriginText(text) {
  originInput.value = text;
}

locateBtn.addEventListener("click", () => {
  requestGeolocation();
});

// ---------------------------------------------------------------------------
// Position update handler
// ---------------------------------------------------------------------------

function handlePositionUpdate(lat, lng, heading) {
  state.currentPos    = { lat, lng };
  state.currentHeading = (heading || 0) % 360;

  updateUserMarker({ lat, lng }, state.currentHeading);
  headingCompass.classList.remove("hidden");

  // Send position to server
  if (state.connected) {
    fetch(`${state.serverUrl}/nav/position`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ lat, lng, heading: state.currentHeading }),
    }).catch(() => {});
  }

  if (state.routeActive) {
    advanceRouteStep(lat, lng);
    const cmd = computeNavCommand(lat, lng, state.currentHeading);
    applyNavCommand(cmd);
    if (state.connected) sendCommandToServer(cmd);
  }
}

// ---------------------------------------------------------------------------
// Route computation
// ---------------------------------------------------------------------------

routeBtn.addEventListener("click", () => {
  if (!state.currentPos) {
    alert("Waiting for location. Please try again.");
    return;
  }
  const dest = destInput.value.trim();
  if (!dest) return;
  computeRoute(state.currentPos, dest);
});

function computeRoute(origin, destination) {
  if (!state.directionsService) return;

  routeBtn.disabled = true;
  routeBtn.textContent = "Calculating…";

  const request = {
    origin: origin,
    destination: destination,
    travelMode: google.maps.TravelMode.WALKING,
    unitSystem: google.maps.UnitSystem.METRIC,
  };

  state.directionsService.route(request, (result, status) => {
    routeBtn.disabled = false;
    routeBtn.innerHTML = `<svg width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2"><path d="M3 12h18M13 6l6 6-6 6"/></svg> Start Navigation`;

    if (status !== google.maps.DirectionsStatus.OK) {
      alert(`Could not get directions: ${status}`);
      return;
    }

    state.route = result;
    state.directionsRenderer.setDirections(result);

    // Parse steps
    const leg = result.routes[0].legs[0];
    state.steps = leg.steps.map(s => ({
      instruction: stripHtml(s.instructions),
      maneuver:    s.maneuver || "straight",
      distance_m:  s.distance.value,
      duration_s:  s.duration.value,
      start_lat:   s.start_location.lat(),
      start_lng:   s.start_location.lng(),
      end_lat:     s.end_location.lat(),
      end_lng:     s.end_location.lng(),
    }));
    state.stepIndex  = 0;
    state.routeActive = true;

    // Build sim path from route polyline
    const path = result.routes[0].overview_path;
    state.simPathPoints  = path;
    state.simPointIndex  = 0;

    // Show route details in sidebar
    renderStepsList();
    stepsSection.classList.remove("hidden");
    instructionCard.classList.remove("hidden");

    // Send route to server
    if (state.connected) sendRouteToServer(leg);

    // Start position tracking
    startWatchingPosition();

    // If geolocation not available/sim mode: simulate walking along path
    if (!navigator.geolocation || state.currentPos === null) {
      startSimulation();
    }

    // Apply first instruction immediately
    const cmd = computeNavCommand(
      state.currentPos ? state.currentPos.lat : state.steps[0].start_lat,
      state.currentPos ? state.currentPos.lng : state.steps[0].start_lng,
      state.currentHeading
    );
    applyNavCommand(cmd);
    if (state.connected) sendCommandToServer(cmd);

    // Zoom to route
    const bounds = new google.maps.LatLngBounds();
    leg.steps.forEach(s => {
      bounds.extend(s.start_location);
      bounds.extend(s.end_location);
    });
    state.map.fitBounds(bounds, { padding: 80 });
  });
}

function renderStepsList() {
  stepsList.innerHTML = "";
  state.steps.forEach((step, i) => {
    const li = document.createElement("li");
    li.className = "step-item" + (i === 0 ? " active-step" : "");
    li.id = `step-${i}`;
    const cmd = MANEUVER_MAP[step.maneuver] || "front";
    li.innerHTML = `
      <div class="step-icon">${ARROW_SVG[cmd] || ARROW_SVG.front}</div>
      <div class="step-text">${step.instruction}</div>
      <div class="step-dist">${fmtDist(step.distance_m)}</div>
    `;
    stepsList.appendChild(li);
  });
}

function highlightStep(idx) {
  const prev = stepsList.querySelector(".active-step");
  if (prev) prev.classList.remove("active-step");
  const cur = $(`step-${idx}`);
  if (cur) {
    cur.classList.add("active-step");
    cur.scrollIntoView({ block: "nearest", behavior: "smooth" });
  }
}

// ---------------------------------------------------------------------------
// Route step advancement
// ---------------------------------------------------------------------------

function advanceRouteStep(lat, lng) {
  if (!state.routeActive || state.steps.length === 0) return;

  let step = state.steps[state.stepIndex];
  if (!step) return;

  const distToEnd = haversineM(lat, lng, step.end_lat, step.end_lng);
  if (distToEnd <= WAYPOINT_REACHED_M) {
    state.stepIndex++;
    if (state.stepIndex >= state.steps.length) {
      // Arrived!
      arrivedAtDestination();
    } else {
      highlightStep(state.stepIndex);
    }
  }
}

function arrivedAtDestination() {
  state.routeActive = false;
  stopWatchingPosition();
  stopSimulation();

  instructionMain.textContent = "You have arrived!";
  instructionDist.textContent = "";
  setInstructionArrow("stop");
  setActiveMotor("stop");
  motorBadge.textContent = "ARRIVED";

  if (state.connected) {
    fetch(`${state.serverUrl}/nav/stop`, { method: "POST" }).catch(() => {});
  }
}

// ---------------------------------------------------------------------------
// Navigation command computation
// ---------------------------------------------------------------------------

/**
 * Decide which belt motor to activate given the current position and heading.
 * Returns one of the 8 direction strings (or "stop").
 */
function computeNavCommand(lat, lng, headingDeg) {
  if (!state.routeActive || state.steps.length === 0) return "stop";

  const step = state.steps[state.stepIndex];
  if (!step) return "stop";

  const distToWaypoint = haversineM(lat, lng, step.end_lat, step.end_lng);
  const isLastStep     = state.stepIndex === state.steps.length - 1;

  // Within arrival threshold for last step
  if (isLastStep && distToWaypoint <= WAYPOINT_REACHED_M) return "stop";

  // Decide phase: are we approaching the turn or just going straight?
  let command;
  if (distToWaypoint <= PREPARE_DISTANCE_M) {
    // Approaching turn – use the step's maneuver direction
    command = MANEUVER_MAP[step.maneuver] || "front";
  } else {
    // Far from the turn – use bearing toward the waypoint for "keep going"
    const bearingToWaypoint = bearingDeg(lat, lng, step.end_lat, step.end_lng);
    const relativeBearing   = ((bearingToWaypoint - headingDeg) + 360) % 360;
    command = sectorToDirection(relativeBearing);
  }

  // Update instruction display
  const distLabel = fmtDist(distToWaypoint);
  const phase     = distToWaypoint <= PREPARE_DISTANCE_M ? "execute" : "prepare";
  instructionMain.textContent = step.instruction;
  instructionDist.textContent = phase === "execute"
    ? `Turn now  ·  ${distLabel}`
    : `In ${distLabel}`;
  setInstructionArrow(command);

  return command;
}

/** Map a 0-360 relative bearing to the nearest 8-direction label. */
function sectorToDirection(relativeBearing) {
  // Sector boundaries at every 45°, centred on each direction:
  // front = [-22.5, 22.5), front_right = [22.5, 67.5), etc.
  const idx = Math.round(relativeBearing / 45) % 8;
  return DIRECTIONS[idx < 0 ? idx + 8 : idx];
}

// ---------------------------------------------------------------------------
// Apply command to local UI
// ---------------------------------------------------------------------------

function applyNavCommand(command) {
  setActiveMotor(command);
  setInstructionArrow(command);
  state.lastCommand = command;
}

// ---------------------------------------------------------------------------
// Server communication
// ---------------------------------------------------------------------------

async function sendCommandToServer(command) {
  if (!state.connected) return;
  const step = state.steps[state.stepIndex];
  const payload = {
    command,
    motor_id:    DIRECTIONS.indexOf(command),
    intensity:   NAV_INTENSITY[command] ?? 0,
    pattern:     NAV_PATTERN[command]   ?? "off",
    instruction: step ? step.instruction : "",
    distance_m:  step ? haversineM(
      state.currentPos.lat, state.currentPos.lng,
      step.end_lat, step.end_lng
    ) : null,
    phase: "execute",
  };
  try {
    await fetch(`${state.serverUrl}/nav/set_command`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
  } catch { /* silently ignore */ }
}

async function sendRouteToServer(leg) {
  if (!state.connected) return;
  const payload = {
    origin:           { lat: leg.start_location.lat(), lng: leg.start_location.lng() },
    destination:      { lat: leg.end_location.lat(),   lng: leg.end_location.lng() },
    dest_name:        leg.end_address,
    steps:            state.steps,
    total_distance_m: leg.distance.value,
    total_duration_s: leg.duration.value,
    active:           true,
  };
  try {
    await fetch(`${state.serverUrl}/nav/set_route`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
  } catch { /* ignore */ }
}

// ---------------------------------------------------------------------------
// Stop navigation
// ---------------------------------------------------------------------------

stopBtn.addEventListener("click", () => {
  stopNavigation();
});

function stopNavigation() {
  state.routeActive = false;
  stopWatchingPosition();
  stopSimulation();

  instructionCard.classList.add("hidden");
  stepsSection.classList.add("hidden");
  if (state.directionsRenderer) state.directionsRenderer.setDirections({ routes: [] });

  setActiveMotor("stop");
  motorBadge.textContent = "—";
  routeBtn.disabled = false;

  if (state.connected) {
    fetch(`${state.serverUrl}/nav/stop`, { method: "POST" }).catch(() => {});
  }
}

// ---------------------------------------------------------------------------
// Hazard status polling
// ---------------------------------------------------------------------------

const HAZARD_ZONE_CLASS = {
  CLEAR:    "",
  ALERT:    "level-1",
  CAUTION:  "level-2",
  WARNING:  "level-3",
  DANGER:   "level-4",
  CRITICAL: "level-5",
};

function startHazardPoll() {
  if (state.hazardPollTimer) return;
  state.hazardPollTimer = setInterval(pollHazardStatus, HAZARD_POLL_MS);
  pollHazardStatus(); // immediate first call
}

function stopHazardPoll() {
  if (state.hazardPollTimer) { clearInterval(state.hazardPollTimer); state.hazardPollTimer = null; }
}

async function pollHazardStatus() {
  if (!state.connected) return;
  try {
    const r = await fetch(`${state.serverUrl}/status`, { signal: AbortSignal.timeout(2000) });
    if (!r.ok) return;
    const data = await r.json();

    // Find worst hazard across all devices
    let worstLevel = 0;
    let worstLabel = "CLEAR";
    const alerts = [];

    Object.values(data.devices || {}).forEach(dev => {
      const lvl = hazardLevelNum(dev.last_zone);
      if (lvl > worstLevel) { worstLevel = lvl; worstLabel = dev.last_zone; }
    });

    // Update badge
    hazardLabel.textContent = worstLabel;
    hazardBadge.className   = `hazard-badge ${HAZARD_ZONE_CLASS[worstLabel] || ""}`;

    // Show per-device active alerts
    hazardAlerts.innerHTML = "";
    Object.entries(data.devices || {}).forEach(([id, dev]) => {
      if (dev.last_zone === "CLEAR") return;
      const chip = document.createElement("div");
      const cls = dev.last_zone === "DANGER" || dev.last_zone === "CRITICAL"
        ? "danger" : dev.last_zone === "WARNING" ? "warning" : "caution";
      chip.className = `hazard-alert-chip ${cls}`;
      chip.textContent = `${id.replace("esp32_", "")}  ${dev.last_zone}`;
      hazardAlerts.appendChild(chip);
    });
  } catch { /* offline */ }
}

function hazardLevelNum(label) {
  return { CLEAR: 0, ALERT: 1, CAUTION: 2, WARNING: 3, DANGER: 4, CRITICAL: 5 }[label] || 0;
}

// ---------------------------------------------------------------------------
// Server connection
// ---------------------------------------------------------------------------

serverUrlInput.addEventListener("change", () => {
  state.serverUrl = serverUrlInput.value.trim().replace(/\/$/, "");
});

connBtn.addEventListener("click", () => {
  if (state.connected) {
    disconnectServer();
  } else {
    connectServer();
  }
});

async function connectServer() {
  const url = serverUrlInput.value.trim().replace(/\/$/, "");
  state.serverUrl = url;
  connStatus.textContent = "Connecting…";
  connDot.className = "conn-dot";
  connBtn.disabled = true;

  try {
    const r = await fetch(`${url}/status`, { signal: AbortSignal.timeout(4000) });
    if (!r.ok) throw new Error(`HTTP ${r.status}`);
    state.connected = true;
    connDot.className   = "conn-dot online";
    connStatus.textContent = "Connected";
    connBtn.textContent = "Disconnect";
    connBtn.disabled    = false;
    startHazardPoll();
  } catch (e) {
    state.connected    = false;
    connDot.className  = "conn-dot error";
    connStatus.textContent = `Failed: ${e.message}`;
    connBtn.textContent = "Connect";
    connBtn.disabled    = false;
  }
}

function disconnectServer() {
  state.connected = false;
  stopHazardPoll();
  connDot.className  = "conn-dot offline";
  connStatus.textContent = "Disconnected";
  connBtn.textContent = "Connect";
}

// ---------------------------------------------------------------------------
// Simulation  (walk along route when real GPS is unavailable)
// ---------------------------------------------------------------------------

function startSimulation() {
  if (state.simTimer) return;
  if (state.simPathPoints.length < 2) return;
  state.simPointIndex = 0;
  state.simTimer = setInterval(simStep, SIM_INTERVAL_MS);
}

function stopSimulation() {
  if (state.simTimer) { clearInterval(state.simTimer); state.simTimer = null; }
}

function simStep() {
  if (!state.routeActive) { stopSimulation(); return; }
  if (state.simPointIndex >= state.simPathPoints.length) {
    stopSimulation();
    arrivedAtDestination();
    return;
  }
  const pt  = state.simPathPoints[state.simPointIndex];
  const next = state.simPathPoints[Math.min(state.simPointIndex + 1, state.simPathPoints.length - 1)];
  const heading = bearingDeg(pt.lat(), pt.lng(), next.lat(), next.lng());

  handlePositionUpdate(pt.lat(), pt.lng(), heading);
  state.simPointIndex++;
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/** Haversine distance in metres between two lat/lng pairs. */
function haversineM(lat1, lng1, lat2, lng2) {
  const R = 6371000;
  const dLat = (lat2 - lat1) * Math.PI / 180;
  const dLng = (lng2 - lng1) * Math.PI / 180;
  const a = Math.sin(dLat / 2) ** 2 +
    Math.cos(lat1 * Math.PI / 180) * Math.cos(lat2 * Math.PI / 180) * Math.sin(dLng / 2) ** 2;
  return R * 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
}

/** Compass bearing (0 = north, clockwise) in degrees from point 1 to point 2. */
function bearingDeg(lat1, lng1, lat2, lng2) {
  const φ1 = lat1 * Math.PI / 180;
  const φ2 = lat2 * Math.PI / 180;
  const Δλ = (lng2 - lng1) * Math.PI / 180;
  const y = Math.sin(Δλ) * Math.cos(φ2);
  const x = Math.cos(φ1) * Math.sin(φ2) - Math.sin(φ1) * Math.cos(φ2) * Math.cos(Δλ);
  return ((Math.atan2(y, x) * 180 / Math.PI) + 360) % 360;
}

/** Format metres as human-readable string. */
function fmtDist(m) {
  if (m == null) return "";
  if (m >= 1000) return `${(m / 1000).toFixed(1)} km`;
  return `${Math.round(m)} m`;
}

/** Strip HTML tags from Google Maps instruction strings. */
function stripHtml(html) {
  const d = document.createElement("div");
  d.innerHTML = html;
  return d.textContent || d.innerText || "";
}

// ---------------------------------------------------------------------------
// Dark map styles
// ---------------------------------------------------------------------------

const DARK_MAP_STYLES = [
  { elementType: "geometry", stylers: [{ color: "#1a1f2e" }] },
  { elementType: "labels.text.stroke", stylers: [{ color: "#111827" }] },
  { elementType: "labels.text.fill", stylers: [{ color: "#7a88a4" }] },
  { featureType: "road", elementType: "geometry", stylers: [{ color: "#252c3d" }] },
  { featureType: "road", elementType: "geometry.stroke", stylers: [{ color: "#1a1f2e" }] },
  { featureType: "road.highway", elementType: "geometry", stylers: [{ color: "#2e3650" }] },
  { featureType: "road", elementType: "labels.text.fill", stylers: [{ color: "#9ea8bb" }] },
  { featureType: "poi", elementType: "geometry", stylers: [{ color: "#1e2435" }] },
  { featureType: "poi", elementType: "labels.text.fill", stylers: [{ color: "#7a88a4" }] },
  { featureType: "poi.park", elementType: "geometry", stylers: [{ color: "#1a2b1a" }] },
  { featureType: "water", elementType: "geometry", stylers: [{ color: "#0d1b2a" }] },
  { featureType: "water", elementType: "labels.text.fill", stylers: [{ color: "#4f8ef7" }] },
  { featureType: "transit", stylers: [{ visibility: "off" }] },
  { featureType: "administrative", elementType: "geometry.stroke", stylers: [{ color: "#2e3650" }] },
  { featureType: "administrative.land_parcel", stylers: [{ visibility: "off" }] },
];

// ---------------------------------------------------------------------------
// Simulation Mode  (polls /sim/state from test_server.py)
// ---------------------------------------------------------------------------

const simModeBtn = $("sim-mode-btn");
const simModeBanner = $("sim-mode-banner");

state.simMode       = false;
state.simPollTimer  = null;

function enableSimMode() {
  state.simMode = true;
  if (simModeBtn)    simModeBtn.textContent    = "Disable Sim Mode";
  if (simModeBtn)    simModeBtn.classList.add("active");
  if (simModeBanner) simModeBanner.classList.remove("hidden");
  stopWatchingPosition();            // don't fight real GPS
  stopSimulation();                  // don't run client-side sim
  state.simPollTimer = setInterval(pollSimState, 500);
  pollSimState();
}

function disableSimMode() {
  state.simMode = false;
  if (simModeBtn)    simModeBtn.textContent    = "Enable Sim Mode";
  if (simModeBtn)    simModeBtn.classList.remove("active");
  if (simModeBanner) simModeBanner.classList.add("hidden");
  if (state.simPollTimer) { clearInterval(state.simPollTimer); state.simPollTimer = null; }
}

if (simModeBtn) {
  simModeBtn.addEventListener("click", () => {
    state.simMode ? disableSimMode() : enableSimMode();
  });
}

async function pollSimState() {
  if (!state.simMode || !state.connected) return;
  try {
    const r = await fetch(`${state.serverUrl}/sim/state`,
                          { signal: AbortSignal.timeout(1500) });
    if (!r.ok) return;
    const data = await r.json();

    // Feed virtual position into the nav pipeline (same path as real GPS)
    if (data.lat != null && data.lng != null) {
      handlePositionUpdate(data.lat, data.lng, data.heading ?? 0);
    }

    // If the server has a running route and we don't, load the route from server
    if (data.running && !state.routeActive && data.nav && data.nav.active) {
      // Reflect server nav command directly when no client route loaded
      applyNavCommand(data.nav.command || "stop");
      if (data.nav.instruction) {
        instructionCard.classList.remove("hidden");
        instructionMain.textContent = data.nav.instruction;
        instructionDist.textContent = data.nav.distance_m != null
          ? fmtDist(data.nav.distance_m) : "";
      }
    }

    // Centre map on moving walker
    if (state.map && data.lat != null) {
      state.map.panTo({ lat: data.lat, lng: data.lng });
    }
  } catch { /* offline */ }
}

// ---------------------------------------------------------------------------
// Boot  (no Maps API case – still init motor ring and server UI)
// ---------------------------------------------------------------------------

(function boot() {
  // Default motor ring state
  setActiveMotor("stop");
  motorBadge.textContent = "—";

  // Allow typing destination even without Maps
  destInput.addEventListener("input", () => {
    if (destInput.value.trim()) routeBtn.disabled = false;
  });

  // Without Maps API, stub route button to show demo mode
  routeBtn.addEventListener("click", () => {
    if (!window.google) {
      instructionCard.classList.remove("hidden");
      instructionMain.textContent = "Demo mode – add Maps API key";
      instructionDist.textContent = "Simulating motor commands…";
      setInstructionArrow("front");
      applyNavCommand("front");
    }
  });
})();
