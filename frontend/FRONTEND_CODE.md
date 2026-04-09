# Frontend Code Reference
**OmniSense-Dual Navigation UI — ECE 445 Group 10**

---

## File Structure

```
frontend/
├── index.html          Main HTML shell — layout, DOM structure, Maps loader
├── style.css           All visual styling (dark theme, components, responsive)
├── app.js              All JavaScript logic — Maps, navigation, motor, server comms
├── config.js           Your API key + server URL (gitignored, never committed)
└── config.example.js   Template to copy into config.js
```

---

## `index.html`

Plain HTML file with no build step. Two main regions:

### `<aside id="sidebar">`
A fixed left panel (360 px wide) containing every control and status section, top to bottom:

| Section | HTML element | Purpose |
|---|---|---|
| Logo + title | `.sidebar-header` | Branding |
| Route inputs | `.route-section` | Origin (read-only), destination search, Start button |
| Instruction card | `#instruction-card` | Hidden until navigation starts; shows turn arrow + text + Stop button |
| Belt motor ring | `.motor-section` | 8-dot ring around a body figure; active dot lights up |
| Turn-by-turn list | `#steps-section` | Scrollable ordered list of route steps; hidden until route loads |
| Hazard status | `.hazard-section` | Colour-coded badge + per-direction alert chips |
| Simulation mode | `.sim-section` | Toggle for hardware-free testing |
| Server footer | `.sidebar-footer` | Server URL input, connect/disconnect button, status dot |

### `<main id="map-container">`
Takes up the full right area. Contains:
- `<div id="map">` — Google Maps renders here
- `#heading-compass` — floating SVG compass (bottom-right of map)
- `.map-controls` — Center-on-me and 3D/2D toggle buttons

### Maps API loader (inline `<script>` at bottom)
Reads `window.OMNISENSE_CONFIG.GOOGLE_MAPS_API_KEY` from `config.js`. If the key is the placeholder string or missing, it renders an instruction message instead and does not load the Maps API. All navigation logic and server communication still works without the map.

---

## `style.css`

Single flat stylesheet, no preprocessor. Key sections:

| Section | What it styles |
|---|---|
| `:root` variables | All colours, spacing, radius, sidebar width — change theme here |
| Layout | `#sidebar` fixed left, `#map-container` fills the rest |
| `.route-section` | Input fields, connector line, Start button |
| `.instruction-card` | Turn arrow circle, instruction text, Stop button |
| `.motor-dot` / `.motor-dot.active` | Belt motor dots; active dot glows blue |
| `.hazard-badge.level-1…5` | Colour ramp from accent → yellow → red; level-5 pulses |
| `.sim-section` / `.btn-sim.active` | Simulation mode button, blue banner |
| `.pac-container` overrides | Forces Google Maps autocomplete dropdown into dark theme |
| `@media (max-width: 700px)` | Collapses map, shows sidebar full-screen on mobile |

---

## `app.js`

Approximately 1000 lines, no dependencies other than the Google Maps API. Sections in order:

### Constants (lines 24–104)
```
DIRECTIONS          8-element array matching algorithm.py motor indices
WAYPOINT_REACHED_M  15 m  — snap to next step
PREPARE_DISTANCE_M  50 m  — start showing upcoming turn command
HAZARD_POLL_MS      2000  — how often to poll /status
NAV_INTENSITY       vibration intensity per direction (0–255)
NAV_PATTERN         vibration pattern name per direction
ARROW_SVG           inline SVG for each of the 9 directions (+ stop)
MANEUVER_MAP        Google Maps maneuver string → 8-direction command
```

### `state` object (line 110)
Single mutable object holding all runtime state:
```javascript
state.map / directionsService / directionsRenderer  // Google Maps objects
state.currentPos      // {lat, lng} — last known position
state.currentHeading  // degrees, 0 = north
state.steps[]         // parsed route steps from Maps API
state.stepIndex       // which step we're currently on
state.routeActive     // true while navigating
state.simPathPoints[] // polyline points for client-side path simulation
state.serverUrl       // Flask server base URL
state.connected       // true after successful /status handshake
state.simMode         // true when polling test_server /sim/state
```

### Motor ring builder (line 176)
Runs once on load. Places 8 `div.motor-dot` elements around a 180 px circle using polar coordinates, one per direction, with short labels (F, FR, R, …).

### `initMap()` — Google Maps callback (line 234)
Called by the Maps API loader. Creates the map with a dark custom style, attaches `DirectionsService` + `DirectionsRenderer`, sets up Places Autocomplete on the destination input, creates the user marker, and calls `requestGeolocation()`.

### Geolocation (line 335)
`requestGeolocation()` — one-shot position on load, reverse-geocodes to a street name for the origin field.  
`startWatchingPosition()` — continuous `watchPosition` during active navigation.  
Both funnel into `handlePositionUpdate(lat, lng, heading)`.

### `handlePositionUpdate(lat, lng, heading)` (line 394)
Central function called on every position change (real GPS, client sim, or server sim mode):
1. Updates `state.currentPos` and `state.currentHeading`
2. Moves the blue marker on the map
3. Rotates the compass needle
4. POSTs position to `/nav/position` on the server
5. If route is active: calls `advanceRouteStep` → `computeNavCommand` → `applyNavCommand` → `sendCommandToServer`

### Route computation (line 422)
`computeRoute(origin, destination)` calls `DirectionsService.route()` for `WALKING` mode. On success:
- Stores steps as `state.steps[]` (each with `{instruction, maneuver, distance_m, start_lat, start_lng, end_lat, end_lng}`)
- Extracts the overview polyline into `state.simPathPoints[]`
- Renders route on map
- Calls `sendRouteToServer()` → `POST /nav/set_route`
- Shows instruction card and steps list

### `computeNavCommand(lat, lng, headingDeg)` (line ~560)
Core navigation logic — returns one of the 9 direction strings:

```
dist_to_next_waypoint = haversine(current, step.end)

if dist <= 50 m:            ← EXECUTE phase
    command = MANEUVER_MAP[step.maneuver]   (e.g. "turn-left" → "left")
else:                       ← PREPARE phase
    bearing = compass bearing to next waypoint
    relative = (bearing − heading + 360) % 360
    command = DIRECTIONS[ round(relative / 45) % 8 ]
```

### `advanceRouteStep(lat, lng)` (line ~530)
Checks if the walker is within `WAYPOINT_REACHED_M` (15 m) of the current step's end point. If so, increments `state.stepIndex`. If the last step is reached, calls `arrivedAtDestination()`.

### Server communication (line ~590)
`sendCommandToServer(command)` — `POST /nav/set_command` with `{command, motor_id, intensity, pattern, instruction, distance_m, phase}`.  
`sendRouteToServer(leg)` — `POST /nav/set_route` with full step list.  
`pollHazardStatus()` — `GET /status`, updates the hazard badge and alert chips.

### Simulation mode (line ~895)
`enableSimMode()` — stops GPS watching, stops client-side polyline simulation, starts polling `GET /sim/state` every 500 ms from `test_server.py`.  
`pollSimState()` — feeds the returned `{lat, lng, heading}` directly into `handlePositionUpdate()`, driving the whole nav pipeline with virtual position.

### Client-side polyline simulation (line ~740)
Fallback when no GPS is available and sim mode is off. Steps through `state.simPathPoints[]` (the route overview polyline) at one point per second, computing bearing between consecutive points for heading.

### Utility functions (line ~800)
```
haversineM(lat1,lng1,lat2,lng2)   distance in metres between two coords
bearingDeg(lat1,lng1,lat2,lng2)   compass bearing 0–360°
fmtDist(metres)                   "45 m" or "1.2 km"
stripHtml(html)                   removes HTML tags from Maps instructions
```

---

## `config.js` / `config.example.js`

```javascript
window.OMNISENSE_CONFIG = {
  GOOGLE_MAPS_API_KEY: "AIzaSy...",   // Maps JS API + Directions + Places
  SERVER_URL: "http://localhost:5000", // Flask server (change for test server: 5001)
};
```

`config.js` is listed in `.gitignore` and never committed. `config.example.js` is the committed template.  
`app.js` reads `SERVER_URL` at startup: `(window.OMNISENSE_CONFIG || {}).SERVER_URL || "http://localhost:5000"`.

---

## Data flow summary

```
Browser GPS / test_server /sim/state
         │
         ▼
handlePositionUpdate(lat, lng, heading)
         │
    ┌────┴────────────────────────────────────┐
    │                                         │
    ▼                                         ▼
advanceRouteStep()                    updateUserMarker()
  (advance stepIndex                    (move map dot,
   when within 15 m)                     rotate compass)
    │
    ▼
computeNavCommand()
  bearing math → 8-sector direction
    │
    ▼
applyNavCommand(command)              sendCommandToServer(command)
  setActiveMotor() → lights             POST /nav/set_command
  motor ring dot                          ↓
  setInstructionArrow()              Body ESP32 polls
                                     GET /nav/get_command
                                     → activates belt motor
```
