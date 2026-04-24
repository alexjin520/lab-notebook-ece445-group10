# OmniSense-Dual Navigation UI — User Guide
**ECE 445 Group 10**

---

## Quick Start

```powershell
# 1. Start the Flask server  (from d:\ECE445)
python server.py

# 2. Serve the frontend
python -m http.server 8080 --directory frontend

# 3. Open in browser
#    http://localhost:8080
```

---

## Overview of UI Sections

```
┌──────────────────────────────────────────────────────────────┐
│  SIDEBAR (left 360 px)          │  MAP (rest of screen)      │
│                                 │                            │
│  [Logo]                         │  • Dark Google Map         │
│                                 │  • Blue route line         │
│  ○ Current Location  [⊕]        │  • Blue dot = you          │
│  ─────────────────────          │                            │
│  ◉ Destination       [Search]   │       [compass]  bottom-R  │
│  [ Start Navigation ]           │       [ ⊕ ]  map controls  │
│  ─────────────────────          │       [3D]                 │
│  ↑ Turn right in 45 m  [Stop]   │                            │
│  ─────────────────────          │                            │
│  Belt Motor Status (ring)       │                            │
│  ─────────────────────          │                            │
│  Turn-by-Turn list              │                            │
│  ─────────────────────          │                            │
│  Hazard Status                  │                            │
│  ─────────────────────          │                            │
│  [Enable Sim Mode]              │                            │
│  ─────────────────────          │                            │
│  Server: http://localhost:5000  │                            │
│  ● Connected    [Disconnect]    │                            │
└──────────────────────────────────────────────────────────────┘
```

---

## Feature 1 — Detect Current Location

**Purpose:** Auto-fill the origin field with your current GPS coordinates.

**Steps:**
1. Open `http://localhost:8080` in a browser.
2. Allow the location permission popup that appears.
3. The **Current Location** field fills with a street name (e.g., `Green St, Champaign`).
4. The blue dot on the map jumps to your position and the compass appears.

**If location is blocked or unavailable:**
- The field shows `Location unavailable`.
- Click the **⊕ locate button** (right of the field) to try again.
- Navigation still works — the route will start from wherever you type in the origin field.

---

## Feature 2 — Route Planning

**Purpose:** Compute a walking route from your current location to any destination.

**Steps:**
1. Click the **Destination** input box.
2. Start typing an address or place name — a dropdown appears (Google Places Autocomplete).
3. Select a suggestion from the dropdown.
4. Click **Start Navigation**.
5. The map zooms to show the full route drawn in blue.
6. The **Turn-by-Turn** list appears in the sidebar.
7. The **instruction card** appears showing the first maneuver.

**Behind the scenes:**
- Google Maps Directions API returns the walking route as a list of steps.
- The route is sent to the Flask server (`POST /nav/set_route`) so the body ESP32 can track progress.

---

## Feature 3 — Live Navigation (Real GPS)

**Purpose:** Guide yourself to the destination with real-time turn-by-turn instructions.

**Steps:**
1. Complete Feature 2 (route planned).
2. Walk. The blue dot on the map follows your real GPS position.
3. Watch the **instruction card** — it shows the upcoming turn and counts down distance.
4. When you are within 50 m of a turn, the instruction switches to **execute** phase (e.g., `Turn now · 12 m`).
5. The **belt motor ring** lights up the correct motor dot for the required direction.
6. When you pass a waypoint (within 15 m), the next step becomes active and highlights in the Turn-by-Turn list.
7. When you reach the destination, the card shows `You have arrived!` and all motors stop.

**Instruction card phases:**

| Distance to turn | Instruction shown | Motor command |
|---|---|---|
| > 50 m | `In 120 m` | Direction toward waypoint (e.g., `FRONT`) |
| ≤ 50 m | `Turn now · 30 m` | Maneuver direction (e.g., `LEFT`) |
| Arrived | `You have arrived!` | `STOP` |

---

## Feature 4 — Belt Motor Ring Visualizer

**Purpose:** Show which of the 8 belt vibration motors is being commanded to activate.

**What you see:**
- 8 dots arranged in a ring around a body figure (F, FR, R, BR, B, BL, L, FL).
- The **active motor dot glows blue**.
- The **command badge** below the ring shows the direction label (e.g., `LEFT`).

**Motor index mapping:**

| Index | Direction | Degrees |
|---|---|---|
| 0 | FRONT | 0° |
| 1 | FRONT RIGHT | 45° |
| 2 | RIGHT | 90° |
| 3 | BACK RIGHT | 135° |
| 4 | BACK | 180° |
| 5 | BACK LEFT | 225° |
| 6 | LEFT | 270° |
| 7 | FRONT LEFT | 315° |

The active motor dot updates on every GPS position update (every ~1 s). The body ESP32 polls `GET /nav/get_command` at ≥10 Hz to receive the same command and activate the physical motor.

---

## Feature 5 — Hazard Status Panel

**Purpose:** Display real-time hazard alerts sent by the wearable sensors.

**What you see:**
- A colour-coded badge showing the worst current hazard level.
- Small chips showing each individual alert direction and severity.

**Hazard levels and colours:**

| Level | Label | Badge colour |
|---|---|---|
| 0 | CLEAR | Green |
| 1 | ALERT | Blue |
| 2 | CAUTION | Blue (brighter) |
| 3 | WARNING | Yellow |
| 4 | DANGER | Red |
| 5 | CRITICAL | Red + pulsing glow |

**How it updates:**
- The frontend polls `GET /status` every 2 seconds.
- The server returns the worst hazard zone across all connected ESP32 devices.
- Badge and chips update automatically — no user action needed.

---

## Feature 6 — Simulation Mode (hardware-free testing)

**Purpose:** Test all navigation features without a phone's GPS or physical hardware, using `test_server.py` to simulate a moving walker.

**Steps:**
1. Start the test server in a separate terminal:
   ```powershell
   python test_server.py    # runs on port 5001
   ```
2. In the navigation UI, change the **Server** URL (bottom of sidebar) to `http://localhost:5001`.
3. Click **Connect** — status dot turns green.
4. Plan a route (Feature 2) — the test server automatically starts walking along the route.
5. Click **Enable Sim Mode** in the sidebar — blue banner appears, `Simulation Mode Active`.
6. The blue dot on the map starts moving. The motor ring, instruction card, and hazard panel all update as if you were physically walking.
7. Open the **Control Panel** at `http://localhost:5001/sim/ui` to:
   - Adjust walker speed (slider)
   - Pause / reset the walk
   - Inject a hazard in any direction instantly
   - Control hazard probability

**To disable sim mode:** Click the button again — the banner disappears and the app returns to real GPS.

---

## Feature 7 — Server Connection

**Purpose:** Link the frontend to the Flask backend so motor commands reach the ESP32.

**Steps:**
1. Confirm the server is running (`python server.py` or `python test_server.py`).
2. The **Server** URL field at the bottom of the sidebar defaults to `http://localhost:5000`.
   - Change to `http://localhost:5001` when using the test server.
   - Change to `http://<PC-IP>:5000` when the ESP32 is on the same WiFi network.
3. Click **Connect**.
4. Status dot turns **green** and shows `Connected`.
5. From this point every navigation command is sent to the server, and hazard alerts are polled from it.

**To find your PC's IP (for ESP32):**
```powershell
ipconfig | Select-String "IPv4"
```

**Connection failure:** Status dot turns **red** with the error. Check that the server is running and the port is not blocked by a firewall.

---

## Feature 8 — Map Controls

| Control | Location | Action |
|---|---|---|
| **⊕ center button** | Bottom-right of map | Re-centres map on current position |
| **3D / 2D toggle** | Below center button | Tilts the map to 45° 3D perspective or returns to flat 2D |
| **Compass** | Bottom-right, above controls | Shows current heading; needle points north |

---

## Feature 9 — Stop Navigation

**Purpose:** Cancel the active route and silence all belt motors.

**Steps:**
1. While navigating, click the red **Stop** button on the instruction card.
2. The route line clears from the map.
3. The instruction card hides.
4. The motor ring resets (all dots dim).
5. A `POST /nav/stop` is sent to the server — the ESP32's belt motors deactivate.

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| Map shows "Add your Google Maps API Key" | `config.js` has placeholder key | Paste real key into `config.js` |
| Destination search has no dropdown | Places API not enabled | Enable **Places API** in Google Cloud Console |
| `Location unavailable` | Browser blocked GPS | Click the lock icon in browser address bar → allow Location |
| Server shows `Failed: …` | Server not running | Run `python server.py` or `python test_server.py` |
| Motor ring does not update | Not connected to server | Click **Connect** first |
| Blue dot does not move | Sim Mode off + no GPS | Enable **Sim Mode** for hardware-free testing |
| Route not drawn | Maps key missing or Directions API disabled | Enable **Directions API** in Google Cloud Console |
