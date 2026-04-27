/* ============================================================================
 *  app_replay.js  —  Log Replay add-on for the OmniSense navigation page
 *
 *  This script binds the replay UI in index.html to the LogReplay engine
 *  defined in log_replay.js. When activated it:
 *
 *    1. Suspends live HTTP polling, sim mode, and outbound nav commands.
 *    2. Loads an `omnisense_*.log` from disk.
 *    3. Walks the recorded NAV timeline in time order, replaying:
 *         · Belt motor ring + motor badge       (deliver / command events)
 *         · Instruction arrow + text + distance (command events)
 *         · Heading compass needle              (body IMU yaw events)
 *         · Hazard panel                        (hazard events)
 *         · Map polyline + origin/dest pins     (route_detail events)
 *         · User marker + breadcrumb trail      (position events)
 *
 *  The replay is non-destructive: when the user disables it everything
 *  resets and live navigation resumes from where the user left off.
 * ========================================================================= */
(() => {
  "use strict";

  // ---------------------------------------------------------------------------
  // Wait until app.js has finished bootstrapping (DOM + globals).
  // ---------------------------------------------------------------------------
  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", boot);
  } else {
    boot();
  }

  function boot() {
    if (!window.OmniSenseLog) {
      console.warn("log_replay.js not loaded — replay disabled");
      return;
    }

    // -------------------------------------------------------------------------
    // DOM refs (created by index.html)
    // -------------------------------------------------------------------------
    const $ = id => document.getElementById(id);
    const replayModeBtn   = $("replay-mode-btn");
    const replayUI        = $("replay-ui");
    const replayBanner    = $("replay-banner");
    const dropZone        = $("replay-drop");
    const fileInput       = $("replay-file");
    const controls        = $("replay-controls");
    const slider          = $("replay-slider");
    const playBtn         = $("replay-play");
    const speedBtns       = ["replay-1x", "replay-4x", "replay-10x"].map($);
    const clearBtn        = $("replay-clear");
    const filenameLabel   = $("replay-filename");
    const timeLabel       = $("replay-time");

    // Refs into app.js's existing UI primitives
    const instructionCard = $("instruction-card");
    const instructionMain = $("instruction-main");
    const instructionDist = $("instruction-dist");
    const motorBadge      = $("motor-command-badge");
    const hazardBadge     = $("hazard-badge");
    const hazardLabel     = $("hazard-label");
    const hazardAlerts    = $("hazard-alerts");
    const compassNeedle   = $("compass-needle");
    const headingCompass  = $("heading-compass");
    const stepsSection    = $("steps-section");
    const stepsList       = $("steps-list");
    const connBtn         = $("conn-btn");
    const routeBtn        = $("route-btn");
    const simModeBtn      = $("sim-mode-btn");
    const stopBtn         = $("stop-btn");

    if (!replayModeBtn) {
      console.warn("Replay UI not found in DOM — index.html may be out of date");
      return;
    }

    // -------------------------------------------------------------------------
    // Local replay state
    // -------------------------------------------------------------------------
    const replay = {
      enabled:       false,
      events:        [],
      route:         null,
      player:        null,
      speed:         4,
      filename:      "",

      // Map artefacts owned by replay (separate from live-mode artefacts so we
      // never trample the user's real route).
      polyline:      null,
      originMarker:  null,
      destMarker:    null,
      userMarker:    null,
      pathTrail:     null,    // breadcrumb of recorded GPS pings
      pathPoints:    [],
    };

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    /** Format ms as "12.3 s" or "1:23.4". */
    const fmtTime = ms => {
      if (!isFinite(ms) || ms <= 0) return "0.0 s";
      const s = ms / 1000;
      if (s < 60) return `${s.toFixed(1)} s`;
      const m = Math.floor(s / 60);
      const r = (s - m * 60).toFixed(1);
      return `${m}:${r.padStart(4, "0")}`;
    };

    /** Live-mode controls we suspend during replay. */
    function setLiveControlsEnabled(enabled) {
      [connBtn, routeBtn, simModeBtn, stopBtn].forEach(b => {
        if (!b) return;
        b.disabled = !enabled;
        b.style.opacity = enabled ? "" : "0.5";
        b.style.pointerEvents = enabled ? "" : "none";
      });
    }

    /** Suspend live polling & outbound traffic without disconnecting. */
    function suspendLive() {
      if (typeof stopHazardPoll === "function") stopHazardPoll();
      if (typeof state !== "undefined" && state) {
        replay._wasConnected = state.connected;
        replay._wasSimMode   = state.simMode;
        replay._wasRouteAct  = state.routeActive;
        // Pretend we're disconnected so app.js skips outbound POSTs.
        state.connected   = false;
        state.simMode     = false;
        state.routeActive = false;
      }
      if (typeof stopSimulation === "function") stopSimulation();
      if (typeof stopWatchingPosition === "function") stopWatchingPosition();
    }

    function resumeLive() {
      if (typeof state !== "undefined" && state) {
        if (replay._wasConnected) state.connected = true;
        if (replay._wasSimMode)   state.simMode   = true;
      }
    }

    // ---- Map artefact lifecycle (only when google.maps is available) ----------
    const hasMap = () =>
      window.google && window.google.maps &&
      typeof state !== "undefined" && state && state.map;

    function clearMapArtefacts() {
      if (replay.polyline)     { replay.polyline.setMap(null);     replay.polyline = null; }
      if (replay.originMarker) { replay.originMarker.setMap(null); replay.originMarker = null; }
      if (replay.destMarker)   { replay.destMarker.setMap(null);   replay.destMarker = null; }
      if (replay.userMarker)   { replay.userMarker.setMap(null);   replay.userMarker = null; }
      if (replay.pathTrail)    { replay.pathTrail.setMap(null);    replay.pathTrail = null; }
      replay.pathPoints = [];
    }

    function drawRoute(route) {
      if (!hasMap() || !route || !route.steps || !route.steps.length) return;
      const path = [];
      route.steps.forEach(s => {
        if (s.start_lat != null && s.start_lng != null)
          path.push({ lat: s.start_lat, lng: s.start_lng });
        if (s.end_lat != null && s.end_lng != null)
          path.push({ lat: s.end_lat,   lng: s.end_lng });
      });
      if (!path.length) return;

      replay.polyline = new google.maps.Polyline({
        path,
        strokeColor: "#9f7aea",
        strokeWeight: 5,
        strokeOpacity: 0.85,
        map: state.map,
      });

      if (route.origin) {
        replay.originMarker = new google.maps.Marker({
          position: route.origin, map: state.map,
          icon: {
            path: google.maps.SymbolPath.CIRCLE,
            fillColor: "#9f7aea", fillOpacity: 1,
            strokeColor: "#fff", strokeWeight: 2, scale: 7,
          },
          title: "Replay origin",
        });
      }
      if (route.destination) {
        replay.destMarker = new google.maps.Marker({
          position: route.destination, map: state.map,
          label: { text: "★", color: "#fff", fontSize: "14px" },
          icon: {
            path: google.maps.SymbolPath.CIRCLE,
            fillColor: "#e05c5c", fillOpacity: 1,
            strokeColor: "#fff", strokeWeight: 2, scale: 12,
          },
          title: route.dest_name || "Replay destination",
        });
      }

      const bounds = new google.maps.LatLngBounds();
      path.forEach(p => bounds.extend(p));
      state.map.fitBounds(bounds, { padding: 80 });
    }

    function drawSteps(route) {
      if (!route || !route.steps) return;
      stepsList.innerHTML = "";
      route.steps.forEach((step, i) => {
        const li = document.createElement("li");
        li.className = "step-item" + (i === 0 ? " active-step" : "");
        li.id = `replay-step-${i}`;
        const cmd   = (typeof MANEUVER_MAP !== "undefined" && MANEUVER_MAP[step.maneuver]) || "front";
        const arrow = (typeof ARROW_SVG !== "undefined" && ARROW_SVG[cmd]) || "";
        const dist  = typeof fmtDist === "function"
          ? fmtDist(step.distance_m || 0)
          : `${Math.round(step.distance_m || 0)} m`;
        li.innerHTML = `
          <div class="step-icon">${arrow}</div>
          <div class="step-text">${step.instruction || ""}</div>
          <div class="step-dist">${dist}</div>
        `;
        stepsList.appendChild(li);
      });
      stepsSection.classList.remove("hidden");
    }

    function highlightReplayStep(idx) {
      const prev = stepsList.querySelector(".active-step");
      if (prev) prev.classList.remove("active-step");
      const cur = $(`replay-step-${idx}`);
      if (cur) {
        cur.classList.add("active-step");
        cur.scrollIntoView({ block: "nearest", behavior: "smooth" });
      }
    }

    function setUserMarker(lat, lng, heading) {
      if (!hasMap()) return;
      const ll = new google.maps.LatLng(lat, lng);
      if (!replay.userMarker) {
        replay.userMarker = new google.maps.Marker({
          position: ll, map: state.map,
          zIndex: 999,
          icon: {
            path: google.maps.SymbolPath.FORWARD_CLOSED_ARROW,
            scale: 5, rotation: heading || 0,
            fillColor: "#9f7aea", fillOpacity: 1,
            strokeColor: "#fff", strokeWeight: 2,
          },
          title: "Replay user position",
        });
      } else {
        replay.userMarker.setPosition(ll);
        const ic = replay.userMarker.getIcon();
        if (ic && typeof ic === "object") {
          replay.userMarker.setIcon({ ...ic, rotation: heading || 0 });
        }
      }

      replay.pathPoints.push({ lat, lng });
      if (!replay.pathTrail) {
        replay.pathTrail = new google.maps.Polyline({
          path: replay.pathPoints,
          strokeColor: "#c5a6ff",
          strokeWeight: 3,
          strokeOpacity: 0.7,
          map: state.map,
        });
      } else {
        replay.pathTrail.setPath(replay.pathPoints);
      }

      if (headingCompass) headingCompass.classList.remove("hidden");
      if (compassNeedle && heading != null) {
        compassNeedle.setAttribute("transform", `rotate(${heading}, 28, 28)`);
      }
    }

    // -------------------------------------------------------------------------
    // Frame handlers (called by NavTimelinePlayer)
    // -------------------------------------------------------------------------
    const HAZARD_ZONE_CLASS = {
      CLEAR: "", ALERT: "level-1", CAUTION: "level-2",
      WARNING: "level-3", DANGER: "level-4", CRITICAL: "level-5",
    };

    const handlers = {
      route(ev) {
        replay.route = ev.route;
        clearMapArtefacts();
        drawRoute(ev.route);
        drawSteps(ev.route);
        if (instructionCard) instructionCard.classList.remove("hidden");
        if (instructionMain) instructionMain.textContent =
          `Replaying route to ${ev.route.dest_name || "destination"}`;
        if (instructionDist) instructionDist.textContent =
          ev.route.total_distance_m
            ? `${Math.round(ev.route.total_distance_m)} m total`
            : "";
      },

      position(ev) {
        setUserMarker(ev.lat, ev.lng, ev.heading || 0);
      },

      command(ev) {
        if (typeof setActiveMotor === "function")     setActiveMotor(ev.cmd);
        if (typeof setInstructionArrow === "function") setInstructionArrow(ev.cmd);
        if (instructionCard) instructionCard.classList.remove("hidden");
        if (instructionMain) instructionMain.textContent =
          ev.instruction || ev.cmd.replace("_", " ").toUpperCase();
        if (instructionDist) instructionDist.textContent =
          ev.distance_m != null ? `${Math.round(ev.distance_m)} m` : "";
        if (motorBadge) motorBadge.textContent =
          ev.cmd.toUpperCase().replace("_", " ");
        if (ev.step_index != null) highlightReplayStep(ev.step_index);
      },

      deliver(ev) {
        if (typeof setActiveMotor === "function")     setActiveMotor(ev.cmd);
        if (typeof setInstructionArrow === "function") setInstructionArrow(ev.cmd);
        if (motorBadge) motorBadge.textContent =
          ev.cmd.toUpperCase().replace("_", " ");
      },

      hazard(ev) {
        if (!hazardBadge || !hazardLabel) return;
        hazardLabel.textContent = ev.label || "CLEAR";
        hazardBadge.className   = `hazard-badge ${HAZARD_ZONE_CLASS[ev.label] || ""}`;
        if (hazardAlerts) {
          hazardAlerts.innerHTML = "";
          if (ev.alert_count > 0 && ev.label !== "CLEAR") {
            const chip = document.createElement("div");
            const cls  = (ev.label === "DANGER" || ev.label === "CRITICAL")
              ? "danger" : ev.label === "WARNING" ? "warning" : "caution";
            chip.className = `hazard-alert-chip ${cls}`;
            chip.textContent = `replay  ${ev.label}  (${ev.alert_count})`;
            hazardAlerts.appendChild(chip);
          }
        }
      },

      imu(ev) {
        if (compassNeedle && ev.yaw_deg != null) {
          compassNeedle.setAttribute(
            "transform", `rotate(${ev.yaw_deg}, 28, 28)`);
        }
        if (headingCompass) headingCompass.classList.remove("hidden");
      },

      stop() {
        if (typeof setActiveMotor === "function") setActiveMotor("stop");
        if (motorBadge) motorBadge.textContent = "STOP";
        if (instructionMain) instructionMain.textContent = "Navigation stopped";
        if (instructionDist) instructionDist.textContent = "";
      },

      tick(elapsed, total /* , idx */) {
        if (timeLabel) timeLabel.textContent =
          `${fmtTime(elapsed)} / ${fmtTime(total)}`;
        if (slider && total > 0 && !slider._dragging) {
          slider.value = Math.round((elapsed / total) * 1000);
        }
        if (playBtn) playBtn.textContent =
          replay.player && replay.player.playing ? "Pause" : "Play";
      },

      rewind() {
        // Reset visible state so a seek to t=0 doesn't carry leftovers.
        clearMapArtefacts();
        if (typeof setActiveMotor === "function") setActiveMotor("front");
        if (motorBadge) motorBadge.textContent = "—";
        if (instructionCard) instructionCard.classList.add("hidden");
        if (stepsSection)    stepsSection.classList.add("hidden");
        if (hazardLabel)     hazardLabel.textContent = "CLEAR";
        if (hazardBadge)     hazardBadge.className = "hazard-badge";
        if (hazardAlerts)    hazardAlerts.innerHTML = "";
      },
    };

    // -------------------------------------------------------------------------
    // File loading
    // -------------------------------------------------------------------------
    function loadFile(file) {
      if (!file) return;
      replay.filename = file.name;
      filenameLabel.textContent = file.name;
      const reader = new FileReader();
      reader.onload = () => {
        const text   = reader.result;
        const events = window.OmniSenseLog.parseNavTimeline(text);
        if (!events.length) {
          alert("No navigation events found in this log.\n" +
                "Make sure the file was recorded after [NAV:ROUTE_DETAIL] and " +
                "[NAV:POSITION] logging was added.");
          return;
        }
        replay.events = events;
        if (replay.player) replay.player.pause();
        replay.player = new window.OmniSenseLog.NavTimelinePlayer({
          events, handlers, speed: replay.speed, tickMs: 150,
        });
        controls.classList.add("active");

        // Render the first route immediately so the map shows the path even
        // before the user hits Play.
        const firstRoute = events.find(e => e.kind === "route");
        if (firstRoute) handlers.route(firstRoute);

        const firstPos = events.find(e => e.kind === "position");
        if (firstPos)   handlers.position(firstPos);

        replay.player.seek(0);
      };
      reader.readAsText(file);
    }

    // -------------------------------------------------------------------------
    // UI wiring
    // -------------------------------------------------------------------------
    function enableReplay() {
      replay.enabled = true;
      replayModeBtn.classList.add("active");
      replayModeBtn.innerHTML = `
        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <rect x="6" y="6" width="12" height="12" rx="1.5"/>
        </svg>
        Disable Log Replay`;
      replayUI.style.display = "block";
      if (replayBanner) replayBanner.classList.remove("hidden");
      suspendLive();
      setLiveControlsEnabled(false);
    }

    function disableReplay() {
      replay.enabled = false;
      if (replay.player) replay.player.pause();
      replayModeBtn.classList.remove("active");
      replayModeBtn.innerHTML = `
        <svg width="14" height="14" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
          <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
          <polyline points="7 10 12 15 17 10"/><line x1="12" y1="15" x2="12" y2="3"/>
        </svg>
        Enable Log Replay`;
      replayUI.style.display = "none";
      if (replayBanner) replayBanner.classList.add("hidden");
      controls.classList.remove("active");
      clearMapArtefacts();
      handlers.rewind();
      setLiveControlsEnabled(true);
      resumeLive();
    }

    replayModeBtn.addEventListener("click", () => {
      replay.enabled ? disableReplay() : enableReplay();
    });

    dropZone.addEventListener("click", () => fileInput.click());
    fileInput.addEventListener("change", e => {
      const f = e.target.files && e.target.files[0];
      if (f) loadFile(f);
    });
    ["dragover", "dragenter"].forEach(ev =>
      dropZone.addEventListener(ev, e => {
        e.preventDefault(); e.stopPropagation();
        dropZone.classList.add("drag");
      }));
    ["dragleave", "dragend"].forEach(ev =>
      dropZone.addEventListener(ev, e => {
        e.preventDefault(); e.stopPropagation();
        dropZone.classList.remove("drag");
      }));
    dropZone.addEventListener("drop", e => {
      e.preventDefault(); e.stopPropagation();
      dropZone.classList.remove("drag");
      const f = e.dataTransfer.files && e.dataTransfer.files[0];
      if (f) loadFile(f);
    });

    playBtn.addEventListener("click", () => {
      if (!replay.player) return;
      replay.player.toggle();
      playBtn.textContent = replay.player.playing ? "Pause" : "Play";
    });

    speedBtns.forEach(btn => {
      if (!btn) return;
      btn.addEventListener("click", () => {
        const s = +btn.id.replace("replay-", "").replace("x", "");
        replay.speed = s;
        if (replay.player) replay.player.setSpeed(s);
        speedBtns.forEach(b => b && b.classList.remove("active"));
        btn.classList.add("active");
      });
    });

    clearBtn.addEventListener("click", () => {
      if (replay.player) replay.player.pause();
      replay.events  = [];
      replay.route   = null;
      replay.player  = null;
      controls.classList.remove("active");
      filenameLabel.textContent = "—";
      timeLabel.textContent     = "0.0 s / 0.0 s";
      handlers.rewind();
    });

    slider.addEventListener("mousedown", () => { slider._dragging = true; });
    slider.addEventListener("touchstart",() => { slider._dragging = true; });
    slider.addEventListener("input", () => {
      if (!replay.player) return;
      const total = replay.player.durationMs();
      const tMs   = (slider.value / 1000) * total;
      replay.player.seek(tMs);
      replay.player.elapsed = tMs;
    });
    slider.addEventListener("change",   () => { slider._dragging = false; });
    slider.addEventListener("mouseup",  () => { slider._dragging = false; });
    slider.addEventListener("touchend", () => { slider._dragging = false; });
  }
})();
