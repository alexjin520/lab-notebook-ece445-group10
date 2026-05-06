/* =============================================================================
   OmniSense — log replay engine
   -----------------------------------------------------------------------------
   Shared parser + playback controller used by the body / head dashboards to
   replay a recorded `omnisense_*.log` file through the existing live render
   pipeline.

   Exposes via `window.OmniSenseLog`:
     - parseOmnisenseLog(text)            → Frame[]  (raw, both modules)
     - filterFramesByModule(frames, mod)  → Frame[]  (only mod)
     - projectBodyFrame(frame)            → object   (shape consumed by the
                                                       body dashboard's
                                                       renderAll)
     - projectHeadFrame(frame)            → object   (shape consumed by the
                                                       head dashboard's
                                                       renderAll)
     - new LogReplay({...})               → playback controller bound to the
                                             DOM ids of the dashboard's replay
                                             toolbar + drop zone

   The parser purposefully tolerates a wide range of formatting (extra spaces,
   missing fields, mixed module logs, etc.) so old log files still play back.
   ============================================================================= */

(function () {
  "use strict";

  const DIRS = ["front", "front_right", "right", "back_right",
                "back",  "back_left",   "left",  "front_left"];

  // ===========================================================================
  // Parsing
  // ===========================================================================

  /**
   * Top-level parser. Splits the log into PKT blocks (delimited by
   * `┌─ PKT #N` / `└─ PKT #N`) and converts each one into a Frame object.
   *
   * Some events (`[NAV:TURN_START]`, `[NAV:TURN_DONE]`, `[NAV:COMMAND]`)
   * are emitted *outside* a PKT block but logically apply to subsequent
   * frames, so the parser carries them in a small piece of running state.
   */
  function parseOmnisenseLog(text) {
    const lines  = text.split(/\r?\n/);
    const frames = [];
    let block    = null;

    let lastNavCmd = null;   // most recent [NAV:COMMAND] payload
    let lastTurn   = null;   // most recent turn-tracking snapshot

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];

      // Out-of-band NAV events (apply to all later frames until superseded)
      const navStart = line.match(
        /\[NAV:TURN_START\]\s+command=(\S+)\s+expected=([-\d.]+)/);
      if (navStart) {
        lastTurn = {
          active: true, completed: false,
          command: navStart[1],
          expected_deg: +navStart[2],
          progress_deg: 0,
          completion_pct: 0,
          start_yaw: null, current_yaw: null,
          elapsed_s: 0, timeout_s: 30,
        };
      }
      const navDone = line.match(
        /\[NAV:TURN_DONE\]\s+command=(\S+)\s+expected=([-\d.]+)\S*\s+actual=([-\d.]+)/);
      if (navDone && lastTurn) {
        lastTurn = { ...lastTurn,
          completed: true, active: false,
          progress_deg: +navDone[3], completion_pct: 1.0 };
      }
      const navTimeout = line.match(/\[NAV:TURN_TIMEOUT\]/);
      if (navTimeout && lastTurn) {
        lastTurn = { ...lastTurn,
          active: false, completed: false, timed_out: true };
      }
      const navCmd = line.match(
        /\[NAV:COMMAND\]\s+cmd=(\S+)\s+motor=(-?\d+)\s+intensity=(\d+)\s+dist=(\S+)\s+phase=(\S+)(?:\s+step=(\S+))?\s+instr='(.*?)'/);
      if (navCmd) {
        lastNavCmd = {
          command: navCmd[1],
          motor_id: +navCmd[2],
          intensity: +navCmd[3],
          distance_m: navCmd[4] === "None" ? null : +navCmd[4],
          phase: navCmd[5],
          instruction: navCmd[7],
          pattern: "off",
          active: navCmd[1] !== "stop",
        };
      }

      // PKT block boundaries
      if (line.includes("┌─ PKT #")) {
        const m = line.match(/PKT #(\d+)\s+device=(\S+)\s+module=(\S+)/);
        if (m) {
          block = { pkt: +m[1], device: m[2], module: m[3], lines: [line] };
        }
      } else if (block) {
        block.lines.push(line);
        if (line.includes("└─ PKT #")) {
          const f = parseFrame(block, lastNavCmd, lastTurn);
          if (f) frames.push(f);
          block = null;
        }
      }
    }
    return frames;
  }

  // ---------------------------------------------------------------------------
  // Nav-specific timeline parser (used by the navigation page replay)
  //
  // Unlike the per-packet parser above, the navigation UI cares about events
  // that happen *between* PKT blocks — most notably user GPS pings, route
  // pushes, and turn-by-turn step commands. We walk every line in time order
  // and emit a flat array of NavEvent objects:
  //
  //   { ts, kind: "route"   , route:    {...} }     // [NAV:ROUTE_DETAIL]
  //   { ts, kind: "position", lat,lng,heading }     // [NAV:POSITION]
  //   { ts, kind: "command" , cmd, motor_id,
  //                            intensity, distance_m,
  //                            phase, step_index, instruction }
  //   { ts, kind: "stop" }                          // [NAV:STOP]
  //   { ts, kind: "deliver" , cmd, motor_id, intensity, pattern, active }
  //   { ts, kind: "hazard"  , label, alerts, level }   // pulled from PKT
  //   { ts, kind: "imu"     , module, yaw_deg, ... }    // pulled from PKT
  //
  // The replay player on index.html walks this list with a clock so the map,
  // motor ring, instruction card, hazard panel and compass animate in lock-
  // step the same way they did during the original recording.
  // ---------------------------------------------------------------------------

  function parseNavTimeline(text) {
    const lines  = text.split(/\r?\n/);
    const events = [];
    let routeAccumulator = null;   // when ROUTE_DETAIL spans multi-line JSON

    for (let i = 0; i < lines.length; i++) {
      const line = lines[i];
      const tsM  = line.match(/^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})/);
      const ts   = tsM ? tsM[1] : null;

      // Route detail (full JSON dump on one line)
      const rd = line.match(/\[NAV:ROUTE_DETAIL\]\s+(\{.*\})\s*$/);
      if (rd) {
        try {
          const route = JSON.parse(rd[1]);
          events.push({ ts, kind: "route", route });
        } catch (e) {
          console.warn("Could not parse NAV:ROUTE_DETAIL JSON:", e);
        }
        continue;
      }

      // Position update
      const pos = line.match(
        /\[NAV:POSITION\]\s+lat=([-\d.]+)\s+lng=([-\d.]+)\s+heading=(\S+)/);
      if (pos) {
        events.push({
          ts, kind: "position",
          lat: +pos[1], lng: +pos[2],
          heading: pos[3] === "None" ? null : +pos[3],
        });
        continue;
      }

      // Phone-issued nav command
      const cmd = line.match(
        /\[NAV:COMMAND\]\s+cmd=(\S+)\s+motor=(-?\d+)\s+intensity=(\d+)\s+dist=(\S+)\s+phase=(\S+)(?:\s+step=(\S+))?\s+instr='(.*?)'/);
      if (cmd) {
        events.push({
          ts, kind: "command",
          cmd:        cmd[1],
          motor_id:  +cmd[2],
          intensity: +cmd[3],
          distance_m: cmd[4] === "None" ? null : +cmd[4],
          phase:      cmd[5],
          step_index: (cmd[6] && cmd[6] !== "?") ? +cmd[6] : null,
          instruction: cmd[7],
        });
        continue;
      }

      if (line.includes("[NAV:STOP]")) {
        events.push({ ts, kind: "stop" });
        continue;
      }

      // Server → body delivery (what the firmware actually receives)
      const deliver = line.match(
        /\[NAV:BODY_DELIVER\].*?\bcmd=(\S+)\s+motor=(-?\d+)\s+intensity=(\d+)\s+pattern=(\S+)\s+active=(\w+)/);
      if (deliver) {
        events.push({
          ts, kind: "deliver",
          cmd: deliver[1], motor_id: +deliver[2],
          intensity: +deliver[3], pattern: deliver[4],
          active: deliver[5] === "True",
        });
        continue;
      }

      // Hazard detection (in-band, useful for the hazard panel)
      const haz = line.match(
        /\[VERIFY:HAZARD_DETECT\][^l]*level=(\d+)\s+label=(\S+).*?alert_count=(\d+)/);
      if (haz) {
        events.push({
          ts, kind: "hazard",
          level: +haz[1], label: haz[2], alert_count: +haz[3],
        });
        continue;
      }

      // Body IMU heading — drives the compass needle
      const imu = line.match(
        /\[VERIFY:IMU_READ\][^m]*module=body\s+roll=\s*([-\d.]+)\s+pitch=\s*([-\d.]+)\s+yaw=\s*([-\d.]+)/);
      if (imu) {
        events.push({
          ts, kind: "imu",
          roll_deg: +imu[1], pitch_deg: +imu[2], yaw_deg: +imu[3],
        });
        continue;
      }
    }
    return events;
  }

  function parseFrame(block, navCmd, turn) {
    const frame = {
      pkt: block.pkt,
      device: block.device,
      module: block.module,                // "head" | "body"
      timestamp: null,

      tof_body: {}, tof_head: {},
      mmwave_body: {}, mmwave_head: {},
      imu_body: {}, imu_head: {},
      fused: {},
      motors: [],

      hazard:   { level: 0, label: "CLEAR", alerts: [] },
      battery:  {}, system: {},
      uptime_s: null, pkt_rate_hz: null,
      proc_ms:  null, latency_ms: null,
      pkt_count: block.pkt,

      nav_motor:   navCmd ? { ...navCmd } :
                            { command: "stop", motor_id: -1, intensity: 0,
                              pattern: "off", active: false, instruction: "",
                              distance_m: null, phase: "idle" },
      turn_status: turn ? { ...turn } : {},
      online: true,
    };

    let tofCount     = 0;
    const tofRaw     = [{}, {}];
    let mmwaveCount  = 0;
    const mmwaveRaw  = [{}, {}];

    for (const line of block.lines) {
      const tsM = line.match(/^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})/);
      if (tsM && !frame.timestamp) frame.timestamp = tsM[1];

      // ── TOF_RAW ──────────────────────────────────────────────────────────
      if (line.includes("[VERIFY:TOF_RAW]")) {
        const tof = {};
        DIRS.forEach(d => {
          const m = line.match(new RegExp("\\b" + d + "=(None|[\\d.]+)m?"));
          if (m) tof[d] = m[1] === "None" ? null : parseFloat(m[1]);
        });
        if (tofCount < 2) tofRaw[tofCount] = tof;
        tofCount++;
      }

      // ── MMWAVE_RAW ───────────────────────────────────────────────────────
      if (line.includes("[VERIFY:MMWAVE_RAW]")) {
        const m = line.match(
          /sensor=(\S+)\s+target=(\w+)(?:\s+range=([\d.]+)m?)?(?:\s+speed=([\d.]+))?/);
        if (m) {
          const sensor    = m[1];
          const hasTarget = m[2] === "yes";
          const entry = {
            targets:        hasTarget ? 1 : 0,
            target_present: hasTarget,
            range_m:  m[3] ? +m[3] : null,
            speed_ms: m[4] ? +m[4] : null,
          };
          // Server logs body mmwave first, then head (each pair is two
          // sensors: front + back).
          if (mmwaveCount < 2) mmwaveRaw[0][sensor] = entry;
          else                 mmwaveRaw[1][sensor] = entry;
          mmwaveCount++;
        }
      }

      // ── IMU_READ ─────────────────────────────────────────────────────────
      if (line.includes("[VERIFY:IMU_READ]")) {
        const m = line.match(
          /module=(\w+)\s+roll=\s*([-\d.]+)\s+pitch=\s*([-\d.]+)\s+yaw=\s*([-\d.]+)\s+accel_z=\s*([-\d.]+)/);
        if (m) {
          const mod = m[1];
          const imu = { roll_deg: +m[2], pitch_deg: +m[3],
                        yaw_deg:  +m[4], accel_z:   +m[5],
                        present: true };
          if (mod === "head") frame.imu_head = imu;
          else                frame.imu_body = imu;
        }
      }

      // ── FUSE ─────────────────────────────────────────────────────────────
      if (line.includes("[VERIFY:FUSE]")) {
        const m = line.match(
          /dir=(\S+)\s+eff_m=\s*(\S+)\s+source=(\S+)\s+zone=(\S+)/);
        if (m) {
          frame.fused[m[1]] = {
            dist_m: m[2] === "None" ? null : parseFloat(m[2]),
            source: m[3],
            zone:   m[4],
          };
        }
      }

      // ── MOTOR_CMD ────────────────────────────────────────────────────────
      if (line.includes("[VERIFY:MOTOR_CMD]")) {
        const activeM   = line.match(/active=\[(.+)\]/);
        const allMotors = DIRS.map((dir, i) => ({
          motor_id: i, direction: dir, zone: "CLEAR",
          intensity: 0, frequency_hz: 0, pattern: "off",
          active: false, distance_m: null, source: "none",
        }));
        if (activeM) {
          const re = /(\w+)\(id=(\d+),zone=(\w+),int=(\d+),freq=(\d+)Hz\)/g;
          let mm;
          while ((mm = re.exec(activeM[1])) !== null) {
            const idx = +mm[2];
            if (allMotors[idx]) {
              allMotors[idx] = { ...allMotors[idx],
                direction:    mm[1],
                zone:         mm[3],
                intensity:   +mm[4],
                frequency_hz:+mm[5],
                active:       true };
            }
          }
        }
        frame.motors = allMotors;
      }

      // ── HAZARD_DETECT ────────────────────────────────────────────────────
      if (line.includes("[VERIFY:HAZARD_DETECT]")) {
        const m  = line.match(/level=(\d+)\s+label=(\S+)/);
        const am = line.match(/alert_count=(\d+)/);
        if (m) {
          frame.hazard = {
            level: +m[1], label: m[2],
            alerts: am ? Array(+am[1]).fill({}) : [],
          };
        }
      }

      // ── POWER ────────────────────────────────────────────────────────────
      if (line.includes("[VERIFY:POWER]")) {
        const m = line.match(
          /voltage_v=([\d.]+)\s+soc_pct=([\d.]+)\s+temp_c=([\d.]+)\s+heap_b=(\d+)\s+rssi=(-?\d+)\s+uptime_s=([\d.]+)/);
        if (m) {
          frame.battery = { voltage_v: +m[1], soc_pct: +m[2] };
          frame.system  = { die_temp_c: +m[3], free_heap_b: +m[4],
                            wifi_rssi: +m[5] };
          frame.uptime_s = +m[6];
        }
      }

      if (line.includes("[VERIFY:PKT_RATE]")) {
        const m = line.match(/rate_hz=([\d.]+)/);
        if (m) frame.pkt_rate_hz = +m[1];
      }
      if (line.includes("[VERIFY:PROC_TIME]")) {
        const m = line.match(/proc_ms=([\d.]+)/);
        if (m) frame.proc_ms = +m[1];
      }
      if (line.includes("[VERIFY:LATENCY_WARN]") ||
          line.includes("[VERIFY:LATENCY_INFO]")) {
        const m = line.match(/latency_ms=([\d.]+)/);
        if (m) frame.latency_ms = +m[1];
      }

      // ── BODY_DELIVER (overrides nav_motor for this frame) ────────────────
      if (line.includes("[NAV:BODY_DELIVER]")) {
        const m = line.match(
          /cmd=(\S+)\s+motor=(-?\d+)\s+intensity=(\d+)\s+pattern=(\S+)\s+active=(\w+)/);
        if (m) {
          frame.nav_motor = {
            command:  m[1],
            motor_id: +m[2],
            intensity:+m[3],
            pattern:  m[4],
            active:   m[5] === "True",
            instruction: navCmd ? navCmd.instruction : "",
            distance_m:  navCmd ? navCmd.distance_m  : null,
            phase:       navCmd ? navCmd.phase       : "idle",
          };
        }
      }
    }

    // The server emits TWO TOF_RAW lines per packet, one for each module.
    // The "absent" module's reading is all-null. Use that to figure out
    // which slot belongs to which module.
    const allNull = arr => Object.values(arr).every(v => v === null);
    const n0 = allNull(tofRaw[0]);
    const n1 = allNull(tofRaw[1]);
    if (n0 && !n1)      { frame.tof_head = tofRaw[0]; frame.tof_body = tofRaw[1]; }
    else if (!n0 && n1) { frame.tof_head = tofRaw[1]; frame.tof_body = tofRaw[0]; }
    else                { frame.tof_body = tofRaw[0]; frame.tof_head = tofRaw[1]; }

    // mmWave order in the log: body pair, then head pair.
    frame.mmwave_body = mmwaveRaw[0];
    frame.mmwave_head = mmwaveRaw[1];

    return frame;
  }

  function filterFramesByModule(frames, mod) {
    if (!mod) return frames;
    return frames.filter(f => f.module === mod);
  }

  // ===========================================================================
  // Per-module projections
  //   These adapt the generic Frame schema into the specific shape consumed
  //   by each dashboard's existing renderAll() function. The goal is that
  //   replay reuses the live render pipeline byte-for-byte.
  // ===========================================================================

  function projectBodyFrame(f) {
    const imu = f.imu_body || {};
    const orientation = imu.yaw_deg !== undefined
      ? { yaw_deg: imu.yaw_deg, mag_calibrated: !!imu.mag_calibrated,
          updated_at: 0 }
      : {};
    return {
      online: true,
      tof:        f.tof_body    || {},
      mmwave:     f.mmwave_body || {},
      imu:        imu,
      fused:      f.fused       || {},
      hazard:     f.hazard,
      nav_motor:  f.nav_motor   || {},
      turn_status: f.turn_status || {},
      orientation,
      battery:    f.battery     || {},
      system:     f.system      || {},
      uptime_s:   f.uptime_s,
      pkt_rate_hz:f.pkt_rate_hz,
      proc_ms:    f.proc_ms,
      latency_ms: f.latency_ms,
      packet_count: f.pkt_count,
      device_id:  f.device,
      _replay:    true,
      _ts:        f.timestamp,
    };
  }

  function projectHeadFrame(f) {
    return {
      online: true,
      tof:        f.tof_head    || {},
      mmwave:     f.mmwave_head || {},
      imu:        f.imu_head    || {},   // empty in current builds → "no IMU"
      fused:      f.fused       || {},
      hazard:     f.hazard,
      motors:     f.motors      || [],
      battery:    f.battery     || {},
      system:     f.system      || {},
      uptime_s:   f.uptime_s,
      pkt_rate_hz:f.pkt_rate_hz,
      proc_ms:    f.proc_ms,
      latency_ms: f.latency_ms,
      packet_count: f.pkt_count,
      device_id:  f.device,
      _replay:    true,
      _ts:        f.timestamp,
    };
  }

  // ===========================================================================
  // Playback controller
  // ===========================================================================

  /**
   * @param {Object} opts
   * @param {string}   opts.module          "body" | "head" (filter target)
   * @param {string}   opts.dropZoneId
   * @param {string}   opts.fileInputId
   * @param {string}   opts.playbackWrapId
   * @param {string}   opts.sliderId
   * @param {string}   opts.counterId
   * @param {string}   opts.playBtnId
   * @param {string}   opts.infoId
   * @param {(view:object, frame:object)=>void} opts.renderFrame
   * @param {(frame:object)=>object} [opts.projectFrame]   defaults to identity
   */
  class LogReplay {
    constructor(opts) {
      Object.assign(this, opts);
      this.frames  = [];
      this.idx     = 0;
      this.playing = false;
      this.timer   = null;
      this.speed   = 10;          // frames per second
      if (!this.projectFrame) this.projectFrame = f => f;
      this._wireDom();
    }

    _wireDom() {
      const fi = document.getElementById(this.fileInputId);
      if (fi) fi.addEventListener("change", e => {
        if (e.target.files && e.target.files[0]) this.loadFile(e.target.files[0]);
      });

      const dz = document.getElementById(this.dropZoneId);
      if (dz) {
        dz.addEventListener("click", () => fi && fi.click());
        dz.addEventListener("dragover", e => {
          e.preventDefault();
          dz.classList.add("drag-over");
        });
        dz.addEventListener("dragleave", () =>
          dz.classList.remove("drag-over"));
        dz.addEventListener("drop", e => {
          e.preventDefault();
          dz.classList.remove("drag-over");
          const file = e.dataTransfer.files[0];
          if (file) this.loadFile(file);
        });
      }

      const sl = document.getElementById(this.sliderId);
      if (sl) sl.addEventListener("input", () => this.seek(+sl.value));
    }

    loadFile(file) {
      if (!file) return;
      const reader = new FileReader();
      reader.onload = e => {
        const text     = e.target.result;
        const all      = parseOmnisenseLog(text);
        const filtered = filterFramesByModule(all, this.module);
        this.frames    = filtered;
        this.idx       = 0;

        const dz = document.getElementById(this.dropZoneId);
        const pw = document.getElementById(this.playbackWrapId);
        if (dz) dz.style.display = "none";
        if (pw) pw.style.display = "";

        const sl = document.getElementById(this.sliderId);
        if (sl) {
          sl.max   = Math.max(0, this.frames.length - 1);
          sl.value = 0;
        }

        const info = document.getElementById(this.infoId);
        if (info) {
          if (this.frames.length === 0) {
            info.textContent =
              `No ${this.module || ""} packets in ${file.name}`.trim();
          } else {
            const skipped = all.length - this.frames.length;
            info.textContent =
              `${this.frames.length} ${this.module || "all"} packets parsed from ${file.name}` +
              (skipped > 0 ? `  (${skipped} other-module skipped)` : "");
          }
        }
        if (this.frames.length) this.render(0);
      };
      reader.readAsText(file);
    }

    render(idx) {
      if (!this.frames.length) return;
      idx = Math.max(0, Math.min(idx, this.frames.length - 1));
      this.idx = idx;
      const f = this.frames[idx];

      const sl = document.getElementById(this.sliderId);
      if (sl) sl.value = idx;
      const ctr = document.getElementById(this.counterId);
      if (ctr) ctr.textContent =
        `PKT #${f.pkt}  (${idx + 1}/${this.frames.length})`;

      const view = this.projectFrame(f);
      this.renderFrame(view, f);
    }

    seek(idx)      { this.render(idx); }
    stepForward()  { if (this.idx < this.frames.length - 1) this.render(this.idx + 1); }
    stepBack()     { if (this.idx > 0) this.render(this.idx - 1); }

    setSpeed(s) {
      this.speed = s;
      if (this.playing) { this._stop(); this._start(); }
    }
    _stop()  { if (this.timer) { clearInterval(this.timer); this.timer = null; } }
    _start() {
      this.timer = setInterval(() => {
        if (this.idx >= this.frames.length - 1) { this.toggle(); return; }
        this.render(this.idx + 1);
      }, 1000 / this.speed);
    }
    toggle() {
      this.playing = !this.playing;
      const btn = document.getElementById(this.playBtnId);
      if (btn) btn.textContent = this.playing ? "⏸ Pause" : "▶ Play";
      if (this.playing) this._start(); else this._stop();
    }

    clear() {
      this._stop();
      this.frames = []; this.idx = 0; this.playing = false;
      const dz = document.getElementById(this.dropZoneId);
      const pw = document.getElementById(this.playbackWrapId);
      if (dz) dz.style.display = "";
      if (pw) pw.style.display = "none";
      const btn = document.getElementById(this.playBtnId);
      if (btn) btn.textContent = "▶ Play";
      const info = document.getElementById(this.infoId);
      if (info) info.textContent = "";
      const fi = document.getElementById(this.fileInputId);
      if (fi) fi.value = "";
    }

    hasFrames() { return this.frames.length > 0; }
  }

  // ===========================================================================
  // Navigation timeline player
  //   The nav page replays a *time-ordered* list of events, not per-packet
  //   frames. We expose a tiny driver that walks the timeline at a chosen
  //   speed and dispatches each event through user-supplied hooks.
  // ===========================================================================

  function tsToMs(ts) {
    // "2026-04-25 20:43:39.052" → epoch-style ms
    if (!ts) return 0;
    const m = ts.match(
      /^(\d{4})-(\d{2})-(\d{2}) (\d{2}):(\d{2}):(\d{2})\.(\d{3})/);
    if (!m) return 0;
    return Date.UTC(+m[1], +m[2] - 1, +m[3],
                    +m[4], +m[5], +m[6], +m[7]);
  }

  class NavTimelinePlayer {
    /**
     * @param {Object} opts
     * @param {Array}  opts.events         output of parseNavTimeline
     * @param {Object} opts.handlers       { route, position, command, deliver,
     *                                       hazard, imu, stop, tick }
     * @param {number} [opts.speed=4]      playback speed multiplier
     * @param {number} [opts.tickMs=200]   render granularity
     */
    constructor(opts) {
      this.events   = opts.events || [];
      this.handlers = opts.handlers || {};
      this.speed    = opts.speed   ?? 4;
      this.tickMs   = opts.tickMs  ?? 200;
      this.idx      = 0;
      this.playing  = false;
      this.timer    = null;

      // Pre-compute ms timestamps + relative offsets from the first event
      this._tsMs = this.events.map(e => tsToMs(e.ts));
      this._t0   = this._tsMs[0] || 0;
    }

    durationMs() {
      if (!this.events.length) return 0;
      return (this._tsMs[this._tsMs.length - 1] || 0) - this._t0;
    }

    /** Drive forward to the absolute log-time `tMs` (relative to first event). */
    seek(tMs) {
      tMs = Math.max(0, Math.min(tMs, this.durationMs()));
      // Fire every event up to and including tMs in order
      this.idx = 0;
      // Reset listener-side state by emitting a synthetic "rewind" first
      if (this.handlers.rewind) this.handlers.rewind();
      while (this.idx < this.events.length &&
             (this._tsMs[this.idx] - this._t0) <= tMs) {
        this._dispatch(this.events[this.idx]);
        this.idx++;
      }
      this.elapsed = tMs;
      if (this.handlers.tick) this.handlers.tick(tMs, this.durationMs(), this.idx);
    }

    play() {
      if (this.playing) return;
      this.playing = true;
      let lastReal = performance.now();
      this.elapsed = this.elapsed ?? 0;
      this.timer = setInterval(() => {
        const now      = performance.now();
        const dtReal   = now - lastReal;
        lastReal = now;
        const dtLog    = dtReal * this.speed;
        this.elapsed   = (this.elapsed || 0) + dtLog;
        const max      = this.durationMs();
        if (this.elapsed >= max) {
          this.elapsed = max;
          this._advanceTo(this.elapsed);
          this.pause();
          return;
        }
        this._advanceTo(this.elapsed);
      }, this.tickMs);
    }
    pause() {
      this.playing = false;
      if (this.timer) { clearInterval(this.timer); this.timer = null; }
    }
    toggle() { this.playing ? this.pause() : this.play(); }
    setSpeed(s) { this.speed = s; }
    reset() {
      this.pause();
      this.idx = 0;
      this.elapsed = 0;
      if (this.handlers.rewind) this.handlers.rewind();
      if (this.handlers.tick) this.handlers.tick(0, this.durationMs(), 0);
    }

    _advanceTo(tMs) {
      while (this.idx < this.events.length &&
             (this._tsMs[this.idx] - this._t0) <= tMs) {
        this._dispatch(this.events[this.idx]);
        this.idx++;
      }
      if (this.handlers.tick) this.handlers.tick(tMs, this.durationMs(), this.idx);
    }

    _dispatch(ev) {
      const h = this.handlers[ev.kind];
      if (h) h(ev);
    }
  }

  window.OmniSenseLog = {
    parseOmnisenseLog,
    filterFramesByModule,
    projectBodyFrame,
    projectHeadFrame,
    parseNavTimeline,
    NavTimelinePlayer,
    tsToMs,
    LogReplay,
    DIRS,
  };
})();
