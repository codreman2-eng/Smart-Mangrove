#include "DropLogicEngine.h"
#include "Ballistics.h"

DropLogicEngine::DropLogicEngine(ConfigManager&   cfg,
                                 IPositionSource& pos,
                                 ActuatorModule&  modA,
                                 ActuatorModule&  modB,
                                 StateMachine&    sm,
                                 Logger&          log)
    : _cfg(cfg), _pos(pos), _modA(modA), _modB(modB), _sm(sm), _log(log) {}

void DropLogicEngine::update() {
    // --- In-flight sequence: advance it regardless of arm state ---
    // (an already-started drop must finish; ARM/DISARM shouldn't abort it
    // mid-flight — that's what the emergency STOP button is for)
    if (_phase == Phase::WAITING_A) {
        if (_modA.isDropping()) return;   // still running, poll() hasn't completed it yet

        uint8_t drop_id = _currentDropId();
        DropResult resA = _modA.lastResult();

        // Why it fired where it did — the gate's view at the trigger. Logged
        // here, not at the trigger: see _release_snap in the header. The
        // logger is still in its drop window this tick, so the row is not
        // sync'd until module B's window closes.
        if (_hover_drop) {
            const HoverInfo& h = _hover_snap;
            char ex[224];
            snprintf(ex, sizeof(ex),
                     "\"n\":%u,\"t\":%.0f,\"h\":%.1f,\"home\":%.1f,\"spd\":%.2f,"
                     "\"pspd\":%.2f,\"rearm\":%.1f,\"hold\":%.1f,\"max\":%.1f",
                     drop_id, h.flight_s, h.height_m, h.home_dist_m, h.speed_mps,
                     h.pos_speed, h.rearm_m, h.hold_s,
                     _cfg.config().hover.max_speed_mps);
            _log.logEvent("hover_release", ex);
        } else if (!_manual_pair) {
            char ex[192];
            snprintf(ex, sizeof(ex),
                     "\"id\":%u,\"dist\":%.2f,\"r\":%.1f,\"spd\":%.2f,\"vel\":%s,"
                     "\"pred\":%s,\"max\":%.1f,\"dwell\":%.1f",
                     drop_id, _release_snap.dist_m, _release_snap.radius_m,
                     _release_snap.speed_mps,
                     _release_snap.speed_valid ? "true" : "false",
                     _release_snap.predicted   ? "true" : "false",
                     _release_max_speed, _release_snap.dwell_s);
            _log.logEvent("release", ex);
        }

        _recordDrop(drop_id, 0, _modA.doorsDropped(), _pending_pos, resA);

        if (resA == DropResult::FAULT) {
            _sm.setFault("Module A fault during drop");
            _phase = Phase::IDLE;
            _hover_drop = false;
            return;
        }

        // Hover mode fires module A alone unless hover.use_b is set — B is
        // uncalibrated as of 2026-10-29.
        if (_hover_drop && !_cfg.config().hover.use_b) {
            _sm.setDropping(false);
            Serial.printf("[DLE] Hover drop %d complete (module A only)\n", drop_id);
            _hover_drop = false;
            _phase = Phase::IDLE;
            return;
        }

        // Module B fires immediately after A confirms.
        // Same defect (c) class: an ignored false return here would have the
        // WAITING_B branch log a phantom B drop on the next tick. B can run
        // out independently of A, so this is not covered by the check on the
        // trigger path.
        Serial.printf("[DLE] Module B — drop %d\n", drop_id);
        if (!_modB.startDrop()) {
            Serial.println("[DLE] *** Module B has no doors left — sequence "
                           "ends unbalanced ***");
            _recordDrop(drop_id, 1, _modB.doorsDropped(), _pending_pos,
                        DropResult::ALREADY_DONE);
            _sm.setDropping(false);
            _phase = Phase::IDLE;
            if (!_manual_pair && !_hover_drop) _next_drop_idx++;
            _manual_pair = false;
            _hover_drop  = false;
            return;
        }
        _phase = Phase::WAITING_B;
        return;
    }

    if (_phase == Phase::WAITING_B) {
        if (_modB.isDropping()) return;

        uint8_t drop_id = _currentDropId();
        DropResult resB = _modB.lastResult();
        _recordDrop(drop_id, 1, _modB.doorsDropped(), _pending_pos, resB);

        _sm.setDropping(false);
        Serial.printf("[DLE] Drop %d complete%s\n",
                      drop_id, _manual_pair ? " (pilot RC trigger)" : "");

        // A pilot-commanded or hover pair must not consume a planned GPS waypoint
        if (!_manual_pair && !_hover_drop) _next_drop_idx++;
        _manual_pair = false;
        _hover_drop  = false;
        _phase = Phase::IDLE;

        if (resB == DropResult::FAULT) {
            _sm.setFault("Module B fault during drop");
        }
        return;
    }

    // --- IDLE: track the approach always, fire only when armed ---
    // The approach is computed disarmed too, so the dashboard shows the
    // aircraft closing on the waypoint before the operator arms. Nothing
    // below the isArmed() check further down can move a servo.
    const bool hover_mode = _cfg.config().hover.enabled;
    if (missionComplete()) {
        _approach = ApproachInfo(); _approach.state = Approach::DONE;
        _dwell_start_ms = 0;
        _hold_start_ms  = 0;
        if (hover_mode) _hover.state = Hover::DONE;   // keep the last numbers on show
        else            _hover = HoverInfo();
        return;
    }

    _pos.update();
    Position cur = _pos.read();
    _trackFlight(cur);

    // Hover mode replaces the waypoint gate entirely; nothing below runs.
    if (hover_mode) {
        _approach = ApproachInfo();
        _dwell_start_ms = 0;
        _updateHover(cur);
        return;
    }
    _hover = HoverInfo();
    _hold_start_ms = 0;

    if (!cur.valid) { _approach = ApproachInfo(); _dwell_start_ms = 0; return; }

    const MissionConfig& mc  = _cfg.config();
    const DropPoint&     dp  = mc.drops[_next_drop_idx];
    const ReleaseConfig& rel = mc.release;

    // ── Phase 4: aim at the IMPACT point, not the release point ─────────
    // A propagule keeps the aircraft's horizontal velocity while it falls, so
    // testing the AIRCRAFT against the waypoint lands every seed downrange —
    // 6.4m at 5 m/s from 8m, twice the target radius. Predict where a seed
    // released right now would land, and test THAT instead.
    //
    // This is a substitution, not a rewrite: everything below is unchanged.
    // predictImpact() fails closed, so on the GNSS fallback, in SIM, or with no
    // velocity health, aim_* stay at the aircraft position and the behaviour is
    // exactly what it was before Phase 4. At zero velocity the prediction
    // degenerates to the same thing anyway.
    double aim_lat = cur.lat_deg, aim_lon = cur.lon_deg;
    float  lead_s  = 0.0f;
    const bool predicted = Ballistics::predictImpact(cur, aim_lat, aim_lon, &lead_s);

    float dist = haversineDistance(aim_lat,    aim_lon,
                                   dp.lat_deg, dp.lon_deg);

    const float speed    = cur.horizontalSpeed();
    const bool  in_rad   = (dist <= dp.radius_m);
    const bool  too_fast = cur.vel_valid && speed > rel.max_speed_mps;
    const float zone     = fmaxf(APPROACH_ZONE_M, 4.0f * dp.radius_m);
    const bool  armed    = _sm.isArmed();
    const uint32_t now_ms = millis();

    // ── Dwell ───────────────────────────────────────────────────────────
    // The window is in radius AND not too fast. The count runs only ARMED,
    // and any exit from the window — or a disarm — starts it over.
    const bool in_window = in_rad && !too_fast;
    if (!in_window || !armed) {
        _dwell_start_ms = 0;
    } else if (_dwell_start_ms == 0) {
        _dwell_start_ms = now_ms ? now_ms : 1;      // 0 means "not running"
        if (rel.dwell_s > 0.0f)
            Serial.printf("[DLE] WP%d in radius%s — settling %.1fs before release\n",
                          dp.id,
                          cur.vel_valid ? (" at " + String(speed, 1) + " m/s").c_str() : "",
                          rel.dwell_s);
    }
    const uint32_t dwell_ms   = (uint32_t)(rel.dwell_s * 1000.0f + 0.5f);
    const uint32_t dwelled_ms = _dwell_start_ms ? now_ms - _dwell_start_ms : 0;
    const bool     settled    = _dwell_start_ms && dwelled_ms >= dwell_ms;

    _approach.wp_id       = dp.id;
    _approach.dist_m      = dist;
    _approach.radius_m    = dp.radius_m;
    _approach.speed_mps   = cur.vel_valid ? speed : 0.0f;
    _approach.speed_valid = cur.vel_valid;
    _approach.predicted   = predicted;
    _approach.dwell_s     = dwelled_ms / 1000.0f;
    _approach.state = !in_rad      ? (dist <= zone ? Approach::APPROACHING : Approach::FAR)
                    : too_fast     ? Approach::SLOW_DOWN
                    : (armed && !settled) ? Approach::SETTLING
                    :                Approach::IN_RADIUS;

    if (!armed) return;

    // Rate-limited to 1Hz. This ran on EVERY loop iteration while armed —
    // ~30 characters at 115200 baud is ~2.6ms of blocking, thousands of
    // times per second, and Serial.printf() stalls once the TX buffer
    // fills. The distance readout was itself the largest source of drop
    // latency, and it only appeared while ARMED, which is precisely when
    // the lag was observed. Also emitted when the distance changes by more
    // than a metre, so an approach is still visible in real time.
    static uint32_t last_dist_ms  = 0;
    static float    last_dist_m   = -1.0f;
    if (now_ms - last_dist_ms >= 1000 || fabsf(dist - last_dist_m) > 1.0f) {
        last_dist_ms = now_ms;
        last_dist_m  = dist;
        if (predicted)
            Serial.printf("[DLE] WP%d impact=%.1fm (r=%.1fm) carry=%.1fm lead=%.2fs\n",
                          dp.id, dist, dp.radius_m,
                          Ballistics::carryDistance(cur), lead_s);
        else
            Serial.printf("[DLE] WP%d dist=%.1fm (r=%.1fm)\n", dp.id, dist, dp.radius_m);
    }

    if (in_rad && too_fast) {
        // Rate-limited like the distance line: this holds for as long as the
        // pilot takes to slow down, and must not flood the UART meanwhile.
        static uint32_t last_slow_ms = 0;
        if (now_ms - last_slow_ms >= 1000) {
            last_slow_ms = now_ms;
            Serial.printf("[DLE] WP%d in radius, %.1f m/s > %.1f — holding for slow-down\n",
                          dp.id, speed, rel.max_speed_mps);
        }
        return;
    }

    if (in_rad && settled) {
        Serial.printf("[DLE] >>> In radius for WP%d at %s after %.1fs — executing drop\n",
                      dp.id,
                      cur.vel_valid ? (String(speed, 1) + " m/s").c_str()
                                    : "unknown speed (no velocity, gate off)",
                      dwelled_ms / 1000.0f);
        _release_snap      = _approach;      // logged when module A finishes
        _release_max_speed = rel.max_speed_mps;
        _dwell_start_ms    = 0;
        _approach.state = Approach::DROPPING;
        _pending_idx = _next_drop_idx;
        _pending_pos = cur;
        _manual_pair = false;

        // ── Defect (c) ────────────────────────────────────────────────
        // startDrop()'s false return used to be ignored here, and _phase was
        // set to WAITING_A regardless. With no drop actually started, the
        // WAITING_A branch fired on the very next tick, recorded a drop using
        // whatever _lastResult happened to be holding, started module B, and
        // logged the whole thing as a SUCCESS with zero motion. The shipped
        // default mission is exactly that case: 10 waypoints, 5 doors.
        //
        // startManualPair() below has always guarded this correctly. The GPS
        // path simply never got the same check — the two trigger paths
        // disagreed, which is the same shape of bug as the two e-stop paths.
        if (!_modA.startDrop()) {
            Serial.printf("[DLE] *** WP%d NOT DROPPED — module A has no doors "
                          "left. Mission cannot continue. ***\n", dp.id);
            _recordDrop(dp.id, 0, _modA.doorsDropped(), cur,
                        DropResult::ALREADY_DONE);
            return;   // _phase stays IDLE; missionComplete() now ends the run
        }

        _sm.setDropping(true);
        Serial.printf("[DLE] Module A — drop %d\n", dp.id);
        _phase = Phase::WAITING_A;
    }
}

const char* DropLogicEngine::approachName(Approach a) {
    switch (a) {
        case Approach::NO_POSITION: return "NO POSITION";
        case Approach::DONE:        return "MISSION DONE";
        case Approach::FAR:         return "FAR";
        case Approach::APPROACHING: return "APPROACHING";
        case Approach::SLOW_DOWN:   return "SLOW DOWN";
        case Approach::SETTLING:    return "SETTLING";
        case Approach::IN_RADIUS:   return "IN RADIUS";
        case Approach::DROPPING:    return "DROPPING";
    }
    return "?";
}

const char* DropLogicEngine::hoverName(Hover h) {
    switch (h) {
        case Hover::OFF:       return "OFF";
        case Hover::NO_DATA:   return "NO DATA";
        case Hover::ON_GROUND: return "ON GROUND";
        case Hover::WAITING:   return "WAITING";
        case Hover::READY:     return "READY";
        case Hover::HOLDING:   return "HOLDING";
        case Hover::DROPPING:  return "DROPPING";
        case Hover::DONE:      return "DONE";
    }
    return "?";
}

void DropLogicEngine::_trackFlight(const Position& cur) {
    const HoverConfig& hc = _cfg.config().hover;
    const uint32_t now = millis();

    // Position-derived speed over a >= 1 s window. Any invalid sample starts
    // the window over, so a gap in the fix can never stretch dt and read as
    // a slow aircraft.
    if (!cur.valid) { _ref_ms = 0; _pos_speed_ms = 0; return; }
    if (!_ref_ms) {
        _ref_pos = cur; _ref_ms = now ? now : 1;
    } else if (now - _ref_ms >= 1000) {
        const float dt = (now - _ref_ms) / 1000.0f;
        _pos_speed    = haversineDistance(_ref_pos.lat_deg, _ref_pos.lon_deg,
                                          cur.lat_deg, cur.lon_deg) / dt;
        _pos_speed_ms = now ? now : 1;
        _ref_pos = cur; _ref_ms = now ? now : 1;
    }

    // Takeoff / landing. Height comes only from OSDK; without it the
    // aircraft is never "airborne" and hover mode can never fire.
    if (!cur.height_valid) return;
    const bool low = cur.height_m < hc.takeoff_height_m;
    if (!_takeoff_ms) {
        if (low) {
            _home = cur; _have_home = true;            // keeps following until launch
        } else {
            _takeoff_ms = now ? now : 1;
            if (!_have_home) { _home = cur; _have_home = true; }
            _low_since_ms = 0;
            char ex[96];
            snprintf(ex, sizeof(ex), "\"h\":%.1f,\"home_lat\":%.7f,\"home_lon\":%.7f",
                     cur.height_m, _home.lat_deg, _home.lon_deg);
            _log.logEvent("takeoff", ex);
            Serial.printf("[DLE] Takeoff detected at %.1f m - flight clock started\n",
                          cur.height_m);
        }
    } else if (low) {
        if (!_low_since_ms) _low_since_ms = now ? now : 1;
        else if (now - _low_since_ms >= LANDED_MS) {
            _log.logEvent("landed", "");
            Serial.println("[DLE] Landed - flight clock and home point reset");
            _takeoff_ms = 0; _low_since_ms = 0;
            _home = cur; _have_home = true;
            _have_last_drop = false;
        }
    } else {
        _low_since_ms = 0;
    }
}

void DropLogicEngine::_updateHover(const Position& cur) {
    const HoverConfig& hc = _cfg.config().hover;
    const uint32_t now   = millis();
    const bool     armed = _sm.isArmed();
    const uint8_t  drops = _hover.drops;

    _hover = HoverInfo();
    _hover.drops = drops;

    if (!cur.valid || !cur.height_valid) {
        _hover.state   = Hover::NO_DATA;
        _hold_start_ms = 0;
        return;
    }

    HoverInfo& h = _hover;
    h.airborne     = _takeoff_ms != 0;
    h.flight_s     = h.airborne ? (now - _takeoff_ms) / 1000.0f : 0.0f;
    h.height_m     = cur.height_m;
    h.height_valid = true;
    h.home_dist_m  = _have_home ? haversineDistance(_home.lat_deg, _home.lon_deg,
                                                    cur.lat_deg, cur.lon_deg) : -1.0f;
    h.speed_valid  = cur.vel_valid;
    h.speed_mps    = cur.vel_valid ? cur.horizontalSpeed() : 0.0f;
    const bool pos_fresh = _pos_speed_ms && (now - _pos_speed_ms) <= 2000;
    h.pos_speed    = pos_fresh ? _pos_speed : -1.0f;
    h.rearm_m      = _have_last_drop ? haversineDistance(_last_drop.lat_deg, _last_drop.lon_deg,
                                                         cur.lat_deg, cur.lon_deg) : -1.0f;

    h.ok_time   = h.airborne && h.flight_s >= hc.min_flight_min * 60.0f;
    h.ok_height = h.height_m >= hc.min_height_m;
    h.ok_home   = _have_home && h.home_dist_m >= hc.min_home_dist_m;
    h.ok_speed  = h.speed_valid && h.speed_mps <= hc.max_speed_mps;
    h.ok_agree  = pos_fresh && _pos_speed <= hc.max_speed_mps + POS_SPEED_TOL_MPS;
    h.ok_rearm  = !_have_last_drop || h.rearm_m >= hc.rearm_dist_m;
    const bool all = h.ok_time && h.ok_height && h.ok_home &&
                     h.ok_speed && h.ok_agree && h.ok_rearm;

    // Like the waypoint dwell: the hold runs only ARMED and restarts on any
    // break, so arming mid-hover waits the full hold_s.
    if (!all || !armed) {
        _hold_start_ms = 0;
    } else if (!_hold_start_ms) {
        _hold_start_ms = now ? now : 1;
        Serial.printf("[DLE] HOVER conditions met - holding %.1fs "
                      "(h=%.1f home=%.0f v=%.1f pv=%.1f)\n",
                      hc.hold_s, h.height_m, h.home_dist_m, h.speed_mps, _pos_speed);
    }
    const uint32_t held_ms = _hold_start_ms ? now - _hold_start_ms : 0;
    h.hold_s = held_ms / 1000.0f;

    h.state = !h.airborne ? Hover::ON_GROUND
            : !all        ? Hover::WAITING
            : !armed      ? Hover::READY
            :               Hover::HOLDING;

    if (!armed) return;

    // 1 Hz trace while armed, for the bench and for a tethered ground check.
    static uint32_t last_trace_ms = 0;
    if (now - last_trace_ms >= 1000) {
        last_trace_ms = now;
        Serial.printf("[DLE] HOVER %s t=%.0fs h=%.1f home=%.0f v=%s pv=%s rearm=%s hold=%.1f "
                      "[%c%c%c%c%c%c]\n",
                      hoverName(h.state), h.flight_s, h.height_m, h.home_dist_m,
                      h.speed_valid ? String(h.speed_mps, 1).c_str() : "none",
                      pos_fresh ? String(_pos_speed, 1).c_str() : "none",
                      _have_last_drop ? String(h.rearm_m, 0).c_str() : "-",
                      h.hold_s,
                      h.ok_time ? 'T' : 't', h.ok_height ? 'H' : 'h',
                      h.ok_home ? 'D' : 'd', h.ok_speed ? 'V' : 'v',
                      h.ok_agree ? 'P' : 'p', h.ok_rearm ? 'R' : 'r');
    }

    if (!all || held_ms < (uint32_t)(hc.hold_s * 1000.0f + 0.5f)) return;

    // ── Fire ────────────────────────────────────────────────────────────
    _hover.drops = drops + 1;
    _hover_snap  = _hover;
    Serial.printf("[DLE] >>> HOVER release #%u - %.0fs airborne, h=%.1f m, "
                  "%.0f m from home, %.1f m/s (pos %.1f), held %.1fs\n",
                  _hover.drops, h.flight_s, h.height_m, h.home_dist_m,
                  h.speed_mps, _pos_speed, h.hold_s);
    _hold_start_ms = 0;
    _pending_pos   = cur;
    _manual_pair   = false;
    _hover_drop    = true;

    // Same defect (c) guard as the waypoint path
    if (!_modA.startDrop()) {
        Serial.println("[DLE] *** HOVER release refused - module A has no doors left ***");
        _recordDrop(_hover.drops, 0, _modA.doorsDropped(), cur, DropResult::ALREADY_DONE);
        _hover_drop = false;
        return;
    }
    _last_drop = cur; _have_last_drop = true;
    _hover.state = Hover::DROPPING;
    _sm.setDropping(true);
    Serial.printf("[DLE] Module A - hover drop %u\n", _hover.drops);
    _phase = Phase::WAITING_A;
}

bool DropLogicEngine::launcherEmpty() const {
    return _modA.isEmpty() || _modB.isEmpty();
}

uint8_t DropLogicEngine::_currentDropId() const {
    if (_manual_pair) return 0;                       // 0 = pilot/RC commanded
    if (_hover_drop)  return _hover.drops;            // hover drops count 1..n
    if (_pending_idx >= _cfg.config().drop_count) return 0;
    return _cfg.config().drops[_pending_idx].id;
}

bool DropLogicEngine::startManualPair() {
    if (_phase != Phase::IDLE) return false;
    if (_modA.isDropping() || _modB.isDropping()) return false;
    // Refuse rather than log a phantom success if module A has nothing left
    if (_modA.isEmpty()) {
        Serial.println("[DLE] RC trigger ignored — Module A is empty");
        return false;
    }

    _pending_pos = _pos.read();   // may be invalid (no fix) — logged as-is
    _manual_pair = true;
    _sm.setDropping(true);
    _approach.state = Approach::DROPPING;
    Serial.println("[DLE] *** PILOT RC TRIGGER — Module A ***");
    _modA.startDrop();
    _phase = Phase::WAITING_A;
    return true;
}

void DropLogicEngine::abort() {
    if (_phase == Phase::IDLE) return;        // nothing in flight, idempotent

    // Log the truth about the door that was interrupted. The module's own
    // stop() has set (or is about to set) DropResult::ABORTED; door number is
    // doorsDropped()+1 because the counter only advances on real completion,
    // so the door being attempted is the next one, not the last one finished.
    const bool a = (_phase == Phase::WAITING_A);
    ActuatorModule& m = a ? _modA : _modB;
    _recordDrop(_currentDropId(), a ? 0 : 1,
                m.doorsDropped() + 1, _pending_pos, DropResult::ABORTED);

    Serial.printf("[DLE] *** ABORTED during module %c — sequence torn down, "
                  "waypoint %d still pending ***\n",
                  a ? 'A' : 'B', _next_drop_idx + 1);

    _phase          = Phase::IDLE;
    _manual_pair    = false;
    _hover_drop     = false;
    _dwell_start_ms = 0;
    _hold_start_ms  = 0;
    // _next_drop_idx deliberately untouched — see abort() in the header.
}

void DropLogicEngine::_recordDrop(uint8_t drop_id, uint8_t module_idx,
                                  uint8_t door_num, const Position& pos,
                                  DropResult result) {
    DropEvent evt;
    evt.timestamp_ms = millis();
    evt.drop_id      = drop_id;
    evt.module       = module_idx;
    evt.door_num     = door_num;
    evt.lat_deg      = pos.lat_deg;
    evt.lon_deg      = pos.lon_deg;
    evt.alt_m        = pos.alt_m;
    evt.success      = (result == DropResult::OK);
    // Was a bare OK/TIMEOUT ternary, which would have relabelled an ABORTED
    // drop as TIMEOUT — a different failure with a different cause, and the
    // log is the only record anyone gets after a flight.
    const char* rs = "TIMEOUT";
    if      (result == DropResult::OK)           rs = "OK";
    else if (result == DropResult::ABORTED)      rs = "ABORTED";
    else if (result == DropResult::FAULT)        rs = "FAULT";
    else if (result == DropResult::ALREADY_DONE) rs = "EMPTY";
    strncpy(evt.result_str, rs, sizeof(evt.result_str));
    _log.logDrop(evt);
}

bool DropLogicEngine::missionComplete() const {
    // launcherEmpty() is the defect (c) half: with more waypoints than doors,
    // the run must END when the doors run out rather than keep evaluating
    // waypoints it can never service. Without it the engine would re-trigger
    // at the same waypoint every loop and spam failure rows.
    //
    // Hover mode has no waypoint list to run out of: it is over when the
    // modules it fires are empty — module A alone unless hover.use_b.
    const HoverConfig& hc = _cfg.config().hover;
    if (hc.enabled) return hc.use_b ? launcherEmpty() : _modA.isEmpty();
    return _next_drop_idx >= _cfg.config().drop_count || launcherEmpty();
}
