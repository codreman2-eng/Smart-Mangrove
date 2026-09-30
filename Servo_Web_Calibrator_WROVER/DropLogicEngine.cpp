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
        _recordDrop(drop_id, 0, _modA.doorsDropped(), _pending_pos, resA);

        if (resA == DropResult::FAULT) {
            _sm.setFault("Module A fault during drop");
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
            if (!_manual_pair) _next_drop_idx++;
            _manual_pair = false;
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

        // A pilot-commanded pair must not consume a planned GPS waypoint
        if (!_manual_pair) _next_drop_idx++;
        _manual_pair = false;
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
    if (missionComplete()) { _approach = ApproachInfo(); _approach.state = Approach::DONE; return; }

    _pos.update();
    Position cur = _pos.read();
    if (!cur.valid) { _approach = ApproachInfo(); return; }

    const MissionConfig& mc = _cfg.config();
    const DropPoint&     dp = mc.drops[_next_drop_idx];

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
    const bool  too_fast = cur.vel_valid && speed > MAX_RELEASE_SPEED_MPS;
    const float zone     = fmaxf(APPROACH_ZONE_M, 4.0f * dp.radius_m);

    _approach.wp_id       = dp.id;
    _approach.dist_m      = dist;
    _approach.radius_m    = dp.radius_m;
    _approach.speed_mps   = cur.vel_valid ? speed : 0.0f;
    _approach.speed_valid = cur.vel_valid;
    _approach.predicted   = predicted;
    _approach.state = in_rad       ? (too_fast ? Approach::SLOW_DOWN : Approach::IN_RADIUS)
                    : dist <= zone ? Approach::APPROACHING
                    :                Approach::FAR;

    if (!_sm.isArmed()) return;

    // Rate-limited to 1Hz. This ran on EVERY loop iteration while armed —
    // ~30 characters at 115200 baud is ~2.6ms of blocking, thousands of
    // times per second, and Serial.printf() stalls once the TX buffer
    // fills. The distance readout was itself the largest source of drop
    // latency, and it only appeared while ARMED, which is precisely when
    // the lag was observed. Also emitted when the distance changes by more
    // than a metre, so an approach is still visible in real time.
    static uint32_t last_dist_ms  = 0;
    static float    last_dist_m   = -1.0f;
    uint32_t now_ms = millis();
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
                          dp.id, speed, MAX_RELEASE_SPEED_MPS);
        }
        return;
    }

    if (in_rad) {
        Serial.printf("[DLE] >>> In radius for WP%d at %s — executing drop\n", dp.id,
                      cur.vel_valid ? (String(speed, 1) + " m/s").c_str()
                                    : "unknown speed (no velocity, gate off)");
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
        case Approach::IN_RADIUS:   return "IN RADIUS";
        case Approach::DROPPING:    return "DROPPING";
    }
    return "?";
}

bool DropLogicEngine::launcherEmpty() const {
    return _modA.isEmpty() || _modB.isEmpty();
}

uint8_t DropLogicEngine::_currentDropId() const {
    if (_manual_pair) return 0;                       // 0 = pilot/RC commanded
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

    _phase       = Phase::IDLE;
    _manual_pair = false;
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
    return _next_drop_idx >= _cfg.config().drop_count || launcherEmpty();
}
