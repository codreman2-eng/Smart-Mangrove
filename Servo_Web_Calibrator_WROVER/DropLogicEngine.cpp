#include "DropLogicEngine.h"

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

        // Module B fires immediately after A confirms
        Serial.printf("[DLE] Module B — drop %d\n", drop_id);
        _modB.startDrop();
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

    // --- IDLE: only evaluate new triggers when armed ---
    if (!_sm.isArmed()) return;
    if (missionComplete()) return;

    _pos.update();
    Position cur = _pos.read();
    if (!cur.valid) return;

    const MissionConfig& mc = _cfg.config();
    const DropPoint&     dp = mc.drops[_next_drop_idx];

    float dist = haversineDistance(cur.lat_deg, cur.lon_deg,
                                   dp.lat_deg,  dp.lon_deg);

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
        Serial.printf("[DLE] WP%d dist=%.1fm (r=%.1fm)\n", dp.id, dist, dp.radius_m);
    }

    if (dist <= dp.radius_m) {
        Serial.printf("[DLE] >>> In radius for WP%d — executing drop\n", dp.id);
        _pending_idx = _next_drop_idx;
        _pending_pos = cur;
        _manual_pair = false;
        _sm.setDropping(true);
        Serial.printf("[DLE] Module A — drop %d\n", dp.id);
        _modA.startDrop();
        _phase = Phase::WAITING_A;
    }
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
    Serial.println("[DLE] *** PILOT RC TRIGGER — Module A ***");
    _modA.startDrop();
    _phase = Phase::WAITING_A;
    return true;
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
    strncpy(evt.result_str,
            result == DropResult::OK ? "OK" : "TIMEOUT",
            sizeof(evt.result_str));
    _log.logDrop(evt);
}

bool DropLogicEngine::missionComplete() const {
    return _next_drop_idx >= _cfg.config().drop_count;
}
