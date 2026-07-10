#include "DropLogicEngine.h"

DropLogicEngine::DropLogicEngine(ConfigManager&   cfg,
                                 IPositionSource& pos,
                                 ActuatorModule&  modA,
                                 ActuatorModule&  modB,
                                 StateMachine&    sm,
                                 Logger&          log)
    : _cfg(cfg), _pos(pos), _modA(modA), _modB(modB), _sm(sm), _log(log) {}

void DropLogicEngine::update() {
    // Only run when ARMED
    if (!_sm.isArmed()) return;

    // All drops done
    if (missionComplete()) return;

    // Get current position
    _pos.update();
    Position cur = _pos.read();
    if (!cur.valid) return;

    // Check the next pending drop point
    const MissionConfig& mc  = _cfg.config();
    const DropPoint&     dp  = mc.drops[_next_drop_idx];

    float dist = haversineDistance(cur.lat_deg, cur.lon_deg,
                                   dp.lat_deg,  dp.lon_deg);

    Serial.printf("[DLE] WP%d dist=%.1fm (r=%.1fm)\n",
                  dp.id, dist, dp.radius_m);

    if (dist <= dp.radius_m) {
        Serial.printf("[DLE] >>> In radius for WP%d — executing drop\n", dp.id);
        _executeDrop(_next_drop_idx);
        _next_drop_idx++;
    }
}

void DropLogicEngine::_executeDrop(uint8_t drop_idx) {
    const DropPoint& dp  = _cfg.config().drops[drop_idx];
    Position         cur = _pos.read();

    _sm.setDropping(true);

    // --- Module A fires first ---
    Serial.printf("[DLE] Module A — drop %d\n", dp.id);
    DropResult resA = _modA.dropNext();
    _recordDrop(dp.id, 0, _modA.doorsDropped(), cur, resA);

    if (resA == DropResult::FAULT) {
        _sm.setFault("Module A fault during drop");
        return;
    }

    // --- Module B fires immediately after A confirms ---
    // Sequential confirm: A's dropNext() is blocking so we're already confirmed
    Serial.printf("[DLE] Module B — drop %d\n", dp.id);
    DropResult resB = _modB.dropNext();
    _recordDrop(dp.id, 1, _modB.doorsDropped(), cur, resB);

    if (resB == DropResult::FAULT) {
        _sm.setFault("Module B fault during drop");
        return;
    }

    _sm.setDropping(false);
    Serial.printf("[DLE] Drop %d complete — A:%s B:%s\n",
                  dp.id,
                  resA == DropResult::OK ? "OK" : "TIMEOUT",
                  resB == DropResult::OK ? "OK" : "TIMEOUT");
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
