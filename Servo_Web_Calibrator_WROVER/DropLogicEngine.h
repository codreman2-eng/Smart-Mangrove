#pragma once
#include <Arduino.h>
#include "ConfigManager.h"
#include "PositionSource.h"
#include "ActuatorModule.h"
#include "StateMachine.h"
#include "Logger.h"

// GPS-radius-triggered sequential A-then-B drop orchestration.
//
// NON-BLOCKING: tracks its own phase (IDLE/WAITING_A/WAITING_B) instead of
// blocking on ActuatorModule::dropNext() the way the original PlatformIO
// version did. Requires the caller's loop() to call modA.poll() and
// modB.poll() BEFORE calling this->update() every iteration — this class
// reacts to isDropping() transitioning to false, which only happens inside
// poll(). See main .ino's loop() for the required call order.
//
// Manual single-module drops (dashboard DROP A/DROP B buttons) do NOT go
// through this class — they call ActuatorModule::startDrop() directly from
// the HTTP handler, same as the original serial 'dropa'/'dropb' commands
// did. This class only owns the automatic GPS-triggered sequence.
class DropLogicEngine {
public:
    DropLogicEngine(ConfigManager&   cfg,
                    IPositionSource& pos,
                    ActuatorModule&  modA,
                    ActuatorModule&  modB,
                    StateMachine&    sm,
                    Logger&          log);

    // Call every loop iteration, AFTER modA.poll()/modB.poll().
    void update();

    // True when all drops are complete
    bool missionComplete() const;

    // True while an automatic A-then-B sequence is in progress — used to
    // guard manual drop buttons from firing mid-sequence.
    bool isBusy() const { return _phase != Phase::IDLE; }

    // Index of the next pending drop (0-based) — for dashboard progress display.
    uint8_t nextDropIndex() const { return _next_drop_idx; }

private:
    enum class Phase { IDLE, WAITING_A, WAITING_B };

    ConfigManager&   _cfg;
    IPositionSource& _pos;
    ActuatorModule&  _modA;
    ActuatorModule&  _modB;
    StateMachine&    _sm;
    Logger&          _log;

    uint8_t  _next_drop_idx = 0;   // index into drops[] array
    Phase    _phase         = Phase::IDLE;
    uint8_t  _pending_idx   = 0;   // drops[] index currently in flight
    Position _pending_pos;         // position captured at trigger time, for logging

    void _recordDrop(uint8_t drop_id, uint8_t module_idx,
                     uint8_t door_num, const Position& pos,
                     DropResult result);
};
