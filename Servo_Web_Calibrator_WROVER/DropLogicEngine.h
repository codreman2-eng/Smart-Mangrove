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

    // Start a balanced A-then-B pair on demand — used by the pilot's RC
    // trigger, which SUPPLEMENTS the GPS waypoint path rather than
    // replacing it. Reuses the same proven WAITING_A/WAITING_B sequencing
    // as an automatic drop (rather than duplicating it in the .ino), so
    // module B still fires only after A reports done, preserving balance.
    //
    // Deliberately does NOT advance the waypoint index and logs with
    // drop_id 0 — a pilot-commanded drop must not silently consume a
    // planned GPS waypoint. Caller still owns the ARMED check.
    // Returns false if a sequence is already in progress or A is empty.
    bool startManualPair();

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
    bool     _manual_pair   = false; // in-flight sequence came from the RC trigger

    // drop id to log for the in-flight sequence (0 = pilot/RC commanded)
    uint8_t _currentDropId() const;

    void _recordDrop(uint8_t drop_id, uint8_t module_idx,
                     uint8_t door_num, const Position& pos,
                     DropResult result);
};
