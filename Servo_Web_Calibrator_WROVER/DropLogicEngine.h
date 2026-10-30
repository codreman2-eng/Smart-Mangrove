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
    // ── Release gate: in radius AND slowed down AND settled ─────────────
    // Being inside the waypoint radius is not enough on its own: the seed
    // must also leave a slow aircraft, so the pilot flies in, slows, and the
    // release happens once horizontal speed is at or below
    // release.max_speed_mps (mission.json, editable from the dashboard; it was
    // a compile-time 2.0 until 2026-10-15). Only enforced when the position
    // source has velocity health (OSDK); on the GNSS fallback there is no
    // speed to test and the gate stays open, which is the pre-gate behaviour.
    //
    // release.dwell_s then asks for that condition to hold, ARMED and
    // continuously, before the drop starts — so a hover drop waits for the
    // aircraft to settle instead of firing on the first qualifying sample.
    // The count runs only while armed: arming inside the radius waits the
    // full dwell rather than firing at once on time spent disarmed.

    // Distance from the next waypoint at which the dashboard starts reporting
    // APPROACHING. Never smaller than a few radii, so a large radius still
    // gets warning before the aircraft is inside it.
    static constexpr float APPROACH_ZONE_M = 20.0f;

    // What the aircraft is doing relative to the next waypoint — for the
    // dashboard. Computed armed or not; only an ARMED engine ever fires.
    enum class Approach : uint8_t {
        NO_POSITION,   // no valid fix
        DONE,          // mission complete / out of doors
        FAR,           // outside the approach zone
        APPROACHING,   // inside the approach zone, not yet in radius
        SLOW_DOWN,     // in radius but faster than release.max_speed_mps
        SETTLING,      // in radius, slow, armed — counting down release.dwell_s
        IN_RADIUS,     // in radius and slow enough — fires if armed
        DROPPING       // A->B sequence running
    };
    struct ApproachInfo {
        Approach state       = Approach::NO_POSITION;
        uint8_t  wp_id       = 0;
        float    dist_m      = 0.0f;    // predicted impact (or aircraft) to waypoint
        float    radius_m    = 0.0f;
        float    speed_mps   = 0.0f;
        bool     speed_valid = false;   // false -> speed gate not enforced
        bool     predicted   = false;   // dist is from the impact predictor
        float    dwell_s     = 0.0f;    // in-radius-and-slow time so far, armed
    };
    const ApproachInfo& approach() const { return _approach; }
    static const char* approachName(Approach a);

    // ── HOVER release mode (HoverConfig in ConfigManager.h) ─────────────
    // How far the position-derived speed may exceed max_speed_mps and still
    // count as agreeing with OSDK velocity. Differencing 1 s of A3 fused
    // position is noisier than the A3's own velocity; this tolerance only
    // has to catch a velocity field that is plainly wrong (bad decode, wrong
    // units, stuck at 0), not a few tenths of jitter.
    static constexpr float POS_SPEED_TOL_MPS = 1.0f;
    // Below takeoff height this long = landed: the flight clock and home
    // point reset, so a second sortie counts its own minutes from its own
    // launch site.
    static constexpr uint32_t LANDED_MS = 10000;

    enum class Hover : uint8_t {
        OFF,          // hover mode disabled
        NO_DATA,      // no position, or no OSDK height — cannot fire, ever
        ON_GROUND,    // never reached takeoff height (or landed)
        WAITING,      // airborne, at least one condition not met
        READY,        // every condition met but DISARMED — would fire if armed
        HOLDING,      // every condition met, armed, counting hold_s
        DROPPING,     // sequence running
        DONE          // out of doors
    };
    struct HoverInfo {
        Hover    state        = Hover::OFF;
        bool     airborne     = false;
        float    flight_s     = 0.0f;   // since takeoff
        float    height_m     = 0.0f;
        bool     height_valid = false;
        float    home_dist_m  = -1.0f;  // -1 = no home point yet
        float    speed_mps    = 0.0f;   // OSDK
        bool     speed_valid  = false;
        float    vspeed_mps   = 0.0f;   // |vertical|, OSDK; valid with speed_valid
        float    pos_speed    = -1.0f;  // position-derived; -1 = not fresh
        float    rearm_m      = -1.0f;  // from last hover drop; -1 = none yet
        float    hold_s       = 0.0f;   // all-conditions time so far, armed
        uint8_t  drops        = 0;      // hover drops this boot
        // One bit per condition, for the dashboard checklist
        bool ok_time = false, ok_height = false, ok_home = false,
             ok_speed = false, ok_agree = false, ok_rearm = false,
             ok_vert = false;
    };
    const HoverInfo& hover() const { return _hover; }
    static const char* hoverName(Hover h);

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

    // Emergency stop: tear down any in-flight sequence.
    //
    // Defect (a), confirmed on hardware 2026-08-25: handleStop() used to stop
    // both modules without touching _phase. ActuatorModule::stop() clears
    // _active, so on the VERY NEXT tick the WAITING_A branch below read
    // !isDropping() as "A finished", and started module B — a real door
    // opening immediately after the operator hit emergency stop. Observed: B
    // began in the same loop iteration as the stop and ran a complete 350ms
    // drive window.
    //
    // Deliberately does NOT advance _next_drop_idx: an aborted waypoint was
    // not serviced, and must still be pending when the operator re-arms.
    void abort();

    // True when no further drop is possible — either every waypoint has been
    // serviced, or the launcher is physically out of doors.
    //
    // The second half is part of the defect (c) fix. A mission with more
    // waypoints than doors is not "still running" once the cover is spent;
    // it is over, and the honest thing is to say so and disarm rather than
    // keep evaluating waypoints that can never be serviced.
    bool missionComplete() const;

    // True once either module has spent all its doors.
    bool launcherEmpty() const;

    // True while an automatic A-then-B sequence is in progress — used to
    // guard manual drop buttons from firing mid-sequence.
    bool isBusy() const { return _phase != Phase::IDLE; }

    // Index of the next pending drop (0-based) — for dashboard progress display.
    uint8_t nextDropIndex() const { return _next_drop_idx; }

    // Rewind mission progress to waypoint 0. MUST be called whenever the
    // waypoint list is edited: the index is a position into drops[], so
    // editing the list underneath it would leave the engine pointing at a
    // different waypoint than the operator thinks, or past the end.
    void resetProgress() { _next_drop_idx = 0; _dwell_start_ms = 0; _hold_start_ms = 0; }

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
    ApproachInfo _approach;          // dashboard view of the next waypoint

    // millis() when the current in-radius-and-slow run began, armed; 0 = not
    // running. Cleared on any exit from the window, on disarm, on firing, on
    // abort and on resetProgress().
    uint32_t _dwell_start_ms = 0;

    // What the gate saw at the moment it fired, for the "release" log row.
    // Written to the log when module A finishes rather than at trigger time:
    // at trigger time the logger is not yet in its drop window, so the row
    // would sync to flash inside module A's drive window and overshoot it.
    ApproachInfo _release_snap;
    float        _release_max_speed = 0.0f;

    // ── Hover mode state ───────────────────────────────────────────────
    HoverInfo _hover;
    HoverInfo _hover_snap;            // the gate's view at the trigger, for the log
    bool      _hover_drop     = false; // in-flight sequence came from hover mode
    uint32_t  _hold_start_ms  = 0;     // 0 = not running
    uint32_t  _takeoff_ms     = 0;     // 0 = on the ground
    uint32_t  _low_since_ms   = 0;     // below takeoff height since (airborne only)
    Position  _home;                   // last on-ground position before takeoff
    bool      _have_home      = false;
    Position  _last_drop;              // where the last hover drop fired
    bool      _have_last_drop = false;
    Position  _ref_pos;                // start of the position-speed window
    uint32_t  _ref_ms         = 0;
    float     _pos_speed      = 0.0f;
    uint32_t  _pos_speed_ms   = 0;     // when _pos_speed was computed; 0 = never

    // Takeoff / landing / home / position-derived speed. Runs every idle
    // tick in either mode so the numbers are there the moment hover mode is
    // switched on.
    void _trackFlight(const Position& cur);
    void _updateHover(const Position& cur);

    // drop id to log for the in-flight sequence (0 = pilot/RC commanded)
    uint8_t _currentDropId() const;

    void _recordDrop(uint8_t drop_id, uint8_t module_idx,
                     uint8_t door_num, const Position& pos,
                     DropResult result);
};
