#pragma once
#include <Arduino.h>

// Result of a single door drop attempt
enum class DropResult {
    OK,               // limit switch confirmed (or timed drop assumed OK)
    TIMEOUT,          // limit switch never triggered — logged as FAIL
    ALREADY_DONE,     // all doors already spent
    FAULT             // hardware error
};

// One physical launcher module (A or B)
// Drives one continuous rotation servo, reads one limit switch.
//
// NON-BLOCKING BY DESIGN: unlike the original PlatformIO version of this
// class (which blocked in a while(millis()...){delay(1);} loop for the
// whole travel_ms), this version uses startDrop()+poll() — same shape as
// the calibrator's already-proven TIMED RUN (timedActive/timedStart/
// timedMs). This matters because this module's loop() also runs
// server.handleClient() for the live dashboard: a blocking sequential
// A-then-B drop would freeze the dashboard for 3+ seconds during exactly
// the moment you'd most want to watch it live — the same class of problem
// this project already learned the hard way ("blocking web handlers cause
// watchdog resets").
//
// PWM writes use ledcAttach(pin,freq,res)+ledcWrite(pin,duty) — the same
// API validated on this exact hardware via the calibrator all session, NOT
// the classic channel-based API PlatformIO's older bundled core forced.
class ActuatorModule {
public:
    // servo_pin    : GPIO for servo PWM signal
    // limit_pin    : GPIO for limit switch (INPUT_PULLUP, active LOW)
    // open_pwm_us  : PWM to run servo (retracts cover)
    // stop_pwm_us  : PWM to stop servo
    // travel_ms    : time to travel exactly one door width (35mm)
    // door_count   : total doors this module has
    // id           : 'A' or 'B' — only used to pick the SIM confirm-flash
    //                pattern so the two modules are distinguishable on the
    //                single shared announce LED. No effect in flight builds.
    ActuatorModule(uint8_t servo_pin, uint8_t limit_pin,
                   uint16_t open_pwm_us, uint16_t stop_pwm_us,
                   uint32_t travel_ms, uint8_t door_count,
                   char id = 'A');

    // Call once at startup
    void begin();

    // Non-blocking: starts the next door drop, returns immediately.
    // Returns false if already empty or a drop is already in progress.
    // Call poll() every loop() iteration to advance/complete it.
    bool startDrop();

    // Call every loop() iteration. Returns true on the exact iteration a
    // drop completes (check lastResult() then). No-op if nothing active.
    bool poll();

    bool isDropping() const { return _active; }
    DropResult lastResult() const { return _lastResult; }

    // ── Drive-window measurement ──────────────────────────────────────
    // How long the servo was ACTUALLY driven on the last completed drop,
    // and how far that ran past travel_ms. The drive window is closed by
    // poll(), which only runs when loop() gets a turn, so any loop stall
    // lands here as overshoot on a continuous-rotation servo — i.e. as
    // extra millimetres of cover travel. This is the direct measurement of
    // the thing worst-loop was only ever a proxy for, and it is what the
    // validation plan's "580ms +/-10%" criterion should be judged on.
    //
    // Meaningful only for TIMED doors. On the last door the limit switch
    // is allowed to stop the drop early, so overshoot is reported as 0.
    uint32_t lastDriveMs()     const { return _last_drive_ms; }
    int32_t  lastOvershootMs() const { return _last_overshoot_ms; }

    // True if all doors have been dropped
    bool isEmpty() const { return _doors_dropped >= _door_count; }

    // Current door index (0-based, 0 = first drop)
    uint8_t doorsDropped() const { return _doors_dropped; }

    // Stop servo immediately — safe in any state, aborts an active drop.
    void stop();

    // Reset door counter (for bench testing / module swap)
    void reset();

    // Raw PWM write, bypassing the door-drop state machine — used by the
    // calibration UI's OPEN/STOP/CLOSE buttons and slider.
    void writeRaw(uint16_t pulse_us);

    // Update calibration values in RAM (used after a "Save Calibration").
    void setCalibration(uint16_t open_pwm_us, uint16_t stop_pwm_us, uint32_t travel_ms);

    uint16_t openPwm() const  { return _open_pwm_us; }
    uint16_t stopPwm() const  { return _stop_pwm_us; }
    uint32_t travelMs() const { return _travel_ms; }
    uint8_t  doorCount() const { return _door_count; }

private:
    uint8_t  _servo_pin;
    uint8_t  _limit_pin;
    uint16_t _open_pwm_us;
    uint16_t _stop_pwm_us;
    uint32_t _travel_ms;
    uint8_t  _door_count;
    char     _id;
    uint8_t  _doors_dropped = 0;

    bool       _active       = false;
    uint32_t   _start_ms     = 0;
    bool       _is_last_door = false;
    uint32_t   _timeout_ms   = 0;
    DropResult _lastResult   = DropResult::OK;
    uint32_t   _last_drive_ms     = 0;
    int32_t    _last_overshoot_ms = 0;

    // SIM-only post-drop confirmation blips. Non-blocking by the same rule
    // as everything else in this class — a blocking flash routine would
    // reintroduce exactly the watchdog/dashboard-freeze defect class this
    // firmware was rewritten to avoid.
    bool     _flashActive    = false;
    bool     _flashOn        = false;
    uint8_t  _flashRemaining = 0;
    uint32_t _flashNext      = 0;
    void _startConfirmFlash(uint8_t count);
    void _updateFlash();

    void _writePWM(uint16_t pulse_us);
    // Timeout = travel_ms * 3 (generous — limit switch should fire sooner)
    static const uint8_t TIMEOUT_MULTIPLIER = 3;
};
