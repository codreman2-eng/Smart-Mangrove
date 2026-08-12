#pragma once
#include <Arduino.h>

// RC PWM input — pilot "drop now" trigger from the DJI M600's A3 Pro
// F-channel (mapped to an RC channel in DJI Assistant 2).
//
// Signal: standard RC PWM — a 1000-2000us pulse repeating at ~50Hz (20ms).
// This SUPPLEMENTS the GPS waypoint auto-fire; it does not replace it.
//
// ── Why an interrupt and not pulseIn() ──────────────────────────────────
// pulseIn() blocks for up to its timeout. This firmware's loop() also runs
// server.handleClient() and the drop-actuator polling; a blocking read
// would freeze the dashboard and can trip the watchdog — the exact defect
// class already hit on this project (see CRITICAL LESSONS LEARNED #6).
// An edge ISR + micros() is non-blocking, ~1-2us resolution, and its
// failure modes are trivial to reason about.
//
// ── Safety rules, all enforced in update() ─────────────────────────────
//   * Range gate      — widths outside [800, 2200]us are counted as
//                       glitches and never reach the band logic.
//   * Debounce        — a band must be seen on N consecutive valid frames
//                       before it is accepted (default 5 ~= 100ms).
//   * Signal loss     — no valid frame for signal_timeout_ms means NO
//                       SIGNAL. Loss can never trigger a drop.
//   * Fail-safe sense — the 1000us end is SAFE. Both "no signal" and
//                       "idle stick" mean do nothing.
//   * Edge-triggered  — fires once on ENTRY into the FIRE band. Holding
//                       the switch does not dump every door; the switch
//                       must return below fire_min and come back.
//   * Cold-start safe — a NO-SIGNAL -> FIRE transition never fires. If the
//                       aircraft powers up with the switch already at FIRE,
//                       the pilot must cycle it back to SAFE first.
//   * Arming          — this class only reports a fire request. It cannot
//                       arm. The caller must still require ARMED state, so
//                       two independent authorities are needed to drop.
class RcPwmInput {
public:
    enum class Band : uint8_t { NONE = 0, SAFE = 1, MID = 2, FIRE = 3 };

    // pin: must be interrupt-capable. GPIO 35 recommended — input-only, so
    // no firmware bug can ever back-drive the flight controller. Input-only
    // pins have NO internal pull-up/pull-down: fit an external 10k pull-down
    // so a disconnected line reads as steady low rather than floating.
    explicit RcPwmInput(uint8_t pin);

    void begin();

    // Call every loop() iteration. Does the atomic snapshot of the ISR
    // state, the range gate, the debounce and the timeout.
    void update();

    // Consumes the one-shot fire request. Returns true at most once per
    // entry into the FIRE band; the caller still owns the ARMED check.
    bool consumeFireEdge();

    bool     hasSignal()   const { return _has_signal; }
    Band     band()        const { return _band; }
    uint16_t widthUs()     const { return _width_us; }      // last accepted
    uint16_t minSeenUs()   const { return _min_seen; }      // calibration aid
    uint16_t maxSeenUs()   const { return _max_seen; }      // calibration aid
    uint32_t frameCount()  const { return _frames_ok; }
    uint32_t glitchCount() const { return _glitches; }
    uint16_t frameRateHz() const { return _rate_hz; }

    void resetStats();

    // Thresholds. safe_max < fire_min, both inside [1000, 2000].
    void setBands(uint16_t safe_max_us, uint16_t fire_min_us,
                  uint8_t frames_to_confirm, uint16_t signal_timeout_ms);
    void setEnabled(bool en) { _enabled = en; }
    bool enabled() const     { return _enabled; }

    uint16_t safeMaxUs()  const { return _safe_max; }
    uint16_t fireMinUs()  const { return _fire_min; }
    uint8_t  framesReq()  const { return _frames_req; }
    uint16_t timeoutMs()  const { return _timeout_ms; }

    static const char* bandName(Band b);

    // Absolute sanity limits — a width outside these never counts as a frame
    static const uint16_t WIDTH_MIN_US = 800;
    static const uint16_t WIDTH_MAX_US = 2200;

private:
    uint8_t  _pin;
    bool     _enabled    = true;

    uint16_t _safe_max   = 1300;
    uint16_t _fire_min   = 1700;
    uint8_t  _frames_req = 5;
    uint16_t _timeout_ms = 500;

    bool     _has_signal = false;
    Band     _band       = Band::NONE;
    Band     _candidate  = Band::NONE;
    uint8_t  _cand_count = 0;
    bool     _fire_edge  = false;

    uint16_t _width_us   = 0;
    uint16_t _min_seen   = 0xFFFF;
    uint16_t _max_seen   = 0;
    uint32_t _frames_ok  = 0;
    uint32_t _glitches   = 0;
    uint32_t _last_ok_ms = 0;

    // frame-rate estimate
    uint32_t _rate_window_ms = 0;
    uint32_t _rate_base      = 0;
    uint16_t _rate_hz        = 0;

    // Snapshot of the last raw frame consumed from the ISR
    uint32_t _last_seq = 0;

    Band _classify(uint16_t us) const;
};
