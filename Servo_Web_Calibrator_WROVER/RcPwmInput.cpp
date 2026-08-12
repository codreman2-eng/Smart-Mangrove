#include "RcPwmInput.h"

// ─── ISR state ─────────────────────────────────────────────────────────
// One RC input on this hardware, so a file-static ISR context is simpler
// and cheaper than an instance-pointer table. If a second channel is ever
// added, promote these to an array indexed by channel.
//
// _seq increments on every accepted frame; update() compares it against
// its own copy to tell "new frame" from "same frame read twice", which is
// what makes signal-loss detection reliable at any loop rate.
static portMUX_TYPE s_rcMux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t s_rise_us = 0;
static volatile uint32_t s_width   = 0;
static volatile uint32_t s_seq     = 0;
static volatile uint32_t s_glitch  = 0;
static uint8_t s_pin = 255;

static void IRAM_ATTR rcEdgeIsr() {
    uint32_t now = micros();
    if (digitalRead(s_pin)) {
        s_rise_us = now;                 // rising edge — start of pulse
        return;
    }
    // Falling edge — pulse width. Wide sanity window here; the strict
    // range gate lives in update() so glitches stay countable/visible.
    uint32_t w = now - s_rise_us;
    // Separate read/write rather than ++ — increment on a volatile is
    // deprecated in C++20 and warns under -Wvolatile.
    if (w >= 400 && w <= 3000) {
        s_width = w;
        s_seq   = s_seq + 1;
    } else {
        s_glitch = s_glitch + 1;
    }
}

// ─── Construction ──────────────────────────────────────────────────────
RcPwmInput::RcPwmInput(uint8_t pin) : _pin(pin) {}

void RcPwmInput::begin() {
    s_pin = _pin;
    // GPIO 34-39 are input-only and have NO internal pull resistors — the
    // external 10k pull-down is what makes a disconnected line read low.
    // INPUT (not INPUT_PULLUP) is correct here for that reason.
    pinMode(_pin, INPUT);
    attachInterrupt(digitalPinToInterrupt(_pin), rcEdgeIsr, CHANGE);
    _last_ok_ms    = millis();
    _rate_window_ms = millis();
    Serial.printf("[RC] PWM input on GPIO%d — bands: SAFE<=%d, FIRE>=%d, "
                  "confirm=%d frames, timeout=%dms\n",
                  _pin, _safe_max, _fire_min, _frames_req, _timeout_ms);
}

const char* RcPwmInput::bandName(Band b) {
    switch (b) {
        case Band::SAFE: return "SAFE";
        case Band::MID:  return "MID";
        case Band::FIRE: return "FIRE";
        default:         return "NONE";
    }
}

RcPwmInput::Band RcPwmInput::_classify(uint16_t us) const {
    if (us <= _safe_max) return Band::SAFE;
    if (us >= _fire_min) return Band::FIRE;
    return Band::MID;
}

void RcPwmInput::setBands(uint16_t safe_max_us, uint16_t fire_min_us,
                          uint8_t frames_to_confirm, uint16_t signal_timeout_ms) {
    safe_max_us = constrain(safe_max_us, 1000, 2000);
    fire_min_us = constrain(fire_min_us, 1000, 2000);
    // Refuse to build overlapping/inverted bands — an inverted pair would
    // classify every width as FIRE, which is the one failure direction that
    // must never be reachable from a config value.
    if (fire_min_us <= safe_max_us + 50) fire_min_us = safe_max_us + 50;
    if (fire_min_us > 2000) { fire_min_us = 2000; safe_max_us = 1950; }

    _safe_max   = safe_max_us;
    _fire_min   = fire_min_us;
    _frames_req = constrain((int)frames_to_confirm, 1, 50);
    _timeout_ms = constrain((int)signal_timeout_ms, 100, 5000);

    // Any threshold change invalidates the debounce in progress
    _candidate  = Band::NONE;
    _cand_count = 0;
}

void RcPwmInput::resetStats() {
    _min_seen  = 0xFFFF;
    _max_seen  = 0;
    _frames_ok = 0;
    _glitches  = 0;
    portENTER_CRITICAL(&s_rcMux);
    s_glitch = 0;
    portEXIT_CRITICAL(&s_rcMux);
}

// ─── Per-loop update ───────────────────────────────────────────────────
void RcPwmInput::update() {
    if (!_enabled) { _has_signal = false; _band = Band::NONE; return; }

    uint32_t seq, width, glitch;
    portENTER_CRITICAL(&s_rcMux);
    seq    = s_seq;
    width  = s_width;
    glitch = s_glitch;
    portEXIT_CRITICAL(&s_rcMux);

    uint32_t now = millis();

    if (seq != _last_seq) {
        _last_seq = seq;
        _glitches = glitch;

        if (width >= WIDTH_MIN_US && width <= WIDTH_MAX_US) {
            _width_us   = (uint16_t)width;
            _last_ok_ms = now;
            _frames_ok++;
            if (_width_us < _min_seen) _min_seen = _width_us;
            if (_width_us > _max_seen) _max_seen = _width_us;

            // Debounce: N consecutive frames in the same band to accept it
            Band b = _classify(_width_us);
            if (b == _candidate) {
                if (_cand_count < 255) _cand_count++;
            } else {
                _candidate  = b;
                _cand_count = 1;
            }

            if (_cand_count >= _frames_req && b != _band) {
                Band prev = _band;
                _band       = b;
                _has_signal = true;
                Serial.printf("[RC] %s -> %s (%dus)\n",
                              bandName(prev), bandName(b), _width_us);

                // Edge-trigger. Deliberately NOT fired on NONE -> FIRE:
                // powering up with the switch already at FIRE must not
                // drop. The pilot has to cycle back below fire_min first.
                if (b == Band::FIRE && (prev == Band::SAFE || prev == Band::MID)) {
                    _fire_edge = true;
                    Serial.println("[RC] *** FIRE edge ***");
                }
            } else if (_cand_count >= _frames_req) {
                _has_signal = true;
            }
        } else {
            _glitches++;   // in-ISR-window but outside the strict RC range
        }
    }

    // Signal loss — never a trigger, always a revocation.
    if (_has_signal && (now - _last_ok_ms) > _timeout_ms) {
        Serial.printf("[RC] SIGNAL LOST (no valid frame for %lums)\n",
                      (unsigned long)(now - _last_ok_ms));
        _has_signal = false;
        _band       = Band::NONE;
        _candidate  = Band::NONE;
        _cand_count = 0;
        _fire_edge  = false;   // discard any unconsumed request
    }

    // Frame-rate estimate over a 1s window (a healthy A3 channel is ~50Hz)
    if (now - _rate_window_ms >= 1000) {
        _rate_hz        = (uint16_t)(_frames_ok - _rate_base);
        _rate_base      = _frames_ok;
        _rate_window_ms = now;
    }
}

bool RcPwmInput::consumeFireEdge() {
    if (!_fire_edge) return false;
    _fire_edge = false;
    return true;
}
