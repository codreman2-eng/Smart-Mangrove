#include "StateMachine.h"

StateMachine::StateMachine(uint8_t led_pin, uint8_t arm_btn_pin)
    : _led_pin(led_pin), _btn_pin(arm_btn_pin) {}

void StateMachine::begin() {
    pinMode(_btn_pin, INPUT_PULLUP);
    FastLED.addLeds<WS2812, 48, GRB>(_leds, 1).setCorrection(TypicalLEDStrip);
    FastLED.setBrightness(40);
    _setState(SystemState::BOOT);
    Serial.println("[SM] State machine started — BOOT");
}

void StateMachine::update() {
    // --- Button handling (long press = ARM/DISARM toggle) ---
    bool btn_low = (digitalRead(_btn_pin) == LOW);

    if (btn_low && !_btn_held) {
        _btn_held    = true;
        _btn_press_t = millis();
    }
    if (!btn_low && _btn_held) {
        _btn_held = false;
        uint32_t held = millis() - _btn_press_t;
        if (held >= ARM_HOLD_MS) {
            if (_state == SystemState::IDLE)  requestArm();
            else if (_state == SystemState::ARMED) requestDisarm();
        }
    }

    // --- Blink LED in ARMED state so it's visible ---
    if (_state == SystemState::ARMED) {
        uint32_t t = millis();
        _leds[0] = (t % 1000 < 500) ? CRGB::Green : CRGB::Black;
        FastLED.show();
    }
}

void StateMachine::requestArm() {
    if (_state == SystemState::IDLE) {
        _setState(SystemState::ARMED);
        Serial.println("[SM] >>> ARMED <<<");
    }
}

void StateMachine::requestDisarm() {
    if (_state == SystemState::ARMED) {
        _setState(SystemState::IDLE);
        Serial.println("[SM] Disarmed → IDLE");
    }
}

void StateMachine::setDropping(bool active) {
    if (active && _state == SystemState::ARMED) {
        _setState(SystemState::DROPPING);
    } else if (!active && _state == SystemState::DROPPING) {
        _setState(SystemState::ARMED);
    }
}

void StateMachine::setFault(const char* reason) {
    Serial.printf("[SM] FAULT: %s\n", reason);
    _setState(SystemState::FAULT);
}

void StateMachine::_setState(SystemState s) {
    _state = s;
    _updateLED();
}

void StateMachine::_updateLED() {
    uint8_t idx = (uint8_t)_state;
    _leds[0] = STATE_COLOUR[idx];
    FastLED.show();
}
