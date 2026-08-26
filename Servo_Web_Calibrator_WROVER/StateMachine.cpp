#include "StateMachine.h"

StateMachine::StateMachine(uint8_t led_pin, uint8_t arm_btn_pin, uint8_t wait_pin)
    : _led_pin(led_pin), _btn_pin(arm_btn_pin), _wait_pin(wait_pin) {}

void StateMachine::begin() {
    pinMode(_btn_pin, INPUT_PULLUP);
    pinMode(_led_pin, OUTPUT);
    digitalWrite(_led_pin, LOW);
    pinMode(_wait_pin, OUTPUT);
    digitalWrite(_wait_pin, LOW);
    _setState(SystemState::BOOT);
    Serial.printf("[SM] State machine started — BOOT (state LED GPIO%d, "
                  "wait LED GPIO%d)\n", _led_pin, _wait_pin);
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

    _updateLED();
    _updateWaitLED();
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

void StateMachine::emergencyStop() {
    // FAULT is deliberately excluded: it is unrecoverable by design, and an
    // emergency stop must never launder a faulted system back into a state
    // that can be armed.
    if (_state == SystemState::FAULT) return;
    _setState(SystemState::IDLE);
    Serial.println("[SM] *** EMERGENCY STOP -> IDLE (disarmed) ***");
}

void StateMachine::completeBoot() {
    if (_state == SystemState::BOOT) {
        _setState(SystemState::IDLE);
        Serial.println("[SM] Boot complete → IDLE");
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
}

void StateMachine::_updateLED() {
    // No WS2812 on this board — states are distinguished by blink cadence
    // instead of colour. Non-blocking (millis()-based), safe to call every
    // loop() iteration alongside the drop-logic engine.
    uint32_t t = millis();
    bool on;
    switch (_state) {
        case SystemState::BOOT:     on = true;               break; // solid
        case SystemState::IDLE:     on = (t % 2000) < 150;    break; // slow single blink
        case SystemState::ARMED:    on = (t % 500)  < 250;    break; // steady 2Hz blink
        case SystemState::DROPPING: on = (t % 100)  < 50;     break; // rapid blink
        case SystemState::FAULT:    on = (t % 1000) < 500;    break; // slow 50% on/off
        default:                    on = false;
    }
    digitalWrite(_led_pin, on ? HIGH : LOW);
}

void StateMachine::_updateWaitLED() {
    // Yellow "WAIT" semaphore — see Pins.h. Deliberately the inverse of the
    // announce LED during DROPPING: yellow steady = holding, yellow dark +
    // announce lit = firing. Makes the A->B handoff readable at a glance,
    // and makes an unexpected re-fire after an emergency stop obvious.
    uint32_t t = millis();
    bool on;
    switch (_state) {
        case SystemState::BOOT:     on = false;             break; // dark until IDLE
        case SystemState::IDLE:     on = (t % 1500) < 120;  break; // slow blink — powered, not armed
        case SystemState::ARMED:    on = true;              break; // STEADY — waiting for trigger
        case SystemState::DROPPING: on = false;             break; // announce LED has the floor
        case SystemState::FAULT:    on = (t % 200)  < 100;  break; // rapid
        default:                    on = false;
    }
    digitalWrite(_wait_pin, on ? HIGH : LOW);
}
