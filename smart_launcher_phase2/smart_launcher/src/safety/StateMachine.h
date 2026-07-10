#pragma once
#include <Arduino.h>
#include <FastLED.h>

enum class SystemState {
    BOOT,     // startup, self-check
    IDLE,     // running but drop logic disabled
    ARMED,    // drop logic active — ready to fire
    DROPPING, // currently executing a drop
    FAULT     // unrecoverable error — servos stopped
};

// RGB LED colours per state
static const CRGB STATE_COLOUR[] = {
    CRGB::Blue,    // BOOT
    CRGB::Yellow,  // IDLE
    CRGB::Green,   // ARMED
    CRGB::Purple,  // DROPPING
    CRGB::Red      // FAULT
};

class StateMachine {
public:
    StateMachine(uint8_t led_pin, uint8_t arm_btn_pin);

    void begin();

    // Call every loop — handles button debounce and LED update
    void update();

    // External modules call these to request transitions
    void requestArm();
    void requestDisarm();
    void setDropping(bool active);
    void setFault(const char* reason);

    SystemState state() const { return _state; }
    bool isArmed()    const { return _state == SystemState::ARMED; }
    bool isDropping() const { return _state == SystemState::DROPPING; }
    bool isFault()    const { return _state == SystemState::FAULT; }

private:
    uint8_t     _led_pin;
    uint8_t     _btn_pin;
    SystemState _state        = SystemState::BOOT;
    uint32_t    _btn_press_t  = 0;
    bool        _btn_held     = false;
    CRGB        _leds[1];

    void _setState(SystemState s);
    void _updateLED();

    // Long press threshold for ARM/DISARM
    static const uint32_t ARM_HOLD_MS = 2000;
};
