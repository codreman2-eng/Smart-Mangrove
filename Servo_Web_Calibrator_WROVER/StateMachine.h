#pragma once
#include <Arduino.h>

enum class SystemState {
    BOOT,     // startup, self-check
    IDLE,     // running but drop logic disabled
    ARMED,    // drop logic active — ready to fire
    DROPPING, // currently executing a drop
    FAULT     // unrecoverable error — servos stopped
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

    // One-time BOOT -> IDLE transition, called once at the end of setup().
    // Deliberately separate from requestDisarm(), which is guarded to only
    // fire from ARMED (its real job is the normal disarm path) and would
    // silently no-op if called from BOOT — which is exactly the bug this
    // method fixes (the system could never leave BOOT before this existed).
    void completeBoot();

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

    void _setState(SystemState s);

    // Plain 2-pin LED — states are distinguished by blink pattern, not
    // colour (no WS2812 on this board). Called every update().
    void _updateLED();

    // Long press threshold for ARM/DISARM
    static const uint32_t ARM_HOLD_MS = 2000;
};
