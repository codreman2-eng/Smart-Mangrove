#pragma once
#include <Arduino.h>

// Result of a single door drop attempt
enum class DropResult {
    OK,               // limit switch confirmed
    TIMEOUT,          // limit switch never triggered — logged as FAIL
    ALREADY_DONE,     // all doors already spent
    FAULT             // hardware error
};

// One physical launcher module (A or B)
// Drives one continuous rotation servo, reads one limit switch
class ActuatorModule {
public:
    // servo_pin    : GPIO for servo PWM signal
    // limit_pin    : GPIO for limit switch (INPUT_PULLUP, active LOW)
    // ledc_channel : LEDC channel (0 for A, 1 for B)
    // open_pwm_us  : PWM to run servo (retracts cover)
    // stop_pwm_us  : PWM to stop servo
    // travel_ms    : time to travel exactly one door width (35mm)
    // door_count   : total doors this module has
    ActuatorModule(uint8_t servo_pin, uint8_t limit_pin,
                   uint8_t ledc_channel,
                   uint16_t open_pwm_us, uint16_t stop_pwm_us,
                   uint32_t travel_ms, uint8_t door_count);

    // Call once at startup
    void begin();

    // Execute one door drop — BLOCKING until limit switch or timeout
    // Returns result; updates internal door counter
    DropResult dropNext();

    // True if all doors have been dropped
    bool isEmpty() const { return _doors_dropped >= _door_count; }

    // Current door index (0-based, 0 = first drop)
    uint8_t doorsDropped() const { return _doors_dropped; }

    // Stop servo immediately (call in FAULT or emergency)
    void stop();

    // Reset door counter (for bench testing / module swap)
    void reset();

private:
    uint8_t  _servo_pin;
    uint8_t  _limit_pin;
    uint8_t  _ledc_channel;
    uint16_t _open_pwm_us;
    uint16_t _stop_pwm_us;
    uint32_t _travel_ms;
    uint8_t  _door_count;
    uint8_t  _doors_dropped = 0;

    // Write PWM to servo via LEDC
    void _writePWM(uint16_t pulse_us);
    static const uint32_t LEDC_FREQ_HZ   = 50;
    static const uint8_t  LEDC_RESOLUTION = 16;   // 16-bit for fine PWM control
    // Timeout = travel_ms * 3 (generous — limit switch should fire sooner)
    static const uint8_t  TIMEOUT_MULTIPLIER = 3;
};
