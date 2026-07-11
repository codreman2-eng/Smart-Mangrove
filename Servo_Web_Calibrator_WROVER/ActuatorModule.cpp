#include "ActuatorModule.h"
#include "Pins.h"

ActuatorModule::ActuatorModule(uint8_t servo_pin, uint8_t limit_pin,
                               uint16_t open_pwm_us, uint16_t stop_pwm_us,
                               uint32_t travel_ms, uint8_t door_count)
    : _servo_pin(servo_pin), _limit_pin(limit_pin),
      _open_pwm_us(open_pwm_us), _stop_pwm_us(stop_pwm_us),
      _travel_ms(travel_ms), _door_count(door_count) {}

void ActuatorModule::begin() {
    pinMode(_limit_pin, INPUT_PULLUP);

    // v3.x LEDC API — validated on this exact board all session
    ledcAttach(_servo_pin, SERVO_FREQ_HZ, SERVO_RES_BITS);

    // Ensure servo starts stopped
    writeRaw(_stop_pwm_us);

    Serial.printf("[ACT] Module init: servo GPIO%d limit GPIO%d\n",
                  _servo_pin, _limit_pin);
}

bool ActuatorModule::startDrop() {
    if (isEmpty() || _active) return false;

    uint8_t door_num = _doors_dropped + 1;
    Serial.printf("[ACT] Dropping door %d/%d ...\n", door_num, _door_count);

    // Limit switch is on the LAST door only — doors 1..N-1 rely purely on
    // travel_ms timing, door N also confirms via the limit switch.
    _is_last_door = (door_num == _door_count);
    _timeout_ms   = _is_last_door ? (_travel_ms * TIMEOUT_MULTIPLIER) : _travel_ms;
    _start_ms     = millis();
    _active       = true;

    writeRaw(_open_pwm_us);
    return true;
}

bool ActuatorModule::poll() {
    if (!_active) return false;

    bool sw_hit    = _is_last_door && (digitalRead(_limit_pin) == LOW);
    bool timed_out = (millis() - _start_ms) >= _timeout_ms;

    if (!sw_hit && !timed_out) return false;   // still running, nothing to report yet

    // --- Stop servo ---
    writeRaw(_stop_pwm_us);
    _active = false;

    uint8_t door_num = _doors_dropped + 1;
    if (_is_last_door) {
        _lastResult = sw_hit ? DropResult::OK : DropResult::TIMEOUT;
        Serial.printf("[ACT] Door %d — limit switch: %s\n",
                      door_num, sw_hit ? "OK" : "TIMEOUT");
    } else {
        _lastResult = DropResult::OK;   // timed drop — assume OK
        Serial.printf("[ACT] Door %d — timed drop OK (%lu ms)\n",
                      door_num, (unsigned long)_travel_ms);
    }

    _doors_dropped++;
    return true;
}

void ActuatorModule::stop() {
    writeRaw(_stop_pwm_us);
    _active = false;
    Serial.println("[ACT] STOP");
}

void ActuatorModule::reset() {
    _doors_dropped = 0;
    Serial.println("[ACT] Reset — doors_dropped = 0");
}

void ActuatorModule::writeRaw(uint16_t pulse_us) {
    _writePWM(pulse_us);
}

void ActuatorModule::setCalibration(uint16_t open_pwm_us, uint16_t stop_pwm_us,
                                    uint32_t travel_ms) {
    _open_pwm_us = open_pwm_us;
    _stop_pwm_us = stop_pwm_us;
    _travel_ms   = travel_ms;
}

void ActuatorModule::_writePWM(uint16_t pulse_us) {
    // Same math as the calibrator's proven servoWrite(): duty = pulse/20000 * 2^16
    uint32_t duty = (uint32_t)constrain(pulse_us, 1000, 2000) * 65535UL / 20000UL;
    ledcWrite(_servo_pin, duty);
}
