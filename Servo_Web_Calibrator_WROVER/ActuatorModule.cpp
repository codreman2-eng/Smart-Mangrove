#include "ActuatorModule.h"
#include "Pins.h"

ActuatorModule::ActuatorModule(uint8_t servo_pin, uint8_t limit_pin,
                               uint16_t open_pwm_us, uint16_t stop_pwm_us,
                               uint32_t travel_ms, uint8_t door_count,
                               char id)
    : _servo_pin(servo_pin), _limit_pin(limit_pin),
      _open_pwm_us(open_pwm_us), _stop_pwm_us(stop_pwm_us),
      _travel_ms(travel_ms), _door_count(door_count), _id(id) {}

void ActuatorModule::begin() {
    pinMode(_limit_pin, INPUT_PULLUP);

    // Announce LED exists in both builds — see PIN_DROP_LED in Pins.h
    pinMode(PIN_DROP_LED, OUTPUT);
    digitalWrite(PIN_DROP_LED, LOW);

#if SIM_MODE
    // Deliberately NO ledcAttach() on the servo pin: with nothing attached
    // the LEDC peripheral never drives GPIO 12/13 at all, so the servo
    // signal lines stay high-impedance. A hard guarantee, not "PWM we
    // believe equals the stop value".
    Serial.printf("[ACT] Module %c init: *** SIM *** announce LED GPIO%d, "
                  "servo GPIO%d NOT attached, limit GPIO%d\n",
                  _id, PIN_DROP_LED, _servo_pin, _limit_pin);
#else
    // v3.x LEDC API — validated on this exact board all session
    ledcAttach(_servo_pin, SERVO_FREQ_HZ, SERVO_RES_BITS);
    Serial.printf("[ACT] Module %c init: servo GPIO%d, announce LED GPIO%d, "
                  "limit GPIO%d\n",
                  _id, _servo_pin, PIN_DROP_LED, _limit_pin);
#endif

    // Ensure servo starts stopped (SIM: ensures the LED starts dark)
    writeRaw(_stop_pwm_us);
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
    // Confirm blips outlive the drop itself, so this must run BEFORE the
    // _active guard below.
    _updateFlash();

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

    // SIM: blip out which door just dropped, so the event is readable by a
    // human without stretching the drive window it just measured.
    _startConfirmFlash(door_num);
    return true;
}

void ActuatorModule::stop() {
    writeRaw(_stop_pwm_us);
    _active      = false;
    _flashActive = false;   // e-stop must leave the announce LED dark
    Serial.printf("[ACT] Module %c STOP\n", _id);
}

void ActuatorModule::reset() {
    _doors_dropped = 0;
    _flashActive   = false;
    Serial.printf("[ACT] Module %c reset — doors_dropped = 0\n", _id);
}

void ActuatorModule::_startConfirmFlash(uint8_t count) {
    _flashRemaining = count;
    _flashOn        = false;
    _flashActive    = (count > 0);
    _flashNext      = millis() + DROP_FLASH_LEAD_MS;
}

void ActuatorModule::_updateFlash() {
    if (!_flashActive) return;
    uint32_t now = millis();
    if ((int32_t)(now - _flashNext) < 0) return;

    if (_flashOn) {
        digitalWrite(PIN_DROP_LED, LOW);
        _flashOn = false;
        if (--_flashRemaining == 0) { _flashActive = false; return; }
        _flashNext = now + (_id == 'A' ? DROP_FLASH_A_OFF_MS : DROP_FLASH_B_OFF_MS);
    } else {
        digitalWrite(PIN_DROP_LED, HIGH);
        _flashOn   = true;
        _flashNext = now + (_id == 'A' ? DROP_FLASH_A_ON_MS : DROP_FLASH_B_ON_MS);
    }
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
    // Announce LED, both builds: lit for exactly the window the servo is (or
    // would be) driven. No stretching, no speed multiplier — the bench test
    // must not distort the timing it exists to verify, and the flight
    // indicator must mean the same thing the bench one did.
    bool driving = (pulse_us != _stop_pwm_us);
    if (driving) _flashActive = false;   // a new drive window pre-empts a pending confirm
    digitalWrite(PIN_DROP_LED, driving ? HIGH : LOW);

#if !SIM_MODE
    // ── THE ONLY DIVERGENCE ── SIM omits this and nothing else.
    // Same math as the calibrator's proven servoWrite(): duty = pulse/20000 * 2^16
    uint32_t duty = (uint32_t)constrain(pulse_us, 1000, 2000) * 65535UL / 20000UL;
    ledcWrite(_servo_pin, duty);
#endif
}
