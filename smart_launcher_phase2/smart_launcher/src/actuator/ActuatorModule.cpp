#include "ActuatorModule.h"

ActuatorModule::ActuatorModule(uint8_t servo_pin, uint8_t limit_pin,
                               uint8_t ledc_channel,
                               uint16_t open_pwm_us, uint16_t stop_pwm_us,
                               uint32_t travel_ms, uint8_t door_count)
    : _servo_pin(servo_pin), _limit_pin(limit_pin),
      _ledc_channel(ledc_channel),
      _open_pwm_us(open_pwm_us), _stop_pwm_us(stop_pwm_us),
      _travel_ms(travel_ms), _door_count(door_count) {}

void ActuatorModule::begin() {
    // Configure limit switch pin with internal pull-up
    pinMode(_limit_pin, INPUT_PULLUP);

    // Configure LEDC for 50 Hz servo PWM
    ledcSetup(_ledc_channel, LEDC_FREQ_HZ, LEDC_RESOLUTION);
    ledcAttachPin(_servo_pin, _ledc_channel);

    // Ensure servo starts stopped
    _writePWM(_stop_pwm_us);

    Serial.printf("[ACT] Module init: servo GPIO%d limit GPIO%d ch%d\n",
                  _servo_pin, _limit_pin, _ledc_channel);
}

DropResult ActuatorModule::dropNext() {
    if (isEmpty()) {
        Serial.println("[ACT] Module empty — no more doors");
        return DropResult::ALREADY_DONE;
    }

    uint8_t door_num = _doors_dropped + 1;
    Serial.printf("[ACT] Dropping door %d/%d ...\n", door_num, _door_count);

    // --- Start servo ---
    _writePWM(_open_pwm_us);

    // --- Wait for limit switch OR timeout ---
    // Limit switch is on the LAST door only.
    // For doors 1-4: we rely purely on travel_ms timing.
    // For door 5 (last): timing + limit switch confirmation.
    bool is_last_door = (door_num == _door_count);
    uint32_t timeout  = _travel_ms * TIMEOUT_MULTIPLIER;
    uint32_t start    = millis();
    bool     sw_hit   = false;

    while (millis() - start < (is_last_door ? timeout : _travel_ms)) {
        if (is_last_door && digitalRead(_limit_pin) == LOW) {
            sw_hit = true;
            break;
        }
        delay(1);   // yield
    }

    // --- Stop servo ---
    _writePWM(_stop_pwm_us);

    // --- Evaluate result ---
    DropResult result;
    if (is_last_door) {
        result = sw_hit ? DropResult::OK : DropResult::TIMEOUT;
        Serial.printf("[ACT] Door %d — limit switch: %s\n",
                      door_num, sw_hit ? "OK" : "TIMEOUT");
    } else {
        result = DropResult::OK;   // timed drop — assume OK
        Serial.printf("[ACT] Door %d — timed drop OK (%d ms)\n",
                      door_num, _travel_ms);
    }

    _doors_dropped++;
    return result;
}

void ActuatorModule::stop() {
    _writePWM(_stop_pwm_us);
    Serial.println("[ACT] STOP");
}

void ActuatorModule::reset() {
    _doors_dropped = 0;
    Serial.println("[ACT] Reset — doors_dropped = 0");
}

void ActuatorModule::_writePWM(uint16_t pulse_us) {
    // Convert microseconds to 16-bit LEDC duty
    // Period = 1/50Hz = 20000 us
    // duty = (pulse_us / 20000) * 2^16
    uint32_t duty = (uint32_t)pulse_us * 65536UL / 20000UL;
    ledcWrite(_ledc_channel, duty);
}
