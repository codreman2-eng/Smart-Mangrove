/**
 * Smart Launcher — Simplest Possible Servo Test
 * Board : Freenove ESP32-S3 WROOM
 * Core  : ESP32 Arduino core v3.x
 *
 * Does ONE thing: drives the servo FORWARD continuously.
 * No Wi-Fi. No web UI. No limit switch. Just forward.
 *
 * If the servo spins → wiring + code are good.
 * If not → it's hardware (power or ground).
 *
 * Wiring (minimum to test):
 *   Servo signal (orange/white) → GPIO 18
 *   Servo power  (red)          → external 5V (UBEC or bench supply)
 *   Servo ground (brown/black)  → external supply GND
 *                                  AND ESP32 GND  (COMMON GROUND — required)
 */

#include <Arduino.h>

#define SERVO_PIN   18      // change to 7 if GPIO 18 doesn't work
#define LED_PIN      2      // built-in LED — shows the sketch is running

#define LEDC_FREQ   50      // 50 Hz for servos
#define LEDC_RES    16      // 16-bit resolution

// Convert microseconds to LEDC duty value
void writeServo(int pulse_us) {
    uint32_t duty = (uint32_t)pulse_us * 65536UL / 20000UL;
    ledcWrite(SERVO_PIN, duty);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SIMPLE SERVO TEST ===");
    Serial.println("Servo should spin FORWARD continuously");

    pinMode(LED_PIN, OUTPUT);

    // Attach servo pin (v3.x API)
    ledcAttach(SERVO_PIN, LEDC_FREQ, LEDC_RES);
    Serial.printf("[OK] Servo attached to GPIO %d\n", SERVO_PIN);

    // Drive servo FORWARD immediately
    writeServo(1700);     // 1700us = forward for a continuous rotation servo
    Serial.println("[OK] Sent 1700us — servo should now be spinning forward");
}

void loop() {
    // Keep sending forward command + blink LED so you know it's alive
    writeServo(1700);

    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);

    Serial.println("[RUN] Servo forward @ 1700us — LED blinking");
}
