/**
 * Smart Launcher — Servo Test, no library needed
 * Board : ESP32 WROVER-E
 * Core  : v3.x
 *
 * Cycles: full forward 3s → stop 2s → full reverse 3s → stop 2s
 * Watch serial monitor to confirm each step.
 *
 * Wiring:
 *   Servo signal → GPIO 13
 *   Servo power  → external 5V
 *   Servo GND    → external GND + ESP32 GND (both!)
 */

#include <Arduino.h>

#define SERVO_PIN  13
#define LED_PIN     2

#define SERVO_FREQ  50
#define SERVO_BITS  16

void servoWrite(int pulse_us) {
    uint32_t duty = (uint32_t)pulse_us * 65535UL / 20000UL;
    ledcWrite(SERVO_PIN, duty);
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SERVO TEST — no library ===");
    pinMode(LED_PIN, OUTPUT);
    ledcAttach(SERVO_PIN, SERVO_FREQ, SERVO_BITS);
    Serial.printf("[OK] Attached GPIO %d at 50Hz 16-bit\n", SERVO_PIN);
    servoWrite(1500);
    Serial.println("[OK] Starting in 1 second...");
    delay(1000);
}

void loop() {
    Serial.println("[1] FORWARD full speed (2000us) — 3s");
    servoWrite(2000);
    digitalWrite(LED_PIN, HIGH);
    delay(3000);

    Serial.println("[2] STOP (1500us) — 2s");
    servoWrite(1500);
    digitalWrite(LED_PIN, LOW);
    delay(2000);

    Serial.println("[3] REVERSE full speed (1000us) — 3s");
    servoWrite(1000);
    digitalWrite(LED_PIN, HIGH);
    delay(3000);

    Serial.println("[4] STOP (1500us) — 2s");
    servoWrite(1500);
    digitalWrite(LED_PIN, LOW);
    delay(2000);
}
