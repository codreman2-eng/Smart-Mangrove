/**
 * Smart Launcher — Simple Servo Test (ESP32Servo library)
 * Board : Freenove ESP32-S3 WROOM
 * Core  : ESP32 Arduino core v3.x
 *
 * Uses the ESP32Servo library instead of raw ledcWrite.
 * This is the most reliable way to drive servos on ESP32-S3.
 *
 * INSTALL THE LIBRARY FIRST:
 *   Arduino IDE → Tools → Manage Libraries
 *   Search: "ESP32Servo" by Kevin Harrington / John K. Bennett
 *   Click Install
 *
 * Wiring:
 *   Servo signal (white/orange) → GPIO 18
 *   Servo power  (red)          → external 5V supply
 *   Servo ground (black/brown)  → external GND + ESP32 GND (common!)
 */

#include <ESP32Servo.h>

Servo myServo;

#define SERVO_PIN  18
#define LED_PIN     2

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== SERVO TEST (ESP32Servo lib) ===");

    pinMode(LED_PIN, OUTPUT);

    // Allocate timers for the library (required on ESP32)
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Standard servo pulse range, 50 Hz
    myServo.setPeriodHertz(50);
    myServo.attach(SERVO_PIN, 500, 2500);   // min/max pulse in microseconds
    Serial.printf("[OK] Servo attached to GPIO %d\n", SERVO_PIN);

    // Drive forward for a continuous rotation servo
    // For CR servos: writeMicroseconds controls speed/direction
    myServo.writeMicroseconds(1700);
    Serial.println("[OK] Sent 1700us — servo should spin forward");
}

void loop() {
    // Keep commanding forward + blink so we know it's alive
    myServo.writeMicroseconds(1700);

    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);

    Serial.println("[RUN] forward 1700us");
}
