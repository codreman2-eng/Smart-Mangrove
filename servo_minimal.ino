/**
 * Smart Launcher — Minimal Serial Servo Control
 * Board: ESP32 Wrover Module
 *
 * Absolute minimum. No WiFi, no LED, no limit switches yet.
 * Just servo control via serial to prove the base works.
 *
 * Commands (type in Serial Monitor @ 115200, newline ending):
 *   a+   A forward    a-  A reverse    a0  A stop
 *   b+   B forward    b-  B reverse    b0  B stop
 *   t NNN   run A forward NNN ms then stop
 *
 * Wiring:
 *   Servo A signal -> GPIO 12
 *   Servo B signal -> GPIO 13
 *   Servo power    -> external 5V
 *   Servo GND      -> external GND + ESP32 GND (common)
 */

#include <Arduino.h>

#define SERVO_A 12
#define SERVO_B 13

// Timed run state
bool     timed = false;
uint32_t timedStart = 0;
uint32_t timedDur = 0;

void writeUS(int pin, int us) {
  us = constrain(us, 1000, 2000);
  uint32_t duty = (uint32_t)us * 65535UL / 20000UL;
  ledcWrite(pin, duty);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println();
  Serial.println("=== MINIMAL SERVO SERIAL ===");

  ledcAttach(SERVO_A, 50, 16);
  ledcAttach(SERVO_B, 50, 16);
  writeUS(SERVO_A, 1500);
  writeUS(SERVO_B, 1500);

  Serial.println("Ready. Commands: a+ a- a0 b+ b- b0 | t NNN");
  Serial.println("Both servos stopped at 1500us.");
}

void loop() {
  // stop timed run
  if (timed && millis() - timedStart >= timedDur) {
    writeUS(SERVO_A, 1500);
    timed = false;
    Serial.print("Timed run done: ");
    Serial.print(timedDur);
    Serial.println(" ms. MEASURE RACK.");
  }

  if (Serial.available()) {
    String c = Serial.readStringUntil('\n');
    c.trim();

    if (c == "a+")      { writeUS(SERVO_A, 1700); Serial.println("A forward 1700"); }
    else if (c == "a-") { writeUS(SERVO_A, 1300); Serial.println("A reverse 1300"); }
    else if (c == "a0") { writeUS(SERVO_A, 1500); Serial.println("A stop 1500"); }
    else if (c == "b+") { writeUS(SERVO_B, 1700); Serial.println("B forward 1700"); }
    else if (c == "b-") { writeUS(SERVO_B, 1300); Serial.println("B reverse 1300"); }
    else if (c == "b0") { writeUS(SERVO_B, 1500); Serial.println("B stop 1500"); }
    else if (c.startsWith("t ")) {
      uint32_t ms = c.substring(2).toInt();
      if (ms >= 50 && ms <= 9000) {
        writeUS(SERVO_A, 1700);
        timedStart = millis();
        timedDur = ms;
        timed = true;
        Serial.print("A running forward for ");
        Serial.print(ms);
        Serial.println(" ms...");
      } else {
        Serial.println("ms must be 50-9000");
      }
    }
    else if (c.length() > 0) {
      Serial.print("Unknown: ");
      Serial.println(c);
    }
  }
}
