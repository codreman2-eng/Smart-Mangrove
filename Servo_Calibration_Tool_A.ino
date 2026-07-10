/*
  Servo Calibration / Identification Tool — MODULE A
  Board: ESP32-WROVER (Freenove)

  PURPOSE:
    Figures out two things about an unknown MG996R-style servo:
      (a) Is it CONTINUOUS ROTATION or POSITIONAL (180°)?
      (b) What are its real min/max pulse widths (they often differ
          from the textbook 1000-2000us)?

  WIRING:
    Servo Signal (orange/yellow) -> GPIO 12   (Module A)
    Servo VCC (red)              -> external 5V supply (NOT ESP32 5V/3V3 pin)
    Servo GND (brown/black)      -> common GND with ESP32

  NOTE ON GPIO 12:
    GPIO12 is a strapping pin on the ESP32 (affects flash voltage selection
    at boot). It only matters if something pulls it HIGH during the boot
    moment; a servo signal wire alone won't normally interfere, but if you
    ever see boot/flash weirdness, move the signal wire to a different pin
    (e.g. 13, 14, 27) and try again.

  LIBRARY REQUIRED:
    "ESP32Servo" (Kevin Harrington / John K. Bennett) - install via
    Arduino IDE -> Tools -> Manage Libraries

  HOW TO USE (Serial Monitor, 115200 baud, line ending = "Newline"):
    1. Send "c" to center the servo at 1500us. Note the shaft position.
    2. Send "+" a few times (or "-") and watch what happens:
         - If it keeps spinning continuously in one direction and speeds
           up/slows down as you move away from 1500 -> CONTINUOUS ROTATION servo.
         - If it moves to a position and then STOPS (holds there, doesn't
           keep spinning) -> POSITIONAL (180°) servo.
    3. Use "a" to auto-sweep slowly from 500us to 2500us and back so you
       can watch/listen for grinding or stalling at the extremes. Send any
       character to stop the sweep immediately if you hear straining.
    4. Use direct numbers (e.g. "800", "2200") to jump straight to a pulse
       width and check behavior at that point.
    5. For a POSITIONAL servo: the min/max pulses where it just stops
       moving further (without straining/buzzing) are your real limits.
       For a CONTINUOUS servo: note the pulse where it visibly stops
       spinning (should be near 1500, but note the exact value - that's
       your true "stop" point, since cheap servos often aren't perfectly
       centered).

  RESULT LOG (Module A):
    True stop measured at 1650us (2026 calibration session).
    Next: sweep around 1650 to pick real open/close offsets before
    redoing travel_ms timing — the old 1700/1300 presets assumed a
    1500us center and are no longer valid.

  SAFETY:
    Never hold a pulse width at an extreme for long if you hear the motor
    straining/buzzing without moving - that's a stall, and it damages
    positional servos. Back off immediately.
*/

#include <ESP32Servo.h>

const int SERVO_PIN = 12;   // Module A

int currentPulse = 1500;   // start centered
int stepSize = 25;         // us per +/- command

Servo myServo;

void setup() {
  Serial.begin(115200);
  delay(500);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myServo.setPeriodHertz(50);
  myServo.attach(SERVO_PIN, 500, 2500); // wide range for testing extremes

  myServo.writeMicroseconds(currentPulse);

  Serial.println("=== Servo Calibration Tool — MODULE A (GPIO 12) ===");
  printHelp();
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("h")) {
      printHelp();
    }
    else if (cmd.equalsIgnoreCase("c")) {
      setPulse(1500);
    }
    else if (cmd == "+" || cmd.equalsIgnoreCase("u")) {
      setPulse(currentPulse + stepSize);
    }
    else if (cmd == "-" || cmd.equalsIgnoreCase("d")) {
      setPulse(currentPulse - stepSize);
    }
    else if (cmd.equalsIgnoreCase("m")) {
      setPulse(500);
      Serial.println("At MIN (500us) - listen for straining!");
    }
    else if (cmd.equalsIgnoreCase("x")) {
      setPulse(2500);
      Serial.println("At MAX (2500us) - listen for straining!");
    }
    else if (cmd.equalsIgnoreCase("a")) {
      autoSweep();
    }
    else if (cmd.startsWith("step ")) {
      int newStep = cmd.substring(5).toInt();
      if (newStep > 0) {
        stepSize = newStep;
        Serial.print("Step size set to: ");
        Serial.println(stepSize);
      }
    }
    else {
      int val = cmd.toInt();
      if (val == 0 && cmd != "0") {
        Serial.println("Unknown command. Type 'h' for help.");
      } else {
        setPulse(val);
      }
    }
  }
}

void setPulse(int us) {
  us = constrain(us, 500, 2500);
  currentPulse = us;
  myServo.writeMicroseconds(currentPulse);
  printStatus();
}

void printStatus() {
  Serial.print("[A] Pulse: ");
  Serial.print(currentPulse);
  Serial.println("us");
}

void autoSweep() {
  Serial.println("Auto-sweeping 500us -> 2500us -> 500us. Send any key to abort.");
  bool aborted = false;

  for (int us = 500; us <= 2500 && !aborted; us += 20) {
    setPulse(us);
    delay(60);
    if (Serial.available() > 0) {
      Serial.readStringUntil('\n'); // consume it
      aborted = true;
    }
  }
  for (int us = 2500; us >= 500 && !aborted; us -= 20) {
    setPulse(us);
    delay(60);
    if (Serial.available() > 0) {
      Serial.readStringUntil('\n');
      aborted = true;
    }
  }

  if (aborted) {
    Serial.println("Sweep aborted by user.");
  } else {
    Serial.println("Sweep complete.");
  }
  setPulse(1500); // return to center/stop for safety
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  c        - center (1500us)");
  Serial.println("  + / u    - increase pulse by step (default 25us)");
  Serial.println("  - / d    - decrease pulse by step");
  Serial.println("  m        - jump to MIN (500us)");
  Serial.println("  x        - jump to MAX (2500us)");
  Serial.println("  a        - auto-sweep 500-2500-500us (send any key to abort)");
  Serial.println("  step N   - set +/- step size to N us (e.g. 'step 50')");
  Serial.println("  <number> - jump directly to that pulse width in us");
  Serial.println("  h        - show this help");
}
