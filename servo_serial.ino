/**
 * Smart Launcher — Serial Servo Controller
 * Board  : ESP32 Wrover Module
 * No web UI. No LED. No WiFi. Just serial commands.
 *
 * Open Serial Monitor at 115200 baud, type commands:
 *
 *   a+       Module A forward (open)
 *   a-       Module A reverse (close)
 *   a0       Module A stop
 *   b+       Module B forward (open)
 *   b-       Module B reverse (close)
 *   b0       Module B stop
 *   ta NNN   Timed run Module A for NNN milliseconds then stop
 *   tb NNN   Timed run Module B for NNN milliseconds then stop
 *   s        Status (current PWM + limit switch states)
 *   help     Show all commands
 */

#include <Arduino.h>

// ── Pins ──────────────────────────────────────────────────────────
#define SERVO_A_PIN   12
#define SERVO_B_PIN   13
#define LIMIT_A_PIN    9
#define LIMIT_B_PIN   10
// NO LED PIN — GPIO 2 is strapping pin, causes crash

// ── LEDC ──────────────────────────────────────────────────────────
#define SERVO_FREQ    50
#define SERVO_BITS    16

// ── State ─────────────────────────────────────────────────────────
int      pwmA = 1500, pwmB = 1500;
bool     runA = false, runB = false;
bool     timedActive = false;
uint8_t  timedPin    = SERVO_A_PIN;
bool*    timedFlag   = nullptr;
uint32_t timedStart  = 0;
uint32_t timedMs     = 0;

// ── Servo write ───────────────────────────────────────────────────
void servoWrite(uint8_t pin, int us) {
    us = constrain(us, 1000, 2000);
    uint32_t duty = (uint32_t)us * 65535UL / 20000UL;
    ledcWrite(pin, duty);
}

void stopAll() {
    servoWrite(SERVO_A_PIN, 1500);
    servoWrite(SERVO_B_PIN, 1500);
    pwmA = 1500; pwmB = 1500;
    runA = false; runB = false;
    timedActive = false;
    Serial.println("[STOP] Both servos stopped");
}

void printStatus() {
    bool lA = (digitalRead(LIMIT_A_PIN) == LOW);
    bool lB = (digitalRead(LIMIT_B_PIN) == LOW);
    Serial.println("--- STATUS ---");
    Serial.printf("  Servo A: %d us  running=%d\n", pwmA, runA);
    Serial.printf("  Servo B: %d us  running=%d\n", pwmB, runB);
    Serial.printf("  Limit A: %s\n", lA ? "HIT" : "clear");
    Serial.printf("  Limit B: %s\n", lB ? "HIT" : "clear");
    Serial.printf("  Timed run: %s\n", timedActive ? "active" : "idle");
    Serial.println("--------------");
}

void printHelp() {
    Serial.println("=== COMMANDS ===");
    Serial.println("  a+        Module A forward (open)  1700us");
    Serial.println("  a-        Module A reverse (close) 1300us");
    Serial.println("  a0        Module A stop            1500us");
    Serial.println("  aNNN      Module A custom PWM (1000-2000)");
    Serial.println("  b+        Module B forward (open)  1300us");
    Serial.println("  b-        Module B reverse (close) 1700us");
    Serial.println("  b0        Module B stop            1500us");
    Serial.println("  bNNN      Module B custom PWM (1000-2000)");
    Serial.println("  ta NNN    Timed run A for NNN ms then stop");
    Serial.println("  tb NNN    Timed run B for NNN ms then stop");
    Serial.println("  stop      Stop both servos NOW");
    Serial.println("  s         Print status");
    Serial.println("  help      Show this list");
    Serial.println("================");
}

void handleCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();

    if (cmd == "help") { printHelp(); return; }
    if (cmd == "s" || cmd == "status") { printStatus(); return; }
    if (cmd == "stop") { stopAll(); return; }

    // Module A commands
    if (cmd.startsWith("a")) {
        String arg = cmd.substring(1);
        int pwm;
        if (arg == "+" || arg == "open")  { pwm = 1700; }
        else if (arg == "-" || arg == "close") { pwm = 1300; }
        else if (arg == "0" || arg == "stop")  { pwm = 1500; }
        else { pwm = arg.toInt(); if (pwm < 1000 || pwm > 2000) { Serial.println("ERR: PWM must be 1000-2000"); return; } }
        pwmA = pwm; runA = (pwm != 1500);
        servoWrite(SERVO_A_PIN, pwm);
        Serial.printf("[A] %d us (%s)\n", pwm, pwm > 1500 ? "FWD" : pwm < 1500 ? "REV" : "STOP");
        return;
    }

    // Module B commands
    if (cmd.startsWith("b")) {
        String arg = cmd.substring(1);
        int pwm;
        if (arg == "+" || arg == "open")  { pwm = 1300; }
        else if (arg == "-" || arg == "close") { pwm = 1700; }
        else if (arg == "0" || arg == "stop")  { pwm = 1500; }
        else { pwm = arg.toInt(); if (pwm < 1000 || pwm > 2000) { Serial.println("ERR: PWM must be 1000-2000"); return; } }
        pwmB = pwm; runB = (pwm != 1500);
        servoWrite(SERVO_B_PIN, pwm);
        Serial.printf("[B] %d us (%s)\n", pwm, pwm > 1500 ? "FWD" : pwm < 1500 ? "REV" : "STOP");
        return;
    }

    // Timed run: "ta 350" or "tb 500"
    if (cmd.startsWith("ta ") || cmd.startsWith("tb ")) {
        char mod = cmd.charAt(1);
        uint32_t ms = cmd.substring(3).toInt();
        if (ms < 50 || ms > 9000) { Serial.println("ERR: ms must be 50-9000"); return; }
        int openPWM = (mod == 'a') ? 1700 : 1300;
        timedPin    = (mod == 'a') ? SERVO_A_PIN : SERVO_B_PIN;
        timedFlag   = (mod == 'a') ? &runA : &runB;
        timedMs     = ms;
        timedStart  = millis();
        timedActive = true;
        if (mod == 'a') { pwmA = openPWM; runA = true; }
        else            { pwmB = openPWM; runB = true; }
        servoWrite(timedPin, openPWM);
        Serial.printf("[T%c] Running %d us for %lu ms — MARK YOUR RACK NOW\n",
                      mod, openPWM, ms);
        return;
    }

    Serial.printf("Unknown command: '%s' — type 'help'\n", cmd.c_str());
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== Servo Serial Controller — WROVER-E ===");

    // Limit switches only — NO LED PIN (GPIO2 = strapping pin)
    pinMode(LIMIT_A_PIN, INPUT_PULLUP);
    pinMode(LIMIT_B_PIN, INPUT_PULLUP);
    Serial.println("[1] GPIO OK");

    ledcAttach(SERVO_A_PIN, SERVO_FREQ, SERVO_BITS);
    ledcAttach(SERVO_B_PIN, SERVO_FREQ, SERVO_BITS);
    servoWrite(SERVO_A_PIN, 1500);
    servoWrite(SERVO_B_PIN, 1500);
    Serial.println("[2] Servos attached GPIO 12 & 13 — both STOP");

    Serial.println("[3] Ready. Type 'help' for commands.");
    printHelp();
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
    // Timed run check
    if (timedActive && millis() - timedStart >= timedMs) {
        servoWrite(timedPin, 1500);
        if (timedFlag) *timedFlag = false;
        timedActive = false;
        Serial.printf("[T] Done — %lu ms elapsed. MEASURE RACK NOW.\n", timedMs);
    }

    // Limit switch auto-stop
    if (runA && digitalRead(LIMIT_A_PIN) == LOW) {
        servoWrite(SERVO_A_PIN, 1500);
        pwmA = 1500; runA = false; timedActive = false;
        Serial.println("[LIM] Module A limit switch HIT — auto stopped");
    }
    if (runB && digitalRead(LIMIT_B_PIN) == LOW) {
        servoWrite(SERVO_B_PIN, 1500);
        pwmB = 1500; runB = false; timedActive = false;
        Serial.println("[LIM] Module B limit switch HIT — auto stopped");
    }

    // Serial command reader
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        handleCommand(cmd);
    }
}
