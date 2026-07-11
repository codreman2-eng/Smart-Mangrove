/**
 * Smart Launcher — Main entry point
 * Freenove ESP32-WROVER
 * ArduCopter/ArduPilot + DJI M600 compatible
 *
 * Pin assignments (defined in platformio.ini build_flags):
 *   PIN_GNSS_RX=4   PIN_GNSS_TX=5
 *   PIN_MAVLINK_RX=16  PIN_MAVLINK_TX=17
 *   PIN_SERVO_A=12  PIN_SERVO_B=13
 *   PIN_LIMIT_A=32  PIN_LIMIT_B=33
 *   PIN_ARM_BUTTON=0
 *   PIN_STATUS_LED=22   (plain LED, not WS2812 — see StateMachine.cpp)
 *
 * NEVER use GPIO 2 (strapping pin, boot-loops if driven OUTPUT) or
 * GPIO 9/10 (permanently reserved for flash+PSRAM on WROVER modules,
 * cannot be regular GPIO in any flash mode). See PROJECT_HANDOFF.md for
 * the full bisection history that found both of these on real hardware.
 */

#include <Arduino.h>
#include "config/ConfigManager.h"
#include "position/IPositionSource.h"
#include "position/GNSSPositionSource.h"
#include "actuator/ActuatorModule.h"
#include "safety/StateMachine.h"
#include "logging/Logger.h"
#include "logic/DropLogicEngine.h"

// ─── Object instantiation ────────────────────────────────────────────────────

ConfigManager   cfg;
Logger          logger;
StateMachine    sm(PIN_STATUS_LED, PIN_ARM_BUTTON);

// Position source — swapped at runtime based on config
GNSSPositionSource gnssSource(1, PIN_GNSS_RX, PIN_GNSS_TX, 9600);
// MavlinkPositionSource mavSource(2, PIN_MAVLINK_RX, PIN_MAVLINK_TX);  // Phase 4
IPositionSource* posSource = &gnssSource;

// Module A: servo on PIN_SERVO_A, limit on PIN_LIMIT_A, LEDC channel 0
// Module B: servo on PIN_SERVO_B, limit on PIN_LIMIT_B, LEDC channel 1
// PWM and timing come from config — defaults used here until config loads
ActuatorModule modA(PIN_SERVO_A, PIN_LIMIT_A, 0, 1700, 1500, 350, 5);
ActuatorModule modB(PIN_SERVO_B, PIN_LIMIT_B, 1, 1300, 1500, 350, 5);

DropLogicEngine* engine = nullptr;

// ─── GPS fix stability tracking ──────────────────────────────────────────────
static uint32_t gps_fix_start_ms  = 0;
static bool     gps_stable        = false;

bool checkGPSStable(IPositionSource* src, uint8_t required_s) {
    if (!src->hasFix()) {
        gps_fix_start_ms = 0;
        gps_stable       = false;
        return false;
    }
    if (gps_fix_start_ms == 0) {
        gps_fix_start_ms = millis();
    }
    if (!gps_stable && (millis() - gps_fix_start_ms >= (uint32_t)required_s * 1000)) {
        gps_stable = true;
        Serial.printf("[MAIN] GPS stable for %d s — ready to ARM\n", required_s);
    }
    return gps_stable;
}

// ─── Serial command handler (bench testing) ──────────────────────────────────
void handleSerial() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "arm")         { sm.requestArm(); }
    else if (cmd == "disarm") { sm.requestDisarm(); }
    else if (cmd == "status") {
        Position p = posSource->read();
        Serial.printf("[CMD] State=%d Armed=%d GPS valid=%d lat=%.6f lon=%.6f\n",
                      (int)sm.state(), sm.isArmed(), p.valid, p.lat_deg, p.lon_deg);
    }
    else if (cmd == "log")    { logger.dumpLog(); }
    else if (cmd == "reset") {
        modA.reset(); modB.reset();
        Serial.println("[CMD] Modules reset");
    }
    else if (cmd == "stop") {
        // Emergency stop — immediately re-writes each module's calibrated
        // stop PWM. Does NOT touch door counters or arm state.
        modA.stop(); modB.stop();
        Serial.println("[CMD] STOP — both servos commanded to stop PWM");
    }
    else if (cmd == "dropa")  {
        // Manual bench test — fire module A once
        DropResult r = modA.dropNext();
        Serial.printf("[CMD] Manual drop A: %s\n",
                      r == DropResult::OK ? "OK" : "TIMEOUT/FAIL");
    }
    else if (cmd == "dropb")  {
        DropResult r = modB.dropNext();
        Serial.printf("[CMD] Manual drop B: %s\n",
                      r == DropResult::OK ? "OK" : "TIMEOUT/FAIL");
    }
    else if (cmd == "help") {
        Serial.println("Commands: arm | disarm | status | log | reset | stop | dropa | dropb");
    }
    else {
        Serial.printf("[CMD] Unknown: '%s' — type 'help'\n", cmd.c_str());
    }
}

// ─── Setup ───────────────────────────────────────────────────────────────────
// Numbered checkpoints on purpose: on this exact board (WROVER-E) two
// separate boot-loop bugs (GPIO 2 strapping pin, GPIO 9/10 reserved for
// flash+PSRAM) were only diagnosable by seeing exactly which checkpoint
// printed last before a silent reset. Keep this pattern for any future
// hardware bring-up on this board — see PROJECT_HANDOFF.md.
void setup() {
    Serial.begin(115200);
    delay(1000);   // matches the timing validated during WROVER-E bring-up
    Serial.println("\n=== Smart Launcher booting ===");

    Serial.println("[1] Config");
    if (!cfg.begin()) {
        Serial.println("[MAIN] Config load FAILED — check /mission.json on LittleFS");
        sm.setFault("No config");
        return;
    }
    cfg.printSummary();

    Serial.println("[2] Logger");
    logger.begin();

    Serial.println("[3] State machine (LED GPIO22, ARM button GPIO0)");
    sm.begin();

    Serial.println("[4] Position source (GNSS)");
    // Select based on config (MAVLink path added in Phase 4)
    posSource = &gnssSource;
    posSource->begin();

    Serial.println("[5] Actuators (servo GPIO12/13, limit GPIO32/33)");
    const MissionConfig& mc = cfg.config();
    modA = ActuatorModule(PIN_SERVO_A, PIN_LIMIT_A, 0,
                          mc.module_a.servo_open_pwm_us,
                          mc.module_a.servo_stop_pwm_us,
                          mc.module_a.travel_ms,
                          mc.module_a.door_count);
    modB = ActuatorModule(PIN_SERVO_B, PIN_LIMIT_B, 1,
                          mc.module_b.servo_open_pwm_us,
                          mc.module_b.servo_stop_pwm_us,
                          mc.module_b.travel_ms,
                          mc.module_b.door_count);
    modA.begin();
    modB.begin();

    Serial.println("[6] Drop logic engine");
    engine = new DropLogicEngine(cfg, *posSource, modA, modB, sm, logger);

    Serial.println("[7] BOOT -> IDLE");
    sm.completeBoot();   // one-time BOOT->IDLE transition (NOT requestDisarm —
                          // that's guarded to only fire from ARMED and would
                          // silently no-op here, leaving the system stuck in BOOT)
    Serial.println("[MAIN] Boot complete — IDLE");
    Serial.println("[MAIN] Type 'help' for bench test commands");
    Serial.println("[MAIN] Hold ARM button 2s to ARM");
    Serial.println("[8] setup() complete — loop starting");
}

// ─── Loop ────────────────────────────────────────────────────────────────────
void loop() {
    // Update state machine (button + LED)
    sm.update();

    // Update position
    posSource->update();

    // Check GPS stability — required before arming in real flight
    // (bench testing: skip with serial 'arm' command)
    if (!gps_stable) {
        const MissionConfig& mc = cfg.config();
        checkGPSStable(posSource, mc.min_gps_fix_duration_s);
    }

    // Run drop logic
    if (engine) engine->update();

    // Serial command handler (bench testing)
    handleSerial();

    // Mission complete notification
    if (engine && engine->missionComplete() && sm.isArmed()) {
        Serial.println("[MAIN] *** MISSION COMPLETE — all drops done ***");
        sm.requestDisarm();
    }

    delay(50);   // 20 Hz main loop — sufficient for 3m radius at typical drone speeds
}
