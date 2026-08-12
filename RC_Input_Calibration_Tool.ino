/*
  RC Input Calibration Tool — Smart Launcher
  Board: ESP32 Wrover Module (Freenove ESP32-WROVER)

  PURPOSE
  Bring-up and calibration for the pilot "drop now" trigger: an RC PWM
  channel from the DJI M600's A3 Pro F-channel (mapped to an RC channel in
  DJI Assistant 2). Reads the pulse train, reports it live, and tells you
  what band thresholds to write into mission.json.

  Deliberately standalone and minimal — no WiFi, no servo, no LittleFS, no
  dependency on the dashboard's source files. Same philosophy as
  servo_minimal.ino / Servo_Calibration_Tool_A.ino: when bringing up an
  unproven hardware path on this board, prove it in isolation first. Every
  web-UI-first attempt on this project cost a debugging session.

  ── WIRING ────────────────────────────────────────────────────────────
    A3 F-channel signal  ->  GPIO 35
    A3 ground            ->  ESP32 GND        (COMMON GROUND IS MANDATORY)
    10k resistor         ->  GPIO 35 to GND   (see below — REQUIRED)

  GPIO 35 is INPUT-ONLY: it cannot be configured as an output, so no
  firmware bug can ever back-drive the flight controller. That safety comes
  with a catch — input-only pins (34-39) have NO internal pull-up or
  pull-down. Without the external 10k pull-down a disconnected signal wire
  floats and reads as noise instead of "no signal".

  ── VOLTAGE — CHECK BEFORE CONNECTING ─────────────────────────────────
  The A3's F-channel outputs should be 3.3V logic, which connects directly.
  MEASURE IT FIRST with a meter or scope. If it reads 5V, fit a divider
  (10k from signal to the pin, 20k from the pin to GND) or a level shifter.
  The ESP32 is not 5V tolerant.

  ── HOW TO CALIBRATE ──────────────────────────────────────────────────
    1. Power the A3 and this board. Open Serial Monitor at 115200.
    2. Confirm "frames" climbs and rate reads ~50 Hz. If not, stop and fix
       the wiring — nothing below is meaningful without a clean signal.
    3. Move the switch to each position and hold ~2s. Watch min/max.
    4. Type 'g' for suggested thresholds based on what was actually seen.
    5. Write those into mission.json's rc_trigger block (or the dashboard's
       RC INPUT card on the Calibration tab).

  ── COMMANDS ──────────────────────────────────────────────────────────
    p          print current settings + stats
    r          reset min/max/frame/glitch stats
    s <us>     set SAFE max threshold
    f <us>     set FIRE min threshold
    g          suggest thresholds from observed min/max
    h          help
*/

#include <Arduino.h>

#define PIN_RC_IN 35

// Strict RC range — anything outside is a glitch, never a frame
static const uint16_t WIDTH_MIN_US = 800;
static const uint16_t WIDTH_MAX_US = 2200;

// Editable thresholds (defaults match the dashboard firmware's defaults)
static uint16_t safe_max = 1300;
static uint16_t fire_min = 1700;

// ─── ISR state ─────────────────────────────────────────────────────────
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
static volatile uint32_t isr_rise   = 0;
static volatile uint32_t isr_width  = 0;
static volatile uint32_t isr_seq    = 0;
static volatile uint32_t isr_glitch = 0;

static void IRAM_ATTR rcEdgeIsr() {
    uint32_t now = micros();
    if (digitalRead(PIN_RC_IN)) { isr_rise = now; return; }
    uint32_t w = now - isr_rise;
    // Separate read/write rather than ++ — increment on a volatile is
    // deprecated in C++20 and warns under -Wvolatile.
    if (w >= 400 && w <= 3000) { isr_width = w; isr_seq = isr_seq + 1; }
    else                       { isr_glitch = isr_glitch + 1; }
}

// ─── Stats ─────────────────────────────────────────────────────────────
static uint16_t width_us = 0, min_seen = 0xFFFF, max_seen = 0;
// Two glitch sources, counted separately then summed for display. They must
// NOT share one variable: the ISR count is a snapshot that gets re-read every
// frame, so folding the loop-side count into it means each new frame wipes the
// running total and the display sticks at 1 instead of climbing.
static uint32_t frames = 0, glitches_soft = 0, glitches = 0;
static uint32_t last_seq = 0, last_ok_ms = 0;
static uint32_t rate_base = 0, rate_win = 0;
static uint16_t rate_hz = 0;

static const char* bandName(uint16_t us) {
    if (us <= safe_max) return "SAFE";
    if (us >= fire_min) return "FIRE";
    return "MID ";
}

static void printSettings() {
    Serial.println();
    Serial.println("=== RC INPUT ===");
    Serial.printf("  Pin         : GPIO%d (input-only, needs external 10k pull-down)\n", PIN_RC_IN);
    Serial.printf("  SAFE band   : <= %d us\n", safe_max);
    Serial.printf("  MID  band   : %d - %d us\n", safe_max + 1, fire_min - 1);
    Serial.printf("  FIRE band   : >= %d us\n", fire_min);
    Serial.printf("  Frames OK   : %lu   glitches: %lu\n",
                  (unsigned long)frames, (unsigned long)glitches);
    if (min_seen == 0xFFFF) Serial.println("  Seen range  : (nothing yet)");
    else Serial.printf("  Seen range  : %d - %d us\n", min_seen, max_seen);
    Serial.printf("  Frame rate  : %d Hz  (expect ~50)\n", rate_hz);
    Serial.println("================");
}

static void suggest() {
    if (min_seen == 0xFFFF || max_seen == 0 || (max_seen - min_seen) < 200) {
        Serial.println("[SUGGEST] Not enough travel seen yet — move the switch "
                       "through ALL positions and hold each ~2s, then retry 'g'.");
        return;
    }
    // Place thresholds at 25% and 75% of the observed travel. Keeps a wide
    // dead band either side so neither end can be reached by jitter alone.
    uint16_t span = max_seen - min_seen;
    uint16_t s = min_seen + span / 4;
    uint16_t f = min_seen + (span * 3) / 4;
    Serial.println();
    Serial.printf("[SUGGEST] From observed travel %d-%d us (%d us span):\n",
                  min_seen, max_seen, span);
    Serial.printf("            safe_max_us = %d\n", s);
    Serial.printf("            fire_min_us = %d\n", f);
    Serial.println("          Apply here with:");
    Serial.printf("            s %d\n            f %d\n", s, f);
    Serial.println("          Then re-check that each switch position lands");
    Serial.println("          in the band you expect BEFORE trusting it.");
    if (min_seen > 1200) {
        Serial.println("[WARN] Lowest width seen is high — is the switch ever");
        Serial.println("       reaching its idle end? SAFE must be reachable.");
    }
}

static void help() {
    Serial.println();
    Serial.println("Commands: p=print  r=reset stats  s <us>=safe max  "
                   "f <us>=fire min  g=suggest  h=help");
}

// ─── Serial command handling ───────────────────────────────────────────
static void handleLine(String line) {
    line.trim();
    if (line.length() == 0) return;
    char c = line.charAt(0);
    int  v = (line.length() > 1) ? line.substring(1).toInt() : 0;

    switch (c) {
        case 'p': printSettings(); break;
        case 'h': help(); break;
        case 'g': suggest(); break;
        case 'r':
            min_seen = 0xFFFF; max_seen = 0; frames = 0;
            glitches = 0; glitches_soft = 0;
            portENTER_CRITICAL(&mux); isr_glitch = 0; portEXIT_CRITICAL(&mux);
            Serial.println("[OK] stats reset");
            break;
        case 's':
            if (v < 1000 || v > 2000) { Serial.println("[ERR] 1000-2000"); break; }
            if (v >= fire_min - 50)   { Serial.println("[ERR] safe_max must be >=50us below fire_min"); break; }
            safe_max = v; Serial.printf("[OK] safe_max = %d\n", safe_max);
            break;
        case 'f':
            if (v < 1000 || v > 2000) { Serial.println("[ERR] 1000-2000"); break; }
            if (v <= safe_max + 50)   { Serial.println("[ERR] fire_min must be >=50us above safe_max"); break; }
            fire_min = v; Serial.printf("[OK] fire_min = %d\n", fire_min);
            break;
        default:
            Serial.printf("[ERR] unknown '%c' — 'h' for help\n", c);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);   // matches the timing validated on this board all project
    Serial.println("\n=== RC Input Calibration Tool ===");
    Serial.println("[1] GPIO35 input (external 10k pull-down required)");
    pinMode(PIN_RC_IN, INPUT);

    Serial.println("[2] Attaching CHANGE interrupt");
    attachInterrupt(digitalPinToInterrupt(PIN_RC_IN), rcEdgeIsr, CHANGE);

    last_ok_ms = millis();
    rate_win   = millis();

    Serial.println("[3] Ready — move the switch through all positions");
    printSettings();
    help();
}

void loop() {
    // --- Serial commands (non-blocking) ---
    static String buf;
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') { handleLine(buf); buf = ""; }
        else if (buf.length() < 32)   { buf += ch; }
    }

    // --- Snapshot ISR state ---
    uint32_t seq, w, g;
    portENTER_CRITICAL(&mux);
    seq = isr_seq; w = isr_width; g = isr_glitch;
    portEXIT_CRITICAL(&mux);

    uint32_t now = millis();

    if (seq != last_seq) {
        last_seq = seq;
        if (w >= WIDTH_MIN_US && w <= WIDTH_MAX_US) {
            width_us   = (uint16_t)w;
            last_ok_ms = now;
            frames++;
            if (width_us < min_seen) min_seen = width_us;
            if (width_us > max_seen) max_seen = width_us;
        } else {
            glitches_soft++;   // passed the ISR window, failed the strict RC range
        }
    }
    glitches = g + glitches_soft;

    if (now - rate_win >= 1000) {
        rate_hz   = (uint16_t)(frames - rate_base);
        rate_base = frames;
        rate_win  = now;
    }

    // --- Live readout, 5 Hz ---
    static uint32_t last_print = 0;
    if (now - last_print >= 200) {
        last_print = now;
        // `frames > 0` is load-bearing, not belt-and-braces: last_ok_ms is
        // seeded in setup(), so without it the first 500ms after boot pass
        // the freshness check with width_us still 0 — which classifies as
        // "SAFE" and shows a reassuring 0us/SAFE readout when in fact
        // nothing has ever been received. Wrong impression, wrong direction.
        bool live = (frames > 0) && ((now - last_ok_ms) < 500);
        if (!live) {
            Serial.printf("  NO SIGNAL   (last valid %lums ago, glitches %lu)   "
                          "check wiring / common ground / voltage\n",
                          (unsigned long)(now - last_ok_ms), (unsigned long)glitches);
        } else {
            Serial.printf("  %4d us  [%s]   min %4d  max %4d   %2d Hz   "
                          "frames %lu  glitches %lu\n",
                          width_us, bandName(width_us),
                          min_seen == 0xFFFF ? 0 : min_seen, max_seen,
                          rate_hz, (unsigned long)frames, (unsigned long)glitches);
        }
    }
}
