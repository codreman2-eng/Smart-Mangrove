/*
  RC Pulse Generator — Arduino Mega 2560
  Companion to RC_Input_Calibration_Tool.ino (ESP32 WROVER-E)

  PURPOSE
  Generates a known-good RC PWM signal so the launcher's pilot-trigger
  reader can be validated BEFORE anything is connected to the DJI M600's
  A3 Pro F-channel. Lets you prove the reader against exact commanded
  values, and — more importantly — exercise the failure modes (out-of-range
  pulses, wrong frame rate, signal loss) that you cannot safely provoke on
  a real aircraft.

  Uses Timer1 hardware PWM, not digitalWrite() or the Servo library:
    - Hardware timer = crystal-referenced, jitter-free. That matters when
      the whole point is being a trustworthy reference.
    - 0.5us resolution at 50Hz (prescaler 8).
    - The Servo library clamps writeMicroseconds() to 544-2400us, which
      would make the out-of-range tests impossible. Timer1 has no such
      limit: this sketch will happily emit 300us or 2800us on demand.

  ============================================================
  !!! WIRING — 5V LOGIC, LEVEL SHIFT IS MANDATORY !!!
  ============================================================
  The Mega drives 5V. ESP32 GPIO 35 is NOT 5V tolerant and has no series
  protection. Connecting directly WILL stress the pad.

    Mega pin 11 ──[ 1k ]──┬────────── ESP32 GPIO 35
                          │
                        [ 2k ]
                          │
    Mega GND ─────────────┴────────── ESP32 GND   (COMMON GROUND REQUIRED)

  Divider maths, with the ESP32's existing 10k pull-down in parallel with
  the 2k leg:
      R2_eff = 2k || 10k = 1.67k
      Vout   = 5V * 1.67 / (1 + 1.67) = 3.12V     <- safe, and well above
                                                     the ~2.475V V_IH
  DO NOT use 10k/20k here. With the 10k pull-down in parallel that gives
  only 2.0V, which is BELOW the logic-high threshold — the ESP32 would see
  a permanently low line and report NO SIGNAL, looking exactly like a
  broken wire.

  Do NOT connect the Mega's 5V rail to the ESP32 anywhere.
  ============================================================

  COMMANDS (Serial Monitor @ 115200, newline ending)
    w <us>   set pulse width in microseconds   (e.g.  w 1500)
    h <hz>   set frame rate in Hz              (e.g.  h 50)
    1 / 2 / 3   presets: 1000us / 1500us / 1900us
    o        toggle output on/off              (signal-loss test)
    s        sweep 1000 -> 2000us over 5s      (for the reader's 'g' suggest)
    b        band walk: 1000/1500/1900, 2s each, repeating
    ?        print status / help

  SUGGESTED TEST ORDER (see SIM_MODE_VALIDATION_PLAN.md)
    1. w 1000 / w 1500 / w 1900   -> reader should show those +/- a few us
    2. h 100, then h 10           -> reader's Hz should follow
    3. w 2500 and w 600           -> reader: NO SIGNAL, glitches CLIMBING
    4. o (off)                    -> reader: NO SIGNAL within ~500ms
    5. s, then 'g' on the reader  -> should suggest ~1250 / ~1750
*/

#include <Arduino.h>

const uint8_t PIN_OUT = 11;      // OC1A on the Mega 2560 — fixed by Timer1

// ─── Live state ────────────────────────────────────────────────────────
uint32_t width_us  = 1500;       // pulse width
uint32_t period_us = 20000;      // 20ms = 50Hz
bool     output_on = true;

// Sweep
bool     sweeping    = false;
uint32_t sweep_start = 0;
const uint32_t SWEEP_MS = 5000;

// Band walk
bool     walking     = false;
uint32_t walk_start  = 0;
uint8_t  walk_step   = 0;
const uint32_t WALK_HOLD_MS = 2000;
const uint16_t WALK_US[3] = { 1000, 1500, 1900 };

// ─── Timer1: Fast PWM, TOP = ICR1 ──────────────────────────────────────
// Prescaler picked from the requested period:
//   /8  -> 0.5us per tick, max period 32.7ms  (>=30Hz)   — best resolution
//   /64 -> 4.0us per tick, max period 262ms   (>=3.8Hz)  — for slow rates
void applyTimer() {
    uint32_t top, cmp;
    uint8_t  presc;

    if (period_us <= 32000UL) {
        presc = _BV(CS11);                 // /8
        top   = period_us * 2UL;
        cmp   = width_us  * 2UL;
    } else {
        presc = _BV(CS11) | _BV(CS10);     // /64
        top   = period_us / 4UL;
        cmp   = width_us  / 4UL;
    }

    if (top > 65535UL) top = 65535UL;
    if (cmp < 1)       cmp = 1;
    if (cmp >= top)    cmp = top - 1;      // pulse must fit inside the frame

    noInterrupts();
    // WGM13:11 = Fast PWM with ICR1 as TOP. COM1A1 connects the pin; with
    // it cleared the pin reverts to normal I/O so digitalWrite() works.
    TCCR1A = _BV(WGM11) | (output_on ? _BV(COM1A1) : 0);
    TCCR1B = _BV(WGM13) | _BV(WGM12) | presc;
    ICR1   = (uint16_t)top;
    OCR1A  = (uint16_t)cmp;
    interrupts();

    if (!output_on) digitalWrite(PIN_OUT, LOW);
}

void setWidth(uint32_t us) {
    // Deliberately permissive: out-of-range widths are the whole point of
    // tests 3 below. Only clamped to what the timer can physically emit.
    if (us < 50)     us = 50;
    if (us > period_us - 50) us = period_us - 50;
    width_us = us;
    applyTimer();
}

void setHz(uint32_t hz) {
    if (hz < 4)   hz = 4;        // /64 prescaler floor
    if (hz > 400) hz = 400;      // pulse must still fit in the frame
    period_us = 1000000UL / hz;
    if (width_us > period_us - 50) width_us = period_us - 50;
    applyTimer();
}

void status() {
    uint32_t hz = 1000000UL / period_us;
    Serial.println();
    Serial.println(F("=== RC PULSE GENERATOR ==="));
    Serial.print(F("  Output   : ")); Serial.println(output_on ? F("ON") : F("OFF"));
    Serial.print(F("  Width    : ")); Serial.print(width_us);  Serial.println(F(" us"));
    Serial.print(F("  Period   : ")); Serial.print(period_us); Serial.print(F(" us  ("));
    Serial.print(hz); Serial.println(F(" Hz)"));
    Serial.print(F("  Pin      : D")); Serial.print(PIN_OUT);
    Serial.println(F("  (5V - use the 1k/2k divider!)"));
    if (width_us < 800 || width_us > 2200) {
        Serial.println(F("  NOTE: outside 800-2200us - the reader should count"));
        Serial.println(F("        this as a GLITCH and report NO SIGNAL."));
    }
    Serial.println(F("=========================="));
    Serial.println(F("w <us> | h <hz> | 1/2/3 | o | s | b | ?"));
}

// ─── Serial commands (non-blocking) ────────────────────────────────────
void handleLine(String line) {
    line.trim();
    if (line.length() == 0) return;
    char c = line.charAt(0);
    long v = (line.length() > 1) ? line.substring(1).toInt() : 0;

    switch (c) {
        case 'w':
            sweeping = walking = false;
            setWidth((uint32_t)v);
            Serial.print(F("[OK] width = ")); Serial.print(width_us); Serial.println(F(" us"));
            if (width_us < 800 || width_us > 2200)
                Serial.println(F("     (out of RC range - expect NO SIGNAL + glitches climbing)"));
            break;

        case 'h':
            setHz((uint32_t)v);
            Serial.print(F("[OK] ")); Serial.print(1000000UL / period_us);
            Serial.print(F(" Hz (period ")); Serial.print(period_us); Serial.println(F(" us)"));
            break;

        case '1': sweeping = walking = false; setWidth(1000); Serial.println(F("[OK] 1000 us (SAFE)")); break;
        case '2': sweeping = walking = false; setWidth(1500); Serial.println(F("[OK] 1500 us (MID)"));  break;
        case '3': sweeping = walking = false; setWidth(1900); Serial.println(F("[OK] 1900 us (FIRE)")); break;

        case 'o':
            output_on = !output_on;
            applyTimer();
            Serial.print(F("[OK] output ")); Serial.println(output_on ? F("ON") : F("OFF - reader should show NO SIGNAL in ~500ms"));
            break;

        case 's':
            sweeping = true; walking = false;
            sweep_start = millis();
            Serial.println(F("[OK] sweeping 1000 -> 2000 us over 5s"));
            Serial.println(F("     then press 'g' on the reader - expect ~1250 / ~1750"));
            break;

        case 'b':
            walking = !walking; sweeping = false;
            walk_start = millis(); walk_step = 0;
            Serial.println(walking ? F("[OK] band walk started (1000/1500/1900, 2s each)")
                                   : F("[OK] band walk stopped"));
            break;

        case '?': status(); break;
        default:  Serial.println(F("[ERR] unknown - '?' for help"));
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(PIN_OUT, OUTPUT);
    digitalWrite(PIN_OUT, LOW);
    applyTimer();
    delay(200);
    Serial.println(F("\n=== RC Pulse Generator (Mega 2560) ==="));
    Serial.println(F("!! Mega is 5V - GPIO35 is NOT 5V tolerant !!"));
    Serial.println(F("!! Use 1k (D11->node) + 2k (node->GND). NOT 10k/20k. !!"));
    status();
}

void loop() {
    static String buf;
    while (Serial.available()) {
        char ch = Serial.read();
        if (ch == '\n' || ch == '\r') { handleLine(buf); buf = ""; }
        else if (buf.length() < 24)   { buf += ch; }
    }

    uint32_t now = millis();

    // Linear sweep 1000 -> 2000us, so the reader's min/max capture sees the
    // full travel and its suggest algorithm has something real to chew on.
    if (sweeping) {
        uint32_t el = now - sweep_start;
        if (el >= SWEEP_MS) {
            sweeping = false;
            setWidth(2000);
            Serial.println(F("[OK] sweep done (held at 2000 us)"));
        } else {
            setWidth(1000 + (1000UL * el) / SWEEP_MS);
        }
    }

    // Repeating SAFE -> MID -> FIRE walk. Useful later for the dashboard's
    // edge-trigger tests, where entry into FIRE must fire exactly once.
    if (walking && (now - walk_start >= WALK_HOLD_MS)) {
        walk_start = now;
        walk_step  = (walk_step + 1) % 3;
        setWidth(WALK_US[walk_step]);
        Serial.print(F("[WALK] ")); Serial.print(WALK_US[walk_step]); Serial.println(F(" us"));
    }
}
