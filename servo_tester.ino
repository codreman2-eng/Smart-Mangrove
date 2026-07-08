/**
 * Smart Launcher — Servo Tester
 * Board : Freenove ESP32-S3 WROOM
 * Built-in LED : GPIO 2
 *
 * Connects to Wi-Fi AP "SmartLauncher" (no password)
 * Open browser → http://192.168.4.1
 *
 * Wiring:
 *   Servo signal → GPIO 18  (Module A)
 *   Servo signal → GPIO 8   (Module B)
 *   Limit switch A → GPIO 9  (other leg to GND)
 *   Limit switch B → GPIO 10 (other leg to GND)
 *   Servo power (red)  → 5V UBEC
 *   Servo ground (blk) → UBEC GND + ESP32 GND (common ground)
 *
 * To change servo pin: edit SERVO_A_PIN / SERVO_B_PIN below
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Pin definitions ──────────────────────────────────────────────
#define SERVO_A_PIN   18
#define SERVO_B_PIN    8
#define LIMIT_A_PIN    9
#define LIMIT_B_PIN   10
#define LED_PIN        2    // Built-in blue LED on Freenove WROOM

// ── LEDC config ──────────────────────────────────────────────────
#define LEDC_FREQ_HZ   50
#define LEDC_RES_BITS  16
#define LEDC_CH_A       0
#define LEDC_CH_B       1

// ── Wi-Fi AP ─────────────────────────────────────────────────────
const char* AP_SSID = "SmartLauncher";
const char* AP_PASS = "";          // open network

WebServer server(80);

// ── State ────────────────────────────────────────────────────────
struct ServoState {
    int     pwm_us     = 1500;    // current commanded pulse
    bool    running    = false;
    bool    limit_hit  = false;
    String  label      = "STOP";
};
ServoState stA, stB;

// ── LEDC helpers ─────────────────────────────────────────────────
void writePWM(uint8_t channel, int pulse_us) {
    uint32_t duty = (uint32_t)pulse_us * 65536UL / 20000UL;
    ledcWrite(channel, duty);
}

void servoStop(uint8_t channel, ServoState& st) {
    st.pwm_us  = 1500;
    st.running = false;
    st.label   = "STOP";
    writePWM(channel, 1500);
}

void servoRun(uint8_t channel, ServoState& st, int pwm_us, const char* lbl) {
    st.pwm_us  = pwm_us;
    st.running = true;
    st.label   = lbl;
    writePWM(channel, pwm_us);
}

// ── HTML page (single file, no external deps) ─────────────────────
const char HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Launcher — Servo Tester</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:'Segoe UI',sans-serif;background:#0f1117;color:#e0e0e0;min-height:100vh;padding:20px}
  h1{color:#4ade80;text-align:center;font-size:1.4rem;margin-bottom:4px}
  .sub{text-align:center;color:#6b7280;font-size:.8rem;margin-bottom:20px}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;max-width:700px;margin:0 auto}
  .card{background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:20px}
  .card h2{font-size:1rem;color:#93c5fd;margin-bottom:16px;letter-spacing:.05em}
  .badge{display:inline-block;padding:4px 12px;border-radius:20px;font-size:.75rem;font-weight:600;margin-bottom:14px}
  .badge.stop{background:#374151;color:#9ca3af}
  .badge.fwd{background:#166534;color:#4ade80}
  .badge.rev{background:#7c2d12;color:#fb923c}
  .badge.limit{background:#7c3aed;color:#c4b5fd}
  .pwm-val{font-size:2rem;font-weight:700;color:#f9fafb;text-align:center;margin:8px 0}
  .pwm-unit{font-size:.8rem;color:#6b7280;text-align:center;margin-bottom:14px}
  input[type=range]{width:100%;accent-color:#4ade80;margin-bottom:14px}
  .btns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px}
  button{padding:10px 0;border:none;border-radius:8px;font-size:.85rem;font-weight:600;cursor:pointer;transition:.15s}
  .btn-fwd{background:#166534;color:#4ade80}
  .btn-fwd:hover{background:#15803d}
  .btn-stop{background:#374151;color:#e5e7eb}
  .btn-stop:hover{background:#4b5563}
  .btn-rev{background:#7c2d12;color:#fb923c}
  .btn-rev:hover{background:#9a3412}
  .status-bar{max-width:700px;margin:16px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:16px}
  .status-bar h2{font-size:.85rem;color:#6b7280;margin-bottom:10px}
  .sw-row{display:flex;gap:20px;align-items:center}
  .sw{display:flex;align-items:center;gap:8px;font-size:.85rem}
  .dot{width:12px;height:12px;border-radius:50%;background:#374151}
  .dot.hit{background:#a855f7}
  .dot.clear{background:#4ade80}
  .log{max-width:700px;margin:12px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:16px;max-height:180px;overflow-y:auto}
  .log h2{font-size:.85rem;color:#6b7280;margin-bottom:8px}
  .log-entry{font-size:.75rem;font-family:monospace;color:#a3e635;padding:2px 0;border-bottom:1px solid #1f2937}
</style>
</head>
<body>
<h1>&#x1F680; Smart Launcher — Servo Tester</h1>
<p class="sub">Freenove ESP32-S3 WROOM &nbsp;|&nbsp; 192.168.4.1</p>

<div class="grid">
  <!-- MODULE A -->
  <div class="card">
    <h2>MODULE A &nbsp; <span style="color:#6b7280;font-size:.75rem">GPIO 18</span></h2>
    <span id="badgeA" class="badge stop">STOP</span><br>
    <div class="pwm-val" id="pwmA">1500</div>
    <div class="pwm-unit">µs PWM</div>
    <input type="range" id="sliderA" min="1000" max="2000" value="1500" step="10"
           oninput="updateSlider('A',this.value)">
    <div class="btns">
      <button class="btn-fwd" onclick="cmd('A','fwd')">&#x25B6; OPEN</button>
      <button class="btn-stop" onclick="cmd('A','stop')">&#x25A0; STOP</button>
      <button class="btn-rev" onclick="cmd('A','rev')">&#x25C4; CLOSE</button>
    </div>
  </div>

  <!-- MODULE B -->
  <div class="card">
    <h2>MODULE B &nbsp; <span style="color:#6b7280;font-size:.75rem">GPIO 8</span></h2>
    <span id="badgeB" class="badge stop">STOP</span><br>
    <div class="pwm-val" id="pwmB">1500</div>
    <div class="pwm-unit">µs PWM</div>
    <input type="range" id="sliderB" min="1000" max="2000" value="1500" step="10"
           oninput="updateSlider('B',this.value)">
    <div class="btns">
      <button class="btn-fwd" onclick="cmd('B','fwd')">&#x25B6; OPEN</button>
      <button class="btn-stop" onclick="cmd('B','stop')">&#x25A0; STOP</button>
      <button class="btn-rev" onclick="cmd('B','rev')">&#x25C4; CLOSE</button>
    </div>
  </div>
</div>

<!-- Limit switch status -->
<div class="status-bar">
  <h2>LIMIT SWITCHES</h2>
  <div class="sw-row">
    <div class="sw"><div id="dotA" class="dot"></div> Module A (GPIO 9)</div>
    <div class="sw"><div id="dotB" class="dot"></div> Module B (GPIO 10)</div>
    <div style="margin-left:auto;font-size:.75rem;color:#4b5563" id="uptime">--</div>
  </div>
</div>

<!-- Event log -->
<div class="log" id="logBox">
  <h2>EVENT LOG</h2>
  <div id="logEntries"></div>
</div>

<script>
// PWM values per module
let pwmA = 1500, pwmB = 1500;

function addLog(msg) {
  const d = document.getElementById('logEntries');
  const t = new Date().toLocaleTimeString();
  const e = document.createElement('div');
  e.className = 'log-entry';
  e.textContent = `[${t}] ${msg}`;
  d.prepend(e);
  if (d.children.length > 40) d.removeChild(d.lastChild);
}

function updateSlider(mod, val) {
  val = parseInt(val);
  if (mod === 'A') { pwmA = val; document.getElementById('pwmA').textContent = val; }
  else             { pwmB = val; document.getElementById('pwmB').textContent = val; }
}

function setBadge(mod, label) {
  const el = document.getElementById('badge' + mod);
  el.textContent = label;
  el.className = 'badge ' + (label==='STOP'?'stop': label.includes('OPEN')?'fwd':'rev');
}

async function cmd(mod, action) {
  let pwm = (mod === 'A') ? pwmA : pwmB;
  // For quick buttons use preset PWMs; slider value used when set
  if (action === 'fwd')  { pwm = (mod === 'A') ? 1700 : 1300; }
  if (action === 'stop') { pwm = 1500; }
  if (action === 'rev')  { pwm = (mod === 'A') ? 1300 : 1700; }

  // Sync slider display
  if (mod === 'A') { document.getElementById('sliderA').value = pwm; document.getElementById('pwmA').textContent = pwm; pwmA = pwm; }
  else             { document.getElementById('sliderB').value = pwm; document.getElementById('pwmB').textContent = pwm; pwmB = pwm; }

  setBadge(mod, action==='fwd'?'OPEN ▶':action==='rev'?'◀ CLOSE':'STOP');
  addLog(`Module ${mod} → ${pwm}µs (${action.toUpperCase()})`);

  try {
    await fetch(`/cmd?mod=${mod}&pwm=${pwm}`);
  } catch(e) { addLog('⚠ Connection error'); }
}

// Custom PWM from slider — send on release
document.getElementById('sliderA').addEventListener('change', () => {
  fetch(`/cmd?mod=A&pwm=${pwmA}`);
  addLog(`Module A custom PWM → ${pwmA}µs`);
});
document.getElementById('sliderB').addEventListener('change', () => {
  fetch(`/cmd?mod=B&pwm=${pwmB}`);
  addLog(`Module B custom PWM → ${pwmB}µs`);
});

// Poll status every 300ms
async function pollStatus() {
  try {
    const r = await fetch('/status');
    const j = await r.json();

    document.getElementById('dotA').className = 'dot ' + (j.limitA ? 'hit' : 'clear');
    document.getElementById('dotB').className = 'dot ' + (j.limitB ? 'hit' : 'clear');
    document.getElementById('uptime').textContent = 'uptime ' + j.uptime_s + 's';

    if (j.limitA) addLog('⚡ Module A limit switch HIT');
    if (j.limitB) addLog('⚡ Module B limit switch HIT');
  } catch(e) {}
}

// Poll every 300ms
setInterval(pollStatus, 300);
addLog('Connected to Smart Launcher tester');
</script>
</body>
</html>
)rawhtml";

// ── HTTP handlers ─────────────────────────────────────────────────
void handleRoot() {
    server.send_P(200, "text/html", HTML);
}

void handleCmd() {
    if (!server.hasArg("mod") || !server.hasArg("pwm")) {
        server.send(400, "text/plain", "missing args");
        return;
    }
    String mod    = server.arg("mod");
    int    pwm_us = server.arg("pwm").toInt();
    pwm_us = constrain(pwm_us, 1000, 2000);

    if (mod == "A") {
        stA.pwm_us  = pwm_us;
        stA.running = (pwm_us != 1500);
        stA.label   = (pwm_us > 1500) ? "OPEN" : (pwm_us < 1500) ? "CLOSE" : "STOP";
        writePWM(LEDC_CH_A, pwm_us);
        Serial.printf("[CMD] Module A → %d µs (%s)\n", pwm_us, stA.label.c_str());
    } else {
        stB.pwm_us  = pwm_us;
        stB.running = (pwm_us != 1500);
        stB.label   = (pwm_us > 1500) ? "OPEN" : (pwm_us < 1500) ? "CLOSE" : "STOP";
        writePWM(LEDC_CH_B, pwm_us);
        Serial.printf("[CMD] Module B → %d µs (%s)\n", pwm_us, stB.label.c_str());
    }
    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool limA = (digitalRead(LIMIT_A_PIN) == LOW);
    bool limB = (digitalRead(LIMIT_B_PIN) == LOW);

    // Auto-stop if limit switch hits while running
    if (limA && stA.running) {
        servoStop(LEDC_CH_A, stA);
        Serial.println("[LIM] Module A limit switch → AUTO STOP");
    }
    if (limB && stB.running) {
        servoStop(LEDC_CH_B, stB);
        Serial.println("[LIM] Module B limit switch → AUTO STOP");
    }

    String json = "{\"limitA\":" + String(limA ? "true" : "false") +
                  ",\"limitB\":"  + String(limB ? "true" : "false") +
                  ",\"pwmA\":"    + String(stA.pwm_us) +
                  ",\"pwmB\":"    + String(stB.pwm_us) +
                  ",\"uptime_s\":" + String(millis() / 1000) + "}";
    server.send(200, "application/json", json);
}

// ── Setup & loop ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== Servo Tester — Freenove ESP32-S3 WROOM ===");

    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Limit switches
    pinMode(LIMIT_A_PIN, INPUT_PULLUP);
    pinMode(LIMIT_B_PIN, INPUT_PULLUP);

    // LEDC servo channels
    ledcSetup(LEDC_CH_A, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcSetup(LEDC_CH_B, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttachPin(SERVO_A_PIN, LEDC_CH_A);
    ledcAttachPin(SERVO_B_PIN, LEDC_CH_B);
    writePWM(LEDC_CH_A, 1500);    // both servos stopped
    writePWM(LEDC_CH_B, 1500);
    Serial.println("[HW] Servos initialised — both at 1500µs (STOP)");

    // Wi-Fi AP
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP started: SSID='%s'  IP=%s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());

    // Routes
    server.on("/",       handleRoot);
    server.on("/cmd",    handleCmd);
    server.on("/status", handleStatus);
    server.begin();
    Serial.println("[HTTP] Server ready → open http://192.168.4.1");
    Serial.println("[INFO] Blink LED = server healthy");

    // Slow blink indicates ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LOW); delay(200);
        digitalWrite(LED_PIN, HIGH); delay(200);
    }
}

void loop() {
    server.handleClient();

    // Heartbeat blink every 2s
    static uint32_t last_blink = 0;
    if (millis() - last_blink > 2000) {
        last_blink = millis();
        digitalWrite(LED_PIN, LOW);  delay(80);
        digitalWrite(LED_PIN, HIGH);
    }
}
