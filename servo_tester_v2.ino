/**
 * Smart Launcher — Servo Tester v2
 * Board : Freenove ESP32-S3 WROOM
 * Core  : ESP32 Arduino core v3.x (new LEDC API)
 *
 * LEDC API change in core v3.x:
 *   OLD (v2.x): ledcSetup(ch, freq, res) + ledcAttachPin(pin, ch)
 *   NEW (v3.x): ledcAttach(pin, freq, res)  ← single call, no channel needed
 *               ledcWrite(pin, duty)         ← uses pin directly, not channel
 *
 * Wiring:
 *   Servo A signal  → GPIO 18
 *   Servo B signal  → GPIO 8
 *   Limit switch A  → GPIO 9  + GND  (COM→GND, NO→GPIO)
 *   Limit switch B  → GPIO 10 + GND
 *   Servo power red → UBEC 5V
 *   Servo GND       → UBEC GND + ESP32 GND (common ground — critical)
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Pin definitions ──────────────────────────────────────────────
#define SERVO_A_PIN    18
#define SERVO_B_PIN     8
#define LIMIT_A_PIN     9
#define LIMIT_B_PIN    10
#define LED_PIN         2     // Built-in blue LED on Freenove WROOM

// ── LEDC config ───────────────────────────────────────────────────
#define LEDC_FREQ_HZ    50
#define LEDC_RES_BITS   16

// ── Wi-Fi AP ──────────────────────────────────────────────────────
const char* AP_SSID = "SmartLauncher";
const char* AP_PASS = "";

WebServer server(80);

// ── State ─────────────────────────────────────────────────────────
int  pwmA     = 1500;
int  pwmB     = 1500;
bool runningA = false;
bool runningB = false;

// ── LEDC helper — v3.x API ───────────────────────────────────────
// ledcWrite in v3.x takes (pin, duty) not (channel, duty)
void writePWM(uint8_t pin, int pulse_us) {
    uint32_t duty = (uint32_t)pulse_us * 65536UL / 20000UL;
    ledcWrite(pin, duty);
}

void stopServo(uint8_t pin, int &pwm_ref, bool &running_ref) {
    pwm_ref     = 1500;
    running_ref = false;
    writePWM(pin, 1500);
}

// ── HTML UI ───────────────────────────────────────────────────────
const char HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Launcher — Servo Tester</title>
<style>
  *{box-sizing:border-box;margin:0;padding:0}
  body{font-family:'Segoe UI',sans-serif;background:#0f1117;color:#e0e0e0;min-height:100vh;padding:16px}
  h1{color:#4ade80;text-align:center;font-size:1.3rem;margin-bottom:2px}
  .sub{text-align:center;color:#6b7280;font-size:.75rem;margin-bottom:16px}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:680px;margin:0 auto}
  .card{background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:18px}
  .card h2{font-size:.95rem;color:#93c5fd;margin-bottom:14px}
  .badge{display:inline-block;padding:3px 12px;border-radius:20px;font-size:.7rem;font-weight:700;margin-bottom:12px}
  .stop{background:#374151;color:#9ca3af}
  .open{background:#166534;color:#4ade80}
  .close{background:#7c2d12;color:#fb923c}
  .pwm-val{font-size:2.2rem;font-weight:700;color:#f9fafb;text-align:center;margin:6px 0 2px}
  .pwm-unit{font-size:.72rem;color:#6b7280;text-align:center;margin-bottom:12px}
  input[type=range]{width:100%;accent-color:#4ade80;cursor:pointer;margin-bottom:12px}
  .btns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:6px}
  button{padding:11px 0;border:none;border-radius:8px;font-size:.82rem;font-weight:700;cursor:pointer;transition:.12s}
  .btn-o{background:#166534;color:#4ade80}.btn-o:hover{background:#15803d}
  .btn-s{background:#374151;color:#e5e7eb}.btn-s:hover{background:#4b5563}
  .btn-c{background:#7c2d12;color:#fb923c}.btn-c:hover{background:#9a3412}
  .status-bar{max-width:680px;margin:12px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:14px}
  .status-bar h2{font-size:.8rem;color:#6b7280;margin-bottom:10px}
  .sw-row{display:flex;gap:24px;align-items:center;flex-wrap:wrap}
  .sw{display:flex;align-items:center;gap:8px;font-size:.82rem}
  .dot{width:13px;height:13px;border-radius:50%;transition:.2s;background:#374151}
  .dot.clear{background:#4ade80}
  .dot.hit{background:#a855f7;box-shadow:0 0 8px #a855f7}
  .uptime{margin-left:auto;font-size:.72rem;color:#4b5563}
  .log{max-width:680px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:14px;max-height:160px;overflow-y:auto}
  .log h2{font-size:.8rem;color:#6b7280;margin-bottom:6px}
  .le{font-size:.72rem;font-family:monospace;color:#a3e635;padding:2px 0;border-bottom:1px solid #1a1e27}
  .note{text-align:center;margin-top:8px;font-size:.68rem;color:#374151}
</style>
</head>
<body>
<h1>&#x1F680; Smart Launcher — Servo Tester</h1>
<p class="sub">Freenove ESP32-S3 WROOM &nbsp;·&nbsp; Core v3.x &nbsp;·&nbsp; 192.168.4.1</p>

<div class="grid">
  <!-- MODULE A -->
  <div class="card">
    <h2>MODULE A <span style="color:#4b5563;font-size:.7rem">GPIO 18</span></h2>
    <span id="bdgA" class="badge stop">STOP</span>
    <div class="pwm-val" id="pwmA">1500</div>
    <div class="pwm-unit">µs PWM</div>
    <input type="range" id="slA" min="1000" max="2000" value="1500" step="10"
           oninput="sliderMove('A',this.value)"
           onchange="sliderRelease('A',this.value)">
    <div class="btns">
      <button class="btn-o" onclick="cmd('A',1700,'OPEN')">&#x25B6; OPEN</button>
      <button class="btn-s" onclick="cmd('A',1500,'STOP')">&#x25A0; STOP</button>
      <button class="btn-c" onclick="cmd('A',1300,'CLOSE')">&#x25C4; CLOSE</button>
    </div>
  </div>

  <!-- MODULE B -->
  <div class="card">
    <h2>MODULE B <span style="color:#4b5563;font-size:.7rem">GPIO 8</span></h2>
    <span id="bdgB" class="badge stop">STOP</span>
    <div class="pwm-val" id="pwmB">1500</div>
    <div class="pwm-unit">µs PWM</div>
    <input type="range" id="slB" min="1000" max="2000" value="1500" step="10"
           oninput="sliderMove('B',this.value)"
           onchange="sliderRelease('B',this.value)">
    <div class="btns">
      <button class="btn-o" onclick="cmd('B',1300,'OPEN')">&#x25B6; OPEN</button>
      <button class="btn-s" onclick="cmd('B',1500,'STOP')">&#x25A0; STOP</button>
      <button class="btn-c" onclick="cmd('B',1700,'CLOSE')">&#x25C4; CLOSE</button>
    </div>
  </div>
</div>

<!-- Limit switches -->
<div class="status-bar">
  <h2>LIMIT SWITCHES</h2>
  <div class="sw-row">
    <div class="sw"><div id="dotA" class="dot"></div>Module A (GPIO 9)</div>
    <div class="sw"><div id="dotB" class="dot"></div>Module B (GPIO 10)</div>
    <div class="uptime" id="up">--</div>
  </div>
</div>

<!-- Event log -->
<div class="log">
  <h2>EVENT LOG</h2>
  <div id="log"></div>
</div>
<p class="note">Module B OPEN/CLOSE directions are reversed vs A — correct for rack-and-pinion balance</p>

<script>
let pA=1500,pB=1500,lastLA=false,lastLB=false;

function ts(){return new Date().toLocaleTimeString();}

function addLog(m){
  const d=document.getElementById('log');
  const e=document.createElement('div');
  e.className='le';
  e.textContent='['+ts()+'] '+m;
  d.prepend(e);
  if(d.children.length>60)d.removeChild(d.lastChild);
}

function setBadge(mod,label){
  const el=document.getElementById('bdg'+mod);
  el.textContent=label;
  el.className='badge '+(label==='STOP'?'stop':label==='OPEN'?'open':'close');
}

function sliderMove(mod,v){
  v=parseInt(v);
  if(mod==='A'){pA=v;document.getElementById('pwmA').textContent=v;}
  else         {pB=v;document.getElementById('pwmB').textContent=v;}
}

async function sliderRelease(mod,v){
  v=parseInt(v);
  const label=v===1500?'STOP':v>1500?'OPEN':'CLOSE';
  setBadge(mod,label);
  addLog('Module '+mod+' custom PWM → '+v+'µs');
  try{await fetch('/cmd?mod='+mod+'&pwm='+v);}
  catch(e){addLog('⚠ connection lost');}
}

async function cmd(mod,pwm,label){
  if(mod==='A'){pA=pwm;document.getElementById('slA').value=pwm;document.getElementById('pwmA').textContent=pwm;}
  else         {pB=pwm;document.getElementById('slB').value=pwm;document.getElementById('pwmB').textContent=pwm;}
  setBadge(mod,label);
  addLog('Module '+mod+' → '+pwm+'µs ('+label+')');
  try{await fetch('/cmd?mod='+mod+'&pwm='+pwm);}
  catch(e){addLog('⚠ connection lost');}
}

async function poll(){
  try{
    const r=await fetch('/status');
    if(!r.ok)return;
    const j=await r.json();
    const dA=document.getElementById('dotA');
    const dB=document.getElementById('dotB');
    dA.className='dot '+(j.limA?'hit':'clear');
    dB.className='dot '+(j.limB?'hit':'clear');
    document.getElementById('up').textContent='up '+j.up+'s';
    if(j.limA&&!lastLA)addLog('⚡ Module A LIMIT HIT — auto stopped');
    if(j.limB&&!lastLB)addLog('⚡ Module B LIMIT HIT — auto stopped');
    lastLA=j.limA; lastLB=j.limB;
  }catch(e){}
}

setInterval(poll,300);
addLog('Ready · ESP32 core v3.x · ledcAttach API');
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
    String mod = server.arg("mod");
    int    pwm = constrain(server.arg("pwm").toInt(), 1000, 2000);

    if (mod == "A") {
        pwmA     = pwm;
        runningA = (pwm != 1500);
        writePWM(SERVO_A_PIN, pwm);
        Serial.printf("[CMD] Module A → %d µs\n", pwm);
    } else {
        pwmB     = pwm;
        runningB = (pwm != 1500);
        writePWM(SERVO_B_PIN, pwm);
        Serial.printf("[CMD] Module B → %d µs\n", pwm);
    }
    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool limA = (digitalRead(LIMIT_A_PIN) == LOW);
    bool limB = (digitalRead(LIMIT_B_PIN) == LOW);

    // Auto-stop servo when limit switch triggers
    if (limA && runningA) {
        stopServo(SERVO_A_PIN, pwmA, runningA);
        Serial.println("[LIM] Module A limit switch → AUTO STOP");
    }
    if (limB && runningB) {
        stopServo(SERVO_B_PIN, pwmB, runningB);
        Serial.println("[LIM] Module B limit switch → AUTO STOP");
    }

    String json = "{\"limA\":"  + String(limA ? "true" : "false") +
                  ",\"limB\":"  + String(limB ? "true" : "false") +
                  ",\"pwmA\":"  + String(pwmA) +
                  ",\"pwmB\":"  + String(pwmB) +
                  ",\"up\":"    + String(millis() / 1000) + "}";
    server.send(200, "application/json", json);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== Servo Tester v2 — ESP32 core v3.x ===");

    // Built-in LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    // Limit switches with internal pull-up — active LOW
    pinMode(LIMIT_A_PIN, INPUT_PULLUP);
    pinMode(LIMIT_B_PIN, INPUT_PULLUP);

    // ── v3.x LEDC API ─────────────────────────────────────────────
    // One call per pin — no channel number, no ledcSetup needed
    ledcAttach(SERVO_A_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);
    ledcAttach(SERVO_B_PIN, LEDC_FREQ_HZ, LEDC_RES_BITS);
    // ──────────────────────────────────────────────────────────────

    writePWM(SERVO_A_PIN, 1500);   // both servos stopped
    writePWM(SERVO_B_PIN, 1500);
    Serial.println("[HW] Servos init OK — GPIO 18 & 8 at 1500µs STOP");

    // Start Wi-Fi access point
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("[WiFi] AP ready: SSID='%s'  IP=%s\n",
                  AP_SSID, WiFi.softAPIP().toString().c_str());

    // Register routes
    server.on("/",       handleRoot);
    server.on("/cmd",    handleCmd);
    server.on("/status", handleStatus);
    server.begin();
    Serial.println("[HTTP] Server started → http://192.168.4.1");
    Serial.println("[INFO] Blue LED blinks every 2s = server alive");

    // 3 quick blinks = ready
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LOW);  delay(150);
        digitalWrite(LED_PIN, HIGH); delay(150);
    }
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
    server.handleClient();

    // Heartbeat blink every 2s
    static uint32_t lastBlink = 0;
    if (millis() - lastBlink > 2000) {
        lastBlink = millis();
        digitalWrite(LED_PIN, LOW);  delay(60);
        digitalWrite(LED_PIN, HIGH);
    }
}
