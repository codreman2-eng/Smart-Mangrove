/**
 * Smart Launcher — Servo Calibrator
 * Board  : ESP32 WROVER-E
 * Core   : v3.x (no external libraries)
 * GPIO   : Servo A = 12, Servo B = 13 (change if needed)
 *
 * Connect to Wi-Fi AP "SmartLauncher" (no password)
 * Open browser → http://192.168.4.1
 *
 * Features:
 *  - OPEN / STOP / CLOSE buttons per module
 *  - Fine PWM slider (1000–2000µs)
 *  - TIMED RUN button: runs servo for exactly N ms then stops
 *    → use this to calibrate how many ms = 35mm travel
 *  - Limit switch live indicators (auto-stop on trigger)
 *  - Event log with timestamps
 *
 * Wiring:
 *  Servo A signal → GPIO 12
 *  Servo B signal → GPIO 13
 *  Limit SW A     → GPIO 9  + GND (COM→GND, NO→GPIO)
 *  Limit SW B     → GPIO 10 + GND
 *  Servo power    → external 5V
 *  Common GND     → external GND + ESP32 GND tied together
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Pin definitions ──────────────────────────────────────────────
#define SERVO_A_PIN   12
#define SERVO_B_PIN   13
#define LIMIT_A_PIN    9
#define LIMIT_B_PIN   10
#define LED_PIN        2

// ── LEDC ─────────────────────────────────────────────────────────
#define SERVO_FREQ    50
#define SERVO_BITS    16

// ── Wi-Fi ─────────────────────────────────────────────────────────
const char* SSID = "SmartLauncher";
const char* PASS = "";

WebServer server(80);

// ── State ─────────────────────────────────────────────────────────
int  pwmA = 1500, pwmB = 1500;
bool runA = false, runB = false;

// Timed run state
bool     timedRunActive = false;
uint8_t  timedRunPin    = SERVO_A_PIN;
uint32_t timedRunStart  = 0;
uint32_t timedRunMs     = 0;
int      timedRunPWM    = 1700;
bool*    timedRunFlag   = nullptr;

// ── Helpers ───────────────────────────────────────────────────────
void servoWrite(uint8_t pin, int us) {
    uint32_t duty = (uint32_t)us * 65535UL / 20000UL;
    ledcWrite(pin, duty);
}

void servoStop(uint8_t pin, int &pwm, bool &running) {
    pwm     = 1500;
    running = false;
    servoWrite(pin, 1500);
}

// ── HTML ──────────────────────────────────────────────────────────
const char HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Smart Launcher Calibrator</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',sans-serif;background:#0f1117;color:#e0e0e0;padding:14px}
h1{color:#4ade80;text-align:center;font-size:1.25rem;margin-bottom:2px}
.sub{text-align:center;color:#6b7280;font-size:.72rem;margin-bottom:14px}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;max-width:700px;margin:0 auto}
.card{background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:16px}
.card h2{font-size:.9rem;color:#93c5fd;margin-bottom:12px}
.badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.68rem;font-weight:700;margin-bottom:10px}
.s-stop{background:#374151;color:#9ca3af}
.s-open{background:#166534;color:#4ade80}
.s-close{background:#7c2d12;color:#fb923c}
.s-timed{background:#1e3a5f;color:#60a5fa}
.pwm-val{font-size:2rem;font-weight:700;color:#f9fafb;text-align:center;margin:4px 0 2px}
.pwm-unit{font-size:.7rem;color:#6b7280;text-align:center;margin-bottom:10px}
input[type=range]{width:100%;accent-color:#4ade80;margin-bottom:10px}
.btns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;margin-bottom:8px}
button{padding:10px 0;border:none;border-radius:8px;font-size:.78rem;font-weight:700;cursor:pointer;transition:.12s}
.b-o{background:#166534;color:#4ade80}.b-o:hover{background:#15803d}
.b-s{background:#374151;color:#e5e7eb}.b-s:hover{background:#4b5563}
.b-c{background:#7c2d12;color:#fb923c}.b-c:hover{background:#9a3412}
.timed-row{display:flex;gap:6px;align-items:center;margin-top:6px}
.timed-row input[type=number]{width:70px;padding:8px 6px;background:#111318;border:1px solid #2a2d35;border-radius:6px;color:#f9fafb;font-size:.85rem;text-align:center}
.b-t{flex:1;background:#1e3a5f;color:#60a5fa;padding:10px 0}.b-t:hover{background:#1e4976}
.progress-bar{width:100%;height:4px;background:#1f2937;border-radius:2px;margin-top:8px;overflow:hidden;display:none}
.progress-fill{height:100%;background:#60a5fa;width:0%;transition:width .1s linear}
.sw-bar{max-width:700px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:14px}
.sw-bar h2{font-size:.8rem;color:#6b7280;margin-bottom:8px}
.sw-row{display:flex;gap:20px;align-items:center;flex-wrap:wrap}
.sw{display:flex;align-items:center;gap:8px;font-size:.8rem}
.dot{width:12px;height:12px;border-radius:50%;background:#374151;transition:.2s}
.dot.clear{background:#4ade80}
.dot.hit{background:#a855f7;box-shadow:0 0 8px #a855f7}
.uptime{margin-left:auto;font-size:.7rem;color:#374151}
.log{max-width:700px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:14px;max-height:170px;overflow-y:auto}
.log h2{font-size:.8rem;color:#6b7280;margin-bottom:6px}
.le{font-size:.7rem;font-family:monospace;color:#a3e635;padding:2px 0;border-bottom:1px solid #1a1e27}
.hint{max-width:700px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:10px;padding:12px;font-size:.72rem;color:#6b7280;line-height:1.6}
.hint strong{color:#93c5fd}
</style>
</head>
<body>
<h1>&#x1F680; Smart Launcher — Calibrator</h1>
<p class="sub">ESP32 WROVER-E &nbsp;·&nbsp; GPIO 12/13 &nbsp;·&nbsp; 192.168.4.1</p>

<div class="grid">
  <!-- MODULE A -->
  <div class="card">
    <h2>MODULE A <span style="color:#374151;font-size:.68rem">GPIO 12</span></h2>
    <span id="bdgA" class="badge s-stop">STOP</span>
    <div class="pwm-val" id="pwmA">1500</div>
    <div class="pwm-unit">µs PWM</div>
    <input type="range" id="slA" min="1000" max="2000" value="1500" step="10"
           oninput="slMove('A',this.value)" onchange="slRelease('A',this.value)">
    <div class="btns">
      <button class="b-o" onclick="cmd('A',1700,'OPEN')">&#x25B6; OPEN</button>
      <button class="b-s" onclick="cmd('A',1500,'STOP')">&#x25A0; STOP</button>
      <button class="b-c" onclick="cmd('A',1300,'CLOSE')">&#x25C4; CLOSE</button>
    </div>
    <!-- Timed run calibration tool -->
    <div class="timed-row">
      <input type="number" id="msA" value="350" min="50" max="5000" step="10">
      <span style="font-size:.7rem;color:#6b7280">ms</span>
      <button class="b-t button" onclick="timedRun('A')">&#x23F1; TIMED RUN</button>
    </div>
    <div class="progress-bar" id="pbA"><div class="progress-fill" id="pfA"></div></div>
  </div>

  <!-- MODULE B -->
  <div class="card">
    <h2>MODULE B <span style="color:#374151;font-size:.68rem">GPIO 13</span></h2>
    <span id="bdgB" class="badge s-stop">STOP</span>
    <div class="pwm-val" id="pwmB">1500</div>
    <div class="pwm-unit">µs PWM</div>
    <input type="range" id="slB" min="1000" max="2000" value="1500" step="10"
           oninput="slMove('B',this.value)" onchange="slRelease('B',this.value)">
    <div class="btns">
      <button class="b-o" onclick="cmd('B',1300,'OPEN')">&#x25B6; OPEN</button>
      <button class="b-s" onclick="cmd('B',1500,'STOP')">&#x25A0; STOP</button>
      <button class="b-c" onclick="cmd('B',1700,'CLOSE')">&#x25C4; CLOSE</button>
    </div>
    <div class="timed-row">
      <input type="number" id="msB" value="350" min="50" max="5000" step="10">
      <span style="font-size:.7rem;color:#6b7280">ms</span>
      <button class="b-t button" onclick="timedRun('B')">&#x23F1; TIMED RUN</button>
    </div>
    <div class="progress-bar" id="pbB"><div class="progress-fill" id="pfB"></div></div>
  </div>
</div>

<!-- Limit switches -->
<div class="sw-bar">
  <h2>LIMIT SWITCHES</h2>
  <div class="sw-row">
    <div class="sw"><div id="dotA" class="dot"></div>Module A (GPIO 9)</div>
    <div class="sw"><div id="dotB" class="dot"></div>Module B (GPIO 10)</div>
    <div class="uptime" id="up">--</div>
  </div>
</div>

<!-- Log -->
<div class="log"><h2>EVENT LOG</h2><div id="log"></div></div>

<!-- Calibration hint -->
<div class="hint">
  <strong>Calibration workflow:</strong><br>
  1. Mark a reference point on the rack with a pen.<br>
  2. Set ms field to <strong>350</strong> and press <strong>TIMED RUN</strong>.<br>
  3. Measure how far the rack moved with a ruler.<br>
  4. Calculate: <strong>new_ms = 350 × (35 ÷ measured_mm)</strong><br>
  5. Repeat until rack moves exactly <strong>35 mm</strong>.<br>
  6. Use slider to find true STOP neutral (servo fully still).<br>
  7. Record both values in <strong>mission.json</strong>.
</div>

<script>
let pA=1500,pB=1500,lastLA=false,lastLB=false;
let timerA=null,timerB=null;

function ts(){return new Date().toLocaleTimeString();}
function addLog(m,col){
  const d=document.getElementById('log');
  const e=document.createElement('div');
  e.className='le';
  e.style.color=col||'#a3e635';
  e.textContent='['+ts()+'] '+m;
  d.prepend(e);
  if(d.children.length>60)d.removeChild(d.lastChild);
}
function setBadge(mod,label,cls){
  const el=document.getElementById('bdg'+mod);
  el.textContent=label;
  el.className='badge '+cls;
}
function slMove(mod,v){
  v=parseInt(v);
  if(mod==='A'){pA=v;document.getElementById('pwmA').textContent=v;}
  else{pB=v;document.getElementById('pwmB').textContent=v;}
}
async function slRelease(mod,v){
  v=parseInt(v);
  const label=v===1500?'STOP':v>1500?'OPEN':'CLOSE';
  const cls=v===1500?'s-stop':v>1500?'s-open':'s-close';
  setBadge(mod,label,cls);
  addLog('Module '+mod+' custom PWM → '+v+'µs');
  await fetch('/cmd?mod='+mod+'&pwm='+v).catch(()=>addLog('⚠ error','#f87171'));
}
async function cmd(mod,pwm,label){
  const cls=label==='STOP'?'s-stop':label==='OPEN'?'s-open':'s-close';
  if(mod==='A'){pA=pwm;document.getElementById('slA').value=pwm;document.getElementById('pwmA').textContent=pwm;}
  else{pB=pwm;document.getElementById('slB').value=pwm;document.getElementById('pwmB').textContent=pwm;}
  setBadge(mod,label,cls);
  addLog('Module '+mod+' → '+pwm+'µs ('+label+')');
  await fetch('/cmd?mod='+mod+'&pwm='+pwm).catch(()=>addLog('⚠ error','#f87171'));
}

// Timed run — runs servo for N ms then auto-stops, shows progress bar
async function timedRun(mod){
  const ms=parseInt(document.getElementById('ms'+mod).value);
  if(ms<50||ms>5000){addLog('⚠ ms must be 50–5000','#f87171');return;}

  // Determine open PWM for this module
  const openPWM = mod==='A' ? 1700 : 1300;

  // Clear any existing timer
  if(mod==='A'&&timerA){clearInterval(timerA);}
  if(mod==='B'&&timerB){clearInterval(timerB);}

  addLog('Module '+mod+' TIMED RUN → '+ms+'ms @ '+openPWM+'µs','#60a5fa');
  setBadge(mod,'TIMED ▶','s-timed');

  // Show progress bar
  const pb=document.getElementById('pb'+mod);
  const pf=document.getElementById('pf'+mod);
  pb.style.display='block';
  pf.style.width='0%';

  // Tell ESP32 to run for exactly ms milliseconds then stop
  await fetch('/timed?mod='+mod+'&ms='+ms).catch(()=>addLog('⚠ error','#f87171'));

  // Animate progress bar client-side
  const start=Date.now();
  const interval=setInterval(()=>{
    const elapsed=Date.now()-start;
    const pct=Math.min(100,(elapsed/ms)*100);
    pf.style.width=pct+'%';
    if(elapsed>=ms){
      clearInterval(interval);
      pf.style.width='100%';
      setTimeout(()=>{pb.style.display='none';pf.style.width='0%';},800);
      setBadge(mod,'STOP','s-stop');
      if(mod==='A'){document.getElementById('slA').value=1500;document.getElementById('pwmA').textContent=1500;}
      else{document.getElementById('slB').value=1500;document.getElementById('pwmB').textContent=1500;}
      addLog('Module '+mod+' timed run complete ('+ms+'ms) — measure rack now!','#4ade80');
    }
  },50);

  if(mod==='A')timerA=interval; else timerB=interval;
}

// Poll limit switches + uptime every 300ms
async function poll(){
  try{
    const j=await(await fetch('/status')).json();
    document.getElementById('dotA').className='dot '+(j.lA?'hit':'clear');
    document.getElementById('dotB').className='dot '+(j.lB?'hit':'clear');
    document.getElementById('up').textContent='up '+j.up+'s';
    if(j.lA&&!lastLA)addLog('⚡ Module A LIMIT HIT','#a855f7');
    if(j.lB&&!lastLB)addLog('⚡ Module B LIMIT HIT','#a855f7');
    lastLA=j.lA; lastLB=j.lB;
  }catch(e){}
}
setInterval(poll,300);
addLog('Connected · WROVER-E · GPIO 12/13');
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
        server.send(400, "text/plain", "bad"); return;
    }
    String mod = server.arg("mod");
    int    pwm = constrain(server.arg("pwm").toInt(), 1000, 2000);
    if (mod == "A") { pwmA = pwm; runA = (pwm != 1500); servoWrite(SERVO_A_PIN, pwm); }
    else            { pwmB = pwm; runB = (pwm != 1500); servoWrite(SERVO_B_PIN, pwm); }
    Serial.printf("[CMD] Module %s → %d µs\n", mod.c_str(), pwm);
    server.send(200, "text/plain", "ok");
}

// Timed run: non-blocking — sets a timer, loop() stops it
void handleTimed() {
    if (!server.hasArg("mod") || !server.hasArg("ms")) {
        server.send(400, "text/plain", "bad"); return;
    }
    String   mod = server.arg("mod");
    timedRunMs   = constrain(server.arg("ms").toInt(), 50, 5000);
    timedRunPWM  = (mod == "A") ? 1700 : 1300;
    timedRunPin  = (mod == "A") ? SERVO_A_PIN : SERVO_B_PIN;
    timedRunFlag = (mod == "A") ? &runA : &runB;

    Serial.printf("[TIMED] Module %s → %d µs for %d ms\n",
                  mod.c_str(), timedRunPWM, timedRunMs);

    servoWrite(timedRunPin, timedRunPWM);
    *timedRunFlag  = true;
    timedRunStart  = millis();
    timedRunActive = true;

    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool lA = (digitalRead(LIMIT_A_PIN) == LOW);
    bool lB = (digitalRead(LIMIT_B_PIN) == LOW);
    if (lA && runA) { servoStop(SERVO_A_PIN, pwmA, runA); Serial.println("[LIM] A auto-stop"); }
    if (lB && runB) { servoStop(SERVO_B_PIN, pwmB, runB); Serial.println("[LIM] B auto-stop"); }
    String j = "{\"lA\":"  + String(lA?"true":"false") +
               ",\"lB\":"  + String(lB?"true":"false") +
               ",\"up\":"  + String(millis()/1000) + "}";
    server.send(200, "application/json", j);
}

// ── Setup & loop ──────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(400);
    Serial.println("\n=== Smart Launcher Calibrator — WROVER-E ===");

    pinMode(LED_PIN,     OUTPUT);
    pinMode(LIMIT_A_PIN, INPUT_PULLUP);
    pinMode(LIMIT_B_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, HIGH);

    // Attach servos — v3.x API, no library
    ledcAttach(SERVO_A_PIN, SERVO_FREQ, SERVO_BITS);
    ledcAttach(SERVO_B_PIN, SERVO_FREQ, SERVO_BITS);
    servoWrite(SERVO_A_PIN, 1500);
    servoWrite(SERVO_B_PIN, 1500);
    Serial.println("[HW] Servos on GPIO 12 & 13 — both at STOP");

    WiFi.softAP(SSID, PASS);
    Serial.printf("[WiFi] AP: '%s'  IP: %s\n", SSID,
                  WiFi.softAPIP().toString().c_str());

    server.on("/",       handleRoot);
    server.on("/cmd",    handleCmd);
    server.on("/timed",  handleTimed);
    server.on("/status", handleStatus);
    server.begin();
    Serial.println("[HTTP] Ready → http://192.168.4.1");
    Serial.println("[INFO] Blink = alive");

    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LOW);  delay(150);
        digitalWrite(LED_PIN, HIGH); delay(150);
    }
}

void loop() {
    server.handleClient();

    // Non-blocking timed run stop
    if (timedRunActive && (millis() - timedRunStart >= timedRunMs)) {
        servoWrite(timedRunPin, 1500);
        if (timedRunFlag) *timedRunFlag = false;
        timedRunActive = false;
        Serial.printf("[TIMED] Done — %lu ms elapsed\n", timedRunMs);
    }

    // Limit switch safety stop every loop iteration
    if (runA && digitalRead(LIMIT_A_PIN) == LOW) {
        servoStop(SERVO_A_PIN, pwmA, runA);
        timedRunActive = false;
        Serial.println("[LIM] A limit hit — auto-stop");
    }
    if (runB && digitalRead(LIMIT_B_PIN) == LOW) {
        servoStop(SERVO_B_PIN, pwmB, runB);
        timedRunActive = false;
        Serial.println("[LIM] B limit hit — auto-stop");
    }

    // Heartbeat blink every 2s
    static uint32_t t = 0;
    if (millis() - t > 2000) {
        t = millis();
        digitalWrite(LED_PIN, LOW);  delay(20);
        digitalWrite(LED_PIN, HIGH);
    }
}
