/**
 * Smart Launcher — Servo Calibrator FINAL v3
 * Board  : ESP32 Wrover Module
 * Fix    : HTML served in chunks to avoid PROGMEM stack overflow
 *          Added startup delay matching working debug sketch
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

#define SERVO_A_PIN   12
#define SERVO_B_PIN   13
#define LIMIT_A_PIN    9
#define LIMIT_B_PIN   10
#define LED_PIN        2
#define SERVO_FREQ    50
#define SERVO_BITS    16

const char* SSID = "SmartLauncher";
WebServer server(80);

int  pwmA = 1500, pwmB = 1500;
bool runA = false, runB = false;
bool     timedActive = false;
uint8_t  timedPin    = SERVO_A_PIN;
bool*    timedFlag   = nullptr;
uint32_t timedStart  = 0;
uint32_t timedMs     = 0;

void servoWrite(uint8_t pin, int us) {
    uint32_t duty = (uint32_t)constrain(us,500,2500) * 65535UL / 20000UL;
    ledcWrite(pin, duty);
}

// ── HTML split into 3 parts to avoid large PROGMEM block ─────────
// Part 1: head + styles
static const char HTML1[] =
"<!DOCTYPE html><html lang='en'><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Servo Calibrator</title><style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:sans-serif;background:#0f1117;color:#e0e0e0;padding:14px}"
"h1{color:#4ade80;text-align:center;font-size:1.2rem;margin-bottom:2px}"
".sub{text-align:center;color:#6b7280;font-size:.72rem;margin-bottom:14px}"
".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;max-width:680px;margin:0 auto}"
".card{background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:16px}"
".card h2{font-size:.88rem;color:#93c5fd;margin-bottom:12px}"
".badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.68rem;font-weight:700;margin-bottom:10px}"
".s-stop{background:#374151;color:#9ca3af}.s-open{background:#166534;color:#4ade80}"
".s-close{background:#7c2d12;color:#fb923c}.s-timed{background:#1e3a5f;color:#60a5fa}"
".pwm-val{font-size:1.9rem;font-weight:700;color:#f9fafb;text-align:center;margin:4px 0 2px}"
".pwm-unit{font-size:.68rem;color:#6b7280;text-align:center;margin-bottom:10px}"
"input[type=range]{width:100%;accent-color:#4ade80;margin-bottom:10px}"
".btns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;margin-bottom:8px}"
"button{padding:10px 0;border:none;border-radius:8px;font-size:.78rem;font-weight:700;cursor:pointer}"
".b-o{background:#166534;color:#4ade80}.b-s{background:#374151;color:#e5e7eb}"
".b-c{background:#7c2d12;color:#fb923c}.b-t{background:#1e3a5f;color:#60a5fa;width:100%;padding:9px 0;margin-top:4px}"
".ms-row{display:flex;gap:6px;align-items:center;margin-top:6px}"
".ms-row input[type=number]{flex:1;padding:7px;background:#111318;border:1px solid #2a2d35;border-radius:6px;color:#f9fafb;font-size:.85rem;text-align:center}"
".ms-row label{font-size:.7rem;color:#6b7280}"
".pbar{width:100%;height:4px;background:#1f2937;border-radius:2px;margin-top:8px;overflow:hidden;display:none}"
".pfill{height:100%;background:#60a5fa;width:0%}"
".sw-bar{max-width:680px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:12px}"
".sw-bar h2{font-size:.78rem;color:#6b7280;margin-bottom:8px}"
".sw-row{display:flex;gap:20px;align-items:center}"
".sw{display:flex;align-items:center;gap:8px;font-size:.78rem}"
".dot{width:12px;height:12px;border-radius:50%;background:#374151;transition:.2s}"
".dot.clear{background:#4ade80}.dot.hit{background:#a855f7;box-shadow:0 0 6px #a855f7}"
".up{margin-left:auto;font-size:.68rem;color:#374151}"
".log{max-width:680px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:12px;max-height:140px;overflow-y:auto}"
".log h2{font-size:.78rem;color:#6b7280;margin-bottom:6px}"
".le{font-size:.68rem;font-family:monospace;color:#a3e635;padding:2px 0;border-bottom:1px solid #1a1e27}"
".hint{max-width:680px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:10px;padding:12px;font-size:.7rem;color:#6b7280;line-height:1.7}"
".hint b{color:#93c5fd}"
"</style></head><body>"
"<h1>&#x1F680; Servo Calibrator</h1>"
"<p class='sub'>ESP32 WROVER-E &nbsp;&#183;&nbsp; GPIO 12/13 &nbsp;&#183;&nbsp; 192.168.4.1</p>";

// Part 2: HTML body (cards + switches + log)
static const char HTML2[] =
"<div class='grid'>"
"<div class='card'><h2>MODULE A <span style='color:#374151;font-size:.66rem'>GPIO 12</span></h2>"
"<span id='bdgA' class='badge s-stop'>STOP</span>"
"<div class='pwm-val' id='pvA'>1500</div><div class='pwm-unit'>us</div>"
"<input type='range' id='slA' min='1000' max='2000' value='1500' step='10' oninput='slMove(\"A\",this.value)' onchange='slDrop(\"A\",this.value)'>"
"<div class='btns'>"
"<button class='b-o' onclick='cmd(\"A\",1700,\"OPEN\")'>&#9654; OPEN</button>"
"<button class='b-s' onclick='cmd(\"A\",1500,\"STOP\")'>&#9632; STOP</button>"
"<button class='b-c' onclick='cmd(\"A\",1300,\"CLOSE\")'>&#9664; CLOSE</button></div>"
"<div class='ms-row'><input type='number' id='msA' value='350' min='50' max='9000' step='10'><label>ms</label></div>"
"<button class='b-t' onclick='timed(\"A\")'>&#9201; TIMED RUN</button>"
"<div class='pbar' id='pbA'><div class='pfill' id='pfA'></div></div></div>"
"<div class='card'><h2>MODULE B <span style='color:#374151;font-size:.66rem'>GPIO 13</span></h2>"
"<span id='bdgB' class='badge s-stop'>STOP</span>"
"<div class='pwm-val' id='pvB'>1500</div><div class='pwm-unit'>us</div>"
"<input type='range' id='slB' min='1000' max='2000' value='1500' step='10' oninput='slMove(\"B\",this.value)' onchange='slDrop(\"B\",this.value)'>"
"<div class='btns'>"
"<button class='b-o' onclick='cmd(\"B\",1300,\"OPEN\")'>&#9654; OPEN</button>"
"<button class='b-s' onclick='cmd(\"B\",1500,\"STOP\")'>&#9632; STOP</button>"
"<button class='b-c' onclick='cmd(\"B\",1700,\"CLOSE\")'>&#9664; CLOSE</button></div>"
"<div class='ms-row'><input type='number' id='msB' value='350' min='50' max='9000' step='10'><label>ms</label></div>"
"<button class='b-t' onclick='timed(\"B\")'>&#9201; TIMED RUN</button>"
"<div class='pbar' id='pbB'><div class='pfill' id='pfB'></div></div></div></div>"
"<div class='sw-bar'><h2>LIMIT SWITCHES</h2><div class='sw-row'>"
"<div class='sw'><div id='dotA' class='dot'></div>Module A (GPIO 9)</div>"
"<div class='sw'><div id='dotB' class='dot'></div>Module B (GPIO 10)</div>"
"<div class='up' id='up'>--</div></div></div>"
"<div class='log'><h2>EVENT LOG</h2><div id='log'></div></div>"
"<div class='hint'><b>Calibration:</b><br>"
"1. Mark rack with pen &rarr; enter ms &rarr; press TIMED RUN.<br>"
"2. Measure displacement. New ms = 350 &times; (35 &divide; measured mm).<br>"
"3. Repeat until rack moves exactly 35 mm.<br>"
"4. Use slider to find true STOP (servo fully still).</div>";

// Part 3: JavaScript
static const char HTML3[] =
"<script>"
"let pA=1500,pB=1500,lA=false,lB=false;"
"function ts(){return new Date().toLocaleTimeString();}"
"function log(m,c){const d=document.getElementById('log');const e=document.createElement('div');"
"e.className='le';e.style.color=c||'#a3e635';e.textContent='['+ts()+'] '+m;d.prepend(e);"
"if(d.children.length>60)d.removeChild(d.lastChild);}"
"function badge(mod,lbl,cls){const el=document.getElementById('bdg'+mod);el.textContent=lbl;el.className='badge '+cls;}"
"function slMove(mod,v){v=parseInt(v);"
"if(mod==='A'){pA=v;document.getElementById('pvA').textContent=v;}"
"else{pB=v;document.getElementById('pvB').textContent=v;}}"
"async function slDrop(mod,v){v=parseInt(v);"
"const lbl=v===1500?'STOP':v>1500?'OPEN':'CLOSE';"
"const cls=v===1500?'s-stop':v>1500?'s-open':'s-close';"
"badge(mod,lbl,cls);log('Mod '+mod+' slider '+v+'us');"
"await fetch('/cmd?mod='+mod+'&pwm='+v).catch(()=>log('error','#f87171'));}"
"async function cmd(mod,pwm,lbl){"
"const cls=lbl==='STOP'?'s-stop':lbl==='OPEN'?'s-open':'s-close';"
"if(mod==='A'){pA=pwm;document.getElementById('slA').value=pwm;document.getElementById('pvA').textContent=pwm;}"
"else{pB=pwm;document.getElementById('slB').value=pwm;document.getElementById('pvB').textContent=pwm;}"
"badge(mod,lbl,cls);log('Mod '+mod+' '+pwm+'us ('+lbl+')');"
"await fetch('/cmd?mod='+mod+'&pwm='+pwm).catch(()=>log('error','#f87171'));}"
"async function timed(mod){"
"const ms=parseInt(document.getElementById('ms'+mod).value);"
"if(isNaN(ms)||ms<50||ms>9000){log('ms must be 50-9000','#f87171');return;}"
"const op=mod==='A'?1700:1300;"
"badge(mod,'TIMED','s-timed');"
"log('Mod '+mod+' TIMED '+ms+'ms @ '+op+'us','#60a5fa');"
"const pb=document.getElementById('pb'+mod);const pf=document.getElementById('pf'+mod);"
"pb.style.display='block';pf.style.width='0%';"
"await fetch('/timed?mod='+mod+'&ms='+ms).catch(()=>log('error','#f87171'));"
"const t0=Date.now();"
"const iv=setInterval(()=>{"
"const pct=Math.min(100,(Date.now()-t0)/ms*100);pf.style.width=pct+'%';"
"if(pct>=100){clearInterval(iv);"
"setTimeout(()=>{pb.style.display='none';pf.style.width='0%';},700);"
"badge(mod,'STOP','s-stop');"
"const sl=document.getElementById('sl'+mod);const pv=document.getElementById('pv'+mod);"
"sl.value=1500;pv.textContent=1500;"
"if(mod==='A')pA=1500;else pB=1500;"
"log('Mod '+mod+' DONE ('+ms+'ms) measure rack now!','#4ade80');}},50);}"
"async function poll(){"
"try{const j=await(await fetch('/status')).json();"
"const nA=j.lA?'hit':'clear';const nB=j.lB?'hit':'clear';"
"if(j.lA&&!lA)log('Module A LIMIT HIT','#a855f7');"
"if(j.lB&&!lB)log('Module B LIMIT HIT','#a855f7');"
"lA=j.lA;lB=j.lB;"
"document.getElementById('dotA').className='dot '+nA;"
"document.getElementById('dotB').className='dot '+nB;"
"document.getElementById('up').textContent='up '+j.up+'s';"
"}catch(e){}}"
"setInterval(poll,350);"
"log('Connected - WROVER-E ready');"
"</script></body></html>";

// ── HTTP handlers ─────────────────────────────────────────────────
void handleRoot() {
    // Stream HTML in 3 parts — avoids single large send buffer
    server.setContentLength(strlen(HTML1) + strlen(HTML2) + strlen(HTML3));
    server.send(200, "text/html", "");
    server.sendContent(HTML1);
    server.sendContent(HTML2);
    server.sendContent(HTML3);
}

void handleCmd() {
    if (!server.hasArg("mod") || !server.hasArg("pwm")) {
        server.send(400, "text/plain", "bad"); return;
    }
    String mod = server.arg("mod");
    int    pwm = constrain(server.arg("pwm").toInt(), 1000, 2000);
    if (mod == "A") { pwmA=pwm; runA=(pwm!=1500); servoWrite(SERVO_A_PIN,pwm); }
    else            { pwmB=pwm; runB=(pwm!=1500); servoWrite(SERVO_B_PIN,pwm); }
    Serial.printf("[CMD] %s %d us\n", mod.c_str(), pwm);
    server.send(200, "text/plain", "ok");
}

void handleTimed() {
    if (!server.hasArg("mod") || !server.hasArg("ms")) {
        server.send(400, "text/plain", "bad"); return;
    }
    String   mod   = server.arg("mod");
    uint32_t ms    = constrain((uint32_t)server.arg("ms").toInt(), 50, 9000);
    int      openPW= (mod=="A") ? 1700 : 1300;
    timedPin       = (mod=="A") ? SERVO_A_PIN : SERVO_B_PIN;
    timedFlag      = (mod=="A") ? &runA : &runB;
    timedMs        = ms;
    timedStart     = millis();
    timedActive    = true;
    servoWrite(timedPin, openPW);
    if (timedFlag) *timedFlag = true;
    Serial.printf("[TIMED] %s %d us for %lu ms\n", mod.c_str(), openPW, ms);
    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool lA = (digitalRead(LIMIT_A_PIN) == LOW);
    bool lB = (digitalRead(LIMIT_B_PIN) == LOW);
    if (lA && runA) { servoWrite(SERVO_A_PIN,1500); pwmA=1500; runA=false; timedActive=false; }
    if (lB && runB) { servoWrite(SERVO_B_PIN,1500); pwmB=1500; runB=false; timedActive=false; }
    String j = "{\"lA\":"  + String(lA?"true":"false") +
               ",\"lB\":"  + String(lB?"true":"false") +
               ",\"up\":"  + String(millis()/1000) + "}";
    server.send(200, "application/json", j);
}

// ── Setup ─────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    delay(1000);   // ← same delay as working debug sketch
    Serial.println("\n=== Servo Calibrator v3 — WROVER-E ===");

    Serial.println("[1] GPIO init");
    pinMode(LED_PIN,     OUTPUT);
    pinMode(LIMIT_A_PIN, INPUT_PULLUP);
    pinMode(LIMIT_B_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, HIGH);

    Serial.println("[2] LEDC attach");
    ledcAttach(SERVO_A_PIN, SERVO_FREQ, SERVO_BITS);
    ledcAttach(SERVO_B_PIN, SERVO_FREQ, SERVO_BITS);
    servoWrite(SERVO_A_PIN, 1500);
    servoWrite(SERVO_B_PIN, 1500);

    Serial.println("[3] WiFi AP");
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAP(SSID, "");
    delay(500);
    Serial.printf("[4] IP: %s\n", WiFi.softAPIP().toString().c_str());

    Serial.println("[5] HTTP server");
    server.on("/",       handleRoot);
    server.on("/cmd",    handleCmd);
    server.on("/timed",  handleTimed);
    server.on("/status", handleStatus);
    server.begin();

    Serial.println("[6] Ready — connect to SmartLauncher → http://192.168.4.1");

    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LOW);  delay(120);
        digitalWrite(LED_PIN, HIGH); delay(120);
    }
}

// ── Loop ──────────────────────────────────────────────────────────
void loop() {
    server.handleClient();

    if (timedActive && millis() - timedStart >= timedMs) {
        servoWrite(timedPin, 1500);
        if (timedFlag) *timedFlag = false;
        timedActive = false;
        Serial.printf("[TIMED] done %lu ms\n", timedMs);
    }
    if (runA && digitalRead(LIMIT_A_PIN)==LOW) {
        servoWrite(SERVO_A_PIN,1500); pwmA=1500; runA=false; timedActive=false;
        Serial.println("[LIM] A stop");
    }
    if (runB && digitalRead(LIMIT_B_PIN)==LOW) {
        servoWrite(SERVO_B_PIN,1500); pwmB=1500; runB=false; timedActive=false;
        Serial.println("[LIM] B stop");
    }
    static uint32_t t=0;
    if (millis()-t>2000) {
        t=millis();
        digitalWrite(LED_PIN,LOW); delay(30);
        digitalWrite(LED_PIN,HIGH);
    }
}
