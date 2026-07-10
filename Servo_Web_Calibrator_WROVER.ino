/*
  Smart Launcher — Web UI Servo Calibrator (WROVER-E, retry)
  Board  : ESP32 Wrover Module  (Tools -> Board -> ESP32 Wrover Module,
           NOT "ESP32 Dev Module" — wrong board = PSRAM misconfig = boot crash)
  Flash  : 8MB, QIO, 80MHz, upload 921600

  WHY THE EARLIER WEB UI ATTEMPTS (servo_calibrator.ino, servo_cal_final.ino)
  NEVER GOT CONFIRMED WORKING — TWO SEPARATE BUGS FOUND AND FIXED:

    Bug 1 (known, but re-introduced): PROJECT_HANDOFF.md records "DO NOT
    USE GPIO 2" as an already-confirmed lesson (strapping pin, OUTPUT use
    causes rst:0x8 TG1WDT_SYS_RESET boot loops). Both earlier web-UI
    calibrators still defined LED_PIN = 2 anyway. Fixed here by moving the
    heartbeat LED to GPIO 22.

    Bug 2 (newly found on real hardware, 2026 flash test): even after
    fixing Bug 1, the board still boot-looped with rst:0x8
    (TG1WDT_SYS_RESET) — this time crashing the instant
    pinMode(9, INPUT_PULLUP) ran for the limit switch. Bisected on actual
    hardware (see conversation/PROJECT_HANDOFF.md): LED-only boots clean,
    adding GPIO 9 crashes immediately, switching FlashMode QIO->DIO made
    NO difference. Root cause: this is a WROVER module — it has in-package
    PSRAM, and on WROVER boards GPIO 6-11 are wired internally to the
    shared flash+PSRAM SPI bus UNCONDITIONALLY (not just in QIO mode like
    a plain WROOM without PSRAM). GPIO 9/10 can never be regular GPIO on
    this board. Every earlier sketch in this repo that used GPIO 9/10 for
    limit switches (servo_serial.ino, servo_calibrator.ino,
    servo_cal_final.ino) was silently broken from the start — it just
    never got far enough to prove it, because Bug 1 crashed first.
    Fixed here by moving limit switches to GPIO 32/33 (ordinary GPIO, no
    flash/PSRAM/strapping function) — confirmed by bisection to boot clean
    through LED + limits + LEDC + WiFi AP.

  THE FIX APPLIED HERE:
    - Heartbeat LED on GPIO 22 (not GPIO 2).
    - Limit switches on GPIO 32/33 (not GPIO 9/10 — those are permanently
      reserved for flash+PSRAM on this WROVER module).
    - Kept everything else already validated as good practice: v3.x LEDC
      API (ledcAttach(pin,freq,res) + ledcWrite(pin,duty)), chunked HTML
      (3 parts, avoids one large PROGMEM block), non-blocking timed servo
      runs (loop() polls millis(), no delay() inside a web handler), 1000ms
      startup delay before first Serial output, and serial checkpoint
      prints through setup() so a crash location is visible if anything
      still goes wrong.

  WIRING:
    Servo A signal -> GPIO 12   Servo B signal -> GPIO 13
    Limit switch A -> GPIO 32 (COM->GND, NO->GPIO, INPUT_PULLUP)
    Limit switch B -> GPIO 33 (COM->GND, NO->GPIO, INPUT_PULLUP)
    Heartbeat LED   -> GPIO 22 (optional — omit if you don't have a spare LED)
    Servo power     -> external 5V (UBEC), NEVER from ESP32 pins
    Servo/limit GND -> external supply GND + ESP32 GND, all tied together

    NOTE: if your limit switches were previously wired to GPIO 9/10 per
    the older sketches, you MUST move those two wires to GPIO 32/33 —
    GPIO 9/10 cannot work on this board, no firmware fix is possible.

  USE:
    Connect to Wi-Fi AP "SmartLauncher" (no password) -> http://192.168.4.1

    Each module card has a "Stop (us)" and "Offset (us)" field instead of
    hardcoded 1700/1300 presets. Module A's true stop was measured at
    1650us with Servo_Calibration_Tool_A.ino — that's the default here.
    Module B has not been measured yet (defaults to 1500 — a placeholder,
    not a confirmed value). Run Servo_Calibration_Tool_B.ino first if you
    haven't, then update Module B's "Stop (us)" field to match before
    trusting its OPEN/CLOSE buttons.

    OPEN/CLOSE are computed live from Stop +/- Offset (mirrored for B,
    since B retracts the opposite direction from A). If OPEN/CLOSE come
    out backwards on your actual servo, tick that module's "Reverse"
    checkbox — flips the direction instantly, no reflash needed.

    TIMED RUN uses whatever OPEN currently computes to. Workflow: mark the
    rack with a pen, press TIMED RUN, measure the displacement with a
    ruler, type the number into "measured mm" and press Apply — it does
    new_ms = current_ms * (door_width / measured_mm) for you and fills in
    the corrected ms field. Press TIMED RUN again to verify, repeat until
    measured ≈ door width (35mm default, editable at the top of the page).

  BUG FIX HISTORY (round 2, after first hands-on hardware test):
    1. Default OPEN/CLOSE direction was backwards on the real servos —
       user had to tick Reverse every page load. Flipped the base
       `opensHigh()` formula so the confirmed-correct direction is now
       the unchecked default; Reverse still available per-module in case
       a given unit needs the opposite.
    2. TIMED RUN never stopped ("ran indefinitely"): every auto-stop path
       (timed-run completion, both limit-switch triggers, in both loop()
       and handleStatus()) hardcoded servoWrite(pin, 1500) to mean
       "stopped". But Module A's real measured stop is 1650us, not 1500 —
       writing 1500 to a servo whose true neutral is 1650 doesn't stop
       it, it commands a ~150us-off-center creep. Fixed by having the
       browser send the module's live "Stop (us)" field as a `stop`
       parameter on every /cmd and /timed request; the firmware now
       tracks stopPwmA/stopPwmB from that and uses them everywhere it
       used to hardcode 1500.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>

// ── Pins ─────────────────────────────────────────────────────────────
#define SERVO_A_PIN   12
#define SERVO_B_PIN   13
#define LIMIT_A_PIN   32   // NOT GPIO 9 — reserved for flash+PSRAM on WROVER
#define LIMIT_B_PIN   33   // NOT GPIO 10 — same reason
#define LED_PIN       22   // heartbeat only — GPIO 2 is NEVER used in this sketch

// ── LEDC (v3.x API) ─────────────────────────────────────────────────
#define SERVO_FREQ    50
#define SERVO_BITS    16

// ── Known calibration state (defaults shown in the UI) ──────────────
#define MODULE_A_STOP_DEFAULT   1650   // measured — see Servo_Calibration_Tool_A.ino
#define MODULE_B_STOP_DEFAULT   1500   // NOT yet measured — placeholder
#define DEFAULT_OFFSET           200

const char* SSID = "SmartLauncher";
WebServer server(80);

int  pwmA = MODULE_A_STOP_DEFAULT, pwmB = MODULE_B_STOP_DEFAULT;
bool runA = false, runB = false;

// Real calibrated stop per module, kept in sync from the browser's "Stop
// (us)" field on every command. NEVER hardcode 1500 to mean "stopped" —
// a module's true neutral (e.g. Module A's measured 1650us) can be far
// from 1500, and writing 1500 to a servo whose real stop is elsewhere
// doesn't stop it, it just commands a different creeping speed/direction.
int stopPwmA = MODULE_A_STOP_DEFAULT;
int stopPwmB = MODULE_B_STOP_DEFAULT;

bool     timedActive = false;
uint8_t  timedPin    = SERVO_A_PIN;
bool*    timedFlag   = nullptr;
uint32_t timedStart   = 0;
uint32_t timedMs      = 0;

void servoWrite(uint8_t pin, int us) {
    uint32_t duty = (uint32_t)constrain(us, 1000, 2000) * 65535UL / 20000UL;
    ledcWrite(pin, duty);
}

// ── HTML split into 3 parts — avoids one large PROGMEM block ────────
static const char HTML1[] =
"<!DOCTYPE html><html lang='en'><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Servo Web Calibrator</title><style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:sans-serif;background:#0f1117;color:#e0e0e0;padding:14px}"
"h1{color:#4ade80;text-align:center;font-size:1.2rem;margin-bottom:2px}"
".sub{text-align:center;color:#6b7280;font-size:.72rem;margin-bottom:14px}"
".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;max-width:720px;margin:0 auto}"
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
".ref-row{display:flex;gap:6px;margin-bottom:8px}"
".ref-row div{flex:1}"
".ref-row label{font-size:.65rem;color:#6b7280;display:block;margin-bottom:2px}"
".ref-row input[type=number]{width:100%;padding:6px;background:#111318;border:1px solid #2a2d35;border-radius:6px;color:#f9fafb;font-size:.8rem;text-align:center}"
".preview{font-size:.66rem;color:#6b7280;text-align:center;margin-bottom:8px}"
".preview b{color:#93c5fd}"
".ms-row{display:flex;gap:6px;align-items:center;margin-top:6px}"
".ms-row input[type=number]{flex:1;padding:7px;background:#111318;border:1px solid #2a2d35;border-radius:6px;color:#f9fafb;font-size:.85rem;text-align:center}"
".ms-row label{font-size:.7rem;color:#6b7280}"
".pbar{width:100%;height:4px;background:#1f2937;border-radius:2px;margin-top:8px;overflow:hidden;display:none}"
".pfill{height:100%;background:#60a5fa;width:0%}"
".sw-bar{max-width:720px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:12px}"
".sw-bar h2{font-size:.78rem;color:#6b7280;margin-bottom:8px}"
".sw-row{display:flex;gap:20px;align-items:center}"
".sw{display:flex;align-items:center;gap:8px;font-size:.78rem}"
".dot{width:12px;height:12px;border-radius:50%;background:#374151;transition:.2s}"
".dot.clear{background:#4ade80}.dot.hit{background:#a855f7;box-shadow:0 0 6px #a855f7}"
".up{margin-left:auto;font-size:.68rem;color:#374151}"
".log{max-width:720px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:12px;max-height:140px;overflow-y:auto}"
".log h2{font-size:.78rem;color:#6b7280;margin-bottom:6px}"
".le{font-size:.68rem;font-family:monospace;color:#a3e635;padding:2px 0;border-bottom:1px solid #1a1e27}"
".hint{max-width:720px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:10px;padding:12px;font-size:.7rem;color:#6b7280;line-height:1.7}"
".hint b{color:#93c5fd}"
"</style></head><body>"
"<h1>&#x1F680; Servo Web Calibrator</h1>"
"<p class='sub'>ESP32 WROVER-E &nbsp;&#183;&nbsp; GPIO 12/13 &nbsp;&#183;&nbsp; LED on GPIO22 (not GPIO2) &nbsp;&#183;&nbsp; 192.168.4.1</p>"
"<p class='sub'>Door width: <input type='number' id='doorWidth' value='35' min='5' max='200' step='1' "
"style='width:56px;background:#111318;border:1px solid #2a2d35;border-radius:4px;color:#f9fafb;text-align:center'> mm"
" &nbsp;(used by Apply Correction below)</p>";

// Part 2: body — module cards with editable stop/offset, switches, log
static const char HTML2[] =
"<div class='grid'>"
"<div class='card'><h2>MODULE A <span style='color:#374151;font-size:.66rem'>GPIO 12</span></h2>"
"<span id='bdgA' class='badge s-stop'>STOP</span>"
"<div class='pwm-val' id='pvA'>1650</div><div class='pwm-unit'>us (last command)</div>"
"<div class='ref-row'>"
"<div><label>Stop (us)</label><input type='number' id='stopA' value='1650' min='1000' max='2000' step='5' oninput='preview(\"A\")'></div>"
"<div><label>Offset (us)</label><input type='number' id='offA' value='200' min='10' max='500' step='5' oninput='preview(\"A\")'></div>"
"</div>"
"<label style='font-size:.65rem;color:#6b7280;display:flex;align-items:center;gap:5px;margin-bottom:6px'>"
"<input type='checkbox' id='revA' onchange='preview(\"A\")'> Reverse OPEN/CLOSE (check if they came out swapped)</label>"
"<div class='preview' id='prevA'>OPEN=<b>1850</b> &nbsp; CLOSE=<b>1450</b></div>"
"<input type='range' id='slA' min='1000' max='2000' value='1650' step='5' oninput='slMove(\"A\",this.value)' onchange='slDrop(\"A\",this.value)'>"
"<div class='btns'>"
"<button class='b-o' onclick='pressOpen(\"A\")'>&#9654; OPEN</button>"
"<button class='b-s' onclick='pressStop(\"A\")'>&#9632; STOP</button>"
"<button class='b-c' onclick='pressClose(\"A\")'>&#9664; CLOSE</button></div>"
"<div class='ms-row'><input type='number' id='msA' value='350' min='50' max='9000' step='10'><label>ms</label></div>"
"<button class='b-t' onclick='timed(\"A\")'>&#9201; TIMED RUN</button>"
"<div class='pbar' id='pbA'><div class='pfill' id='pfA'></div></div>"
"<div class='ms-row'><input type='number' id='measA' placeholder='measured mm' min='0' max='500' step='0.5'>"
"<button class='b-t' style='flex:0 0 auto;padding:8px 10px;margin-top:0' onclick='applyCorrection(\"A\")'>Apply</button></div></div>"
"<div class='card'><h2>MODULE B <span style='color:#374151;font-size:.66rem'>GPIO 13</span></h2>"
"<span id='bdgB' class='badge s-stop'>STOP</span>"
"<div class='pwm-val' id='pvB'>1500</div><div class='pwm-unit'>us (last command)</div>"
"<div class='ref-row'>"
"<div><label>Stop (us)</label><input type='number' id='stopB' value='1500' min='1000' max='2000' step='5' oninput='preview(\"B\")'></div>"
"<div><label>Offset (us)</label><input type='number' id='offB' value='200' min='10' max='500' step='5' oninput='preview(\"B\")'></div>"
"</div>"
"<label style='font-size:.65rem;color:#6b7280;display:flex;align-items:center;gap:5px;margin-bottom:6px'>"
"<input type='checkbox' id='revB' onchange='preview(\"B\")'> Reverse OPEN/CLOSE (check if they came out swapped)</label>"
"<div class='preview' id='prevB'>OPEN=<b>1300</b> &nbsp; CLOSE=<b>1700</b></div>"
"<input type='range' id='slB' min='1000' max='2000' value='1500' step='5' oninput='slMove(\"B\",this.value)' onchange='slDrop(\"B\",this.value)'>"
"<div class='btns'>"
"<button class='b-o' onclick='pressOpen(\"B\")'>&#9654; OPEN</button>"
"<button class='b-s' onclick='pressStop(\"B\")'>&#9632; STOP</button>"
"<button class='b-c' onclick='pressClose(\"B\")'>&#9664; CLOSE</button></div>"
"<div class='ms-row'><input type='number' id='msB' value='350' min='50' max='9000' step='10'><label>ms</label></div>"
"<button class='b-t' onclick='timed(\"B\")'>&#9201; TIMED RUN</button>"
"<div class='pbar' id='pbB'><div class='pfill' id='pfB'></div></div>"
"<div class='ms-row'><input type='number' id='measB' placeholder='measured mm' min='0' max='500' step='0.5'>"
"<button class='b-t' style='flex:0 0 auto;padding:8px 10px;margin-top:0' onclick='applyCorrection(\"B\")'>Apply</button></div></div></div>"
"<div class='sw-bar'><h2>LIMIT SWITCHES</h2><div class='sw-row'>"
"<div class='sw'><div id='dotA' class='dot'></div>Module A (GPIO 32)</div>"
"<div class='sw'><div id='dotB' class='dot'></div>Module B (GPIO 33)</div>"
"<div class='up' id='up'>--</div></div></div>"
"<div class='log'><h2>EVENT LOG</h2><div id='log'></div></div>"
"<div class='hint'><b>Workflow:</b><br>"
"1. Set Stop (us) to the value found with Servo_Calibration_Tool_A/B.ino.<br>"
"2. OPEN/CLOSE are Stop &#177; Offset (mirrored for B). If OPEN/CLOSE come out "
"backwards on your unit, tick that module's <b>Reverse</b> box &mdash; no reflash needed.<br>"
"3. Mark the rack with a pen &rarr; enter a starting ms (350 is a reasonable first guess) &rarr; press <b>TIMED RUN</b>.<br>"
"4. Measure how far the rack moved with a ruler &rarr; type it into <b>measured mm</b> &rarr; press <b>Apply</b>. "
"This does the ms &times; (door_width &divide; measured) math for you and fills in the corrected ms.<br>"
"5. Press TIMED RUN again with the corrected ms. Repeat until measured &#8776; door width.<br>"
"6. Record final Stop/Offset/travel_ms (and whether Reverse was needed) per module into mission.json.</div>";

// Part 3: JavaScript
static const char HTML3[] =
"<script>"
"function ts(){return new Date().toLocaleTimeString();}"
"function log(m,c){const d=document.getElementById('log');const e=document.createElement('div');"
"e.className='le';e.style.color=c||'#a3e635';e.textContent='['+ts()+'] '+m;d.prepend(e);"
"if(d.children.length>60)d.removeChild(d.lastChild);}"
"function badge(mod,lbl,cls){const el=document.getElementById('bdg'+mod);el.textContent=lbl;el.className='badge '+cls;}"
"function clamp(v){return Math.min(2000,Math.max(1000,v));}"
"function stopOf(mod){return parseInt(document.getElementById('stop'+mod).value)||1500;}"
"function offOf(mod){return parseInt(document.getElementById('off'+mod).value)||200;}"
"function isRev(mod){return document.getElementById('rev'+mod).checked;}"
"function opensHigh(mod){return(mod==='A')===isRev(mod);}"
"function openPWM(mod){const s=stopOf(mod),o=offOf(mod);return clamp(opensHigh(mod)?s+o:s-o);}"
"function closePWM(mod){const s=stopOf(mod),o=offOf(mod);return clamp(opensHigh(mod)?s-o:s+o);}"
"function preview(mod){document.getElementById('prev'+mod).innerHTML="
"'OPEN=<b>'+openPWM(mod)+'</b> &nbsp; CLOSE=<b>'+closePWM(mod)+'</b>';}"
"function syncSlider(mod,v){document.getElementById('sl'+mod).value=v;document.getElementById('pv'+mod).textContent=v;}"
"function slMove(mod,v){syncSlider(mod,v);}"
"async function sendPwm(mod,pwm,lbl,cls){"
"syncSlider(mod,pwm);badge(mod,lbl,cls);log('Mod '+mod+' '+pwm+'us ('+lbl+')');"
"await fetch('/cmd?mod='+mod+'&pwm='+pwm+'&stop='+stopOf(mod)).catch(()=>log('error','#f87171'));}"
"async function slDrop(mod,v){v=parseInt(v);const s=stopOf(mod);"
"const lbl=v===s?'STOP':(opensHigh(mod)===(v>s)?'OPEN':'CLOSE');"
"const cls=lbl==='STOP'?'s-stop':lbl==='OPEN'?'s-open':'s-close';"
"await sendPwm(mod,v,lbl,cls);}"
"function pressOpen(mod){sendPwm(mod,openPWM(mod),'OPEN','s-open');}"
"function pressStop(mod){sendPwm(mod,stopOf(mod),'STOP','s-stop');}"
"function pressClose(mod){sendPwm(mod,closePWM(mod),'CLOSE','s-close');}"
"function applyCorrection(mod){"
"const measEl=document.getElementById('meas'+mod);"
"const measured=parseFloat(measEl.value);"
"const doorW=parseFloat(document.getElementById('doorWidth').value)||35;"
"if(!measured||measured<=0){log('Mod '+mod+': enter measured mm first','#f87171');return;}"
"const msEl=document.getElementById('ms'+mod);"
"const cur=parseInt(msEl.value)||350;"
"let next=Math.round(cur*(doorW/measured));"
"next=Math.max(50,Math.min(9000,next));"
"msEl.value=next;"
"log('Mod '+mod+' correction: '+cur+'ms \\u00d7 ('+doorW+'/'+measured+'mm) = '+next+'ms \\u2014 press TIMED RUN to verify','#60a5fa');"
"}"
"async function timed(mod){"
"const ms=parseInt(document.getElementById('ms'+mod).value);"
"if(isNaN(ms)||ms<50||ms>9000){log('ms must be 50-9000','#f87171');return;}"
"const op=openPWM(mod);"
"badge(mod,'TIMED','s-timed');"
"log('Mod '+mod+' TIMED '+ms+'ms @ '+op+'us','#60a5fa');"
"const pb=document.getElementById('pb'+mod);const pf=document.getElementById('pf'+mod);"
"pb.style.display='block';pf.style.width='0%';"
"await fetch('/timed?mod='+mod+'&ms='+ms+'&pwm='+op+'&stop='+stopOf(mod)).catch(()=>log('error','#f87171'));"
"const t0=Date.now();"
"const iv=setInterval(()=>{"
"const pct=Math.min(100,(Date.now()-t0)/ms*100);pf.style.width=pct+'%';"
"if(pct>=100){clearInterval(iv);"
"setTimeout(()=>{pb.style.display='none';pf.style.width='0%';},700);"
"const s=stopOf(mod);badge(mod,'STOP','s-stop');syncSlider(mod,s);"
"log('Mod '+mod+' DONE ('+ms+'ms) measure rack now!','#4ade80');}},50);}"
"async function poll(){"
"try{const j=await(await fetch('/status')).json();"
"const nA=j.lA?'hit':'clear';const nB=j.lB?'hit':'clear';"
"document.getElementById('dotA').className='dot '+nA;"
"document.getElementById('dotB').className='dot '+nB;"
"document.getElementById('up').textContent='up '+j.up+'s';"
"}catch(e){}}"
"setInterval(poll,350);"
"preview('A');preview('B');"
"log('Connected - WROVER-E - LED on GPIO22, GPIO2 untouched');"
"</script></body></html>";

// ── HTTP handlers ─────────────────────────────────────────────────
void handleRoot() {
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
    // Browser always sends the module's current "Stop (us)" field alongside
    // every command — keep the server's notion of "stopped" in sync with it.
    if (server.hasArg("stop")) {
        int stopArg = constrain(server.arg("stop").toInt(), 1000, 2000);
        if (mod == "A") stopPwmA = stopArg; else stopPwmB = stopArg;
    }
    if (mod == "A") { pwmA = pwm; runA = (pwm != stopPwmA); servoWrite(SERVO_A_PIN, pwm); }
    else            { pwmB = pwm; runB = (pwm != stopPwmB); servoWrite(SERVO_B_PIN, pwm); }
    Serial.printf("[CMD] %s %d us (stop=%d)\n", mod.c_str(), pwm,
                  mod == "A" ? stopPwmA : stopPwmB);
    server.send(200, "text/plain", "ok");
}

void handleTimed() {
    if (!server.hasArg("mod") || !server.hasArg("ms") || !server.hasArg("pwm") || !server.hasArg("stop")) {
        server.send(400, "text/plain", "bad"); return;
    }
    String   mod     = server.arg("mod");
    uint32_t ms      = constrain((uint32_t)server.arg("ms").toInt(), 50, 9000);
    int      openPW  = constrain(server.arg("pwm").toInt(), 1000, 2000);
    int      stopArg = constrain(server.arg("stop").toInt(), 1000, 2000);
    if (mod == "A") stopPwmA = stopArg; else stopPwmB = stopArg;
    timedPin    = (mod == "A") ? SERVO_A_PIN : SERVO_B_PIN;
    timedFlag   = (mod == "A") ? &runA : &runB;
    timedMs     = ms;
    timedStart  = millis();
    timedActive = true;
    servoWrite(timedPin, openPW);
    if (timedFlag) *timedFlag = true;
    Serial.printf("[TIMED] %s %d us for %lu ms (stop=%d)\n", mod.c_str(), openPW, ms, stopArg);
    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool lA = (digitalRead(LIMIT_A_PIN) == LOW);
    bool lB = (digitalRead(LIMIT_B_PIN) == LOW);
    if (lA && runA) { servoWrite(SERVO_A_PIN, stopPwmA); runA = false; timedActive = false; }
    if (lB && runB) { servoWrite(SERVO_B_PIN, stopPwmB); runB = false; timedActive = false; }
    String j = "{\"lA\":"  + String(lA?"true":"false") +
               ",\"lB\":"  + String(lB?"true":"false") +
               ",\"up\":"  + String(millis()/1000) + "}";
    server.send(200, "application/json", j);
}

// ── Setup — checkpoint prints so a crash location is visible ───────
void setup() {
    Serial.begin(115200);
    delay(1000);   // matches the working debug_boot.ino timing
    Serial.println("\n=== Servo Web Calibrator — WROVER-E retry ===");

    Serial.println("[1] GPIO init (LED on 22, NOT 2)");
    pinMode(LED_PIN,     OUTPUT);
    pinMode(LIMIT_A_PIN, INPUT_PULLUP);
    pinMode(LIMIT_B_PIN, INPUT_PULLUP);
    digitalWrite(LED_PIN, HIGH);

    Serial.println("[2] LEDC attach (v3.x API)");
    ledcAttach(SERVO_A_PIN, SERVO_FREQ, SERVO_BITS);
    ledcAttach(SERVO_B_PIN, SERVO_FREQ, SERVO_BITS);
    servoWrite(SERVO_A_PIN, MODULE_A_STOP_DEFAULT);
    servoWrite(SERVO_B_PIN, MODULE_B_STOP_DEFAULT);

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

    Serial.println("[6] Ready — connect to SmartLauncher -> http://192.168.4.1");

    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, LOW);  delay(120);
        digitalWrite(LED_PIN, HIGH); delay(120);
    }
    Serial.println("[7] setup() complete — loop starting");
}

// ── Loop ─────────────────────────────────────────────────────────
void loop() {
    server.handleClient();

    if (timedActive && millis() - timedStart >= timedMs) {
        servoWrite(timedPin, timedPin == SERVO_A_PIN ? stopPwmA : stopPwmB);
        if (timedFlag) *timedFlag = false;
        timedActive = false;
        Serial.printf("[TIMED] done %lu ms\n", timedMs);
    }
    if (runA && digitalRead(LIMIT_A_PIN) == LOW) {
        servoWrite(SERVO_A_PIN, stopPwmA); runA = false; timedActive = false;
        Serial.println("[LIM] A stop");
    }
    if (runB && digitalRead(LIMIT_B_PIN) == LOW) {
        servoWrite(SERVO_B_PIN, stopPwmB); runB = false; timedActive = false;
        Serial.println("[LIM] B stop");
    }
    static uint32_t t = 0;
    if (millis() - t > 2000) {
        t = millis();
        digitalWrite(LED_PIN, LOW);  delay(30);
        digitalWrite(LED_PIN, HIGH);
    }
}
