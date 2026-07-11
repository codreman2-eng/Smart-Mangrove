/*
  Smart Launcher — Mission Dashboard + Calibrator (Phase 3)
  Board  : ESP32 Wrover Module  (Tools -> Board -> ESP32 Wrover Module,
           NOT "ESP32 Dev Module" — wrong board = PSRAM misconfig = boot crash)
  Flash  : 8MB, QIO, 80MHz, upload 921600

  This sketch merges two things that were previously separate:
    1. The servo calibrator (Stop/Offset/Reverse per module, slider,
       TIMED RUN, Apply Correction) — extensively hands-on validated on
       this exact board across many sessions. See PROJECT_HANDOFF.md for
       the full bug history (GPIO 2 strapping pin, GPIO 9/10 permanently
       reserved for flash+PSRAM on WROVER modules, direction-reversal fix,
       real-stop-tracking fix).
    2. The mission-control architecture from smart_launcher_phase2/ (a
       PlatformIO project): ConfigManager, StateMachine, Logger,
       GNSSPositionSource, DropLogicEngine — ported here near-verbatim as
       sibling files, EXCEPT ActuatorModule, which is a real rewrite:
       (a) uses ledcAttach(pin,...)/ledcWrite(pin,...) matching the
       calibrator's proven API, not PlatformIO's older forced
       ledcSetup/ledcAttachPin API; (b) is non-blocking (startDrop()+poll()
       instead of a blocking while-loop), because this sketch's loop()
       also runs server.handleClient() for the live dashboard — a blocking
       multi-second drop would freeze the dashboard during exactly the
       moment you'd most want to watch it.

  WHY THIS TOOLCHAIN (Arduino IDE / arduino-cli) AND NOT PLATFORMIO:
    PlatformIO's official espressif32 platform bundles an older
    Arduino-ESP32 core without ledcAttach(pin,...) at all. On its first
    live drop test, a stop value (1650us) measured via this calibrator did
    NOT stop the servo through that older core/API — it kept slowly
    rotating even after a full reboot re-wrote the same stop command.
    Re-measured via this calibrator afterward: true stop was 1670us, not
    1650. The two toolchains' differing underlying LEDC drivers is the
    likely explanation. This sketch's toolchain has the actual track
    record of calibration numbers holding up under re-verification — see
    PROJECT_HANDOFF.md for the full incident writeup.

    IMPORTANT: that's "proven more trustworthy by repeated measurement,"
    not "proven correct in an absolute sense." Recalibrate each module's
    stop value fresh after any significant firmware change — don't carry
    forward old numbers from memory or from a different toolchain.

  WIRING:
    Servo A signal -> GPIO 12   Servo B signal -> GPIO 13
    Limit switch A -> GPIO 32 (COM->GND, NO->GPIO, INPUT_PULLUP)
    Limit switch B -> GPIO 33 (COM->GND, NO->GPIO, INPUT_PULLUP)
    Heartbeat/state LED -> GPIO 22 (plain 2-pin LED, NOT WS2812)
    GNSS RX/TX -> GPIO 4/5 (UART1, u-blox M8N, 9600 baud)
    Servo power     -> external 5V (UBEC), NEVER from ESP32 pins
    Servo/limit GND -> external supply GND + ESP32 GND, all tied together
    NEVER USE GPIO 2 (strapping pin) or GPIO 9/10 (flash+PSRAM on WROVER).

  USE:
    Connect to Wi-Fi AP "SmartLauncher" (no password) -> http://192.168.4.1
    Two tabs: Mission Control (default) and Calibration. A sticky
    EMERGENCY STOP button is always visible regardless of tab.

    Calibration tab: same workflow as before — set Stop(us)/Offset(us),
    tick Reverse if OPEN/CLOSE come out backwards, TIMED RUN + measured-mm
    + Apply Correction to dial in travel_ms. Press "Save Calibration" to
    persist into mission.json (LittleFS) — nothing is written to flash on
    every slider drag, only on explicit save.

    Mission Control tab: ARM (confirmed) enables GPS-radius-triggered
    autonomous drops via DropLogicEngine. Manual DROP A / DROP B require
    ARMED state and a two-tap confirm (today's incident is exactly why —
    friction only on actions that cause motion, never on ones that stop
    it). Flight log viewer + Clear.

  mission.json self-seeds from a PROGMEM default (MissionDefault.h) on
  first boot if the file doesn't exist — Arduino IDE has no bundled
  equivalent of PlatformIO's `pio run --target uploadfs`.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "Pins.h"
#include "ConfigManager.h"
#include "PositionSource.h"
#include "GNSSPositionSource.h"
#include "Logger.h"
#include "StateMachine.h"
#include "ActuatorModule.h"
#include "DropLogicEngine.h"
#include "MissionDefault.h"

// ─── Objects ────────────────────────────────────────────────────────────
const char* SSID = "SmartLauncher";
WebServer server(80);

ConfigManager      cfg;
Logger             logger;
StateMachine       sm(PIN_LED, PIN_ARM_BUTTON);
GNSSPositionSource gnssSource(1, PIN_GNSS_RX, PIN_GNSS_TX, 9600);
IPositionSource*   posSource = &gnssSource;

// Constructed with placeholder values here, rebuilt from config in setup()
ActuatorModule modA(PIN_SERVO_A, PIN_LIMIT_A, 1700, 1500, 350, 5);
ActuatorModule modB(PIN_SERVO_B, PIN_LIMIT_B, 1300, 1500, 350, 5);

DropLogicEngine* engine = nullptr;

// ─── GPS fix stability tracking ────────────────────────────────────────
static uint32_t gps_fix_start_ms = 0;
static bool     gps_stable       = false;

bool checkGPSStable(IPositionSource* src, uint8_t required_s) {
    if (!src->hasFix()) { gps_fix_start_ms = 0; gps_stable = false; return false; }
    if (gps_fix_start_ms == 0) gps_fix_start_ms = millis();
    if (!gps_stable && (millis() - gps_fix_start_ms >= (uint32_t)required_s * 1000)) {
        gps_stable = true;
        Serial.printf("[MAIN] GPS stable for %d s\n", required_s);
    }
    return gps_stable;
}

// ─── Calibration-only raw PWM jog state (Calibration tab) ─────────────
// Separate from ActuatorModule's door-drop startDrop()/poll() — raw jogs
// (OPEN/STOP/CLOSE buttons, slider, TIMED RUN) don't count doors. Same
// non-blocking shape validated all session.
bool     calRunningA = false, calRunningB = false;
bool     calTimedActive = false;
bool     calTimedIsA    = true;
uint32_t calTimedStart  = 0;
uint32_t calTimedMs     = 0;

// ─── HTML split into chunks — avoids one large PROGMEM block ──────────
static const char HTML1[] =
"<!DOCTYPE html><html lang='en'><head>"
"<meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Smart Launcher</title><style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:sans-serif;background:#0f1117;color:#e0e0e0;padding:14px;padding-bottom:70px}"
"h1{color:#4ade80;text-align:center;font-size:1.2rem;margin-bottom:2px}"
".sub{text-align:center;color:#6b7280;font-size:.72rem;margin-bottom:14px}"
".tabs{display:flex;gap:6px;max-width:720px;margin:0 auto 14px}"
".tab-btn{flex:1;padding:10px;border:none;border-radius:8px;background:#1c1f26;color:#6b7280;font-weight:700;font-size:.82rem;cursor:pointer}"
".tab-btn.active{background:#1e3a5f;color:#93c5fd}"
".tab-panel{display:none}.tab-panel.active{display:block}"
".grid{display:grid;grid-template-columns:1fr 1fr;gap:10px;max-width:720px;margin:0 auto}"
".card{background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:16px}"
".card h2{font-size:.88rem;color:#93c5fd;margin-bottom:12px}"
".badge{display:inline-block;padding:3px 10px;border-radius:20px;font-size:.68rem;font-weight:700;margin-bottom:10px}"
".s-stop{background:#374151;color:#9ca3af}.s-open{background:#166534;color:#4ade80}"
".s-close{background:#7c2d12;color:#fb923c}.s-timed{background:#1e3a5f;color:#60a5fa}"
".s-armed{background:#166534;color:#4ade80}.s-idle{background:#713f12;color:#fbbf24}"
".s-dropping{background:#581c87;color:#e9d5ff}.s-fault{background:#7f1d1d;color:#fca5a5}"
".s-boot{background:#1e3a5f;color:#93c5fd}"
".pwm-val{font-size:1.9rem;font-weight:700;color:#f9fafb;text-align:center;margin:4px 0 2px}"
".pwm-unit{font-size:.68rem;color:#6b7280;text-align:center;margin-bottom:10px}"
"input[type=range]{width:100%;accent-color:#4ade80;margin-bottom:10px}"
".btns{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;margin-bottom:8px}"
"button{padding:10px 0;border:none;border-radius:8px;font-size:.78rem;font-weight:700;cursor:pointer}"
".b-o{background:#166534;color:#4ade80}.b-s{background:#374151;color:#e5e7eb}"
".b-c{background:#7c2d12;color:#fb923c}.b-t{background:#1e3a5f;color:#60a5fa;width:100%;padding:9px 0;margin-top:4px}"
".b-danger{background:#7f1d1d;color:#fca5a5;width:100%;padding:10px 0;margin-top:4px}"
".b-confirm{background:#b45309;color:#fef3c7}"
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
".sw-row{display:flex;gap:20px;align-items:center;flex-wrap:wrap}"
".sw{display:flex;align-items:center;gap:8px;font-size:.78rem}"
".dot{width:12px;height:12px;border-radius:50%;background:#374151;transition:.2s}"
".dot.clear{background:#4ade80}.dot.hit{background:#a855f7;box-shadow:0 0 6px #a855f7}"
".up{margin-left:auto;font-size:.68rem;color:#374151}"
".log{max-width:720px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:12px;padding:12px;max-height:180px;overflow-y:auto}"
".log h2{font-size:.78rem;color:#6b7280;margin-bottom:6px}"
".le{font-size:.68rem;font-family:monospace;color:#a3e635;padding:2px 0;border-bottom:1px solid #1a1e27}"
".hint{max-width:720px;margin:10px auto 0;background:#1c1f26;border:1px solid #2a2d35;border-radius:10px;padding:12px;font-size:.7rem;color:#6b7280;line-height:1.7}"
".hint b{color:#93c5fd}"
".row2{display:flex;gap:10px;max-width:720px;margin:0 auto 10px}"
".row2 .card{flex:1}"
".kv{display:flex;justify-content:space-between;font-size:.75rem;padding:3px 0;color:#9ca3af}"
".kv b{color:#e5e7eb}"
".estop{position:fixed;bottom:0;left:0;right:0;background:#7f1d1d;color:#fecaca;border:none;"
"padding:16px;font-size:1rem;font-weight:800;letter-spacing:.05em;cursor:pointer;z-index:50}"
".estop:active{background:#991b1b}"
".toast{position:fixed;top:10px;left:50%;transform:translateX(-50%);background:#7f1d1d;color:#fecaca;"
"padding:10px 18px;border-radius:8px;font-size:.8rem;font-weight:700;z-index:100;display:none;"
"max-width:90%;text-align:center;box-shadow:0 4px 12px rgba(0,0,0,.4)}"
".toast.ok{background:#166534;color:#4ade80}"
"</style></head><body>"
"<div class='toast' id='toast'></div>"
"<h1>&#x1F680; Smart Launcher</h1>"
"<p class='sub'>ESP32 WROVER-E &nbsp;&#183;&nbsp; GPIO 12/13 servos &nbsp;&#183;&nbsp; LED 22 &nbsp;&#183;&nbsp; 192.168.4.1</p>"
"<div class='tabs'>"
"<button class='tab-btn active' id='tabBtnMission' onclick='showTab(\"mission\")'>Mission Control</button>"
"<button class='tab-btn' id='tabBtnCal' onclick='showTab(\"cal\")'>Calibration</button>"
"</div>";

// Mission Control tab
static const char HTML_MISSION[] =
"<div class='tab-panel active' id='tabMission'>"
"<div class='row2'>"
"<div class='card'><h2>STATE</h2>"
"<span id='stateBadge' class='badge s-boot'>BOOT</span>"
"<div class='kv'>Armed: <b id='armedVal'>no</b></div>"
"<div class='kv'>GPS fix: <b id='gpsVal'>--</b></div>"
"<div class='kv'>Lat/Lon: <b id='latlonVal'>--</b></div>"
"<div class='kv'>Sats/HDOP: <b id='satsVal'>--</b></div>"
"<div class='btns' style='margin-top:10px'>"
"<button class='b-o' id='armBtn' onclick='armTap()'>ARM</button>"
"<button class='b-s' onclick='doFetch(\"/disarm\")'>DISARM</button>"
"</div></div>"
"<div class='card'><h2>PROGRESS</h2>"
"<div class='kv'>Module A doors: <b id='doorsA'>0/5</b></div>"
"<div class='kv'>Module B doors: <b id='doorsB'>0/5</b></div>"
"<div class='kv'>Mission drops: <b id='dropIdx'>0/10</b></div>"
"<div class='btns' style='margin-top:10px'>"
"<button class='b-o' id='dropABtn' onclick='dropTap(\"A\")'>DROP A</button>"
"<button class='b-o' id='dropBBtn' onclick='dropTap(\"B\")'>DROP B</button>"
"</div></div></div>"
"<div class='sw-bar'><h2>LIMIT SWITCHES</h2><div class='sw-row'>"
"<div class='sw'><div id='dotA' class='dot'></div>Module A (GPIO 32)</div>"
"<div class='sw'><div id='dotB' class='dot'></div>Module B (GPIO 33)</div>"
"<div class='up' id='up'>--</div></div></div>"
"<div class='log'><h2>FLIGHT LOG <button class='b-danger' style='width:auto;padding:4px 10px;font-size:.65rem;float:right' onclick='clearLogTap()'>Clear</button></h2>"
"<pre id='flightLog' style='font-size:.68rem;color:#a3e635;white-space:pre-wrap'>(not loaded — tap Refresh)</pre>"
"<button class='b-t' onclick='refreshLog()'>Refresh Log</button></div>"
"</div>";

// Calibration tab — module cards with editable stop/offset, switches, log
static const char HTML2[] =
"<div class='tab-panel' id='tabCal'>"
"<p class='sub'>Door width: <input type='number' id='doorWidth' value='35' min='5' max='200' step='1' "
"style='width:56px;background:#111318;border:1px solid #2a2d35;border-radius:4px;color:#f9fafb;text-align:center'> mm"
" &nbsp;(used by Apply Correction below)</p>"
"<div class='grid'>"
"<div class='card'><h2>MODULE A <span style='color:#374151;font-size:.66rem'>GPIO 12</span></h2>"
"<span id='bdgA' class='badge s-stop'>STOP</span>"
"<div class='pwm-val' id='pvA'>1650</div><div class='pwm-unit'>us (last command)</div>"
"<div class='ref-row'>"
"<div><label>Stop (us)</label><input type='number' id='stopA' value='1670' min='1000' max='2000' step='5' oninput='preview(\"A\")'></div>"
"<div><label>Offset (us)</label><input type='number' id='offA' value='200' min='10' max='500' step='5' oninput='preview(\"A\")'></div>"
"</div>"
"<label style='font-size:.65rem;color:#6b7280;display:flex;align-items:center;gap:5px;margin-bottom:6px'>"
"<input type='checkbox' id='revA' onchange='preview(\"A\")'> Reverse OPEN/CLOSE (check if they came out swapped)</label>"
"<div class='preview' id='prevA'>OPEN=<b>1850</b> &nbsp; CLOSE=<b>1450</b></div>"
"<input type='range' id='slA' min='1000' max='2000' value='1670' step='5' oninput='slMove(\"A\",this.value)' onchange='slDrop(\"A\",this.value)'>"
"<div class='btns'>"
"<button class='b-o' onclick='pressOpen(\"A\")'>&#9654; OPEN</button>"
"<button class='b-s' onclick='pressStop(\"A\")'>&#9632; STOP</button>"
"<button class='b-c' onclick='pressClose(\"A\")'>&#9664; CLOSE</button></div>"
"<div class='ms-row'><input type='number' id='msA' value='580' min='50' max='9000' step='10'><label>ms</label></div>"
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
"<button class='b-t' style='max-width:720px;display:block;margin:10px auto' onclick='saveCal()'>&#128190; Save Calibration to mission.json</button>"
"<div class='log'><h2>EVENT LOG</h2><div id='log'></div></div>"
"<div class='hint'><b>Workflow:</b><br>"
"1. Set Stop (us) — start from the last saved value, re-verify with a fresh watch-test if unsure.<br>"
"2. OPEN/CLOSE are Stop &#177; Offset (mirrored for B). Tick <b>Reverse</b> if they came out swapped.<br>"
"3. Mark the rack &rarr; TIMED RUN &rarr; measure &rarr; type into <b>measured mm</b> &rarr; <b>Apply</b> &rarr; repeat until measured &#8776; door width.<br>"
"4. Press <b>Save Calibration</b> when done — nothing is written to flash before that.</div>"
"</div>"
"<button class='estop' onclick='doFetch(\"/stop\")'>&#9724; EMERGENCY STOP</button>";

// JavaScript
static const char HTML3[] =
"<script>"
"function ts(){return new Date().toLocaleTimeString();}"
"function log(m,c){const d=document.getElementById('log');if(!d)return;const e=document.createElement('div');"
"e.className='le';e.style.color=c||'#a3e635';e.textContent='['+ts()+'] '+m;d.prepend(e);"
"if(d.children.length>60)d.removeChild(d.lastChild);}"
// Visible on BOTH tabs (unlike the event log, which only lives on
// Calibration) — server-side rejections (409 armed/blocked, 400 missing
// confirm, etc.) were previously silent, which is exactly what read as
// 'calibration doesn't work while armed' / 'drops work randomly'. They
// were working as designed (blocked), just never communicated.
"function toast(msg,isError){"
"const t=document.getElementById('toast');"
"t.textContent=msg;t.className='toast'+(isError?'':' ok');t.style.display='block';"
"clearTimeout(window._toastTimer);"
"window._toastTimer=setTimeout(()=>{t.style.display='none';},3500);}"
"function showTab(t){"
"document.getElementById('tabMission').className='tab-panel'+(t==='mission'?' active':'');"
"document.getElementById('tabCal').className='tab-panel'+(t==='cal'?' active':'');"
"document.getElementById('tabBtnMission').className='tab-btn'+(t==='mission'?' active':'');"
"document.getElementById('tabBtnCal').className='tab-btn'+(t==='cal'?' active':'');"
"}"
"async function doFetch(url){"
"try{const r=await fetch(url);"
"if(!r.ok){const t=await r.text().catch(()=>'');toast('Blocked ('+r.status+'): '+t,true);log('Blocked: '+url+' -> '+r.status+' '+t,'#f87171');}"
"return r;"
"}catch(e){toast('Network error',true);log('Network error: '+url,'#f87171');return null;}}"
// --- Calibration (unchanged behaviour from the proven calibrator) ---
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
"const r=await doFetch('/cmd?mod='+mod+'&pwm='+pwm+'&stop='+stopOf(mod));"
"if(!r||!r.ok)return;"   // doFetch already toasted/logged the reason (usually: armed/dropping)
"syncSlider(mod,pwm);badge(mod,lbl,cls);log('Mod '+mod+' '+pwm+'us ('+lbl+')');}"
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
"const r=await doFetch('/timed?mod='+mod+'&ms='+ms+'&pwm='+op+'&stop='+stopOf(mod));"
"if(!r||!r.ok)return;"   // doFetch already toasted/logged why (usually: armed/dropping)
"badge(mod,'TIMED','s-timed');"
"log('Mod '+mod+' TIMED '+ms+'ms @ '+op+'us','#60a5fa');"
"const pb=document.getElementById('pb'+mod);const pf=document.getElementById('pf'+mod);"
"pb.style.display='block';pf.style.width='0%';"
"const t0=Date.now();"
"const iv=setInterval(()=>{"
"const pct=Math.min(100,(Date.now()-t0)/ms*100);pf.style.width=pct+'%';"
"if(pct>=100){clearInterval(iv);"
"setTimeout(()=>{pb.style.display='none';pf.style.width='0%';},700);"
"const s=stopOf(mod);badge(mod,'STOP','s-stop');syncSlider(mod,s);"
"log('Mod '+mod+' DONE ('+ms+'ms) measure rack now!','#4ade80');}},50);}"
"async function saveCal(){"
"const body='stopA='+stopOf('A')+'&offA='+offOf('A')+'&revA='+(isRev('A')?1:0)+'&msA='+document.getElementById('msA').value"
"+'&stopB='+stopOf('B')+'&offB='+offOf('B')+'&revB='+(isRev('B')?1:0)+'&msB='+document.getElementById('msB').value;"
"const r=await fetch('/savecal?'+body).catch(()=>null);"
"log(r&&r.ok?'Calibration saved to mission.json':'Save FAILED',r&&r.ok?'#4ade80':'#f87171');}"
// --- Mission control ---
"let armConfirming=false,dropConfirming={A:false,B:false},clearConfirming=false;"
"function armTap(){"
"const b=document.getElementById('armBtn');"
"if(!armConfirming){armConfirming=true;b.textContent='CONFIRM?';b.className='b-confirm';"
"setTimeout(()=>{if(armConfirming){armConfirming=false;b.textContent='ARM';b.className='b-o';}},5000);return;}"
"armConfirming=false;b.textContent='ARM';b.className='b-o';doFetch('/arm');}"
"function dropTap(mod){"
"const b=document.getElementById('drop'+mod+'Btn');"
"if(!dropConfirming[mod]){dropConfirming[mod]=true;b.textContent='CONFIRM DROP '+mod+'?';b.className='b-confirm';"
"setTimeout(()=>{if(dropConfirming[mod]){dropConfirming[mod]=false;b.textContent='DROP '+mod;b.className='b-o';}},5000);return;}"
"dropConfirming[mod]=false;b.textContent='DROP '+mod;b.className='b-o';"
"doFetch('/drop'+mod.toLowerCase()+'?confirm=1');}"
"function clearLogTap(){"
"if(!clearConfirming){clearConfirming=true;log('Tap Clear again within 5s to confirm','#f87171');"
"setTimeout(()=>{clearConfirming=false;},5000);return;}"
"clearConfirming=false;doFetch('/clearlog').then(()=>refreshLog());}"
"async function refreshLog(){"
"try{const r=await fetch('/log');const t=await r.text();"
"document.getElementById('flightLog').textContent=t;}catch(e){}}"
"const STATE_NAMES=['BOOT','IDLE','ARMED','DROPPING','FAULT'];"
"const STATE_CLASS=['s-boot','s-idle','s-armed','s-dropping','s-fault'];"
"async function poll(){"
"try{const j=await(await fetch('/status')).json();"
"document.getElementById('dotA').className='dot '+(j.lA?'hit':'clear');"
"document.getElementById('dotB').className='dot '+(j.lB?'hit':'clear');"
"document.getElementById('up').textContent='up '+j.up+'s';"
"const sb=document.getElementById('stateBadge');"
"sb.textContent=STATE_NAMES[j.state]||'?';sb.className='badge '+(STATE_CLASS[j.state]||'s-boot');"
"document.getElementById('armedVal').textContent=j.armed?'yes':'no';"
"document.getElementById('gpsVal').textContent=j.gpsValid?'valid':'no fix';"
"document.getElementById('latlonVal').textContent=j.gpsValid?(j.lat.toFixed(6)+', '+j.lon.toFixed(6)):'--';"
"document.getElementById('satsVal').textContent=j.gpsValid?(j.sats+' sats / hdop '+j.hdop.toFixed(1)):'--';"
"document.getElementById('doorsA').textContent=j.doorsA+'/'+j.doorCountA;"
"document.getElementById('doorsB').textContent=j.doorsB+'/'+j.doorCountB;"
"document.getElementById('dropIdx').textContent=j.dropIdx+'/'+j.dropTotal;"
"}catch(e){}}"
"setInterval(poll,350);"
"setInterval(refreshLog,5000);"
"preview('A');preview('B');refreshLog();"
"log('Connected - Mission Dashboard ready');"
"</script></body></html>";

// ─── Calibration raw-PWM write helper ──────────────────────────────────
void calWriteRaw(bool isModA, uint16_t pwm) {
    if (isModA) modA.writeRaw(pwm); else modB.writeRaw(pwm);
}

// ─── HTTP handlers — root / calibration (guarded while armed/dropping) ─
void handleRoot() {
    size_t total = strlen(HTML1) + strlen(HTML_MISSION) + strlen(HTML2) + strlen(HTML3);
    server.setContentLength(total);
    server.send(200, "text/html", "");
    server.sendContent(HTML1);
    server.sendContent(HTML_MISSION);
    server.sendContent(HTML2);
    server.sendContent(HTML3);
}

bool calibrationBlocked() {
    // A stale calibration click during a real mission is a real hazard now
    // that both feature sets share one servo driver.
    return sm.isArmed() || sm.isDropping();
}

void handleCmd() {
    if (calibrationBlocked()) { server.send(409, "text/plain", "blocked: armed/dropping"); return; }
    if (!server.hasArg("mod") || !server.hasArg("pwm")) {
        server.send(400, "text/plain", "bad"); return;
    }
    String mod = server.arg("mod");
    bool   isA = (mod == "A");
    int    pwm = constrain(server.arg("pwm").toInt(), 1000, 2000);

    if (server.hasArg("stop")) {
        int stopArg = constrain(server.arg("stop").toInt(), 1000, 2000);
        ActuatorModule& m = isA ? modA : modB;
        m.setCalibration(m.openPwm(), stopArg, m.travelMs());
    }
    uint16_t stopNow = isA ? modA.stopPwm() : modB.stopPwm();
    if (isA) calRunningA = (pwm != stopNow); else calRunningB = (pwm != stopNow);
    calWriteRaw(isA, pwm);

    Serial.printf("[CMD] %s %d us (stop=%d)\n", mod.c_str(), pwm, stopNow);
    server.send(200, "text/plain", "ok");
}

void handleTimed() {
    if (calibrationBlocked()) { server.send(409, "text/plain", "blocked: armed/dropping"); return; }
    if (!server.hasArg("mod") || !server.hasArg("ms") || !server.hasArg("pwm") || !server.hasArg("stop")) {
        server.send(400, "text/plain", "bad"); return;
    }
    String   mod     = server.arg("mod");
    bool     isA      = (mod == "A");
    uint32_t ms       = constrain((uint32_t)server.arg("ms").toInt(), 50, 9000);
    int      openPW   = constrain(server.arg("pwm").toInt(), 1000, 2000);
    int      stopArg  = constrain(server.arg("stop").toInt(), 1000, 2000);

    ActuatorModule& m = isA ? modA : modB;
    m.setCalibration(m.openPwm(), stopArg, m.travelMs());

    calTimedIsA    = isA;
    calTimedMs     = ms;
    calTimedStart  = millis();
    calTimedActive = true;
    if (isA) calRunningA = true; else calRunningB = true;
    calWriteRaw(isA, openPW);

    Serial.printf("[TIMED] %s %d us for %lu ms (stop=%d)\n", mod.c_str(), openPW, ms, stopArg);
    server.send(200, "text/plain", "ok");
}

void handleSaveCal() {
    // stopA/offA/revA/msA + stopB/offB/revB/msB
    uint16_t stopA = constrain(server.arg("stopA").toInt(), 1000, 2000);
    uint16_t offA  = constrain(server.arg("offA").toInt(), 10, 500);
    bool     revA  = server.arg("revA") == "1";
    uint32_t msA   = constrain((uint32_t)server.arg("msA").toInt(), 50, 9000);
    uint16_t stopB = constrain(server.arg("stopB").toInt(), 1000, 2000);
    uint16_t offB  = constrain(server.arg("offB").toInt(), 10, 500);
    bool     revB  = server.arg("revB") == "1";
    uint32_t msB   = constrain((uint32_t)server.arg("msB").toInt(), 50, 9000);

    uint16_t openA = computeOpenPwm(true,  stopA, offA, revA);
    uint16_t openB = computeOpenPwm(false, stopB, offB, revB);

    cfg.setModuleA(openA, stopA, msA, revA, offA);
    cfg.setModuleB(openB, stopB, msB, revB, offB);
    bool ok = cfg.save();

    if (ok) {
        modA.setCalibration(openA, stopA, msA);
        modB.setCalibration(openB, stopB, msB);
        Serial.printf("[CAL] Saved: A open=%d stop=%d ms=%lu rev=%d | B open=%d stop=%d ms=%lu rev=%d\n",
                      openA, stopA, msA, revA, openB, stopB, msB, revB);
    }
    server.send(ok ? 200 : 500, "text/plain", ok ? "ok" : "save failed");
}

// ─── HTTP handlers — mission control ───────────────────────────────────
void handleArm() {
    sm.requestArm();
    server.send(200, "text/plain", "ok");
}

void handleDisarm() {
    sm.requestDisarm();
    server.send(200, "text/plain", "ok");
}

void handleDropA() {
    if (!server.hasArg("confirm") || server.arg("confirm") != "1") {
        server.send(400, "text/plain", "confirm required"); return;
    }
    if (!sm.isArmed()) { server.send(409, "text/plain", "not armed"); return; }
    if (engine && engine->isBusy()) { server.send(409, "text/plain", "auto sequence in progress"); return; }
    bool started = modA.startDrop();
    server.send(started ? 200 : 409, "text/plain", started ? "ok" : "cannot start");
}

void handleDropB() {
    if (!server.hasArg("confirm") || server.arg("confirm") != "1") {
        server.send(400, "text/plain", "confirm required"); return;
    }
    if (!sm.isArmed()) { server.send(409, "text/plain", "not armed"); return; }
    if (engine && engine->isBusy()) { server.send(409, "text/plain", "auto sequence in progress"); return; }
    bool started = modB.startDrop();
    server.send(started ? 200 : 409, "text/plain", started ? "ok" : "cannot start");
}

void handleStop() {
    // Emergency stop — no confirm, always allowed, idempotent.
    modA.stop();
    modB.stop();
    calRunningA = calRunningB = false;
    calTimedActive = false;
    sm.requestDisarm();
    Serial.println("[CMD] *** EMERGENCY STOP ***");
    server.send(200, "text/plain", "stopped");
}

void handleLog() {
    logger.streamToHttp(server);
}

void handleClearLog() {
    logger.clearLog();
    server.send(200, "text/plain", "ok");
}

void handleStatus() {
    bool lA = (digitalRead(PIN_LIMIT_A) == LOW);
    bool lB = (digitalRead(PIN_LIMIT_B) == LOW);
    // Limit-switch auto-stop for raw calibration jogs only (door-drop
    // limit-switch handling lives inside ActuatorModule::poll()).
    if (lA && calRunningA) { modA.writeRaw(modA.stopPwm()); calRunningA = false; calTimedActive = false; }
    if (lB && calRunningB) { modB.writeRaw(modB.stopPwm()); calRunningB = false; calTimedActive = false; }

    Position p = posSource->read();
    const MissionConfig& mc = cfg.config();

    String j = "{";
    j += "\"lA\":"  + String(lA ? "true" : "false");
    j += ",\"lB\":" + String(lB ? "true" : "false");
    j += ",\"up\":" + String(millis() / 1000);
    j += ",\"state\":" + String((int)sm.state());
    j += ",\"armed\":" + String(sm.isArmed() ? "true" : "false");
    j += ",\"gpsValid\":" + String(p.valid ? "true" : "false");
    j += ",\"lat\":" + String(p.lat_deg, 6);
    j += ",\"lon\":" + String(p.lon_deg, 6);
    j += ",\"sats\":" + String(p.satellites);
    j += ",\"hdop\":" + String(p.hdop, 1);
    j += ",\"doorsA\":" + String(modA.doorsDropped());
    j += ",\"doorCountA\":" + String(modA.doorCount());
    j += ",\"doorsB\":" + String(modB.doorsDropped());
    j += ",\"doorCountB\":" + String(modB.doorCount());
    j += ",\"dropIdx\":" + String(engine ? engine->nextDropIndex() : 0);
    j += ",\"dropTotal\":" + String(mc.drop_count);
    j += "}";
    server.send(200, "application/json", j);
}

// ─── Setup — numbered checkpoints so a crash location is visible ──────
void setup() {
    Serial.begin(115200);
    delay(1000);   // matches the timing validated all session on this board
    Serial.println("\n=== Smart Launcher Mission Dashboard ===");

    Serial.println("[1] Config (LittleFS + mission.json, self-seed if missing)");
    if (!cfg.begin(MISSION_DEFAULT_JSON)) {
        Serial.println("[MAIN] Config load FAILED");
        // Continue anyway — sm.begin() below still runs so the dashboard is
        // reachable and can show a FAULT state rather than bricking silently.
    }
    cfg.printSummary();

    Serial.println("[2] Logger");
    logger.begin();

    Serial.println("[3] State machine (LED GPIO22, ARM button GPIO0 best-effort)");
    sm.begin();

    Serial.println("[4] Position source (GNSS)");
    posSource = &gnssSource;
    posSource->begin();

    Serial.println("[5] Actuators (servo GPIO12/13, limit GPIO32/33)");
    const MissionConfig& mc = cfg.config();
    modA = ActuatorModule(PIN_SERVO_A, PIN_LIMIT_A,
                          mc.module_a.servo_open_pwm_us, mc.module_a.servo_stop_pwm_us,
                          mc.module_a.travel_ms, mc.module_a.door_count);
    modB = ActuatorModule(PIN_SERVO_B, PIN_LIMIT_B,
                          mc.module_b.servo_open_pwm_us, mc.module_b.servo_stop_pwm_us,
                          mc.module_b.travel_ms, mc.module_b.door_count);
    modA.begin();
    modB.begin();

    Serial.println("[6] Drop logic engine");
    engine = new DropLogicEngine(cfg, *posSource, modA, modB, sm, logger);

    Serial.println("[7] WiFi AP");
    WiFi.mode(WIFI_AP);
    delay(100);
    WiFi.softAP(SSID, "");
    delay(500);
    Serial.printf("[8] IP: %s\n", WiFi.softAPIP().toString().c_str());

    Serial.println("[9] HTTP server");
    server.on("/",         handleRoot);
    server.on("/cmd",      handleCmd);
    server.on("/timed",    handleTimed);
    server.on("/savecal",  handleSaveCal);
    server.on("/status",   handleStatus);
    server.on("/arm",      handleArm);
    server.on("/disarm",   handleDisarm);
    server.on("/dropa",    handleDropA);
    server.on("/dropb",    handleDropB);
    server.on("/stop",     handleStop);
    server.on("/log",      handleLog);
    server.on("/clearlog", handleClearLog);
    server.begin();

    Serial.println("[10] BOOT -> IDLE");
    sm.completeBoot();

    Serial.println("[11] Ready — connect to SmartLauncher -> http://192.168.4.1");
    Serial.println("[12] setup() complete — loop starting");
}

// ─── Loop ────────────────────────────────────────────────────────────
void loop() {
    server.handleClient();

    // Door-drop polling — MUST run before engine->update() so it sees
    // isDropping() already transitioned to false on completion this tick.
    modA.poll();
    modB.poll();
    if (engine) engine->update();

    // GPS stability tracking (informational — dashboard ARM has no gate,
    // matches the original bench-test-friendly design)
    if (!gps_stable) {
        const MissionConfig& mc = cfg.config();
        checkGPSStable(posSource, mc.min_gps_fix_duration_s);
    }

    // Calibration raw-jog timed run (Calibration tab TIMED RUN button)
    if (calTimedActive && millis() - calTimedStart >= calTimedMs) {
        ActuatorModule& m = calTimedIsA ? modA : modB;
        m.writeRaw(m.stopPwm());
        if (calTimedIsA) calRunningA = false; else calRunningB = false;
        calTimedActive = false;
        Serial.printf("[TIMED] done %lu ms\n", calTimedMs);
    }

    // State machine (button debounce + LED blink pattern)
    sm.update();

    // Mission complete notification
    if (engine && engine->missionComplete() && sm.isArmed()) {
        Serial.println("[MAIN] *** MISSION COMPLETE — all drops done ***");
        sm.requestDisarm();
    }
}
