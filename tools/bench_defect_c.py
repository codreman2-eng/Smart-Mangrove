# -*- coding: utf-8 -*-
"""SIM_MODE_VALIDATION_PLAN.md step 12 - defect (c).

(c): the GPS trigger ignored startDrop()'s false return. With module A EMPTY
the WAITING_A branch fired anyway, recorded a drop from a stale _lastResult,
started module B, and logged a full SUCCESS with ZERO motion.

Needs more waypoints than doors. 10 waypoints / 5 doors: waypoints 1-5 empty
both modules, waypoint 6 is the defect case.

RESULT 2026-09-01 (COM10, SIM build): none of (c)'s symptoms occur. WP1-WP5
fire cleanly (+0ms), the run ends at WP5 with "MISSION COMPLETE", and WP6
produces NO [DLE] output at all - no evaluation, no drop, no module B start.

Note what that does NOT prove. missionComplete() is checked at
DropLogicEngine.cpp:73, BEFORE any waypoint evaluation, and launcherEmpty() is
`_modA.isEmpty() || _modB.isEmpty()`. So the instant either module empties the
run ends, and the GPS-trigger guard at DropLogicEngine.cpp:123 is unreachable
from this path by construction - no door-count configuration reaches it. This
script proves the SYMPTOMS are unreachable, not that the inner guard works.

It also surfaced a latent hazard: door 5 of both modules ends in
`limit switch: TIMEOUT`, driving 3x travel_ms (A 1740ms vs 580ms target,
B 1050ms vs 350ms) because the switches on GPIO 32/33 are unwired on the
bench. Harmless in SIM; with servos connected the last door of every module
would overdrive by three door pitches. Wire the limit switches before
SIM_MODE goes to 0.
"""
import sys, time
sys.stdout.reconfigure(encoding="utf-8", errors="replace")
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM10"
LON  = "-79.5322405"
LATS = ["9.0222514", "9.0224514", "9.0226514", "9.0228514",
        "9.0230514", "9.0232514"]          # WP1..WP6

ser = serial.Serial(PORT, 115200, timeout=0.02)
T0 = time.time()

def pump(sec):
    end, buf = time.time() + sec, b""
    while time.time() < end:
        b = ser.read(512)
        if not b:
            continue
        buf += b
        while b"\n" in buf:
            one, buf = buf.split(b"\n", 1)
            t = one.decode("utf-8", "replace").rstrip("\r")
            if t and "[OSDK]" not in t and "no frames at all" not in t:
                print("[%+7.3fs] %s" % (time.time() - T0, t))

def send(c):
    print("\n[%+7.3fs] >>> %s" % (time.time() - T0, c))
    ser.write((c + "\n").encode()); ser.flush()

# Reset: door counters and _next_drop_idx are RAM-only, so the run must
# start from a known-clean board or "doorsA=5" proves nothing.
ser.setDTR(False); ser.setRTS(True); time.sleep(0.15)
ser.reset_input_buffer(); ser.setRTS(False)
print("=== reset, waiting for boot ===")
pump(5.0)

send("arm"); pump(1.0)

for i, lat in enumerate(LATS, start=1):
    tag = "  <<< WP%d - DEFECT (c) CASE (A should be EMPTY)" % i if i == 6 else ""
    print("\n" + "=" * 66)
    print("=== WAYPOINT %d%s" % (i, tag))
    print("=" * 66)
    send("arm"); pump(0.4)          # re-arm if a prior drop auto-disarmed
    send("simpos %s %s" % (lat, LON))
    pump(3.0)
    send("status"); pump(1.2)

print("\n" + "=" * 66)
print("=== FINAL")
print("=" * 66)
send("status"); pump(1.5)
ser.close()
