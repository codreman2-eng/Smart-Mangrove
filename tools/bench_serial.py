# -*- coding: utf-8 -*-
"""Serial bench driver for the Smart Launcher (ESP32 WROVER-E).

Drives the dashboard's serial command interface from a PC, so bench tests can
be run and TIMED precisely without a browser, without the SmartLauncher AP,
and without a human hitting a key inside a 580ms window.

That last point is the reason this exists. SIM_MODE_VALIDATION_PLAN.md step 10
requires the emergency stop to land INSIDE module A's drive window while the
engine is in WAITING_A. A person typing into a serial monitor cannot reliably
hit that, and an ambiguous miss is indistinguishable from "the defect did not
reproduce" - which is exactly the wrong answer to get from a safety test.

It found real defects on 2026-08-25: (a) e-stop starting module B, (b) e-stop
failing to leave DROPPING, and (e) an aborted drop logging as a SUCCESS. It
then caught the first fix landing on only one of the two e-stop code paths.

USAGE
    python bench_serial.py boot [secs]        reset the board, capture boot
    python bench_serial.py listen [secs]      passive listen
    python bench_serial.py cmd <secs> C [C..] send commands ("wait:N" to pause)
    python bench_serial.py drop               happy-path A->B drop
    python bench_serial.py estop [offset_s]   plan steps 9+10, timed e-stop

Board commands available: status | clearlog | arm | disarm | stop | wp |
                          simpos <lat> <lon> | simclear

NOTE: close the Arduino Serial Monitor first or the port is held.
"""
import sys, time

try:
    import serial            # pyserial
except ImportError:
    sys.exit("pyserial not installed:  pip install pyserial")

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

# The CH340 port moves between sessions - it has been COM6 and COM10. Check
# Device Manager (or Arduino IDE Tools -> Port) rather than trusting this.
PORT = "COM10"
BAUD = 115200

# The waypoint saved on the device as of 2026-08-25. `wp` prints the current
# list if this has changed.
WP_LAT, WP_LON = "9.022251", "-79.532240"


def open_port():
    """Open PORT, or fail with the list of ports that DO exist.

    The CH340 moves between sessions on this machine, so "could not open
    COM10" is the single most likely way this script fails. Printing the real
    candidates turns a stack trace into an answer. Override with:
        python bench_serial.py <mode> --port COMxx
    """
    port = PORT
    if "--port" in sys.argv:
        port = sys.argv[sys.argv.index("--port") + 1]
    try:
        return serial.Serial(port, BAUD, timeout=0.02)
    except serial.SerialException as e:
        try:
            from serial.tools import list_ports
            found = list(list_ports.comports())
        except Exception:
            found = []
        msg = ["could not open %s (%s)" % (port, e.__class__.__name__)]
        if found:
            msg.append("ports present right now:")
            for p in found:
                msg.append("    %-8s %s" % (p.device, p.description))
            msg.append("the launcher is the CH340; the Cube Orange+ is not this board.")
            msg.append("re-run with:  --port COMxx")
        else:
            msg.append("no serial ports at all - is the board plugged in?")
        msg.append("also check nothing else holds it (Arduino Serial Monitor, "
                   "Mission Planner).")
        sys.exit("\n".join(msg))


def reset(ser):
    """ESP32 auto-reset: EN <- RTS, IO0 <- DTR.

    IO0 is held HIGH so the chip runs the application instead of dropping into
    the ROM bootloader.
    """
    ser.setDTR(False)          # IO0 HIGH
    ser.setRTS(True)           # EN LOW  -> held in reset
    time.sleep(0.15)
    ser.reset_input_buffer()
    ser.setRTS(False)          # EN HIGH -> run


def pump(ser, seconds, t0=None):
    """Read and print for `seconds`, timestamping each line if t0 is given."""
    end = time.time() + seconds
    line = b""
    while time.time() < end:
        b = ser.read(512)
        if not b:
            continue
        line += b
        while b"\n" in line:
            one, line = line.split(b"\n", 1)
            txt = one.decode("utf-8", "replace").rstrip("\r")
            if not txt:
                continue
            if t0 is None:
                print(txt)
            else:
                print("[%+7.3fs] %s" % (time.time() - t0, txt))


def send(ser, c):
    ser.write((c + "\n").encode())
    ser.flush()


def run_schedule(ser, schedule, total):
    """Send commands at exact offsets from t0, capturing continuously.

    Timestamps are relative to the FIRST scheduled command, so where an event
    landed relative to the drive window is evidence rather than assumption.
    """
    t0 = time.time()
    i = 0
    line = b""
    while time.time() - t0 < total:
        while i < len(schedule) and (time.time() - t0) >= schedule[i][0]:
            print("\n[%+7.3fs] >>> %s" % (time.time() - t0, schedule[i][1]))
            send(ser, schedule[i][1])
            i += 1
        b = ser.read(512)
        if not b:
            continue
        line += b
        while b"\n" in line:
            one, line = line.split(b"\n", 1)
            txt = one.decode("utf-8", "replace").rstrip("\r")
            if txt:
                print("[%+7.3fs] %s" % (time.time() - t0, txt))


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "listen"
    ser = open_port()
    try:
        if mode == "boot":
            reset(ser)
            pump(ser, float(sys.argv[2]) if len(sys.argv) > 2 else 8)

        elif mode == "listen":
            pump(ser, float(sys.argv[2]) if len(sys.argv) > 2 else 20)

        elif mode == "cmd":
            secs = float(sys.argv[2])
            ser.reset_input_buffer()
            for c in sys.argv[3:]:
                if c.startswith("wait:"):
                    pump(ser, float(c[5:]))
                    continue
                print("\n>>> %s" % c)
                send(ser, c)
                pump(ser, 1.0)
            pump(ser, secs)

        elif mode == "drop":
            # Happy path. Reboot first so door counters and _next_drop_idx
            # start clean - they are RAM only.
            reset(ser); time.sleep(4.0); ser.reset_input_buffer()
            send(ser, "arm"); time.sleep(0.6)
            run_schedule(ser, [(0.0, "simpos %s %s" % (WP_LAT, WP_LON)),
                               (4.0, "status")], 6.0)

        elif mode == "estop":
            # Plan steps 9 + 10. The stop must land inside module A's 580ms
            # window, with the engine in WAITING_A.
            offset = float(sys.argv[2]) if len(sys.argv) > 2 else 0.250
            reset(ser); time.sleep(4.0); ser.reset_input_buffer()
            send(ser, "arm"); time.sleep(0.6); ser.reset_input_buffer()
            run_schedule(ser, [(0.0,        "simpos %s %s" % (WP_LAT, WP_LON)),
                               (offset,     "stop"),
                               (offset + .08, "status"),   # before B could finish
                               (5.0,        "status")], 8.0)
            print("\nPASS (post-fix) means ALL of:")
            print("  * no '[DLE] Module B' and no second '[ACT] Dropping door'")
            print("  * '[SM] *** EMERGENCY STOP -> IDLE (disarmed) ***'")
            print("  * status -> state=1 armed=0 dropping=0, doorsB=0")
            print("  * the [DROP] row says ABORTED, and dropIdx is unchanged")
        else:
            sys.exit(__doc__)
    finally:
        ser.close()


main()
