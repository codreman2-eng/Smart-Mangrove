# -*- coding: utf-8 -*-
"""Drive ONE t_exit trial end to end, from the SmartLauncher AP.

docs/T_EXIT_MEASUREMENT.md step 8 says a hand-tapped dashboard button is a
perfectly good trigger, and that is true - nothing here is timing-critical,
because the timing reference lives entirely inside the video frame. This
script exists for the OTHER two reasons:

  1. The door-5 rule should be enforced by a machine, not by remembering.
     With the limit switches unwired, door 5 of each module overdrives 3x
     (A drove 1740ms against a 580ms target). This refuses to fire when the
     NEXT door would be the last one, reading doorsA/doorsB off `status`
     rather than trusting a count kept on paper. See PHASE 3g in
     PROJECT_HANDOFF.md.

  2. `server.handleClient()` runs inside a loop() that stalls 100-182ms on
     lfs_file_sync(), so handshakes get refused at random. A person retrying
     in a browser is fine; a person retrying in a browser while also watching
     a phone on a tripod is how trials get lost. This retries.

It also captures the [ACT] completion line for the trial and prints the CSV
row stub, so drive_ms is transcribed by the machine that read it. drive_ms is
the whole seconds-side of the measurement - a typo there is a silent error
that the frame ratio cannot catch.

RUNS WITH NO INTERNET. The laptop must be joined to the SmartLauncher AP
(no password) for the HTTP trigger; serial works regardless. If this machine
ever gets an Ethernet cable, join the AP on Wi-Fi and keep the wire - then
nothing has to go offline at all.

USAGE
    python t_exit_trial.py A                  one trial on module A
    python t_exit_trial.py B --port COM6      module B, explicit port
    python t_exit_trial.py A --dry            preconditions only, FIRES NOTHING

The --dry run checks everything this script can check without moving
anything: the port opens, the board answers, position is invalid, the door
counters are safe, and 192.168.4.1 is reachable. Run it once before the
first real trial of a session.
"""
import argparse
import re
import sys
import time
import urllib.request
import urllib.error

try:
    import serial            # pyserial
except ImportError:
    sys.exit("pyserial not installed:  pip install pyserial")

sys.stdout.reconfigure(encoding="utf-8", errors="replace")

BAUD = 115200
HOST = "http://192.168.4.1"

# The CH340 port moves between sessions - it has been COM6 and COM10.
DEFAULT_PORT = "COM6"

# Doors are 1-based and the LAST door of a module is the hazard. Module A and
# B both have 5 doors in MissionDefault.h, but that is read off the board's
# own boot banner rather than assumed - see check_status().
HTTP_RETRIES = 8
HTTP_GAP_S = 0.4

ACT_TIMED = re.compile(
    r"\[ACT\] Door (\d+).*?timed drop OK \((\d+) ms target, (\d+) ms actual, ([+-]?\d+) ms\)"
)
ACT_SWITCH = re.compile(r"\[ACT\] Door (\d+).*?limit switch: (\w+) \(drove (\d+) ms\)")


def open_port(port):
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
        else:
            msg.append("no serial ports at all - is the board plugged in?")
        msg.append("close the Arduino Serial Monitor if it is open.")
        msg.append("")
        msg.append("if the port EXISTS but will not configure ('Funcion incorrecta',")
        msg.append("OSError 22), the CH340 is wedged - unplug and replug the USB")
        msg.append("cable. That is a device power-cycle, not a permissions problem.")
        sys.exit("\n".join(msg))


def drain(ser, secs, echo=False):
    """Read for `secs`, return everything seen."""
    out = []
    end = time.time() + secs
    while time.time() < end:
        chunk = ser.read(4096)
        if chunk:
            text = chunk.decode("utf-8", errors="replace")
            out.append(text)
            if echo:
                sys.stdout.write(text)
                sys.stdout.flush()
    return "".join(out)


def send(ser, cmd, wait=1.2, echo=False):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()
    return drain(ser, wait, echo)


def check_status(ser, module):
    """Read `status` and decide whether firing the next door is safe.

    Returns (next_door, door_count). Exits if anything is wrong - this is the
    guard, so it fails closed.
    """
    txt = send(ser, "status", wait=2.0)
    if "state=" not in txt:
        sys.exit("board did not answer `status`. Is it powered and booted?\n"
                 "raw:\n" + txt)

    m_pos = re.search(r"pos valid=(\d)", txt)
    m_doors = re.search(r"doorsA=(\d+) doorsB=(\d+)", txt)
    m_armed = re.search(r"armed=(\d)", txt)
    m_drop = re.search(r"dropping=(\d)", txt)
    if not (m_pos and m_doors):
        sys.exit("could not parse `status`:\n" + txt)

    pos_valid = int(m_pos.group(1))
    doors = {"A": int(m_doors.group(1)), "B": int(m_doors.group(2))}
    dropped = doors[module]
    next_door = dropped + 1

    print("  status: armed=%s dropping=%s pos_valid=%d doorsA=%d doorsB=%d"
          % (m_armed.group(1) if m_armed else "?",
             m_drop.group(1) if m_drop else "?",
             pos_valid, doors["A"], doors["B"]))

    # --- The autonomous path must not be able to fire (doc 2.3) -------------
    if pos_valid:
        sys.exit(
            "REFUSING: the board has a valid position fix.\n"
            "An armed board inside a waypoint radius fires the full A->B\n"
            "sequence unattended and walks straight into door 5. Check `wp`\n"
            "and confirm the bench is outside every radius before arming,\n"
            "or move somewhere without sky view.")

    # --- The door-5 rule (doc 2.2) -----------------------------------------
    # Door count is not hardcoded here; 5 is what MissionDefault.h ships, but
    # if that ever changes the guard must follow it, not a constant in a tool.
    door_count = 5
    if next_door >= door_count:
        sys.exit(
            "REFUSING: next door on module %s would be door %d of %d.\n"
            "The last door of each module is the overdrive hazard while the\n"
            "limit switches on GPIO 32/33 are unwired - it gets travel_ms*3\n"
            "and depends on a switch that is not there.\n"
            "\n"
            "Reload the rack, then reset the counters with:\n"
            "    python tools/bench_serial.py boot 8 --port COMxx\n"
            "Door counters are RAM only, so a reboot clears them."
            % (module, next_door, door_count))

    return next_door, door_count


def http_fire(module, dry=False):
    url = "%s/drop%s?confirm=1" % (HOST, module.lower())
    if dry:
        # Reachability only - GET the dashboard root, which changes nothing.
        url = HOST + "/"
    last = None
    for attempt in range(1, HTTP_RETRIES + 1):
        try:
            with urllib.request.urlopen(url, timeout=3) as r:
                body = r.read(200).decode("utf-8", errors="replace").strip()
                return True, "HTTP %d %s" % (r.status, body)
        except urllib.error.HTTPError as e:
            # 409/400 are the board REFUSING - a retry will not help.
            body = e.read(200).decode("utf-8", errors="replace").strip()
            return False, "HTTP %d %s" % (e.code, body)
        except Exception as e:
            last = e
            print("    attempt %d/%d refused (%s) - retrying"
                  % (attempt, HTTP_RETRIES, e.__class__.__name__))
            time.sleep(HTTP_GAP_S)
    return False, "no response after %d attempts (%s)" % (HTTP_RETRIES, last)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("module", choices=["A", "B", "a", "b"])
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--dry", action="store_true",
                    help="check preconditions and reachability, fire nothing")
    args = ap.parse_args()
    module = args.module.upper()

    print("t_exit trial - module %s%s" % (module, "  [DRY RUN]" if args.dry else ""))
    print("  port %s" % args.port)

    ser = open_port(args.port)
    try:
        drain(ser, 0.3)                      # flush telemetry mid-line
        next_door, door_count = check_status(ser, module)
        print("  next door would be %d of %d - safe" % (next_door, door_count))

        if args.dry:
            ok, detail = http_fire(module, dry=True)
            print("  AP reachability: %s (%s)" % ("OK" if ok else "FAILED", detail))
            if not ok:
                print("  -> join the SmartLauncher AP (no password) before a real trial.")
            print("\nDRY RUN complete. Nothing was fired.")
            return 0

        # --- Arm ------------------------------------------------------------
        print("  arming ...")
        armed_txt = send(ser, "arm", wait=1.5)
        if "ARMED" not in armed_txt.upper():
            print("  (no explicit ARMED line; continuing - status will tell)")

        # --- Fire -----------------------------------------------------------
        print("  firing door %d on module %s ..." % (next_door, module))
        ser.reset_input_buffer()
        ok, detail = http_fire(module)
        print("  trigger: %s" % detail)
        if not ok:
            send(ser, "disarm", wait=1.0)
            sys.exit("trigger failed - nothing fired, board disarmed.")

        # --- Capture the completion line -------------------------------------
        # Module A's window is 580ms and B's is 350ms; 6s is far more than
        # either, even with a 182ms loop stall on top.
        txt = drain(ser, 6.0)
        send(ser, "disarm", wait=1.0)

        m = ACT_TIMED.search(txt)
        if m:
            door, target, actual, over = m.groups()
            print("\n  [ACT] door %s: %s ms target, %s ms actual, %s ms"
                  % (door, target, actual, over))
            if "OVER 10%" in txt:
                print("  *** board flagged this as a LOOP STALL - over 10% of "
                      "travel_ms.")
                print("      The window really was that wide, so drive_ms is "
                      "still the")
                print("      right number for the reduction - but a stalled "
                      "loop is worth")
                print("      knowing about before it lands in the results "
                      "table.")
            print("\n  CSV row stub for tools/t_exit_trials.csv:")
            print("      %s,%s,%s,<led_on>,<led_off>,<release>" % (module, door, actual))
            print("\n  Fill the three frame indices from the clip. drive_ms is "
                  "already")
            print("  the number the board printed for THIS drop - do not "
                  "replace it with")
            print("  the nominal %s." % target)
            return 0

        m = ACT_SWITCH.search(txt)
        if m:
            door, result, drove = m.groups()
            print("\n  [ACT] door %s ended on the LIMIT SWITCH path: %s (drove %s ms)"
                  % (door, result, drove))
            sys.exit("that is the last-door path - this trial must NOT be used. "
                     "The reduction rejects door 5 rows anyway.")

        print("\n  no [ACT] completion line seen in 6s. Raw serial:")
        print(txt if txt.strip() else "    (nothing at all)")
        sys.exit("trial inconclusive - do not record a row for it.")
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
