# -*- coding: utf-8 -*-
"""Reduce the t_exit bench trials to the constant Ballistics.h needs.

WHAT t_exit IS. Seconds between the drop command and the propagule actually
leaving the module. Ballistics.h::leadTime() adds it to the fall time, so it is
carry the aircraft's own velocity applies to the seed BEFORE it is even in the
air. It is 0.0 in the firmware today and worth up to 2.9m of along-track error
at 5 m/s. See docs/T_EXIT_MEASUREMENT.md for how the clips are shot.

THE MEASUREMENT IS A RATIO, NOT A FRAME RATE. The announce LED (GPIO 14) is lit
for exactly the servo drive window and nothing else - ActuatorModule::_writePWM
sets it from `pulse_us != stop_pwm`, with no stretching and no speed multiplier,
which is the whole reason SIM mode was built that way. The board also prints the
true width of that window in milliseconds on the [ACT] completion line, measured
immediately after the stop write. So:

    t_exit = drive_ms * (release - led_on) / (led_off - led_on)

The phone's frame rate cancels out. That matters more than it looks: 240fps
phone clips are routinely 239.76fps, and are just as routinely exported into a
30fps container at 8x slow, so any answer that trusted the metadata would be
wrong by a factor of 8 and still look plausible. Here the video supplies only a
ratio of frame counts and the already-verified drive window supplies the
seconds.

    python tools/t_exit_reduce.py [trials.csv]
    python tools/t_exit_reduce.py --selftest
"""
import sys, math, os

DEFAULT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           "t_exit_trials.csv")

# Speeds the error budget in PROJECT_HANDOFF.md PHASE 4 RESEARCH is quoted at.
SPEEDS = (2.0, 5.0, 8.0)

# Module A's nominal drive window, from MissionDefault.h. Used only to work out
# how far behind the trigger module B's seed leaves - B does not start until A's
# window has closed (DropLogicEngine WAITING_A -> B), so B carries A's whole
# window on top of its own t_exit.
TRAVEL_MS_A = 580

# Above this, a single constant is doing more harm than good and the spread
# should be understood before the number is trusted. 0.05s at 5 m/s = 0.25m,
# which is inside the 0.5-1m of GPS position error already accepted.
SD_WARN_S = 0.05


class Trial(object):
    def __init__(self, module, door, drive_ms, led_on, led_off, release, lineno):
        self.module   = module
        self.door     = door
        self.drive_ms = drive_ms
        self.led_on   = led_on
        self.led_off  = led_off
        self.release  = release
        self.lineno   = lineno

    @property
    def window_frames(self):
        return self.led_off - self.led_on

    @property
    def exit_frames(self):
        return self.release - self.led_on

    @property
    def t_exit_s(self):
        return (self.drive_ms / 1000.0) * self.exit_frames / self.window_frames

    @property
    def fraction(self):
        """Where in the drive window the seed left, 0..1."""
        return float(self.exit_frames) / self.window_frames

    @property
    def implied_fps(self):
        """Sanity number only - never used in the answer."""
        return self.window_frames / (self.drive_ms / 1000.0)


def parse(path):
    """Read the trials file, rejecting rows that cannot mean anything.

    Every gate here is a mistake that would otherwise produce a confident wrong
    constant rather than an error, which is exactly the failure mode
    Ballistics.h refuses to have (it ships 0.0 instead of a guess).
    """
    trials, errs = [], []
    try:
        fh = open(path, "r")
    except IOError as e:
        sys.exit("cannot read %s (%s)" % (path, e.__class__.__name__))
    with fh:
        for n, raw in enumerate(fh, 1):
            line = raw.split("#", 1)[0].strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) != 6:
                errs.append("line %d: expected 6 fields, got %d" % (n, len(parts)))
                continue
            try:
                mod = parts[0].upper()
                door = int(parts[1])
                drive_ms = float(parts[2])
                led_on, led_off, release = (int(parts[3]), int(parts[4]), int(parts[5]))
            except ValueError:
                errs.append("line %d: non-numeric field" % n)
                continue

            if mod not in ("A", "B"):
                errs.append("line %d: module must be A or B, got %r" % (n, mod))
                continue
            if door >= 5:
                errs.append("line %d: door %d - the LAST door overdrives 3x with the "
                            "limit switches unwired. This row should not exist."
                            % (n, door))
                continue
            if led_off <= led_on:
                errs.append("line %d: led_off must be after led_on" % n)
                continue
            if drive_ms <= 0:
                errs.append("line %d: drive_ms must be positive" % n)
                continue

            t = Trial(mod, door, drive_ms, led_on, led_off, release, n)

            # The seed cannot leave before the servo starts, and if it leaves
            # after the window closed it did not fall out of a moving cover -
            # it was shaken loose, or the cover jammed and freed late. Either
            # way the row is not measuring t_exit.
            if t.exit_frames < 0:
                errs.append("line %d: release is BEFORE led_on - seed left before "
                            "the servo moved? Re-check the frame indices." % n)
                continue
            if t.exit_frames > t.window_frames:
                errs.append("line %d: release is AFTER the window closed "
                            "(t_exit %.3fs > drive %.3fs). Cover jam or a "
                            "hung-up seed - discard, do not average it in."
                            % (n, t.t_exit_s, drive_ms / 1000.0))
                continue
            trials.append(t)
    return trials, errs


def stats(values):
    n = len(values)
    mean = sum(values) / n
    if n > 1:
        sd = math.sqrt(sum((v - mean) ** 2 for v in values) / (n - 1))
    else:
        sd = float("nan")
    return mean, sd, min(values), max(values)


def report(trials):
    by_mod = {}
    for t in trials:
        by_mod.setdefault(t.module, []).append(t)

    print("PER-TRIAL")
    print("  %-3s %-4s %9s %7s %7s %8s %9s %8s"
          % ("mod", "door", "drive_ms", "win_fr", "exit_fr", "frac", "t_exit_s", "~fps"))
    for t in sorted(trials, key=lambda x: (x.module, x.lineno)):
        print("  %-3s %-4d %9.0f %7d %7d %8.3f %9.4f %8.1f"
              % (t.module, t.door, t.drive_ms, t.window_frames, t.exit_frames,
                 t.fraction, t.t_exit_s, t.implied_fps))

    fps_all = [t.implied_fps for t in trials]
    if max(fps_all) / min(fps_all) > 1.15:
        print("")
        print("  *** the implied frame rates disagree by more than 15%. The clips")
        print("      were not all shot at the same rate, or a frame index is wrong.")

    means = {}
    print("")
    print("PER MODULE")
    for mod in sorted(by_mod):
        ts = [t.t_exit_s for t in by_mod[mod]]
        mean, sd, lo, hi = stats(ts)
        means[mod] = (mean, sd, len(ts))
        print("")
        print("  Module %s  (n=%d)" % (mod, len(ts)))
        print("    mean t_exit   %.4f s" % mean)
        print("    sd            %s" % ("n/a - one trial" if len(ts) < 2 else "%.4f s" % sd))
        print("    min / max     %.4f / %.4f s" % (lo, hi))
        print("    mean fraction of the drive window: %.1f%%"
              % (100.0 * sum(t.fraction for t in by_mod[mod]) / len(ts)))
        if len(ts) < 5:
            print("    *** only %d trials. Take at least 5 before setting a constant." % len(ts))
        elif not math.isnan(sd) and sd > SD_WARN_S:
            print("    *** sd %.3fs is above the %.3fs guide - at 5 m/s that is "
                  "+/-%.2fm of" % (sd, SD_WARN_S, 5.0 * sd))
            print("        residual spread a single constant cannot remove.")

        print("    what this is worth, per module:")
        for sp in SPEEDS:
            resid = "" if math.isnan(sd) else "   +/- %.2f m" % (sp * sd)
            print("      @%.0f m/s   corrects %5.2f m%s" % (sp, sp * mean, resid))

    return means


def verdict(means):
    print("")
    print("WHAT TO DO WITH IT")
    if "A" not in means:
        print("  No module A trials. A is the module the mission fires first and")
        print("  the one T_EXIT_S most directly describes - measure it.")
        return

    a_mean, a_sd, a_n = means["A"]
    print("  Set, in BOTH Servo_Web_Calibrator_WROVER/Ballistics.h and")
    print("  tools/test_ballistics.py:")
    print("")
    print("      static const float T_EXIT_S = %.3ff;   // module A, n=%d, measured"
          % (a_mean, a_n))
    print("")

    if "B" in means:
        b_mean, b_sd, b_n = means["B"]
        gap = abs(b_mean - a_mean)
        pooled = 0.0
        if not (math.isnan(a_sd) or math.isnan(b_sd)):
            pooled = math.sqrt(a_sd ** 2 + b_sd ** 2)
        print("  Module B measures %.4f s, %.4f s away from A." % (b_mean, gap))
        if pooled and gap > 2.0 * pooled:
            print("  That gap is larger than the trial noise (2 sd = %.4f s), so the"
                  % (2.0 * pooled))
            print("  two modules genuinely differ and ONE global constant cannot")
            print("  describe both. Ballistics.h has no per-module concept today.")
        else:
            print("  That is inside the trial noise, so one constant covers both.")

        # The structural point, true regardless of the numbers.
        b_total = TRAVEL_MS_A / 1000.0 + b_mean
        print("")
        print("  SEPARATELY, AND INDEPENDENT OF ANY OF THE ABOVE:")
        print("  DropLogicEngine does not start module B until module A's window")
        print("  has CLOSED. So B's seed leaves %.0f + %.0f = %.0f ms after the"
              % (TRAVEL_MS_A, b_mean * 1000, b_total * 1000))
        print("  trigger, against A's %.0f ms - both from one predicted impact"
              % (a_mean * 1000))
        print("  point. At 5 m/s the pair lands %.1f m apart no matter what"
              % (5.0 * (b_total - a_mean)))
        print("  T_EXIT_S is set to. That is a sequencing question, not a")
        print("  constant question, and this measurement is what makes it visible.")

    print("")
    print("  THEN, BEFORE TRUSTING ANYTHING: tools/test_ballistics.py checks the")
    print("  PROJECT_HANDOFF.md fall-carry table through carry(), which now")
    print("  includes t_exit and will FAIL against a table that is pure fall.")
    print("  Split it - keep the table checked against fall time alone and add")
    print("  separate cases for the total. Do NOT widen the tolerance until it")
    print("  passes; that would delete the only check on this constant.")
    print("  Re-run `balltest` on the target afterwards: fallTime is unchanged,")
    print("  but every carry and dLat expectation moves.")


def selftest():
    """Synthetic trials with an answer known in closed form.

    Built so the ratio really is doing the work: the two modules are filmed at
    different frame rates (240 and 120) and different window widths, and both
    must still reduce to the t_exit they were constructed from.
    """
    print("SELFTEST - synthetic frames, known answer")
    print("")
    cases = [
        # (module, drive_ms, fps, true t_exit)
        ("A", 580, 240.0, 0.300),
        ("A", 580, 240.0, 0.300),
        ("B", 350, 120.0, 0.200),
    ]
    fails = []
    for i, (mod, drive_ms, fps, want) in enumerate(cases):
        led_on = 1000 + i          # deliberately not starting at 0
        led_off = led_on + int(round(fps * drive_ms / 1000.0))
        release = led_on + int(round(fps * want))
        t = Trial(mod, 1, drive_ms, led_on, led_off, release, i)
        got = t.t_exit_s
        # One frame of quantisation at 120fps is 8.3ms; allow two.
        ok = abs(got - want) <= 2.0 / fps
        print("  %s drive=%dms fps=%.0f  t_exit %.4f  expect %.4f  %s"
              % (mod, drive_ms, fps, got, want, "ok" if ok else "*** FAIL ***"))
        if not ok:
            fails.append(i)
        implied = t.implied_fps
        if abs(implied - fps) > 1.0:
            print("      implied fps %.1f != %.0f  *** FAIL ***" % (implied, fps))
            fails.append(i)

    print("")
    print("  the frame rate cancels: same t_exit from 240fps and 120fps clips: %s"
          % ("ok" if not fails else "NO"))
    print("")
    print("%s" % ("SELFTEST PASSED" if not fails else "SELFTEST FAILED"))
    return 1 if fails else 0


def main():
    args = list(sys.argv[1:])
    if "--selftest" in args:
        return selftest()

    path = args[0] if args else DEFAULT_CSV
    trials, errs = parse(path)

    if errs:
        print("REJECTED ROWS")
        for e in errs:
            print("  " + e)
        print("")

    if not trials:
        print("No usable trials in %s." % path)
        print("Shoot the clips first - docs/T_EXIT_MEASUREMENT.md has the rig,")
        print("the safety preconditions and the frame-reading method.")
        print("Check the arithmetic here meanwhile with:  --selftest")
        return 1

    print("t_exit from %d trial(s) in %s" % (len(trials), os.path.basename(path)))
    print("")
    means = report(trials)
    verdict(means)
    return 0


if __name__ == "__main__":
    sys.exit(main())
