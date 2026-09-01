# -*- coding: utf-8 -*-
"""Reference values for the impact predictor in Servo_Web_Calibrator_WROVER/Ballistics.h

The firmware's own `balltest` serial command runs the SAME vectors on the real
ESP32. This file is the independent second implementation to check it against:
if the two agree, a transcription error in the C++ has to have been made twice,
in two languages, in the same direction.

It also re-derives the fall-carry table in PROJECT_HANDOFF.md PHASE 4 RESEARCH,
so the 6.4m figure that motivates the whole exercise is checked rather than
quoted.

    python tools/test_ballistics.py

Any assertion failure means the firmware constants and this file have diverged
- most likely G, T_EXIT_S, or the metres-per-degree constant.
"""
import math, sys

G          = 9.80665
T_EXIT_S   = 0.0        # UNMEASURED - keep in step with Ballistics.h
M_PER_DEG  = 111320.0


def fall_time(h, vd=0.0):
    """Time to fall h metres, released with vertical velocity vd (positive DOWN)."""
    if not h > 0.0:
        return 0.0
    disc = vd * vd + 2.0 * G * h
    if not disc > 0.0:
        return 0.0
    return (-vd + math.sqrt(disc)) / G


def lead_time(h, vd=0.0):
    return T_EXIT_S + fall_time(h, vd)


def carry(speed, h, vd=0.0):
    return speed * lead_time(h, vd)


def impact_offset(lat, vn, ve, h, vd=0.0):
    """Returns (dlat_deg, dlon_deg) of the predicted impact point."""
    lead = lead_time(h, vd)
    return (vn * lead / M_PER_DEG,
            ve * lead / (M_PER_DEG * math.cos(math.radians(lat))))


def close(a, b, tol):
    return abs(a - b) <= tol


def main():
    fails = []

    def check(label, got, want, tol):
        ok = close(got, want, tol)
        print("  %-34s %15.9f   expect %15.9f  %s"
              % (label, got, want, "ok" if ok else "*** FAIL ***"))
        if not ok:
            fails.append(label)

    print("fallTime(h, vd=0)")
    check("h= 5m", fall_time(5.0),  1.00980, 1e-5)
    check("h= 8m", fall_time(8.0),  1.27732, 1e-5)
    check("h=15m", fall_time(15.0), 1.74904, 1e-5)

    print("\nvertical velocity at h=8m")
    check("vd=+1 (descending)", fall_time(8.0,  1.0), 1.17942, 1e-5)
    check("vd=-1 (climbing)",   fall_time(8.0, -1.0), 1.38336, 1e-5)

    # A descent must shorten the fall and a climb must lengthen it. If the DJI
    # vertical sign convention is ever flipped in OsdkPositionSource, this is
    # the relationship that inverts - and the drops move by about a metre.
    assert fall_time(8.0, 1.0) < fall_time(8.0) < fall_time(8.0, -1.0)
    print("  ordering climb > level > descend: ok")

    print("\nPROJECT_HANDOFF.md fall-carry table (PHASE 4 RESEARCH)")
    # NOTE the 5 m/s cell for h=5m: PROJECT_HANDOFF.md originally printed 5.1m.
    # 5 * 1.00981 = 5.049, which rounds to 5.0. Corrected in the handoff and in
    # Ballistics.h after this test caught it - the 2 and 8 m/s cells were right,
    # so it was a single rounding slip, not a formula error.
    table = {5.0:  (1.01, 2.0, 5.0, 8.1),
             8.0:  (1.28, 2.6, 6.4, 10.2),
             15.0: (1.75, 3.5, 8.7, 14.0)}
    for h, (t, c2, c5, c8) in table.items():
        check("h=%-5g fall time" % h, fall_time(h), t, 0.005)
        check("h=%-5g carry @2 m/s" % h, carry(2.0, h), c2, 0.05)
        check("h=%-5g carry @5 m/s" % h, carry(5.0, h), c5, 0.05)
        check("h=%-5g carry @8 m/s" % h, carry(8.0, h), c8, 0.05)

    print("\nimpact offset at lat 9.0222514, h=8m")
    dlat, _ = impact_offset(9.0222514, 5.0, 0.0, 8.0)
    check("5 m/s north  dLat (deg)", dlat, 0.000057372, 1e-8)
    check("5 m/s north  metres",     dlat * M_PER_DEG, 6.38660, 1e-4)
    _, dlon = impact_offset(9.0222514, 0.0, 5.0, 8.0)
    check("5 m/s east   metres",
          dlon * M_PER_DEG * math.cos(math.radians(9.0222514)), 6.38660, 1e-4)

    # The one that matters most: at zero velocity the predictor must reduce to
    # release-at-waypoint exactly, or hover drops have silently changed.
    dlat0, dlon0 = impact_offset(9.0222514, 0.0, 0.0, 8.0)
    assert dlat0 == 0.0 and dlon0 == 0.0
    print("\n  v=0 degenerates to the aircraft position exactly: ok")

    print("\n%s" % ("ALL CHECKS PASSED" if not fails
                    else "FAILURES: %s" % ", ".join(fails)))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
