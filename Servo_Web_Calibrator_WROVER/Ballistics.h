#pragma once
#include <Arduino.h>
#include "PositionSource.h"

// ─── Impact-point prediction (Phase 4) ──────────────────────────────────────
//
// THE PROBLEM. A propagule keeps the aircraft's horizontal velocity while it
// falls. The launcher used to release AT the waypoint, so every seed landed
// downrange:
//
//     height   fall time   @2 m/s   @5 m/s   @8 m/s
//       5 m      1.01 s     2.0 m    5.0 m    8.1 m
//       8 m      1.28 s     2.6 m    6.4 m   10.2 m
//      15 m      1.75 s     3.5 m    8.7 m   14.0 m
//
// 6.4m past target at 5 m/s from 8m is TWICE the 3m radius. Chasing 10cm of
// GPS sampling accuracy while ignoring this is straining a gnat.
//
// THE APPROACH. Do not compute "where do I release". Compute where the
// propagule would land if released right now, every cycle, and let
// DropLogicEngine fire when THAT enters the target radius. This handles turns,
// speed changes and wind-on-the-aircraft for free, and it degenerates to
// exactly the old release-at-waypoint behaviour at zero velocity — so a hover
// drop is unchanged.
//
// It also changes what `radius_m` MEANS. Today it is a fudge factor absorbing
// GPS noise and sampling gaps: too small and the trigger never fires, too large
// and it fires early, because the check trips on circle ENTRY rather than at
// closest approach. Once the thing being tested against the circle is the
// predicted landing point, the radius becomes an accuracy tolerance — "drop
// when I predict I will land within X metres".
//
// WHAT IS STILL MISSING, and roughly what it costs (see PROJECT_HANDOFF.md
// PHASE 4 RESEARCH for the full error budget):
//   * t_exit          up to 2.9m — bench measurement, no drone needed
//   * wind on the falling propagule    ~1-2m, empirical
//   * propagule drag                   needs a calibration factor from real drops
//   * terrain/tide offset              height is above TAKEOFF, not the surface
// Everything here is pure arithmetic on values already on the wire, so it can
// be reasoned about and self-tested without an aircraft — `balltest` on the
// serial console runs the vectors below on the real target.
namespace Ballistics {

static const float G = 9.80665f;

// Seconds between the drop command and the propagule actually leaving the
// module. UNMEASURED — this is a bench job with a 240fps phone, the same
// technique that measured the drive window.
//
// Deliberately 0.0 rather than a guess. A guessed value biases EVERY drop in
// one direction and looks authoritative in the log; 0 leaves the error equal to
// the true t_exit and openly one-sided. Set it once it is measured, not before.
static const float T_EXIT_S = 0.0f;

// Refuse to predict outside these. Bad height or velocity data must degrade to
// release-at-waypoint, never produce a confident wrong answer far from target.
static const float MAX_HEIGHT_M  = 120.0f;
static const float MAX_SPEED_MPS = 30.0f;

// Time to fall `height_m`, released with vertical velocity vd (positive DOWN).
// Solves  h = vd*t + 0.5*g*t^2  for t:  t = (-vd + sqrt(vd^2 + 2gh)) / g
inline float fallTime(float height_m, float vd_mps) {
    if (!(height_m > 0.0f)) return 0.0f;              // also rejects NaN
    const float disc = vd_mps * vd_mps + 2.0f * G * height_m;
    if (!(disc > 0.0f)) return 0.0f;
    return (-vd_mps + sqrtf(disc)) / G;
}

// Total time from "fire now" to impact.
inline float leadTime(float height_m, float vd_mps) {
    return T_EXIT_S + fallTime(height_m, vd_mps);
}

// How far downrange the propagule travels before landing, in metres.
inline float carryDistance(const Position& p) {
    return p.horizontalSpeed() * leadTime(p.height_m, p.vd_mps);
}

// Predict where a propagule released RIGHT NOW would land.
//
// Returns false — leaving the outputs untouched — whenever the inputs are not
// good enough to predict from. The caller MUST then fall back to the aircraft
// position, which is exactly the pre-Phase-4 behaviour. Failing closed like
// this is the point: a launcher that silently predicts from garbage is worse
// than one that does not predict at all.
inline bool predictImpact(const Position& p, double& out_lat, double& out_lon,
                          float* out_lead_s = nullptr) {
    if (!p.valid || !p.vel_valid || !p.height_valid) return false;

    const float h = p.height_m;
    if (!(h >= 0.0f) || h > MAX_HEIGHT_M) return false;

    const float speed = p.horizontalSpeed();
    if (!(speed >= 0.0f) || speed > MAX_SPEED_MPS) return false;

    const float lead = leadTime(h, p.vd_mps);
    if (out_lead_s) *out_lead_s = lead;

    // Flat-earth offset. Over the tens of metres involved the error is far
    // below GPS noise, and it keeps this cheap enough to run every cycle.
    const double north_m = (double)p.vn_mps * lead;
    const double east_m  = (double)p.ve_mps * lead;

    const double cos_lat = cos(p.lat_deg * DEG_TO_RAD);
    if (fabs(cos_lat) < 1e-6) return false;           // degenerate near the poles

    out_lat = p.lat_deg + north_m / 111320.0;
    out_lon = p.lon_deg + east_m  / (111320.0 * cos_lat);
    return true;
}

}  // namespace Ballistics
