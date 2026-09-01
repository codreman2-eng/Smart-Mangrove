#pragma once
#include <Arduino.h>

// Unified position struct used by all modules
struct Position {
    double  lat_deg  = 0.0;
    double  lon_deg  = 0.0;
    float   alt_m    = 0.0f;
    float   hdop     = 99.0f;   // horizontal dilution of precision (lower = better)
    uint8_t satellites = 0;
    bool    valid    = false;

    // ── Ballistics inputs (Phase 4) ──────────────────────────────────────
    // A propagule keeps the aircraft's horizontal velocity while it falls, so
    // releasing AT the waypoint lands it downrange - 6.4m at 5 m/s from 8m,
    // twice the target radius. Predicting the impact point needs height above
    // the drop surface and a velocity vector; both are already on the A3's
    // wire and were simply not being read.
    //
    // ONLY OsdkPositionSource fills these. GNSS and SIM leave them invalid and
    // the predictor degrades cleanly to today's release-at-waypoint behaviour,
    // which is also exactly what it does at zero velocity.
    float   height_m     = 0.0f;   // height above TAKEOFF point, NOT above ground
    bool    height_valid = false;

    // Ground-frame velocity in NED. vd is positive DOWNWARD (descending).
    // See OsdkPositionSource.cpp for the sign convention and why it is not
    // yet verified against a real climb.
    float   vn_mps    = 0.0f;
    float   ve_mps    = 0.0f;
    float   vd_mps    = 0.0f;
    bool    vel_valid = false;

    // Horizontal speed, the quantity that actually drives the fall carry.
    float horizontalSpeed() const { return sqrtf(vn_mps * vn_mps + ve_mps * ve_mps); }
};

// Pure abstract interface — both GNSS and MAVLink implement this
class IPositionSource {
public:
    virtual ~IPositionSource() {}
    virtual void     begin()  = 0;
    virtual void     update() = 0;      // call every loop iteration
    virtual Position read()   = 0;
    virtual bool     hasFix() = 0;
};

// Haversine distance between two GPS coordinates — returns metres
inline float haversineDistance(double lat1, double lon1,
                               double lat2, double lon2) {
    const float R = 6371000.0f;   // Earth radius in metres
    float dLat = radians(lat2 - lat1);
    float dLon = radians(lon2 - lon1);
    float a = sinf(dLat / 2) * sinf(dLat / 2)
            + cosf(radians(lat1)) * cosf(radians(lat2))
            * sinf(dLon / 2) * sinf(dLon / 2);
    return R * 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
}
