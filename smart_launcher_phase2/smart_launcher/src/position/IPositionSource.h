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
