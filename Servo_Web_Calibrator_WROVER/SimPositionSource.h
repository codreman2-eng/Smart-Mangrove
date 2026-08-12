#pragma once
#include "Pins.h"

#if SIM_MODE

#include "PositionSource.h"

// Fake position source — SIM builds only.
//
// Lets DropLogicEngine's Haversine radius check and its sequential A->B
// firing be validated indoors with no GPS fix and no sky view. Positions
// are injected over HTTP (`/simpos?lat=..&lon=..`); until one is injected
// it reports no fix, so the engine sits idle exactly as it would with an
// unlocked M8N.
//
// This whole file is compiled out when SIM_MODE is 0 — a flight build has
// no position-injection surface at all, by construction rather than by
// convention.
class SimPositionSource : public IPositionSource {
public:
    void begin() override {
        Serial.println("[SIMPOS] Fake position source active — no fix until "
                       "injected via /simpos?lat=..&lon=..");
    }

    void update() override {}                       // nothing to parse
    Position read()   override { return _p; }
    bool     hasFix() override { return _p.valid; }

    void inject(double lat, double lon) {
        _p.lat_deg    = lat;
        _p.lon_deg    = lon;
        _p.alt_m      = 8.0f;
        _p.hdop       = 0.9f;    // plausible good-fix values so the dashboard
        _p.satellites = 12;      // renders the same fields it would in flight
        _p.valid      = true;
        Serial.printf("[SIMPOS] Injected %.6f, %.6f\n", lat, lon);
    }

    void clear() {
        _p = Position();
        Serial.println("[SIMPOS] Cleared — reporting no fix");
    }

private:
    Position _p;   // .valid defaults false
};

#endif  // SIM_MODE
