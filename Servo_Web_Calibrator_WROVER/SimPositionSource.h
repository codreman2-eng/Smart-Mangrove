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
        // MUST forward. Every other method delegates to the fallback, and
        // begin() originally did not — so UART2 was never initialised and
        // the OSDK link never received a byte, while the boot log still
        // looked healthy. The missing [OSDK]/[GNSS] init lines were the only
        // symptom. If a method is added to this class, delegate it here too.
        if (_real) _real->begin();
    }

    // Falls through to a real source until a position is injected. That is
    // what makes the walk test possible: a SIM build (LEDs, no servo output)
    // driven by REAL aircraft position, so waypoint triggering can be proven
    // by carrying the M600 towards a waypoint and watching GPIO 14 — no
    // flight, nothing that can move. Injecting overrides it for indoor work;
    // /simpos?clear=1 hands control back.
    void setFallback(IPositionSource* real) { _real = real; }

    void update() override { if (_real) _real->update(); }
    Position read()   override { return _p.valid ? _p : (_real ? _real->read() : _p); }
    bool     hasFix() override { return _p.valid || (_real && _real->hasFix()); }

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
    Position         _p;             // .valid defaults false
    IPositionSource* _real = nullptr; // used until a position is injected
};

#endif  // SIM_MODE
