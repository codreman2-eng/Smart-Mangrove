// Host test of the REAL DropLogicEngine / ActuatorModule / StateMachine in
// hover release mode, flown through simulated GS Pro missions.
#include <Arduino.h>
#include <vector>
#include <functional>
#include "ConfigManager.h"
#include "PositionSource.h"
#include "ActuatorModule.h"
#include "StateMachine.h"
#include "Logger.h"
#include "DropLogicEngine.h"

uint32_t   g_ms    = 1000;
bool       g_quiet = true;
SerialStub Serial;

// --- Pieces of the firmware that need LittleFS/ArduinoJson -----------------
#include "setHover.inc"                       // extracted from ConfigManager.cpp by run_host_sim.py
static std::vector<std::string> g_events;
void Logger::logEvent(const char* type, const char* extra) {
    g_events.push_back(std::string(type) + " " + (extra ? extra : ""));
    if (!g_quiet) std::printf("%8.2f  [EVT] %s %s\n", g_ms / 1000.0, type, extra ? extra : "");
}
void Logger::logDrop(const DropEvent&) {}

// --- Simulated aircraft ----------------------------------------------------
struct FakePos : IPositionSource {
    Position p;
    void begin() override {}
    void update() override {}
    Position read() override { return p; }
    bool hasFix() override { return p.valid; }
};

static const double LAT0 = 9.0, LON0 = -79.5;
static const double M_LAT = 1.0 / 111320.0;
static const double M_LON = 1.0 / (111320.0 * cos(9.0 * DEG_TO_RAD));

struct Sim {
    ConfigManager  cfg;
    FakePos        pos;
    ActuatorModule modA{12, 32, 1470, 1670, 580, 5, 'A'};
    ActuatorModule modB{13, 33, 1300, 1500, 350, 5, 'B'};
    StateMachine   sm{22, 0, 27};
    Logger         log;
    DropLogicEngine eng{cfg, pos, modA, modB, sm, log};

    double n = 0, e = 0, h = 0;          // metres from takeoff point
    float  vn = 0, ve = 0, vd = 0;       // vd positive DOWN
    bool   vel_ok = true, valid = true;
    float  gust = 0.1f;               // velocity noise amplitude at hover
    uint32_t rng = 12345;
    float gn = 0, ge = 0, tgn = 0, tge = 0;
    struct Drop { double t, n, e, h; };
    std::vector<Drop> drops;
    uint8_t lastA = 0, lastB = 0;
    int     bDrops = 0;

    explicit Sim(HoverConfig hc) {
        hc.enabled = true;
        cfg.setHover(hc);
        modA.begin(); modB.begin(); sm.begin(); sm.completeBoot();
        g_events.clear();
    }
    float jitter(float a) {             // deterministic noise in [-a, a]
        rng = rng * 1103515245u + 12345u;
        return a * (((rng >> 8) & 0xFFFF) / 32767.5f - 1.0f);
    }
    void tick() {
        g_ms += 20;
        const float dt = 0.02f;
        n += vn * dt; e += ve * dt; h -= vd * dt;
        if (h < 0) h = 0;
        // gusts: a slowly wandering velocity error (0.5 s correlation), applied to
        // both the reported velocity and the real motion, so position agrees
        if ((g_ms / 20) % 25 == 0) { tgn = jitter(gust); tge = jitter(gust); }
        gn += (tgn - gn) * 0.08f; ge += (tge - ge) * 0.08f;
        n += gn * dt; e += ge * dt;
        Position& p = pos.p;
        p.valid = valid;
        // A3 fused position wanders a few cm at hover
        p.lat_deg = LAT0 + (n + jitter(0.05f)) * M_LAT;
        p.lon_deg = LON0 + (e + jitter(0.05f)) * M_LON;
        p.alt_m = h; p.height_m = h + jitter(0.3f); p.height_valid = valid;
        p.vn_mps = vn + gn; p.ve_mps = ve + ge; p.vd_mps = vd + jitter(0.05f);
        p.vel_valid = vel_ok && valid;
        modA.poll(); modB.poll();
        eng.update();
        sm.update();
        if (eng.missionComplete() && sm.isArmed()) sm.requestDisarm();   // as loop() does
        if (modA.doorsDropped() < lastA) lastA = modA.doorsDropped();   // a reset, not a drop
        if (modA.doorsDropped() != lastA) {
            lastA = modA.doorsDropped();
            drops.push_back({g_ms / 1000.0, n, e, h});
        }
        if (modB.doorsDropped() != lastB) { lastB = modB.doorsDropped(); bDrops++; }
    }
    void run(float secs) { for (int i = 0; i < (int)(secs / 0.02f); i++) tick(); }
    // Ramp velocity linearly to a target over `secs`
    void ramp(float tn, float te, float td, float secs) {
        int k = std::max(1, (int)(secs / 0.02f));
        float sn = (tn - vn) / k, se = (te - ve) / k, sd = (td - vd) / k;
        for (int i = 0; i < k; i++) { vn += sn; ve += se; vd += sd; tick(); }
        vn = tn; ve = te; vd = td;
    }
    void climbTo(double target, float rate) {          // rate > 0 = up
        ramp(vn, ve, -rate, 0.5f);
        while (h < target - 0.05) tick();
        ramp(vn, ve, 0, 0.5f);
    }
    void descendTo(double target, float rate) {
        ramp(vn, ve, rate, 0.5f);
        while (h > target + 0.05 && h > 0) tick();
        ramp(vn, ve, 0, 0.5f);
    }
    // GS Pro "straight route" leg: accelerate, cruise, decelerate to a stop
    void flyTo(double tn, double te, float speed) {
        double dn = tn - n, de = te - e, d = sqrt(dn * dn + de * de);
        if (d < 0.01) return;
        float un = dn / d, ue = de / d;
        ramp(un * speed, ue * speed, 0, 1.5f);
        while (true) {
            double rn = tn - n, re = te - e, rd = sqrt(rn * rn + re * re);
            if (rd <= speed * 0.75) break;            // ~decel distance for a 1.5 s ramp
            tick();
        }
        ramp(0, 0, 0, 1.5f);
    }
    void arm() { sm.requestArm(); }
};



static HoverConfig cfg(float spd) {
    HoverConfig h;
    h.min_flight_min = 0.25f; h.min_height_m = 2; h.max_height_m = 7;
    h.min_home_dist_m = 30; h.max_speed_mps = spd; h.max_vspeed_mps = 1.0f;
    h.hold_s = 5; h.rearm_dist_m = 10; h.takeoff_height_m = 1.5f; h.use_b = false;
    return h;
}
int main() {
    g_quiet = true;
    const float gusts[] = {0.6f, 0.9f, 1.2f, 1.5f, 1.8f};
    std::printf("drops out of 5 (worst miss distance m) -- 5 runs each, different noise seeds\n");
    std::printf("gust amp |   spd1.0 h20   |   spd1.2 h20   |   spd1.5 h20\n");
    for (float g : gusts) {
        std::printf("  %.1f    ", g);
        for (float spd : {1.0f, 1.2f, 1.5f}) for (float hov : {20.0f}) {
            int tot = 0; double worst = 0;
            for (uint32_t seed = 1; seed <= 5; seed++) {
                Sim s(cfg(spd)); s.rng = seed * 7919u; s.gust = 0.1f;
                s.arm(); s.run(3); s.climbTo(4, 1.5f);
                const double wp[5] = {40, 60, 80, 100, 120};
                size_t before = 0;
                for (double w : wp) {
                    s.flyTo(w, 0, 3.0f); s.gust = g; s.run(hov); s.gust = 0.1f;
                    if (s.drops.size() > before) {
                        auto& d = s.drops.back();
                        worst = std::max(worst, sqrt(pow(d.n - w, 2) + pow(d.e, 2)));
                        before = s.drops.size();
                    }
                }
                tot += (int)s.drops.size();
            }
            std::printf("|  %2d/25 (%.1f)   ", tot, worst);
        }
        std::printf("\n");
    }
}
