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




static int fails = 0;
static void expect(const char* w, bool ok) { std::printf("  %s %s\n", ok ? "PASS" : "FAIL", w); if (!ok) fails++; }
static HoverConfig fin() {
    HoverConfig h;
    h.min_flight_min = 0.25f; h.min_height_m = 2; h.max_height_m = 7;
    h.min_home_dist_m = 25; h.max_speed_mps = 1.2f; h.max_vspeed_mps = 1.0f;
    h.hold_s = 5; h.rearm_dist_m = 8; h.takeoff_height_m = 1.5f; h.use_b = false;
    return h;
}
// positions are metres north of the takeoff point (line already shifted west)
static void plan(const char* name, const double* wp, float gust) {
    int tot = 0; double worst = 0; bool clean = true;
    for (uint32_t seed = 1; seed <= 10; seed++) {
        Sim s(fin()); s.rng = seed * 104729u;
        s.arm(); s.run(3); s.climbTo(4, 1.5f);
        for (int i = 0; i < 5; i++) {
            s.flyTo(wp[i], 0, 3.0f); s.gust = gust; s.run(20); s.gust = 0.1f;
        }
        size_t n = s.drops.size();
        s.climbTo(20, 3.0f); s.flyTo(0, 0, 10.0f); s.descendTo(0, 2.0f); s.run(12);
        clean = clean && s.drops.size() == n && s.bDrops == 0;     // nothing during RTH/landing
        tot += (int)n;
        for (auto& d : s.drops) {
            double best = 1e9;
            for (int i = 0; i < 5; i++) best = std::min(best, sqrt(pow(d.n - wp[i], 2) + pow(d.e, 2)));
            worst = std::max(worst, best);
        }
    }
    std::printf("[%s] gust %.1f: %d/50 drops, worst %.1f m from nearest waypoint\n", name, gust, tot, worst);
    expect("all 50 drops fire", tot == 50);
    expect("nothing during RTH climb / return / landing, B untouched", clean);
}
int main() {
    g_quiet = true;
    const double opt1[5] = {40, 58, 76, 94, 112};   // takeoff 22 m behind the south goal line
    const double opt2[5] = {32, 45, 58, 71, 84};    // takeoff 6 m inside the pitch
    for (float g : {0.3f, 0.6f, 0.9f}) { plan("option 1", opt1, g); plan("option 2", opt2, g); }
    {
        Sim s(fin()); s.arm(); s.run(3); s.climbTo(4, 1.5f); s.flyTo(22, 0, 3.0f); s.run(25);
        expect("hover 22 m from takeoff never drops (limit 25 m)", s.drops.empty());
    }
    std::printf(fails ? "\n%d FAILED\n" : "\nALL PASS\n", fails);
    return fails;
}
