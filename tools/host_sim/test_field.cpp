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
    uint32_t rng = 12345;
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
        Position& p = pos.p;
        p.valid = valid;
        // A3 fused position wanders a few cm at hover
        p.lat_deg = LAT0 + (n + jitter(0.05f)) * M_LAT;
        p.lon_deg = LON0 + (e + jitter(0.05f)) * M_LON;
        p.alt_m = h; p.height_m = h + jitter(0.3f); p.height_valid = valid;
        p.vn_mps = vn + jitter(0.1f); p.ve_mps = ve + jitter(0.1f); p.vd_mps = vd + jitter(0.05f);
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
static void expect(const char* what, bool ok) {
    std::printf("  %s %s\n", ok ? "PASS" : "FAIL", what);
    if (!ok) fails++;
}
static HoverConfig field() {        // soccer-field values, 4 m mission
    HoverConfig h;
    h.min_flight_min = 0.25f; h.min_height_m = 2; h.max_height_m = 7;
    h.min_home_dist_m = 30; h.max_speed_mps = 0.8f; h.max_vspeed_mps = 1.0f;
    h.hold_s = 5; h.rearm_dist_m = 10; h.takeoff_height_m = 1.5f; h.use_b = false;
    return h;
}
static void mission(double alt, float speed, bool& ok5, int& n, double& worst) {
    Sim s(field());
    s.arm(); s.run(5);
    s.climbTo(alt, 1.5f);
    const double wp[5] = {40, 60, 80, 100, 120};
    for (double w : wp) { s.flyTo(w, 0, speed); s.run(15); }
    s.climbTo(20, 3.0f);                       // RTH climb at WP5
    s.flyTo(0, 0, 10.0f);
    s.descendTo(0, 2.0f);
    s.run(12);
    n = (int)s.drops.size(); worst = 0; ok5 = (n == 5);
    for (int i = 0; i < n && i < 5; i++) {
        double d = sqrt(pow(s.drops[i].n - wp[i], 2) + pow(s.drops[i].e, 2));
        worst = std::max(worst, d);
        std::printf("     drop %d t=%.1fs at %.1f m (wp %.0f) h=%.1f\n", i + 1, s.drops[i].t,
                    s.drops[i].n, wp[i], s.drops[i].h);
    }
    ok5 = ok5 && s.bDrops == 0 && !s.sm.isArmed();
}
int main(int argc, char**) {
    g_quiet = argc < 2;
    bool ok; int n; double worst;
    std::printf("\n[F1] field mission at 4 m, 3 m/s legs, 15 s hovers, RTH 20 m\n");
    mission(4.0, 3.0f, ok, n, worst);
    expect("5 drops, A only, disarmed after", ok);
    expect("every drop within 1 m of its waypoint", n == 5 && worst < 1.0);
    std::printf("\n[F2] same mission if GS Pro forces 5 m\n");
    mission(5.0, 3.0f, ok, n, worst);
    expect("5 drops at 5 m too", ok && worst < 1.0);
    std::printf("\n[F3] same mission at 3 m\n");
    mission(3.0, 3.0f, ok, n, worst);
    expect("5 drops at 3 m too", ok && worst < 1.0);
    {
        std::printf("\n[F4] RTH with doors left after drop 2\n");
        Sim s(field());
        s.arm(); s.run(3); s.climbTo(4, 1.5f);
        s.flyTo(40, 0, 3); s.run(15); s.flyTo(60, 0, 3); s.run(15);
        s.flyTo(75, 0, 3);
        s.climbTo(20, 3.0f); s.run(5); s.flyTo(0, 0, 10); s.descendTo(0, 2); s.run(12);
        expect("only the 2 planned drops", s.drops.size() == 2);
    }
    {
        std::printf("\n[F5] pilot hovers 3 m over the takeoff area for a while\n");
        Sim s(field());
        s.arm(); s.run(3); s.climbTo(3, 1.5f); s.flyTo(15, 0, 2); s.run(40);
        expect("no drop inside 30 m", s.drops.empty());
    }
    {
        std::printf("\n[F6] creeping at 1.0 m/s in wind (over 0.8)\n");
        Sim s(field());
        s.arm(); s.run(3); s.climbTo(4, 1.5f); s.flyTo(40, 0, 3);
        s.ramp(1.0f, 0, 0, 1.0f); s.run(15);
        expect("no drop at 1.0 m/s", s.drops.empty());
    }
    {
        std::printf("\n[F7] second sortie: empty -> re-arm fails -> RESET -> 5 more\n");
        Sim s(field());
        s.arm(); s.run(3); s.climbTo(4, 1.5f);
        const double wp[5] = {40, 60, 80, 100, 120};
        for (double w : wp) { s.flyTo(w, 0, 3.0f); s.run(15); }
        s.flyTo(0, 0, 8.0f); s.descendTo(0, 1.5f); s.run(12);     // land at home
        expect("first sortie: 5 drops", s.drops.size() == 5);
        s.arm(); s.run(1);
        expect("re-arm without reset does not hold (the field bug)", !s.sm.isArmed());
        s.eng.resetDoors();
        expect("door counters back to 0", s.modA.doorsDropped() == 0 && s.modB.doorsDropped() == 0);
        s.arm(); s.run(1);
        expect("arm holds after reset", s.sm.isArmed());
        s.climbTo(4, 1.5f);
        for (double w : wp) { s.flyTo(w, 0, 3.0f); s.run(15); }
        expect("second sortie: 5 more drops at the same points", s.drops.size() == 10);
        expect("module B still never fired", s.bDrops == 0);
    }
    {
        std::printf("\n[F8] reset pressed while airborne keeps the takeoff point\n");
        Sim s(field());
        s.arm(); s.run(3); s.climbTo(4, 1.5f);
        s.flyTo(20, 0, 3.0f);                           // 20 m out, inside 30 m
        s.sm.requestDisarm(); s.run(1);
        s.eng.resetDoors();
        s.arm(); s.run(1);
        s.run(15);                                      // hover 20 m from takeoff
        expect("still no drop inside 30 m of the real takeoff", s.drops.empty());
    }
    std::printf(fails ? "\n%d FAILED\n" : "\nALL PASS\n", fails);
    return fails ? 1 : 0;
}
