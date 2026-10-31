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
        p.alt_m = h; p.height_m = h + jitter(0.05f); p.height_valid = valid;
        p.vn_mps = vn + jitter(0.1f); p.ve_mps = ve + jitter(0.1f); p.vd_mps = vd + jitter(0.05f);
        p.vel_valid = vel_ok && valid;
        modA.poll(); modB.poll();
        eng.update();
        sm.update();
        if (eng.missionComplete() && sm.isArmed()) sm.requestDisarm();   // as loop() does
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

static HoverConfig tomorrow() {     // the values going on the board
    HoverConfig h;
    h.min_flight_min = 0.25f; h.min_height_m = 5; h.max_height_m = 15;
    h.min_home_dist_m = 30; h.max_speed_mps = 1.0f; h.max_vspeed_mps = 1.0f;
    h.hold_s = 5; h.rearm_dist_m = 10; h.takeoff_height_m = 3; h.use_b = false;
    return h;
}

int main(int argc, char**) {
    g_quiet = argc < 2;

    // 1. The planned GS Pro mission: 5 waypoints 15 m apart, 8 m, hover 15 s,
    //    5 m/s legs, finish action RTH (climb to 30 m, fly home, land).
    {
        std::printf("\n[1] GS Pro mission, 5 drop waypoints, finish = RTH\n");
        Sim s(tomorrow());
        s.arm(); s.run(5);                         // armed on the ground
        s.climbTo(8, 2.0f);                        // GS Pro takeoff climb
        const double wp[5][2] = {{60, 0}, {60, 15}, {60, 30}, {60, 45}, {60, 60}};
        for (auto& w : wp) { s.flyTo(w[0], w[1], 5.0f); s.run(15); }
        s.climbTo(30, 3.0f);                       // RTH climb at WP5
        s.flyTo(0, 0, 8.0f);
        s.descendTo(0, 2.0f);
        s.run(12);
        expect("exactly 5 drops", s.drops.size() == 5);
        bool at_wps = s.drops.size() == 5;
        for (size_t i = 0; i < s.drops.size() && i < 5; i++) {
            double dn = s.drops[i].n - wp[i][0], de = s.drops[i].e - wp[i][1];
            bool near = sqrt(dn * dn + de * de) < 1.0 && fabs(s.drops[i].h - 8) < 0.5;
            std::printf("     drop %zu at t=%.1fs n=%.1f e=%.1f h=%.1f\n", i + 1,
                        s.drops[i].t, s.drops[i].n, s.drops[i].e, s.drops[i].h);
            at_wps = at_wps && near;
        }
        expect("each drop within 1 m of its waypoint, at 8 m", at_wps);
        expect("module B never fired", s.bDrops == 0);
        expect("disarmed itself after the 5th door", !s.sm.isArmed());
    }

    // 2. Doors left when RTH starts: the climb, the high transit stop and the
    //    landing must not fire.
    {
        std::printf("\n[2] RTH with doors left: climb, stop at 30 m, land\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(8, 2.0f);
        s.flyTo(60, 0, 5.0f); s.run(15);           // drop 1
        s.flyTo(60, 25, 5.0f);                     // 25 m on, then RTH immediately
        s.climbTo(30, 3.0f);                       // vertical climb in place
        s.run(10);                                 // pause at RTH altitude
        s.flyTo(0, 0, 8.0f);
        s.descendTo(0, 2.0f);
        s.run(12);
        expect("only the planned drop (RTH climb / 30 m pause / landing blocked)", s.drops.size() == 1);
    }

    // 3. Landing far from home (pilot lands in the field, 2 m/s descent)
    {
        std::printf("\n[3] Landing away from home\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(12, 2.0f);
        s.flyTo(80, 0, 5.0f);
        s.descendTo(0, 2.0f);
        s.run(12);
        expect("no drop while descending to land", s.drops.empty());
    }

    // 4. Turns: 3 s stops at non-drop waypoints must not fire (hold 5 s)
    {
        std::printf("\n[4] Brief 3 s stops at turn waypoints\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(8, 2.0f);
        s.flyTo(60, 0, 5.0f); s.run(3);
        s.flyTo(60, 20, 5.0f); s.run(3);
        s.flyTo(80, 20, 5.0f); s.run(3);
        expect("no drop on 3 s stops", s.drops.empty());
    }

    // 5. Creeping at 1.5 m/s (wind, pilot nudging) — over the 1.0 limit
    {
        std::printf("\n[5] Creeping at 1.5 m/s\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(8, 2.0f);
        s.flyTo(60, 0, 5.0f);
        s.ramp(1.5f, 0, 0, 1.0f); s.run(15);
        expect("no drop at 1.5 m/s", s.drops.empty());
    }

    // 6. Hover at 20 m (mission altitude set too high)
    {
        std::printf("\n[6] Hover at 20 m, above the 15 m maximum\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(20, 3.0f);
        s.flyTo(60, 0, 5.0f); s.run(15);
        expect("no drop above max height", s.drops.empty());
    }

    // 7. No OSDK velocity (broadcast trimmed in Assistant 2)
    {
        std::printf("\n[7] No OSDK velocity\n");
        Sim s(tomorrow());
        s.vel_ok = false;
        s.arm(); s.run(3);
        s.climbTo(8, 2.0f);
        s.flyTo(60, 0, 5.0f); s.run(15);
        expect("no drop without velocity", s.drops.empty());
    }

    // 8. Drop waypoint inside 30 m of takeoff
    {
        std::printf("\n[8] Waypoint 25 m from takeoff\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(8, 2.0f);
        s.flyTo(25, 0, 5.0f); s.run(15);
        expect("no drop inside 30 m", s.drops.empty());
    }

    // 9. Close first waypoint: reached ~12 s after takeoff, 15 s hover
    {
        std::printf("\n[9] First waypoint reached quickly (35 m out)\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(8, 2.0f);
        s.flyTo(35, 0, 5.0f); s.run(15);
        expect("still drops inside the 15 s hover", s.drops.size() == 1);
    }

    // 10. OSDK dropout during the hover restarts the hold, drop still happens
    {
        std::printf("\n[10] 0.6 s OSDK dropout mid-hover\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(8, 2.0f);
        s.flyTo(60, 0, 5.0f);
        s.run(3); s.valid = false; s.run(0.6f); s.valid = true;
        s.run(12);
        expect("drop after the hold restarts", s.drops.size() == 1);
    }

    // 11. Gentle altitude settle (0.4 m/s) during the hover is still a hover
    {
        std::printf("\n[11] 0.4 m/s vertical settle while hovering\n");
        Sim s(tomorrow());
        s.arm(); s.run(3);
        s.climbTo(10, 2.0f);
        s.flyTo(60, 0, 5.0f);
        s.ramp(0, 0, 0.4f, 0.5f); s.run(8); s.ramp(0, 0, 0, 0.5f); s.run(5);
        expect("drops (|vz| 0.4 <= 1.0)", s.drops.size() == 1);
    }

    // 12. Disarmed for the whole flight: nothing fires
    {
        std::printf("\n[12] Never armed\n");
        Sim s(tomorrow());
        s.climbTo(8, 2.0f);
        s.flyTo(60, 0, 5.0f); s.run(15);
        expect("no drop while disarmed", s.drops.empty());
    }

    std::printf(fails ? "\n%d FAILED\n" : "\nALL PASS\n", fails);
    return fails ? 1 : 0;
}
