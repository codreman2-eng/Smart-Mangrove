// Host test of the 2026-10-31 field-test fixes, on the REAL DropLogicEngine:
//   - landing is still detected after the rack runs out of doors
//   - hover.need_mission holds drops back unless the A3 reports a mission
//   - onGround(), which slows the armed telemetry rate on the ground
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
    bool   dji = false;                  // A3 reports a GS Pro mission running
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
        eng.setDjiMission(dji);
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

static HoverConfig board1031() {    // the values on the board after the 2026-10-31 test
    HoverConfig h;
    h.min_flight_min = 0.25f; h.min_height_m = 5; h.max_height_m = 11;
    h.min_home_dist_m = 25; h.max_speed_mps = 0.5f; h.max_vspeed_mps = 1.0f;
    h.hold_s = 2; h.rearm_dist_m = 8; h.takeoff_height_m = 1.5f; h.use_b = false;
    return h;
}
static int count(const char* ev) {
    int n = 0;
    for (auto& e : g_events) n += e.rfind(std::string(ev) + " ", 0) == 0;
    return n;
}

int main(int argc, char**) {
    g_quiet = argc < 2;

    // 1. Flight 3 of 2026-10-31: 10 m, 1.5 m/s, GS Pro hover 5 s, first WP
    //    23 m out (inside the 25 m guard), then three more.
    {
        std::printf("\n[1] 2026-10-31 flight 3 shape, board values, hover 5 s\n");
        Sim s(board1031());
        s.arm(); s.run(3);
        s.climbTo(10, 2.0f);
        const double wp[4][2] = {{0, -22.7}, {33.3, -24.0}, {32.8, -48.9}, {-5.3, -47.5}};
        for (auto& w : wp) { s.flyTo(w[0], w[1], 1.5f); s.run(5); }
        expect("3 drops (first WP held by the 25 m guard)", s.drops.size() == 3);
        bool near = s.drops.size() == 3;
        for (size_t i = 0; i < s.drops.size() && i < 3; i++) {
            double dn = s.drops[i].n - wp[i + 1][0], de = s.drops[i].e - wp[i + 1][1];
            std::printf("     drop %zu  %.2f m from WP%zu\n", i + 1, sqrt(dn * dn + de * de), i + 1);
            near = near && sqrt(dn * dn + de * de) < 1.0;
        }
        expect("each within 1 m of its waypoint", near);
    }

    // 2. Rack empties in the air, then the aircraft lands. Before the fix
    //    "landed" never came until RESET DOOR COUNT (seen twice on 10-31).
    {
        std::printf("\n[2] Out of doors in the air, then land\n");
        Sim s(board1031());
        s.arm(); s.run(3);
        s.climbTo(10, 2.0f);
        for (int i = 0; i < 5; i++) { s.flyTo(40 + 12 * i, 0, 1.5f); s.run(5); }
        expect("5 drops, rack empty", s.drops.size() == 5 && s.eng.missionComplete());
        expect("disarmed itself", !s.sm.isArmed());
        expect("still airborne, so not on the ground", !s.eng.onGround());
        s.flyTo(0, 0, 5.0f);
        s.descendTo(0, 2.0f);
        s.run(12);
        expect("landed logged with no door reset", count("landed") == 1);
        expect("onGround() after landing", s.eng.onGround());
        int before = count("takeoff");
        s.eng.resetDoors();
        s.arm(); s.run(1);
        s.climbTo(10, 2.0f);
        expect("next takeoff logged straight after a reset", count("takeoff") == before + 1);
        expect("flight clock restarted (hover t < 15 s)", s.eng.hover().flight_s < 15.0f);
    }

    // 3. need_mission: a stop outside the mission must not fire
    {
        std::printf("\n[3] Only during the DJI mission\n");
        HoverConfig h = board1031(); h.need_mission = true;
        Sim s(h);
        s.arm(); s.run(3);
        s.climbTo(10, 2.0f);
        s.flyTo(40, 0, 1.5f); s.run(6);
        expect("no mission reported -> no drop", s.drops.empty());
        expect("checklist shows the mission condition missing", !s.eng.hover().ok_mission);
        s.dji = true;
        s.flyTo(55, 0, 1.5f); s.run(6);
        expect("mission running -> drops", s.drops.size() == 1);
        s.flyTo(70, 0, 1.5f);
        s.dji = false;                              // mission ends, pilot stops (F1/F2 on 10-31)
        s.run(10);
        expect("stop after the mission ended -> no drop", s.drops.size() == 1);
        s.dji = true; s.run(1); s.dji = false; s.run(5);
        expect("mission flag dropping mid-hold restarts the hold (no drop)", s.drops.size() == 1);
    }

    // 4. need_mission off: same stops fire as before (default unchanged)
    {
        std::printf("\n[4] need_mission off ignores the mission flag\n");
        Sim s(board1031());
        s.arm(); s.run(3);
        s.climbTo(10, 2.0f);
        s.flyTo(40, 0, 1.5f); s.run(6);
        expect("drops with no mission reported", s.drops.size() == 1);
    }

    // 5. onGround() needs OSDK height; without it the 1 Hz rate stays
    {
        std::printf("\n[5] onGround()\n");
        Sim s(board1031());
        s.arm(); s.run(3);
        expect("armed on the pad, height arriving -> on ground", s.eng.onGround());
        s.valid = false; s.run(1);
        expect("no OSDK data -> not known to be on ground", !s.eng.onGround());
        s.valid = true; s.run(1);
        s.climbTo(10, 2.0f);
        expect("airborne -> not on ground", !s.eng.onGround());
    }

    std::printf(fails ? "\n%d FAILED\n" : "\nALL PASS\n", fails);
    return fails ? 1 : 0;
}
