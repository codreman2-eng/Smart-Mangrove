// Minimal host stand-in for the Arduino core: enough for DropLogicEngine,
// ActuatorModule and StateMachine to compile and run on a PC.
#pragma once
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <cstdarg>
#include <string>

extern uint32_t g_ms;          // simulated clock, owned by the test
extern bool     g_quiet;       // suppress firmware prints
inline uint32_t millis() { return g_ms; }
inline uint32_t micros() { return g_ms * 1000u; }
inline void delay(uint32_t) {}

#define HIGH 1
#define LOW 0
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline int  digitalRead(int) { return HIGH; }
inline bool ledcAttach(int, int, int) { return true; }
inline void ledcWrite(int, uint32_t) {}

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
#define radians(d) ((d) * DEG_TO_RAD)
template <class T, class L, class H>
inline T constrain(T v, L lo, H hi) { return v < lo ? (T)lo : (v > hi ? (T)hi : v); }
#define F(x) x

class String {
public:
    std::string s;
    String() {}
    String(const char* c) : s(c ? c : "") {}
    String(const std::string& x) : s(x) {}
    String(double v, int dec) { char b[48]; snprintf(b, sizeof b, "%.*f", dec, v); s = b; }
    String(float v, int dec) : String((double)v, dec) {}
    String(int v) : s(std::to_string(v)) {}
    String(unsigned v) : s(std::to_string(v)) {}
    String(long v) : s(std::to_string(v)) {}
    String(unsigned long v) : s(std::to_string(v)) {}
    const char* c_str() const { return s.c_str(); }
    size_t length() const { return s.size(); }
    String operator+(const String& o) const { return String(s + o.s); }
    String operator+(const char* o) const { return String(s + o); }
    String& operator+=(const String& o) { s += o.s; return *this; }
};
inline String operator+(const char* a, const String& b) { return String(std::string(a) + b.s); }

struct SerialStub {
    void printf(const char* f, ...) {
        if (g_quiet) return;
        va_list ap; va_start(ap, f); std::printf("%8.2f  ", g_ms / 1000.0); std::vprintf(f, ap); va_end(ap);
    }
    void println(const char* m = "") { if (!g_quiet) std::printf("%8.2f  %s\n", g_ms / 1000.0, m); }
    void println(const String& m) { println(m.c_str()); }
    void print(const char* m) { if (!g_quiet) std::printf("%s", m); }
};
extern SerialStub Serial;
