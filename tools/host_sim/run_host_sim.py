"""Build and run the hover-release host simulations -- no ESP32 needed.

Compiles the REAL DropLogicEngine / ActuatorModule / StateMachine from
Servo_Web_Calibrator_WROVER/ against the stubs in stubs/, with zig's bundled
clang (no system compiler on this PC). ConfigManager.cpp needs ArduinoJson, so
only ConfigManager::setHover is pulled out of it, by brace matching, into
build/setHover.inc on every run -- the copy can never drift from the firmware.

    python -m pip install --target <somewhere> ziglang   (once)
    set PYTHONPATH=<somewhere>
    python tools/host_sim/run_host_sim.py               all tests
    python tools/host_sim/run_host_sim.py test_field -v  one test, firmware prints on

test_hover / test_field / test_final / test_fixes are pass/fail (exit code = failures).
test_wind / test_wind2 print the gust tables behind the max_speed_mps choice.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, "..", "..", "Servo_Web_Calibrator_WROVER"))
BUILD = os.path.join(HERE, "build")
TESTS = ["test_hover", "test_field", "test_final", "test_fixes", "test_wind", "test_wind2"]
SOURCES = ["DropLogicEngine.cpp", "ActuatorModule.cpp", "StateMachine.cpp"]


def extract_set_hover():
    with open(os.path.join(FW, "ConfigManager.cpp"), encoding="utf-8") as f:
        src = f.read()
    start = src.index("void ConfigManager::setHover(")
    depth, i = 0, src.index("{", start)
    while True:
        depth += {"{": 1, "}": -1}.get(src[i], 0)
        i += 1
        if depth == 0:
            break
    with open(os.path.join(BUILD, "setHover.inc"), "w", encoding="utf-8") as f:
        f.write(src[start:i] + "\n")


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("-")]
    verbose = "-v" in sys.argv
    tests = args or TESTS
    os.makedirs(BUILD, exist_ok=True)
    extract_set_hover()
    failed = 0
    for t in tests:
        exe = os.path.join(BUILD, t + ".exe")
        cmd = [sys.executable, "-m", "ziglang", "c++", "-std=c++17", "-w", "-O1",
               "-I", os.path.join(HERE, "stubs"), "-I", BUILD, "-I", FW,
               os.path.join(HERE, t + ".cpp")] + [os.path.join(FW, s) for s in SOURCES] + ["-o", exe]
        print(f"=== {t}", flush=True)
        if subprocess.call(cmd) != 0:
            print(f"{t}: BUILD FAILED")
            failed += 1
            continue
        rc = subprocess.call([exe] + (["-v"] if verbose else []))
        failed += rc != 0
    print(f"\n{failed} test program(s) failed" if failed else "\nall test programs passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
