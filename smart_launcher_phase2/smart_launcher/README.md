# Smart Launcher — Firmware

ESP32-S3 autonomous seed release system for drone-based mangrove reforestation.

## Hardware
- MCU: ESP32-S3 DevKitC-1 N8R8
- GNSS: u-blox M8N (UART1, GPIO 4/5)
- Servo A: FS5106R continuous rotation (GPIO 18, LEDC ch0)
- Servo B: FS5106R continuous rotation (GPIO 8,  LEDC ch1)
- Limit switch A: GPIO 9  (INPUT_PULLUP, active LOW)
- Limit switch B: GPIO 10 (INPUT_PULLUP, active LOW)
- ARM button:     GPIO 0  (INPUT_PULLUP, active LOW, hold 2s)
- Status LED:     GPIO 48 (WS2812 RGB)

## Pin Summary
| Signal        | GPIO | Notes                        |
|---------------|------|------------------------------|
| GNSS RX       |  4   | UART1 ← GNSS TX              |
| GNSS TX       |  5   | UART1 → GNSS RX              |
| MAVLink RX    | 16   | UART2 ← Cube Orange TELEM2   |
| MAVLink TX    | 17   | UART2 → Cube Orange TELEM2   |
| Servo A PWM   | 18   | LEDC ch0, 50Hz               |
| Servo B PWM   |  8   | LEDC ch1, 50Hz               |
| Limit SW A    |  9   | Pull-up, active LOW          |
| Limit SW B    | 10   | Pull-up, active LOW          |
| ARM button    |  0   | Pull-up, hold 2s to ARM      |
| Status LED    | 48   | WS2812 RGB                   |

## LED Status
| Colour        | State    | Meaning                      |
|---------------|----------|------------------------------|
| Blue          | BOOT     | Starting up                  |
| Yellow        | IDLE     | Ready, not armed             |
| Green blink   | ARMED    | Drop logic active            |
| Purple        | DROPPING | Executing a drop             |
| Red           | FAULT    | Error — check serial output  |

## Serial Commands (115200 baud, bench testing)
```
arm      — force ARM (bypasses GPS check)
disarm   — disarm
status   — print current state and GPS position
dropa    — manually trigger Module A (one door)
dropb    — manually trigger Module B (one door)
reset    — reset door counters on both modules
log      — dump flight log to serial
help     — show this list
```

## Mission Config (LittleFS: /mission.json)
Edit data/mission.json before uploading filesystem.
Key fields:
- position_source: "GNSS" or "MAVLINK"
- modules.A/B.servo_open_pwm_us: PWM to run servo (calibrate on bench)
- modules.A/B.travel_ms: time to travel 35mm (calibrate on bench)
- drops[].radius_m: trigger radius in metres

## Build & Upload
```bash
# Build firmware
pio run

# Upload firmware
pio run --target upload

# Upload filesystem (mission.json)
pio run --target uploadfs

# Monitor serial
pio device monitor
```

## Calibration (do this before first flight)
1. Flash firmware and filesystem
2. Open serial monitor
3. Type 'dropa' — measure how far Module A rack actually moved
4. If too far/short, adjust modules.A.travel_ms in mission.json
5. Repeat for Module B
6. Re-upload filesystem: pio run --target uploadfs
