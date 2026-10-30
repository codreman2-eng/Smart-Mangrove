#pragma once
#include "PositionSource.h"
#include <HardwareSerial.h>

// One non-broadcast frame worth recording, queued for the flight log.
//
// WHY THIS EXISTS (reports/Estado de waypoints en el M600.md): DJI's Open
// Protocol defines two unsolicited mission pushes on this same API UART,
// besides the 0x02/0x00 broadcast:
//   0x02/0x03 Ground Station State   mission_type, waypoint_index,
//                                    current_status (0 pre-mission,
//                                    1 in-action, 5 first waypt, 6 reached),
//                                    error_notification, reserved u16
//   0x02/0x04 Waypoint Event         incident_type, waypoint_index,
//                                    current_status (4 pre-action,
//                                    6 post-action), reserved x2
// (layouts from OSDK 3.9 dji_ack.hpp, #pragma pack(1)). Whether the A3 emits
// them for a GS Pro mission, to a never-activated device, is UNKNOWN — DJI's
// only public answer was not encouraging. This only LISTENS so a bench or
// simulator run can settle it. Nothing here is fed to drop logic.
struct OsdkMissionMsg {
    enum Kind : uint8_t {
        STATUS    = 0,   // 0x02/0x03
        EVENT     = 1,   // 0x02/0x04
        NEW_CMD   = 2,   // first sighting of any other cmd_set/cmd_id pair
        ENCRYPTED = 3,   // first encrypted frame: cmd bytes are unreadable
    };
    Kind     kind;
    uint32_t ms;
    uint8_t  cmd_set, cmd_id, enc;
    uint16_t len;          // payload bytes, cmd_set/cmd_id included
    uint8_t  raw[8];       // payload from cmd_set on, zero-padded
};

// Position from the DJI M600's A3 Pro Onboard SDK broadcast.
//
// READ-ONLY BY CONSTRUCTION. The A3 broadcasts unprompted — verified on
// real hardware with the adapter's TX physically disconnected — so this
// class never transmits, never activates, and never requests control
// authority. That keeps it inside the project's rule (consume host data,
// never command through the host) and keeps DJI's SDK Return-Home failsafe
// out of reach, since that is tied to holding control authority.
//
// WIRING
//   A3 API TX -> ESP32 GPIO 25 (RX2)   [330R series optional]
//   A3 API GND-> ESP32 GND             MANDATORY
//   A3 API RX / VCC: NOT CONNECTED
// Both sides are 3.3V; no level shifter.
//
// !! Serial2 defaults to GPIO 16/17, which are the PSRAM bus on WROVER
//    modules. The pins MUST be passed explicitly to begin(). Same family of
//    fault as the GPIO 9/10 bug already logged in PROJECT_HANDOFF.md.
//
// FRAME FORMAT (recovered from a live capture, see tools/osdk_sniff.py)
//   header 12B: 0xAA | len(10b) ver(6b) | session(5b) ack(1b) | ... |
//               seq(16b) | crc16(16b)
//   payload   : cmd_set(1) cmd_id(1) passFlag(uint16) then the enabled
//               fields IN BIT ORDER
//   trailer   : crc32(4)
//   CRC16 poly 0xA001 init 0x3AA3 over the first 10 header bytes
//   CRC32 poly 0xEDB88320 init 0x3AA3 over everything but the last 4 bytes
//
// Unlike the discovery sniffer — which deliberately did NOT gate on CRC,
// because the constants were unknown — this class DOES reject bad CRCs.
// The constants are now verified (602/602 frames), so a failing CRC here
// means corruption, and corrupt position data must never reach drop logic.
class OsdkPositionSource : public IPositionSource {
public:
    OsdkPositionSource(uint8_t uart_num, int rx_pin, int tx_pin,
                       uint32_t baud = 921600);

    void     begin()  override;
    void     update() override;
    Position read()   override;
    bool     hasFix() override;

    // True when no valid POSITION field has arrived recently. The caller
    // uses this to fall back to the onboard GNSS rather than flying on a
    // position that stopped updating — a stale value is more dangerous than
    // an absent one, because it still looks valid.
    bool     isStale(uint32_t max_age_ms = 500) const;

    uint32_t frames()    const { return _frames; }
    uint32_t crcErrors() const { return _crc_err; }
    uint32_t resyncs()   const { return _resyncs; }
    uint16_t passFlag()  const { return _last_flag; }
    uint32_t lastFixMs() const { return _last_fix_ms; }
    // Link vs fix are different questions: indoors the A3 broadcasts at full
    // rate with health 0, and "no fix" must not read as "no link".
    uint32_t lastFrameMs() const { return _last_frame_ms; }
    uint8_t  gpsHealth()   const { return _last_health; }   // 0-5; fix needs >= 3

    // Mission pushes (see OsdkMissionMsg). STATUS is queued only when its
    // bytes change — if the A3 does send it, it may repeat at a steady rate,
    // and the flight log has a hard cap. The counters still count every frame.
    uint32_t missionStatusFrames() const { return _mis_status; }
    uint32_t missionEventFrames()  const { return _mis_event; }
    uint32_t encryptedFrames()     const { return _enc_frames; }
    // Pops the oldest queued message; false when the queue is empty.
    bool     popMission(OsdkMissionMsg& out);

    // Latest decoded field values, for the dashboard's DJI MISSION card. Only
    // meaningful once the matching counter above is non-zero. Display only —
    // like everything mission-related here, nothing reaches drop logic.
    struct MissionLast {
        uint8_t  a = 0, wp = 0, st = 0;   // STATUS: type/wp/status; EVENT: incident/wp/status
        uint32_t ms = 0;                  // millis() of the last frame, 0 = never
    };
    const MissionLast& lastMissionStatus() const { return _last_mis_status; }
    const MissionLast& lastMissionEvent()  const { return _last_mis_event; }

private:
    HardwareSerial* _serial;
    uint8_t  _uart_num;
    int      _rx_pin, _tx_pin;
    uint32_t _baud;

    Position _last;
    uint32_t _last_fix_ms = 0;
    uint32_t _last_frame_ms = 0;   // any CRC-good frame, fix or not
    uint16_t _last_flag   = 0;
    uint32_t _frames = 0, _crc_err = 0, _resyncs = 0;
    uint8_t  _last_health   = 0;   // reported even when the fix gate rejects
    uint32_t _last_report_ms = 0;

    // One max-size frame. The length field is 10 bits, so 1023 is the
    // hard ceiling and no allocation can grow past it.
    static const uint16_t BUF_SZ = 1024;
    uint8_t  _buf[BUF_SZ];
    uint16_t _len = 0;

    // Bounded work per update() so a burst can never monopolise loop().
    static const uint16_t MAX_BYTES_PER_UPDATE = 512;

    // ── Mission pushes ────────────────────────────────────────────────
    uint32_t _mis_status = 0, _mis_event = 0, _enc_frames = 0;
    uint8_t  _last_status[6] = {0};
    bool     _have_status = false;
    MissionLast _last_mis_status, _last_mis_event;

    // Small ring; on overflow the oldest is overwritten. 8 is plenty: the
    // loop drains it every pass, and events arrive at most a few per waypoint.
    static const uint8_t MQ_SZ = 8;
    OsdkMissionMsg _mq[MQ_SZ];
    uint8_t  _mq_head = 0, _mq_count = 0;

    // cmd pairs already reported as NEW_CMD (broadcast 0x02/0x00 excluded).
    static const uint8_t SEEN_SZ = 16;
    uint16_t _seen[SEEN_SZ];
    uint8_t  _seen_n = 0;

    void _consume();
    bool _decode(const uint8_t* payload, uint16_t n);
    void _onOtherFrame(const uint8_t* payload, uint16_t n, uint8_t enc);
    void _queue(OsdkMissionMsg::Kind k, const uint8_t* payload, uint16_t n,
                uint8_t enc);
};

// Prefers a primary source, falls back to a secondary when the primary goes
// stale. Exists so one loose connector cannot take the launcher's position
// away entirely — and so the same firmware still runs on the DIY ArduPilot
// aircraft, which has no OSDK at all.
class FallbackPositionSource : public IPositionSource {
public:
    FallbackPositionSource(OsdkPositionSource& primary, IPositionSource& secondary,
                           uint32_t stale_ms = 500)
        : _p(primary), _s(secondary), _stale_ms(stale_ms) {}

    void begin() override  { _p.begin(); _s.begin(); }
    void update() override { _p.update(); _s.update(); }

    Position read() override {
        bool use_primary = !_p.isStale(_stale_ms) && _p.hasFix();
        if (use_primary != _on_primary) {
            _on_primary = use_primary;
            Serial.printf("[POS] source -> %s\n", use_primary ? "OSDK" : "GNSS(fallback)");
        }
        return use_primary ? _p.read() : _s.read();
    }
    bool hasFix() override {
        return (!_p.isStale(_stale_ms) && _p.hasFix()) || _s.hasFix();
    }
    bool onPrimary() const { return _on_primary; }

private:
    OsdkPositionSource& _p;
    IPositionSource&    _s;
    uint32_t _stale_ms;
    bool     _on_primary = false;
};
