#include "OsdkPositionSource.h"

// ─── CRC — verified against 602 live frames, see tools/osdk_sniff.py ────
static uint16_t osdkCrc16(const uint8_t* d, uint16_t n, uint16_t crc = 0x3AA3) {
    for (uint16_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

static uint32_t osdkCrc32(const uint8_t* d, uint16_t n, uint32_t crc = 0x00003AA3) {
    for (uint16_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (uint8_t b = 0; b < 8; b++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : crc >> 1;
    }
    return crc;
}

// ─── Broadcast field sizes, bits 0..5 ──────────────────────────────────
// Derived empirically, and they check out: the five fields preceding
// POSITION sum to 68, which plus the 4-byte cmd_set/cmd_id/passFlag preamble
// puts POSITION at payload offset 72 — exactly where a 16-byte run of
// float64 zeros (no GPS fix) and a stable barometric altitude were observed.
//
// The offset MUST be computed from the passFlag rather than hardcoded to 72.
// Fields are packed in bit order, so turning any of TIME/QUATERNION/ACCEL/
// ANG_RATE off in DJI Assistant 2 slides POSITION earlier. A hardcoded 72
// would silently decode garbage the moment that config changed.
// VELOCITY must stay ON since Phase 4: the predictor and the release speed
// gate need it, and without it both switch off (the gate fails open).
static const uint8_t FIELD_SZ[6] = {
    15,   // bit 0  TIME
    16,   // bit 1  QUATERNION  (4 x float32)
    12,   // bit 2  ACCEL       (3 x float32)
    13,   // bit 3  VELOCITY    (3 x float32 + health)
    12,   // bit 4  ANG_RATE    (3 x float32)
    25    // bit 5  POSITION    (float64 lat, float64 lon, float32 alt,
};        //                     float32 height, uint8 health)

static const uint8_t  FLAG_POSITION = 5;
static const uint8_t  FLAG_VELOCITY = 3;

// DJI's broadcast velocity is a ground-frame vector; the horizontal pair maps
// to North/East. The VERTICAL SIGN IS NOT VERIFIED on this aircraft, so it is
// named rather than assumed silently.
//
// It matters less than it looks: vertical velocity only enters fall time, and
// only second-order. At 8m a 1 m/s vertical error moves the impact point about
// 1m, against the ~6.4m the horizontal terms are correcting. A wrong sign here
// degrades accuracy; it cannot send a drop somewhere absurd.
//
// TO VERIFY (no flight needed beyond a climb): watch `vd` against the trend of
// the barometric `alt_m` decoded below. Climbing must give NEGATIVE vd. If they
// disagree, flip this constant. Do that before trusting a drop from a
// descending or climbing pass.
static const bool     OSDK_VZ_IS_UP = true;
static const uint8_t  HDR_LEN       = 12;
static const uint8_t  SOF           = 0xAA;
static const double   RAD2DEG       = 57.29577951308232;

OsdkPositionSource::OsdkPositionSource(uint8_t uart_num, int rx_pin, int tx_pin,
                                       uint32_t baud)
    : _uart_num(uart_num), _rx_pin(rx_pin), _tx_pin(tx_pin), _baud(baud) {
    _serial = (uart_num == 1) ? &Serial1 : &Serial2;
}

void OsdkPositionSource::begin() {
    // 11.4 kB/s continuous with 231-byte frames every 20ms, against a 128-byte
    // hardware FIFO — smaller than a single frame. Without a larger driver
    // buffer any loop() hiccup silently drops bytes, which presents as a
    // flaky cable rather than as the software problem it actually is.
    // 2048 bytes buys roughly 180ms of slack. MUST precede begin().
    _serial->setRxBufferSize(2048);
    // Pins passed EXPLICITLY: Serial2 defaults to GPIO 16/17, which are the
    // PSRAM bus on WROVER modules.
    _serial->begin(_baud, SERIAL_8N1, _rx_pin, _tx_pin);
    Serial.printf("[OSDK] UART%d @ %lu, RX=GPIO%d TX=GPIO%d (read-only; "
                  "TX is never driven)\n",
                  _uart_num, (unsigned long)_baud, _rx_pin, _tx_pin);
}

void OsdkPositionSource::update() {
    uint16_t budget = MAX_BYTES_PER_UPDATE;
    while (_serial->available() && budget--) {
        if (_len < BUF_SZ) _buf[_len++] = (uint8_t)_serial->read();
        else               _len = 0;          // desynced beyond recovery
    }
    _consume();

    // Periodic health line. Field diagnosis needs to separate "nothing on the
    // wire" from "frames arriving but no fix" from "fix but stale", and those
    // are indistinguishable from the LEDs or from a browser that may not be
    // in range. Serial is the one channel always available on the bench.
    uint32_t now = millis();
    if (now - _last_report_ms >= 5000) {
        _last_report_ms = now;
        Serial.printf("[OSDK] frames=%lu crcErr=%lu resync=%lu flag=0x%04X "
                      "lat=%.7f lon=%.7f health=%u fix=%s mis=%lu evt=%lu enc=%lu\n",
                      (unsigned long)_frames, (unsigned long)_crc_err,
                      (unsigned long)_resyncs, _last_flag,
                      _last.lat_deg, _last.lon_deg, _last_health,
                      hasFix() ? "YES" : "no",
                      (unsigned long)_mis_status, (unsigned long)_mis_event,
                      (unsigned long)_enc_frames);
        if (_frames == 0)
            Serial.println("[OSDK] no frames at all - check GPIO25 wiring, "
                           "common ground, and that the A3 is powered");
    }
}

void OsdkPositionSource::_consume() {
    uint16_t i = 0;
    while (i + HDR_LEN <= _len) {
        if (_buf[i] != SOF) { i++; continue; }

        uint32_t w0 = (uint32_t)_buf[i] | ((uint32_t)_buf[i+1] << 8) |
                      ((uint32_t)_buf[i+2] << 16) | ((uint32_t)_buf[i+3] << 24);
        uint16_t length = (w0 >> 8) & 0x3FF;
        if (length < HDR_LEN + 4 || length > BUF_SZ) { i++; _resyncs++; continue; }
        if (i + length > _len) break;                     // need more bytes

        const uint8_t* fr = &_buf[i];
        uint16_t hcrc = (uint16_t)fr[10] | ((uint16_t)fr[11] << 8);
        if (osdkCrc16(fr, 10) != hcrc) { i++; _resyncs++; continue; }

        uint32_t tcrc = (uint32_t)fr[length-4] | ((uint32_t)fr[length-3] << 8) |
                        ((uint32_t)fr[length-2] << 16) | ((uint32_t)fr[length-1] << 24);
        if (osdkCrc32(fr, length - 4) != tcrc) {
            _crc_err++;                 // corrupt: drop it, never decode it
            i += length;
            continue;
        }

        _frames++;
        _last_frame_ms = millis();
        const uint8_t* pl  = fr + HDR_LEN;
        const uint16_t pn  = length - HDR_LEN - 4;
        const uint8_t  enc = (fr[4] >> 5) & 0x07;   // header bytes 4-7: pad(5) enc(3)
        if (enc == 0 && pn >= 2 && pl[0] == 0x02 && pl[1] == 0x00)
            _decode(pl, pn);                        // the position broadcast
        else
            _onOtherFrame(pl, pn, enc);             // listened to, never decoded into position
        i += length;
    }

    if (i > 0) {                        // shuffle the unconsumed tail down
        _len -= i;
        memmove(_buf, _buf + i, _len);
    }
}

bool OsdkPositionSource::_decode(const uint8_t* p, uint16_t n) {
    if (n < 4) return false;
    if (p[0] != 0x02 || p[1] != 0x00) return false;      // not broadcast (_consume already routes these away)

    uint16_t flag = (uint16_t)p[2] | ((uint16_t)p[3] << 8);
    _last_flag = flag;
    if (!(flag & (1u << FLAG_POSITION))) return false;    // no POSITION here

    // Offset computed from the flag, not hardcoded — see FIELD_SZ note above
    uint16_t off = 4;
    for (uint8_t b = 0; b < FLAG_POSITION; b++)
        if (flag & (1u << b)) off += FIELD_SZ[b];
    if (off + FIELD_SZ[FLAG_POSITION] > n) return false;

    double lat, lon;
    float  alt, height;
    memcpy(&lat,    p + off,      8);
    memcpy(&lon,    p + off + 8,  8);
    memcpy(&alt,    p + off + 16, 4);
    memcpy(&height, p + off + 20, 4);
    uint8_t health = p[off + 24];
    _last_health     = health;     // kept for the health line even if gated out

    _last.alt_m      = alt;
    _last.satellites = 0;              // sat count lives in the GPS field
    _last.hdop       = (health >= 3) ? 1.0f : 99.0f;

    // `height` is DJI's height above the TAKEOFF point, and it was parsed here
    // and thrown away for the whole of Phase 3d because Position had nowhere to
    // put it. It is the exact quantity the fall-time calculation needs, and it
    // was already on the wire — one struct member and one assignment.
    //
    // NOTE it is height above TAKEOFF, not above the drop surface. On a tidal
    // mangrove flat that difference is real; a per-waypoint ground_offset_m
    // belongs in mission.json, because the operator knows it and the aircraft
    // does not.
    _last.height_m     = height;
    _last.height_valid = (health >= 3) && isfinite(height) &&
                         height > -10.0f && height < 500.0f;

    // A frame arriving is NOT a fix. Indoors the A3 broadcasts happily with
    // lat/lon at exactly 0.0 and health 0 — accepting that would hand drop
    // logic a "valid" position off the coast of Africa. Require a non-zero
    // fix AND a usable health value.
    // ── VELOCITY (bit 3) ────────────────────────────────────────────────
    // Free to reach: bit 3 sits BELOW POSITION, so its size is already in
    // FIELD_SZ and the same offset-from-flag walk finds it with no new layout
    // work and no fresh capture. (STATUS, bit 11, is NOT free — it needs GPS,
    // RTK, MAG, RC and GIMBAL sized first.)
    _last.vel_valid = false;
    if (flag & (1u << FLAG_VELOCITY)) {
        uint16_t voff = 4;
        for (uint8_t b = 0; b < FLAG_VELOCITY; b++)
            if (flag & (1u << b)) voff += FIELD_SZ[b];

        if (voff + FIELD_SZ[FLAG_VELOCITY] <= n) {
            float vx, vy, vz;
            memcpy(&vx, p + voff,     4);
            memcpy(&vy, p + voff + 4, 4);
            memcpy(&vz, p + voff + 8, 4);
            // DJI packs this as `health : 1` plus 7 reserved bits.
            const bool vhealth = (p[voff + 12] & 0x01) != 0;

            const float vd = OSDK_VZ_IS_UP ? -vz : vz;
            if (vhealth && isfinite(vx) && isfinite(vy) && isfinite(vz) &&
                fabsf(vx) < 50.0f && fabsf(vy) < 50.0f && fabsf(vz) < 50.0f) {
                _last.vn_mps    = vx;
                _last.ve_mps    = vy;
                _last.vd_mps    = vd;
                _last.vel_valid = true;
            }
        }
    }

    bool fix = (health >= 3) && (lat != 0.0 || lon != 0.0);
    if (fix) {
        _last.lat_deg = lat * RAD2DEG;
        _last.lon_deg = lon * RAD2DEG;
        _last.valid   = true;
        _last_fix_ms  = millis();
    } else {
        _last.valid = false;
    }
    return true;
}

// ─── Everything that is not the position broadcast ─────────────────────
// Listen-only. Before this existed, _decode() discarded every frame that was
// not 0x02/0x00 without a trace, so if the A3 was already sending mission
// pushes nobody would have known.
void OsdkPositionSource::_onOtherFrame(const uint8_t* p, uint16_t n, uint8_t enc) {
    // An encrypted payload hides cmd_set/cmd_id too, so it cannot be sorted.
    // Report the first one; the counter keeps the rest.
    if (enc != 0) {
        if (_enc_frames++ == 0) {
            Serial.printf("[OSDK] first ENCRYPTED frame (enc=%u, %u bytes) - "
                          "contents unreadable without activation\n", enc, n);
            _queue(OsdkMissionMsg::ENCRYPTED, p, n, enc);
        }
        return;
    }
    if (n < 2) return;
    const uint8_t set = p[0], id = p[1];

    if (set == 0x02 && id == 0x03) {                 // Ground Station State
        _mis_status++;
        uint8_t cur[6] = {0};
        memcpy(cur, p + 2, min<uint16_t>(6, n - 2));
        _last_mis_status.a  = cur[0];
        _last_mis_status.wp = cur[1];
        _last_mis_status.st = cur[2];
        _last_mis_status.ms = millis();
        if (_have_status && memcmp(cur, _last_status, sizeof(cur)) == 0) return;
        memcpy(_last_status, cur, sizeof(cur));
        _have_status = true;
        Serial.printf("[OSDK] MISSION status type=%u wp=%u st=%u err=%u (%u bytes)\n",
                      cur[0], cur[1], cur[2], cur[3], n);
        _queue(OsdkMissionMsg::STATUS, p, n, enc);
        return;
    }
    if (set == 0x02 && id == 0x04) {                 // Waypoint Event
        _mis_event++;
        _last_mis_event.a  = n > 2 ? p[2] : 0;
        _last_mis_event.wp = n > 3 ? p[3] : 0;
        _last_mis_event.st = n > 4 ? p[4] : 0;
        _last_mis_event.ms = millis();
        Serial.printf("[OSDK] MISSION event inc=%u wp=%u st=%u (%u bytes)\n",
                      n > 2 ? p[2] : 0, n > 3 ? p[3] : 0, n > 4 ? p[4] : 0, n);
        _queue(OsdkMissionMsg::EVENT, p, n, enc);
        return;
    }

    // Any other pair: report its first sighting only. 0x02/0x01 (lost of
    // flight control) or 0x02/0x02 (mobile pass-through) would land here.
    const uint16_t key = ((uint16_t)set << 8) | id;
    for (uint8_t k = 0; k < _seen_n; k++)
        if (_seen[k] == key) return;
    if (_seen_n >= SEEN_SZ) return;
    _seen[_seen_n++] = key;
    Serial.printf("[OSDK] first frame of cmd 0x%02X/0x%02X (%u bytes)\n", set, id, n);
    _queue(OsdkMissionMsg::NEW_CMD, p, n, enc);
}

void OsdkPositionSource::_queue(OsdkMissionMsg::Kind k, const uint8_t* p,
                                uint16_t n, uint8_t enc) {
    OsdkMissionMsg& m = _mq[_mq_head];
    m.kind    = k;
    m.ms      = millis();
    m.cmd_set = (enc == 0 && n > 0) ? p[0] : 0;
    m.cmd_id  = (enc == 0 && n > 1) ? p[1] : 0;
    m.enc     = enc;
    m.len     = n;
    memset(m.raw, 0, sizeof(m.raw));
    memcpy(m.raw, p, min<uint16_t>(sizeof(m.raw), n));
    _mq_head = (_mq_head + 1) % MQ_SZ;
    if (_mq_count < MQ_SZ) _mq_count++;             // full: oldest overwritten
}

bool OsdkPositionSource::popMission(OsdkMissionMsg& out) {
    if (_mq_count == 0) return false;
    out = _mq[(_mq_head + MQ_SZ - _mq_count) % MQ_SZ];
    _mq_count--;
    return true;
}

Position OsdkPositionSource::read() {
    Position p = _last;
    if (isStale()) p.valid = false;    // never hand back a frozen position
    return p;
}

bool OsdkPositionSource::hasFix() {
    return _last.valid && !isStale();
}

bool OsdkPositionSource::isStale(uint32_t max_age_ms) const {
    if (_last_fix_ms == 0) return true;
    return (millis() - _last_fix_ms) > max_age_ms;
}
