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
// VELOCITY/ANG_RATE off in DJI Assistant 2 — which is the recommended way to
// cut this link from 231 bytes to ~60 — slides POSITION earlier. A hardcoded
// 72 would silently decode garbage the moment that config changed.
static const uint8_t FIELD_SZ[6] = {
    15,   // bit 0  TIME
    16,   // bit 1  QUATERNION  (4 x float32)
    12,   // bit 2  ACCEL       (3 x float32)
    13,   // bit 3  VELOCITY    (3 x float32 + health)
    12,   // bit 4  ANG_RATE    (3 x float32)
    25    // bit 5  POSITION    (float64 lat, float64 lon, float32 alt,
};        //                     float32 height, uint8 health)

static const uint8_t  FLAG_POSITION = 5;
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
                      "lat=%.7f lon=%.7f health=%u fix=%s\n",
                      (unsigned long)_frames, (unsigned long)_crc_err,
                      (unsigned long)_resyncs, _last_flag,
                      _last.lat_deg, _last.lon_deg, _last_health,
                      hasFix() ? "YES" : "no");
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
        _decode(fr + HDR_LEN, length - HDR_LEN - 4);
        i += length;
    }

    if (i > 0) {                        // shuffle the unconsumed tail down
        _len -= i;
        memmove(_buf, _buf + i, _len);
    }
}

bool OsdkPositionSource::_decode(const uint8_t* p, uint16_t n) {
    if (n < 4) return false;
    if (p[0] != 0x02 || p[1] != 0x00) return false;      // not broadcast

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

    // A frame arriving is NOT a fix. Indoors the A3 broadcasts happily with
    // lat/lon at exactly 0.0 and health 0 — accepting that would hand drop
    // logic a "valid" position off the coast of Africa. Require a non-zero
    // fix AND a usable health value.
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
