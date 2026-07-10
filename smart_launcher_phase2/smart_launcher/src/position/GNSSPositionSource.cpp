#include "GNSSPositionSource.h"

GNSSPositionSource::GNSSPositionSource(uint8_t uart_num, int rx_pin,
                                       int tx_pin, uint32_t baud)
    : _uart_num(uart_num), _rx_pin(rx_pin), _tx_pin(tx_pin), _baud(baud) {
    // Select hardware serial instance
    if (uart_num == 1)      _serial = &Serial1;
    else if (uart_num == 2) _serial = &Serial2;
    else                    _serial = &Serial1;   // fallback
}

void GNSSPositionSource::begin() {
    _serial->begin(_baud, SERIAL_8N1, _rx_pin, _tx_pin);
    Serial.printf("[GPS] UART%d started on RX=%d TX=%d @%d baud\n",
                  _uart_num, _rx_pin, _tx_pin, _baud);
}

void GNSSPositionSource::update() {
    while (_serial->available()) {
        char c = _serial->read();
        if (_gps.encode(c)) {
            // New sentence decoded — update last known position
            if (_gps.location.isValid()) {
                _last.lat_deg    = _gps.location.lat();
                _last.lon_deg    = _gps.location.lng();
                _last.alt_m      = _gps.altitude.isValid()
                                   ? (float)_gps.altitude.meters() : 0.0f;
                _last.hdop       = _gps.hdop.isValid()
                                   ? (float)_gps.hdop.value() / 100.0f : 99.0f;
                _last.satellites = _gps.satellites.isValid()
                                   ? (uint8_t)_gps.satellites.value() : 0;
                _last.valid      = true;
            }
        }
    }
}

Position GNSSPositionSource::read() {
    return _last;
}

bool GNSSPositionSource::hasFix() {
    return _last.valid && (_last.hdop < 5.0f) && (_last.satellites >= 4);
}
