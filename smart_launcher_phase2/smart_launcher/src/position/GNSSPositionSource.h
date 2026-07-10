#pragma once
#include "IPositionSource.h"
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

class GNSSPositionSource : public IPositionSource {
public:
    // uart_num: 1 or 2 (hardware serial port)
    // rx_pin, tx_pin: GPIO numbers
    // baud: 9600 default for u-blox M8N
    GNSSPositionSource(uint8_t uart_num, int rx_pin, int tx_pin,
                       uint32_t baud = 9600);

    void     begin()  override;
    void     update() override;
    Position read()   override;
    bool     hasFix() override;

private:
    HardwareSerial* _serial;
    TinyGPSPlus     _gps;
    uint8_t         _uart_num;
    int             _rx_pin;
    int             _tx_pin;
    uint32_t        _baud;
    Position        _last;
};
