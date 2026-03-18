#ifndef _TCS34725_H
#define _TCS34725_H

#include "SimpleI2C.h"

class TCS34725
{
public:
    TCS34725(SimpleI2C &i2c);
    void begin();
    void setIntegrationTime(uint8_t atime);
    void setGain(uint8_t gain);
    void readRGB(uint8_t &r, uint8_t &g, uint8_t &b);                 // ← 8 bits
    void readRaw(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b); // ← NUEVO: valores sin normalizar

private:
    SimpleI2C &_i2c;
    static constexpr uint8_t ADDRESS = 0x29;
    static constexpr uint8_t COMMAND_BIT = 0x80;

    void write8(uint8_t reg, uint8_t value);
    void read16(uint8_t reg, uint16_t &value);
};

#endif