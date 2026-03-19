#ifndef __LCD_I2C_H__
#define __LCD_I2C_H__

#include "SimpleI2C"
#include "SimpleGPIO"

#pragma once

class LCD_I2C
{
public:
    LCD_I2C() : _col(16), _row(2), _addr(0x27), _backlightPin(LCD_MASK_BACKLIGHT) {}
    ~LCD_I2C();
    SimpleI2C i2c;
    void setup();
    void clear();
    void setCursor(_col, _row);
    //void print();
    void backlight();
    void noBacklight();
    void writechar(char c);

private:
    SimpleGPIO ///???
    uint8_t _col;
    uint8_t _row;
    uint8_t _addr;
    uint8_t _backlightPin;
}

#endif // __LCD_I2C_H__