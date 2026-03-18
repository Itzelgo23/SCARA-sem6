#include "TCS34725.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

TCS34725::TCS34725(SimpleI2C &i2c) : _i2c(i2c) {}

void TCS34725::begin()
{
    _i2c.setup(ADDRESS);
    write8(0x00, 0x01); // PON
    vTaskDelay(pdMS_TO_TICKS(3));
    write8(0x00, 0x03); // PON + AEN
    vTaskDelay(pdMS_TO_TICKS(700));
}

void TCS34725::setIntegrationTime(uint8_t atime)
{
    write8(0x01, atime);
}

void TCS34725::setGain(uint8_t gain)
{
    write8(0x0F, gain);
}
void TCS34725::readRGB(uint8_t &r, uint8_t &g, uint8_t &b)
{
    uint16_t red, green, blue, clear;
    read16(0x14, clear);
    read16(0x16, red);
    read16(0x18, green);
    read16(0x1A, blue);

    //valores de calibración manual (ajústalos según tu entorno)
    uint16_t r_min = 214, r_max = 25506;
    uint16_t g_min = 253, g_max = 44032;
    uint16_t b_min = 233, b_max = 38761;

    // regla de tres calibrada
    r = (red - r_min) * 255 / (r_max - r_min);
    g = (green - g_min) * 255 / (g_max - g_min);
    b = (blue - b_min) * 255 / (b_max - b_min);

     // limitar a 0–255
    if (r > 255)
        r = 255;
    else if (r <= 0)
        r = 0;
    if (g > 255)
        g = 255;
    else if (g <= 0)
        g = 0;
    if (b > 255)
        b = 255;
    else if (b <= 0)
        b = 0;
}
 

void TCS34725::readRaw(uint16_t &c, uint16_t &r, uint16_t &g, uint16_t &b)
{
    read16(0x14, c);
    read16(0x16, r);
    read16(0x18, g);
    read16(0x1A, b);
}

void TCS34725::write8(uint8_t reg, uint8_t value)
{
    uint8_t cmd = COMMAND_BIT | reg;
    uint8_t data[] = {cmd, value};
    _i2c.write(data, 2);
}

void TCS34725::read16(uint8_t reg, uint16_t &value)
{
    uint8_t cmd = COMMAND_BIT | reg;
    uint8_t buffer[2];
    _i2c.read(&cmd, 1, buffer, 2);
    value = buffer[1] << 8 | buffer[0];
}
