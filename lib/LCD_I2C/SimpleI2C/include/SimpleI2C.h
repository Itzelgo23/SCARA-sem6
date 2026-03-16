#ifndef _SIMPLE_I2C_H
#define _SIMPLE_I2C_H

#include "driver/i2c.h"

class SimpleI2C {
public:
    void setup(uint8_t address);
    esp_err_t write(uint8_t *data, size_t size);
    esp_err_t read(uint8_t *data_in, size_t size_in, uint8_t *data_out, size_t size_out);

private:
    uint8_t _address;
};

#endif