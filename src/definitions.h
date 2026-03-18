#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <AS5600.h>
#include <SimpleTimer.h>
#include <SimpleI2C.h>


enum MagStatus
{
    Detection = 0,
    Error = 1
};

#pragma region I2C defines
uint8_t SDA_PIN = 21;
uint8_t SCL_PIN = 22;
#pragma endregion

#pragma region Library defines
SimpleI2C i2c;
AS5600 magEncoder;
SimpleTimer timer;
#pragma endregion

#pragma region PIN defines
#pragma endregion

#pragma region AS5600 defines
uint8_t mag_status = 0;
#pragma endregion

#pragma region Time Polling defines
uint64_t prev = 0, current = 0;
uint64_t dt_us = 10000; // 10ms
#pragma endregion

MagStatus status;



#endif // __DEFINITIONS_H__