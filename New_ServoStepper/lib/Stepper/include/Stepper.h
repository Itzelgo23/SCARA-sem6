#ifndef __STEPPER_H__
#define __STEPPER_H__
#pragma once

#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <SimpleGPIO.h>
#include <SimplePWM.h>
#include <math.h>


typedef enum {
    Infinite_screw,
    Cupholder
}motor_type;

class Stepper
{
    public:
    Stepper();
    ~Stepper();
    void setup(uint8_t pin[2], const uint8_t ch, TimerConfig *stepper_timer);
    void setSpeed(int speed);

    private:
    SimpleGPIO dirctn;
    SimplePWM step;
    int _MOTOR_STEPS;
    int8_t _position;
    uint8_t _sequence;
    int _step;
    bool dir;
    //1.8 grados por paso 

};
#endif // __STEPPER_H__