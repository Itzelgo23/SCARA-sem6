#ifndef __STEPPER_H__
#define __STEPPER_H__
#pragma once

#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <SimpleGPIO.h>
#include <SimplePWM.h>
#include <math.h>

class Stepper
{
    public:
    Stepper();
    ~Stepper();
    void setup(uint8_t pin[2], const uint8_t ch, TimerConfig *stepper_timer, float max_freq);
    void setSpeed(int speed);
    float set(float control, float error);

    private:
    SimpleGPIO dirctn;
    SimplePWM step;
    float _freq = 0.0;
    float _maxfreq;
    //1.8 grados por paso 

};
#endif // __STEPPER_H__