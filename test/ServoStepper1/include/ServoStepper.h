#ifndef __SERVOSTEPPER_H__
#define __SERVOSTEPPER_H__

#include <SimpleGPIO.h>
#include <Stepper.h>
#include <esp_timer.h>
#include <esp_attr.h>
#include "driver/pcnt.h"

class ServoStepper
{
private:
    static void IRAM_ATTR isrHandler(void *arg);
    void IRAM_ATTR handleStep();
    SimpleGPIO pulse;
    SimpleGPIO sensor;
    Stepper stepper;
    int _step_pin;
    int _counter = 0;
    float _deg_pulse = 1.8f;
    uint8_t _microsteps = 4;
    int _angle = 0;
    int _target_angle = 0;
    int _freq = 0;
    uint8_t _minfreq = 0;

    pcnt_unit_t _pcnt_unit = PCNT_UNIT_0;
    int16_t _pcnt_count = 0;

public:
    ServoStepper();
    ~ServoStepper();
    void setup(uint8_t pins[2], uint8_t ch, TimerConfig *stepper_timer, uint8_t microsteps, uint8_t min_freq);
    int begin(float control, float error, int max_freq);
    // int getAngle(int counter);
    int getAngle();
    void counter(int8_t dir);
    void setHome(int homefreq);
    void isHome();
};

#endif // __SERVOSTEPPER_H__