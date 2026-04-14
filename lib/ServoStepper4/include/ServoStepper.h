#ifndef __SERVOSTEPPER_H__
#define __SERVOSTEPPER_H__

#include <SimpleGPIO.h>
#include <Stepper.h>

class ServoStepper
{
private:
    SimpleGPIO pulse;
    SimpleGPIO sensor;
    Stepper stepper;
    volatile int _counter = 0;
    float _deg_pulse = 1.8f;
    uint8_t _microsteps;
    float _angle = 0.0;
    float _target_angle = 0.0;
    volatile float _freq = 0.0;
    float _maxfreq = 0.0;


public:
    ServoStepper();
    ~ServoStepper();
    void setup(uint8_t pins[2], uint8_t ch, TimerConfig *stepper_timer, uint8_t microsteps, int max_freq);
    float set(float control, float error);
    void setSpeed(float freq);
    float getAngle();
    void setHome(int homefreq);
    void isHome();
    void IRAM_ATTR handler();
};

#endif // __SERVOSTEPPER_H__