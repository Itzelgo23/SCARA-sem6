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
    int _counter = 0;
    float _deg_pulse = 1.8f;
    uint8_t _microsteps;
    float _angle = 0.0;
    float _target_angle = 0.0;
    float _freq = 0.0;
    uint8_t _minfreq = 0;

public:
    ServoStepper();
    ~ServoStepper();
    void setup(uint8_t pins[2],uint8_t pulse_pin, uint8_t ch, TimerConfig *stepper_timer, void (*interrupt)(void *), uint64_t debounce, uint8_t microsteps, uint8_t min_freq);
    float begin(float control, float error, int max_freq);
    void setSpeed(float freq);
    //int getAngle(int counter);
    float getAngle();
    void counter(int8_t dir);
    void setHome(int homefreq);
    void isHome();
};

#endif // __SERVOSTEPPER_H__