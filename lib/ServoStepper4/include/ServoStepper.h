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
    volatile int _dir = 0;
    float _deg_pulse = 1.8f;
    uint8_t _microsteps;
    float _angle = 0.0;
    float _target_angle = 0.0;
    volatile float _freq = 0.0;
    float _maxfreq = 0.0;
    int64_t _prev_pulse_t = 0;
    int _inst_speed = 0;
    gpio_num_t _gpio_num;

    void IRAM_ATTR handler();

public:
    ServoStepper();
    ~ServoStepper();
    void setup(uint8_t pins[2], uint8_t ch, TimerConfig *stepper_timer, uint8_t microsteps, int max_freq);
    float set(float control, float error);
    void setSpeed(float freq);
    float getSpeed();
    float getAngle();
    void isHome();
};

#endif // __SERVOSTEPPER_H__