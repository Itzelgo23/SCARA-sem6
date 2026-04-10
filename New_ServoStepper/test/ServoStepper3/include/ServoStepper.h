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
    float _counter = 0;
    float _deg_pulse = 1.8f;
    uint8_t _microsteps;
    int _angle = 0;
    int _target_angle = 0;
    int _freq = 0;
    uint8_t _minfreq = 0;
    uint64_t _dt_us; 

public:
    ServoStepper();
    ~ServoStepper();
    void setup(uint8_t pins[2], uint8_t ch, 
        TimerConfig *stepper_timer,uint64_t dt_us, 
        uint8_t microsteps, uint8_t min_freq);
    int begin(float control, float error, int max_freq);
    //int getAngle(int counter);
    int getAngle();
    void counter(int8_t dir);
    void setHome(int homefreq);
    void isHome();
};

#endif // __SERVOSTEPPER_H__