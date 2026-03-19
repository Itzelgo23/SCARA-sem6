#ifndef __STEPPER_H__
#define __STEPPER_H__

#include <SimpleGPIO.h>
#include <SimplePWM.h>
#include <math.h>

class Stepper
{
public:
    Stepper();
    void setup(const uint8_t drive_pins[2], const uint8_t stepper_ch, TimerConfig *stepper_timer);
    void setSpeed(float speed);
    //void SetDuty(float duty);

private:
    SimpleGPIO dir;
    SimplePWM step;

};

#endif // __STEPPER_H__