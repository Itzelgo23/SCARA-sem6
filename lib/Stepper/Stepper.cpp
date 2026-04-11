#include "Stepper.h"

Stepper::Stepper()
{
    _MOTOR_STEPS = 200;
    _position = 0;
}

Stepper::~Stepper()
{
}

void Stepper::setup(uint8_t pin[2], const uint8_t ch, TimerConfig *stepper_timer)
{
    dirctn.setup(pin[0], GPIO_MODE_OUTPUT);
    step.setup(pin[1], ch, stepper_timer);

}

void Stepper::setSpeed(int speed)
{
    uint32_t speeed = speed;
    
    if (speed >= 5){
        dirctn.set(1);
        step.setDuty(10);
        step.setFrequency((speeed));
    }
    else if (fabs(speed) < 5)
    {
        step.setDuty(0);
        step.setFrequency(5);
    }
    else
    {
        dirctn.set(0);
        speed = -speed;
        step.setDuty(10);
        step.setFrequency((speed));
    }
    
    printf("%ld\n", speeed);
    
}

