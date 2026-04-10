#include "Stepper.h"

Stepper::Stepper()
{
}

Stepper::~Stepper()
{
}

void Stepper::setup(uint8_t pin[2], const uint8_t ch, TimerConfig *stepper_timer, float max_freq)
{
    dirctn.setup(pin[0], GPIO_MODE_OUTPUT);
    step.setup(pin[1], ch, stepper_timer);
    _maxfreq = max_freq;

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
float Stepper::set(float control, float error)
{

    if (control > _maxfreq)
        _freq = _maxfreq;
    else if (control < -_maxfreq)
        _freq = -_maxfreq;
    else
        _freq = control;

    setSpeed(_freq);
    return _freq;
}

