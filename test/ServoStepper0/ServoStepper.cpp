#include "ServoStepper.h"

ServoStepper::ServoStepper()
{
    _counter = 0;
    _angle = 0;
}

ServoStepper::~ServoStepper()
{
}

void ServoStepper::setup(uint8_t pins[2], uint8_t pulse_pin, uint8_t ch, TimerConfig *stepper_timer, void (*interrupt)(void *), uint64_t debounce, uint8_t microsteps, uint8_t min_freq)
{
    stepper.setup(pins, ch, stepper_timer);
    pulse.setup(pulse_pin, GPI, GPIO_PULLDOWN_ONLY);
    pulse.addInterrupt(GPIO_INTR_POSEDGE, interrupt, debounce);
    _microsteps = microsteps;
    _minfreq = min_freq;
}

float ServoStepper::begin(float control, float error, int max_freq)
{

    
    /*if (fabs(error) < 0.45f)
    {
        _freq = 0;
        //printf("Target reached! Angle: %d | Error: %.2f | Control: %f\n", _angle, error, control);
    }*/

    // else
    //{
    if (control > max_freq)
        _freq = max_freq;
    // else
    else if (control < -max_freq)
        _freq = -max_freq;
    else
        _freq = control;
    //}

    stepper.setSpeed(_freq);
    // Mostrar información de control
    // printf("Angle: %d | Error: %.2f | Control: %f\n",_angle, error, control);
    return _freq;
}

// int ServoStepper::getAngle(int counter)

void ServoStepper::setSpeed(float freq)
{
    stepper.setSpeed(freq);
}

float ServoStepper::getAngle()
{
    _angle = _counter * _deg_pulse / _microsteps;
    //_angle = counter * _deg_pulse / _microsteps;
    // printf("angle: %d\n", _angle);
    return _angle;
}

void ServoStepper::counter(int8_t dir)
{
    _counter += dir;
}

void ServoStepper::setHome(int homefreq)
{
    stepper.setSpeed(homefreq);
}

void ServoStepper::isHome()
{
    stepper.setSpeed(0);
    _counter = 0;
    _angle = 0;
    // printf("Homing done! counter: %d | angle: %d\n", _counter, _angle);
}
