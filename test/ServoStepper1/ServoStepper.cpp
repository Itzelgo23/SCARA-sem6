#include "ServoStepper.h"

ServoStepper::ServoStepper()
{
    _counter = 0;
    _angle = 0;
}

ServoStepper::~ServoStepper()
{
}

void ServoStepper::setup(uint8_t pins[2], uint8_t ch, TimerConfig *stepper_timer, uint8_t microsteps, uint8_t min_freq)
{
    stepper.setup(pins, ch, stepper_timer);
    _step_pin = pins[1]; 
    

}

int ServoStepper::begin(float control, float error, int max_freq)
{

    if (abs(error) < 2)
    {
        _freq = 0;
        // printf("Target reached! Angle: %d | Error: %.2f | Control: %f\n", _angle, error, control);
    }

    else
    {
        int freq_max = (abs(error) < 50) ? _minfreq : max_freq;
        if (control > freq_max)
        {
            control = freq_max;
        }
        if (control < -freq_max)
        {
            control = -freq_max;
        }
        _freq = control;
    }
    printf("freq: %d\n", _freq);
    stepper.setSpeed(_freq);
    // Mostrar información de control
    // printf("Angle: %d | Error: %.2f | Control: %f\n",_angle, error, control);
    return _freq;
}

// int ServoStepper::getAngle(int counter)

int ServoStepper::getAngle()
{
    pcnt_get_counter_value(_pcnt_unit, &_pcnt_count);

    if (_freq < 0)
        _counter = -_pcnt_count;
    else
        _counter = _pcnt_count;

    _angle = _counter * _deg_pulse / _microsteps;

    printf("angle: %d\n", _angle);

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

    pcnt_counter_clear(_pcnt_unit);  // 🔥 reset real hardware

    _counter = 0;
    _angle = 0;
    // printf("Homing done! counter: %d | angle: %d\n", _counter, _angle);
}
