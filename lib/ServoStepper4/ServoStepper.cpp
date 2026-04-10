#include "ServoStepper.h"

ServoStepper::ServoStepper()
{
    _counter = 0;
    _angle = 0;
}

ServoStepper::~ServoStepper()
{
}

void ServoStepper::setup(uint8_t pins[2], uint8_t ch, TimerConfig *stepper_timer, uint8_t microsteps, int max_freq)
{
    
    stepper.setup(pins, ch, stepper_timer, max_freq);

    gpio_set_intr_type((gpio_num_t)pins[1], GPIO_INTR_POSEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add((gpio_num_t)pins[1], [](void *arg)
                         { static_cast<ServoStepper *>(arg)->handler(); }, this);
    _microsteps = microsteps;
    _maxfreq = max_freq;
}

void ServoStepper::handler()
{
    if (_freq > 0)
        _counter++;
    else
        _counter--;
}

float ServoStepper::set(float control, float error)
{

    if (control > _maxfreq)
        _freq = _maxfreq;
    else if (control < -_maxfreq)
        _freq = -_maxfreq;
    else
        _freq = control;

    stepper.setSpeed(_freq);
    return _freq;
}


void ServoStepper::setSpeed(float freq)
{
    stepper.setSpeed(freq);
}

float ServoStepper::getAngle()
{
    _angle = _counter * _deg_pulse / _microsteps;
    // printf("angle: %d\n", _angle);
    return _angle;
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
