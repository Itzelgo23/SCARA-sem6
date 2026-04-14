#include "ServoStepper.h"

ServoStepper::ServoStepper()
{
    _counter = 0;
    _angle = 0;
}

ServoStepper::~ServoStepper()
{
    gpio_isr_handler_remove(_gpio_num);
}

void ServoStepper::setup(uint8_t pins[2], uint8_t ch, TimerConfig *stepper_timer, uint8_t microsteps, int max_freq)
{
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    _gpio_num = (gpio_num_t)pins[1];
    gpio_set_direction((gpio_num_t)pins[1], GPIO_MODE_INPUT);
    gpio_set_intr_type((gpio_num_t)pins[1], GPIO_INTR_POSEDGE);
    gpio_isr_handler_add((gpio_num_t)pins[1], [](void *arg)
                         { static_cast<ServoStepper *>(arg)->handler(); }, this);

    // LEDC setup AFTER
    stepper.setup(pins, ch, stepper_timer,max_freq);

    _microsteps = microsteps;
    _maxfreq = max_freq;
}

void ServoStepper::handler()
{
    _counter += _dir;

    /*int64_t current_pulse_t = esp_timer_get_time();    
    if (_prev_pulse_t != 0)
    {
        int64_t dt = current_pulse_t - _prev_pulse_t;

        if (dt > 0)
        {
            float step_angle = _deg_pulse / _microsteps;

            float speed = (step_angle * 1000000.0f) / dt;

            if (_freq > 0)
                _inst_speed = speed;
            else
                _inst_speed = -speed;
        }
    }
    _prev_pulse_t = current_pulse_t;

    if (_freq > 0)
        _counter++;
    else
        _counter--;
        */
}

float ServoStepper::set(float control, float error)
{

    if (control > _maxfreq)
        _freq = _maxfreq;
    else if (control < -_maxfreq)
        _freq = -_maxfreq;
    else
        _freq = control;

    _dir = (_freq > 0) ? 1 : (_freq < 0) ? -1 : 0;

    printf("Control: %.2f | Error: %.2f | Set freq: %.2f\n", control, error, _freq);
    stepper.setSpeed(_freq);
    return _freq;
}

void ServoStepper::setSpeed(float freq)
{
    stepper.setSpeed(freq);
}

float ServoStepper::getSpeed()
{
    return _inst_speed;
}

float ServoStepper::getAngle()
{
    _angle = _counter * _deg_pulse / _microsteps;
    printf("angle: %.2f | counter: %d\n", _angle, _counter);
    return _angle;
}

void ServoStepper::isHome()
{
    stepper.setSpeed(0);
    _counter = 0;
    _angle = 0;
    // printf("Homing done! counter: %d | angle: %.2f\n", _counter, _angle);
}
