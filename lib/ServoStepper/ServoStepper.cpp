#include "ServoStepper.h"

ServoStepper::ServoStepper()
{
    _counter = 0;
    _angle = 0;
}

ServoStepper::~ServoStepper()
{
}

void ServoStepper::setup(uint8_t pins[2], uint8_t ch, float step_angle, TimerConfig *stepper_timer, uint8_t microsteps, uint8_t min_freq,int64_t timeout_us);
{
    stepper.setup(pins, ch, stepper_timer);

    _gpio_num[0] = (gpio_num_t)pins[0];
    _step_angle = step_angle;
    _timeout_us = timeout_us;
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pin_bit_mask = (1ULL << _gpio_num[0]);
    gpio_config(&io_conf);
    //gpio_uninstall_isr_service();
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(_gpio_num[0], [](void *arg)
                         { static_cast<ServoStepper *>(arg)->handler(); }, this);
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.pin_bit_mask = (1ULL << _gpio_num[1]);
    gpio_config(&io_conf);

    _microsteps = microsteps;
    _minfreq = min_freq;
}

int ServoStepper::begin(float control, float error, int max_freq)
{

    if (abs(error) < 1)
    {
        _freq = 0;
        //printf("Target reached! Angle: %d | Error: %.2f | Control: %f\n", _angle, error, control);
    }

    else
    {
        int freq_max = (abs(error) < 50) ? _minfreq : max_freq;
        if (control > freq_max){
            control = freq_max;}
        if (control < -freq_max){
            control = -freq_max;}
        _freq = control;
    }

    stepper.setSpeed(_freq);
    // Mostrar información de control
    //printf("Angle: %d | Error: %.2f | Control: %f\n",_angle, error, control);
    return _freq;
}

//int ServoStepper::getAngle(int counter)

int ServoStepper::getAngle()
{
    _angle = _counter * _deg_pulse / _microsteps;
    //_angle = counter * _deg_pulse / _microsteps;
    //printf("angle: %d\n", _angle);
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
    //printf("Homing done! counter: %d | angle: %d\n", _counter, _angle);
}
