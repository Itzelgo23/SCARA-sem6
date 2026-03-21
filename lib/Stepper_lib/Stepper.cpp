#include <Stepper.h>

Stepper::Stepper()
{

}

void Stepper::setup(const uint8_t drive_pins[2], const uint8_t stepper_ch, TimerConfig *stepper_timer, int freq_range[2])
{
    dir.setup(drive_pins[0], GPO);
    step.setup(drive_pins[1], stepper_ch, stepper_timer);
    _freqRange[0] = freq_range[0];
    _freqRange[1] = freq_range[1];
}

void Stepper::setSpeed(float speed)
{
    step.setDuty(50);
    if (speed > 0)
        dir.set(1);
    else if (fabs(speed)< 0.1){
        step.setDuty(0);
        return;
    }
    else
    {
        dir.set(0);
        speed = -speed;
    }
    uint32_t speeed = speed;
    printf("%ld\n", speeed);
    step.setFrequency((speeed));
}

int Stepper::begin(float control, float error)
{

    if (abs(error) < 1)
    {
        _freq = 0;
        //printf("Target reached! Angle: %d | Error: %.2f | Control: %f\n", _angle, error, control);
    }

    else
    {
        int freq_max = (abs(error) < 50) ? _freqRange[0] : _freqRange[1];
        if (control > _freqRange[1]){
            control = _freqRange[1];}
        if (control < -_freqRange[1]){
            control = -_freqRange[1];}
        _freq = control;
        printf("frequency: %d\n", _freq);
    }

    setSpeed(_freq);
    // Mostrar información de control
    //printf("Angle: %d | Error: %.2f | Control: %f\n",_angle, error, control);
    return _freq;
}
