#include <Stepper.h>

Stepper::Stepper()
{
}

void Stepper::setup(const uint8_t drive_pins[2], const uint8_t stepper_ch, TimerConfig *stepper_timer)
{
    dir.setup(drive_pins[0], GPO);
    step.setup(drive_pins[1], stepper_ch, stepper_timer);
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
