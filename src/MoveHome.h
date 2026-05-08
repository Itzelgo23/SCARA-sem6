#include "definitions.h"

bool MoveHome(float home_freq)
{
    pressed = LimitSwitch.get();
    if (pressed && !prev_pressed)
    {
        prev_pressed = pressed;
        Base_Motor.isHome();
        //pid[1].reset();
        ref[1] = 0.0;
        is_home = true;
        printf("Home position detected. Pressed: %d\n", pressed);
    }
    else
    {
        prev_pressed = pressed;
        Base_Motor.setSpeed(home_freq);
        is_home = false;
    }

    return is_home;
}