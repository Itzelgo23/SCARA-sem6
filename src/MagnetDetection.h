#include "definitions.h"

MagStatus DetectMagnet(MotorTypes &state_in, MotorTypes &state_out)
{
    bool mag0_ok = (magE[0].MagnetDetection() == MD);
    bool mag1_ok = (magE[1].MagnetDetection() == MD);

    if (mag0_ok && mag1_ok)
    {
        state_out = state_in;
        return Detection;
    }
    else
        return Error;
}
