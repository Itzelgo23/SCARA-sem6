#ifndef MAGNETDETECTION_H
#define MAGNETDETECTION_H
#include "definitions.h"

MagStatus DetectMagnet(MotorTypes &state_in, MotorTypes &state_out)
{

    if (magE.MagnetDetection() == MD)
    {
        state_out = state_in;
        return Detection;
    }
    else
        return Error;
}

#endif // MAGNETDETECTION_H
