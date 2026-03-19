#include <PID.h>

PID::PID()
{
}

void PID::setup(float gains[3], float dt)
{
    _Kp = gains[0];
    _Ki = gains[1];
    _Kd = gains[2];
    _dt = dt;
}

float PID::calculate(float error)
{
    float u = _Kp*error;
    u += _Kd*(error - _prev_error)/_dt;
    _integral += (_dt / 2)*(error+_prev_error);
    u += _Ki*_integral;
    _prev_error = error;
    return u;
}