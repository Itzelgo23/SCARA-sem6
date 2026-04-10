#ifndef __PID_H__
#define __PID_H__

class PID
{
public:
    PID();
    void setup(float gains[3], float dt);
    float calculate(float error);
    void reset();
private:
float _Kp,_Ki,_Kd,_dt;
float _prev_error = 0;
float _integral = 0;

};
#endif // __PID_H__