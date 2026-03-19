#ifndef __HBRIDGE_H__
#define __HBRIDGE_H__
#include <SimplePWM.h>
#include <SimplePWM.h>


class HBridge
{
private:
    SimplePWM pwm[2];
public:
    HBridge();
    ~HBridge();
    void setup(uint8_t pins[2], uint8_t channel[2],TimerConfig *config);
    void setSpeed(float duty);
};

#endif // __HBRIDGE_H__
/*Implement the HBridge Class with the following methods:
setup: Configures the two PWM control pins, main PWM parameters, and the threshold value that initiates motor movement.
setSpeed: Receives a signed PWM value to control both speed and direction of rotation.*/
