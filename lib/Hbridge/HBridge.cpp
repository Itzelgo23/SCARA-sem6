
#include "HBridge.h"

HBridge::HBridge()
{

};


HBridge::~HBridge()
{

};


void HBridge::setup(uint8_t pins[2], uint8_t channel[2],TimerConfig *config)
{
   pwm[0].setup(pins[0],channel[0],config);
   pwm[1].setup(pins[1],channel[1],config);

   /*    if (duty_offset > 0.001)
    {
        _mapping = true;
        _coeff[0] = duty_offset;
        _coeff[1] = (100.0f - duty_offset) / 100.0f;
    }
   */
}


void HBridge::setSpeed(float duty)
{
    if(duty<0){
        pwm[0].setDuty(-duty);
        pwm[1].setDuty(0);
    }
    if(duty>=0){
        pwm[0].setDuty(0);
        pwm[1].setDuty(duty);
    }

}



/*Implement the HBridge Class with the following methods:
setup: Configures the two PWM control pins, main PWM parameters, 
and the threshold value that initiates motor movement.
setSpeed: Receives a signed PWM value to control both speed and direction of rotation.*/