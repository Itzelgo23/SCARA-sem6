#include "definitions.h"

void PIDmotors(float move_ref, uint8_t robot_section, float &control_out, float &error_out, float &measurement_out, float &speed_out)
{
    int i = robot_section - 1; // 0-3 index
    int j; // for DC motor selection
    float measurement;
    if (i < 0 || i > 3)
    {
        printf("Invalid motor index\n");
        control_out = 0.0;
        error_out = 0.0;
        return;
    }

    // correct sensor based on motor type
    if (robot_section == Base)
    {
        measurement = Base_Motor.getAngle();
        error_out = move_ref - measurement;
        speed_out = Base_Motor.getSpeed();
        //printf("Stepper angle: %.2f\n", measurement);
    }
        
    else if(robot_section == Shoulder)
    {
        magE.readRawAngle();
        measurement = magE.getTotalAngle();
        error_out = move_ref - measurement;
        speed_out = magE.getSpeed();

        //printf("Stepper angle: %.2f\n", measurement);
    }
    else if (robot_section == Elbow || robot_section == Wrist)
    {
        if (robot_section == Elbow)
            j = 0;
        if (robot_section == Wrist)
            j = 1;
        measurement = quadE[j].getAngle();
        error_out = move_ref - measurement;
        speed_out = quadE[j].getSpeed();
        //printf("DC angle: %.2f\n", measurement);
    }
    else
    {
        printf("Incorrect robot selection type\n");
        control_out = 0;
        error_out = 0;
        return;
    }
    measurement_out = measurement;
    
    
    control_out = pid[i].calculate(error_out);
    //printf("i: %d\n", i);
    //printf("Error: %.2f| Control: %.2f\n", error_out, control_out);

    prev_error[i] = error_out;
}