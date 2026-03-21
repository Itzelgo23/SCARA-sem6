#include "definitions.h"

void PIDmotors(int move_ref, uint8_t robot_section, float &control_out, float &error_out)
{
    int i = robot_section - 1; // 0-3 index
    int measurement;
    if (i < 0 || i > 3)
    {
        printf("Invalid motor index\n");
        control_out = 0;
        error_out = 0;
        return;
    }

    // correct sensor based on motor type
    if (robot_section == Base || robot_section == Shoulder)
    {
        magE[i].readRawAngle();
        measurement = magE[i].getTotalAngle();
        printf("Stepper angle: %d\n", measurement);
    }
    else if (robot_section == Elbow || robot_section == Wrist)
    {
        measurement = quadE[i].getAngle();
        printf("DC angle: %d\n", measurement);
    }
    else
    {
        printf("Incorrect robot selection type\n");
        control_out = 0;
        error_out = 0;
        return;
    }
    // error[i] = measurement[i] - ref;
    error_out = move_ref - measurement;

    // prev_error[i] = error[i];
    control_out = pid[i].calculate(error_out);
    printf("Error: %d| Control: %d\n", error_out,control_out);

    prev_error[i] = error_out;

}