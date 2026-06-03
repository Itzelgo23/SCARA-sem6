#include "definitions.h"
#include "Kinematics.h"
#include "PIDmotors.h"
#include "cm2deg.h"

bool setPick(float pick_coords1,float pick_coords2,float pick_coords3)
{
    bool Pick = false;
    float location[3] = {pick_coords1, pick_coords2, pick_coords3};
    Gripper_Motor.set(0);
    angle_AS5600 = cm2deg(5, 8.0f);
    PIDmotors(angle_AS5600, Base, control[0], error[0], angle_S[0], speed_S[0]);
    getIK(location, L1, L2, num_solutions, solutions);
    PIDmotors(solutions[0][0], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
    PIDmotors(solutions[0][1], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
    PIDmotors(solutions[0][2], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);
    Base_Motor.set(control[0], error[0]);
    Shoulder_Motor.set(control[1], error[1]);
    Elbow_Motor.setSpeed(control[2]);
    Wrist_Motor.setSpeed(control[3]);
    if (fabs(error[1]) < 5.0f && fabs(error[2]) < 5.0f && fabs(error[3]) < 5.0f)
    {
        angle_AS5600 = cm2deg(0, 8.0f);
        PIDmotors(angle_AS5600, Base, control[0], error[0], angle_S[0], speed_S[0]);
        Gripper_Motor.set(1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        Pick = true;
    }
    return Pick;
}

bool setPlace(float place_coords[3])
{
    bool Place = false;
    angle_AS5600 = cm2deg(5, 8.0f);
    PIDmotors(angle_AS5600, Base, control[0], error[0], angle_S[0], speed_S[0]);
    getIK(place_coords, L1, L2, num_solutions, solutions);
    PIDmotors(solutions[0][0], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
    PIDmotors(solutions[0][1], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
    PIDmotors(solutions[0][2], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);
    Base_Motor.set(control[0], error[0]);
    Shoulder_Motor.set(control[1], error[1]);
    Elbow_Motor.setSpeed(control[2]);
    Wrist_Motor.setSpeed(control[3]);
    if (fabs(error[1]) < 5.0f && fabs(error[2]) < 5.0f && fabs(error[3]) < 5.0f)
    {
        angle_AS5600 = cm2deg(0, 8.0f);
        PIDmotors(angle_AS5600, Base, control[0], error[0], angle_S[0], speed_S[0]);
        Gripper_Motor.set(0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        Place = true;
    }
    return Place;
}