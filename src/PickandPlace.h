#include "definitions.h"
#include "Kinematics.h"
#include "PIDmotors.h"
#include "cm2deg.h"

enum PickState
{
    PICK_MOVE_UP,
    PICK_MOVE_DOWN,
    PICK_CLOSE_GRIPPER,
    PICK_FINISHED
};
static PickState pickState = PICK_MOVE_UP;

enum PlaceState
{
    PLACE_MOVE_UP,
    PLACE_MOVE_DOWN,
    PLACE_OPEN_GRIPPER,
    PLACE_FINISHED
};
static PlaceState placeState = PLACE_MOVE_UP;

bool setPick(float pick_coords1, float pick_coords2, float pick_coords3)
{
    float location[3] = {pick_coords1, pick_coords2, pick_coords3};

    getIK(location, L1, L2, num_solutions, solutions);

    PIDmotors(solutions[0][0], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
    PIDmotors(solutions[0][1], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
    PIDmotors(solutions[0][2], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);

    Shoulder_Motor.set(control[1], error[1]);
    Elbow_Motor.setSpeed(control[2]);
    Wrist_Motor.setSpeed(control[3]);

    switch (pickState)
    {
    case PICK_MOVE_UP:

        angle_AS5600 = cm2deg(5.0, 8.0f);

        PIDmotors(angle_AS5600,
                  Base,
                  control[0],
                  error[0],
                  angle_S[0],
                  speed_S[0]);

        Base_Motor.set(control[0], error[0]);

        //printf("Pick UP error=%.2f\n", error[0]);

        if (fabs(error[0]) < 2.0f)
        {
            printf("Pick reached UP\n");
            pickState = PICK_MOVE_DOWN;
        }

        break;

    case PICK_MOVE_DOWN:

        angle_AS5600 = cm2deg(0.0, 8.0f);

        PIDmotors(angle_AS5600,
                  Base,
                  control[0],
                  error[0],
                  angle_S[0],
                  speed_S[0]);

        Base_Motor.set(control[0], error[0]);

        //printf("Pick DOWN error=%.2f\n", error[0]);

        if (fabs(error[0]) < 2.0f)
        {
            printf("Pick reached DOWN\n");
            pickState = PICK_CLOSE_GRIPPER;
        }

        break;

    case PICK_CLOSE_GRIPPER:

        Gripper_Motor.set(1);

        printf("Pick: Gripper closed\n");
        vTaskDelay(pdMS_TO_TICKS(1000)); // Esperar a que el gripper se cierre
        pickState = PICK_FINISHED;

        break;

    case PICK_FINISHED:

        pickState = PICK_MOVE_UP; // preparar siguiente pick
        return true;
    }

    return false;
}

bool setPlace(float place_coords[3])
{
    getIK(place_coords, L1, L2, num_solutions, solutions);

    PIDmotors(solutions[0][0], Shoulder,
              control[1], error[1],
              angle_S[1], speed_S[1]);

    PIDmotors(solutions[0][1], Elbow,
              control[2], error[2],
              angle_DC[0], speed_DC[0]);

    PIDmotors(solutions[0][2], Wrist,
              control[3], error[3],
              angle_DC[1], speed_DC[1]);

    Shoulder_Motor.set(control[1], error[1]);
    Elbow_Motor.setSpeed(control[2]);
    Wrist_Motor.setSpeed(control[3]);

    switch (placeState)
    {
    case PLACE_MOVE_UP:

        angle_AS5600 = cm2deg(5.0f, 8.0f);

        PIDmotors(angle_AS5600,
                  Base,
                  control[0],
                  error[0],
                  angle_S[0],
                  speed_S[0]);

        Base_Motor.set(control[0], error[0]);

        //printf("Place UP error=%.2f\n", error[0]);

        if (fabs(error[0]) < 2.0f)
        {
            printf("Place reached UP\n");
            placeState = PLACE_MOVE_DOWN;
        }

        break;

    case PLACE_MOVE_DOWN:

        angle_AS5600 = cm2deg(0.0f, 8.0f);

        PIDmotors(angle_AS5600,
                  Base,
                  control[0],
                  error[0],
                  angle_S[0],
                  speed_S[0]);

        Base_Motor.set(control[0], error[0]);

        //printf("Place DOWN error=%.2f\n", error[0]);

        if (fabs(error[0]) < 2.0f)
        {
            printf("Place reached DOWN\n");
            placeState = PLACE_OPEN_GRIPPER;
        }

        break;

    case PLACE_OPEN_GRIPPER:

        printf("Place: Opening gripper\n");

        Gripper_Motor.set(0);

        vTaskDelay(pdMS_TO_TICKS(1000));

        placeState = PLACE_FINISHED;

        break;

    case PLACE_FINISHED:

        printf("Place finished\n");

        placeState = PLACE_MOVE_UP; // reset para la siguiente vez

        return true;
    }

    return false;
}