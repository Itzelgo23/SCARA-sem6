#include "definitions.h"
#include "cstring"
#include "cmath"

double Rad2Deg(double radians)
{
    return radians * (180.0 / M_PI);
}

double Deg2Rad(float deg)
{
    return deg * (M_PI / 180.0f);
}

void multiplyMatrices(float A[4][4], float B[4][4], float C[4][4])
{
    float temp[4][4] = {0};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                temp[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    std::copy(&temp[0][0], &temp[0][0] + 16, &C[0][0]);
}

void getHM(float DH_parameters[4], float (&H)[4][4])
{
    _t = Deg2Rad(DH_parameters[0]);
    _d = DH_parameters[1];
    _a = Deg2Rad(DH_parameters[2]);
    _r = DH_parameters[3];

    H[0][0] = cosf(_t);
    H[0][1] = -sinf(_t) * cosf(_a);
    H[0][2] = sinf(_t) * sinf(_a);
    H[0][3] = _r * cosf(_t);

    H[1][0] = sinf(_t);
    H[1][1] = cosf(_t) * cosf(_a);
    H[1][2] = -cosf(_t) * sinf(_a);
    H[1][3] = _r * sinf(_t);

    H[2][0] = 0.0f;
    H[2][1] = sinf(_a);
    H[2][2] = cosf(_a);
    H[2][3] = _d;

    H[3][0] = 0.0f;
    H[3][1] = 0.0f;
    H[3][2] = 0.0f;
    H[3][3] = 1.0f;
}

void rotm2eul(float T[4][4], float (&euler)[3])
{

    float r11 = T[0][0];
    float r21 = T[1][0];
    float r31 = T[2][0];
    float r32 = T[2][1];
    float r33 = T[2][2];

    // pitch
    euler[1] = atan2(-r31, sqrt(r32 * r32 + r33 * r33));

    // Avoid singularity
    if (fabs(cos(euler[1])) > 1e-6)
    {                               // pitch
        euler[2] = atan2(r32, r33); // roll
        euler[0] = atan2(r21, r11); // yaw
    }
    else
    {
        // Gimbal lock case
        euler[2] = 0;
        euler[0] = atan2(-T[0][1], T[1][1]);
    }

    const float rad2deg = 180.0 / M_PI;

    euler[2] *= rad2deg;
    euler[1] *= rad2deg;
    euler[0] *= rad2deg;
}

void getFK(float lengths[5], float d2, float q1, float q3, float q4, float (&T_final)[4][4], float (&euler)[3])
{
    // SCARA dimensions
    float d1 = lengths[0]; // height base
    float L1 = lengths[1]; // arm1
    float L2 = lengths[2]; // arm2
    float b1 = lengths[3]; // height between arms
    float b2 = lengths[4]; // height gripper

    // dh parameters
    float dh[5][4] = {
    // theta, d, alpha, r
        {q1, d1, 0.0, 0.0}, // Shoulder -- rotation and preestablished height to base
        {0.0, d2, 0.0, 0.0},   // Base -- prismatic
        {q3, 0.0, 0.0, L1},  // Elbow -- rotation and arm length
        {q4, -b1, 0.0, L2},  // Wrist -- rotation, height between arms and arm length
        {0.0, -b2, 0.0, 0.0}  //Gripper -- no movement, just height from arm to gripper
    };

    float I[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};
    std::copy(&I[0][0], &I[0][0] + 16, &T_final[0][0]);

    float H_temp[4][4];
    for (int i = 0; i < 5; i++)
    {
        getHM(dh[i], H_temp);
        multiplyMatrices(T_final, H_temp, T_final);
    }
    rotm2eul(T_final, euler);
}

void getIK(float op_vars[4], float DH_parameters[2][4], float &num_solutions, float (&solutions)[2][4])
{
    std::memset(solutions, 0, sizeof(solutions));

    x = op_vars[0];
    y = op_vars[1];
    z = op_vars[2];
    tool_angle = op_vars[3];

    _L1 = DH_parameters[0][3];
    _L2 = DH_parameters[1][3];

    _p = sqrt((_L1 * _L1) + (_L2 * _L2));

    if (_p > _L1 + _L2 || _p < fabs(_L1 - _L2))
    {
        num_solutions = 0; // No solutions
        return;
    }

    _gamma = atan2(y, x);
    _alpha = acos((_L1 * _L1 + _p * _p - _L2 * _L2) / (2 * _L1 * _p));
    _beta = acos((_L1 * _L1 + _L2 * _L2 - _p * _p) / (2 * _L1 * _L2));

    solutions[0][0] = z;
    solutions[0][1] = Rad2Deg(_gamma - _alpha);
    solutions[0][2] = Rad2Deg(M_PI - _beta);
    solutions[0][3] = tool_angle - solutions[0][1] - solutions[0][2];

    if (_p == _L1 + _L2 || _p == fabs(_L1 - _L2))
    {
        num_solutions = 1; // One solution
        return;
    }

    solutions[1][0] = z;
    solutions[1][1] = Rad2Deg(_gamma + _alpha);
    solutions[1][2] = Rad2Deg(_beta - M_PI);
    solutions[1][3] = tool_angle - solutions[1][1] - solutions[1][2];
    num_solutions = 2; // Two solutions
}

void findBestSolution(float solutions[2][4], float current[4],int &index)
{  
     //solutions[2][4] from getIK, current[4] from getFK, index is the output for the best solution

    /*calculate angle differences for each solution and find the one with the smallest total difference
    give the most importance to the base as it is the slowest to move, then shoulder, elbow and wrist

    Elbow cannot move more than around 130° (if 0 is paralell to system)
    Maybe Shoulder cannot turn more than 270°? -- wire control, faster movement
    Base cannot move less than 0 and more than cm2deg(15) //cm
    */

    float weights[4] = {4.0, 10.0, 2.0, 1.0};//Base, Shoulder, Elbow, Wrist

    float bestCost = 1e9;
    index = -1;

    for (int i = 0; i < 2; i++)
    {
        float d0 = fabs(solutions[i][0] - current[0]);
        float d1 = fabs(solutions[i][1] - current[1]);
        float d2 = fabs(solutions[i][2] - current[2]);
        float d3 = fabs(solutions[i][3] - current[3]);

        // ===== JOINT LIMITS =====
        // base prismatic
        if (solutions[i][0] < 0.0 || solutions[i][0] > 6750.0) // 15cm in ° for a picth of 8mm
            //error state, return without setting index to ensure no movement
            continue;

        // elbow ±130°
        if (fabs(solutions[i][2]) > 130.0)
            //error state, return without setting index to ensure no movement
            continue;

        // shoulder example ±135°
        if (fabs(solutions[i][1]) > 135.0)
            //error state, return without setting index to ensure no movement
            continue;

        //finds best solution based on weighted distance to current position, giving more importance to shoulder and base

        float cost =
            weights[0] * d0 +
            weights[1] * d1 +
            weights[2] * d2 +
            weights[3] * d3;

        if (cost < bestCost)
        {
            bestCost = cost;
            index = i;
            //returns index between 0 and 1 for the best solution, -1 if no valid solution
        }
    }
}