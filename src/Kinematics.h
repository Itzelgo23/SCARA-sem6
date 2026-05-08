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

void getFK(float lengths[3], float d1, float q2, float q3, float q4, float (&T_final)[4][4], float (&euler)[3])
{
    // SCARA dimensions
    float A0 = lengths[0]; // height base
    float A1 = lengths[1]; // arm1
    float A2 = lengths[2]; // arm2

    // dh parameters
    float dh[4][4] = {
        // theta, d, alpha, r
        {0.0, d1, 0.0, 0.0}, // P
        {q2, A0, 0.0, A1},   // R
        {q3, 0.0, 0.0, A2},  // R
        {q4, 0.0, 0.0, 0.0}  // R
    };


    // theta, d, alpha, r
    //Correct dh model, have to change code.
    /*
        {q1, d1, 0.0, 0.0}, // Shoulder
        {0.0, d2, 0.0, 0.0},   // Base
        {q2, 0.0, 0.0, L1},  // Elbow
        {q3, -b1, 0.0, L2},  // Wrist
        {0.0, -b2, 0.0, 0.0}  //Gripper
        */

    float I[4][4] = {
        {1, 0, 0, 0},
        {0, 1, 0, 0},
        {0, 0, 1, 0},
        {0, 0, 0, 1}};
    std::copy(&I[0][0], &I[0][0] + 16, &T_final[0][0]);

    float H_temp[4][4];
    for (int i = 0; i < 4; i++)
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

void findBestSolution(float solutions[2][4], float DH_parameters[2][4], float &index)
{
    // calculate angle differences for each solution and find the one with the smallest total difference
    // give the most importance to the base as it is the slowest to move, then shoulder, elbow and wrist
}