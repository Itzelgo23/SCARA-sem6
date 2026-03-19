#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <esp_timer.h>
#include <esp_xt_wdt.h>
#include <AS5600.h>
#include <SimpleTimer.h>
#include <SimpleI2C.h>
#include <Stepper.h>
#include <QuadratureEncoder.h>
#include <PID.h>
#include <HBridge.h>


//UART2 RX 16 y TX 17
//Wrist and Gripper in second (SLAVE) ESP32

//Pins used: 13,16,17,18,19,21,22,25,26,27,32,33,35
//Pins left: 4,23

enum MagStatus
{
    Detection = 0,
    Error = 1
};
MagStatus status;

enum MotorTypes
{
    Initial = 0,
    Base = 1, //Stepper1
    Shoulder = 2, //Stepper2
    Elbow = 3, //DC1
    Wrist = 4, //DC2
    Error = 5
};
MotorTypes motor_case;

#pragma region TIMERS defines
TimerConfig Base_config = {
    .timer = LEDC_TIMER_0,
    .frequency = 100,
    .bit_resolution = LEDC_TIMER_14_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

TimerConfig Shoulder_config = {
    .timer = LEDC_TIMER_1,
    .frequency = 100,
    .bit_resolution = LEDC_TIMER_14_BIT,
    .mode = LEDC_LOW_SPEED_MODE};

TimerConfig DC_config = {
    .timer = LEDC_TIMER_2,
    .frequency = 4000,
    .bit_resolution = LEDC_TIMER_14_BIT,
    .mode = LEDC_HIGH_SPEED_MODE};
#pragma endregion

#pragma region I2C defines
uint8_t SDA_PIN = 21;
uint8_t SCL_PIN = 22;
#pragma endregion

#pragma region Library defines
SimpleTimer timer;
SimpleI2C i2c;

//Control
AS5600 magE_Base;
AS5600 magE_Shoulder;
QuadratureEncoder quad_E_Elbow;
PID pid;

//Motors
Stepper Base_Motor;
Stepper Shoulder_Motor;
HBridge Elbow_Motor;
HBridge Wrist_Motor;

//End of race sensor
SimpleGPIO EoR;

#pragma endregion

//--------------------------
//Pin and channel definitions
//--------------------------
#pragma region PIN defines
//Stepper pins
uint8_t B_pins[2] = {32, 33}; //dir, step
//magEncoder SDA 21 SCL 22

uint8_t S_pins[2] = {25, 26}; //dir, step
//magEncoder SDA ... SCL ...

//DC pins
uint8_t E_pins[2] = {27, 13};
uint8_t quad_E_pins[2] = {18,19};

//End of race
uint8_t EoR_pin = 35;
#pragma endregion

#pragma region Channel defines
uint8_t B_ch = 0;
uint8_t S_ch = 1;
uint8_t E_ch[2] = {2,3};
#pragma endregion

//--------------------------
//Motor and control variables
//--------------------------
#pragma region DC variables
const float degrees_per_edge = 0.36437f;
#pragma endregion

#pragma region Stepper variables
const float step_angle = 1.8f;
#pragma endregion

#pragma region AS5600 defines
uint8_t mag_status = 0;
#pragma endregion

#pragma region PID variables
int B_measurement = 0;
int S_measurement = 0;
int E_measurement = 0;

int B_error = 0;
int S_error = 0;
int E_error = 0;

int B_prev_error = 0;
int S_prev_error = 0;
int E_prev_error = 0;

int B_control = 0;
int S_control = 0;
int E_control = 0;


float PID_gains[3] = {1.0, 0.2, 0.0};
#pragma endregion

//--------------------------
//Time polling variables
//--------------------------
#pragma region Time Polling defines
uint64_t prev = 0, current = 0;
uint64_t dt_us = 10000; // 10ms
#pragma endregion

#endif // __DEFINITIONS_H__