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
#include <SimpleUART.h>

// UART2 RX 16 y TX 17
// Wrist and Gripper in second (SLAVE) ESP32

// Pins used: 13,16,17,18,19,21,22,25,26,27,32,33,35
// Pins left: 4,23

// Pin 14 outputs PWM signal at boot, strapping pin
// para gripper

// puedo usar strapping pins, o outputs PWM signal at boot para pines dir del stepper

// Quad encoders in input pins/strapping pins
enum MagStatus
{
    Detection = 0,
    Error = 1
};
MagStatus status = Detection;

enum MotorTypes
{
    Initial = 0,
    Base = 1,     // Stepper1
    Shoulder = 2, // Stepper2
    Elbow = 3,    // DC1
    Wrist = 4,    // DC2
    Gripper = 5
};
MotorTypes motor_case = Base;

#pragma region TIMERS defines
TimerConfig Base_config = {
    .timer = LEDC_TIMER_0,
    .frequency = 300,
    .bit_resolution = LEDC_TIMER_10_BIT,
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

TimerConfig Extra_config = {
    .timer = LEDC_TIMER_3,
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

// Control
AS5600 magE[2];
QuadratureEncoder quadE[2];
PID pid[4];

// Motors
Stepper Base_Motor;
Stepper Shoulder_Motor;
HBridge Elbow_Motor;
HBridge Wrist_Motor;
HBridge Air_pump;

// End of race sensor
SimpleGPIO EoR;

SimpleUART uart(115200);

#pragma endregion

#pragma region UART varaibles
char buffer_in[32];
#pragma endregion
//--------------------------
// Pin and channel definitions
//--------------------------
#pragma region PIN defines
// Stepper pins
uint8_t B_pins[2] = {32, 33}; // dir, step
// magEncoder SDA 21 SCL 22

uint8_t S_pins[2] = {25, 26}; // dir, step
// magEncoder SDA ... SCL ...

// DC pins
uint8_t E_pins[2] = {27, 13};
uint8_t quad_E_pins[2] = {36, 39};

uint8_t W_pins[2] = {18, 19};
uint8_t quad_W_pins[2] = {34, 14};

uint8_t A_pins[2] = {4, 23};
// End of race
uint8_t EoR_pin = 35;
#pragma endregion

#pragma region Channel defines
uint8_t B_ch = 0;
uint8_t S_ch = 1;
uint8_t E_ch[2] = {2, 3};
uint8_t W_ch[2] = {4, 5};
uint8_t A_ch[2] = {6, 7};
#pragma endregion

//--------------------------
// Motor and control variables
//--------------------------
#pragma region DC variables
const float DpE_Elbow = 0.36437f;
const float DpE_Wrist = 0.36437f;
float speed_DC[2] = {0.0,0.0};
float angle_DC[2] = {0.0,0.0};
#pragma endregion

#pragma region Stepper variables
const float step_angle = 1.8f;
int f_range[2] = {20, 100};
float speed_S[2] = {0.0,0.0};
float angle_S[2] = {0.0,0.0};
#pragma endregion

#pragma region AS5600 defines
uint8_t mag_status = 0;
#pragma endregion

#pragma region PID variables

float error[4] = {0.0, 0.0, 0.0, 0.0}; // base,Shoulder,Elbow,Wrist

float prev_error[4] = {0.0, 0.0, 0.0, 0.0};

float control[4];

int ref = 0;

float PID_gains[3] = {1.0, 0.2, 0.0};
uint64_t PID_us     = 10000;
#pragma endregion

//--------------------------
// Time polling variables
//--------------------------
#pragma region Time Polling defines
uint64_t prev = 0, current = 0;
uint64_t dt_us = 1000; // 1ms
#pragma endregion

#endif // __DEFINITIONS_H__