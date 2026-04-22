#ifndef __DEFINITIONS_H__
#define __DEFINITIONS_H__

#include <esp_timer.h>
#include <esp_xt_wdt.h>
#include <AS5600.h>
#include <SimpleTimer.h>
#include <SimpleI2C.h>
#include <SimpleGPIO.h>
#include <Stepper.h>
#include <QuadratureEncoder.h>
#include <PID.h>
#include <HBridge.h>
#include <SimpleUART.h>
#include <ServoStepper.h>
// UART2 RX 16 y TX 17
// Wrist and Gripper in second (SLAVE) ESP32

//2 steppers (4), 2 DC (4), 1 mag encoder (SDA, SCL), 2 quad encoder
//1 limit switch, 1 UART (16,17)

// Pins used: 4,13,16,17,18,19,21,22,23,25,26,27,32,33,34,35,36,39


// Pin 2 connected to on-board LED, must be floating or LOW to enter flashing mode
// Pin 5,15 outputs PWM signal at boot, strapping pin
// Pin 14 outputs PWM signal at boot

//Pin 12 boot fails if pulled high, strapping pin
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
    Gripper = 5,
    Home = 6
};
MotorTypes motor_case = Initial;

enum PS4state {
    xUp = 0,
    xDown = 1,
    xRight = 2,
    xLeft = 3,

    yUp = 4,
    yDown = 5,
    yRight = 6,
    yLeft = 7,

    d_up = 8,
    d_down = 9,
    d_right = 10,
    d_left = 11,

    a = 12,
};
PS4state PS4_state = xUp;

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
    .frequency = 100,
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
AS5600 magE;
QuadratureEncoder quadE[2];
PID pid[4];

// Motors
ServoStepper Base_Motor; //up and down
Stepper Shoulder_Motor; //left and right
HBridge Elbow_Motor; //left and right
HBridge Wrist_Motor; //left and right
HBridge Air_pump;

// Limit switch sensor
SimpleGPIO LimitSwitch;

SimpleUART uart(115200);

#pragma endregion

#pragma region UART varaibles
char buffer_in[32];
static int index = 0;
#pragma endregion
//--------------------------
// Pin and channel definitions
//--------------------------
#pragma region PIN defines
// Stepper pins
uint8_t B_pins[2] = {32, 33}; // dir, step
// magEncoder SDA 21 SCL 22
//UART 16 & 17

uint8_t S_pins[2] = {25, 26}; // dir, step

// DC pins
uint8_t E_pins[2] = {18, 19}; //white,red
uint8_t quad_E_pins[2] = {36, 39}; //VP, VN //green, yellow

uint8_t W_pins[2] = {27, 13}; //white,red
uint8_t quad_W_pins[2] = {34, 35}; //yellow, green

uint8_t G_pins = 4;
// Limit switch pin
uint8_t LS_pin = 23;
#pragma endregion

#pragma region Channel defines
uint8_t B_ch = 0;
uint8_t S_ch = 1;
uint8_t E_ch[2] = {2, 3};
uint8_t W_ch[2] = {4, 5};
uint8_t G_ch[2] = {6, 7};
#pragma endregion

//--------------------------
// Motor and control variables
//--------------------------
#pragma region DC variables
const float DpE_Elbow = 0.36437f;
const float DpE_Wrist = 0.36437f;
float speed_DC[2] = {0.0,0.0};
float angle_DC[2] = {0.0,0.0};
float max_DC_freq[2] = {90.0f,100.0f}; //Elbow, Wrist
#pragma endregion

#pragma region Stepper variables
//0.42A when shoulder moves, 0.31A when not moving.
const float step_angle = 1.8f;
float max_freq[2] = {1000.0f,50.0f}; //base, shoulder
float speed_S[2] = {0.0,0.0};
float angle_S[2] = {0.0,0.0};
float home_freq;
#pragma endregion

#pragma region AS5600 defines
uint8_t mag_status = 0;
float angle_AS5600 = 0.0;
#pragma endregion

#pragma region Limit Switch variables
bool pressed = false;
bool prev_pressed = false;
bool is_home = false;
bool home_reached = false;
#pragma endregion

#pragma region PID variables

float error[4] = {0.0, 0.0, 0.0, 0.0}; // base,Shoulder,Elbow,Wrist

float prev_error[4] = {0.0, 0.0, 0.0, 0.0};

float control[4];

float ref[4] = {0.0, 0.0, 0.0, 0.0};
float set_ref = 0.0;

float PID_B_gains[3] = {30.0, 0.0, 0.0}; //no encoder
float PID_S_gains[3] = {12.5, 0.0, 0.0}; //mag encoder
float PID_E_gains[3] = {1.2f, 0.0f, 0.0};  
float PID_W_gains[3] = {1.81f, 0.0f, 0.0};  
uint64_t PID_us     = 10000;
#pragma endregion

//--------------------------
// Time polling variables
//--------------------------
#pragma region Time Polling defines
uint64_t prev = 0, current = 0;
uint64_t dt_us = 10000; // 1ms
#pragma endregion

#endif // __DEFINITIONS_H__