#include <ServoStepper.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <PID.h>
#include <SimpleUART.h>

static int index = 0;

ServoStepper stepper;
PID pid;

SimpleUART uart(115200);

uint64_t prev = 0, current = 0;
uint64_t dt_us = 1000; // 1 ms

uint8_t step_pin[2] = {32,33}; //dir, step
uint8_t step_ch = 0;
float freq = 100.0;

float error = 0.0; // base,Shoulder,Elbow,Wrist
float measurement = 0.0;
float prev_error = 0.0;
float control = 0.0;
float ref = 0.0;
float PID_gains[3] = {100, 0.0, 0.0};
uint64_t PID_us     = 2000;

uint64_t debounce = 500;

char buffer[64];

TimerConfig Base_config = {
    .timer = LEDC_TIMER_0,
    .frequency = 100,
    .bit_resolution = LEDC_TIMER_10_BIT,
    .mode = LEDC_LOW_SPEED_MODE};


enum MotorTypes
{
    Initial = 0,
    Base = 1,     // Stepper1
    Shoulder = 2, // Stepper2
    Elbow = 3,    // DC1
    Wrist = 4,    // DC2
    Gripper = 5
};