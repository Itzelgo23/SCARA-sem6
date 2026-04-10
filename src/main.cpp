#include "definitions.h"
#include "PIDmotors.h"
#include "MagnetDetection.h"

static void IRAM_ATTR
interrupt_AS5600(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    i2c.setup_master(21, 22, 100000, I2C_NUM_1);
    magE[0].setup(i2c);
    magE[1].setup(i2c);
    // magE[1].setup(i2c);
    timer.setup(interrupt_AS5600, "AS5600 Timer");
    timer.startPeriodic(dt_us);

    Base_Motor.setup(B_pins, B_ch, &Base_config, 4, max_freq);
    Shoulder_Motor.setup(S_pins, S_ch, &Shoulder_config, max_freq);
    Elbow_Motor.setup(E_pins, E_ch, &DC_config);
    Wrist_Motor.setup(W_pins, W_ch, &DC_config);
    //Air_pump.setup(G_pins, G_ch, &Extra_config);

    quadE[0].setup(quad_E_pins, DpE_Elbow);
    quadE[1].setup(quad_W_pins, DpE_Wrist);

    EoR.setup(EoR_pin, GPI, GPIO_PULLDOWN_ONLY);

    pid[0].setup(PID_gains, PID_us / 1000000.0f);
    pid[1].setup(PID_gains, PID_us / 1000000.0f);
    pid[2].setup(PID_DC_gains, PID_us / 1000000.0f);
    pid[3].setup(PID_DC_gains, PID_us / 1000000.0f);

    prev = esp_timer_get_time();
    while (1)
    {
        current = esp_timer_get_time();
        if (current - prev >= dt_us)
        {
            prev = current;
            switch (status)
            {
            case Detection:
                switch (motor_case)
                {
                case Initial:
                {
                    Base_Motor.setSpeed(0);
                    Shoulder_Motor.setSpeed(0);
                    Elbow_Motor.setSpeed(0);
                    Wrist_Motor.setSpeed(0);
                    uint8_t magnetRead = magE[1].readMagnet();
                    printf("Magnet Status: %d\n", magnetRead);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;
                }
                case Base: // Stepper1
                {
                    printf("moving base motor, ref: %d\n", ref);
                    PIDmotors(ref, Base, control[0], error[0],angle_S[0],speed_S[0]);
                    Base_Motor.set(control[0], error[0]);
                    //printf("Angle: %.2f | Speed: %.2f | Current time: %d\n",angle_S[0],speed_S[0],current);  
                    break;
                }
                case Shoulder: // Stepper2
                {
                    PIDmotors(ref, Shoulder, control[1], error[1],angle_S[1],speed_S[1]);
                    Shoulder_Motor.set(control[1], error[1]);
                    break;
                }
                case Elbow: // DC1
                {
                    PIDmotors(ref, Elbow, control[2], error[2],angle_DC[0],speed_DC[0]);
                    Elbow_Motor.setSpeed(control[2]);
                    break;
                }
                case Wrist: // DC2
                {
                    PIDmotors(ref, Wrist, control[3], error[3],angle_DC[1],speed_DC[1]);
                    Wrist_Motor.setSpeed(control[3]);
                    break;
                }
                case Gripper:
                {
                    Air_pump.setSpeed(100);
                    break;
                }

                default:
                    break;
                }
                break;

            case Error:
                Base_Motor.setSpeed(0);
                Shoulder_Motor.setSpeed(0);
                Elbow_Motor.setSpeed(0);
                Wrist_Motor.setSpeed(0);
                break;
            }
            int len = uart.available();
            if (len)
            {
                int motor_tmp;
                uart.read(buffer_in, len);
                printf("RX RAW: [%s]\n", buffer_in);
                sscanf(buffer_in, "%d,%d", &motor_tmp, &ref);
                motor_case = (MotorTypes)motor_tmp;
            }
        }

        /*if (timer.interruptAvailable())
        {
            status = DetectMagnet(motor_case,motor_case);
        }*/
    }
}