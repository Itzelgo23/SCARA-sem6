#include "definitions.h"
#include "PIDmotors.h"
#include "MoveHome.h"
#include "MagnetDetection.h"
#include "cm2deg.h"

static void IRAM_ATTR
interrupt_AS5600(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();
    // gpio_install_isr_service(ESP_INTR_FLAG_IRAM);

    i2c.setup_master(21, 22, 100000, I2C_NUM_1);
    magE.setup(i2c);
    timer.setup(interrupt_AS5600, "AS5600 Timer");
    timer.startPeriodic(dt_us);

    Shoulder_Motor.setup(S_pins, S_ch, &Shoulder_config, max_freq[1]);
    Elbow_Motor.setup(E_pins, E_ch, &DC_config, max_DC_freq[0]);
    Wrist_Motor.setup(W_pins, W_ch, &DC_config, max_DC_freq[1]);
    // Air_pump.setup(G_pins, G_ch, &Extra_config);

    LimitSwitch.setup(LS_pin, GPI, GPIO_PULLDOWN_ONLY);

    pid[0].setup(PID_B_gains, PID_us / 1000000.0f);
    pid[1].setup(PID_S_gains, PID_us / 1000000.0f);
    pid[2].setup(PID_E_gains, PID_us / 1000000.0f);
    pid[3].setup(PID_W_gains, PID_us / 1000000.0f);

    Base_Motor.setup(B_pins, B_ch, &Base_config, 4, max_freq[0]);
    quadE[0].setup(quad_E_pins, DpE_Elbow);
    quadE[1].setup(quad_W_pins, DpE_Wrist);

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

                    uint8_t magnetRead = magE.readMagnet();
                    printf("Magnet Status: %d\n", magnetRead);
                    vTaskDelay(pdMS_TO_TICKS(500));
                    break;
                }
                case Base: // Stepper1
                {
                    Shoulder_Motor.setSpeed(0);
                    Elbow_Motor.setSpeed(0);
                    Wrist_Motor.setSpeed(0);
                    Air_pump.setSpeed(0);

                    ref[0] = set_ref;
                    // printf("moving base motor, ref: %.2f\n", ref[0]);
                    angle_AS5600 = cm2deg(ref[0], 8.0f);
                    PIDmotors(angle_AS5600, Base, control[0], error[0], angle_S[0], speed_S[0]);
                    // printf("cm: %.2f |deg: %.2f\n", ref[0], angle_AS5600);
                    Base_Motor.set(control[0], error[0]);
                    break;
                }
                case Shoulder: // Stepper2
                {
                    Base_Motor.setSpeed(0);
                    Elbow_Motor.setSpeed(0);
                    Wrist_Motor.setSpeed(0);
                    Air_pump.setSpeed(0);
                    ref[1] = set_ref;
                    /*if (set_ref == 0)
                    {
                        float turns = round(angle_S[1] / 1080.0f);
                        ref[1] = turns * 1080.0f;
                    }
                    
                    else
                    {
                        ref[1] = 3 * set_ref;
                    }*/

                    PIDmotors(ref[1], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
                    Shoulder_Motor.set(control[1], error[1]);
                    printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[1], angle_S[1], control[1], error[1]);
                    break;
                }
                case Elbow: // DC1
                {
                    Base_Motor.setSpeed(0);
                    Shoulder_Motor.setSpeed(0);
                    Wrist_Motor.setSpeed(0);
                    Air_pump.setSpeed(0);

                    ref[2] = set_ref;
                    PIDmotors(ref[2], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
                    Elbow_Motor.setSpeed(control[2]);
                    printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[2], angle_DC[0], control[2], error[2]);
                    //printf("%.2f,%.2f,%d\n", angle_DC[0], speed_DC[0], current);
                    //Elbow_Motor.setSpeed(ref[2]);
                    //printf("%.2f,%.2f,%d\n",quadE[0].getAngle(),quadE[0].getSpeed(),current);
                    break;
                }
                case Wrist: // DC2
                {
                    Base_Motor.setSpeed(0);
                    Shoulder_Motor.setSpeed(0);
                    Elbow_Motor.setSpeed(0);
                    Air_pump.setSpeed(0);

                    ref[3] = set_ref;
                    PIDmotors(ref[3], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);
                    Wrist_Motor.setSpeed(control[3]);
                    printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[3], angle_DC[1], control[3], error[3]);
                    break;
                }
                case Gripper:
                {
                    Base_Motor.setSpeed(0);
                    Shoulder_Motor.setSpeed(0);
                    Elbow_Motor.setSpeed(0);
                    Wrist_Motor.setSpeed(0);

                    Air_pump.setSpeed(100);
                    break;
                }
                case Home:
                {
                    home_reached = MoveHome(-home_freq);
                    printf("Moving to home, freq: %.2f, reached: %d\n", home_freq, home_reached);
                    if (home_reached)
                    {
                        printf("Home position reached\n");
                        // Add for motor to retract 10 steps to ensure it's off the limit switch
                        motor_case = Initial; // Reset to initial after homing
                    }
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
                Air_pump.setSpeed(0);
                break;
            }
            /*int len = uart.available();
            if (len)
            {
                int motor_tmp;
                uart.read(buffer_in, len);
                printf("RX RAW: [%s]\n", buffer_in);
                sscanf(buffer_in, "%d,%f,%f", &motor_tmp, &set_ref, &home_freq);
                motor_case = (MotorTypes)motor_tmp;
            }*/
        }

        /*if (timer.interruptAvailable())
        {
            status = DetectMagnet(motor_case,motor_case);
        }*/

        while (uart.available())
        {
            // printf("hola: \n");
            char c;
            int motor_tmp;
            uart.read(&c, 1);

            if (c == '\n')
            {
                buffer_in[index] = '\0';

                sscanf(buffer_in, "%d,%f,%f", &motor_tmp, &set_ref, &home_freq);
                motor_case = (MotorTypes)motor_tmp;
                index = 0;
                // printf("recibido: %s\n",buffer);
            }
            else if (index < sizeof(buffer_in) - 1)
            {
                buffer_in[index++] = c;
                // printf("elseif:\n");
            }
            else
            {
                index = 0;
                // printf("overflow \n");
            }
        }
    }
}