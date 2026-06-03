#include "definitions.h"
#include "PIDmotors.h"
#include "MoveHome.h"
#include "MagnetDetection.h"
#include "cm2deg.h"
#include "Kinematics.h"
#include "PickandPlace.h"

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
    // Gripper_Motor.setup(G_pins, G_ch, &Extra_config,);
    Gripper_Motor.setup(G_pins, GPIO_MODE_OUTPUT, GPIO_PULLUP_ONLY);

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
            Pick_done = false;
            Place_done = false;
            switch (status)
            {
            case Detection:
                switch (robot_state)
                {
                case Init:
                {
                    printf("Initialization Mode\n");
                    break;
                }
                case JointJog:
                {
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
                        if (set_ref1 != 30.0f)
                            set_ref1 = 30.0f;
                        if (set_ref2 != 0.0f)
                            set_ref2 = 0.0f;
                        if (set_ref3 != 0.0f)
                            set_ref3 = 0.0f;
                        Shoulder_Motor.setSpeed(0);
                        Elbow_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        PID_B_gains[0] = set_ref1;
                        PID_B_gains[1] = set_ref2;
                        PID_B_gains[2] = set_ref3;
                        ref[0] = set_ref4;
                        // printf("moving base motor, ref: %.2f\n", ref[0]);
                        angle_AS5600 = cm2deg(ref[0], 8.0f);
                        PIDmotors(angle_AS5600, Base, control[0], error[0], angle_S[0], speed_S[0]);
                        // printf("cm: %.2f |deg: %.2f\n", ref[0], angle_AS5600);
                        Base_Motor.set(control[0], error[0]);
                        break;
                    }
                    case Shoulder: // Stepper2
                    {
                        if (set_ref1 != 15.0f)
                            set_ref1 = 15.0f;
                        if (set_ref2 != 0.0f)
                            set_ref2 = 0.0f;
                        if (set_ref3 != 0.0f)
                            set_ref3 = 0.0f;
                        Base_Motor.setSpeed(0);
                        Elbow_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        PID_S_gains[0] = set_ref1;
                        PID_S_gains[1] = set_ref2;
                        PID_S_gains[2] = set_ref3;
                        ref[1] = set_ref4;
                        PIDmotors(ref[1], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
                        Shoulder_Motor.set(control[1], error[1]);
                        height = deg2cm(Base_Motor.getAngle(), 8.0f);
                        // getFK(lengths,height, magE.getTotalAngle(), quadE[0].getAngle(), quadE[1].getAngle(), T_final, euler);
                        // printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[1], angle_S[1], control[1], error[1]);
                        break;
                    }
                    case Elbow: // DC1
                    {
                        /*if (set_ref1 != 1.2f)
                            set_ref1 = 1.2f;
                        if (set_ref2 != 0.0f)
                            set_ref2 = 0.0f;
                        if (set_ref3 != 0.0f)
                            set_ref3 = 0.0f;
                            */
                        Base_Motor.setSpeed(0);
                        Shoulder_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        /*PID_E_gains[0] = set_ref1;
                        PID_E_gains[1] = set_ref2;
                        PID_E_gains[2] = set_ref3;
                        */
                        ref[2] = set_ref4;
                        PIDmotors(ref[2], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
                        //Elbow_Motor.setSpeed(control[2]);
                        printf("Elbow -- ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[2], angle_DC[0], control[2], error[2]);
                        // printf("%.2f,%.2f,%d\n", angle_DC[0], speed_DC[0], current);
                        Elbow_Motor.setSpeed(ref[2]);
                        // printf("%.2f,%.2f,%d\n",quadE[0].getAngle(),quadE[0].getSpeed(),current);
                        break;
                    }
                    case Wrist: // DC2
                    {
                        if (set_ref1 != 10.0f)
                            set_ref1 = 1.2f;
                        if (set_ref2 != 0.0f)
                            set_ref2 = 0.0f;
                        if (set_ref3 != 0.0f)
                            set_ref3 = 0.0f;
                        Base_Motor.setSpeed(0);
                        Shoulder_Motor.setSpeed(0);
                        Elbow_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        PID_W_gains[0] = set_ref1;
                        PID_W_gains[1] = set_ref2;
                        PID_W_gains[2] = set_ref3;
                        ref[3] = set_ref4;
                        PIDmotors(ref[3], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);
                        Wrist_Motor.setSpeed(control[3]);
                        // printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[3], angle_DC[1], control[3], error[3]);
                        break;
                    }
                    case Gripper:
                    {
                        printf("Activating gripper\n");
                        Base_Motor.setSpeed(0);
                        Shoulder_Motor.setSpeed(0);
                        Elbow_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);

                        Gripper_Motor.set(1);
                        break;
                    }
                    case Forward:
                    {
                        getFK(lengths, height, magE.getTotalAngle(), quadE[0].getAngle(), quadE[1].getAngle(), T_final, euler);
                        printf("End Effector Position: x=%.2f | y=%.2f | orientation=%.2f \n", euler[0], euler[1], euler[2]);
                        break;
                    }
                    case Inverse:
                    {
                        location[0]=set_ref1;
                        location[1]=set_ref2;
                        location[2]=set_ref3;
                        printf("Target Position: x=%.2f | y=%.2f | orientation=%.2f \n", location[0], location[1], location[2]);
                        getIK(location, L1, L2, num_solutions, solutions);
                        PIDmotors(solutions[0][0], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
                        PIDmotors(solutions[0][1], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
                        PIDmotors(solutions[0][2], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);
                        if (fabs(error[1]) < 5.0f && fabs(error[2]) < 5.0f && fabs(error[3]) < 5.0f)
                        {
                            printf("Inverse kinematics solution reached\n"); 
                        }
                        break;
                    }
                    default:
                        break;
                    }
                    break;
                }

                case Coordinates:
                {
                    getIK(location, L1, L2, num_solutions, solutions);
                    angle_AS5600 = cm2deg(5, 8.0f);
                    PIDmotors(angle_AS5600, Base, control[0], error[0], angle_S[0], speed_S[0]);
                    PIDmotors(solutions[0][0], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
                    PIDmotors(solutions[0][1], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
                    PIDmotors(solutions[0][2], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);
                    break;
                }

                case PID_control:
                {
                    printf("PID Control Mode\n");
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
                        //Elbow_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        PID_B_gains[0] = set_ref1;
                        PID_B_gains[1] = set_ref2;
                        PID_B_gains[2] = set_ref3;
                        ref[0] = set_ref4;
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
                        //Elbow_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        PID_S_gains[0] = set_ref1;
                        PID_S_gains[1] = set_ref2;
                        PID_S_gains[2] = set_ref3;
                        ref[1] = set_ref4;
                        PIDmotors(ref[1], Shoulder, control[1], error[1], angle_S[1], speed_S[1]);
                        Shoulder_Motor.set(control[1], error[1]);
                        height = deg2cm(Base_Motor.getAngle(), 8.0f);
                        // getFK(lengths,height, magE.getTotalAngle(), quadE[0].getAngle(), quadE[1].getAngle(), T_final, euler);
                        // printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[1], angle_S[1], control[1], error[1]);
                        break;
                    }
                    case Elbow: // DC1
                    {
                        Base_Motor.setSpeed(0);
                        Shoulder_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        PID_E_gains[0] = set_ref1;
                        PID_E_gains[1] = set_ref2;
                        PID_E_gains[2] = set_ref3;
                        ref[2] = set_ref4;
                        PIDmotors(ref[2], Elbow, control[2], error[2], angle_DC[0], speed_DC[0]);
                        Elbow_Motor.setSpeed(control[2]);
                        printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[2], angle_DC[0], control[2], error[2]);
                        // printf("%.2f,%.2f,%d\n", angle_DC[0], speed_DC[0], current);
                        // Elbow_Motor.setSpeed(ref[2]);
                        // printf("%.2f,%.2f,%d\n",quadE[0].getAngle(),quadE[0].getSpeed(),current);
                        break;
                    }
                    case Wrist: // DC2
                    {
                        Base_Motor.setSpeed(0);
                        Shoulder_Motor.setSpeed(0);
                        //Elbow_Motor.setSpeed(0);
                        Gripper_Motor.set(0);

                        PID_W_gains[0] = set_ref1;
                        PID_W_gains[1] = set_ref2;
                        PID_W_gains[2] = set_ref3;
                        ref[3] = set_ref4;
                        PIDmotors(ref[3], Wrist, control[3], error[3], angle_DC[1], speed_DC[1]);
                        Wrist_Motor.setSpeed(control[3]);
                        // printf("ref: %.2f | angle: %.2f | control: %.2f | error: %.2f\n", ref[3], angle_DC[1], control[3], error[3]);
                        break;
                    }
                    case Gripper:
                    {
                        printf("Activating gripper\n");
                        Base_Motor.setSpeed(0);
                        Shoulder_Motor.setSpeed(0);
                        //Elbow_Motor.setSpeed(0);
                        Wrist_Motor.setSpeed(0);

                        Gripper_Motor.set(1);
                        break;
                    }
                    default:
                        break;
                    }
                    
                    break;
                }

                case PickPlace:
                {
                    if (set_ref4 == 1.0f)
                    {
                        if (!Pick_done)
                        {
                            Pick_done = setPick(set_ref1, set_ref2, set_ref3);

                            if (Pick_done)
                                printf("Pick successful\n");
                        }
                        else if (!Place_done)
                        {
                            Place_done = setPlace(Place1);

                            if (Place_done)
                                printf("Place successful\n");
                        }
                    }
                    else if (set_ref4 == 2.0f)
                    {
                        // Pick
                        if (!Pick_done)
                        {
                            Pick_done = setPick(set_ref1, set_ref2, set_ref3);

                            if (Pick_done)
                                printf("Pick successful\n");
                        }
                        else if (!Place_done)
                        {
                            Place_done = setPlace(Place2);

                            if (Place_done)
                                printf("Place successful\n");
                        }
                    }
                    else if (set_ref4 == 3.0f)
                    {
                        // Pick
                        if (!Pick_done)
                        {
                            Pick_done = setPick(set_ref1, set_ref2, set_ref3);

                            if (Pick_done)
                                printf("Pick successful\n");
                        }
                        else if (!Place_done)
                        {
                            Place_done = setPlace(Place3);

                            if (Place_done)
                                printf("Place successful\n");
                        }
                    }
                    else if (set_ref4 == 4.0f)
                    {
                        // Pick
                        if (!Pick_done)
                        {
                            Pick_done = setPick(set_ref1, set_ref2, set_ref3);

                            if (Pick_done)
                                printf("Pick successful\n");
                        }
                        else if (!Place_done)
                        {
                            Place_done = setPlace(Place4);

                            if (Place_done)
                                printf("Place successful\n");
                        }
                    }
                    else
                    {
                        printf("Invalid ID\n");
                    }
                    break;
                }

                case Home:
                {
                    home_reached = MoveHome(-fabs(home_freq));
                    printf("Moving to home, freq: %.2f, reached: %d\n", home_freq, home_reached);
                    if (home_reached)
                    {
                        printf("Home position reached\n");
                        // Add for motor to retract 10 steps to ensure it's off the limit switch
                        robot_state = Init; // Reset to initial after homing
                    }
                    break;
                }
                }

            case Error:
                Base_Motor.setSpeed(0);
                Shoulder_Motor.setSpeed(0);
                //Elbow_Motor.setSpeed(0);
                Wrist_Motor.setSpeed(0);
                Gripper_Motor.set(0);
                break;
            }
            //getFK(lengths, height, magE.getTotalAngle(), quadE[0].getAngle(), quadE[1].getAngle(), T_final, euler);
            
            //send_message = sprintf(message, "%d,%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", Pick_done, Place_done, angle_S[1], angle_DC[0], angle_DC[1],euler[0],euler[1],deg2cm(angle_S[0], 8.0f));
            //uart.write(message, send_message);
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
            int robot_tmp;
            uart.read(&c, 1);

            if (c == '\n')
            {
                buffer_in[uart_index] = '\0';

                sscanf(buffer_in, "%d,%d,%f,%f,%f,%f", &robot_tmp, &motor_tmp, &set_ref1, &set_ref2, &set_ref3, &set_ref4);
                robot_state = (RobotState)robot_tmp;
                motor_case = (MotorTypes)motor_tmp;
                uart_index = 0;
                // printf("recibido: %s\n",buffer);
            }
            else if (uart_index < sizeof(buffer_in) - 1)
            {
                buffer_in[uart_index++] = c;
                // printf("elseif:\n");
            }
            else
            {
                uart_index = 0;
                // printf("overflow \n");
            }
        }
    }
}