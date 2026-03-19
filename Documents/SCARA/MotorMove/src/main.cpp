#include "definitions.h"

static void IRAM_ATTR
interrupt_AS5600(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    i2c.setup_master(21, 22, 100000, I2C_NUM_1);
    magE_Base.setup(i2c);
    timer.setup(interrupt_AS5600, "AS5600 Timer");
    timer.startPeriodic(dt_us);

    Base_Motor.setup(B_pins, B_ch, &Base_config);
    Shoulder_Motor.setup(S_pins, S_ch, &Shoulder_config);
    Elbow_Motor.setup(E_pins, E_pins, &DC_config);

    quad_E_Elbow.setup(quad_E_pins, degrees_per_edge);

    EoR.setup(EoR_pin, GPI, GPIO_PULLDOWN_ONLY);

    prev = esp_timer_get_time();
    while (1)
    {
        current = esp_timer_get_time();
        if (current - prev >= dt_us)
        {
            prev = current;
            switch (motor_case)
            {
            case Initial:
                uint8_t magnetRead = magE_Base.readMagnet();
                printf("Magnet Status: %d\n", magnetRead);
                vTaskDelay(pdMS_TO_TICKS(500));

                break;
            case Base: // Stepper1
                Base_Motor.setSpeed(100);
                break;
            case Shoulder: // Stepper2
                Shoulder_Motor.setSpeed(100);
                break;
            case Elbow: // DC1
                Elbow_Motor.setSpeed(100);
                break;
            case Wrist: // DC2
                // UART
                break;

            default:
                break;
            }
        }

        if (timer.interruptAvailable())
        {
            if (magE_Base.MagnetDetection() != MD)
                status = Error;
            else
                status = Detection;

            switch (status)
            {
            case Detection:
                magE_Base.readRawAngle();
                printf("Angle: %.2f\n", magE_Base.getTotalAngle());
                break;

            case Error:
                magE_Base.MagnetDetection();
                break;
            default:
                break;
            }
        }
    }
}