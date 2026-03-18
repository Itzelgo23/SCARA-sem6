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
    magEncoder.setup(i2c);
    timer.setup(interrupt_AS5600, "AS5600 Timer");
    timer.startPeriodic(dt_us);
    
    while (1)
    {
        uint8_t magnetRead = magEncoder.readMagnet();
        printf("Magnet Status: %d\n", magnetRead);
        vTaskDelay(pdMS_TO_TICKS(500)); // Delay to prevent excessive I2C reads, adjust as needed
        if (timer.interruptAvailable())
        {
            if (magEncoder.MagnetDetection() != MD)
                status = Error;
            else
                status = Detection;

            switch (status)
            {
            case Detection:
                magEncoder.readRawAngle();
                printf("Angle: %.2f\n", magEncoder.getTotalAngle());
                break;

            case Error:
                magEncoder.MagnetDetection();
                break;
            default:
                break;
            }
        }
    }

    // Interrupt si magnet_status != MD
    // Enter magnet error case
    // The MagnetDetction function is constantly read until magnet_status == MD
}