#include "definitions.h"

static void IRAM_ATTR
interrupt_AS5600(void *arg)
{
    timer.setInterrupt();
}

extern "C" void app_main()
{
    esp_task_wdt_deinit();

    magEncoder.begin();
    timer.setup(interrupt_AS5600, "AS5600 Timer");
    timer.startPeriodic(dt_us);
    while (1)
    {
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