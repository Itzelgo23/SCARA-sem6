#include "definitions.h"
volatile int counter = 0;

volatile bool pulse_Interrupt = false;


extern "C" void app_main()
{
    esp_task_wdt_deinit();

    stepper.setup(step_pin, step_ch, &Base_config, 4, 750);

    pid.setup(PID_gains, PID_us / 1000000.0f);

    prev = esp_timer_get_time();
    while (1)
    {   
        current = esp_timer_get_time();
        if (current - prev >= dt_us)
        {
            prev = current;
            //stepper.begin(0, 0, 400);
            measurement = stepper.getAngle();
            error = ref - measurement;
            control = pid.calculate(error);
            // PIDmotors(ref, Base, control, error,measurement,measurement);
            freq = stepper.set(control, error);
            printf("Ref: %.2f, Meas: %.2f, Err: %.2f, Ctrl: %.2f\n", ref, measurement, error, control);
        }
        while (uart.available())
        {
            // printf("hola: \n");
            char c;
            uart.read(&c, 1);

            if (c == '\n')
            {
                buffer[index] = '\0';

                sscanf(buffer, "%f", &ref);

                index = 0;
                // printf("recibido: %s\n",buffer);
            }
            else if (index < sizeof(buffer) - 1)
            {
                buffer[index++] = c;
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