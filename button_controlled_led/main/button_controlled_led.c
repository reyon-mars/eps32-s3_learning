#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LED_GPIO GPIO_NUM_10
#define BUTTON_GPIO GPIO_NUM_9

#define DEBOUNCE_DELAY 50


void app_main(void)
{
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    gpio_set_direction(BUTTON_GPIO, GPIO_MODE_INPUT);
    gpio_pullup_en(BUTTON_GPIO);
    gpio_pulldown_dis(BUTTON_GPIO); 

    int led_state = 0;
    int last_button_state = 1;

    while (1) {
        int button_state = gpio_get_level(BUTTON_GPIO);

        if (last_button_state == 1 && button_state == 0) {
            vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY));
            button_state = gpio_get_level(BUTTON_GPIO);

            if (button_state == 0) {
                led_state = !led_state;
                gpio_set_level(LED_GPIO, led_state);
                printf("Button pressed. LED is now %s\n", led_state ? "ON" : "OFF");
            }
        }
        last_button_state = button_state;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
