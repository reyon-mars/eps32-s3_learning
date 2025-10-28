#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"

static const char *TAG = "FlashingPattern";

#define LED_GPIO GPIO_NUM_2

void app_main(void)
{
    ESP_LOGI( TAG, "Starting the program." );
    const uint32_t delays_ms[20] = { 1500, 1000, 800, 600, 400, 200, 100, 80, 70, 60, 50, 40, 30, 20, 10, 10, 10, 10, 10 , 10};
    bool led_state = false;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    uint8_t curr_idx = 0;
    ESP_LOGI(TAG, "Starting LED flashing pattern...");

    while (true)
    {
        led_state = true;
        gpio_set_level(LED_GPIO, led_state);
        vTaskDelay(pdMS_TO_TICKS(delays_ms[curr_idx % 10]));

        led_state = false;
        gpio_set_level(LED_GPIO, led_state);
        vTaskDelay(pdMS_TO_TICKS(delays_ms[curr_idx % 10]));

        curr_idx++;
    }
}
