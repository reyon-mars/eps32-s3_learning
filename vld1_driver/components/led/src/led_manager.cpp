#include "led_manager.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char* TAG = "LedManager";

LedManager::LedManager(gpio_num_t led_pin) noexcept : pin_(led_pin) {}

void LedManager::init() noexcept {
    gpio_set_direction(pin_, GPIO_MODE_OUTPUT);
    gpio_set_level(pin_, 0);
    ESP_LOGI(TAG, "LED pin %d initialized", pin_);
}

void LedManager::blink(int times, int delay_ms) noexcept {
    for (int i = 0; i < times; ++i) {
        gpio_set_level(pin_, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(pin_, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void LedManager::set(bool on) noexcept {
    gpio_set_level(pin_, on ? 1 : 0);
}
