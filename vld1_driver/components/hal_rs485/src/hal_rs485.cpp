#include "hal_rs485.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "driver/uart.h"

namespace hal {
static const char* TAG = "hal::Rs485";

Rs485::Rs485(hal::Uart& uart, gpio_num_t de_re_pin) noexcept : uart_(uart), de_re_pin_(de_re_pin) {}

esp_err_t Rs485::init() noexcept {
    gpio_config_t io_conf = {};
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << static_cast<uint32_t>(de_re_pin_));
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io_conf);
    gpio_set_level(de_re_pin_, 0);
    ESP_LOGI(TAG, "RS485 DE/RE pin %d initialized", de_re_pin_);
    return ESP_OK;
}

esp_err_t Rs485::send(const uint8_t* data, size_t len, TickType_t timeout) noexcept {
    gpio_set_level(de_re_pin_, 1); // enable driver
    int written = uart_.write(data, len);
    if (written < 0) {
        ESP_LOGE(TAG, "UART write failed");
        gpio_set_level(de_re_pin_, 0);
        return ESP_FAIL;
    }
    // ensure TX finished
    uart_wait_tx_done(uart_.port(), timeout);
    gpio_set_level(de_re_pin_, 0); // disable driver
    return ESP_OK;
}

}
