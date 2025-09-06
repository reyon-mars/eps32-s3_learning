#pragma once
#include "hal_uart.hpp"
#include "driver/gpio.h"

namespace hal {

class Rs485 {
public:
    Rs485(hal::Uart& uart, gpio_num_t de_re_pin) noexcept;
    esp_err_t init() noexcept;
    esp_err_t send(const uint8_t* data, size_t len, TickType_t timeout = pdMS_TO_TICKS(100)) noexcept;

private:
    hal::Uart& uart_;
    gpio_num_t de_re_pin_;
};

}
