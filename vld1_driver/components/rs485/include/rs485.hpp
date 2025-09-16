#pragma once
#include "uart.hpp"
#include "driver/gpio.h"
#include "mbcontroller.h"       // for mbcontroller defines and api
#include "modbus_params.h"
#include "esp_err.h"

class rs485
{
public:
    rs485(uart &uart_no, gpio_num_t de_re_pin) noexcept;
    ~rs485() noexcept;

    esp_err_t init(uint8_t slave_addr, size_t input_reg_count) noexcept;

    esp_err_t write(const uint16_t *data, size_t count, uint16_t start_addr = 0) noexcept;

private:
    esp_err_t configure_pins() noexcept;
    void free_registers() noexcept;

    gpio_num_t de_re_pin_;
    uart &uart_;
    void *mbc_slave_handle_;
    uint8_t slave_address_;
    uint16_t *input_registers_;
    size_t input_count_;
};
