#pragma once 
#include <cstdint>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_err.h"

struct lr1121_version{
    uint8_t hw_version{};
    uint8_t use_case{};
    uint8_t fw_major{};
    uint8_t fw_minor{};
};

enum class lr1121_err : int32_t{
    OK = ESP_OK,
    timeout_busy = 0x10001,
    spi_transfer = 0x10002,
    invalid_arg  = 0x10003,
};

struct lr1121_pins{
    gpio_num_t nss; 
    gpio_num_t busy;
    gpio_num_t rst;
};

