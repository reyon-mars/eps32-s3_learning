#pragma once

#include "driver/i2c.h"
#include <cstdint>

class I2CMaster
{
public:
    I2CMaster(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq);
    esp_err_t write(uint8_t addr, const uint8_t *data, size_t len) const;

private:
    i2c_port_t portNum;
};