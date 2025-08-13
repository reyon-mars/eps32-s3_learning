#include "driver/i2c.h"
#include "i2c_master.hpp"
#include "esp_log.h"

I2CMaster::I2CMaster(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq) : portNum(port)
{
    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = scl;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq;

    if (i2c_param_config(portNum, &conf) != ESP_OK ||
        i2c_driver_install(portNum, conf.mode, 0, 0, 0) != ESP_OK)
    {
        ESP_LOGE("I2C_MASTER", "I2C_MASTER install failed.");
    }
}

esp_err_t I2CMaster::write( uint8_t addr, const uint8_t *data, size_t len ) const
{
    return i2c_master_write_to_device(portNum, addr, data, len, pdMS_TO_TICKS(1000));
}