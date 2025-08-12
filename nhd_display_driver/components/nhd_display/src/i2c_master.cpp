#include "driver/i2c.h"
#include "i2c_master.hpp"

I2CMaster::I2CMaster( i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq ) : portNum( port )
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq
    };

    if( i2c_param_config( portNum, &conf ) != ESP_OK || 
        i2c_driver_install( portNum, conf.mode, 0, 0, 0 ) != ESP_Ok ){
            throw std::runtime_error( "Failed to initialize I2C master. " );
    }
}

esp_err_t I2CMaster::write( uint8_t addr, const uint8_t* data, size_t len ) const {
    return i2c_master_write_to_device( portNum, addr, data, len, pdMS_TO_TICKS(1000) );
}