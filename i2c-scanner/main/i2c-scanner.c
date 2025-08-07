#include <stdio.h>
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SCL_IO   GPIO_NUM_19
#define I2C_MASTER_SDA_IO   GPIO_NUM_18
#define I2C_MASTER_FREQ_HZ  400000
#define ACK_CHECK_EN        true
#define ACK_CHECK_DIS       false
#define TAG                 "I2C_SCANNER"


void i2c_master_init(){
    
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK( i2c_param_config( I2C_MASTER_NUM, &conf ) );
    ESP_ERROR_CHECK( i2c_driver_install( I2C_MASTER_NUM, conf.mode, 0, 0, 0 ));

}

void i2c_scan_bus(){
    printf("Starting I2C bus scan.....\n");

    uint8_t address;
    esp_err_t err;
    int devices_found = 0;

    for( address = 0x03; address <=0x77; ++address ){
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start( cmd );
        i2c_master_write_byte( cmd, (address << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN );
        i2c_master_stop(cmd);

        err = i2c_master_cmd_begin( I2C_MASTER_NUM, cmd, pdMS_TO_TICKS( 1000 ) );
        i2c_cmd_link_delete( cmd );
        
        if (err == ESP_OK )
        {
            printf("Found device at 0x%02X\n", address );
            devices_found++;
        }
        else if( err == ESP_ERR_TIMEOUT ){
            printf( "Bus error at 0x%02X\n", address );
        }
    }
    if( devices_found == 0 ){
        printf( "No I2C devices found.\n" );
    } else{
        printf( "Scan complete %d device(s) found.\n", devices_found );
    }
}


void app_main(void)
{

    i2c_master_init();
    while (1)
    {
        i2c_scan_bus();
        vTaskDelay( pdMS_TO_TICKS( 5000 ));
    }
}
