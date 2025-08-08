#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


#define I2C_MASTER_SCL_IO          20
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_LCD_ADDR               0x3C  

#define I2C_TIMEOUT_MS             1000

#define LCD_COMMAND_MODE           0x00  
#define LCD_DATA_MODE              0x40  


static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

static void i2c_lcd_send_command(uint8_t cmd)
{
    uint8_t data[2] = { LCD_COMMAND_MODE, cmd };
    i2c_master_write_to_device(I2C_MASTER_NUM, I2C_LCD_ADDR, data, sizeof(data), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_lcd_send_data(uint8_t data_byte)
{
    uint8_t data[2] = { LCD_DATA_MODE, data_byte };
    i2c_master_write_to_device(I2C_MASTER_NUM, I2C_LCD_ADDR, data, sizeof(data), I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static void i2c_lcd_display_string(const char *str)
{
    while (*str) {
        i2c_lcd_send_data((uint8_t)*str++);
    }
}

static void i2c_lcd_init(void)
{

    vTaskDelay(50 / portTICK_PERIOD_MS);

    i2c_lcd_send_command(0x38); 
    vTaskDelay(5 / portTICK_PERIOD_MS);

    i2c_lcd_send_command(0x0C);
    vTaskDelay(5 / portTICK_PERIOD_MS);

    i2c_lcd_send_command(0x01); 
    vTaskDelay(2 / portTICK_PERIOD_MS);

    i2c_lcd_send_command(0x06);
}

void app_main(void)
{
    i2c_master_init();
    i2c_lcd_init();
    i2c_lcd_display_string("Hello, World!");
}
