#include <stdio.h>
#include "esp_log.h"
#include <string.h>
#include <stdint.h>
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG                        "I2C_LCD"
#define LCD_RST_PIN                18
#define I2C_MASTER_SCL_IO          20
#define I2C_MASTER_SDA_IO          21
#define LCD_BACKLIGHT_PIN          GPIO_NUM_10
#define LCD_ENABLE_PIN             GPIO_NUM_11
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         100000
#define I2C_LCD_ADDR               0x3C  

#define I2C_TIMEOUT_MS             1000

#define LCD_COMMAND_MODE           0x00  
#define LCD_DATA_MODE              0x40  

void gpio_init( gpio_num_t pin_no ){
    gpio_config_t gpio_conf = {
        .pin_bit_mask = ( 1ULL << pin_no ),
        .mode = GPIO_MODE_OUTPUT, 
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config( &gpio_conf );
}

void backlight_init( void ){
    gpio_init( LCD_BACKLIGHT_PIN );
    gpio_init( LCD_ENABLE_PIN );
    gpio_set_level( LCD_BACKLIGHT_PIN, 1 );
    gpio_set_level( LCD_ENABLE_PIN, 1 );
}

void lcd_backlight_on( void ){
    gpio_set_level( LCD_BACKLIGHT_PIN, 1 );
}

void lcd_backlight_off( void ){
    gpio_set_level( LCD_BACKLIGHT_PIN, 0 );
}

void lcd_reset_pin_init( void ){
    gpio_config_t reset_pin_conf = {
        .pin_bit_mask = ( 1ULL << LCD_RST_PIN ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config( &reset_pin_conf );
    gpio_set_level( LCD_RST_PIN, 1 );
}

void lcd_reset( void ){
    gpio_set_level( LCD_RST_PIN, 0 );
    vTaskDelay( pdMS_TO_TICKS(10) );
    gpio_set_level( LCD_RST_PIN, 1 );
    vTaskDelay( pdMS_TO_TICKS(50) );
}

static void i2c_lcd_display_variable_length_string( const char* str ){
    if( str == NULL ){ return ; }

    size_t str_len = strlen( str );
    
    if( str_len == 0 ){ return ; }

    uint8_t *data = (uint8_t*)malloc( str_len + 1 );
    
    if( data == NULL ){
        return ;
    }
    data[0] = LCD_DATA_MODE;
    memcpy( &data[1], str, str_len );
    
    esp_err_t ret = i2c_master_write_to_device(
        I2C_MASTER_NUM,
        I2C_LCD_ADDR,
        data,
        str_len + 1,
        I2C_TIMEOUT_MS / portTICK_PERIOD_MS
    );

    if( ret != ESP_OK ){
        ESP_LOGE( TAG, "I2C write failed: %s\n", esp_err_to_name( ret ) );
    }
    free(data);
}

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
    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_lcd_send_command(0x39);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    i2c_lcd_send_command(0x14);
    i2c_lcd_send_command(0x78);
    i2c_lcd_send_command(0x5E);
    i2c_lcd_send_command(0x6D);
    i2c_lcd_send_command(0x0C);
    i2c_lcd_send_command(0x01);
    i2c_lcd_send_command(0x06);
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

void app_main(void)
{
    i2c_master_init();
    lcd_reset_pin_init();
    lcd_reset();
    backlight_init();
    i2c_lcd_init();

    while( 1 ){
        vTaskDelay( pdMS_TO_TICKS( 2500 ));
        lcd_backlight_on();
        i2c_lcd_init();
        i2c_lcd_display_string( "-" );
        i2c_lcd_display_variable_length_string( "Hello, World!" );
        i2c_lcd_display_string( "-" );
        vTaskDelay( pdMS_TO_TICKS( 2500 ));
        lcd_reset();
        lcd_backlight_off();
    }
    
}
