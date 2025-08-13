#include "freertos/FreeRTOS.h"
#include "i2c_master.hpp"
#include "lcd.hpp"
#include "freertos/task.h"
#include "esp_log.h"

constexpr gpio_num_t I2C_MASTER_SCL_IO     = GPIO_NUM_20;
constexpr gpio_num_t I2C_MASTER_SDA_IO     = GPIO_NUM_21;
constexpr gpio_num_t RESET_PIN             = GPIO_NUM_18;
constexpr gpio_num_t BACKLIGHT_PIN         = GPIO_NUM_10;
constexpr gpio_num_t ENABLE_PIN            = GPIO_NUM_11;
constexpr i2c_port_t I2C_MASTER_NUM        = I2C_NUM_0;
constexpr uint32_t   I2C_MASTER_FREQ_HZ    = 100000;
constexpr uint8_t    LINE_TWO              = 0x80 | 0x40;

extern "C" void app_main(void)
{

    I2CMaster i2c(I2C_MASTER_NUM, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    NHDLcd lcd(i2c, RESET_PIN, BACKLIGHT_PIN, ENABLE_PIN);

    int count = 0;
    lcd.setCursor(1, 1);

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(2500));
        lcd.backlightOn();
        lcd.display('-');
        lcd.display("Hello, World!");
        lcd.display('-');
        lcd.sendCommand(LINE_TWO);
        lcd.display("Count: ");
        lcd.display(count++);
        vTaskDelay(pdMS_TO_TICKS(2500));
        lcd.clear();
    }
}
