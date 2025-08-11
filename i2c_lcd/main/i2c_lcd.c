#include <cstdio>
#include <cstring>
#include <cstdint>
#include <cstdlib>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class I2CLcd {
public:
    static constexpr const char* TAG = "I2C_LCD";
    static constexpr gpio_num_t LCD_RST_PIN = GPIO_NUM_18;
    static constexpr gpio_num_t LCD_BACKLIGHT_PIN = GPIO_NUM_10;
    static constexpr gpio_num_t LCD_ENABLE_PIN = GPIO_NUM_11;
    static constexpr gpio_num_t I2C_MASTER_SCL_IO = GPIO_NUM_20;
    static constexpr gpio_num_t I2C_MASTER_SDA_IO = GPIO_NUM_21;
    static constexpr i2c_port_t I2C_MASTER_NUM = I2C_NUM_0;
    static constexpr uint32_t I2C_MASTER_FREQ_HZ = 100000;
    static constexpr uint8_t I2C_LCD_ADDR = 0x3C;
    static constexpr int I2C_TIMEOUT_MS = 1000;
    static constexpr uint8_t LCD_COMMAND_MODE = 0x00;
    static constexpr uint8_t LCD_DATA_MODE = 0x40;

    I2CLcd() = default;

    void init() {
        i2c_master_init();
        lcd_reset_pin_init();
        lcd_reset();
        backlight_init();
        lcd_hw_init();
    }

    void backlightOn() {
        gpio_set_level(LCD_BACKLIGHT_PIN, 1);
    }

    void backlightOff() {
        gpio_set_level(LCD_BACKLIGHT_PIN, 0);
    }

    void reset() {
        lcd_reset();
    }

    void display(const char* str) {
        while (*str) {
            sendData(static_cast<uint8_t>(*str++));
        }
    }

    void displayVarLen(const char* str) {
        if (!str) return;
        size_t len = std::strlen(str);
        if (len == 0) return;

        uint8_t* data = static_cast<uint8_t*>(std::malloc(len + 1));
        if (!data) return;

        data[0] = LCD_DATA_MODE;
        std::memcpy(&data[1], str, len);

        esp_err_t ret = i2c_master_write_to_device(
            I2C_MASTER_NUM, I2C_LCD_ADDR, data, len + 1,
            I2C_TIMEOUT_MS / portTICK_PERIOD_MS
        );

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "I2C write failed: %s\n", esp_err_to_name(ret));
        }
        std::free(data);
    }

    void sendCmd(uint8_t cmd) {
        uint8_t data[2] = { LCD_COMMAND_MODE, cmd };
        i2c_master_write_to_device(
            I2C_MASTER_NUM, I2C_LCD_ADDR, data, sizeof(data),
            I2C_TIMEOUT_MS / portTICK_PERIOD_MS
        );
    }

private:
    void gpioInit(gpio_num_t pin) {
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << pin),
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&conf);
    }

    void backlight_init() {
        gpioInit(LCD_BACKLIGHT_PIN);
        gpioInit(LCD_ENABLE_PIN);
        gpio_set_level(LCD_BACKLIGHT_PIN, 1);
        gpio_set_level(LCD_ENABLE_PIN, 1);
    }

    void lcd_reset_pin_init() {
        gpio_config_t conf = {
            .pin_bit_mask = (1ULL << LCD_RST_PIN),
            .mode = GPIO_MODE_OUTPUT,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .intr_type = GPIO_INTR_DISABLE
        };
        gpio_config(&conf);
        gpio_set_level(LCD_RST_PIN, 1);
    }

    void lcd_reset() {
        gpio_set_level(LCD_RST_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(LCD_RST_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    void i2c_master_init() {
        i2c_config_t conf = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_MASTER_SDA_IO,
            .sda_pullup_en = GPIO_PULLUP_ENABLE,
            .scl_io_num = I2C_MASTER_SCL_IO,
            .scl_pullup_en = GPIO_PULLUP_ENABLE,
            .master = { .clk_speed = I2C_MASTER_FREQ_HZ }
        };
        i2c_param_config(I2C_MASTER_NUM, &conf);
        i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    }

    void sendData(uint8_t dataByte) {
        uint8_t data[2] = { LCD_DATA_MODE, dataByte };
        i2c_master_write_to_device(
            I2C_MASTER_NUM, I2C_LCD_ADDR, data, sizeof(data),
            I2C_TIMEOUT_MS / portTICK_PERIOD_MS
        );
    }

    void lcd_hw_init() {
        vTaskDelay(50 / portTICK_PERIOD_MS);
        sendCmd(0x38);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sendCmd(0x39);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        sendCmd(0x14);
        sendCmd(0x78);
        sendCmd(0x5E);
        sendCmd(0x6D);
        sendCmd(0x0C);
        sendCmd(0x01);
        sendCmd(0x06);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
};

void app_main(void) {
    I2CLcd lcd;
    lcd.init();

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(2500));
        lcd.backlightOn();
        lcd.display("-");
        lcd.displayVarLen("Hello, World!");
        lcd.display("-");
        vTaskDelay(pdMS_TO_TICKS(2500));
        lcd.reset();
        lcd.backlightOff();
    }
}
