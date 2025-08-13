#pragma once

#include <array>
#include <string_view>
#include <string>
#include <vector>
#include <algorithm>
#include "i2c_master.hpp"
#include "gpio_pin.hpp"
#include "esp_log.h"

class NHDLcd
{
public:
    explicit NHDLcd(const I2CMaster &i2c, const gpio_num_t rstPin, const gpio_num_t bcklghtPin, const gpio_num_t enPin);
    void reset(void);
    void sendCommand(uint8_t cmd) const;
    void sendVarCommand(uint8_t* cmd, const size_t cmdLen) const;
    void sendData(uint8_t data_byte) const;
    void display(const std::string_view &str) const;
    void display(int number) const;
    void display(const char character) const;
    void setCursor(const uint8_t row, const uint8_t col) const;
    void backlightOn(void);
    void backlightOff(void);
    void clear(void);

private:
    void init() const;
    static void checkWrite(esp_err_t err);

    const I2CMaster &i2cMaster;
    GPIOPin resetPin;
    GPIOPin backlightPin;
    GPIOPin enablePin;

    static constexpr const char *TAG = "I2C_LCD";

    gpio_num_t LCD_RST_PIN;
    gpio_num_t LCD_BACKLIGHT_PIN;
    gpio_num_t LCD_ENABLE_PIN;

    static constexpr uint8_t LCD_ADDRESS = 0x3C;
    static constexpr uint8_t SET_CURSOR = 0x80;
    static constexpr uint8_t CLEAR_DISPLAY = 0x01;
    static constexpr uint8_t MOVE_LINE_TWO = 0x80 | 0x40;
    static constexpr uint8_t LCD_COMMAND_MODE = 0x00;
    static constexpr uint8_t LCD_DATA_MODE = 0x40;
};
