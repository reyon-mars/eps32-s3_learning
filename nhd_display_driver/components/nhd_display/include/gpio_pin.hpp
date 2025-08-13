#pragma once

#include "driver/gpio.h"

class GPIOPin
{
public:
    enum class Mode : uint8_t
    {
        in = GPIO_MODE_INPUT,
        out = GPIO_MODE_OUTPUT,
        InputOutput = GPIO_MODE_INPUT_OUTPUT,
        OutputOpenDrain = GPIO_MODE_OUTPUT_OD,
        InputOutputOpenDrain = GPIO_MODE_INPUT_OUTPUT_OD
    };

    explicit GPIOPin(gpio_num_t pin, Mode mode, bool pullup, bool pulldown);
    void setLevel(int level) const;

private:
    gpio_num_t pinNo;
};