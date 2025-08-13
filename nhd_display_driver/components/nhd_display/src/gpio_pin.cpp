#include "gpio_pin.hpp"
#include "driver/gpio.h"

GPIOPin::GPIOPin( gpio_num_t pin, Mode mode, bool pullup, bool pulldown ) : pinNo( pin )
{
    gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pinNo),
        .mode = static_cast<gpio_mode_t>(mode),
        .pull_up_en = (pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE),
        .pull_down_en = (pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE),
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config( &conf );
    gpio_set_level( pinNo, 1 );
}

void GPIOPin::setLevel( int level ) const {
    gpio_set_level( pinNo, level );
}