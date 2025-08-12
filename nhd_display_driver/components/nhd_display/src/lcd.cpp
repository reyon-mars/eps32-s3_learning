#include "lcd.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "cstring"

NHDLcd::NHDLcd( const I2CMaster& i2c, const gpio_num_t rstPin, const gpio_num_t bcklghtPin, const gpio_num_t enPin ) :
    i2cMaster( i2c ),
    resetPin( rstPin ),
    backlightPin( bcklghtPin ),
    enablePin( enPin )
{
    reset();
    backlightPin.setLevel( 1 );
    enablePin.setLevel( 1 );
    init();
}

void NHDLcd::reset( void ){
    resetPin.setLevel( 0 );
    vTaskDelay( pdMS_TO_TICKS( 10 ) );
    resetPin.setLevel( 1 );
    vTaskDelay( pdMS_TO_TICKS( 50 ) );
}

void NHDLcd::sendCommand( uint8_t cmd ) const {
    std::array<uint8_t, 2> data { LCD_COMMAND_MODE, cmd };
    checkWrite( i2cMaster.write( LCD_ADDRESS, data, data.size() ));
}

void NHDLcd::sendData( uint8_t data_byte ) const {
    std::array<uint8_t, 2> data { LCD_DATA_MODE, data_byte };
    checkWrite( i2cMaster.write( LCD_ADDRESS, data, data.size() ));
}

void NHDLcd::dispaly