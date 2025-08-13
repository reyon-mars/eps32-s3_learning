#include "lcd.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

NHDLcd::NHDLcd( const I2CMaster& i2c, const gpio_num_t rstPin, const gpio_num_t bcklghtPin, const gpio_num_t enPin ) :
    i2cMaster( i2c ),
    resetPin( rstPin, GPIOPin::Mode::out, false, false ),
    backlightPin( bcklghtPin, GPIOPin::Mode::out, false, false ),
    enablePin( enPin, GPIOPin::Mode::out, false, false )
{
    reset();
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
    checkWrite( i2cMaster.write( LCD_ADDRESS, data.data(), data.size() ));
}

void NHDLcd::sendVarCommand( uint8_t* cmd, const size_t cmdLen ) const{
    checkWrite( i2cMaster.write( LCD_ADDRESS, cmd, cmdLen ));
}

void NHDLcd::sendData( uint8_t data_byte ) const {
    std::array<uint8_t, 2> data { LCD_DATA_MODE, data_byte };
    checkWrite( i2cMaster.write( LCD_ADDRESS, data.data(), data.size() ));
}

void NHDLcd::display( const std::string_view& str ) const
{
    if (str.empty()){
        return;
    }

    std::vector<uint8_t> buf(str.size() + 1);
    buf[0] = LCD_DATA_MODE;
    const uint8_t* src = reinterpret_cast<const uint8_t*>(str.data());
    uint8_t* dest = &buf[1];
    const size_t count = str.size();
    for (size_t i = 0; i < count; ++i) {
        dest[i] = src[i];
    }

    checkWrite(i2cMaster.write( LCD_ADDRESS, buf.data(), buf.size()));
}

void NHDLcd::display( const char character ) const{
    sendData( static_cast<uint8_t>( character ) );
}

void NHDLcd::display( int number ) const{
    if( number == 0 ){
        display( '0' );
        return;
    }
    if( number < 0 ){
        display( '-' );
        auto absNumber = static_cast<int64_t>( number ) * -1;
        number = static_cast<int>( absNumber );
    }

    std::string buf;
    buf.reserve( 11 );
    while( number ){
        buf.push_back( static_cast<char> ( '0' + ( number % 10 )));
        number /= 10;
    }
    std::reverse( buf.begin(), buf.end() );
    display( buf );
}

void NHDLcd::setCursor( const uint8_t row, const uint8_t col ) const {
    if( row == 0 || row > 2 || col == 0 || col > 20 ){
        return;
    }
    constexpr std::array<uint8_t, 2> rowOffsets{ 0x00, 0x40 };
    uint8_t addr = rowOffsets[ row - 1 ] + ( col % 20 );
    addr &= 0x7F;
    sendCommand( SET_CURSOR | addr );
}

void NHDLcd::backlightOff( void ){
    backlightPin.setLevel( 0 );
}

void NHDLcd::backlightOn( void ){
    backlightPin.setLevel( 1 );
}

void NHDLcd::clear( void ){
    sendCommand( CLEAR_DISPLAY );
}

void NHDLcd::init( void ) const {
    vTaskDelay(pdMS_TO_TICKS(50));
    sendCommand(0x38);
    vTaskDelay(pdMS_TO_TICKS(10));
    sendCommand(0x39);
    vTaskDelay(pdMS_TO_TICKS(10));
    std::array< uint8_t, 7 > cmdList = {
                                            0x14,
                                            0x78,
                                            0x5E,
                                            0x6D,
                                            0x0C,
                                            0x01,
                                            0x06
                                        };
    sendVarCommand( cmdList.data(), cmdList.size() );
    vTaskDelay(pdMS_TO_TICKS(10));
}

void NHDLcd::checkWrite( esp_err_t err ){
    if( err != ESP_OK ){
        ESP_LOGE( TAG, "I2C write failed: %s ", esp_err_to_name( err ) );
    }
}