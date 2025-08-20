#include "driver/spi_master.h"
#include "driver/gpio.h"

#define PIN_NUM_MISO  19
#define PIN_NUM_MOSI  23
#define PIN_NUM_CLK   18
#define PIN_NUM_CS    5
#define PIN_NUM_BUSY  21
#define PIN_NUM_RESET 22

spi_device_handle_t lora;

void lora_init_spi() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1
    };

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000, // 1 MHz to start
        .mode = 0,
        .spics_io_num = PIN_NUM_CS,
        .queue_size = 1,
    };

    spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    spi_bus_add_device(SPI2_HOST, &devcfg, &lora);
}

uint32_t lora_get_version() {
    uint8_t cmd = 0xC0;  // Get Version
    uint8_t rx[3] = {0};

    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &cmd,
    };
    spi_device_transmit(lora, &t);

    spi_transaction_t r = {
        .length = 24, // 3 bytes
        .rx_buffer = rx,
    };
    spi_device_transmit(lora, &r);

    return (rx[0] << 16) | (rx[1] << 8) | rx[2];
}
