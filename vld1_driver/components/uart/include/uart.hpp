#pragma once
#include <cstddef>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"

class uart
{
public:
    uart(uart_port_t port, int tx_pin, int rx_pin, size_t rx_buf_size = 512) noexcept;

    esp_err_t init(int baud = 115200,
                   uart_word_length_t data_bits = UART_DATA_8_BITS,
                   uart_parity_t parity = UART_PARITY_EVEN,
                   uart_stop_bits_t stop_bits = UART_STOP_BITS_1) noexcept;

    int read(uint8_t *dst, size_t max_len, TickType_t ticks_to_wait) noexcept;
    int write(const uint8_t *data, size_t len) noexcept;

    uart_port_t port() const noexcept { return port_; }

private:
    uart_port_t port_;
    int tx_pin_;
    int rx_pin_;
    size_t rx_buf_size_;
};
