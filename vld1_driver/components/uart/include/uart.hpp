#pragma once
#include <cstddef>
#include <cstdint>
#include "freertos/FreeRTOS.h"
#include "driver/uart.h"

class uart
{
public:
    uart(uart_port_t port, int tx_pin, int rx_pin, int baud_rate = 115200, size_t rx_buf_size = 512) noexcept;

    esp_err_t init(uart_word_length_t data_bits = UART_DATA_8_BITS,
                   uart_parity_t parity = UART_PARITY_EVEN,
                   uart_stop_bits_t stop_bits = UART_STOP_BITS_1) noexcept;

    int read(uint8_t *dst, size_t max_len, TickType_t ticks_to_wait) noexcept;
    int write(const uint8_t *data, size_t len) noexcept;

    uart_port_t port() const noexcept { return port_; }

    int baud(void) const noexcept { return baud_rate_; }

private:
    uart_port_t port_;
    int tx_pin_;
    int rx_pin_;
    int baud_rate_;
    size_t rx_buf_size_;
};
