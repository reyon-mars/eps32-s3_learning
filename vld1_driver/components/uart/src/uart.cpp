#include "uart.hpp"
#include "esp_log.h"
#include "driver/uart.h"
#include "esp_err.h"
#include <cstring>

static constexpr char TAG[] = "Uart";

uart::uart(uart_port_t port, int tx_pin, int rx_pin, int baud_rate, size_t rx_buf_size) noexcept
    : port_(port), tx_pin_(tx_pin), rx_pin_(rx_pin), baud_rate_(baud_rate), rx_buf_size_(rx_buf_size) {}

esp_err_t uart::init(uart_word_length_t data_bits, uart_parity_t parity, uart_stop_bits_t stop_bits) noexcept
{
    uart_config_t cfg = {};
    cfg.baud_rate = baud_rate_;
    cfg.data_bits = data_bits;
    cfg.parity = parity;
    cfg.stop_bits = stop_bits;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    esp_err_t err = uart_param_config(port_, &cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_set_pin(port_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin failed: %s", esp_err_to_name(err));
        return err;
    }

    err = uart_driver_install(port_, rx_buf_size_ * 2, 0, 0, NULL, 0);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "UART%u initialized (TX=%d RX=%d @%d)", port_, tx_pin_, rx_pin_, baud);
    return ESP_OK;
}

int uart::read(uint8_t *dst, size_t max_len, TickType_t ticks_to_wait) noexcept
{
    return uart_read_bytes(port_, dst, max_len, ticks_to_wait);
}

int uart::write(const uint8_t *data, size_t len) noexcept
{
    int written = uart_write_bytes(port_, reinterpret_cast<const char *>(data), len);
    if (written > 0)
    {
        uart_wait_tx_done(port_, pdMS_TO_TICKS(100));
    }
    return written;
}
