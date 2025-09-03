#include "serial_channel.h"
#include "driver/uart.h"
#include "esp_log.h"

SerialChannel::SerialChannel(uart_port_t port, gpio_num_t rx, gpio_num_t tx, int baud)
: uart_num(port), rxPin(rx), txPin(tx), baudRate(baud) {}

esp_err_t SerialChannel::begin() {
    uart_config_t config = {
        .baud_rate = baudRate,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(uart_num, 2048, 2048, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, txPin, rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    return ESP_OK;
}

esp_err_t SerialChannel::send(const char *opcode, const void *payload, size_t len) {
    uart_write_bytes(uart_num, opcode, 4);
    uart_write_bytes(uart_num, (const char*)payload, len);
    return ESP_OK;
}

esp_err_t SerialChannel::receive(void *buf, size_t len, uint32_t timeout_ms) {
    int r = uart_read_bytes(uart_num, (uint8_t*)buf, len, pdMS_TO_TICKS(timeout_ms));
    return (r == (int)len) ? ESP_OK : ESP_FAIL;
}

void SerialChannel::flush() {
    uart_flush(uart_num);
}

esp_err_t SerialChannel::updateBaud(int baud) {
    baudRate = baud;
    return uart_set_baudrate(uart_num, baud);
}
