#include "uart_logger.h"
#include <cstdarg>
#include <cstdio>

uart_logger::uart_logger(uart_port_t uart_port)
    : m_uartPort(uart_port)
{}

void uart_logger::init() {
    uart_config_t uart_config = {};  // zero initialize everything
    uart_config.baud_rate = LOG_UART_BAUDRATE;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    uart_config.source_clk = UART_SCLK_APB;
    
    ESP_ERROR_CHECK(uart_param_config(m_uartPort, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(m_uartPort, LOG_UART_BUF_SIZE * 2, 0, 0, nullptr, 0));
}

void uart_logger::log(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    if(len > 0) {
        uart_write_bytes(m_uartPort, buffer, len);
    }
}
