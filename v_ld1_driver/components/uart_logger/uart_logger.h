#pragma once
#include "board_config.h"
#include "driver/uart.h"
#include "esp_log.h"

class uart_logger {
public:
    uart_logger(uart_port_t uart_port = LOG_UART_PORT);
    void init();
    void log(const char* format, ...);
private:
    uart_port_t m_uartPort;
};
