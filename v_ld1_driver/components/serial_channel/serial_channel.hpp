#pragma once
#include "driver/uart.h"
#include "esp_err.h"
#include "driver/gpio.h"

class serial_channel {
public:
    serial_channel(uart_port_t port, gpio_num_t rx, gpio_num_t tx, int baud);
    esp_err_t begin();
    esp_err_t send(const char *opcode, const void *payload, size_t len);
    esp_err_t receive(void *buf, size_t len, uint32_t timeout_ms);
    void flush();
    esp_err_t updateBaud(int baud);

private:
    uart_port_t uart_num;
    gpio_num_t rxPin, txPin;
    int baudRate;
};
