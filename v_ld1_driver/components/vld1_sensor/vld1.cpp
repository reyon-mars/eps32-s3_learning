#include "vld1.h"
#include "esp_log.h"

static const char* TAG = "vld1";

vld1::vld1(uart_port_t uart_port)
    : m_uartPort(uart_port), m_uartQueue(nullptr)
{}

void vld1::init() {
    uart_config_t uart_config = {
        .baud_rate = vld1_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB
    };
    ESP_ERROR_CHECK(uart_param_config(m_uartPort, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(m_uartPort, vld1_UART_TX_PIN, vld1_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(m_uartPort, vld1_UART_BUF_SIZE * 2, vld1_UART_BUF_SIZE * 2, 10, &m_uartQueue, 0));
    ESP_LOGI(TAG, "vld1 UART initialized");
}

int vld1::readData(uint8_t* buffer, size_t length) {
    if (!buffer || length == 0) return 0;
    int len = uart_read_bytes(m_uartPort, buffer, length, pdMS_TO_TICKS(100));
    return len; // Returns number of bytes read
}
