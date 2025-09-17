#include "uart.hpp"
#include "rs485_slave.hpp"
#include "vld1.hpp"
#include "averager.hpp"
#include "led.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static constexpr char TAG[] = "app_main";

// pins
constexpr int VLD1_TX_PIN = 12;
constexpr int VLD1_RX_PIN = 13;
constexpr uart_port_t UART_VLD1 = UART_NUM_1;

constexpr int RS485_TX_PIN = 17;
constexpr int RS485_RX_PIN = 18;
constexpr gpio_num_t RS485_RE_DE_PIN = GPIO_NUM_5;
constexpr uart_port_t UART_RS485 = UART_NUM_2;

constexpr gpio_num_t MAIN_LED_PIN = GPIO_NUM_2;

struct AppContext {
    uart* vld_uart;
    uart* rs485_uart;
    rs485* rs485_slave;
    vld1* vld;
    batch_averager* averager;
    led* main_led;
};

static void uart_read_task(void* arg) {
    AppContext* ctx = static_cast<AppContext*>(arg);

    uint8_t buffer[512];
    int buf_len = 0;

    while (true) {
        int read_bytes = ctx->vld_uart->read(buffer + buf_len, sizeof(buffer) - buf_len, pdMS_TO_TICKS(100));
        if (read_bytes > 0) {
            buf_len += read_bytes;
            int parsed_len = 0;
            do {
                char header[5] = {0};
                uint8_t payload[128] = {0};
                uint32_t payload_len = 0;

                parsed_len = ctx->vld->parse_message(buffer, buf_len, header, payload, &payload_len);
                if (parsed_len > 0) {
                    if (std::strcmp(header, "PDAT") == 0 && payload_len >= 6) {
                        float distance_m;
                        std::memcpy(&distance_m, payload, sizeof(float));
                        uint16_t magnitude = static_cast<uint16_t>(payload[4]) | (static_cast<uint16_t>(payload[5]) << 8);
                        uint16_t distance_mm = static_cast<uint16_t>(distance_m * 1000.0f);

                        ctx->averager->add_sample(distance_m);
                        uint16_t avg_mm = ctx->averager->average_millimeters();

                        // forward over RS485 Modbus input registers
                        ctx->rs485_slave->write(&distance_mm, 1, 0);           // addr 0
                        ctx->rs485_slave->write(&magnitude, 1, 1);             // addr 1
                        ctx->rs485_slave->write(&avg_mm, 1, 2);                // addr 2

                        if (ctx->averager->is_complete()) {
                            ESP_LOGI(TAG, "Batch complete: avg=%.3f m (%u mm)", ctx->averager->average_meters(), avg_mm);
                            ctx->averager->reset();
                        }
                    } else if (std::strcmp(header, "RESP") == 0) {
                        ESP_LOGI(TAG, "VLD1 RESP received");
                    } else if (std::strcmp(header, "VERS") == 0) {
                        char fw[65] = {0};
                        int copy_len = (payload_len < 64) ? static_cast<int>(payload_len) : 64;
                        std::memcpy(fw, payload, copy_len);
                        ESP_LOGI(TAG, "VLD1 Firmware: %s", fw);
                    }

                    // shift buffer
                    std::memmove(buffer, buffer + parsed_len, buf_len - parsed_len);
                    buf_len -= parsed_len;
                }
            } while (parsed_len > 0 && buf_len > 0);
        }
    }
}

extern "C" void app_main() {
    ESP_LOGI(TAG, "Starting minimal VLD1 app");

    // UARTs
    static uart vld_uart(UART_VLD1, VLD1_TX_PIN, VLD1_RX_PIN, 115200, 512);
    static uart rs485_uart(UART_RS485, RS485_TX_PIN, RS485_RX_PIN, 115200, 512);

    vld_uart.init();
    rs485_uart.init();

    // RS485 slave with 3 input registers
    static rs485 rs485_slave(rs485_uart, RS485_RE_DE_PIN);
    rs485_slave.init(1, MB_PARAM_INPUT, 3);

    // Main LED
    static led main_led(MAIN_LED_PIN);
    main_led.init();
    main_led.blink(1, 100);

    // VLD1 parser
    static vld1 vld(vld_uart);

    // Batch averager
    static batch_averager averager(20);

    // Application context
    static AppContext ctx = {
        &vld_uart,
        &rs485_uart,
        &rs485_slave,
        &vld,
        &averager,
        &main_led
    };

    // Spawn UART read task
    xTaskCreate(uart_read_task, "uart_read_task", 4096, &ctx, 10, nullptr);

    // Periodically request distance measurement
    while (true) {
        const uint8_t payload_gnfd = 0x04;
        vld1::vld1_header_t header{};
        std::memcpy(header.header, "GNFD", 4);
        header.payload_len = 1;
        vld.send_packet(header, &payload_gnfd);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
