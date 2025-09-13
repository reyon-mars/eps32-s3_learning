#include "hal_uart.hpp"
#include "hal_rs485.hpp"
#include "led_manager.hpp"
#include "vld1.hpp"
#include "modbus_forwarder.hpp"
#include "averager.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "app_main";

// pins (copy from your original)
constexpr int VLD1_TX_PIN = 12;
constexpr int VLD1_RX_PIN = 13;
constexpr uart_port_t UART_VLD1 = UART_NUM_1;

constexpr int RS485_TX_PIN = 17;
constexpr int RS485_RX_PIN = 18;
constexpr gpio_num_t RS485_RE_DE_PIN = GPIO_NUM_5;
constexpr uart_port_t UART_RS485 = UART_NUM_2;

constexpr gpio_num_t LED1_PIN = GPIO_NUM_4;
constexpr gpio_num_t MAIN_LED_PIN = GPIO_NUM_2;

struct AppContext {
    hal::Uart* vld_uart;
    hal::Uart* rs485_uart;
    hal::Rs485* rs485;
    Vld1* vld;
    ModbusForwarder* modbus;
    BatchAverager* averager;
    LedManager* led;
};

static void uart_read_task(void* arg) {
    AppContext* ctx = static_cast<AppContext*>(arg);

    uint8_t buffer[512];
    int buf_len = 0;

    while (true) {
        int read = ctx->vld_uart->read(buffer + buf_len, sizeof(buffer) - buf_len, pdMS_TO_TICKS(100));
        if (read > 0) {
            buf_len += read;
            int parsed_len = 0;
            do {
                char header[5] = {0};
                uint8_t payload[128] = {0};
                uint32_t payload_len = 0;
                parsed_len = ctx->vld->parseMessage(buffer, buf_len, header, payload, &payload_len);
                if (parsed_len > 0) {
                    if (std::strcmp(header, "PDAT") == 0 && payload_len >= 6) {
                        float distance_m;
                        std::memcpy(&distance_m, payload, sizeof(float));
                        uint16_t magnitude = static_cast<uint16_t>(payload[4]) | (static_cast<uint16_t>(payload[5]) << 8);

                        uint16_t distance_mm = static_cast<uint16_t>(distance_m * 1000.0f);

                        ctx->averager->addSample(distance_m);
                        uint16_t avg_mm = ctx->averager->averageMillimeters();

                        // forward every sample with current running batch average
                        ctx->modbus->forwardPDAT(distance_mm, magnitude, avg_mm);

                        // if batch complete, log and reset
                        if (ctx->averager->isComplete()) {
                            int64_t elapsed = ctx->averager->elapsedUs();
                            ESP_LOGI(TAG, "Batch complete: avg=%.3f m (%u mm), time=%" PRId64 " us", ctx->averager->averageMeters(), avg_mm, elapsed);
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
    ESP_LOGI(TAG, "Starting modular V-LD1 app");

    // create HAL objects
    static hal::Uart vld_uart(UART_VLD1, VLD1_TX_PIN, VLD1_RX_PIN, 512);
    static hal::Uart rs485_uart(UART_RS485, RS485_TX_PIN, RS485_RX_PIN, 512);

    vld_uart.init(115200);
    rs485_uart.init(115200);

    static hal::Rs485 rs485(rs485_uart, RS485_RE_DE_PIN);
    rs485.init();

    static LedManager led_main(MAIN_LED_PIN);
    led_main.init();
    led_main.blink(1, 100);

    static Vld1 vld(vld_uart);
    static ModbusForwarder modbus(rs485);
    static BatchAverager averager(20);

    static AppContext ctx = {
        &vld_uart,
        &rs485_uart,
        &rs485,
        &vld,
        &modbus,
        &averager,
        &led_main
    };

    // spawn reader task
    xTaskCreate(uart_read_task, "uart_read_task", 4096, &ctx, 10, NULL);

    // example loop that triggers GNFD (distance read)
    while (true) {
        const uint8_t payload_gnfd = 0x04;
        vld.sendPacket("GNFD", &payload_gnfd, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

