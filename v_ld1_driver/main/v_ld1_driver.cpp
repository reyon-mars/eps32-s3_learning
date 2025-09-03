#include "board_config.h"
#include "vld1.h"
#include "uart_logger.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static vld1 sensor;
static uart_logger logger;

extern "C" void app_main() {
    logger.init();
    sensor.init();

    // Sensor task
    xTaskCreate([](void*) {
        ESP_LOGI("SensorTask", "Sensor task started");
        ESP_LOGI("RS485Task", "RS485 task started");
        uint8_t buffer[vld1_UART_BUF_SIZE];
        while(true) {
            int len = sensor.readData(buffer, sizeof(buffer));
            if(len > 0) {
                // For simplicity, print raw bytes as hex
                for(int i = 0; i < len; i++) {
                    char hex[4];
                    snprintf(hex, sizeof(hex), "%02X ", buffer[i]);
                    uart_write_bytes(LOG_UART_PORT, hex, 3);
                }
                uart_write_bytes(LOG_UART_PORT, "\r\n", 2);
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }, "SensorTask", SENSOR_TASK_STACK, nullptr, SENSOR_TASK_PRIORITY, nullptr);
}
