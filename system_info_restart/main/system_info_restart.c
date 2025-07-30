#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_log.h"
#include "driver/uart.h"

#define TAG "SYSTEM_INFO"
#define BUF_SIZE (1024)

void print_system_info(void) {
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    printf("\n========== System Info ==========\n");
    printf("ESP32 Chip Revision: %d\n", chip_info.revision);
    printf("Number of CPU cores: %d\n", chip_info.cores);
    printf("Wireless features   : %s%s%s\n",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT " : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "N/A");

    printf("Flash memory size   : %lu MB\n", flash_size / (1024 * 1024));
    printf("Free heap size      : %u bytes\n", esp_get_free_heap_size());
    printf("Minimum heap ever   : %u bytes\n", esp_get_minimum_free_heap_size());

    printf("IDF Version         : %s\n", esp_get_idf_version());
    printf("=================================\n\n");
}

void uart_command_listener_task(void *param) {
    const int uart_num = UART_NUM_0;
    uint8_t data[BUF_SIZE];

    while (1) {
        // Read bytes from UART with timeout of 1 tick
        int len = uart_read_bytes(uart_num, data, BUF_SIZE - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            data[len] = '\0';  // Null-terminate input for string comparison
            ESP_LOGI(TAG, "Received: %s", data);

            if (strncmp((char *)data, "restart", 7) == 0) {
                ESP_LOGI(TAG, "Restarting system in 1 second...");
                vTaskDelay(pdMS_TO_TICKS(1000));
                esp_restart();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // Reduce CPU usage
    }
}

void app_main(void) {
    // Print system info on boot
    print_system_info();

    // Configure UART0 (USB-CDC)
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);

    // Create a task to listen for UART input commands
    xTaskCreate(uart_command_listener_task, "uart_listener", 4096, NULL, 5, NULL);
}
