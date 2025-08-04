#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#define TAG "RESET_MONITOR"
#define MAX_INPUT_LEN 128

void reset_monitor_task(void *arg) {
    char input[MAX_INPUT_LEN];

    while (true) {
        printf("\nType 'restart' to reboot the ESP32-S3:\n> ");
        fflush(stdout);  // Ensure prompt is visible

        if (fgets(input, MAX_INPUT_LEN, stdin) != NULL) {
            // Remove trailing newline
            input[strcspn(input, "\r\n")] = '\0';

            ESP_LOGI(TAG, "Received input: '%s'", input);

            if (strcmp(input, "restart") == 0) {
                ESP_LOGI(TAG, "Restarting now...");
                vTaskDelay(pdMS_TO_TICKS(500));  // Let logs flush
                esp_restart();
            } else {
                printf("Unrecognized command: '%s'\n", input);
            }
        } else {
            ESP_LOGW(TAG, "Failed to read input. Retrying...");
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Reset Command via Console Started");
    xTaskCreate(reset_monitor_task, "reset_monitor_task", 4096, NULL, 5, NULL);
}
