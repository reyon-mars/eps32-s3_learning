#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_chip_info.h"
#include "esp_log.h"

static const char* TAG = "SYSTEM_INFO";

void app_main(void)
{
    // Get chip info
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    ESP_LOGI(TAG, "This is ESP32-S3 with %d CPU core(s)", chip_info.cores);
    ESP_LOGI(TAG, "WiFi%s%s",
             (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
             (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
    ESP_LOGI(TAG, "Silicon revision: %d", chip_info.revision);
    ESP_LOGI(TAG, "Flash: %dMB %s flash", spi_flash_get_chip_size() / (1024 * 1024),
             (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    // Print heap size
    ESP_LOGI(TAG, "Free heap: %d bytes", esp_get_free_heap_size());

    // Delay for observation
    vTaskDelay(pdMS_TO_TICKS(5000));

    // Restart device
    ESP_LOGW(TAG, "Restarting now...");
    fflush(stdout);
    esp_restart();
}
