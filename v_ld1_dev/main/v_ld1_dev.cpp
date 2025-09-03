#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define UART_VLD1 UART_NUM_1
#define VLD1_TX_PIN GPIO_NUM_12
#define VLD1_RX_PIN GPIO_NUM_13
#define BUF_SIZE 512

#define LED1_PIN GPIO_NUM_4
#define LED2_PIN GPIO_NUM_5
#define MAIN_LED_PIN GPIO_NUM_2

static const char *TAG = "VLD1_UART";

#pragma pack(push, 1)
struct vld1_packet_header
{
    char header[4];
    uint32_t length;
};
#pragma pack(pop)

void init_uart_vld1()
{
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_EVEN;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_param_config(UART_VLD1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_VLD1, VLD1_TX_PIN, VLD1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_VLD1, BUF_SIZE * 2, 0, 0, NULL, 0));
}

void init_leds()
{
    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MAIN_LED_PIN, GPIO_MODE_OUTPUT);
}

void blink_led(gpio_num_t pin, int times, int delay_ms)
{
    for (int i = 0; i < times; i++)
    {
        gpio_set_level(pin, 1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(pin, 0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void send_vld1_packet(const char *header, const uint8_t *payload, uint32_t payload_len)
{
    uint8_t buffer[BUF_SIZE];
    memcpy(buffer, header, 4);
    buffer[4] = payload_len & 0xFF;
    buffer[5] = (payload_len >> 8) & 0xFF;
    buffer[6] = (payload_len >> 16) & 0xFF;
    buffer[7] = (payload_len >> 24) & 0xFF;
    if (payload_len > 0 && payload != nullptr)
    {
        memcpy(buffer + 8, payload, payload_len);
    }
    uart_write_bytes(UART_VLD1, (const char *)buffer, 8 + payload_len);
    uart_wait_tx_done(UART_VLD1, pdMS_TO_TICKS(100));
}

int parse_vld1_message(uint8_t *buffer, int len, char *out_header, uint8_t *out_payload, uint32_t *out_len)
{
    if (len < 8)
        return 0;
    memcpy(out_header, buffer, 4);
    *out_len = buffer[4] | (buffer[5] << 8) | (buffer[6] << 16) | (buffer[7] << 24);
    if (len < 8 + *out_len)
        return 0;
    if (*out_len > 0)
        memcpy(out_payload, buffer + 8, *out_len);
    return 8 + *out_len;
}

void uart_read_task(void *arg)
{
    uint8_t buffer[BUF_SIZE];
    int buf_len = 0;

    while (1)
    {
        int len = uart_read_bytes(UART_VLD1, buffer + buf_len, BUF_SIZE - buf_len, pdMS_TO_TICKS(100));
        if (len > 0)
        {
            buf_len += len;

            int parsed_len = 0;
            do
            {
                char header[5] = {0};
                uint8_t payload[128] = {0};
                uint32_t payload_len = 0;

                parsed_len = parse_vld1_message(buffer, buf_len, header, payload, &payload_len);
                if (parsed_len > 0)
                {
                    if (strcmp(header, "RESP") == 0)
                    {
                        ESP_LOGI(TAG, "RESP OK");
                    }
                    else if (strcmp(header, "VERS") == 0)
                    {
                        char fw[65] = {0};
                        int copy_len = (payload_len < 64) ? payload_len : 64;
                        memcpy(fw, payload, copy_len);
                        ESP_LOGI(TAG, "Firmware: %s", fw);
                    }
                    else if (strcmp(header, "PDAT") == 0)
                    {
                        if (payload_len >= 6)
                        {
                            float distance;
                            memcpy(&distance, payload, sizeof(float));
                            uint16_t magnitude = payload[4] | (payload[5] << 8);
                            ESP_LOGI(TAG, "Distance [m]: %.3f m", distance);
                            ESP_LOGI(TAG, "Magnitude [db] : %u db", (magnitude / 100));
                        }
                    }
                    memmove(buffer, buffer + parsed_len, buf_len - parsed_len);
                    buf_len -= parsed_len;
                }
            } while (parsed_len > 0 && buf_len > 0);
        }
    }
}

void vld1_init_sequence()
{
    uint8_t payload_init = 0x00;
    send_vld1_packet("INIT", &payload_init, 1);
    blink_led(MAIN_LED_PIN, 2, 200);
}

void vld1_read_distance_sequence()
{
    uint8_t payload_gnfd = 0x04;
    send_vld1_packet("GNFD", &payload_gnfd, 1);
}

void vld1_exit_sequence()
{
    send_vld1_packet("GBYE", nullptr, 0);
}

extern "C" void app_main()
{
    ESP_LOGI(TAG, "Starting V-LD1 UART Firmware");

    init_uart_vld1();
    init_leds();

    gpio_set_level(MAIN_LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(MAIN_LED_PIN, 0);

    xTaskCreate(uart_read_task, "uart_read_task", 4096, NULL, 10, NULL);

    vld1_init_sequence();

    while (1)
    {
        vld1_read_distance_sequence();
        vTaskDelay(pdMS_TO_TICKS(1000));
        blink_led(LED1_PIN, 1, 50);
    }
}
