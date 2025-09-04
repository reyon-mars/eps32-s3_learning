#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>

#define SIZE(x) sizeof(x)
#define UART_VLD1 UART_NUM_1
#define VLD1_TX_PIN GPIO_NUM_12
#define VLD1_RX_PIN GPIO_NUM_13
#define BUF_SIZE 512

#define UART_RS485 UART_NUM_2
#define RS485_TX_PIN GPIO_NUM_17
#define RS485_RX_PIN GPIO_NUM_18
#define RS485_RE_DE_PIN GPIO_NUM_5

#define LED1_PIN GPIO_NUM_4
#define LED2_PIN GPIO_NUM_5
#define MAIN_LED_PIN GPIO_NUM_2

static const char *TAG = "VLD1_UART";

enum class vld1_distance_range_t : uint8_t { range_20 = 0, range_50 = 1 };
enum class target_filter_t : uint8_t { strongest = 0, nearest = 1, farthest = 2 };
enum class precision_mode_t : uint8_t { low = 0, high = 1 };

#pragma pack(push, 1)
struct vld1_packet_header { char header[4]; uint32_t length; };
#pragma pack(pop)

void init_uart_vld1() {
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

void init_uart_rs485() {
    uart_config_t uart_config = {};
    uart_config.baud_rate = 115200;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_param_config(UART_RS485, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_RS485, RS485_TX_PIN, RS485_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(UART_RS485, BUF_SIZE * 2, 0, 0, NULL, 0));

    gpio_set_direction(RS485_RE_DE_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RS485_RE_DE_PIN, 0);
}

void send_rs485(const uint8_t* data, size_t len) {
    gpio_set_level(RS485_RE_DE_PIN, 1);
    uart_write_bytes(UART_RS485, (const char*)data, len);
    uart_wait_tx_done(UART_RS485, pdMS_TO_TICKS(100));
    gpio_set_level(RS485_RE_DE_PIN, 0);
}

void init_leds() {
    gpio_set_direction(LED1_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(MAIN_LED_PIN, GPIO_MODE_OUTPUT);
}

void blink_led(gpio_num_t pin, int times, int delay_ms) {
    for(int i=0;i<times;i++){
        gpio_set_level(pin,1);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
        gpio_set_level(pin,0);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }
}

void send_vld1_packet(const char *header, const uint8_t *payload, uint32_t payload_len) {
    uint8_t buffer[BUF_SIZE];
    memcpy(buffer, header, 4);
    buffer[4] = payload_len & 0xFF;
    buffer[5] = (payload_len >> 8) & 0xFF;
    buffer[6] = (payload_len >> 16) & 0xFF;
    buffer[7] = (payload_len >> 24) & 0xFF;
    if(payload_len > 0 && payload != nullptr) memcpy(buffer+8, payload, payload_len);
    uart_write_bytes(UART_VLD1, (const char*)buffer, 8+payload_len);
    uart_wait_tx_done(UART_VLD1, pdMS_TO_TICKS(100));
}

int parse_vld1_message(uint8_t *buffer, int len, char *out_header, uint8_t *out_payload, uint32_t *out_len) {
    if(len < 8) return 0;
    memcpy(out_header, buffer, 4);
    *out_len = buffer[4] | (buffer[5]<<8) | (buffer[6]<<16) | (buffer[7]<<24);
    if(len < 8 + *out_len) return 0;
    if(*out_len>0) memcpy(out_payload, buffer+8, *out_len);
    return 8+*out_len;
}

uint16_t modbus_crc16(const uint8_t* buf, size_t len) {
    uint16_t crc = 0xFFFF;
    for(size_t pos=0; pos<len; pos++) {
        crc ^= (uint16_t)buf[pos];
        for(int i=0;i<8;i++) {
            if(crc & 0x0001) crc = (crc >> 1) ^ 0xA001;
            else crc = crc >> 1;
        }
    }
    return crc;
}

void forward_pdat_modbus(float distance, uint16_t magnitude) {
    uint8_t frame[13];
    frame[0] = 0x01; 
    frame[1] = 0x10; 
    frame[2] = 0x00; 
    frame[3] = 0x00; 
    frame[4] = 0x00; 
    frame[5] = 0x03;
    frame[6] = 0x06;  

    memcpy(&frame[7], &distance, sizeof(float)); 
    frame[11] = magnitude & 0xFF;               
    frame[12] = (magnitude >> 8) & 0xFF;         

    uint16_t crc = modbus_crc16(frame, 13);      
    uint8_t full_frame[15];
    memcpy(full_frame, frame, 13);
    full_frame[13] = crc & 0xFF;                 
    full_frame[14] = (crc >> 8) & 0xFF;          

    gpio_set_level(RS485_RE_DE_PIN, 1);
    uart_write_bytes(UART_RS485, (const char*)full_frame, 15);
    uart_wait_tx_done(UART_RS485, pdMS_TO_TICKS(100));
    gpio_set_level(RS485_RE_DE_PIN, 0);
}


void uart_read_task(void *arg) {
    uint8_t buffer[BUF_SIZE];
    int buf_len=0;
    while(1) {
        int len = uart_read_bytes(UART_VLD1, buffer+buf_len, BUF_SIZE-buf_len, pdMS_TO_TICKS(100));
        if(len>0) {
            buf_len+=len;
            int parsed_len;
            do {
                char header[5]={0};
                uint8_t payload[128]={0};
                uint32_t payload_len=0;
                parsed_len = parse_vld1_message(buffer, buf_len, header, payload, &payload_len);
                if(parsed_len>0){
                    if(strcmp(header,"RESP")==0){
                        ESP_LOGI(TAG,"RESP OK");
                    } else if(strcmp(header,"VERS")==0){
                        char fw[65]={0};
                        int copy_len = (payload_len<64)?payload_len:64;
                        memcpy(fw,payload,copy_len);
                        ESP_LOGI(TAG,"Firmware: %s", fw);
                    } else if(strcmp(header,"PDAT")==0){
                        if(payload_len>=6){
                            float distance;
                            memcpy(&distance,payload,sizeof(float));
                            uint16_t magnitude = payload[4] | (payload[5]<<8);
                            ESP_LOGI(TAG,"Distance [m]: %.3f m", distance);
                            ESP_LOGI(TAG,"Magnitude [dB]: %u", magnitude);
                            forward_pdat_modbus(distance, magnitude);
                        }
                    }
                    memmove(buffer, buffer+parsed_len, buf_len-parsed_len);
                    buf_len -= parsed_len;
                }
            } while(parsed_len>0 && buf_len>0);
        }
    }
}

void vld1_init_sequence() {
    uint8_t payload_init=0x00;
    send_vld1_packet("INIT",&payload_init,1);
    blink_led(MAIN_LED_PIN,2,200);
}

void vld1_read_distance_sequence() {
    uint8_t payload_gnfd=0x04;
    send_vld1_packet("GNFD",&payload_gnfd,1);
}

void vld1_set_range_param(vld1_distance_range_t range) {
    uint8_t payload = static_cast<uint8_t>(range);
    send_vld1_packet("RRAI",&payload,SIZE(payload));
}

void vld1_set_thres_offset(uint8_t val) {
    send_vld1_packet("THOF",&val,SIZE(val));
}

void vld1_set_min_range_filter(uint16_t val) {
    send_vld1_packet("MIRA",(uint8_t*)&val,SIZE(val));
}

void vld1_set_max_range_filter(uint16_t val) {
    send_vld1_packet("MARA",(uint8_t*)&val,SIZE(val));
}

void vld1_set_target_filter(target_filter_t filter) {
    uint8_t payload = static_cast<uint8_t>(filter);
    send_vld1_packet("TGFI",&payload,SIZE(payload));
}

void vld1_set_precision_mode(precision_mode_t mode) {
    uint8_t payload = static_cast<uint8_t>(mode);
    send_vld1_packet("PREC",&payload,SIZE(payload));
}

void vld1_exit_sequence() {
    send_vld1_packet("GBYE",nullptr,0);
}

extern "C" void app_main() {
    ESP_LOGI(TAG,"Starting V-LD1 + Modbus RTU Firmware");

    init_uart_vld1();
    init_uart_rs485();
    init_leds();

    gpio_set_level(MAIN_LED_PIN,1);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(MAIN_LED_PIN,0);

    xTaskCreate(uart_read_task,"uart_read_task",4096,NULL,10,NULL);

    vld1_init_sequence();

    while(1){
        vld1_read_distance_sequence();
        vTaskDelay(pdMS_TO_TICKS(1000));
        blink_led(LED1_PIN,1,50);
    }
}
