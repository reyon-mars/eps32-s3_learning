#pragma once

// V-LD1 UART
#define vld1_UART_PORT       UART_NUM_1
#define vld1_UART_TX_PIN     12   // V-LD1 TX -> ESP32 RX
#define vld1_UART_RX_PIN     13   // V-LD1 RX -> ESP32 TX
#define vld1_UART_BAUDRATE   115200
#define vld1_UART_BUF_SIZE   1024

// RS485 UART
#define RS485_UART_PORT      UART_NUM_2
#define RS485_TX_PIN         17   // RS485 DI
#define RS485_RX_PIN         18   // RS485 DO
#define RS485_RE_DE_PIN      5    // RE/DE control
#define RS485_UART_BAUDRATE  9600
#define RS485_UART_BUF_SIZE  1024

// LEDs
#define LED1_PIN             4
#define LED2_PIN             15
#define MAIN_LED_PIN         2

// USB UART logger
#define LOG_UART_PORT        UART_NUM_0
#define LOG_UART_BAUDRATE    115200
#define LOG_UART_BUF_SIZE    1024

// FreeRTOS task settings
#define SENSOR_TASK_STACK       4096
#define SENSOR_TASK_PRIORITY    10
#define LOG_TASK_STACK          4096
#define LOG_TASK_PRIORITY       5
