#pragma once
#include "uart.hpp"
#include "rs485_slave.hpp"
#include "vld1.hpp"
#include "averager.hpp"
#include "led.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

struct AppContext {
    uart* vld_uart;
    uart* rs485_uart;
    rs485* rs485_slave;
    vld1* vld;
    batch_averager* averager;
    led* main_led;
};

class Application {
public:
    Application(uart& vld, uart& rs, rs485& rs_slave, vld1& v, batch_averager& avg, led& led_main)
        : ctx_{ &vld, &rs, &rs_slave, &v, &avg, &led_main } {}

    void start();

private:
    static void uart_read_task(void* arg);
    AppContext ctx_;
};
