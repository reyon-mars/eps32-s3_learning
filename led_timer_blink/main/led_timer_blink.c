#include <stdio.h>
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#define LED_GPIO GPIO_NUM_9

static const char* TAG = "TIMER_BLINK";

volatile bool led_on = false;

void periodic_timer_callback( void* arg ){
    led_on = !led_on;
    gpio_set_level( LED_GPIO, led_on );
    ESP_LOGI( TAG, "Toggled led: %s", led_on ? "ON" : "OFF" );
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = ( 1ULL << LED_GPIO ),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0, 
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config( &io_conf );
    
    esp_timer_create_args_t  timer_args = {
        .callback = &periodic_timer_callback, 
        .arg = NULL, 
        .dispatch_method = ESP_TIMER_TASK,
        .name = "led_timer"
    };
    esp_timer_handle_t periodic_timer;

    ESP_ERROR_CHECK( esp_timer_create( &timer_args, &periodic_timer ));

    ESP_ERROR_CHECK( esp_timer_start_periodic( periodic_timer, 5000000 ));


    ESP_LOGI( TAG, "Started periodic timer to blink led every 5000ms .");



}
