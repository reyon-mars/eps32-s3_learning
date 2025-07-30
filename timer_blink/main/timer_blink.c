#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "driver/gpio.h"

#define LED_GPIO GPIO_NUM_4
#define TAG      "TIMER_BLINK"

TimerHandle_t blink_timer;

void blink_timer_callback( TimerHandle_t xTimer ){
    static bool led_state = false;
    led_state = !led_state;
    gpio_set_level( LED_GPIO, led_state );
    ESP_LOGI( TAG , "LED %s.", led_state ? "ON" : "OFF" );
}



void app_main(void)
{
    gpio_reset_pin( LED_GPIO );
    gpio_set_direction( LED_GPIO, GPIO_MODE_OUTPUT );

    blink_timer = xTimerCreate(
        "BlinkTimer",
        pdMS_TO_TICKS( 500 ),
        pdTRUE,
        (void* ) 0,
        blink_timer_callback
    );

    if( blink_timer == NULL ){
        ESP_LOGE( TAG, "Failed to create the timer. " );
    } else {
        xTimerStart( blink_timer, 0 );
    }
}
