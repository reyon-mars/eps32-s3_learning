#include <stdio.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"


#define LED_GPIO GPIO_NUM_9
#define BUTTON_GPIO GPIO_NUM_7

static const char* TAG = "INTERRUPT_LED";
static volatile bool led_state = false;
static volatile int64_t last_interrupt_time = 0;

#define DEBOUNCE_TIME_US 200000

static void IRAM_ATTR button_isr_handler( void* arg ){
    int64_t now = esp_timer_get_time();
    if( now - last_interrupt_time < DEBOUNCE_TIME_US ) return ;

    last_interrupt_time = now;
    led_state = !led_state;
    gpio_set_level( LED_GPIO, led_state );
}

void app_main(void)
{
    gpio_config_t led_config = {
        .pin_bit_mask = ( 1ULL << LED_GPIO ),
        .mode = GPIO_MODE_OUTPUT, 
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config( &led_config );

    gpio_config_t button_config = {
        .pin_bit_mask = ( 1ULL << BUTTON_GPIO ), 
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE, 
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config( &button_config );

    gpio_install_isr_service( 0 );
    gpio_isr_handler_add( BUTTON_GPIO, button_isr_handler, NULL );

    ESP_LOGI( TAG, "Interrupt based LED toggle setup complete" );

    while( 1 ){
        vTaskDelay( pdMS_TO_TICKS( 1000 ));
    }
}
