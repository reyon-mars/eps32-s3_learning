#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"


#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO      (9)
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_13_BIT
#define LED_FREQUENCY       (1000)

static const char* TAG = "PWM_FADE";

void app_main(void)
{
    ESP_LOGI( TAG, "Configuring LEDC PWM....");
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config( &ledc_timer );

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = LEDC_OUTPUT_IO, 
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config( &ledc_channel );

    while( 1 ){
        ESP_LOGI( TAG, "Fading in.... " );
        for( int duty = 0; duty <= 8191; duty += 20 ){
            ledc_set_duty( LEDC_MODE, LEDC_CHANNEL, duty );
            ledc_update_duty( LEDC_MODE, LEDC_CHANNEL );
            vTaskDelay( pdMS_TO_TICKS( 10 ) );
        }

        ESP_LOGI( TAG, "Fading out...." );
        for( int duty = 8191; 0 <= duty; duty -= 20 ){
            ledc_set_duty( LEDC_MODE, LEDC_CHANNEL, duty );
            ledc_update_duty( LEDC_MODE, LEDC_CHANNEL );
            vTaskDelay( pdMS_TO_TICKS( 10 ));
        }
    }
}
