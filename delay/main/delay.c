#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include <inttypes.h>

#define TAG "DELAY"
#define DICE "DICE"

int dice_roll(){
    int random = esp_random();
    random = ( random < 0 ) ? random * -1 : random;
    random %= 6;
    return random + 1;
}

void app_main(void)
{
    int i = 0;

    while( 1 ){
        vTaskDelay( 1000 / portTICK_PERIOD_MS );
        ESP_LOGI( TAG, "The current iteration is %d", i++ );
        ESP_LOGI( DICE, "The current dice face is %d", dice_roll() );
    }
}
