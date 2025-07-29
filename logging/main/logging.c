#include <stdio.h>
#include "esp_log.h"

void app_main( void ){
    esp_log_level_set( "LOG", ESP_LOG_INFO );
    ESP_LOGE( "LOG" , "This is an error." );
    ESP_LOGW( "LOG" , "This is a warning." );
    ESP_LOGI( "LOG" , "This is an Infor." );
    ESP_LOGD( "LOG" , "This is a debut Info." );
    ESP_LOGV( "LOG" , "This is a verbose Info." );

    int count = 1;
    
    ESP_LOGE( "TAG 2" , "This is an error. %d", count++);
    ESP_LOGW( "TAG 2" , "This is a warning. %d", count++ );
    ESP_LOGI( "TAG 2" , "This is an Infor. %d", count++ );
    ESP_LOGD( "TAG 2" , "This is a debut Info. %d", count++ );
    ESP_LOGV( "TAG 2" , "This is a verbose Info. %d", count++ );

}