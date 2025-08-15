#include "esp_wifi.h"
#include "esp_task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static void init_nvs(){
    esp_err_t ret = nvs_flash_init();
    if( ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND ){
        ESP_ERROR_CHECK( nvs_flash_erase() );
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
}

static void wifi_scan(){
    wifi_init_config_t conf = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init( &conf ) );

    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    ESP_LOGI( "WIFI_SCAN", "Starting scan....." );
    wifi_scan_config_t scan_conf = {};
    scan_conf.ssid = NULL;
    scan_conf.bssid = NULL;
    scan_conf.channel = 0;
    scan_conf.show_hidden = true;
    scan_conf.scan_type = WIFI_SCAN_TYPE_ACTIVE;

    ESP_ERROR_CHECK( esp_wifi_scan_start( &scan_conf, true ));
    uint16_t ap_count = 0;
    ESP_ERROR_CHECK( esp_wifi_scan_get_ap_num( &ap_count ) );
    ESP_LOGI( "WIFI_SCAN", "Found %d access points.", ap_count );

    wifi_ap_record_t *ap_records = ( wifi_ap_record_t* )( malloc( sizeof(wifi_ap_record_t) * ap_count ));
    ESP_ERROR_CHECK( esp_wifi_scan_get_ap_records( &ap_count, ap_records ));
    
    for( int i = 0; i < ap_count; i++ ){
        ESP_LOGI( "WIFI_SCAN", "[%2d] SSID: %s | RSSI: %d dBm | Auth: %d | Hidden: %s ", 
        i+1,
        (char*) ap_records[i].ssid, 
        ap_records[i].rssi,
        ap_records[i].authmode,
        ap_records[i].ssid[0] ? "NO" : "YES" );
    }
    free(ap_records);
}



void app_main(void)
{
    init_nvs();
    ESP_ERROR_CHECK( esp_netif_init() );
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    esp_netif_create_default_wifi_sta( );

    wifi_scan();

}
