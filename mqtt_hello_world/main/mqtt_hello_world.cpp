#include <cstring>
#include <string>
#include <string_view>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"


static constexpr const char* TAG = "MQTT_DEMO";


#ifndef CONFIG_WIFI_SSID
#error "Set WiFi SSID via menuconfig: App Configuration -> WIFI_SSID"
#endif
#ifndef CONFIG_WIFI_PASSWORD
#error "Set WiFi Password via menuconfig: App Configuration -> WIFI_PASSWORD"
#endif
#ifndef CONFIG_MQTT_BROKER_URI
#error "Set MQTT BROKER URI via menuconfig: App Configuration -> MQTT_BROKER_URI"
#endif
#ifndef CONFIG_MQTT_TOPIC
#error "Set MQTT TOPIC via menuconfig: App Configuration -> MQTT_TOPIC"
#endif
#ifndef CONFIG_MQTT_PUBLISH_PERIOD_MS
#define CONFIG_MQTT_PUBLISH_PERIOD_MS 5000
#endif
#ifndef CONFIG_MQTT_CLIENT_ID
#define CONFIG_MQTT_CLIENT_ID "esp32s3-hello"
#endif

static EventGroupHandle_t s_wifi_event_group;
static constexpr int WIFI_CONNECTED_BIT = BIT0;

static void wifi_init_sta();
static void mqtt_app_start();
static void publisher_task(void*);


static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi STA start -> connecting...");
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected -> reconnecting...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        auto* event = static_cast<ip_event_got_ip_t*>(event_data);
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static esp_mqtt_client_handle_t s_mqtt = nullptr;

static void log_mqtt_data(const esp_mqtt_event_t* event)
{
    std::string topic(event->topic, event->topic_len);
    std::string data(event->data, event->data_len);
    ESP_LOGI(TAG, "MQTT DATA: topic='%s' payload='%s'", topic.c_str(), data.c_str());
}

static void mqtt_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data)
{
    auto* event = static_cast<esp_mqtt_event_t*>(event_data);
    switch (static_cast<esp_mqtt_event_id_t>(event_id)) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            esp_mqtt_client_subscribe(event->client, CONFIG_MQTT_TOPIC, 0);
            if (s_mqtt == nullptr) s_mqtt = event->client;
            xTaskCreatePinnedToCore(publisher_task, "publisher_task", 4096, event->client, 5, nullptr, tskNO_AFFINITY);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Subscribed (msg_id=%d)", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "Unsubscribed (msg_id=%d)", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGD(TAG, "Published (msg_id=%d)", event->msg_id);
            break;

        case MQTT_EVENT_DATA:
            log_mqtt_data(event);
            break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT event error");
            break;

        default:
            ESP_LOGD(TAG, "Other MQTT event id: %d", event_id);
            break;
    }
}

static void publisher_task(void* pv)
{
    auto client = static_cast<esp_mqtt_client_handle_t>(pv);
    const char* topic = CONFIG_MQTT_TOPIC;
    int counter = 0;

    for (;;) {
        std::string payload = "Hello, World! #" + std::to_string(counter++);
        int msg_id = esp_mqtt_client_publish(client, topic, payload.c_str(), 0 /*len*/, 0 /*qos*/, 0 /*retain*/);
        ESP_LOGI(TAG, "Published: '%s' (msg_id=%d) to %s", payload.c_str(), msg_id, topic);
        vTaskDelay(pdMS_TO_TICKS(CONFIG_MQTT_PUBLISH_PERIOD_MS));
    }
}

static void mqtt_app_start()
{
    esp_mqtt_client_config_t cfg = {};
    cfg.broker.address.uri = CONFIG_MQTT_BROKER_URI;
    cfg.credentials.client_id = CONFIG_MQTT_CLIENT_ID;

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, client));
    ESP_ERROR_CHECK(esp_mqtt_client_start(client));
}

static void wifi_init_sta()
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, nullptr, nullptr));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, nullptr, nullptr));

    wifi_config_t wifi_cfg{};
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.sta.ssid), CONFIG_WIFI_SSID, sizeof(wifi_cfg.sta.ssid)-1);
    std::strncpy(reinterpret_cast<char*>(wifi_cfg.sta.password), CONFIG_WIFI_PASSWORD, sizeof(wifi_cfg.sta.password)-1);
    wifi_cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_cfg.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Waiting for WiFi connection...");
    xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi connected");
}

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    wifi_init_sta();
    mqtt_app_start();
}
