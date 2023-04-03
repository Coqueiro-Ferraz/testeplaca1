#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "freertos/event_groups.h"

#define TAG "WIFI"

esp_netif_t * wifi_netif;

static EventGroupHandle_t wifi_events;
static const int CONNECTED_GOT_IP = BIT0;
static const int DISCONNECTED = BIT1;


void wifi_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    switch(event_id)
    {
        case SYSTEM_EVENT_STA_START:
            ESP_LOGI(TAG, "Conectando...");
            esp_wifi_connect();
            break;
        case SYSTEM_EVENT_STA_CONNECTED:
            ESP_LOGI(TAG, "Conectado com sucesso");
            break;
        case SYSTEM_EVENT_STA_DISCONNECTED:
            ESP_LOGI(TAG, "Desconectado");
            xEventGroupSetBits(wifi_events, DISCONNECTED);
            break;
        case IP_EVENT_STA_GOT_IP:
            ESP_LOGI(TAG, "IP Obitido");
            xEventGroupSetBits(wifi_events, CONNECTED_GOT_IP);
            break;
        default:
            break;
    }
}

void wifi_init(void)
{
    wifi_init_config_t wifiCfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_wifi_init(&wifiCfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler,NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler,NULL));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_events = xEventGroupCreate();

}

esp_err_t wifi_connect_sta(const char * ssid, const char * pwd, int timeout)
{
    wifi_netif = esp_netif_create_default_wifi_sta();

    wifi_config_t wifiCfg;
    memset(&wifiCfg,0,sizeof(wifi_config_t));
    strncpy((char *)wifiCfg.sta.ssid, ssid, sizeof(wifiCfg.sta.ssid)-1);
    strncpy((char *)wifiCfg.sta.password, pwd, sizeof(wifiCfg.sta.password)-1);

    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifiCfg);
    esp_wifi_start();

    EventBits_t event_result = xEventGroupWaitBits(wifi_events, CONNECTED_GOT_IP || DISCONNECTED, pdTRUE, pdFALSE, pdMS_TO_TICKS(timeout));
    if(event_result)
    {
        return ESP_OK;
    }
    else
    {
        return ESP_FAIL;
    }
}
void wifi_disconnect(void)
{
    esp_wifi_disconnect();
    esp_wifi_stop();
}