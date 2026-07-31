#include "ota.h"
#include "key.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_bt.h"

static const char *TAG = "OTA";

#define WIFI_SSID      "1234"
#define WIFI_PASS      "123456789"
#define OTA_URL        "https://radar-communication-1452689283.cos.ap-guangzhou.myqcloud.com/Radar_Communication.bin"
#define LONG_PRESS_MS  3000

extern const char ca_cert_pem_start[] asm("_binary_ca_cert_pem_start");
extern const char ca_cert_pem_end[]   asm("_binary_ca_cert_pem_end");

static int s_retry_num = 0;
static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < 10)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        }
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           pdMS_TO_TICKS(10000));
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "connected to ap SSID:%s", WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to connect to SSID:%s", WIFI_SSID);
    }
}

static void ota_task(void *pvParameter)
{
    ESP_LOGI(TAG, "Stopping BLE before OTA...");
    esp_bluedroid_disable();
    esp_bt_controller_disable();
    ESP_LOGI(TAG, "BLE disabled, starting OTA download");

    esp_http_client_config_t config = {
        .url = OTA_URL,
        .cert_pem = ca_cert_pem_start,
        .timeout_ms = 5000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    ESP_LOGI(TAG, "Attempting to download update from %s", OTA_URL);
    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA update successful, restarting in 3 seconds...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart();
    }
    else
    {
        ESP_LOGE(TAG, "OTA update failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "disconnecting WiFi...");
    esp_wifi_disconnect();
    esp_wifi_stop();

    ESP_LOGI(TAG, "OTA failed, restarting to restore BLE...");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    vTaskDelete(NULL);
}

void ota_check_key_task(void *pvParameter)
{
    TickType_t press_start = 0;
    bool pressing = false;
    bool ota_triggered = false;

    while (1)
    {
        if (key_read(KEY3) == 0)
        {
            if (!pressing)
            {
                pressing = true;
                press_start = xTaskGetTickCount();
            }
            else
            {
                TickType_t elapsed = xTaskGetTickCount() - press_start;
                if (elapsed >= pdMS_TO_TICKS(LONG_PRESS_MS) && !ota_triggered)
                {
                    ota_triggered = true;
                    ESP_LOGI(TAG, "KEY3 long press detected (>3s), starting OTA...");

                    wifi_init_sta();
                    xTaskCreate(&ota_task, "ota_task", 8192, NULL, 5, NULL);
                }
            }
        }
        else
        {
            if (pressing)
            {
                TickType_t elapsed = xTaskGetTickCount() - press_start;
                if (elapsed < pdMS_TO_TICKS(LONG_PRESS_MS))
                {
                    ESP_LOGI(TAG, "KEY3 short press, ignored (press time: %d ms)",
                             (int)(elapsed * portTICK_PERIOD_MS));
                }
                pressing = false;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void ota_init(void)
{
    xTaskCreate(ota_check_key_task, "ota_check_key", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "OTA key monitor started (KEY3, GPIO41, long press >3s to trigger)");
}