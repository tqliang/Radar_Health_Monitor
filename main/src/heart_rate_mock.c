#include "heart_rate.h"
#include "radar_receiver.h"
#include "esp_log.h"

static const char *HR_TAG = "heart_rate";

/* Private variables */
static uint8_t heart_rate;

/* Public functions */
uint8_t get_heart_rate(void)
{
    return heart_rate;
}

void update_heart_rate(void)
{
    if (radar_data_available())
    {
        float raw = radar_get_heart_bpm();// 从雷达接收器获取心率数据
        if (raw > 0.0f)
        {
            heart_rate = (uint8_t)(raw + 0.5f);
            ESP_LOGI(HR_TAG, "Radar heart rate: %.1f -> %u bpm", (double)raw, heart_rate);
        }
    }
}