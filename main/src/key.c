#include "key.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* GPIO 引脚映射表 */
static const uint8_t key_gpio[KEY_NUM] = {
    [KEY1] = 15,
    [KEY2] = 8,
    [KEY3] = 41,
    [KEY4] = 4,
};

void key_init(void)
{
    for (int i = 0; i < KEY_NUM; i++) {
        gpio_reset_pin(key_gpio[i]);
        gpio_set_direction(key_gpio[i], GPIO_MODE_INPUT);
        gpio_set_pull_mode(key_gpio[i], GPIO_PULLUP_ONLY);
    }
}

uint8_t key_read(uint8_t key_num)
{
    if (key_num >= KEY_NUM) {
        return 1;
    }
    return (uint8_t)gpio_get_level(key_gpio[key_num]);
}

uint8_t key_is_pressed(uint8_t key_num)
{
    if (key_num >= KEY_NUM) {
        return 0;
    }
    /* 按键上拉，按下为低电平，做简单消抖 */
    if (gpio_get_level(key_gpio[key_num]) == 0) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        if (gpio_get_level(key_gpio[key_num]) == 0) {
            return 1;
        }
    }
    return 0;
}