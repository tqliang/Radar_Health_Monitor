#include "led.h"

/* Private variables */
static uint8_t led_state;

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP
static led_strip_handle_t led_strip;
#endif

/* Public functions */
uint8_t get_led_state(void)
{
    return led_state;
}

#ifdef CONFIG_EXAMPLE_BLINK_LED_STRIP//地址可寻址 LED，使用 LED 驱动库控制

void led_on(void)
{

    led_strip_set_pixel(led_strip, 0, 16, 16, 16);
    led_strip_refresh(led_strip);
    led_state = true;
}

void led_off(void)
{
    led_strip_clear(led_strip);
    led_state = false;
}

void led_init(void)
{
    led_strip_config_t strip_config = //LED strip 配置结构体，用于配置 LED strip 的参数
    {
        .strip_gpio_num = 48,
        .max_leds = 1, // at least one LED on board
    };
#if CONFIG_EXAMPLE_BLINK_LED_STRIP_BACKEND_RMT
    led_strip_rmt_config_t rmt_config = //RMT 配置结构体，用于配置 RMT 设备的参数
    {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,//不使用 DMA，因为只有一个 LED，数据量很小，使用 DMA 反而可能增加开销
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));//创建一个 RMT 设备的 LED strip 实例  
#elif CONFIG_EXAMPLE_BLINK_LED_STRIP_BACKEND_SPI
    led_strip_spi_config_t spi_config = {
        .spi_bus = SPI2_HOST,
        .flags.with_dma = true,
    };
    ESP_ERROR_CHECK(
        led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
#else
#error "unsupported LED strip backend"
#endif
    /* Set all LED off to clear all pixels */
    led_off();
}

#elif CONFIG_EXAMPLE_BLINK_LED_GPIO//普通 GPIO 控制的 LED，直接设置 GPIO 电平即可

void led_on(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, false);
}

void led_off(void)
{
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, true);
}

void led_init(void)
{
    gpio_reset_pin(CONFIG_EXAMPLE_BLINK_GPIO);
    gpio_set_direction(CONFIG_EXAMPLE_BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(CONFIG_EXAMPLE_BLINK_GPIO, 1);
}

#else
#error "unsupported LED type"
#endif
