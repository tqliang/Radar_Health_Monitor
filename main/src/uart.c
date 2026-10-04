#include "uart.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *UART_TAG = "UART1";

void uart1_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, UART1_TX_GPIO, UART1_RX_GPIO,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(UART_TAG, "UART1 initialized, TX: GPIO%d, RX: GPIO%d, baud: 115200",
             UART1_TX_GPIO, UART1_RX_GPIO);
}