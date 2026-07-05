
#include "uart2_radar.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "uart2_radar";

/* ---------------- CRC-8 (MAXIM/DALLAS, 多项式 0x31) ----------------
 * poly = x^8 + x^5 + x^4 + 1 = 0x31 (bits 7..0)
 * init = 0x00, no final XOR
 * 广泛用于 1-Wire / SMBus, 此处复用作短帧完整性校验
 * ------------------------------------------------------------------ */
static uint8_t crc8_maxim(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
                crc = (uint8_t)((crc << 1) ^ 0x31);
            else
                crc = (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* -----------------------------------------------------------------------
 * 初始化 UART2
 * ----------------------------------------------------------------------- */
void uart2_radar_init(void)
{
    uart_config_t uart_cfg = {
        .baud_rate = UART2_RADAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    /* 安装驱动: 1KB 内部缓冲足够, 我们只用 TX */
    esp_err_t err = uart_driver_install(UART2_RADAR_NUM,
                                        UART2_RADAR_BUF_SIZE,
                                        0,          /* 不使用 tx 缓冲 */
                                        10,         /* event queue 深度 (未使用) */
                                        NULL,       /* 不创建 event queue */
                                        0);         /* intr_alloc_flags */
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_driver_install() failed: %s", esp_err_to_name(err));
        return;
    }

    err = uart_param_config(UART2_RADAR_NUM, &uart_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_param_config() failed: %s", esp_err_to_name(err));
        return;
    }

    /* 绑定 GPIO: TX=39, RX=40, RTS/CTS 不使用 (-1) */
    err = uart_set_pin(UART2_RADAR_NUM,
                       UART2_RADAR_TXD,
                       UART2_RADAR_RXD,
                       UART2_RADAR_RTS,
                       UART2_RADAR_CTS);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "uart_set_pin(TX=%d,RX=%d) failed: %s",
                 UART2_RADAR_TXD, UART2_RADAR_RXD, esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "UART2 inited: baud=%d, TX=GPIO%d, RX=GPIO%d",
             UART2_RADAR_BAUD, UART2_RADAR_TXD, UART2_RADAR_RXD);
}

/* -----------------------------------------------------------------------
 * 把 "呼吸 bpm + 心率 bpm" 打成二进制帧, 通过 UART2 阻塞发送
 *   帧格式详见 uart2_radar.h
 * ----------------------------------------------------------------------- */
void uart2_radar_send_frame(float breath_bpm, float heart_bpm, uint32_t frame_cnt)
{
    uint8_t frame[UART2_FRAME_SIZE];

    /* --- 1. 帧头 --- */
    frame[0] = UART2_FRAME_HEADER_0;
    frame[1] = UART2_FRAME_HEADER_1;

    /* --- 2. 标志字节 --- */
    frame[2] = UART2_FRAME_FLAG;

    /* --- 3. frame_cnt (小端, uint32) --- */
    memcpy(&frame[3], &frame_cnt, 4);

    /* --- 4. breath_bpm (小端, IEEE-754 float32) --- */
    memcpy(&frame[7], &breath_bpm, 4);

    /* --- 5. heart_bpm (小端, IEEE-754 float32) --- */
    memcpy(&frame[11], &heart_bpm, 4);

    /* --- 6. CRC-8 (覆盖字节 0..14) --- */
    frame[15] = crc8_maxim(frame, 15);

    /* 阻塞发送, 对 16 字节可在 ~1.4ms 内完成 */
    int n = uart_write_bytes(UART2_RADAR_NUM, frame, UART2_FRAME_SIZE);
    if (n != UART2_FRAME_SIZE)
    {
        /* 理论上不应发生, 若发生说明 driver 内部队列已满 */
        ESP_LOGW(TAG, "uart_write_bytes expected %d bytes, got %d",
                 UART2_FRAME_SIZE, n);
    }
}