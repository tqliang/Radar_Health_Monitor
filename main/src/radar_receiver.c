#include "radar_receiver.h"
#include <string.h>
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *RADAR_TAG = "radar_rx";

/* ---------------- 环形缓冲区 (5 帧均值滤波) ---------------- */
typedef struct
{
    float breath_bpm[RADAR_FILTER_WINDOW];
    float heart_bpm[RADAR_FILTER_WINDOW];
    uint8_t head;       /* 写入位置 */
    uint8_t count;      /* 已填充帧数 (0..5) */
    float breath_mean;  /* 当前呼吸均值 */
    float heart_mean;   /* 当前心率均值 */
    bool available;     /* 是否有有效数据 */
} radar_filter_buf_t;

static radar_filter_buf_t g_filter;

/* ---------------- CRC-8 (MAXIM/DALLAS, 多项式 0x31) ---------------- */
static uint8_t radar_crc8(const uint8_t *data, uint32_t len)
{
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (uint8_t)((crc << 1) ^ 0x31);
            }
            else
            {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

/* ---------------- 更新均值滤波输出 ---------------- */
static void radar_filter_update(void)
{
    if (g_filter.count == 0)
    {
        g_filter.available = false;
        return;
    }

    float sum_breath = 0.0f;
    float sum_heart  = 0.0f;

    for (uint8_t i = 0; i < g_filter.count; i++)
    {
        sum_breath += g_filter.breath_bpm[i];
        sum_heart  += g_filter.heart_bpm[i];
    }

    g_filter.breath_mean = sum_breath / (float)g_filter.count;
    g_filter.heart_mean  = sum_heart  / (float)g_filter.count;
    g_filter.available   = true;
}

/* ---------------- 将一帧数据压入环形缓冲区 ---------------- */
static void radar_filter_push(float breath, float heart)
{
    g_filter.breath_bpm[g_filter.head] = breath;
    g_filter.heart_bpm[g_filter.head]  = heart;

    g_filter.head = (g_filter.head + 1) % RADAR_FILTER_WINDOW;

    if (g_filter.count < RADAR_FILTER_WINDOW)
    {
        g_filter.count++;
    }

    radar_filter_update();
}

/* ---------------- 解析一帧并压入滤波器 ---------------- */
static bool radar_parse_frame(const uint8_t *frame, uint32_t len)
{
    if (len != RADAR_FRAME_SIZE)
    {
        return false;
    }

    /* 校验帧头 */
    if (frame[0] != RADAR_FRAME_HEADER_0 || frame[1] != RADAR_FRAME_HEADER_1)
    {
        return false;
    }

    /* 校验标志字节 */
    if (frame[2] != RADAR_FRAME_FLAG)
    {
        return false;
    }

    /* CRC8 校验 (覆盖字节 0..14) */
    uint8_t expected_crc = radar_crc8(frame, RADAR_FRAME_SIZE - 1);
    if (expected_crc != frame[RADAR_FRAME_SIZE - 1])
    {
        ESP_LOGW(RADAR_TAG, "CRC mismatch: calc=0x%02X, recv=0x%02X",
                 expected_crc, frame[RADAR_FRAME_SIZE - 1]);
        return false;
    }

    /* 提取 frame_cnt (小端 uint32) */
    uint32_t frame_cnt;
    memcpy(&frame_cnt, &frame[3], 4);

    /* 提取 breath_bpm (小端 IEEE-754 float32) */
    float breath_bpm;
    memcpy(&breath_bpm, &frame[7], 4);

    /* 提取 heart_bpm (小端 IEEE-754 float32) */
    float heart_bpm;
    memcpy(&heart_bpm, &frame[11], 4);

    ESP_LOGI(RADAR_TAG, "Frame #%lu: breath=%.1f bpm, heart=%.1f bpm",
             (unsigned long)frame_cnt, (double)breath_bpm, (double)heart_bpm);

    /* 压入滤波器 */
    radar_filter_push(breath_bpm, heart_bpm);

    return true;
}

/* ---------------- UART 接收任务 ---------------- */
static void radar_uart_rx_task(void *param)
{
    (void)param;
    ESP_LOGI(RADAR_TAG, "Radar UART RX task started");

    uint8_t rx_byte;
    uint8_t frame_buf[RADAR_FRAME_SIZE];
    uint8_t frame_pos = 0;

    /* 状态机: 0 = 等待帧头 0xAA, 1 = 等待帧头 0x55, 2 = 接收帧体 */
    uint8_t state = 0;

    while (1)
    {
        int n = uart_read_bytes(UART_NUM_1, &rx_byte, 1, pdMS_TO_TICKS(100));
        if (n <= 0)
        {
            continue;
        }

        switch (state)
        {
        case 0: /* 等待 0xAA */
            if (rx_byte == RADAR_FRAME_HEADER_0)
            {
                frame_buf[0] = rx_byte;
                frame_pos = 1;
                state = 1;
            }
            break;

        case 1: /* 等待 0x55 */
            if (rx_byte == RADAR_FRAME_HEADER_1)
            {
                frame_buf[1] = rx_byte;
                frame_pos = 2;
                state = 2;
            }
            else if (rx_byte == RADAR_FRAME_HEADER_0)
            {
                /* 0xAA 后跟的不是 0x55, 但又是 0xAA, 重新开始 */
                frame_buf[0] = rx_byte;
                frame_pos = 1;
                /* state 保持 1, 继续等待 0x55 */
            }
            else
            {
                /* 不是合法帧头, 回到初始状态 */
                state = 0;
            }
            break;

        case 2: /* 接收剩余帧体 */
            frame_buf[frame_pos++] = rx_byte;
            if (frame_pos >= RADAR_FRAME_SIZE)
            {
                /* 完整帧接收完毕, 解析 */
                radar_parse_frame(frame_buf, RADAR_FRAME_SIZE);
                state = 0;
            }
            break;

        default:
            state = 0;
            break;
        }
    }
}

/* ---------------- 公共接口 ---------------- */

void radar_receiver_init(void)
{
    /* 初始化滤波器 */
    memset(&g_filter, 0, sizeof(g_filter));

    /* 启动 UART 接收任务 */
    xTaskCreate(radar_uart_rx_task, "radar_rx", 2 * 1024, NULL, 5, NULL);

    ESP_LOGI(RADAR_TAG, "Radar receiver initialized, filter window=%d", RADAR_FILTER_WINDOW);
}

bool radar_data_available(void)
{
    return g_filter.available;
}

float radar_get_breath_bpm(void)
{
    return g_filter.breath_mean;
}

float radar_get_heart_bpm(void)
{
    return g_filter.heart_mean;
}