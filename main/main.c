#include <stdio.h>
#include <string.h>
#include "driver/spi_master.h"      /* SPI 主机驱动 */
#include "driver/gpio.h"            /* GPIO 驱动，含中断配置 */
#include "driver/uart.h"            /* UART 驱动，用于提高串口波特率 */
#include "esp_log.h"                /* ESP 日志库（ESP_LOGI / ESP_LOGE 等） */
#include "freertos/FreeRTOS.h"      /* FreeRTOS 基础定义 */
#include "freertos/task.h"          /* FreeRTOS 任务 API */
#include "esp_system.h"             /* ESP 系统相关 */
#include "freertos/semphr.h"        /* FreeRTOS 信号量，用于中断与任务间同步 */

/* ---------- 雷达驱动相关头文件 ---------- */
#include "bgt60tr13c_driver.h"      /* 雷达驱动函数声明 */
#include "bgt60tr13c_regs.h"        /* 寄存器地址、位掩码、SPI 协议相关宏 */

/* --- 在 MCU 端完成呼吸/心率检测, 不向上位机发送原始雷达数据 --- */
#include "vital_signs.h"
/* --- UART2 独立通道: GPIO39=TXD2, GPIO40=RXD2, 115200 打包传输呼吸/心率 --- */
#include "uart2_radar.h"

static const char * TAG = "spi-test-runner";

#define SPI_HOST        SPI2_HOST 
#define SPI_CLK_SPEED   10 

#define SPI_CS_PIN              GPIO_NUM_17   /* SPI 片选（Chip Select） —— 低电平有效 */
#define SPI_SCK_PIN             GPIO_NUM_12   /* SPI 时钟（Serial Clock） */
#define SPI_MOSI_PIN            GPIO_NUM_14   /* SPI 主机输出/从机输入（Master Out Slave In） */
#define SPI_MISO_PIN            GPIO_NUM_15   /* SPI 主机输入/从机输出（Master In Slave Out） */
#define RADAR_IRQ_PIN           GPIO_NUM_13   /* 雷达中断输出：FIFO 达到 CREF 阈值时触发上升沿 */
#define RADAR_RESET_PIN         GPIO_NUM_16   /* 雷达硬件复位引脚（低电平复位） */
#define RADAR_3V3TO1V8_POWER_PIN GPIO_NUM_1   /* 3.3V → 1.8V 电源使能（雷达内部参考电压） */

#define PRINT_CHUNK_BEFORE_YIELD 32


SemaphoreHandle_t xSemaphore = NULL;


void IRAM_ATTR gpio_radar_isr_handler(void *arg)
{
    xSemaphoreGiveFromISR(xSemaphore, NULL);
}


void xensiv_bgt60tr13c_radar_task(void *pvParameters)
{
    ESP_LOGI(TAG, "starting radar task for continuous frame capture");

    uint32_t frame_size_samples = 0;
    esp_err_t err_check;

    err_check = get_frame_size(&frame_size_samples);
    if (err_check != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get frame_size_samples: %s", esp_err_to_name(err_check));
        vTaskDelete(NULL); return;
    }

    if (frame_size_samples == 0)
    {
        ESP_LOGE(TAG, "Frame size (samples) is 0. Check radar configuration.");
        vTaskDelete(NULL); return;
    }

    uint32_t temp_buf_len_bytes = 255;


    uint16_t *frame_buf = (uint16_t *)malloc(frame_size_samples * sizeof(uint16_t));
    uint8_t  *temp_buf  = (uint8_t *)malloc(temp_buf_len_bytes * sizeof(uint8_t));

    /* 内存分配失败 —— 直接删除任务（FreeRTOS malloc 失败返回 NULL） */
    if (frame_buf == NULL || temp_buf == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate memory for buffers");
        if (xSemaphore != NULL)
        {
            vSemaphoreDelete(xSemaphore);
            xSemaphore = NULL;
        }
        vTaskDelete(NULL);
        return;
    }
    memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t));

    /* 打印关键尺寸信息，方便在串口监视器中核对 */
    ESP_LOGI(TAG, "frame_size (number of 16-bit samples for frame_buf): %lu", frame_size_samples);
    ESP_LOGI(TAG, "frame_buf size (bytes): %lu", frame_size_samples * sizeof(uint16_t));
    ESP_LOGI(TAG, "temp_buf_len_bytes (bytes to read per FIFO transaction): %lu", temp_buf_len_bytes);

    /* ---- 初始化 MCU 端呼吸/心率检测器 ---- */
    vital_signs_init();
    ESP_LOGI(TAG, "vital-signs detector inited (fs=%.2f Hz, bins=%d, window(breath)=%d, window(heart)=%d).",
             (double)VS_FS_HZ, VS_NUM_SAMPLES_PER_CHIRP,
             VS_BREATH_WINDOW_FRAMES, VS_HEART_WINDOW_FRAMES);

    err_check = xensiv_bgt60tr13c_start_frame_capture();
    if (err_check != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start frame capture: %s", esp_err_to_name(err_check));
        free(frame_buf);
        free(temp_buf);
        if (xSemaphore != NULL) { vSemaphoreDelete(xSemaphore); xSemaphore = NULL; }
        vTaskDelete(NULL); return;
    }

    uint32_t current_idx = 0;                 /* 已写入 frame_buf 的 sample 数 */
    uint32_t total_frames_collected_count = 0;/* 已完整采集到的帧总数（调试用） */

    for (;;)
    {
        if (xSemaphoreTake(xSemaphore, portMAX_DELAY) == pdTRUE)
        {

            // 新增FIFO硬件溢出检测
            uint32_t fstat = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_FSTAT_TR13C);
            // 溢出 + 下溢标志合并判断
            uint32_t fifo_err_mask = XENSIV_BGT60TR13C_REG_FSTAT_FOF_ERR_MSK | XENSIV_BGT60TR13C_REG_FSTAT_FUF_ERR_MSK;
            if (fstat & fifo_err_mask)
            {
                if (fstat & XENSIV_BGT60TR13C_REG_FSTAT_FOF_ERR_MSK)
                {
                    ESP_LOGE(TAG, "HW FIFO OVERFLOW ERROR! Data lost");
                }
                if (fstat & XENSIV_BGT60TR13C_REG_FSTAT_FUF_ERR_MSK)
                {
                    ESP_LOGE(TAG, "HW FIFO UNDERFLOW ERROR! Read empty FIFO");
                }

                // 执行FIFO/状态机软件复位
                uint32_t main_reg = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
                main_reg |= XENSIV_BGT60TR13C_REG_MAIN_RESET_MSK;
                xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, main_reg, true);
                vTaskDelay(pdMS_TO_TICKS(10));
                // 必须清除复位位，否则芯片持续复位无法采集
                main_reg &= ~XENSIV_BGT60TR13C_REG_MAIN_RESET_MSK;
                xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, main_reg, true);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            memset(temp_buf, 0, temp_buf_len_bytes);
            err_check = xensiv_bgt60tr13c_fifo_read(temp_buf, temp_buf_len_bytes, 0);

            if (err_check != ESP_OK)
            {
                ESP_LOGE(TAG, "FIFO read error: %s. Resetting frame progress. Total frames before error: %lu",
                         esp_err_to_name(err_check), total_frames_collected_count);
                current_idx = 0;
                memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t));
                vTaskDelay(pdMS_TO_TICKS(100)); /* 给雷达一点恢复时间 */
                ESP_LOGI(TAG, "Attempting to re-start frame capture after FIFO error.");
                xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO);
                vTaskDelay(pdMS_TO_TICKS(10));
                xensiv_bgt60tr13c_start_frame_capture();
                continue;  /* 跳过本次迭代，重新等待下一次中断 */
            }


            for (uint32_t i = 0; (i + 2) < temp_buf_len_bytes; i += 3)
            {
                /* 提取第 1 个 sample（占 12 bit） */
                if (current_idx < frame_size_samples)
                {
                    frame_buf[current_idx] = (temp_buf[i] << 4) | (temp_buf[i + 1] >> 4);
                    current_idx++;
                }
                else
                {
                    break; /* frame_buf 已满，跳出解包循环 */
                }

                /* 提取第 2 个 sample（占 12 bit，从 B1 低 4 位 + B2 组成） */
                if (current_idx < frame_size_samples)
                {
                    frame_buf[current_idx] = ((temp_buf[i + 1] & 0x0F) << 8) | temp_buf[i + 2];
                    current_idx++;
                }
                else
                {
                    break;
                }
            }

            if (current_idx >= frame_size_samples)
            {
                total_frames_collected_count++;

                float breath_bpm = 0.0f, heart_bpm = 0.0f;
                vital_signs_process_frame(frame_buf, frame_size_samples,
                                          &breath_bpm, &heart_bpm);

               
                int target_bin = vital_signs_last_bin();
                printf("[VS] frame=%lu  bin=%d  breath=%.1f bpm  heart=%.1f bpm\n",
                       total_frames_collected_count, target_bin,
                       (double)breath_bpm, (double)heart_bpm);
                fflush(stdout);

                uart2_radar_send_frame(breath_bpm, heart_bpm,
                                       total_frames_collected_count);

                
                current_idx = 0;
                memset(frame_buf, 0, frame_size_samples * sizeof(uint16_t));

                err_check = xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_FIFO);
                if (err_check != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to soft reset FIFO post-process: %s", esp_err_to_name(err_check));
                }
                vTaskDelay(pdMS_TO_TICKS(20));

                err_check = xensiv_bgt60tr13c_start_frame_capture();
                vTaskDelay(pdMS_TO_TICKS(15));
                if (err_check != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to re-start frame capture post-process: %s", esp_err_to_name(err_check));
                }
            } /* end: 收满一帧的处理 */

        } /* end: 信号量获取成功 */

    } /* end: for(;;) 主循环 */


    /* ── 理论上永远执行不到的清理代码（任务以删除自身结束） ── */
    free(frame_buf);
    free(temp_buf);
    if (xSemaphore != NULL)
    {
        vSemaphoreDelete(xSemaphore);
        xSemaphore = NULL;
    }
    vTaskDelete(NULL);
}



void app_main(void)
{
    uart_set_baudrate(UART_NUM_0, 921600);

    uart2_radar_init();

    esp_err_t ret;

    gpio_config_t power_conf =
    {
        .pin_bit_mask = (1ULL << RADAR_3V3TO1V8_POWER_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };
    gpio_config(&power_conf);
    gpio_set_level(RADAR_3V3TO1V8_POWER_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));                 /* LDO 输出电压稳定需要时间 */

    int power_level = gpio_get_level(RADAR_3V3TO1V8_POWER_PIN);
    ESP_LOGI(TAG, "Radar 3V3->1V8 power enabled (GPIO %d), actual level: %d",
             RADAR_3V3TO1V8_POWER_PIN, power_level);

    gpio_config_t reset_conf =
    {
        .pin_bit_mask = (1ULL << RADAR_RESET_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
    };

    gpio_config(&reset_conf);
    gpio_set_level(RADAR_RESET_PIN, 0);             /* 拉低 → 复位 */
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(RADAR_RESET_PIN, 1);             /* 拉高 → 释放复位 */
    vTaskDelay(pdMS_TO_TICKS(10));
    ESP_LOGI(TAG, "Radar RESET pin released (GPIO %d)", RADAR_RESET_PIN);

    spi_bus_config_t bus_config =
    {
        .miso_io_num     = SPI_MISO_PIN,
        .mosi_io_num     = SPI_MOSI_PIN,
        .sclk_io_num     = SPI_SCK_PIN,
        .quadwp_io_num   = -1,        /* -1 表示不使用（只做标准 SPI，非 Quad SPI） */
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096
    };

    ret = spi_bus_initialize(SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return;
    }
    ESP_LOGI(TAG, "Successfully initialized SPI bus!");

    spi_device_interface_config_t dev_config =
    {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits   = 0,
        .clock_speed_hz = SPI_CLK_SPEED * 1000 * 1000,   /* 转换为 Hz：10 MHz */
        .mode         = 0,
        .spics_io_num = SPI_CS_PIN,
        .queue_size   = 7,
    };

    ret = xensiv_bgt60tr13c_init(SPI_HOST, &dev_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xensiv_bgt60tr13c_init failed: %s.", esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(TAG, "xensiv_bgt60tr13c_init successful.");
    }

    ESP_LOGI(TAG, "Configuring radar with settings from bgt60tr13c_config.h");
    ret = xensiv_bgt60tr13c_configure();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "xensiv_bgt60tr13c_configure failed: %s", esp_err_to_name(ret));
        return;
    }
    else
    {
        ESP_LOGI(TAG, "Radar configured with settings from bgt60tr13c_config.h");
    }


    ESP_LOGI(TAG, "Setting CCR2 for endless frames.");

    uint32_t ccr2_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    ESP_LOGI(TAG, "Original CCR2 value: 0x%08lX", ccr2_val);

    /* 提取 frame_length 部分（bit12..23），低 12 位(MAX_FRAME_CNT)清零 */
    uint32_t ccr2_frame_len_part = (ccr2_val & 0x00FFF000);
    uint32_t ccr2_to_write       = ccr2_frame_len_part;

    ESP_LOGI(TAG, "CCR2 value to write for endless frames (MAX_FRAME_CNT=0): 0x%08lX", ccr2_to_write);
    ret = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_CCR2, ccr2_to_write, true);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set CCR2 for endless frames: %s.", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI(TAG, "Successfully set CCR2 for endless frames.");
    }

    uint32_t new_ccr2_val = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CCR2);
    ESP_LOGI(TAG, "New CCR2 value read back: 0x%08lX", new_ccr2_val);

    if ((new_ccr2_val & 0x00000FFF) != 0)
    {
        ESP_LOGW(TAG, "MAX_FRAME_CNT in CCR2 (0x%08lX) is not 0 after setting! "
                      "Value: 0x%lX. Continuous mode might not be enabled.",
                 new_ccr2_val, (new_ccr2_val & 0xFFF));
    }
    else
    {
        ESP_LOGI(TAG, "MAX_FRAME_CNT in CCR2 is 0. Configured for endless frames.");
    }

    gpio_config_t io_conf =
    {
        .pin_bit_mask = (1ULL << RADAR_IRQ_PIN),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);

    xSemaphore = xSemaphoreCreateBinary();
    if (xSemaphore == NULL)
    {
        ESP_LOGE(TAG, "Failed to create semaphore");
        return;
    }

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE)
    {
        ESP_LOGE(TAG, "Failed to install ISR service: %s", esp_err_to_name(ret));
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return;
    }

    /* 把 RADAR_IRQ_PIN 和 gpio_radar_isr_handler 绑定 */
    ret = gpio_isr_handler_add(RADAR_IRQ_PIN, gpio_radar_isr_handler, NULL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
        if (ret != ESP_ERR_INVALID_STATE) gpio_uninstall_isr_service();
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL; return;
    }

    ESP_LOGI(TAG, "Creating radar task");
    BaseType_t task_created = xTaskCreate(xensiv_bgt60tr13c_radar_task,
                                          "radar-task", 8192 * 2, NULL, 5, NULL);
    if (task_created != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create radar task");
        gpio_isr_handler_remove(RADAR_IRQ_PIN);
        vSemaphoreDelete(xSemaphore); xSemaphore = NULL;
    }
    else
    {
        ESP_LOGI(TAG, "Radar task created");
    }

    /* app_main 结束后不会退出，系统会继续调度 radar_task 和其他系统任务 */
    ESP_LOGI(TAG, "app_main finished setup.");
}