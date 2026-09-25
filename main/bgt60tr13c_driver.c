#include <string.h>
#include <stdlib.h>
#include "bgt60tr13c_driver.h"
#include "bgt60tr13c_regs.h"
#include "bgt60tr13c_config.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "bgt60tr13c-driver";//日志标签
spi_device_handle_t spi;
bool radar_configured = false;

// 一帧数据的样本数
static uint32_t frame_size = XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP * 
                                    XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME * 
                                    XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS;

// 字节序转换函数，将大端转换为小端
uint32_t xensiv_bgt60tr13c_platform_word_reverse(uint32_t word) 
{
    return ((word & 0x000000FF) << 24) |
           ((word & 0x0000FF00) << 8) |
           ((word & 0x00FF0000) >> 8) |
           ((word & 0xFF000000) >> 24);
}

esp_err_t xensiv_bgt60tr13c_init(spi_host_device_t spi_host, spi_device_interface_config_t *dev_config) 
{
    /* 确保 dev_config 已配置 */
    assert(dev_config != NULL);
    
    /* 将设备挂载到 SPI 总线上 */
    ESP_ERROR_CHECK(spi_bus_add_device(spi_host, dev_config, &spi));

    /* 读芯片ID，验证雷达是否正确连接 */
    uint32_t chip_id = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_CHIP_ID);//读芯片ID

    uint32_t chip_id_digital = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_MSK) >> XENSIV_BGT60TR13C_REG_CHIP_ID_DIGITAL_ID_POS;//提取数字芯片ID
    uint32_t chip_id_rf = (chip_id & XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_MSK) >> XENSIV_BGT60TR13C_REG_CHIP_ID_RF_ID_POS;//提取RF芯片ID

    if ((chip_id_digital == 3U) && (chip_id_rf == 3U)) 
    {
        ESP_LOGI(TAG, "BGT60TR13C Verified. Digital Chip ID: %lu RF Chip ID: %lu", chip_id_digital, chip_id_rf);
    } 
    else if ((chip_id_digital == 6U) && (chip_id_rf == 6U)) 
    {
        ESP_LOGI(TAG, "BGT60UTR13D Verified. Digital Chip ID: %lu RF Chip ID: %lu", chip_id_digital, chip_id_rf);
    } 
    else 
    {
        ESP_LOGE(TAG, "Chip Verification failed. Returned Chip ID: %lu (digital=%lu, rf=%lu). Make sure radar is properly connected.", chip_id, chip_id_digital, chip_id_rf);
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    /* 软件复位内部状态 */
    ESP_RETURN_ON_ERROR(xensiv_bgt60tr13c_soft_reset(XENSIV_BGT60TR13C_RESET_SW), TAG, "Failed to soft reset radar SW");

   /* 加载 bgt60tr13c_config.h 中的通用配置;
    FIFO >= 1055 个样本 (~1582 字节) 时触发中断 ; HS 模式关闭 */
    ESP_RETURN_ON_ERROR(xensiv_bgt60tr13c_configure(), TAG, "Failed to configure radar registers");

    /* 配置期间绕过 FIFO 错误通知 */
    radar_configured = true;

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_configure() //配置雷达寄存器
{
    for(uint8_t reg_idx = 0; reg_idx < XENSIV_BGT60TR13C_CONF_NUM_REGS; reg_idx++) 
    {
        uint32_t val = radar_init_register_list[reg_idx];
        uint32_t reg_addr = ((val & XENSIV_BGT60TR13C_SPI_REGADR_MSK) >> XENSIV_BGT60TR13C_SPI_REGADR_POS);
        uint32_t reg_data = ((val & XENSIV_BGT60TR13C_SPI_DATA_MSK) >>XENSIV_BGT60TR13C_SPI_DATA_POS);

        xensiv_bgt60tr13c_set_reg(reg_addr, reg_data, true);
    }
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_start_frame_capture() 
{
    /* 读取 MAIN 寄存器的当前值，确保不会覆盖其他位 */
    uint32_t tx_data = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);

    /* 设置帧采集启动位 */
    tx_data |= XENSIV_BGT60TR13C_REG_MAIN_FRAME_START_MSK;
    
    /* 写回 MAIN 寄存器启动帧采集。不验证，因为该位是只写自清零位 */
    ESP_RETURN_ON_ERROR(xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tx_data, false), TAG, "Failed to start radar frame capture");

    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_fifo_read(uint8_t *frame_buf, uint32_t buf_size, uint32_t words_to_read) 
{
    /* 确保缓冲区大小合法 —— 必须是 3 的倍数（对应 24-bit FIFO 字） */
    if ((buf_size % 3) != 0) 
    {
        ESP_LOGE(TAG, "Invalid buffer size. Must be a multiple of 3");
        return ESP_ERR_INVALID_SIZE;
    }

    /* 占用 SPI 总线 */
    esp_err_t ret = spi_device_acquire_bus(spi, portMAX_DELAY);
    if (ret != ESP_OK) 
    {
        ESP_LOGE("FIFO", "Failed to acquire SPI bus");
        return ret;
    }

    /* 
     * 构建 Burst 读命令 
     * 根据数据手册 Table 52: NBURSTS=0 表示无界读取
     */
    uint32_t burst_cmd = XENSIV_BGT60TR13C_SPI_BURST_MODE_CMD |
                         (XENSIV_BGT60TR13C_REG_FIFO_TR13C << XENSIV_BGT60TR13C_SPI_BURST_MODE_SADR_POS);
    // RWB=0 读操作, NBURSTS=0 无界读取 —— 默认值已是 0

    /* 对 ESP32 做字节翻转以匹配正确字节序 */
    burst_cmd = xensiv_bgt60tr13c_platform_word_reverse(burst_cmd);

    /* 
     * 为完整事务分配缓冲区:
     * TX: 4 字节 (burst 命令) + buf_size 字节 (提供时钟用的空数据)
     * RX: 4 字节 (GSR0 + 填充)  + buf_size 字节 (实际 FIFO 数据)
     */
    uint32_t total_length = 4 + buf_size;
    uint8_t *tx_buffer = calloc(total_length, 1);
    uint8_t *rx_buffer = malloc(total_length);
    
    if (tx_buffer == NULL || rx_buffer == NULL) 
    {
        ESP_LOGE(TAG, "Failed to allocate transaction buffers");
        free(tx_buffer);
        free(rx_buffer);
        spi_device_release_bus(spi);
        return ESP_ERR_NO_MEM;
    }

    /* 把 burst 命令拷贝到 TX 缓冲区 */
    memcpy(tx_buffer, &burst_cmd, 4);
    /* TX 缓冲区剩余部分已经全为零 (用于提供时钟、泵出 FIFO 数据的空字节) */

    /* 
     * 单次 SPI 事务: 发送命令 + 空字节，接收 GSR0 + FIFO 数据
     * 遵循数据手册 Figure 54 协议，在一次连续事务中完成
     */
    spi_transaction_t transaction = 
    {
        .cmd = 0,
        .addr = 0,
        .length = total_length * 8,      // TX 总长度 (bit)
        .tx_buffer = tx_buffer,
        .rxlength = total_length * 8,    // RX 总长度 (bit)
        .rx_buffer = rx_buffer,
        .flags = 0
    };

    ret = spi_device_polling_transmit(spi, &transaction);
    
    if (ret == ESP_OK) 
    {
        /* 检查 GSR0 是否有错误 (接收数据的第一字节) */
        uint8_t gsr0_status = rx_buffer[0];//GSR0 错误码
        ret = xensiv_bgt60tr13c_check_gsr0_err(gsr0_status);
        
        if (ret == ESP_OK) 
        {
            /* 拷贝 FIFO 数据 (跳过前 4 字节，即 GSR0 + 填充) */
            memcpy(frame_buf, &rx_buffer[4], buf_size);
        } 
        else 
        {
            ESP_LOGE(TAG, "GSR0 error detected: 0x%02X", gsr0_status);
        }
    } 
    else 
    {
        ESP_LOGE(TAG, "SPI FIFO transaction failed");
    }

    /* 清理缓冲区 */
    free(tx_buffer);
    free(rx_buffer);

    /* 释放 SPI 总线 */
    spi_device_release_bus(spi);

    return ret;
}

esp_err_t xensiv_bgt60tr13c_set_reg(uint32_t reg_addr, uint32_t data, bool verify_transaction) 
{
    /* 准备命令字节 (7-bit 地址 + R/W 位) */
    uint8_t tx_buffer[4];

    tx_buffer[0] = (reg_addr << 1) | 0x01; // R/W 位 = 1，表示写操作

    /* 准备 24-bit 数据字节 (高位先行) */
    tx_buffer[1] = (data >> 16) & 0xFF;
    tx_buffer[2] = (data >> 8) & 0xFF;
    tx_buffer[3] = data & 0xFF;

    /* 执行 SPI 事务 */
    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * sizeof(tx_buffer),
        .tx_buffer = tx_buffer,
        .flags = SPI_TRANS_USE_RXDATA
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_transmit(spi, &t));
    
    /* 记录 GSR0 错误码 */
    ESP_ERROR_CHECK_WITHOUT_ABORT(xensiv_bgt60tr13c_check_gsr0_err(t.rx_data[0]));

    if (verify_transaction) // 验证写入是否成功
    {
        uint32_t check_rx = xensiv_bgt60tr13c_get_reg(reg_addr) & 0x00FFFFFF;
        uint32_t check_tx = data & 0x00FFFFFF;
        if (check_rx != check_tx) 
        {
            ESP_LOGW(TAG, "Verification for transaction failed. Tried to write: %lu, instead register reads: %lu", check_tx, check_rx);
        } 
        else 
        {
            ESP_LOGI(TAG, "Transaction verified. Successful write to register %lu", reg_addr);
        }
    }
    return ESP_OK;
}

uint32_t xensiv_bgt60tr13c_get_reg(uint32_t reg_addr) 
{
    /* 准备命令字节 (7-bit 地址 + R/W 位) */
    uint8_t tx_buffer[4] = {0};
    tx_buffer[0] = (reg_addr << 1) | 0x00; // R/W 位 = 0，表示读操作
    
    /* 执行 SPI 事务 */
    spi_transaction_t t = {
        .cmd = 0,
        .addr = 0,
        .length = 8 * sizeof(tx_buffer),
        .tx_buffer = tx_buffer,
        .rxlength = 8 * sizeof(tx_buffer),
        .flags     = SPI_TRANS_USE_RXDATA
    };

    ESP_ERROR_CHECK_WITHOUT_ABORT(spi_device_polling_transmit(spi, &t));
    
    /* 记录 GSR0 错误码 */
    ESP_ERROR_CHECK_WITHOUT_ABORT(xensiv_bgt60tr13c_check_gsr0_err(t.rx_data[0]));

    // 跳过 GSR0 字节 (t.rx_data[0])，只返回 24-bit 寄存器数据
    return (t.rx_data[1] << 16) | (t.rx_data[2] << 8) | t.rx_data[3];
}

esp_err_t xensiv_bgt60tr13c_soft_reset(xensiv_bgt60tr13c_reset_t reset_type) 
{
    uint32_t tmp;
    int32_t status;

    tmp = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
    tmp |= (uint32_t)reset_type;
    status = xensiv_bgt60tr13c_set_reg(XENSIV_BGT60TR13C_REG_MAIN, tmp, false);

    uint32_t timeout = XENSIV_BGT60TR13C_RESET_WAIT_TIMEOUT;
    if (status == ESP_OK)
    {
        while (timeout > 0U)
        {
            tmp = xensiv_bgt60tr13c_get_reg(XENSIV_BGT60TR13C_REG_MAIN);
            if (((tmp & (uint32_t)reset_type) == 0U))
            {
                break;
            }
            --timeout;
        }
    }

    if (status == ESP_OK)
    {
        if (timeout == 0U)
        {
            return ESP_ERR_TIMEOUT;
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(XENSIV_BGT60TR13C_SOFT_RESET_DELAY_MS));
        }
    }
    return ESP_OK;
}

esp_err_t get_frame_size(uint32_t *external_frame_size) 
{
    *external_frame_size = frame_size;
    return ESP_OK;
}

esp_err_t get_interrupt_frame_size_trigger(uint32_t *external_frame_size) //获取中断触发帧大小
{
    *external_frame_size = XENSIV_BGT60TR13C_IRQ_TRIGGER_FRAME_SIZE;//中断触发帧大小
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_check_gsr0_err(uint8_t gsr0_err_code) 
{
    esp_err_t ret = ESP_OK;
    /* 假设错误码是 8 位，MSB 在左边 */
    uint8_t error_masked = gsr0_err_code & (XENSIV_BGT60TR13C_REG_GSR0_FOU_ERR_MSK | XENSIV_BGT60TR13C_REG_GSR0_SPI_BURST_ERR_MSK | XENSIV_BGT60TR13C_REG_GSR0_CLK_NUM_ERR_MSK);

    if (error_masked & XENSIV_BGT60TR13C_REG_GSR0_CLK_NUM_ERR_MSK) 
    {
        ESP_LOGE(TAG, "CLOCK NUMBER ERROR OCCURRED");
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if (error_masked & XENSIV_BGT60TR13C_REG_GSR0_SPI_BURST_ERR_MSK) 
    {
        ESP_LOGE(TAG, "SPI BURST READ ERROR OCCURRED");
        ret = ESP_ERR_INVALID_RESPONSE;
    }
    if ((error_masked & XENSIV_BGT60TR13C_REG_GSR0_FOU_ERR_MSK) && radar_configured)
    {
        ESP_LOGW(TAG, "RADAR FIFO OVERFLOW/UNDERFLOW ERROR DETECTED");
        ret = ESP_OK;
    }

    return ret;
}

esp_err_t xensiv_bgt60tr13c_read_back_registers(void) 
{
    ESP_LOGI(TAG, "=== Reading Back All Configured Registers ===");
    
    for(uint8_t reg_idx = 0; reg_idx < XENSIV_BGT60TR13C_CONF_NUM_REGS; reg_idx++) 
    {
        uint32_t expected_val = radar_init_register_list[reg_idx];
        uint32_t reg_addr = ((expected_val & XENSIV_BGT60TR13C_SPI_REGADR_MSK) >>
                                XENSIV_BGT60TR13C_SPI_REGADR_POS);
        uint32_t expected_data = ((expected_val & XENSIV_BGT60TR13C_SPI_DATA_MSK) >>
                                    XENSIV_BGT60TR13C_SPI_DATA_POS);
        
        uint32_t actual_data = xensiv_bgt60tr13c_get_reg(reg_addr);
        
        ESP_LOGI(TAG, "Reg[0x%02lX]: Expected=0x%06lX, Actual=0x%06lX %s", 
                 reg_addr, expected_data, actual_data, 
                 (expected_data == actual_data) ? "✓" : "✗");
        
        // 加一个小延迟，防止日志输出过于频繁
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    ESP_LOGI(TAG, "=== Register Readback Complete ===");
    return ESP_OK;
}

esp_err_t xensiv_bgt60tr13c_verify_configuration(void) 
{
    ESP_LOGI(TAG, "=== Verifying Configuration ===");
    
    uint32_t mismatched_registers = 0;
    
    for(uint8_t reg_idx = 0; reg_idx < XENSIV_BGT60TR13C_CONF_NUM_REGS; reg_idx++) 
    {
        uint32_t expected_val = radar_init_register_list[reg_idx];
        uint32_t reg_addr = ((expected_val & XENSIV_BGT60TR13C_SPI_REGADR_MSK) >>
                                XENSIV_BGT60TR13C_SPI_REGADR_POS);
        uint32_t expected_data = ((expected_val & XENSIV_BGT60TR13C_SPI_DATA_MSK) >>
                                    XENSIV_BGT60TR13C_SPI_DATA_POS);
        
        uint32_t actual_data = xensiv_bgt60tr13c_get_reg(reg_addr);
        
        if (expected_data != actual_data) 
        {
            ESP_LOGE(TAG, "MISMATCH Reg[0x%02lX]: Expected=0x%06lX, Actual=0x%06lX", 
                     reg_addr, expected_data, actual_data);
            mismatched_registers++;
        }
    }
    
    if (mismatched_registers == 0) 
    {
        ESP_LOGI(TAG, "All %d registers match expected values", XENSIV_BGT60TR13C_CONF_NUM_REGS);
        return ESP_OK;
    } 
    else 
    {
        ESP_LOGE(TAG, "%lu registers do not match expected values", mismatched_registers);
        return ESP_ERR_INVALID_STATE;
    }
}

esp_err_t xensiv_bgt60tr13c_read_specific_registers(const uint32_t *reg_addresses, uint32_t num_regs) 
{
    ESP_LOGI(TAG, "=== Reading Specific Registers ===");
    
    for(uint32_t i = 0; i < num_regs; i++) 
    {
        uint32_t reg_addr = reg_addresses[i];
        uint32_t reg_data = xensiv_bgt60tr13c_get_reg(reg_addr);
        ESP_LOGI(TAG, "Reg[0x%02lX] = 0x%06lX", reg_addr, reg_data);
    }
    
    return ESP_OK;
}