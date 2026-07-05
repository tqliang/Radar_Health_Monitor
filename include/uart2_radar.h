#ifndef UART2_RADAR_H
#define UART2_RADAR_H

#include <stdint.h>

#define UART2_RADAR_NUM            UART_NUM_1    /* ESP32-S3 UART1/2 均可, 这里用 UART1 (逻辑名 UART2) */
#define UART2_RADAR_TXD            39            /* GPIO39 = TXD2 */
#define UART2_RADAR_RXD            40            /* GPIO40 = RXD2 */
#define UART2_RADAR_RTS            (-1)
#define UART2_RADAR_CTS            (-1)
#define UART2_RADAR_BAUD           115200
#define UART2_RADAR_BUF_SIZE       (1024)

/* 
 帧协议 (二进制, 16 字节固定长度)
 *
 *   字节  0    1     2    3    4    5    6    7    8    9   10   11   12   13   14   15
 *        +---+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *        | HDR |FLAG|   frame_cnt   |  breath_raw  |  heart_raw   |  CRC8 |
 *        +---+----+----+----+----+----+----+----+----+----+----+----+----+----+
 *          0xAA 0x55   uint32 LE      float LE       float LE
 *
 *   - HDR = 帧头 0xAA55 (固定, 便于上位机同步)
 *   - FLAG = 0x01 固定, 预留 (未来可扩展多类型帧)
 *   - frame_cnt = 帧序号, 每发一帧自增, 方便上位机检测丢帧
 *   - breath_raw = 呼吸率 bpm (IEEE-754 单精度浮点数, Little-Endian)
 *   - heart_raw  = 心率 bpm  (IEEE-754 单精度浮点数, Little-Endian)
 *   - CRC8 = MAXIM/DALLAS 多项式 0x31, 覆盖字节 0..12
 *   - 总长度 = 16 字节, 约 1.4 ms @ 115200, 不会阻塞
 
 */


#define UART2_FRAME_HEADER_0       0xAA
#define UART2_FRAME_HEADER_1       0x55
#define UART2_FRAME_FLAG           0x01
#define UART2_FRAME_SIZE           (16)

void uart2_radar_init(void);

void uart2_radar_send_frame(float breath_bpm, float heart_bpm, uint32_t frame_cnt);

#endif /* UART2_RADAR_H */