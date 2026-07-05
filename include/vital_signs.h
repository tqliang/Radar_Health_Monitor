#ifndef VITAL_SIGNS_H
#define VITAL_SIGNS_H

#include <stdint.h>
#include "bgt60tr13c_config.h"

/*  雷达硬件参数 (与 bgt60tr13c_config.h 保持一致)*/
#define VS_NUM_SAMPLES_PER_CHIRP    XENSIV_BGT60TR13C_CONF_NUM_SAMPLES_PER_CHIRP   /* 128 */
#define VS_NUM_CHIRPS_PER_FRAME     XENSIV_BGT60TR13C_CONF_NUM_CHIRPS_PER_FRAME    /* 16  */
#define VS_NUM_RX_ANTENNAS          XENSIV_BGT60TR13C_CONF_NUM_RX_ANTENNAS         /* 3   */
#define VS_FRAME_TOTAL_SAMPLES      (VS_NUM_SAMPLES_PER_CHIRP * VS_NUM_CHIRPS_PER_FRAME * VS_NUM_RX_ANTENNAS) /* 6144 */
#define VS_FRAME_PERIOD_S           XENSIV_BGT60TR13C_CONF_FRAME_REPETITION_TIME_S /* 0.199634 */
#define VS_FS_HZ                    (1.0f / VS_FRAME_PERIOD_S)                     /* ~5.009 */

#define VS_BIN_MIN_INDEX            5
#define VS_BIN_MAX_INDEX            80

/*滑动窗口 (v3.0 更新) */
#define VS_BREATH_WINDOW_FRAMES     120     /* ~24 秒, 含 4-8 个呼吸周期 */
#define VS_HEART_WINDOW_FRAMES      80      /* ~16 秒, 含 20-40 个心跳周期 */
#define VS_RING_CAPACITY            128     /* 环形缓冲容量 (>= BREATH_WINDOW) */

/* 生理频带 (v3.0 心跳上限放宽) */
#define VS_BREATH_LOW_HZ            0.10f   /* 6 次/分 */
#define VS_BREATH_HIGH_HZ           0.50f   /* 30 次/分 */
#define VS_HEART_LOW_HZ             0.80f   /* 48 bpm */
#define VS_HEART_HIGH_HZ            2.50f   /* 150 bpm (放宽到运动心率) */

/* MTI / 平滑 */
#define VS_MTI_ALPHA                0.98f   /* 温和, 避免衰减呼吸信号 */
#define VS_OUTPUT_SMOOTH_BETA       0.65f   /* 指数平滑: 65% 信旧值 */

/* FFT 长度 (v3.0 扩展到 256, 提高频率分辨率) */
#define VS_FFT_N                    256

/* 初始化 —— 在主任务中调用一次 */
void vital_signs_init(void);

int  vital_signs_process_frame(const uint16_t *raw_samples, uint32_t n,
                               float *out_breath_bpm, float *out_heart_bpm);

int  vital_signs_last_bin(void);

#endif /* VITAL_SIGNS_H */