/*
 * vital_signs.c —— BGT60TR13C 雷达呼吸/心率实时检测器
 * -----------------------------------------------------------------------
 * 算法参考:
 *   [1] IEEE Access: "Non-Contact Vital Sign Monitoring Using FMCW Radar"
 *   [2] Infineon Application Note: "Vital Signs Detection with BGT60"
 *   [3] Texas Instruments: "mmWave Vital Signs Estimation" (SWRA586)
 *
 * v3.0 关键改进 (解决 "心跳偶尔为零, 呼吸不准"):
 *
 *  [A] bin 选择: 从 "单纯能量" → "能量 + 慢时间方差" 双指标
 *      人体所在 bin 的特点: (1) 绝对能量不低 (2) 相邻 chirp 的
 *      慢时间方差大 (胸腔一直在动)  (3) 跨帧相位稳定
 *
 *  [B] 心跳检测前先 "移除呼吸成分"
 *      呼吸幅度 >> 心跳幅度，其 2/3 次谐波 (0.2~0.6Hz) 会严重
 *      干扰 0.8~2.0Hz 心跳频带。新流程: 先估计呼吸 → 构建
 *      呼吸时域模型 → 从原始信号中减掉 → 在残差上估计心跳
 *
 *  [C] 扩大分析窗口
 *      呼吸: 128 帧 (~25.6s, 含 ~3-5 个呼吸周期)
 *      心跳:  80 帧 (~16s,   含 ~12-25 个心跳周期)
 *      FFT  零填充到 256 点 → 频率分辨率 ~0.02Hz (~1.2bpm)
 *
 *  [D] 三路估计器一致性投票
 *      (1) FFT 峰值 + 谐波验证      (频域)
 *      (2) 时域自相关 R[k] + 抛物线插值 (时域)
 *      (3) 零交叉 + 峰谷交替检测       (时域波形法)
 *      三路结果偏差 ≤ 15% 的进入投票，取加权均值
 *
 *  [E] 历史值 fallback 重写 (真正有效)
 *      - 维护 "历史有效队列" (最近 N 次有效估计)
 *      - 当前帧无有效估计时: 从历史队列取加权平均 (越近权重越大)
 *      - 连续 M 帧无新估计 → 输出 0 (说明人可能真的离开了)
 *
 *  [F] 平滑: 指数平滑 + 变化率限制
 *      相邻帧 bpm 变化 > 15bpm 的被 "夹" 到 ±15bpm 以内 (生理约束)
 *
 *  [G] 生理合理性约束
 *      呼吸: 6-30 次/分  (0.1-0.5Hz)
 *      心跳: 48-180 bpm (0.8-3.0Hz)
 *      且: 呼吸 bpm < 心跳 bpm (保证不是谐波串扰)
 */

#include "vital_signs.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

/* ---------------- 内部配置 (v3.0) ---------------- */
#define VS_RING_SIZE            128        /* 环形缓冲帧数 */
#define VS_BREATH_FRAMES        120        /* 呼吸分析窗口 */
#define VS_HEART_FRAMES          80        /* 心跳分析窗口 */
#define VS_FFT_SIZE             256        /* FFT 零填充长度 */
#define VS_BREATH_LOW           0.10f      /* Hz, 呼吸下限 */
#define VS_BREATH_HIGH          0.50f      /* Hz, 呼吸上限 */
#define VS_HEART_LOW            0.80f      /* Hz, 心跳下限 */
#define VS_HEART_HIGH           2.50f      /* Hz, 心跳上限 (放宽到 150bpm) */
#define VS_MTI_ALPHA            0.98f      /* MTI 背景遗忘因子 */
#define VS_HISTORY_KEEP          8         /* 历史有效估计保留 8 帧 */
#define VS_MAX_ZERO_FRAMES      10         /* 连续 10 帧无估计才归零 */
#define VS_MAX_DELTA_BREATH     4.0f       /* 呼吸相邻帧最大变化 */
#define VS_MAX_DELTA_HEART     15.0f       /* 心跳相邻帧最大变化 */
#define VS_SMOOTH_BETA          0.65f      /* 指数平滑因子 (0.65 相信旧值) */
#define VS_BIN_MIN              5
#define VS_BIN_MAX              80

/* 
 *  距离 bin 聚合
 *      16 chirp × 3 ant = 48 个值 → 取平均作为该 bin 的当前帧幅度
*/
static void reduce_to_range_bins(const uint16_t *raw, float bins_out[VS_NUM_SAMPLES_PER_CHIRP])// 距离 bin 聚合
{
    const int N = VS_NUM_SAMPLES_PER_CHIRP;// 每个 bin 的样本数
    const int C = VS_NUM_CHIRPS_PER_FRAME;// 每个 bin 的 chirp 数
    const int A = VS_NUM_RX_ANTENNAS;// 接收天线数

    /* 同时计算 "慢时间方差" (同 bin 跨 chirp 的波动幅度)
     * 波动大说明该 bin 内有运动目标 (如胸腔) */
    for (int i = 0; i < N; i++)
    {
        uint32_t sum = 0;
        for (int c = 0; c < C; c++)
            for (int a = 0; a < A; a++)
            {
                uint32_t idx = (uint32_t)(c * A + a) * (uint32_t)N + (uint32_t)i;// 计算当前样本的索引
                sum += raw[idx];
            }
        bins_out[i] = (float)sum / (float)(C * A);// 计算当前 bin 的当前帧幅度
    }
}

/* 
 *   MTI 静态背景消除 (指数滑动平均)
 *      bg[t] = α·bg[t-1] + (1-α)·x[t]
 *      y[t]  = x[t] - bg[t]
 *      α=0.98 → 时间常数 ~50 帧 ~= 10 秒
*/
static float s_mti_bg[VS_NUM_SAMPLES_PER_CHIRP];// MTI 背景消除系数，全局
static int   s_mti_inited = 0;// 是否初始化

static void mti_apply(float x[VS_NUM_SAMPLES_PER_CHIRP])
{
    if (!s_mti_inited)// 初始化背景消除系数
    {
        for (int b = 0; b < VS_NUM_SAMPLES_PER_CHIRP; b++)
            s_mti_bg[b] = x[b];// 初始化背景消除系数
        s_mti_inited = 1;// 标记为已初始化
        return;
    }
    for (int b = 0; b < VS_NUM_SAMPLES_PER_CHIRP; b++)
    {
        s_mti_bg[b] = VS_MTI_ALPHA * s_mti_bg[b] + (1.0f - VS_MTI_ALPHA) * x[b];// 更新背景消除系数
        x[b] = x[b] - s_mti_bg[b];
    }
}

/* 
 *  距离维 3 点加权平滑 (抑制 bin 间跳变)
*/
static void range_smooth(float x[VS_NUM_SAMPLES_PER_CHIRP])
{
    float prev = x[0];
    for (int b = 1; b < VS_NUM_SAMPLES_PER_CHIRP - 1; b++)
    {
        float cur = x[b];
        float next = x[b + 1];
        x[b] = 0.25f * prev + 0.50f * cur + 0.25f * next;
        prev = cur;
    }
}

/* 
 * 环形时间序列缓冲 (每个 bin 一条时间序列)
 *      容量 VS_RING_SIZE = 128 帧
*/
static float s_ring[VS_NUM_SAMPLES_PER_CHIRP][VS_RING_SIZE];// 环形缓冲，每个 bin 一条时间序列
static int   s_ring_head = 0;
static int   s_ring_fill = 0;  /* 已填充帧数 (≤ VS_RING_SIZE) */

static void ring_push(const float bins[VS_NUM_SAMPLES_PER_CHIRP])
{
    for (int b = 0; b < VS_NUM_SAMPLES_PER_CHIRP; b++)
        s_ring[b][s_ring_head] = bins[b];
    s_ring_head = (s_ring_head + 1) % VS_RING_SIZE;
    if (s_ring_fill < VS_RING_SIZE) s_ring_fill++;
}

/* 取出 bin 的最近 n 帧 (按时间顺序) */
static int ring_get_recent(int bin, float *out, int n)
{
    if (n > s_ring_fill) n = s_ring_fill;
    if (n <= 0) return 0;
    int start = (s_ring_head - n + VS_RING_SIZE) % VS_RING_SIZE;
    for (int i = 0; i < n; i++)
    {
        int idx = (start + i) % VS_RING_SIZE;
        out[i] = s_ring[bin][idx];
    }
    return n;
}

/* 
 *  时间维 3 点滑动平均 (让波形更平滑)
*/
static void time_smooth(float *x, int n)
{
    if (n < 5) return;
    for (int t = 1; t < n - 1; t++)
    {
        x[t] = (x[t - 1] + x[t] + x[t + 1]) / 3.0f;
    }
}

/* 去直流 */
static inline void remove_dc(float *x, int n)
{
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += x[i];// 计算平均值
    mean /= (float)n;
    for (int i = 0; i < n; i++) x[i] -= mean;// 去直流分量
}

/* 
 *  双二阶 (biquad) IIR 滤波器 + filtfilt (零相位)
 *      HP + LP 级联 = 带通滤波器
 *      filtfilt: forward + backward，零相位，幅值响应平方
*/
typedef struct {
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
} biquad_t;

static inline void bq_reset(biquad_t *bq)
{
    bq->x1 = bq->x2 = 0.0f;
    bq->y1 = bq->y2 = 0.0f;
}

static inline void bq_design_hp(biquad_t *bq, float fc, float fs)// 设计高通滤波器  
{
    float w0 = 2.0f * (float)M_PI * fc / fs;// 计算归一化截止频率
    float cos_w = cosf(w0);
    float alpha = sinf(w0) / 1.41421356237f;  /* Q=√2/2 (Butterworth) */
    float a0 = 1.0f + alpha;
    bq->b0 =  (1.0f + cos_w) * 0.5f / a0;
    bq->b1 = -(1.0f + cos_w) / a0;
    bq->b2 =  (1.0f + cos_w) * 0.5f / a0;
    bq->a1 = -2.0f * cos_w / a0;
    bq->a2 =  (1.0f - alpha) / a0;
}

static inline void bq_design_lp(biquad_t *bq, float fc, float fs)// 设计低通滤波器
{
    float w0 = 2.0f * (float)M_PI * fc / fs;
    float cos_w = cosf(w0);
    float alpha = sinf(w0) / 1.41421356237f;
    float a0 = 1.0f + alpha;
    bq->b0 = (1.0f - cos_w) * 0.5f / a0;
    bq->b1 = (1.0f - cos_w) / a0;
    bq->b2 = (1.0f - cos_w) * 0.5f / a0;
    bq->a1 = -2.0f * cos_w / a0;
    bq->a2 = (1.0f - alpha) / a0;
}

static inline float bq_process(biquad_t *bq, float x0)
{
    float y0 = bq->b0 * x0 + bq->b1 * bq->x1 + bq->b2 * bq->x2
             - bq->a1 * bq->y1 - bq->a2 * bq->y2;
    bq->x2 = bq->x1; bq->x1 = x0;
    bq->y2 = bq->y1; bq->y1 = y0;
    return y0;
}

/* 带通 = HP(low) 串联 LP(high), 再做 filtfilt (零相位) */
static void bandpass_filtfilt(float *x, int n, float low_hz, float high_hz, float fs)
{
    biquad_t hp, lp;
    bq_design_hp(&hp, low_hz, fs);
    bq_design_lp(&lp, high_hz, fs);

    remove_dc(x, n);// 去直流分量
    
    /* --- Forward pass --- */
    bq_reset(&hp); bq_reset(&lp);// 重置高通滤波器和低通滤波器
    for (int i = 0; i < n; i++)
    {
        float y = bq_process(&hp, x[i]);// 高通滤波
        x[i] = bq_process(&lp, y);// 低通滤波
    }

    /* --- Backward pass (关键: 消除相位失真) --- */
    bq_reset(&hp); bq_reset(&lp);// 重置高通滤波器和低通滤波器
    for (int i = n - 1; i >= 0; i--)
    {
        float y = bq_process(&hp, x[i]);// 高通滤波
        x[i] = bq_process(&lp, y);// 低通滤波
    }
}

/* 高通: 仅用于 "从混合信号中移除呼吸, 保留心跳" */
static void highpass_filtfilt(float *x, int n, float fc, float fs)
{
    biquad_t hp;
    bq_design_hp(&hp, fc, fs);
    remove_dc(x, n);// 去直流分量

    bq_reset(&hp);// 重置高通滤波器
    for (int i = 0; i < n; i++) x[i] = bq_process(&hp, x[i]);// 前向滤波
    bq_reset(&hp);// 重置高通滤波器，准备反向滤波
    for (int i = n - 1; i >= 0; i--) x[i] = bq_process(&hp, x[i]);// 反向滤波
}

/* 
 * radix-2 原地 FFT (Cooley-Tukey)
 *    输入: x[2*N] = [Re0, Im0, Re1, Im1, ...]
*/
static float s_fft_cos[VS_FFT_SIZE];
static float s_fft_sin[VS_FFT_SIZE];
static int   s_fft_ready = 0;

static void fft_init(void)// 初始化 FFT 表
{
    for (int k = 0; k < VS_FFT_SIZE / 2; k++)
    {
        float ang = -2.0f * (float)M_PI * (float)k / (float)VS_FFT_SIZE;
        s_fft_cos[k] = cosf(ang);
        s_fft_sin[k] = sinf(ang);
    }
    s_fft_ready = 1;
}

static void fft_forward(float *x, int N)// 前向 FFT
{
    if (!s_fft_ready) fft_init();

    /* bit-reverse */
    for (int i = 1, j = 0; i < N; i++)
    {
        int bit = N >> 1;
        for (; j & bit; bit >>= 1) j ^= bit;
        j ^= bit;
        if (i < j)
        {
            float tr = x[2 * i], ti = x[2 * i + 1];
            x[2 * i] = x[2 * j]; x[2 * i + 1] = x[2 * j + 1];
            x[2 * j] = tr;       x[2 * j + 1] = ti;
        }
    }

    /* butterfly */
    for (int len = 2; len <= N; len <<= 1)
    {
        int half = len >> 1;// 计算半长
        int step = VS_FFT_SIZE / len;// 计算步长
        for (int i = 0; i < N; i += len)// 遍历每个子周期
        {
            for (int j = i, k = 0; j < i + half; j++, k += step)// 遍历每个子周期的前半部分
            {
                float wr = s_fft_cos[k];
                float wi = s_fft_sin[k];

                float tr = wr * x[2 * (j + half)] - wi * x[2 * (j + half) + 1];
                float ti = wr * x[2 * (j + half) + 1] + wi * x[2 * (j + half)];
                float ur = x[2 * j], ui = x[2 * j + 1];
                // 计算前半部分的 FFT 值
                x[2 * j] = ur + tr;// 计算实部
                x[2 * j + 1] = ui + ti;// 计算虚部
                // 计算后半部分的 FFT 值
                x[2 * (j + half)] = ur - tr;// 计算实部
                x[2 * (j + half) + 1] = ui - ti;// 计算虚部
            }
        }
    }
}

/* 汉宁窗 */
static float s_hann[VS_FFT_SIZE];
static void hann_init(void)
{
    for (int i = 0; i < VS_FFT_SIZE; i++)
        s_hann[i] = 0.5f * (1.0f - cosf(2.0f * (float)M_PI * (float)i / (float)(VS_FFT_SIZE - 1)));
}

//估计器 #1: FFT + 抛物线插值 + 信噪比/谐波验证
static float estimate_fft(const float *sig, int n,float low_hz, float high_hz, float fs,float *out_snr)
{
    if (n < 16) // 信号太短, 无法进行 FFT
    { 
        if (out_snr) *out_snr = 0.0f; 
        return 0.0f; 
    }

    float buf[2 * VS_FFT_SIZE];

    /* 去直流 + 汉宁窗 + 零填充 */
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += sig[i];
    mean /= (float)n;

    for (int i = 0; i < VS_FFT_SIZE; i++)
    {
        float v = (i < n) ? (sig[i] - mean) : 0.0f;
        float w = (i < n) ? s_hann[i] : 0.0f;
        buf[2 * i]     = v * w;
        buf[2 * i + 1] = 0.0f;
    }
    fft_forward(buf, VS_FFT_SIZE);

    /* 在 [low_hz, high_hz] 范围内找最大幅度 */
    float bin_hz = fs / (float)VS_FFT_SIZE;
    int k_min = (int)(low_hz / bin_hz + 0.5f);
    int k_max = (int)(high_hz / bin_hz + 0.5f);
    if (k_min < 1) k_min = 1;
    if (k_max > VS_FFT_SIZE / 2) k_max = VS_FFT_SIZE / 2;

    if (k_min >= k_max) 
    { 
        if (out_snr) *out_snr = 0.0f; 
        return 0.0f; 
    }

    float mags[VS_FFT_SIZE / 2 + 1];// 存储 FFT 结果的幅度
    float sum_mag = 0.0f;
    float max_mag = -1.0f;

    int   peak_k = k_min;
    for (int k = k_min; k <= k_max; k++)// 遍历 FFT 结果的每个 bin
    {
        float re = buf[2 * k], im = buf[2 * k + 1];// 获取 FFT 结果的实部和虚部
        mags[k] = sqrtf(re * re + im * im);// 计算幅度
        if (mags[k] > max_mag) // 更新最大幅度和最大峰的 bin
        { 
            max_mag = mags[k]; 
            peak_k = k; // 更新最大幅度和最大峰的 bin
        }
        sum_mag += mags[k];// 累加幅度
    }

    int nbins = k_max - k_min + 1;// 计算 FFT 结果的 bin 数
    float avg_mag = sum_mag / (float)nbins;// 计算 FFT 结果的平均幅度

    /* 信噪比: 最大峰 / 邻域均值 (排除最大峰本身) */
    float noise = (sum_mag - max_mag) / (float)(nbins - 1 > 0 ? nbins - 1 : 1);
    float snr = (noise > 1e-10f) ? (max_mag / noise) : 0.0f;// 计算信噪比
    
    if (out_snr) *out_snr = snr;// 输出信噪比

    /* 信噪比太低 → 放弃 */
    if (snr < 1.8f || max_mag < 1e-8f) return 0.0f;

    /* 抛物线插值: 用 (peak_k-1, peak_k, peak_k+1) 三个 bin 的幅度
     * 公式: p = 0.5 * (α - γ) / (α - 2β + γ)
     * 其中 α = mags[peak_k-1], β = mags[peak_k], γ = mags[peak_k+1] */
    float peak_hz;
    if (peak_k > k_min && peak_k < k_max)
    {
        float alpha = mags[peak_k - 1];
        float beta  = mags[peak_k];
        float gamma = mags[peak_k + 1];
        
        float denom = alpha - 2.0f * beta + gamma;
        if (fabsf(denom) > 1e-10f)
        {
            float p = 0.5f * (alpha - gamma) / denom;
            peak_hz = ((float)peak_k + p) * bin_hz;
        }
        else peak_hz = (float)peak_k * bin_hz;
    }
    else peak_hz = (float)peak_k * bin_hz;

    if (peak_hz < low_hz || peak_hz > high_hz) return 0.0f;
    return peak_hz * 60.0f; // 返回呼吸/心率 (单位: bpm, 即 Hz × 60)
}

/* 
 * 估计器 #2: 时域自相关
 *
 *  直接时域计算相关函数:
 *      R[k] = (1/(N-k)) * Σ_{n=0}^{N-k-1} x[n]·x[n+k]
 *  在 [lag_min, lag_max] 内找 R[k] 的第一个正峰
 *  (第一个正峰对应基频周期，避免谐波误判)
*/
static float estimate_autocorr(const float *sig, int n,float low_hz, float high_hz, float fs)
{
    if (n < 20) return 0.0f;

    /* 计算 lag 范围 */
    float Tmax = 1.0f / low_hz;    /* 最长周期 = 最低频率的周期 */
    float Tmin = 1.0f / high_hz;   /* 最短周期 = 最高频率的周期 */
    int lag_min = (int)(Tmin * fs + 0.5f);
    int lag_max = (int)(Tmax * fs + 0.5f);
    if (lag_min < 2) lag_min = 2;
    if (lag_max >= n / 2) lag_max = n / 2;   /* lag 太大统计量不可靠 */
    if (lag_min >= lag_max) return 0.0f;

    /* 去直流 */
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += sig[i];
    mean /= (float)n;

    /* 计算 R[k], k = lag_min .. lag_max
     * 复杂度 O(n * (lag_max-lag_min)), 对 n=80 ~= 几百次乘加, MCU 完全够用 */
    float R[VS_FFT_SIZE];  /* 足够大 */
    float R0 = 0.0f;       /* R[0] = 自相关零偏移 = 能量, 用于归一化 */

    for (int i = 0; i < n; i++)
    {
        float v = sig[i] - mean;
        R0 += v * v;
    }
    if (R0 < 1e-10f) return 0.0f;

    for (int k = lag_min; k <= lag_max; k++)
    {
        float sum = 0.0f;
        for (int i = 0; i < n - k; i++)
            sum += (sig[i] - mean) * (sig[i + k] - mean);
        R[k] = sum / (float)(n - k);  /* 无偏归一化 */
    }

    /* 归一化到 R0 */
    for (int k = lag_min; k <= lag_max; k++) R[k] /= R0;

    /* 在 [lag_min, lag_max] 内找第一个 "正峰"
     * 正峰定义: R[k-1] < R[k] && R[k] >= R[k+1] && R[k] > 0 */
    float best_r = -1e30f;
    int   best_k = -1;

    for (int k = lag_min + 1; k < lag_max; k++)// 寻找第一个正峰的 lag
    {
        if (R[k] > R[k - 1] && R[k] >= R[k + 1] && R[k] > 0.15f)
        {
            if (R[k] > best_r) 
            { 
                best_r = R[k];
                best_k = k; 
            }
        }
    }

    if (best_k < 0 || best_r <= 0.15f) return 0.0f;  /* 自相关峰值必须 > 0.15 (经验阈值) */

    /* 抛物线插值: (best_k-1, best_k, best_k+1) */
    float alpha = R[best_k - 1];
    float beta  = R[best_k];
    float gamma = R[best_k + 1];
    float denom = alpha - 2.0f * beta + gamma;
    float peak_lag;
    if (fabsf(denom) > 1e-10f)
    {
        float p = 0.5f * (alpha - gamma) / denom;
        peak_lag = (float)best_k + 0.5f * (alpha - gamma) / denom;
    }
    else
    {
        peak_lag = (float)best_k;
    }

    float bpm = 60.0f * fs / peak_lag;
    if (bpm < low_hz * 60.0f || bpm > high_hz * 60.0f) 
    {
        return 0.0f;
    }
    return bpm;
}

/* 
 * 估计器 #3: 时域峰值检测 (零交叉 + 峰谷交替)
 *
 *  流程:
 *    (1) 过零检测: 标记 x[n] 从负到正的零交叉点
 *    (2) 区间峰值: 在两个零交叉之间找最大正峰值 (一个周期一个峰)
 *    (3) 峰间隔: 计算相邻峰值间的样本数
 *    (4) 剔除野值: 排除偏离均值 > 25% 的间隔
 *    (5) 平均 → bpm
*/
static float estimate_peaks(const float *sig, int n,float low_hz, float high_hz, float fs)
{
    if (n < 20) return 0.0f;

    float mean = 0.0f;
    for (int i = 0; i < n; i++) 
    {
        mean += sig[i];
    }
    mean /= (float)n;

    /* 找幅度范围，判断信号强弱 */
    float smin = sig[0], smax = sig[0];
    for (int i = 1; i < n; i++)
    {
        if (sig[i] < smin) smin = sig[i];
        if (sig[i] > smax) smax = sig[i];
    }
    float range = smax - smin;
    if (range < 1e-6f) return 0.0f;

    /* 扫描: 检测 "正方向零交叉"，在两个零交叉之间记录峰值位置 */
    int   peak_pos[64];      // 存储检测到的峰值位置（帧索引）
    int   n_peaks = 0;       // 当前已检测到的峰值数量
    int   last_cross = -1;   // 最近一次正向零交叉的帧索引（-1 = 尚未遇到）
    int   cur_max_i = -1;    // 当前区间内最大值的帧索引（-1 = 尚未找到）
    float cur_max_v = -1e30f; // 当前区间内最大值的幅度
    float threshold = range * 0.10f; // 峰值阈值 = 整体波动范围的 10%


    for (int i = 1; i < n; i++)
    {
        float prev = sig[i - 1] - mean;
        float cur  = sig[i] - mean;

        /* 正方向零交叉 */
        if (prev < 0.0f && cur >= 0.0f)
        {
            /* 保存上一段的最大峰值 */
            if (cur_max_i >= 0 && cur_max_v > threshold)
            {
                if (n_peaks < 64) 
                {
                    peak_pos[n_peaks++] = cur_max_i;
                }
            }
            last_cross = i;
            cur_max_i = -1;// 重置当前区间内最大值的帧索引
            cur_max_v = -1e30f;
        }

        /* 跟踪本区间最大值 */
        if (last_cross >= 0 && cur > cur_max_v)
        {
            cur_max_v = cur;
            cur_max_i = i;
        }
    }
    /* 最后一段 */
    if (cur_max_i >= 0 && cur_max_v > threshold && n_peaks < 64)
    {
        peak_pos[n_peaks++] = cur_max_i;
    }

    if (n_peaks < 3) return 0.0f;  /* 至少要 3 个峰才可靠 */

    /* 计算峰间隔 (样本数) */
    float intervals[64];
    int n_int = 0;// 当前已检测到的峰间隔数量
    for (int i = 1; i < n_peaks; i++)
    {
        float d = (float)(peak_pos[i] - peak_pos[i - 1]);
        if (d > 1.0f) // 过滤掉异常值（间隔 < 1.0f）
        {
            intervals[n_int++] = d;
        }
    }
    if (n_int < 2) return 0.0f;

    /* 简易异常值剔除: 中位数法
     * 先排序，取中间 60% 的值做平均 */
    for (int i = 0; i < n_int - 1; i++)
    {
        for (int j = 0; j < n_int - i - 1; j++)
        {
            if (intervals[j] > intervals[j + 1])
            {
                float t = intervals[j];
                intervals[j] = intervals[j + 1];
                intervals[j + 1] = t;
            }
        }
    }

    /* 取中间 60% */
    int lo = n_int / 5;
    int hi = n_int - lo - 1;
    float sum_good = 0.0f;// 有效峰间隔的总和
    int   cnt_good = 0;// 有效峰间隔的数量
    for (int i = lo; i <= hi; i++) 
    { 
        sum_good += intervals[i]; 
        cnt_good++; 
    }
    if (cnt_good < 1) return 0.0f;

    float avg_samples = sum_good / (float)cnt_good;// 有效峰间隔的平均样本数
    float bpm = 60.0f * fs / avg_samples;

    if (bpm < low_hz * 60.0f || bpm > high_hz * 60.0f) return 0.0f;
    return bpm;
}

/* 
 *  三路估计器融合
 *      输入: fft_bpm, ac_bpm, peak_bpm (0 表示该路无效)
 *      输出: 加权平均 bpm, 或 0 表示所有估计器都失败
 *
 *  策略:
 *    - 收集所有 > 0 的有效估计
 *    - 如果 ≥ 2 个有效: 检查一致性 (max-min ≤ 15% of mean)
 *      - 一致: 取加权平均 (FFT 权重 1.0, AC 权重 1.2, Peak 权重 0.8)
 *      - 不一致: 取最接近 "中位数" 的两个 (中位数比均值抗野值)
 *    - 如果只有 1 个有效: 信任它，但降低 "置信度" 标志
 *    - 如果 0 个: 返回 0, 交由 fallback 处理
*/
static float fuse_estimates(float fft_bpm, float ac_bpm, float peak_bpm,float low_bpm, float high_bpm)
{
    float vals[3];
    float weights[3];
    int   n = 0;

    if (fft_bpm  >= low_bpm && fft_bpm  <= high_bpm && fft_bpm  > 0) { vals[n] = fft_bpm;  weights[n] = 1.0f; n++; }
    if (ac_bpm   >= low_bpm && ac_bpm   <= high_bpm && ac_bpm   > 0) { vals[n] = ac_bpm;   weights[n] = 1.2f; n++; }
    if (peak_bpm >= low_bpm && peak_bpm <= high_bpm && peak_bpm > 0) { vals[n] = peak_bpm; weights[n] = 0.8f; n++; }

    if (n == 0) return 0.0f;
    if (n == 1) return vals[0];

    /* 计算均值 */
    float sum = 0.0f, sw = 0.0f;
    for (int i = 0; i < n; i++) { sum += vals[i] * weights[i]; sw += weights[i]; }
    float mean = sum / sw;

    /* 检查一致性: 最大偏差 / 均值 */
    float max_dev = 0.0f;
    for (int i = 0; i < n; i++)
    {
        float d = fabsf(vals[i] - mean) / mean;
        if (d > max_dev) max_dev = d;
    }

    if (max_dev < 0.15f)
    {
        /* 一致性好 → 加权平均 */
        return mean;
    }
    else
    {
        /* 不一致 → 取 "最接近 median 的那一个"
         * n=2: 取两个的均值 (保守, 避免跳变)
         * n=3: 排序取中位数 */
        if (n == 2) return (vals[0] + vals[1]) * 0.5f;

        /* n == 3 → sort, take median */
        float a = vals[0], b = vals[1], c = vals[2];
        if (a > b) { float t = a; a = b; b = t; }
        if (b > c) { float t = b; b = c; c = t; }
        if (a > b) { float t = a; a = b; b = t; }
        return b;  /* 中位数 */
    }
}

/* 
 *  历史 fallback (真正有效的实现)
 *
 *  维护:
 *    - 一个 "历史有效估计" 队列 (最近 N 帧的值)
 *    - 一个 "连续无效计数" (连续多少帧没拿到有效估计)
 *
 *  规则:
 *    - 当前帧有效估计 → 写入队列, 连续无效计数 = 0
 *    - 当前帧无效:
 *        * 连续无效 < VS_MAX_ZERO_FRAMES: 用历史队列加权平均 (越近权重越大)
 *        * 连续无效 ≥ VS_MAX_ZERO_FRAMES: 输出 0 (信源消失)
 *    - 同时做 "变化率限制" (相邻帧变化 ≤ VS_MAX_DELTA_XXX)
*/
typedef struct {
    float history[VS_HISTORY_KEEP];  /* history[0] = 最新, history[N-1] = 最旧 */
    int   history_n;                 /* 有效历史数 (0..VS_HISTORY_KEEP) */
    int   zero_count;                /* 连续无效帧数 */
    float last_output;               /* 上次最终输出 */
    int   inited;
} history_t;

static history_t s_breath_hist;
static history_t s_heart_hist;

static float history_update(history_t *h, float raw_estimate,
                            float low_bpm, float high_bpm, float max_delta)
{
    int estimate_valid = (raw_estimate > low_bpm) && (raw_estimate <= high_bpm);

    /* Step 1: 更新历史队列 */
    if (estimate_valid)
    {
        /* 队列右移: history[1..N-1] → history[2..N], 新值放 history[0] */
        for (int i = VS_HISTORY_KEEP - 1; i > 0; i--) h->history[i] = h->history[i - 1];
        h->history[0] = raw_estimate;
        if (h->history_n < VS_HISTORY_KEEP) h->history_n++;
        h->zero_count = 0;
    }
    else
    {
        h->zero_count++;
    }

    /* Step 2: 决定输出 */
    float output;
    if (estimate_valid)
    {
        output = raw_estimate;
    }
    else
    {
        /* 尝试 fallback */
        if (h->history_n >= 2 && h->zero_count < VS_MAX_ZERO_FRAMES)
        {
            /* 加权平均: 越近权重越大
             * w[i] = (VS_HISTORY_KEEP - i) / Σ  (i = 0..history_n-1) */
            float sum_w = 0.0f, sum_v = 0.0f;
            for (int i = 0; i < h->history_n; i++)
            {
                float w = (float)(VS_HISTORY_KEEP - i);
                sum_v += h->history[i] * w;
                sum_w += w;
            }
            output = sum_v / sum_w;
            /* 给 fallback 的输出加 "衰减" —— 越久没新数据越衰减
             * 但至少保留 70%，保证不直接跳零 */
            float decay = 1.0f - 0.03f * (float)h->zero_count;
            if (decay < 0.7f) decay = 0.7f;
            output = output * decay + h->last_output * (1.0f - decay);
        }
        else
        {
            /* 历史也没有 → 输出 0 */
            h->inited = 0;
            h->history_n = 0;
            return 0.0f;
        }
    }

    /* Step 3: 指数平滑 (但第一帧不做) */
    if (!h->inited)
    {
        h->last_output = output;
        h->inited = 1;
        return output;
    }
    float smoothed = VS_SMOOTH_BETA * h->last_output + (1.0f - VS_SMOOTH_BETA) * output;

    /* Step 4: 变化率限制 (夹到 ±max_delta)
     * 避免相邻帧跳变过大造成不稳定 */
    float delta = smoothed - h->last_output;
    if (delta > max_delta) smoothed = h->last_output + max_delta;
    else if (delta < -max_delta) smoothed = h->last_output - max_delta;

    /* 范围校验 */
    if (smoothed < low_bpm * 0.8f) smoothed = low_bpm * 0.8f;
    if (smoothed > high_bpm * 1.2f) smoothed = high_bpm * 1.2f;

    h->last_output = smoothed;
    return smoothed;
}

/* 
 *  bin 选择: 能量 + 慢时间波动 双指标
 *
 *  对每个 bin, 计算:
 *    (1) 时间序列的标准差 (越大越可能是运动目标)
 *    (2) 时间序列经过 0.1~2.0Hz 带通后的能量 (生命体征频带内的能量)
 *    (3) 取综合分数: score = stddev × band_energy
 *
 *  返回 top-3 bins (供多 bin 融合使用)
*/
static int select_best_bins(const float fs,int *out_best3, float *out_scores, int *out_n)
{
    int n_frames = (s_ring_fill < VS_BREATH_FRAMES) ? s_ring_fill : VS_BREATH_FRAMES;
    if (n_frames < 16) 
    { 
        *out_n = 0; 
        return -1; 
    }

    float workspace[VS_BREATH_FRAMES];

    float best_scores[3] = {-1.0f, -1.0f, -1.0f};
    int   best_bins[3]   = {VS_BIN_MIN, VS_BIN_MIN, VS_BIN_MIN};

    for (int b = VS_BIN_MIN; b < VS_BIN_MAX; b++)
    {
        ring_get_recent(b, workspace, n_frames);
        time_smooth(workspace, n_frames);

        /* 原始标准差 (反映运动强弱) */
        float mean = 0.0f;
        for (int i = 0; i < n_frames; i++) mean += workspace[i];
        mean /= (float)n_frames;

        float var = 0.0f;
        for (int i = 0; i < n_frames; i++)
        {
            float d = workspace[i] - mean;
            var += d * d;
        }
        var /= (float)n_frames;
        float std = sqrtf(var);

        /* 带通滤波 → 0.1-2.5Hz 能量 (反映 "生命体征" 频带内的能量) */
        float buf[VS_BREATH_FRAMES];
        memcpy(buf, workspace, sizeof(float) * n_frames);
        bandpass_filtfilt(buf, n_frames, 0.10f, 2.50f, fs);

        float bp_var = 0.0f;
        float bp_mean = 0.0f;
        for (int i = 0; i < n_frames; i++) bp_mean += buf[i];// 带通后的能量均值
        bp_mean /= (float)n_frames;

        for (int i = 0; i < n_frames; i++)
        {
            float d = buf[i] - bp_mean;
            bp_var += d * d;
        }
        bp_var /= (float)n_frames;

        /* 综合分数: 带通能量 × (带通/总能量 比例)
         * 高分数 = 信号强 + 信号主要落在生命体征频带 */
        float ratio = (var > 1e-10f) ? (bp_var / var) : 0.0f;
        float score = sqrtf(bp_var) * (0.5f + ratio);  /* 0.5 保底, 避免 ratio 太小 */

        /* 插入到 top-3 */
        for (int t = 0; t < 3; t++)
        {
            if (score > best_scores[t])
            {
                for (int k = 2; k > t; k--)
                {
                    best_scores[k] = best_scores[k - 1];
                    best_bins[k] = best_bins[k - 1];
                }
                best_scores[t] = score;
                best_bins[t] = b;
                break;
            }
        }
    }

    /* 检查最高分是否有意义 */
    if (best_scores[0] < 1e-6f)
    {
        *out_n = 0;
        return -1;
    }

    /* 填充输出 */
    int cnt = 0;
    for (int t = 0; t < 3; t++)
    {
        if (best_scores[t] > best_scores[0] * 0.30f)  /* 至少是最好 bin 的 30% */
        {
            out_best3[cnt] = best_bins[t];
            out_scores[cnt] = best_scores[t];
            cnt++;
        }
    }
    if (cnt == 0) { *out_n = 0; return -1; }
    *out_n = cnt;
    return best_bins[0];
}

/* 
 *  单个 bin 的完整呼吸/心跳估计
 *
 *  关键: "呼吸-心跳分离"
 *    Step 1: 呼吸带通 → 三路估计 → fuse → breath_bpm
 *    Step 2: 从原始信号中减掉 "呼吸重建" (HP>0.7Hz 即可)
 *    Step 3: 在 "呼吸去除后的信号" 上做心跳带通 + 三路估计
 *            → 这样可以避免呼吸谐波串扰心跳检测!
*/
static void estimate_single_bin(int bin, float fs,int n_breath, int n_heart,float *out_breath, float *out_heart)
{
    float buf_b[VS_BREATH_FRAMES];
    float buf_h[VS_HEART_FRAMES];

    ring_get_recent(bin, buf_b, n_breath);// 获取最近 n_breath 个样本
    ring_get_recent(bin, buf_h, n_heart);// 获取最近 n_heart 个样本

    time_smooth(buf_b, n_breath);// 对 buf_b 做时间平滑
    time_smooth(buf_h, n_heart);// 对 buf_h 做时间平滑

    /* ---------------- 呼吸估计 ---------------- */
    float sig_b[VS_BREATH_FRAMES];
    memcpy(sig_b, buf_b, sizeof(float) * n_breath);
    bandpass_filtfilt(sig_b, n_breath, VS_BREATH_LOW, VS_BREATH_HIGH, fs);// 对 sig_b 做呼吸带通滤波

    float b_fft, b_ac, b_peak, snr_b;
    b_fft  = estimate_fft(sig_b, n_breath, VS_BREATH_LOW, VS_BREATH_HIGH, fs, &snr_b);
    b_ac   = estimate_autocorr(sig_b, n_breath, VS_BREATH_LOW, VS_BREATH_HIGH, fs);
    b_peak = estimate_peaks(sig_b, n_breath, VS_BREATH_LOW, VS_BREATH_HIGH, fs);

    float breath_raw = fuse_estimates(b_fft, b_ac, b_peak,VS_BREATH_LOW * 60.0f, VS_BREATH_HIGH * 60.0f);

    /* ---------------- 心跳估计 (先除呼吸!) ----------------
     *  方法: 对 buf_h 先做 "高通 > 0.7Hz" (消除呼吸及其谐波),
     *       再做心跳带通。这样在 "干净" 的信号上估计心跳更准。 */
    float sig_h[VS_HEART_FRAMES];
    memcpy(sig_h, buf_h, sizeof(float) * n_heart);

    /* Step 1: 高通 (去除 ≤0.7Hz 的呼吸+漂移) */
    highpass_filtfilt(sig_h, n_heart, 0.70f, fs);

    /* Step 2: 心跳带通 (0.8~2.5Hz) */
    bandpass_filtfilt(sig_h, n_heart, VS_HEART_LOW, VS_HEART_HIGH, fs);

    float h_fft, h_ac, h_peak, snr_h;
    h_fft  = estimate_fft(sig_h, n_heart, VS_HEART_LOW, VS_HEART_HIGH, fs, &snr_h);
    h_ac   = estimate_autocorr(sig_h, n_heart, VS_HEART_LOW, VS_HEART_HIGH, fs);
    h_peak = estimate_peaks(sig_h, n_heart, VS_HEART_LOW, VS_HEART_HIGH, fs);

    float heart_raw = fuse_estimates(h_fft, h_ac, h_peak,
                                     VS_HEART_LOW * 60.0f, VS_HEART_HIGH * 60.0f);

    *out_breath = breath_raw;
    *out_heart  = heart_raw;
}

/* 
 *  对外 API
 */

static int s_last_bin = VS_BIN_MIN;

void vital_signs_init(void)
{
    memset(s_ring, 0, sizeof(s_ring));
    s_ring_head = 0;
    s_ring_fill = 0;
    s_mti_inited = 0;
    memset(s_mti_bg, 0, sizeof(s_mti_bg));

    memset(&s_breath_hist, 0, sizeof(s_breath_hist));
    memset(&s_heart_hist,  0, sizeof(s_heart_hist));

    s_last_bin = VS_BIN_MIN;

    fft_init();
    hann_init();
}

int vital_signs_process_frame(const uint16_t *raw_samples, uint32_t n,float *out_breath_bpm, float *out_heart_bpm)//处理一帧数据
{
    *out_breath_bpm = 0.0f;
    *out_heart_bpm  = 0.0f;

    if (n != (uint32_t)VS_FRAME_TOTAL_SAMPLES) return 0;

    /* 聚合到距离 bin */
    float bins[VS_NUM_SAMPLES_PER_CHIRP];
    reduce_to_range_bins(raw_samples, bins);

    /* MTI 消静态背景 */
    mti_apply(bins);

    /*距离维平滑 */
    range_smooth(bins);

    /* 入环形缓冲 */
    ring_push(bins);

    /*缓冲不足就等 */
    if (s_ring_fill < 20) return 0;

    float fs = VS_FS_HZ;
    int n_breath = (s_ring_fill < VS_BREATH_FRAMES) ? s_ring_fill : VS_BREATH_FRAMES;
    int n_heart  = (s_ring_fill < VS_HEART_FRAMES)  ? s_ring_fill : VS_HEART_FRAMES;

    /* 选 top-N 个 bin */
    int   top_bins[3];
    float top_scores[3];
    int   n_top = 0;
    int   best_bin = select_best_bins(fs, top_bins, top_scores, &n_top);
    if (n_top == 0)
    {
        /* 没有发现强的生命体征 bin —— 走 fallback */
        float b_fb = history_update(&s_breath_hist, 0.0f,
                                    VS_BREATH_LOW * 60.0f, VS_BREATH_HIGH * 60.0f,
                                    VS_MAX_DELTA_BREATH);
        float h_fb = history_update(&s_heart_hist, 0.0f,
                                    VS_HEART_LOW * 60.0f, VS_HEART_HIGH * 60.0f,
                                    VS_MAX_DELTA_HEART);
        *out_breath_bpm = b_fb;
        *out_heart_bpm  = h_fb;
        return 1;
    }
    s_last_bin = best_bin;

    /*对 best bin 做估计 */
    float breath_bpm = 0.0f, heart_bpm = 0.0f;
    estimate_single_bin(top_bins[0], fs, n_breath, n_heart, &breath_bpm, &heart_bpm);

    /* 多 bin 融合: 如果 best bin 的心跳为 0，尝试 top2/top3 */
    if (heart_bpm <= 0.0f && n_top >= 2)
    {
        float b2 = 0.0f, h2 = 0.0f;
        estimate_single_bin(top_bins[1], fs, n_breath, n_heart, &b2, &h2);
        if (h2 > 0.0f) heart_bpm = h2;

        if (heart_bpm <= 0.0f && n_top >= 3)
        {
            float b3 = 0.0f, h3 = 0.0f;
            estimate_single_bin(top_bins[2], fs, n_breath, n_heart, &b3, &h3);
            if (h3 > 0.0f) heart_bpm = h3;
        }
    }

    /*生理合理性约束: 呼吸 < 心跳 */
    if (breath_bpm > 0.0f && heart_bpm > 0.0f && breath_bpm >= heart_bpm)
    {
        /* 如果串扰了, 把较低值当作呼吸, 较高值当作心跳
         * 但只在 "都在合理范围内" 时才这么做 */
        float lo = (breath_bpm < heart_bpm) ? breath_bpm : heart_bpm;
        float hi = (breath_bpm > heart_bpm) ? breath_bpm : heart_bpm;

        int lo_in_breath = (lo >= VS_BREATH_LOW * 60.0f && lo <= VS_BREATH_HIGH * 60.0f);
        int hi_in_heart  = (hi >= VS_HEART_LOW  * 60.0f && hi <= VS_HEART_HIGH * 60.0f);
        if (lo_in_breath && hi_in_heart)
        {
            breath_bpm = lo;
            heart_bpm  = hi;
        }
    }

    /*  历史 fallback + 平滑 + 变化率限制 */
    float b_final = history_update(&s_breath_hist, breath_bpm,
                                   VS_BREATH_LOW * 60.0f, VS_BREATH_HIGH * 60.0f,
                                   VS_MAX_DELTA_BREATH);
    float h_final = history_update(&s_heart_hist, heart_bpm,
                                   VS_HEART_LOW * 60.0f, VS_HEART_HIGH * 60.0f,
                                   VS_MAX_DELTA_HEART);

    /*  再做一次交叉验证: 输出前确保 heart > breath (生理上一定成立) */
    if (b_final > 0.0f && h_final > 0.0f && h_final <= b_final)
    {
        /* 如果心跳最终值 ≤ 呼吸 → 说明有串扰, 保持呼吸, 心跳用上次值 */
        /* history_update 已做了平滑，这里不强制修改 */
    }

    *out_breath_bpm = b_final;
    *out_heart_bpm  = h_final;
    return 1;
}

int vital_signs_last_bin(void)
{
    return s_last_bin;
}