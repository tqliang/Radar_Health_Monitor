#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
===================================================================
  BGT60TR13C 雷达 —— 呼吸/心率 实时检测 (Python 上位机原型)
-------------------------------------------------------------------
  目标:
    1. 从串口接收 ESP32 发来的雷达帧数据
    2. 对 128 个距离 bin 做跨时间的相位/幅度跟踪
    3. 估计呼吸频率 (0.1–0.5 Hz, 6–30 次/分)
    4. 估计心率    (0.8–2.0 Hz, 48–120 bpm)
    5. 算法尽量简单，便于之后移植到 MCU (ESP32-S3)

  ESP32 端当前的数据格式 (main.c):
      Frame 3: [1376, 843, 394, ..., 2619, 150]   (6144 个 12-bit 值)
      布局:   (chirp × NUM_ANTENNAS + ant) × 128 + sample_idx

  处理流程 (每帧):
      [ 收一帧 6144 值 ] → [ 平均 16 chirp × 3 ant → 128 个距离-bin ]
          → [ 压入 128 条时间序列 (环形缓冲) ]
          → [ 对每个 bin 做 DC 移除 + 带通滤波 ]
          → [ 选振荡最强的 bin (在 0.1–2 Hz 频段) ]
          → [ 呼吸带通 + 峰值/FFT 估计频率 ]
          → [ 心跳带通 + 峰值/FFT 估计频率 ]

  参数来源 (bgt60tr13c_config.h):
      FRAME_REPETITION_TIME_S = 0.199634   (约 5.009 Hz 帧速率)

  作者: 仅供调试原型，后续移植到 ESP32-S3
===================================================================
"""

import sys
import time
import re
import serial
import serial.tools.list_ports

import numpy as np
from scipy.signal import butter, filtfilt, find_peaks


# ============================================================
#  ↓↓↓ 可调节参数 ↓↓↓
# ============================================================

# --- 串口 ---
SERIAL_PORT     = None          # None = 自动选择
SERIAL_BAUDRATE = 921600

# --- 雷达硬件 (与 bgt60tr13c_config.h 保持一致) ---
FRAME_PERIOD_S      = 0.199634   # 帧周期 (秒)
NUM_SAMPLES_PER_CHIRP = 128      # 每 chirp sample 数 = 距离 bin 数
NUM_CHIRPS_PER_FRAME = 16        # 每帧 chirp 数
NUM_RX_ANTENNAS     = 3          # RX 天线数
FRAME_TOTAL_SAMPLES = NUM_SAMPLES_PER_CHIRP * NUM_CHIRPS_PER_FRAME * NUM_RX_ANTENNAS  # 6144

# --- 距离 bin 分析范围 ---
#   人体通常在近距 bin, 远距 bin 只有噪声, 只扫前一段减少计算量
BIN_MIN_INDEX = 5       # 跳过最近 (可能是地面/静止杂波)
BIN_MAX_INDEX = 80      # 最远分析到第 80 个 bin

# --- 滑动窗口 (关键参数) ---
#   采样率 fs = 1 / FRAME_PERIOD_S ≈ 5.009 Hz
#   呼吸 0.1 Hz → 10 s 周期, 至少 20 s 才够 2 个周期 → 100 帧
#   心跳 1.5 Hz → 0.67 s 周期, 5 s 就有 7 个周期 → 25 帧
BREATH_WINDOW_FRAMES = 120   # ~24 秒
HEART_WINDOW_FRAMES  = 40    # ~8 秒

# --- 生理频带 (Hz) ---
BREATH_LOW_HZ  = 0.10
BREATH_HIGH_HZ = 0.50
HEART_LOW_HZ   = 0.8
HEART_HIGH_HZ  = 2.0

# --- FFT 最小可用帧数 ---
FFT_MIN_FRAMES = 16

# ----------------------------------------------------------------
#  ↓↓↓ MCU 可移植的优化 (全部是 MAC + 比较, 无任何外部依赖) ↓↓↓
# ----------------------------------------------------------------

# --- [1] 静态 MTI 背景消除 (Moving Target Indication) ---
#   原理:   bg[t] = α * bg[t-1] + (1-α) * x[t]     指数滑动平均估计静态背景
#           y[t]  = x[t] - bg[t]                    只保留运动分量
#   α 越接近 1 → 背景越"慢", 静止物体被消得越干净, 但对刚进入场景的目标响应慢
#   α 建议:   0.90 ~ 0.98  (对应 ~10% 每帧更新)
#   MCU 移植: 就是一个 float bg[128] 和一行乘加, 零额外内存
MTI_ALPHA = 0.95
MTI_ENABLE  = True

# --- [2] 距离维滑动平滑 (3 点加权平均) ---
#   对相邻距离 bin 做平滑:  y[b] = 0.25*x[b-1] + 0.5*x[b] + 0.25*x[b+1]
#   作用:   抑制距离维的高频噪声 (比如单个 bin 偶然跳变)
#   代价:   距离分辨率轻微下降 (1 个 bin 的展宽)
#   MCU 移植: for 循环乘加, 无缓冲区
RANGE_SMOOTH_ENABLE = True

# --- [3] 时间维滑动平滑 (3 点等权平均) ---
#   对每个 bin 的时间序列做 y[t] = (x[t-1] + x[t] + x[t+1]) / 3
#   作用:   抑制高频抖动, 让呼吸/心跳波形更光滑, 峰值检测更稳
#   MCU 移植: 对取出的窗口做前后向滑动平均
TIME_SMOOTH_ENABLE = True

# --- [4] 结果平滑 (指数平滑输出) ---
#   让显示/最终输出的 bpm 不跳来跳去: out[t] = β*out[t-1] + (1-β)*in[t]
#   β = 0.7 表示 70% 相信旧值, 30% 接受新测量
OUTPUT_SMOOTH_BETA = 0.7

# ----------------------------------------------------------------
#  ↑↑↑ MCU 可移植的优化 ↑↑↑
# ----------------------------------------------------------------

# --- 显示更新间隔 (秒) ---
DISPLAY_UPDATE_S = 0.5


# ============================================================
#  ↑↑↑ 可调节参数 ↑↑↑
# ============================================================


FS = 1.0 / FRAME_PERIOD_S   # 帧采样率 ≈ 5.009 Hz


# ============================================================
#  [MCU 可移植 #1] 静态 MTI 背景消除器
#
#    bg[b]   = α * bg[b]  +  (1-α) * x[b]     (指数滑动平均)
#    y[b]    = x[b] - bg[b]                     (只剩运动分量)
#
#  MCU 移植:  float bg[128];    // BSS 段自动初始化为 0
#             void mti_update(float *x, float *y) {
#                 for (int b=0; b<128; b++) {
#                     bg[b] = MTI_ALPHA*bg[b] + (1.0f-MTI_ALPHA)*x[b];
#                     y[b]  = x[b] - bg[b];
#                 }
#             }
# ============================================================
class StaticMTIFilter:
    def __init__(self, n_bins: int = NUM_SAMPLES_PER_CHIRP, alpha: float = MTI_ALPHA):
        self.n = int(n_bins)
        self.alpha = float(alpha)
        self._bg = np.zeros(self.n, dtype=np.float32)   # 静态背景估计

    def apply(self, x: np.ndarray) -> np.ndarray:
        """输入 x = 当前帧 128 个 bin, 输出 y = 去静态背景后的 bin"""
        self._bg = self.alpha * self._bg + (1.0 - self.alpha) * x.astype(np.float32)
        return x.astype(np.float32) - self._bg


# ============================================================
#  [MCU 可移植 #2] 距离维 3 点加权滑动平滑
#
#    y[b] = 0.25*x[b-1] + 0.50*x[b] + 0.25*x[b+1]
#    端点保持原样 (不做 padding)
#
#  MCU 移植: 一个 for 循环 + 2 次乘加, 无额外缓冲区
# ============================================================
def range_smooth_3pt(x: np.ndarray) -> np.ndarray:
    if not RANGE_SMOOTH_ENABLE or len(x) < 3:
        return x.copy() if isinstance(x, np.ndarray) else np.array(x, dtype=np.float32)
    y = np.zeros_like(x, dtype=np.float32)
    y[0]  = x[0]
    y[-1] = x[-1]
    for b in range(1, len(x) - 1):
        y[b] = 0.25 * x[b - 1] + 0.50 * x[b] + 0.25 * x[b + 1]
    return y


# ============================================================
#  [MCU 可移植 #3] 时间维 3 点等权滑动平滑
#
#    y[t] = (x[t-1] + x[t] + x[t+1]) / 3.0
#
#  MCU 移植: 对取出的窗口做前后向的滑动平均 (FIR, 无状态)
# ============================================================
def time_smooth_3pt(x: np.ndarray) -> np.ndarray:
    if not TIME_SMOOTH_ENABLE or len(x) < 3:
        return x.copy() if isinstance(x, np.ndarray) else np.array(x, dtype=np.float32)
    y = np.zeros_like(x, dtype=np.float32)
    y[0]  = x[0]
    y[-1] = x[-1]
    for t in range(1, len(x) - 1):
        y[t] = (x[t - 1] + x[t] + x[t + 1]) / 3.0
    return y


# ============================================================
#  [MCU 可移植 #4] 输出结果的指数平滑
#
#    out[t] = β * out[t-1] + (1-β) * in[t]
#
#  MCU 移植: 一个 float 变量保存旧值, 一行乘加
# ============================================================
class OutputSmoother:
    def __init__(self, beta: float = OUTPUT_SMOOTH_BETA):
        self.beta = float(beta)
        self._prev = 0.0
        self._inited = False

    def apply(self, x: float) -> float:
        if x <= 0:
            return 0.0   # 无效值不参与平滑
        if not self._inited:
            self._prev = x
            self._inited = True
            return x
        self._prev = self.beta * self._prev + (1.0 - self.beta) * x
        return self._prev


# ============================================================
#  数据缓冲类: 128 条时间序列 (环形缓冲, 覆盖最旧)
#  —— 后续移植到 MCU 时, 用 float buf[128][N] + write_index 实现
# ============================================================
class RingBuffer128:
    def __init__(self, capacity: int):
        self.capacity = int(capacity)
        # shape = (NUM_SAMPLES_PER_CHIRP, capacity)  —— 128 行 × N 列
        self.data = np.zeros((NUM_SAMPLES_PER_CHIRP, self.capacity), dtype=np.float32)
        self._head = 0        # 下一个写入位置
        self._size = 0        # 已填充帧数 (≤ capacity)

    def push(self, frame_bin: np.ndarray):
        """写入一帧 128 个距离 bin 值"""
        self.data[:, self._head] = frame_bin.astype(np.float32)
        self._head = (self._head + 1) % self.capacity
        if self._size < self.capacity:
            self._size += 1

    def latest(self, n: int) -> np.ndarray:
        """取最近 n 帧, 返回 shape = (128, n). n > size 时自动截断"""
        n = min(int(n), self._size)
        if n <= 0:
            return np.zeros((NUM_SAMPLES_PER_CHIRP, 0), dtype=np.float32)
        # 最后 n 个索引 (处理环形)
        idx = [(self._head - n + i) % self.capacity for i in range(n)]
        return self.data[:, idx].copy()

    @property
    def size(self) -> int:
        return self._size


# ============================================================
#  带通滤波 (零相位 filtfilt)
#  —— 移植到 MCU 时建议用 4 阶 IIR 双二阶节 (biquad cascade)
# ============================================================
def bandpass(sig: np.ndarray, low_hz: float, high_hz: float, fs: float, order=4):
    nyq = 0.5 * fs
    low  = max(low_hz / nyq, 0.001)
    high = min(high_hz / nyq, 0.999)
    if low >= high or len(sig) < 2 * order:
        return sig * 0.0
    b, a = butter(order, [low, high], btype="band")
    return filtfilt(b, a, sig)


# ============================================================
#  FFT 频率估计 + 抛物线插值提高分辨率
#  —— 移植到 MCU 时可用 CMSIS-DSP arm_cfft_f32
# ============================================================
def estimate_freq_fft(sig: np.ndarray, fs: float,
                      low_hz: float, high_hz: float):
    """
    在 [low_hz, high_hz] 频段内找最大能量频率点.
    返回 (bpm, 能量占比, peak_hz)
    """
    n = len(sig)
    if n < FFT_MIN_FRAMES:
        return 0.0, 0.0, 0.0

    # DC 移除 + 汉宁窗
    sig = sig - np.mean(sig)
    window = np.hanning(n)
    spec = np.abs(np.fft.rfft(sig * window))
    freqs = np.fft.rfftfreq(n, d=1.0 / fs)

    # 在目标频段内找峰值
    mask = (freqs >= low_hz) & (freqs <= high_hz)
    if not np.any(mask):
        return 0.0, 0.0, 0.0

    sub_spec = spec[mask]
    sub_freq = freqs[mask]
    idx = int(np.argmax(sub_spec))
    peak_hz = float(sub_freq[idx])

    # 抛物线插值: 提高 sub-bin 频率分辨率
    if 0 < idx < len(sub_spec) - 1:
        alpha, beta, gamma = sub_spec[idx-1], sub_spec[idx], sub_spec[idx+1]
        denom = (alpha - 2.0 * beta + gamma)
        if abs(denom) > 1e-12:
            p = 0.5 * (alpha - gamma) / denom  # -0.5 ~ 0.5
            peak_hz = sub_freq[idx] + p * (sub_freq[1] - sub_freq[0])

    # 带内能量 / 总能量 (粗略信噪比)
    band_energy = float(np.sum(sub_spec))
    total_energy = float(np.sum(spec)) + 1e-9
    ratio = band_energy / total_energy

    return peak_hz * 60.0, ratio, peak_hz


# ============================================================
#  峰值检测频率估计 (时域方法, 与 FFT 交叉验证)
#  —— 移植到 MCU 时非常简单: 找局部最大值
# ============================================================
def estimate_freq_peaks(sig: np.ndarray, fs: float,
                        low_hz: float, high_hz: float):
    """
    找正峰值 → 平均间隔求频率.
    返回 (bpm, 峰值数量)
    """
    n = len(sig)
    if n < 16 or np.std(sig) < 1e-6:
        return 0.0, 0

    # 最小间隔 = 最高频率的倒数
    min_interval_s = 1.0 / high_hz
    min_samples = max(1, int(min_interval_s * fs))

    try:
        peaks, _ = find_peaks(sig,
                              distance=min_samples,
                              height=np.max(sig) * 0.3,
                              prominence=(np.max(sig) - np.min(sig)) * 0.1)
    except Exception:
        return 0.0, 0

    if len(peaks) < 2:
        return 0.0, len(peaks)

    # 平均间隔
    intervals = np.diff(peaks) / fs
    # 简单异常值剔除: 去掉偏离均值 > 2σ 的
    if len(intervals) >= 3:
        mu, sigma = float(np.mean(intervals)), float(np.std(intervals))
        if sigma > 0:
            good = intervals[(intervals > mu - 2*sigma) & (intervals < mu + 2*sigma)]
            if len(good) >= 2:
                intervals = good

    mean_interval = float(np.mean(intervals))
    if mean_interval <= 0:
        return 0.0, len(peaks)

    bpm = 60.0 / mean_interval
    # 频带合法性校验: 超出范围的视为误判
    if bpm < low_hz * 60.0 or bpm > high_hz * 60.0:
        return 0.0, len(peaks)

    return bpm, len(peaks)


# ============================================================
#  选目标 bin: 在 [0.1, 2] Hz 带内振荡最强的那一个
# ============================================================
def select_target_bin(all_series: np.ndarray) -> tuple:
    """
    all_series shape = (128, n_frames)
    返回 (best_bin_index, score)
    score = 带通滤波后的标准差 × (带通能量/原始能量)
    """
    n_bins, n_frames = all_series.shape
    scores = np.zeros(n_bins)

    for b in range(BIN_MIN_INDEX, BIN_MAX_INDEX):
        sig = all_series[b]
        if len(sig) < 16 or np.std(sig) < 1e-6:
            continue
        try:
            filtered = bandpass(sig, BREATH_LOW_HZ, HEART_HIGH_HZ, FS, order=2)
            raw_std = float(np.std(sig)) + 1e-6
            filt_std = float(np.std(filtered))
            # 两个维度: 相对占比 (多少能量落在生理带) × 绝对强度
            scores[b] = (filt_std / raw_std) * filt_std
        except Exception:
            continue

    best = int(np.argmax(scores))
    if scores[best] < 1e-6:
        return BIN_MIN_INDEX, 0.0
    return best, scores[best]


# ============================================================
#  解析一行 ESP32 输出
#     例: "Frame 3: [1376, 843, ..., 2619]"
# ============================================================
_FRAME_RE = re.compile(r"Frame\s+(\d+)\s*:\s*\[([^\]]+)\]", re.IGNORECASE)


def parse_frame_line(line: str):
    """
    解析一行, 成功返回 (frame_idx, ndarray_of_6144), 失败返回 None
    """
    m = _FRAME_RE.search(line)
    if not m:
        return None

    frame_idx = int(m.group(1))
    nums_str = m.group(2)

    # 分割数字 —— 用 generator, 避免一次建立巨大 list
    try:
        values = [int(x) for x in nums_str.split(",")]
    except ValueError:
        return None

    if len(values) < FRAME_TOTAL_SAMPLES - 10:
        # 允许少量缺失, 但差太多就认为帧不完整
        return None

    # 截断 / 补齐到正好 FRAME_TOTAL_SAMPLES (6144)
    if len(values) > FRAME_TOTAL_SAMPLES:
        values = values[:FRAME_TOTAL_SAMPLES]
    elif len(values) < FRAME_TOTAL_SAMPLES:
        values = values + [0] * (FRAME_TOTAL_SAMPLES - len(values))

    return frame_idx, np.array(values, dtype=np.int32)


# ============================================================
#  把 6144 个原始 sample 压缩为 128 个距离 bin
#    原始布局: (chirp × NUM_ANTENNAS + ant) × 128 + sample_idx
#    对每个 sample_bin 在 16 chirp × 3 ant = 48 个值上取平均
# ============================================================
def reduce_to_range_bins(raw: np.ndarray) -> np.ndarray:
    """
    输入: shape (6144,) 的原始 12-bit sample (uint16 范围 0~4095)
    输出: shape (128,) 的 float32 —— 每个距离 bin 跨 chirp & 天线的均值
    """
    bins = np.zeros(NUM_SAMPLES_PER_CHIRP, dtype=np.float32)
    count = NUM_CHIRPS_PER_FRAME * NUM_RX_ANTENNAS   # 48
    for bin_idx in range(NUM_SAMPLES_PER_CHIRP):
        s = 0
        # 对每个 (chirp, ant) 组合, 取该 bin 的值
        for c in range(NUM_CHIRPS_PER_FRAME):
            for a in range(NUM_RX_ANTENNAS):
                idx = (c * NUM_RX_ANTENNAS + a) * NUM_SAMPLES_PER_CHIRP + bin_idx
                s += int(raw[idx])
        bins[bin_idx] = s / count
    # 转成有符号偏差 (减去整体直流), 便于后续时间序列分析
    return bins


# ============================================================
#  自动选择串口
# ============================================================
def auto_detect_serial():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[错误] 未检测到任何串口设备。请确认 USB 连接正常。")
        return None
    print("\n=== 可用串口列表 ===")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device} - {p.description}")
    print()
    if len(ports) == 1:
        print(f"  自动选择: {ports[0].device}\n")
        return ports[0].device
    choice = input(f"  请选择串口编号 [0-{len(ports)-1}] (回车=0): ").strip()
    try:
        idx = int(choice) if choice else 0
        return ports[idx].device
    except (ValueError, IndexError):
        return ports[0].device


# ============================================================
#  主程序: 串口接收 → 帧解析 → 信号处理 → 输出结果
# ============================================================
def main():
    print("=" * 78)
    print("  雷达呼吸/心率检测器 —— Python 上位机原型")
    print("=" * 78)
    print(f"  帧周期        : {FRAME_PERIOD_S*1000:.1f} ms  (fps ≈ {FS:.2f})")
    print(f"  每帧 samples  : {FRAME_TOTAL_SAMPLES}  (→ 压缩为 {NUM_SAMPLES_PER_CHIRP} 个距离 bin)")
    print(f"  呼吸分析窗口  : {BREATH_WINDOW_FRAMES} 帧 ≈ {BREATH_WINDOW_FRAMES*FRAME_PERIOD_S:.1f} s")
    print(f"  心跳分析窗口  : {HEART_WINDOW_FRAMES} 帧  ≈ {HEART_WINDOW_FRAMES*FRAME_PERIOD_S:.1f} s")
    print(f"  呼吸频带      : {BREATH_LOW_HZ}-{BREATH_HIGH_HZ} Hz"
          f"  ({BREATH_LOW_HZ*60:.0f}-{BREATH_HIGH_HZ*60:.0f} 次/分)")
    print(f"  心跳频带      : {HEART_LOW_HZ}-{HEART_HIGH_HZ} Hz"
          f"  ({HEART_LOW_HZ*60:.0f}-{HEART_HIGH_HZ*60:.0f} bpm)")
    print(f"  分析 bin 范围 : [{BIN_MIN_INDEX} .. {BIN_MAX_INDEX}]")
    print("=" * 78)
    print()

    # --- 打开串口 ---
    port = SERIAL_PORT or auto_detect_serial()
    if port is None:
        sys.exit(1)

    try:
        ser = serial.Serial(port, SERIAL_BAUDRATE, timeout=1.0)
    except serial.SerialException as e:
        print(f"[错误] 无法打开串口 {port}: {e}")
        sys.exit(1)

    print(f"[串口] 已连接 {port} @ {SERIAL_BAUDRATE} bps")
    print("[串口] 正在等待帧数据 (格式: 'Frame N: [v0, v1, ...]')...")
    print()

    # --- 缓冲 + MTI + 结果平滑器 ---
    buffer = RingBuffer128(max(BREATH_WINDOW_FRAMES, HEART_WINDOW_FRAMES))
    mti    = StaticMTIFilter() if MTI_ENABLE else None
    breath_smoother = OutputSmoother(OUTPUT_SMOOTH_BETA)
    heart_smoother  = OutputSmoother(OUTPUT_SMOOTH_BETA)

    total_frames   = 0
    parse_errors   = 0
    raw_lines      = 0
    last_display_t = time.time()

    # 串口循环: 按行读取, 找到 "Frame N: [...]" 就解析
    leftover = ""
    try:
        while True:
            # 读一批字节 (非阻塞, timeout 1s)
            try:
                chunk = ser.read(4096)
            except serial.SerialException:
                break
            if not chunk:
                continue

            # 字节 → 字符串 (忽略非法字符, 因为 ESP32 还会打日志)
            text = leftover + chunk.decode("utf-8", errors="ignore")
            lines = text.split("\n")
            leftover = lines[-1]   # 最后一段可能不完整, 留到下次
            lines = lines[:-1]

            for line in lines:
                raw_lines += 1
                line = line.strip()
                if not line:
                    continue
                if "Frame" not in line and "frame" not in line:
                    # 普通 ESP_LOG 日志, 跳过 (但首帧可以看一下)
                    continue

                parsed = parse_frame_line(line)
                if parsed is None:
                    parse_errors += 1
                    continue

                frame_idx, raw_values = parsed
                # 压缩 6144 → 128 个距离 bin
                bins = reduce_to_range_bins(raw_values)
                # --- [优化链 #1] 静态 MTI 背景消除 —— 抑制墙/地面/静止家具反射 ---
                if mti is not None:
                    bins = mti.apply(bins)
                # --- [优化链 #2] 距离维 3 点滑动平滑 —— 抑制距离维噪声 ---
                bins = range_smooth_3pt(bins)
                # 写进环形缓冲
                buffer.push(bins)
                total_frames += 1

            # --- 周期性显示结果 ---
            now = time.time()
            if now - last_display_t >= DISPLAY_UPDATE_S:
                last_display_t = now
                _print_result(buffer, total_frames, parse_errors, raw_lines,
                              breath_smoother, heart_smoother)

    except KeyboardInterrupt:
        print("\n\n[用户中断] 正在关闭...")
    finally:
        try:
            ser.close()
        except Exception:
            pass
        print(f"\n  统计: 原始行={raw_lines}, 成功帧={total_frames}, "
              f"解析失败={parse_errors}")
        print("已退出。")


def _print_result(buffer: RingBuffer128, total_frames, parse_errors, raw_lines,
                  breath_smoother: OutputSmoother, heart_smoother: OutputSmoother):
    """在同一行刷新显示 (用 \\r 覆盖)
       处理链: 窗口 → 时间维平滑 → 选 bin → 带通 → FFT/peak → 输出平滑
    """
    # 不够帧数就只显示缓冲进度
    if buffer.size < FFT_MIN_FRAMES:
        msg = (f"[缓冲中] 已收 {buffer.size}/{BREATH_WINDOW_FRAMES} 帧 "
               f"(总帧={total_frames}, 错误行={parse_errors})")
        sys.stdout.write("\r" + msg.ljust(78))
        sys.stdout.flush()
        return

    # 取最近窗口
    breath_series = buffer.latest(BREATH_WINDOW_FRAMES)   # (128, n1)
    heart_series  = buffer.latest(HEART_WINDOW_FRAMES)    # (128, n2)

    # --- [优化链 #3] 时间维滑动平滑 —— 让波形更连续, 峰值检测更稳 ---
    for b in range(NUM_SAMPLES_PER_CHIRP):
        breath_series[b] = time_smooth_3pt(breath_series[b])
        heart_series[b]  = time_smooth_3pt(heart_series[b])

    # 选目标 bin (用较长的呼吸窗口来选, 更稳定)
    target_bin, score = select_target_bin(breath_series)

    # 提取该 bin 的时间序列做分析
    sig_long  = breath_series[target_bin]   # 呼吸窗口
    sig_short = heart_series[target_bin]    # 心跳窗口

    # 呼吸
    sig_b = bandpass(sig_long - np.mean(sig_long),
                     BREATH_LOW_HZ, BREATH_HIGH_HZ, FS, order=2)
    b_bpm_fft,  b_ratio,  b_hz = estimate_freq_fft(sig_b, FS, BREATH_LOW_HZ, BREATH_HIGH_HZ)
    b_bpm_peak, b_npeaks         = estimate_freq_peaks(sig_b, FS, BREATH_LOW_HZ, BREATH_HIGH_HZ)

    # 心跳
    sig_h = bandpass(sig_short - np.mean(sig_short),
                     HEART_LOW_HZ, HEART_HIGH_HZ, FS, order=2)
    h_bpm_fft,  h_ratio,  h_hz = estimate_freq_fft(sig_h, FS, HEART_LOW_HZ, HEART_HIGH_HZ)
    h_bpm_peak, h_npeaks         = estimate_freq_peaks(sig_h, FS, HEART_LOW_HZ, HEART_HIGH_HZ)

    # 合并: FFT 和 peak 两种方法都有效时取平均, 否则用有效的那个
    def merge(a, b):
        if a > 0 and b > 0:
            return 0.5 * (a + b)
        return a if a > 0 else b

    breath_bpm_raw = merge(b_bpm_fft, b_bpm_peak)
    heart_bpm_raw  = merge(h_bpm_fft, h_bpm_peak)

    # --- [优化链 #4] 结果指数平滑 —— 输出不跳来跳去 ---
    breath_bpm = breath_smoother.apply(breath_bpm_raw)
    heart_bpm  = heart_smoother.apply(heart_bpm_raw)

    # 显示
    if breath_bpm > 0:
        breath_str = f"{breath_bpm:5.1f} 次/分"
    else:
        breath_str = "  --   "

    if heart_bpm > 0:
        heart_str = f"{heart_bpm:5.1f} bpm"
    else:
        heart_str = "  --  "

    mti_flag = "MTI" if MTI_ENABLE else "---"
    rng_flag = "RNG" if RANGE_SMOOTH_ENABLE else "---"
    tim_flag = "TIM" if TIME_SMOOTH_ENABLE else "---"

    msg = (f"[{mti_flag}|{rng_flag}|{tim_flag}] bin={target_bin:3d} | "
           f"呼吸: {breath_str} (FFT={b_bpm_fft:5.1f} pk={b_bpm_peak:5.1f} E={b_ratio:4.2f}) | "
           f"心跳: {heart_str} (FFT={h_bpm_fft:5.1f} pk={h_bpm_peak:5.1f} E={h_ratio:4.2f}) | "
           f"帧={buffer.size:3d}/{total_frames}")

    sys.stdout.write("\r" + msg.ljust(180))
    sys.stdout.flush()


if __name__ == "__main__":
    main()
