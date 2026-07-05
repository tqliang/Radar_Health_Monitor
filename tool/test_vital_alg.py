#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
算法验证脚本 —— 用模拟数据验证 C 实现的算法是否能正确估计呼吸/心率
运行: python3 tool/test_vital_alg.py
"""

import numpy as np
import sys

# -------- 模拟参数 (与 MCU 硬件保持一致) --------
FRAME_PERIOD_S = 0.199634
FS = 1.0 / FRAME_PERIOD_S          # ~5.009 Hz
N_BREATH = 120                      # ~24 秒
N_HEART = 80                        # ~16 秒
TOTAL_FRAMES = 200

# 目标值 (测试用)
BREATH_BPM_TARGET = 15.0           # 15 次/分
HEART_BPM_TARGET = 75.0            # 75 次/分
BREATH_FREQ = BREATH_BPM_TARGET / 60.0
HEART_FREQ = HEART_BPM_TARGET / 60.0

print(f"目标呼吸: {BREATH_BPM_TARGET:.1f} bpm ({BREATH_FREQ*60:.1f} bpm / {BREATH_FREQ:.3f} Hz)")
print(f"目标心跳: {HEART_BPM_TARGET:.1f} bpm ({HEART_FREQ*60:.1f} bpm / {HEART_FREQ:.3f} Hz)")
print(f"采样率: {FS:.3f} Hz, 呼吸窗口: {N_BREATH} 帧, 心跳窗口: {N_HEART} 帧")
print()

# -------- 构造模拟信号 --------
# 假设: bin 42 是人体所在的 bin, 其他 bin 是噪声
N_BINS = 128
SIGNAL_BIN = 42
np.random.seed(42)

# 模拟雷达回波:
#   - 呼吸: 较大幅度 (~150 ADC units, 0.25 Hz)
#   - 心跳: 较小幅度 (~30 ADC units, 1.25 Hz)
#   - 噪声: ~50 ADC units
#   - 直流: 2000
t = np.arange(TOTAL_FRAMES) * FRAME_PERIOD_S

# 目标 bin 的信号 (模拟 MTI 后只剩交流成分)
sig_breath = 150.0 * np.sin(2 * np.pi * BREATH_FREQ * t + 0.3)
sig_heart  = 30.0  * np.sin(2 * np.pi * HEART_FREQ  * t + 1.2)
# 心跳叠加呼吸谐波 (模拟真实情况)
sig_heart += 8.0 * np.sin(2 * np.pi * HEART_FREQ * 2 * t)
noise = np.random.randn(TOTAL_FRAMES) * 20.0

signal_bin = sig_breath + sig_heart + noise

# 其他 bin: 纯噪声 (但静态均值较强, 模拟墙/地板)
all_bins = np.zeros((N_BINS, TOTAL_FRAMES))
for b in range(N_BINS):
    if b == SIGNAL_BIN:
        all_bins[b] = signal_bin
    else:
        # 静态 + 少量噪声
        all_bins[b] = (2000.0 + np.random.randn() * 50) + np.random.randn(TOTAL_FRAMES) * 5.0


print("=== 各估计器的表现 (用 SIGNAL_BIN 的数据) ===")
sig_test = signal_bin - np.mean(signal_bin)

# --- FFT 估计 (用于呼吸) ---
from scipy.signal import butter, filtfilt

def estimate_fft(sig, fs, low_hz, high_hz):
    n = len(sig)
    window = np.hanning(n)
    spec = np.abs(np.fft.rfft(sig * window))
    freqs = np.fft.rfftfreq(n, d=1.0/fs)
    mask = (freqs >= low_hz) & (freqs <= high_hz)
    sub_spec = spec[mask]
    sub_freq = freqs[mask]
    idx = int(np.argmax(sub_spec))
    peak_hz = float(sub_freq[idx])
    return peak_hz * 60.0

# --- 自相关估计 ---
def estimate_autocorr(sig, fs, low_hz, high_hz):
    n = len(sig)
    mean = np.mean(sig)
    sig0 = sig - mean
    R0 = np.sum(sig0**2)
    lag_min = int(low_hz * 60.0 * n / 60.0 / (fs))  # 最小 lag = fs/high_hz
    lag_min = int(fs / high_hz)
    lag_max = int(fs / low_hz)
    if lag_max >= n // 2: lag_max = n // 2

    R = np.zeros(lag_max + 1)
    for k in range(lag_min, lag_max + 1):
        R[k] = np.sum(sig0[:n-k] * sig0[k:]) / (n - k) / R0

    # 找正峰
    best_k = -1
    best_r = -1e30
    for k in range(lag_min + 1, lag_max):
        if R[k] > R[k-1] and R[k] >= R[k+1] and R[k] > 0.15:
            if R[k] > best_r:
                best_r = R[k]
                best_k = k

    if best_k < 0:
        return 0.0
    peak_lag = float(best_k)
    # 抛物线插值
    if best_k > lag_min and best_k < lag_max:
        alpha, beta, gamma = R[best_k-1], R[best_k], R[best_k+1]
        denom = alpha - 2*beta + gamma
        if abs(denom) > 1e-10:
            peak_lag = float(best_k) + 0.5 * (alpha - gamma) / denom
    return 60.0 * fs / peak_lag

# --- 峰值检测估计 ---
def estimate_peaks(sig, fs, low_hz, high_hz):
    from scipy.signal import find_peaks
    n = len(sig)
    if n < 20 or np.std(sig) < 1e-6:
        return 0.0
    sig0 = sig - np.mean(sig)
    min_distance = int(fs / high_hz)
    threshold = (np.max(sig0) - np.min(sig0)) * 0.10

    # 简单: 找正峰
    peaks, _ = find_peaks(sig0, distance=min_distance, height=threshold)
    if len(peaks) < 3:
        return 0.0
    intervals = np.diff(peaks) / fs
    # 中间 60%
    intervals_sorted = np.sort(intervals)
    lo = len(intervals_sorted) // 5
    hi = len(intervals_sorted) - lo - 1
    if hi <= lo:
        return 0.0
    avg_int = np.mean(intervals_sorted[lo:hi+1])
    bpm = 60.0 / avg_int
    if bpm < low_hz * 60 or bpm > high_hz * 60:
        return 0.0
    return bpm


# --- 带通滤波器 (模拟 filtfilt) ---
def bandpass(sig, low, high, fs, order=2):
    b, a = butter(order, [low, high], btype="band", fs=fs)
    return filtfilt(b, a, sig)

def highpass(sig, fc, fs, order=2):
    b, a = butter(order, fc, btype="high", fs=fs)
    return filtfilt(b, a, sig)


# -------- 呼吸估计 --------
sig_b = bandpass(sig_test, 0.10, 0.50, FS)
b_fft = estimate_fft(sig_b, FS, 0.10, 0.50)
b_ac  = estimate_autocorr(sig_b, FS, 0.10, 0.50)
b_pk  = estimate_peaks(sig_b, FS, 0.10, 0.50)
print(f"呼吸: FFT={b_fft:.1f} bpm, AC={b_ac:.1f} bpm, Peak={b_pk:.1f} bpm (target={BREATH_BPM_TARGET:.1f})")

# -------- 心跳估计 (先除呼吸) --------
sig_h = highpass(sig_test, 0.70, FS)
sig_h = bandpass(sig_h, 0.80, 2.50, FS)
h_fft = estimate_fft(sig_h, FS, 0.80, 2.50)
h_ac  = estimate_autocorr(sig_h, FS, 0.80, 2.50)
h_pk  = estimate_peaks(sig_h, FS, 0.80, 2.50)
print(f"心跳: FFT={h_fft:.1f} bpm, AC={h_ac:.1f} bpm, Peak={h_pk:.1f} bpm (target={HEART_BPM_TARGET:.1f})")

# -------- 直接估计 (不除呼吸) 做对比 --------
sig_h_raw = bandpass(sig_test, 0.80, 2.50, FS)
hr_fft_raw = estimate_fft(sig_h_raw, FS, 0.80, 2.50)
hr_ac_raw  = estimate_autocorr(sig_h_raw, FS, 0.80, 2.50)
print(f"心跳(不除呼吸对比): FFT={hr_fft_raw:.1f}, AC={hr_ac_raw:.1f}")

# -------- bin 选择验证 --------
print("\n=== bin 选择 (能量+带通能量) ===")
scores = []
for b in range(5, 80):
    s = all_bins[b] - np.mean(all_bins[b])
    bp = bandpass(s, 0.10, 2.50, FS)
    std_raw = np.std(s)
    std_bp = np.std(bp)
    ratio = std_bp / (std_raw + 1e-9)
    score = std_bp * (0.5 + ratio)
    scores.append((b, score))

scores.sort(key=lambda x: -x[1])
print("Top-5 bins (score):")
for i, (b, s) in enumerate(scores[:5]):
    is_target = "  <-- 人体 bin" if b == SIGNAL_BIN else ""
    print(f"  {i+1}. bin={b:3d}, score={s:.2f}{is_target}")
print(f"目标 bin={SIGNAL_BIN}, 被选为 Top-{[i+1 for i,(b,s) in enumerate(scores) if b==SIGNAL_BIN][0]}")

# -------- 频谱分析 (调试) --------
print("\n=== 频域分析 (FFT 幅度) ===")
import numpy.fft as fft
sig_fft = np.abs(fft.rfft(sig_test - np.mean(sig_test)))
freqs = fft.rfftfreq(len(sig_test), d=1.0/FS)

# 找几个显著峰
mask_breath = (freqs >= 0.1) & (freqs <= 0.5)
mask_heart = (freqs >= 0.8) & (freqs <= 2.5)
idx_b = np.argmax(sig_fft[mask_breath])
idx_h = np.argmax(sig_fft[mask_heart])
freq_breath = freqs[mask_breath][idx_b]
freq_heart = freqs[mask_heart][idx_h]
print(f"最大呼吸频带峰: {freq_breath*60:.1f} bpm ({freq_breath:.3f} Hz)")
print(f"最大心跳频带峰: {freq_heart*60:.1f} bpm ({freq_heart:.3f} Hz)")

# 谐波检查
print(f"\n呼吸基频谐波: 2x={freq_breath*2*60:.1f} bpm, 3x={freq_breath*3*60:.1f} bpm, 4x={freq_breath*4*60:.1f} bpm")
print(f"注意: 呼吸的 4x 谐波 = {freq_breath*4*60:.1f} bpm, 这正是真实心跳附近, 容易串扰!")

print("\n=== 测试完成 ===")