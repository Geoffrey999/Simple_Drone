"""黑匣子数据分析：针对"左偏"问题的判读。

设计目标不是输出一堆数字，而是直接给出**可判读的结论**：

  1. roll 是否随时间单调漂移？漂移速率多少？        -> 区分"发散"与"固定偏置"
  2. 陀螺零偏(gyro_y 的均值)在飞行中是否变化？       -> 定位"零偏漂移"这条链路
  3. 加速度计合成幅值是否长期偏离 1g？               -> 定位"振动污染重力参考"
  4. accel_confidence 是否掉下来？                   -> 直接看重力修正有没有被关掉
  5. sample_period 是否稳定？                        -> 验证 dt 修复是否生效
  6. 四路电机的差动是否持续朝一侧？                  -> 看控制器在往哪边推

用法：
    python bb_analyze.py <flash镜像.bin> [--plot] [--csv out.csv]
"""

from __future__ import annotations

import argparse
import sys

# --- 控制台编码安全网 -------------------------------------------------------
# 中文 Windows 控制台是 GBK(cp936)。单独运行本脚本（不经 bb.py）时，
# 打印任何 GBK 无法表示的字符都会抛 UnicodeEncodeError，表现为"直接闪退"。
# errors="replace" 让不支持的字符降级成 '?'，绝不崩溃。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")     # type: ignore[attr-defined]
    except Exception:
        pass

try:
    import numpy as np
except ImportError:                                # pragma: no cover
    print("[!] 缺少 numpy，无法分析。请先安装：pip install numpy")
    raise SystemExit(1)

from bb_format import (DIAG_DIV, FRAMES_PER_SECTOR, TYPE_DATA, TYPE_DIAG,
                       TYPE_RC, TYPE_ALT, RC_FMT_TARGET, parse_flash)

# 阈值（判读用，可按机型调整）
DRIFT_RATE_NOTICE = 0.05      # deg/s，超过就提示存在单调漂移
DRIFT_RATE_WARN = 0.20        # deg/s，明显发散
BIAS_DRIFT_NOTICE = 0.20      # deg/s，陀螺偏置变化
ACCEL_DEV_NOTICE = 0.05       # g，合成幅值偏离 1g


def _slope(x: np.ndarray, y: np.ndarray) -> float:
    """最小二乘斜率（y 对 x）。"""
    if x.size < 3:
        return float("nan")
    xm = x - x.mean()
    denom = float((xm * xm).sum())
    if denom <= 0.0:
        return float("nan")
    return float((xm * (y - y.mean())).sum() / denom)


def _rolling_mean(y: np.ndarray, win: int) -> np.ndarray:
    if y.size < win or win < 2:
        return y
    k = np.ones(win) / win
    return np.convolve(y, k, mode="valid")


def analyze(raw: bytes, verbose: bool = True, run_index: int = -1,
            trim_roll: float = 0.0, trim_pitch: float = 0.0):
    """返回 (log, run, report 字典)。

    run_index 选择分析哪一段记录（0 = 最新）。默认 -1 表示自动：
    优先选"最新且时长足够"的一段——因为落地后短暂解锁也会写一小段，
    如果只看最新一段，可能只拿到 0.1 秒，误以为数据丢了。

    trim_roll / trim_pitch：把这两个量从"实测姿态"和"设定值"里一起减掉。
    固件把 ctrl.roll_offset_deg / ctrl.pitch_offset_deg 直接加在目标角上，
    所以摇杆回中时目标角是 offset 而不是 0；想让曲线以"回中点 = 0"显示就传它们。
    只影响显示/统计，不改磁盘上的原始文件。
    """
    log = parse_flash(raw)

    # ── trim 归零（实测与设定值一起平移，保持"设定值 vs 实测"的关系不变）──
    if trim_roll or trim_pitch:
        log.angle[:, 0] -= np.float32(trim_roll)
        log.angle[:, 1] -= np.float32(trim_pitch)
        log.rc[:, 4] -= np.float32(trim_roll)      # 摇杆帧里的目标 roll
        log.rc[:, 5] -= np.float32(trim_pitch)     # 摇杆帧里的目标 pitch
        if verbose:
            print(f"[trim 归零] 实测与设定值都减去 roll {trim_roll:+.3f}°, "
                  f"pitch {trim_pitch:+.3f}°（曲线以回中点 = 0 显示）\n")

    segments = log.runs()

    rep: dict = {"frames_total": int(log.seq.size), "run_frames": 0,
                 "runs": [int(s.size) for s in segments]}

    # ── 存储概览：无论几段都打印，用来判断"数据到底还在不在 Flash 里" ──
    if verbose:
        n_blank = int(np.count_nonzero(~log.blank))
        secs_with_data = 0
        for h in log.headers:
            if not h.valid:
                continue
            lo = h.phys_sector * FRAMES_PER_SECTOR
            hi = lo + FRAMES_PER_SECTOR
            if not log.blank[lo:hi].all():
                secs_with_data += 1
        print(f"存储概览: 共 {log.blank.size} 个帧槽位，"
              f"其中非空帧 {n_blank}（{100.0*n_blank/max(1,log.blank.size):.1f}%），"
              f"有数据的扇区 {secs_with_data} 个")
        print(f"          相当于最长可能 {n_blank/1000.0:.1f} 秒的记录量\n")

    if not segments:
        rep["error"] = "没有找到任何有效飞行数据"
        if verbose:
            print(log.describe_headers())
            print(rep["error"])
        return log, np.array([], dtype=int), rep

    # ── 选数据 ──
    #   默认：整片全部非空帧（按写入顺序）—— 最可靠，不会因为分段判据出错而丢数据。
    #   --run K：只看第 K 段（靠 seq 连续性分段，仅在确实有多段时使用）。
    if run_index >= 0:
        if run_index >= len(segments):
            rep["error"] = f"只有 {len(segments)} 段，没有第 {run_index} 段"
            if verbose:
                print(rep["error"])
            return log, np.array([], dtype=int), rep
        run = segments[run_index]
        if verbose:
            print(f"已选择第 {run_index} 段：{run.size} 帧\n")
    else:
        run = log.all_frames()
        if verbose:
            print(f"使用整片全部数据：{run.size} 帧（≈ {run.size/1000.0:.1f} 秒）")
            if len(segments) > 1:
                print(f"  注：按 seq 连续性还可分出 {len(segments)} 段，"
                      f"想看其中某一段用 --list-runs 查看后再 --run K")
            print()

    if verbose and run_index >= 0:
        print(f"整片里共有 {len(segments)} 段记录（从新到旧）：")
        for k, s in enumerate(segments):
            sec_lo = int(log.sector_of[s[0]])
            sec_hi = int(log.sector_of[s[-1]])
            print(f"  第{k}段: {s.size:6d} 帧 ≈ {s.size/1000.0:6.1f} s   "
                  f"物理扇区 {sec_lo}~{sec_hi}")
        print()

    # ★ 剔除遥控摇杆帧：它的 int16 槽位含义与数据帧不同
    #   （里面是摇杆/arm/mode 而不是姿态/陀螺），参与统计会造出假数据。
    #   bb_format 已不再为摇杆帧写入姿态/陀螺数组，这里再剔一遍，
    #   保证均值/标准差不会被 NaN 或摇杆值污染。
    #
    #   ⚠ 注意保留 run_all：摇杆数据要从**未剔除**的集合里取，
    #   否则"先把摇杆帧删掉、再去里面数摇杆帧"，会永远得到 0 个，
    #   误报"固件是旧版未记录摇杆"。
    run_all = run
    run = run[(log.ftype[run] != TYPE_RC) & (log.ftype[run] != TYPE_ALT)]
    rep["run_frames"] = int(run.size)

    if run.size == 0:
        rep["error"] = "没有找到任何有效飞行数据"
        if verbose:
            print(log.describe_headers())
            print(rep["error"])
        return log, run, rep

    # ---------------------------------------------------------- 基本量
    t = np.arange(run.size) / 1000.0            # 名义时间(s)，1kHz 记录
    roll = log.roll[run]
    pitch = log.pitch[run]
    yaw = log.yaw[run]
    gyro = log.gyro[run]
    accel = log.accel[run]
    anorm = np.linalg.norm(accel, axis=1)
    motor = log.motor[run]
    throttle = log.throttle[run]
    diag_conf = log.diag_conf[run]
    diag_period = log.diag_period[run]
    is_data = log.ftype[run] == TYPE_DATA

    # 只对普通数据帧做统计（诊断帧的序号槽被复用，其它字段仍有效）
    rep["duration_s"] = float(run.size / 1000.0)
    rep["data_frames"] = int(is_data.sum())

    # ---------------------------------------------------------- 1. 姿态漂移
    rep["roll_start"] = float(roll[:200].mean()) if roll.size >= 200 else float(roll[0])
    rep["roll_end"] = float(roll[-200:].mean()) if roll.size >= 200 else float(roll[-1])
    rep["roll_range"] = (float(roll.min()), float(roll.max()))
    rep["roll_drift_rate"] = _slope(t, roll)          # deg/s
    rep["roll_drift_total"] = float(rep["roll_end"] - rep["roll_start"])
    # 全程均值：用来和“固件内算的加计倾角”相减，得到真实的姿态估计误差
    rep["roll_mean"] = float(np.nanmean(roll))
    rep["pitch_mean"] = float(np.nanmean(pitch))
    rep["pitch_drift_rate"] = _slope(t, pitch)
    rep["yaw_drift_rate"] = _slope(t, yaw)

    # 单调性：roll 与时间的相关系数
    if roll.size > 10:
        cc = np.corrcoef(t, roll)[0, 1]
        rep["roll_time_corr"] = float(cc) if np.isfinite(cc) else float("nan")
    else:
        rep["roll_time_corr"] = float("nan")

    # ---------------------------------------------------------- 2. 陀螺零偏
    # 用滑动均值看零偏的低频分量是否在漂
    win = max(50, min(2000, run.size // 20))
    gyro_y_lp = _rolling_mean(gyro[:, 1], win)
    rep["gyro_y_mean"] = float(gyro[:, 1].mean())
    rep["gyro_y_lp_start"] = float(gyro_y_lp[:50].mean()) if gyro_y_lp.size > 50 else float("nan")
    rep["gyro_y_lp_end"] = float(gyro_y_lp[-50:].mean()) if gyro_y_lp.size > 50 else float("nan")
    rep["gyro_y_drift"] = float(rep["gyro_y_lp_end"] - rep["gyro_y_lp_start"])
    rep["gyro_x_drift"] = float(
        (_rolling_mean(gyro[:, 0], win)[-50:].mean() if gyro.shape[0] > 50 else np.nan)
        - (_rolling_mean(gyro[:, 0], win)[:50].mean() if gyro.shape[0] > 50 else np.nan))

    # ---------------------------------------------------------- 3. 重力参考
    rep["accel_norm_mean"] = float(anorm.mean())
    rep["accel_norm_std"] = float(anorm.std())
    rep["accel_dev_mean"] = float(np.abs(anorm - 1.0).mean())
    rep["accel_dev_p95"] = float(np.percentile(np.abs(anorm - 1.0), 95))
    rep["accel_z_mean"] = float(accel[:, 2].mean())

    # 逐轴均值/标准差 + 交流分量(去均值后的 RMS)
    # ★ 这两个数字必须分开看，才能区分两种完全不同的毛病：
    #   · 均值明显偏离(如 ax=+0.5g)  -> 零偏/标定错误，重力方向本身就是错的
    #   · 均值正常但 AC-RMS 很大      -> 真实振动，高频噪声把 |a| 抬起来
    for k, axis in enumerate("xyz"):
        col = accel[:, k]
        col = col[np.isfinite(col)]
        if col.size:
            mu = float(col.mean())
            rep[f"accel_{axis}_mean"] = mu
            rep[f"accel_{axis}_std"] = float(col.std())
            rep[f"accel_{axis}_ac"] = float(np.sqrt(np.mean((col - mu) ** 2)))
        else:
            rep[f"accel_{axis}_mean"] = rep[f"accel_{axis}_std"] = float("nan")
            rep[f"accel_{axis}_ac"] = float("nan")

    # ---------------------------------------------------------- 4. 置信度 / 周期
    dvalid = np.isfinite(diag_conf)
    rep["diag_frames"] = int(dvalid.sum())
    if dvalid.any():
        rep["conf_min"] = float(np.nanmin(diag_conf))
        rep["conf_mean"] = float(np.nanmean(diag_conf))
        rep["conf_low_ratio"] = float((diag_conf[dvalid] < 0.75).mean())
    else:
        rep["conf_min"] = rep["conf_mean"] = rep["conf_low_ratio"] = float("nan")

    pvalid = np.isfinite(diag_period)
    if pvalid.any():
        rep["period_mean_us"] = float(np.nanmean(diag_period))
        rep["period_min_us"] = float(np.nanmin(diag_period))
        rep["period_max_us"] = float(np.nanmax(diag_period))
    else:
        rep["period_mean_us"] = rep["period_min_us"] = rep["period_max_us"] = float("nan")

    # ---------------------------------------------------------- 5. 电机差动
    # m1/m2 为左侧，m3/m4 为右侧（见 flycontrol.c 混控）
    left = (motor[:, 0] + motor[:, 1]) * 0.5
    right = (motor[:, 2] + motor[:, 3]) * 0.5
    rep["motor_left_mean"] = float(left.mean())
    rep["motor_right_mean"] = float(right.mean())
    rep["motor_diff_mean"] = float((left - right).mean())
    rep["motor_diff_drift"] = _slope(t, left - right)
    # 单路均值：机架/电机不对称会直接吃掉混控余量（见 flycontrol.c 的混控抗饱和）
    rep["motor_means"] = [float(motor[:, k].mean()) for k in range(4)]
    # 混控抗饱和（flags bit2，2026-10-24 之后的固件才置位）
    # 老固件这一位永远是 0 —— 那种情况不显示，免得误报"0% 饱和"
    _sat = (log.flags[run] & 0x04) != 0
    rep["mix_sat_ratio"] = float(_sat.sum() / is_data.sum()) if is_data.any() else None
    if (rep["mix_sat_ratio"] == 0.0) and (not np.any(_sat)):
        rep["mix_sat_ratio"] = None
    # ---------------------------------------------------------- 5b. 频谱（振动/振荡）
    # 用 FFT 分成两个带：
    #   10~30Hz  -> 内环自激振荡（应尽量小）
    #   >100Hz   -> 电机/桨的结构振动残留（受 IMU 内部带宽与陷波影响）
    def _band(x, lo, hi):
        v = x[np.isfinite(x)]
        if v.size < 256:
            return float("nan"), float("nan")
        v = v - v.mean()
        spec = np.abs(np.fft.rfft(v * np.hanning(v.size)))
        fr = np.fft.rfftfreq(v.size, 1.0 / 1000.0)
        m = (fr >= lo) & (fr <= hi)
        if not m.any():
            return float("nan"), float("nan")
        rms = float(np.sqrt(np.mean(np.fft.irfft(
            np.where(m, np.fft.rfft(v), 0), v.size) ** 2)))
        return rms, float(fr[m][int(np.argmax(spec[m]))])

    rep["osc_rms"],   rep["osc_peak"]   = _band(gyro[:, 1], 10.0, 30.0)
    rep["vib_rms"],   rep["vib_peak"]   = _band(gyro[:, 1], 100.0, 480.0)
    rep["vibx_rms"],  _                 = _band(gyro[:, 0], 100.0, 480.0)
    rep["accl_vib"],  _                 = _band(anorm,    100.0, 480.0)

    rep["throttle_mean"] = float(throttle.mean())
    rep["battery_first"] = int(log.battery[run][0])
    rep["battery_last"] = int(log.battery[run][-1])

    # ---------------------------------------------------------- 6. 遥控摇杆
    # 摇杆帧稀少（每 32ms 一帧），单独取。
    # 目的：区分"操作者在打杆修正"与"操作者回中但控制器把飞机推走"。
    rcmask = np.isfinite(log.rc[run_all, 0])
    if rcmask.sum() >= 5:
        rc = log.rc[run_all][rcmask]             # (N, 6)
        rep["rc_n"] = int(rcmask.sum())
        rep["rc_roll_mean"] = float(rc[:, 0].mean())
        rep["rc_pitch_mean"] = float(rc[:, 1].mean())
        rep["rc_thr_mean"] = float(rc[:, 3].mean())
        rep["rc_roll_absmean"] = float(np.abs(rc[:, 0] - 1500.0).mean())
        rep["rc_pitch_absmean"] = float(np.abs(rc[:, 1] - 1500.0).mean())
        rep["acc_roll"]  = float(rc[:, 4].mean())
        rep["acc_pitch"] = float(rc[:, 5].mean())
        # 摇杆回中程度：|roll-1500| < 40 视为基本回中
        rep["rc_roll_center_ratio"] = float(
            (np.abs(rc[:, 0] - 1500.0) < 40.0).mean())
        rep["rc_pitch_center_ratio"] = float(
            (np.abs(rc[:, 1] - 1500.0) < 40.0).mean())
    else:
        rep["rc_n"] = int(rcmask.sum())

    # ---------------------------------------------------------- 6. 丢帧
    # 不能用序号做判据：诊断帧的序号槽存的是 conf/period。
    # 用"数据帧之间的索引间隔"判断更可靠：无丢帧时间隔恒为 DIAG_DIV。
    data_pos = np.flatnonzero(is_data)
    if data_pos.size > 2:
        d = np.diff(data_pos)
        over = d[d > DIAG_DIV]
        rep["gaps"] = int(over.size)
        rep["frames_lost"] = int((over - DIAG_DIV).sum())
    else:
        rep["gaps"] = 0
        rep["frames_lost"] = 0

    # ---------------------------------------------------------- 7. 高度估计
    # 高度帧(0x04)按 50Hz 记的是融合器内部量。这里只统计 baro_ok=1 的样本，
    # 否则会把"气压计没起来"那段的冻结值也算进去。
    alt_s = _alt_series(log, run)
    rep["alt_present"] = alt_s is not None
    if alt_s is not None:
        ok = (alt_s["status"] & 0x01) != 0
        rep["alt_n"] = alt_s["n_frames"]
        rep["alt_ok_ratio"] = float(ok.mean()) if ok.size else 0.0
        rep["alt_clamped_ratio"] = float(((alt_s["status"] & 0x02) != 0).mean())
        rep["alt_gain_rejected"] = bool(((alt_s["status"] & 0x04) != 0).any())
        if ok.sum() >= 2:
            baro = alt_s["baro"][ok]
            fused = alt_s["alt"][ok]
            vvel = alt_s["vvel"][ok]
            innov = alt_s["innov"][ok]
            bias = alt_s["bias"][ok]
            rep["alt_baro_range"] = (float(np.nanmin(baro)), float(np.nanmax(baro)))
            rep["alt_range"] = (float(np.nanmin(fused)), float(np.nanmax(fused)))
            rep["alt_track_err"] = float(np.nanmean(np.abs(fused - baro)))
            rep["alt_track_err_max"] = float(np.nanmax(np.abs(fused - baro)))
            rep["alt_vvel_rms"] = float(np.nanstd(vvel))
            # 稳态段（后半段）：报告里的整体 RMS 会被起飞/切入那几秒主导，
            # 光看整体数会误判"一直在抖"。稳态 RMS 才代表定高真正的工作质量。
            idx_ok = np.flatnonzero(ok)
            if idx_ok.size >= 40:
                steady = np.zeros_like(ok)
                steady[idx_ok[idx_ok.size // 2:]] = True
                rep["alt_steady_vvel_rms"] = float(np.nanstd(vvel[steady]))
                rep["alt_steady_innov_rms"] = float(np.nanstd(innov[steady]))
                rep["alt_steady_err_max"] = float(np.nanmax(np.abs(fused[steady] - baro[steady])))
            rep["alt_vvel_max"] = float(np.nanmax(np.abs(vvel)))
            rep["alt_innov_absmean"] = float(np.nanmean(np.abs(innov)))
            rep["alt_innov_p95"] = float(np.nanpercentile(np.abs(innov), 95))
            rep["alt_innov_max"] = float(np.nanmax(np.abs(innov)))
            rep["alt_thr_median"] = float(np.nanmedian(alt_s["thr"][ok]))
            # 定高修正量贴着 ±600 限幅的比例：环路"满舵打摆"的直接证据
            rep["alt_sat_ratio"] = float(np.mean(np.abs(alt_s["thr"][ok]) >= 599.0))
            # 悬停油门学习：学到的基准油门 & 有效比例
            hv = alt_s["hover_valid"][ok] != 0
            rep["alt_hover_valid_ratio"] = float(hv.mean()) if hv.size else 0.0
            if hv.any():
                hb = alt_s["hover"][ok][hv]
                rep["alt_hover_range"] = (float(np.nanmin(hb)), float(np.nanmax(hb)))
                rep["alt_hover_last"] = float(hb[-1])
            rep["alt_bias_first"] = float(bias[0])
            rep["alt_bias_last"] = float(bias[-1])
            rep["alt_dt_median_ms"] = float(np.nanmedian(alt_s["baro_dt"][ok]))
            rep["alt_thr_range"] = (float(np.nanmin(alt_s["thr"][ok])),
                                    float(np.nanmax(alt_s["thr"][ok])))
            rep["alt_press_range"] = (float(np.nanmin(alt_s["press"][ok])),
                                      float(np.nanmax(alt_s["press"][ok])))

        # 原始 50Hz 样本（不走 1kHz ZOH 网格）——用来评估气压计本身的噪声，
        # 以及检测"读到跟不上转换"造成的重复样本。
        allf = log.all_frames()
        ai = allf[log.ftype[allf] == TYPE_ALT]
        if ai.size >= 10:
            raw_baro = np.asarray(log.alt[ai, 0], dtype=np.float64)
            steps = np.diff(raw_baro)
            rep["alt_samples"] = int(ai.size)
            rep["alt_baro_std"] = float(np.nanstd(raw_baro))
            rep["alt_dup_ratio"] = float(np.mean(steps == 0.0))
            nz = steps[steps != 0.0]
            rep["alt_baro_step"] = float(np.nanstd(nz)) if nz.size > 2 else float("nan")
        else:
            rep["alt_samples"] = int(ai.size)

    if verbose:
        _print_report(rep, log)
    return log, run, rep


def _print_report(rep: dict, log) -> None:
    print("=" * 72)
    print("黑匣子分析报告")
    print("=" * 72)
    print(log.describe_headers())
    print()

    if rep.get("error"):
        print(rep["error"])
        return

    print(f"记录长度      : {rep['duration_s']:.1f} s "
          f"({rep['run_frames']} 帧, 其中数据帧 {rep['data_frames']})")
    print(f"丢帧          : {rep['frames_lost']} 帧 ({rep['gaps']} 处断点)")
    print(f"电量          : {rep['battery_first']}% -> {rep['battery_last']}%")
    print(f"油门(均值)    : {rep['throttle_mean']:.0f} / 4200")
    print()

    print("-- 1. 姿态漂移 --------------------------------------------------")
    print(f"  roll 起/止  : {rep['roll_start']:+.2f}deg -> {rep['roll_end']:+.2f}deg  "
          f"(净变化 {rep['roll_drift_total']:+.2f}deg)")
    print(f"  roll 范围   : {rep['roll_range'][0]:+.2f}deg ~ {rep['roll_range'][1]:+.2f}deg")
    print(f"  roll 漂移率 : {rep['roll_drift_rate']:+.4f} deg/s   "
          f"(与时间相关系数 {rep['roll_time_corr']:+.3f})")
    print(f"  pitch 漂移率: {rep['pitch_drift_rate']:+.4f} deg/s")
    print(f"  yaw   漂移率: {rep['yaw_drift_rate']:+.4f} deg/s")
    print()

    print("-- 2. 陀螺零偏 --------------------------------------------------")
    print(f"  gyro_y 均值 : {rep['gyro_y_mean']:+.4f} deg/s")
    print(f"  gyro_y 低频 : {rep['gyro_y_lp_start']:+.4f} -> "
          f"{rep['gyro_y_lp_end']:+.4f} deg/s  (变化 {rep['gyro_y_drift']:+.4f})")
    print(f"  gyro_x 低频变化: {rep['gyro_x_drift']:+.4f} deg/s")
    print()

    print("-- 3. 重力参考(加速度计) ---------------------------------------")
    print(f"  |a| 均值/标准差: {rep['accel_norm_mean']:.4f} / {rep['accel_norm_std']:.4f} g")
    print(f"  ||a|-1g| 均值  : {rep['accel_dev_mean']:.4f} g  "
          f"(95 分位 {rep['accel_dev_p95']:.4f})")
    print(f"  --- 逐轴分解（均值=零偏/标定；AC=振动强度）---")
    for axis in "xyz":
        print(f"  accel_{axis}: 均值 {rep[f'accel_{axis}_mean']:+.4f} g   "
              f"标准差 {rep[f'accel_{axis}_std']:.4f} g   "
              f"AC-RMS {rep[f'accel_{axis}_ac']:.4f} g")
    # 水平轴 AC-RMS：悬停时该值应当很小；超过 0.1g 基本可以确认有明显振动
    hor_ac = float(np.nanmean([rep["accel_x_ac"], rep["accel_y_ac"]]))
    hor_dc = float(np.hypot(rep["accel_x_mean"], rep["accel_y_mean"]))
    rep["accel_hor_ac"] = hor_ac
    rep["accel_hor_dc"] = hor_dc
    print(f"  水平合成: 直流偏置 {hor_dc:.4f} g   交流(振动) {hor_ac:.4f} g")
    print()

    print("-- 4. 解算内部量 -----------------------------------------------")
    print(f"  诊断帧数    : {rep['diag_frames']}")
    print(f"  confidence  : 最小 {rep['conf_min']:.2f} / 均值 {rep['conf_mean']:.2f}"
          f"  (低于 0.75 的比例 {rep['conf_low_ratio']*100:.1f}%)")
    print(f"  解算周期    : 均值 {rep['period_mean_us']:.0f} us  "
          f"范围 {rep['period_min_us']:.0f} ~ {rep['period_max_us']:.0f} us")
    print()

    print("-- 5. 电机差动 --------------------------------------------------")
    _mm = rep.get("motor_means")
    if _mm is not None:
        print(f"  单路均值   : m1 {_mm[0]:.0f}  m2 {_mm[1]:.0f}  m3 {_mm[2]:.0f}  m4 {_mm[3]:.0f}"
              f"    极差 {max(_mm)-min(_mm):.0f} PWM")
        if (max(_mm) - min(_mm)) > 100.0:
            print(f"  [!] 四路均值相差 {max(_mm)-min(_mm):.0f} PWM —— 机架重心偏了，")
            print("      或者某个电机/桨/电调效率不同。悬停时混控要长期偏置，")
            print("      该侧行程被吃掉，更容易触发混控饱和（'一摇杆就爬升'）。")
    print(f"  左侧均值 {rep['motor_left_mean']:.0f} / 右侧均值 {rep['motor_right_mean']:.0f}")
    print(f"  左-右 均值 {rep['motor_diff_mean']:+.1f}  漂移率 {rep['motor_diff_drift']:+.3f} /s")
    if rep.get("mix_sat_ratio") is not None:
        print(f"  混控抗饱和 : {rep['mix_sat_ratio']*100:.1f}% 的样本差动超出行程被缩放")
        if rep["mix_sat_ratio"] > 0.01:
            print("               （说明机动时四路会顶到限位。若老日志里这个比例高，")
            print("                 推力会随打杆凭空变大 —— 表现为'一摇杆就爬升'）")
    print()

    print("-- 5b. 频谱（振动 / 自激）-----------------------------------------")
    print(f"  内环自激  10~30Hz : RMS {rep['osc_rms']:6.2f} deg/s   主峰 {rep['osc_peak']:5.1f} Hz")
    print(f"  结构振动  >100Hz : gyroY {rep['vib_rms']:6.2f}  gyroX {rep['vibx_rms']:6.2f} deg/s"
          f"   主峰 {rep['vib_peak']:5.1f} Hz   加计 {rep['accl_vib']:.4f} g")
    print("  判读：自激 RMS 小(<5) 且 结构振动 RMS 小(<8) 才算健康；")
    print("        改了 IMU 带宽/W 陷波参数后，对比这两个数就能知道代价")
    print()

    print("-- 6. 遥控摇杆（1000~2000，中位 1500）----------------------------")
    if rep.get("rc_n", 0) >= 5:
        print(f"  帧数 {rep['rc_n']}")
        print(f"  roll  均值 {rep['rc_roll_mean']:7.1f}  平均偏离中位 {rep['rc_roll_absmean']:5.1f}"
              f"  回中占比 {rep['rc_roll_center_ratio']*100:5.1f}%")
        print(f"  pitch 均值 {rep['rc_pitch_mean']:7.1f}  平均偏离中位 {rep['rc_pitch_absmean']:5.1f}"
              f"  回中占比 {rep['rc_pitch_center_ratio']*100:5.1f}%")
        print(f"  油门  均值 {rep['rc_thr_mean']:7.1f}")
        ar = rep.get("acc_roll", float("nan")); ap = rep.get("acc_pitch", float("nan"))
        er = rep.get("roll_mean", float("nan")); ep = rep.get("pitch_mean", float("nan"))
        print(f"  加计推算倾角(固件内计算): roll {ar:+7.3f}  pitch {ap:+7.3f} deg")
        print(f"  姿态估计均值            : roll {er:+7.3f}  pitch {ep:+7.3f} deg")
        print(f"  估计偏差(= 姿态 - 加计)   : roll {er-ar:+7.3f}  pitch {ep-ap:+7.3f} deg"
              f"   ← 真实的姿态估计误差")
    else:
        print(f"  本段只有 {rep.get('rc_n', 0)} 个摇杆帧 —— 固件是旧版（未记录摇杆），"
              f"重新烧录后再飞才有这个数据")
    print()

    print("-- 7. 高度估计 --------------------------------------------------")
    if not rep.get("alt_present"):
        print("  本段没有高度帧(type=4) —— 两种情况：")
        print("    · 固件还是旧版（没有 BB_TYPE_ALT）→ 重新烧录")
        print("    · 固件是新的但气压高度一直不可用 → 定高被禁止，ALT 帧也不会发。")
        print("      这时看 BB:STAT 的 ok、或检查 BMP280（I2C）是否上电正常。")
    else:
        _dt = rep.get("alt_dt_median_ms")
        _hz = (1000.0 / _dt) if _dt else 0.0
        print(f"  高度帧数    : {rep['alt_n']} (约 {_hz:.0f}Hz)，baro_ok 占比 "
              f"{rep['alt_ok_ratio']*100:.1f}%")
        if rep.get("alt_dt_median_ms") is not None:
            print(f"  气压更新间隔: 中位 {rep['alt_dt_median_ms']:.1f} ms "
                  f"(正常应接近固件的读取周期 50ms；明显偏大说明 I2C 慢或任务被抢占)")
        if rep.get("alt_range") is not None:
            print(f"  气压高度    : {rep['alt_baro_range'][0]:+.2f} ~ "
                  f"{rep['alt_baro_range'][1]:+.2f} m   (原始样本 "
                  f"{rep.get('alt_samples', 0)} 个，标准差 "
                  f"{rep.get('alt_baro_std', float('nan'))*100:.1f} cm，"
                  f"相邻有效样本差 {rep.get('alt_baro_step', float('nan'))*100:.1f} cm)")
            print(f"  融合高度    : {rep['alt_range'][0]:+.2f} ~ "
                  f"{rep['alt_range'][1]:+.2f} m")
            print(f"  融合-气压   : 平均 |差| {rep['alt_track_err']*100:.1f} cm  "
                  f"(最大 {rep['alt_track_err_max']*100:.1f} cm)")
            print(f"  垂直速度    : RMS {rep['alt_vvel_rms']*100:.1f} cm/s   "
                  f"最大 {rep['alt_vvel_max']*100:.1f} cm/s")
            if rep.get("alt_steady_vvel_rms") is not None:
                print(f"  稳态(后半段): 垂速 RMS {rep['alt_steady_vvel_rms']*100:.1f} cm/s   "
                      f"创新 RMS {rep['alt_steady_innov_rms']*1000:.0f} mm   "
                      f"融合-气压最大 {rep['alt_steady_err_max']*100:.1f} cm")
                print("                （整体 RMS 常被起飞/切入的几秒拉高，稳态这行才是真实水平）")
            print(f"  创新 innov  : 平均 {rep['alt_innov_absmean']*1000:.1f} mm   "
                  f"95% {rep['alt_innov_p95']*1000:.1f} mm   最大 {rep['alt_innov_max']*1000:.1f} mm")
            print(f"  加计偏置    : {rep['alt_bias_first']:+.4f} -> "
                  f"{rep['alt_bias_last']:+.4f} m/s²")
            print(f"  气压原始值  : {rep['alt_press_range'][0]:.2f} ~ "
                  f"{rep['alt_press_range'][1]:.2f} hPa")
            print(f"  定高修正量  : {rep['alt_thr_range'][0]:+.0f} ~ "
                  f"{rep['alt_thr_range'][1]:+.0f} PWM "
                  f"(中位 {rep.get('alt_thr_median', float('nan')):+.0f}，限幅 ±600，"
                  f"贴限幅 {rep.get('alt_sat_ratio', 0)*100:.0f}%)")
            # 贴着 ±600 的频率高 = 环路在满舵打摆（电机/声音会明显抖）
            if rep.get("alt_sat_ratio", 0) > 0.02:
                print(f"  [!] 定高修正量有 {rep['alt_sat_ratio']*100:.0f}% 的样本贴着 ±600 ——"
                      f" 定高环在满舵打摆")
                print("      按可能性排查：")
                print("      ① 定高 PID 的 dt 与实际高度更新率不符（D 项被放大）")
                print("      ② pid.altitude.dlpf 太大（=1.0 表示 D 项完全没滤波）")
                print("      ③ pid.altitude_velocity.kp 太小（阻尼不足，只能靠位置过冲来回拉）")
            if rep.get("alt_hover_range") is not None:
                print(f"  基准油门    : 学到的悬停油门 "
                      f"{rep['alt_hover_range'][0]:.0f} ~ {rep['alt_hover_range'][1]:.0f} PWM"
                      f"（有效 {rep.get('alt_hover_valid_ratio', 0)*100:.0f}%，"
                      f"最后 {rep.get('alt_hover_last', float('nan')):.0f}）")
            else:
                print("  基准油门    : 本段还没学到（hover_valid=0，仍用电压补偿值）")
            # 悬停时修正量应当接近 0；长期偏离说明基准油门不对
            thr_med = rep.get("alt_thr_median", 0.0)
            if abs(thr_med) > 200.0:
                print(f"  [!] 修正量中位 {thr_med:+.0f} PWM 长期偏离 0 —— "
                      f"基准油门与真实悬停油门不匹配")
                print("      （悬停油门还没学会，或 thr.tho_h/tho_l 与实际机体不符）")
            if rep.get("alt_clamped_ratio", 0) > 0.02:
                print(f"  [!] 创新被限幅的样本 {rep['alt_clamped_ratio']*100:.1f}% ——"
                      f" 估计值与气压计相差超过 2m 的次数很多")
                print("      两种可能：")
                print("      ① 气压计坏样本（对照上面的“重复样本”、气压范围是否离谱）")
                print("      ② 估计器跟不上真实高度（est.Kp/Kv/Kb 太小 / 下降太快）")
                print("      → 若是 ②，把 est.Kp/Kv/Kb 设成 2.4/0.16/12 再飞（见 README）")
            if rep.get("alt_gain_rejected"):
                print("  [!] est.Kp/Kv/Kb 不满足稳定域 Kb·Kp>Kv，固件已退回默认值")
            if rep.get("alt_dup_ratio", 0.0) > 0.3:
                print(f"  [!] {rep['alt_dup_ratio']*100:.0f}% 的相邻气压样本完全相同 ——"
                      f" 读取周期短于 BMP280 的转换时间")
                print("      （×16 过采样单次转换约 40ms，而固件 20ms 读一次）")
                print("      后果：有效更新率只有约一半，每个值还偏老 20~40ms。")
                print("      处理：读取周期改 40~50ms，或过采样降到 ×4 并打开 IIR 滤波。")
    print()

    print("=" * 72)
    print("判读")
    print("=" * 72)

    findings = []

    rate = abs(rep["roll_drift_rate"])
    if rate >= DRIFT_RATE_WARN:
        findings.append(
            f"[!] roll 以 {rep['roll_drift_rate']:+.3f} deg/s 持续漂移，飞 "
            f"{rep['duration_s']:.0f}s 累计 {rep['roll_drift_total']:+.1f}deg。"
            f"这是**发散型**漂移（越飞越歪），说明姿态基准在移动，不是固定偏置。")
    elif rate >= DRIFT_RATE_NOTICE:
        findings.append(
            f"[~] roll 存在弱漂移 {rep['roll_drift_rate']:+.3f} deg/s"
            f"（累计 {rep['roll_drift_total']:+.1f}deg）。")
    else:
        findings.append(
            f"[OK] roll 基本不漂（{rep['roll_drift_rate']:+.3f} deg/s），"
            f"若仍感觉左偏，更可能是**固定偏置**（基准偏了恒定角度）或机械问题。")

    if abs(rep["roll_time_corr"]) > 0.7:
        findings.append(
            f"[i] roll 与时间的相关性 {rep['roll_time_corr']:+.2f}，"
            f"呈明确的**单调趋势** —— 与'飞得越久越偏'的描述一致。")

    if np.isfinite(rep["gyro_y_drift"]) and abs(rep["gyro_y_drift"]) > BIAS_DRIFT_NOTICE:
        findings.append(
            f"[!] 陀螺 Y 轴低频分量在飞行中变化了 {rep['gyro_y_drift']:+.3f} deg/s ——"
            f"**零偏在漂**。零偏 × 时间就是姿态误差，这条链路可以直接解释漂移。")
    else:
        findings.append(
            f"[OK] 陀螺 Y 轴偏置稳定（变化 {rep['gyro_y_drift']:+.3f} deg/s），"
            f"不像零偏温漂/电流致漂。")

    if rep["accel_dev_mean"] > ACCEL_DEV_NOTICE:
        findings.append(
            f"[~] 加速度计合成幅值**偏离** 1g 平均 {rep['accel_dev_mean']:.3f} g"
            f"（是 |‖a‖−1g| 的均值，不是幅值本身；各轴均值见上一节）——"
            f"振动/加速度噪声偏大，重力参考被污染，Madgwick 会被拉偏。")
    else:
        findings.append(
            f"[OK] 加速度计合成幅值贴近 1g（偏离均值 {rep['accel_dev_mean']:.4f} g），"
            f"重力参考本身没有被振动严重污染。")

    if np.isfinite(rep["conf_min"]) and rep["conf_min"] < 0.75:
        findings.append(
            f"[!] accel_confidence 最低掉到 {rep['conf_min']:.2f} ——"
            f"**重力修正被削弱过**，那段时间姿态纯靠陀螺积分，零偏会直接累积成角度误差。")
    elif np.isfinite(rep["conf_min"]):
        findings.append(
            f"[OK] accel_confidence 全程 >= {rep['conf_min']:.2f}，重力参考一直在工作，"
            f"排除'置信度被振动打掉'这条链路。")

    if np.isfinite(rep["period_mean_us"]):
        spread = rep["period_max_us"] - rep["period_min_us"]
        if abs(rep["period_mean_us"] - 1000.0) > 100.0 or spread > 300.0:
            findings.append(
                f"[~] 实测解算周期 {rep['period_mean_us']:.0f} us（范围 "
                f"{rep['period_min_us']:.0f}~{rep['period_max_us']:.0f}），"
                f"与名义 1000 us 有偏差 —— 说明写死 dt 确实是错的，"
                f"姿态解算的等效增益/重力修正强度都被改变了。")
        else:
            findings.append(
                f"[OK] 实测解算周期 {rep['period_mean_us']:.0f} us 接近名义 1000 us。")

    # ---- 摇杆判读：区分"操作者在修正"与"控制器自己推走" ----
    if rep.get("rc_n", 0) >= 5:
        r_off = rep["rc_roll_absmean"]
        p_off = rep["rc_pitch_absmean"]
        if r_off > 40.0 or p_off > 40.0:
            findings.append(
                f"[!] 操作者在**持续打杆修正**（roll 平均偏离中位 {r_off:.0f}、"
                f"pitch {p_off:.0f}，满量程 500）。"
                f"说明存在**恒定的机械偏置**（重心偏/电机座倾斜/单侧电机效率低），"
                f"控制器本身没算错 —— 应做机械配平，或在遥控器上打配平（trim）。")
            rg = rep.get("raw_gyro_y", float("nan"))
            fg = rep.get("gyro_y_mean", float("nan"))
            if np.isfinite(rg):
                findings.append(
                    f"[i] 陀螺 Y：原始 {rg:+.3f} dps，滤波后 {fg:+.3f} dps。"
                    f"两者接近 -> 偏移来自传感器/零偏；"
                    f"差异大 -> 滤波器引入了偏移。")
        else:
            findings.append(
                f"[OK] 摇杆基本回中（roll 偏离 {r_off:.0f}、pitch {p_off:.0f}）。"
                f"若飞机仍持续往一侧飞，问题在**控制器/姿态估计**侧，不是操作造成的。")

    if rep["frames_lost"] > 0:
        findings.append(f"[~] 记录期间丢了 {rep['frames_lost']} 帧，分析时注意时间轴有缺口。")

    for f in findings:
        print(" " + f)
    print()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="W25Q128 黑匣子数据分析")
    ap.add_argument("image", help="BB:DUMP 导出的 Flash 镜像（16MB）")
    ap.add_argument("--csv", help="把连续段导出为 CSV")
    ap.add_argument("--plot", action="store_true", help="生成曲线图（需要 matplotlib）")
    ap.add_argument("--save-plot", help="图片输出路径（配合 --plot）")
    ap.add_argument("--run", type=int, default=-1,
                    help="只分析第几段记录（0=最新）。默认分析整片全部数据")
    ap.add_argument("--trim-roll", type=float, default=0.0,
                    help="把实测姿态和设定值都减去这个角度(deg)，用于把 "
                         "ctrl.roll_offset_deg 当零点（默认 0 = 不处理）")
    ap.add_argument("--trim-pitch", type=float, default=0.0,
                    help="同上，pitch 方向")
    ap.add_argument("--list-runs", action="store_true",
                    help="只列出所有分段（不分析），配合 --run 使用")
    args = ap.parse_args(argv)

    try:
        with open(args.image, "rb") as fp:
            raw = fp.read()
    except OSError as exc:
        print(f"[!] 打不开文件：{exc}")
        return 1

    print(f"[analyze] 读取 {args.image}（{len(raw)} 字节 = {len(raw)/1048576:.2f} MB）")

    # --list-runs：只列出分段，不做分析
    if args.list_runs:
        try:
            lg = parse_flash(raw)
        except Exception as exc:                   # noqa: BLE001
            print(f"[!] 解析失败：{type(exc).__name__}: {exc}")
            return 1
        segs = lg.runs()
        print(f"\n共 {len(segs)} 段（按 seq 连续性划分，从新到旧）：")
        for k, s in enumerate(segs[:40]):
            print(f"  第{k:2d}段: {s.size:6d} 帧 ≈ {s.size/1000.0:6.1f} s   "
                  f"物理扇区 {int(lg.sector_of[s[0]])}~{int(lg.sector_of[s[-1]])}")
        if len(segs) > 40:
            print(f"  ...（共 {len(segs)} 段，只显示前 40）")
        print("\n用法： python bb.py analyze %s --run K" % args.image)
        return 0

    try:
        log, run, rep = analyze(raw, run_index=args.run,
                                trim_roll=args.trim_roll,
                                trim_pitch=args.trim_pitch)
    except MemoryError:
        print("[!] 内存不足：镜像太大。请关闭其它程序后重试。")
        return 1
    except Exception as exc:                       # noqa: BLE001
        import traceback
        print(f"[!] 分析失败：{type(exc).__name__}: {exc}")
        for ln in traceback.format_exc().splitlines()[-8:]:
            print("    " + ln)
        return 1

    if args.csv and run.size:
        print(f"[analyze] 导出 CSV → {args.csv}", flush=True)
        try:
            _write_csv(args.csv, log, run)
        except Exception as exc:                   # noqa: BLE001
            import traceback
            print(f"[!] 导出 CSV 失败：{type(exc).__name__}: {exc}")
            for ln in traceback.format_exc().splitlines()[-6:]:
                print("    " + ln)
            return 1

    if args.plot and run.size:
        print("[analyze] 生成曲线图 ...", flush=True)
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception:  # noqa: BLE001
            print("未安装 matplotlib，跳过绘图（pip install matplotlib）")
            print("提示：加 --csv 可以把数据导出成表格自己画。")
            return 0

        try:
            _make_plot(plt, log, run, args.save_plot)
        except Exception as exc:                   # noqa: BLE001
            import traceback
            print(f"[!] 绘图失败（分析结果不受影响）：{type(exc).__name__}: {exc}")
            for ln in traceback.format_exc().splitlines()[-6:]:
                print("    " + ln)

        # 阶跃响应叠加图（每个阶跃对齐到 t=0）。
        # 单独一张图，失败也不影响主图。
        try:
            base = args.save_plot or "blackbox_plot.png"
            _make_step_plot(plt, log, run, base)
        except Exception as exc:                   # noqa: BLE001
            print(f"[!] 阶跃响应图失败：{type(exc).__name__}: {exc}")

    return 0


# 绘图最多画这么多点：原始 49 万点画 9 条线对 matplotlib 负担过重，
# 降采样后曲线形状完全一样，但内存/耗时降一个数量级。
PLOT_MAX_POINTS = 20000


def _setpoint_series(log, idx_run):
    """把摇杆帧里的**设定值**零阶保持到给定的数据帧索引上。

    为什么要独立成函数：`_write_csv` 与 `_make_plot` 都要这份数据，
    而它的正确性依赖两个容易出错的细节（摇杆帧被排除过、
    旧固件的 reserved 是栈垃圾），集中一处便于单独验证。

    返回 (sp_roll, sp_pitch)，长度与 idx_run 相同；日志是旧格式时返回 (None, None)。
    """
    try:
        allf = log.all_frames()
        rc = allf[log.ftype[allf] == TYPE_RC]
        if rc.size < 2:
            return None, None
        # 必须是特征字节，不能用 0/1——旧固件没写过这个字节，
        # 它是未初始化的栈垃圾，恰好等于 1 的概率不小。
        if not np.all(log.rc_fmt[rc] == RC_FMT_TARGET):
            return None, None
        r = np.asarray(log.rc[rc, 4], dtype=np.float64)
        p = np.asarray(log.rc[rc, 5], dtype=np.float64)
        # 值域校验：角度模式下目标角不可能超过 ±45°
        if (not np.all(np.isfinite(r))) or (np.max(np.abs(r)) > 45.0):
            return None, None
        if (not np.all(np.isfinite(p))) or (np.max(np.abs(p)) > 45.0):
            return None, None
        pos = np.clip(np.searchsorted(rc, np.asarray(idx_run), side="right") - 1,
                      0, rc.size - 1)
        return r[pos], p[pos]
    except Exception:
        return None, None


def _time_axis(log, run):
    """真实时间轴（秒）。

    ⚠ 不能用"槽位号 / 1000"：高度帧(50Hz)和摇杆帧是插进来的**额外槽位**，
    它们不各占 1ms（高度帧借用当前序号、不递增；摇杆帧与数据帧同拍）。
    用槽位号当时间，整条轴会被拉长 3%(摇杆)+5%(高度) ≈ 8%，
    看高度/垂速的响应时间就会系统性偏慢。
    这里按"只有占序号的槽位才走 1ms"累加，得到与固件 1kHz 序号一致的时间。
    """
    ftype = log.ftype[run]
    step_ms = np.where(ftype == TYPE_ALT, 0.0, 1.0)
    t = np.cumsum(step_ms) / 1000.0
    if t.size:
        t -= t[0]
    return t


def _alt_series(log, idx_run):
    """把高度帧(BB_TYPE_ALT)里的量零阶保持到给定帧索引上。

    和 _setpoint_series 同一个套路：高度帧在 run 里是独立槽位，
    要按"时间上最近的前一帧"取值，才能和姿态/油门对齐。

    返回 dict（键：baro/alt/vvel/innov/bias/baro_dt/thr/press/status），
    长度与 idx_run 相同；这份日志没有高度帧时返回 None。
    """
    try:
        allf = log.all_frames()
        ai = allf[log.ftype[allf] == TYPE_ALT]
        if ai.size < 2:
            return None
        pos = np.clip(np.searchsorted(ai, np.asarray(idx_run), side="right") - 1,
                      0, ai.size - 1)
        out = {"n_frames": int(ai.size)}
        for key, col in (("baro", 0), ("alt", 1), ("vvel", 2), ("innov", 3),
                         ("bias", 4), ("baro_dt", 5), ("thr", 8), ("press", 7)):
            out[key] = np.asarray(log.alt[ai, col], dtype=np.float64)[pos]
        out["hover"] = np.asarray(log.alt_hover[ai], dtype=np.float64)[pos]
        out["hover_valid"] = np.asarray(log.alt_hover_valid[ai], dtype=np.uint8)[pos]
        out["status"] = np.asarray(log.alt_status[ai], dtype=np.uint8)[pos]
        return out
    except Exception:
        return None


def _write_csv(path, log, run):
    """导出 CSV。

    关键：先把需要的列**一次性**取成 Python 列表(tolist)，再逐行写。
    原实现每行都写 log.roll[i] / log.pitch[i] / log.yaw[i]，
    而这三个是 @property（返回 self.angle[:, k] 的新视图对象），
    49 万行 × 3 次 = 150 万次对象创建，既慢又吃内存。
    """
    import csv

    # 一次性取出(用 tolist 转成纯 Python 值，后续访问零开销)
    idx_run = run.tolist()
    ftype = log.ftype[run].tolist()
    ang = log.angle[run].tolist()      # [[roll,pitch,yaw], ...]
    gyro = log.gyro[run].tolist()
    accel = log.accel[run].tolist()
    motor = log.motor[run].tolist()
    throttle = log.throttle[run].tolist()
    conf = log.diag_conf[run].tolist()
    period = log.diag_period[run].tolist()
    batt = log.battery[run].tolist()

    # 设定值（目标角）：存在摇杆帧里，而 run 已把摇杆帧剔除，
    # 所以要从全帧序列里取摇杆帧，再零阶保持到数据帧网格上。
    sp_roll, sp_pitch = _setpoint_series(log, run)
    if sp_roll is not None:
        sp_roll = sp_roll.tolist()
        sp_pitch = sp_pitch.tolist()

    # 时间轴：按真实 1ms 序号累加（不能直接用槽位号，见 _time_axis）
    t_axis = _time_axis(log, run).tolist()

    # 高度帧（50Hz）同样零阶保持到本网格上
    alt_s = _alt_series(log, run)
    if alt_s is not None:
        altd = {k: alt_s[k].tolist() for k in
                ("baro", "alt", "vvel", "innov", "bias", "baro_dt", "thr", "press")}
        alt_ok = ((alt_s["status"] & 0x01) != 0).astype(int).tolist()
    else:
        altd = None
        alt_ok = None

    with open(path, "w", newline="", encoding="utf-8") as fp:
        w = csv.writer(fp)
        w.writerow(["idx", "t_s", "type", "roll_deg", "pitch_deg", "yaw_deg",
                    "gyro_x_dps", "gyro_y_dps", "gyro_z_dps",
                    "accel_x_g", "accel_y_g", "accel_z_g",
                    "m1", "m2", "m3", "m4", "throttle",
                    "conf", "period_us", "battery",
                    "sp_roll_deg", "sp_pitch_deg",
                    "baro_m", "alt_m", "vvel_mps", "innov_m",
                    "bias_mps2", "baro_dt_ms", "thr_corr", "press_hpa", "alt_ok",
                    "hover_base_pwm"])
        for k in range(len(idx_run)):
            a, g, ac, m = ang[k], gyro[k], accel[k], motor[k]
            w.writerow([
                k, f"{t_axis[k]:.3f}", ftype[k],
                f"{a[0]:.2f}", f"{a[1]:.2f}", f"{a[2]:.2f}",
                f"{g[0]:.2f}", f"{g[1]:.2f}", f"{g[2]:.2f}",
                f"{ac[0]:.4f}", f"{ac[1]:.4f}", f"{ac[2]:.4f}",
                f"{m[0]:.0f}", f"{m[1]:.0f}", f"{m[2]:.0f}", f"{m[3]:.0f}",
                f"{throttle[k]:.0f}",
                "" if not np.isfinite(conf[k]) else f"{conf[k]:.2f}",
                "" if not np.isfinite(period[k]) else f"{period[k]:.0f}",
                batt[k],
                "" if sp_roll  is None else f"{sp_roll[k]:.2f}",
                "" if sp_pitch is None else f"{sp_pitch[k]:.2f}",
            ] + ([] if altd is None else [
                f"{altd['baro'][k]:+.3f}", f"{altd['alt'][k]:+.3f}",
                f"{altd['vvel'][k]:+.3f}", f"{altd['innov'][k]:+.4f}",
                f"{altd['bias'][k]:+.4f}", f"{altd['baro_dt'][k]:.1f}",
                f"{altd['thr'][k]:+.0f}", f"{altd['press'][k]:.2f}",
                alt_ok[k], f"{alt_s['hover'][k]:.0f}",
            ]))
    print(f"已导出 CSV: {path}（{len(idx_run)} 行）")
    if altd is None:
        print("      注：baro_m/alt_m/... 这几列为空 —— 这份日志没有高度帧(type=4)，")
        print("          要么是旧固件，要么气压高度一直不可用（定高被禁，ALT 帧不发）。")
    else:
        print("      已包含高度列 baro_m / alt_m / vvel_mps / innov_m / bias_mps2 / thr_corr")
    if sp_roll is None:
        print("      注：sp_roll_deg / sp_pitch_deg 为空 —— 这份日志的摇杆帧没有记实际目标角")
        print("          （旧固件；reserved=0，槽位存的是加计推算倾角）。重新烧录后就有。")
    else:
        print("      已包含设定值列 sp_roll_deg / sp_pitch_deg（从摇杆帧零阶保持，32ms 分辨率）")


def _make_step_plot(plt, log, run, base_path) -> None:
    """把检测到的每个阶跃都对齐到 t=0 叠在一张图上。

    这是调 PID 直接要看的图：目标虚线、实测实线，
    上升快慢、超调多少、有没有振荡一目了然。

    销毁性设计：任何异常都被吞掉并打印提示，
    不会影响主图和文字报告。
    """
    try:
        import bb_step

        # ⚠ run 已把摇杆帧排除，直接传给 build_axis 会得到 rc_sp=None（没设定值）。
        #   这里取同一时间跨度的全帧集合（含摇杆帧）。
        allf = log.all_frames()
        if run.size:
            allf = allf[(allf >= run[0]) & (allf <= run[-1])]
        axis = bb_step.build_axis(log, allf)
        if axis is None or axis.get("rc_sp_roll") is None:
            print("        无法恢复设定值（无摇杆帧或旧格式），跳过阶跃响应图", flush=True)
            return

        rows = []
        for which in ("roll", "pitch"):
            for r in bb_step.analyse_steps(axis, which):
                rows.append((which, r))
        if not rows:
            print("        未检测到真阶跃，跳过阶跃响应图", flush=True)
            return

        span = 1200          # 阶跃后看 1.2 秒
        fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
        titles = {"roll": "Roll step response", "pitch": "Pitch step response"}
        colors = {"roll": "C0", "pitch": "C1"}
        drawn = {"roll": 0, "pitch": 0}

        for which, r in rows:
            i0 = int(r["t_step_ms"])
            pre = 200
            t_axis = np.arange(-pre, span) / 1000.0
            a0 = i0 - pre
            a1 = i0 + span
            if a0 < 0 or a1 > axis["n_data"]:
                continue
            ang = axis["angle_" + which][a0:a1]
            sp = axis["rc_sp_" + which][a0:a1]
            ax = axes[0] if which == "roll" else axes[1]
            c = colors[which]
            ax.plot(t_axis, ang, lw=1.0, color=c, alpha=0.85)
            ax.plot(t_axis, sp, lw=1.4, ls="--", color="k", alpha=0.55)
            drawn[which] += 1

        for k, which in enumerate(("roll", "pitch")):
            axes[k].axvline(0.0, color="r", lw=0.8, ls=":")
            axes[k].axhline(0.0, color="0.7", lw=0.6)
            axes[k].set_ylabel("angle (deg)")
            axes[k].grid(True, alpha=0.3)
            axes[k].set_title("%s  (%d steps)\n solid = measured, dashed = setpoint"
                              % (titles[which], drawn[which]))
        axes[1].set_xlabel("time since step (s)")

        fig.tight_layout()
        out = base_path.replace(".png", "") + "_step.png"
        fig.savefig(out, dpi=110)
        print(f"        已保存阶跃响应图: {out}"
              f"（roll {drawn['roll']} 次 / pitch {drawn['pitch']} 次）", flush=True)
    except Exception as exc:              # 图画不出来不能影响文字报告
        print(f"        阶跃响应图生成失败（已跳过）: {exc}", flush=True)


def _make_plot(plt, log, run, save_plot) -> None:
    """把 6 条曲线画到一张图并存盘。单独成函数：出问题不影响前面的文字结论。

    注意：先降采样再画。原始一段飞行可达 49 万点，9 条线全画会让
    matplotlib 占掉大量内存和时间（严重时进程被系统杀掉，看起来像"闪退"）。
    降采样到 2 万点后曲线形状不变，负担降一个数量级。
    """
    import numpy as np

    # 一次性把需要的列取出来（避免反复访问 property 产生新视图对象）
    ang = log.angle[run]
    gyro = log.gyro[run]
    accel = log.accel[run]
    motor = log.motor[run]

    n = run.size
    step = max(1, n // PLOT_MAX_POINTS)
    sel = slice(None, None, step)

    # 真实时间轴（按 1ms 序号累加，不是槽位号；见 _time_axis）
    t = _time_axis(log, run)[sel]
    rpy = ang[sel]
    gyr = gyro[sel]
    mot = motor[sel]
    an = np.linalg.norm(accel[sel], axis=1)

    print(f"        （{n} 点 → 降采样 1/{step} → {t.size} 点）", flush=True)

    # 设定值（目标角）——用与 CSV 完全相同的取法，保证图与表一致
    sp_r_all, sp_p_all = _setpoint_series(log, run)
    if sp_r_all is not None:
        sp_r = sp_r_all[sel]
        sp_p = sp_p_all[sel]
    else:
        sp_r = sp_p = None

    # 高度帧（有就多画两块：高度 / 垂直速度）
    alt_s = _alt_series(log, run)
    if alt_s is not None:
        alt_baro = alt_s["baro"][sel]
        alt_fused = alt_s["alt"][sel]
        alt_vvel = alt_s["vvel"][sel] * 100.0          # -> cm/s，看起来更直观
        alt_inn = alt_s["innov"][sel] * 100.0          # -> cm
        alt_thr = alt_s["thr"][sel]
        n_panel = 9
    else:
        n_panel = 7

    fig, axes = plt.subplots(n_panel, 1, figsize=(13, 2.5 * n_panel), sharex=True)

    axes[0].plot(t, rpy[:, 0], lw=0.8, label="roll")
    axes[0].plot(t, rpy[:, 1], lw=0.8, label="pitch")
    axes[0].set_ylabel("angle (deg)")
    axes[0].legend(loc="upper left")
    axes[0].set_title("Attitude")

    # ── 设定值 vs 实际：这就是阶跃响应曲线。
    # 虚线=目标（设定值），实线=实测角度。两根线的距离就是跟随误差。
    axes[1].plot(t, rpy[:, 0], lw=0.8, color="C0", label="roll actual")
    axes[1].plot(t, rpy[:, 1], lw=0.8, color="C1", label="pitch actual")
    if sp_r is not None:
        axes[1].plot(t, sp_r, lw=1.3, ls="--", color="C0", label="roll setpoint")
        axes[1].plot(t, sp_p, lw=1.3, ls="--", color="C1", label="pitch setpoint")
        axes[1].set_title("Setpoint vs actual  (dashed = target, solid = measured) "
                          "— step response")
    else:
        axes[1].set_title("Setpoint vs actual — target not logged "
                          "(old firmware: RC frame slot held accel tilt, not target)")
    axes[1].set_ylabel("angle (deg)")
    axes[1].legend(loc="upper left", ncol=2, fontsize=8)

    axes[2].plot(t, gyr[:, 1], lw=0.6, label="gyro_y (deg/s)")
    axes[2].plot(t, gyr[:, 0], lw=0.6, label="gyro_x")
    axes[2].set_ylabel("rate (deg/s)")
    axes[2].legend(loc="upper left")
    axes[2].set_title("Gyro (bias drift shows as slow offset)")

    axes[3].plot(t, an, lw=0.6)
    axes[3].axhline(1.0, color="r", ls="--", lw=0.8)
    axes[3].set_ylabel("|a| (g)")
    axes[3].set_title("Accel magnitude (should stay near 1g)")

    axes[4].plot(t, mot[:, 0], lw=0.7, label="m1(L)")
    axes[4].plot(t, mot[:, 1], lw=0.7, label="m2(L)")
    axes[4].plot(t, mot[:, 2], lw=0.7, label="m3(R)")
    axes[4].plot(t, mot[:, 3], lw=0.7, label="m4(R)")
    axes[4].set_ylabel("motor PWM")
    axes[4].legend(loc="upper left", ncol=4)
    axes[4].set_title("Motors")

    conf = log.diag_conf[run][sel]
    axes[5].plot(t, conf, ".", ms=2)
    axes[5].set_ylim(-0.05, 1.05)
    axes[5].set_ylabel("accel_confidence")
    axes[5].set_title("Gravity-correction weight (dips = gravity reference lost)")

    per = log.diag_period[run][sel]
    axes[6].plot(t, per, ".", ms=2)
    axes[6].axhline(1000.0, color="r", ls="--", lw=0.8)
    axes[6].set_ylabel("sample period (us)")
    axes[6].set_title("Attitude solve period (real dt)")

    if alt_s is not None:
        # 高度：虚线=气压计(融合器的输入)，实线=融合输出。
        # 两根线几乎重合是对的（T=5s 量级的滤波，短时靠加计、长时跟气压计）。
        axes[7].plot(t, alt_baro, lw=0.8, ls="--", color="0.55", label="baro (input)")
        axes[7].plot(t, alt_fused, lw=1.0, color="C2", label="fused altitude")
        axes[7].set_ylabel("altitude (m)")
        axes[7].legend(loc="upper left", ncol=2, fontsize=8)
        axes[7].set_title("Altitude: barometer vs fused estimate")

        axes[8].plot(t, alt_vvel, lw=0.8, color="C4", label="vertical velocity (cm/s)")
        axes[8].plot(t, alt_thr, lw=0.8, alpha=0.8,
                     label="alt-hold throttle correction (PWM)")
        axes[8].set_ylabel("vvel (cm/s) / corr (PWM)")
        axes[8].legend(loc="upper left", ncol=2, fontsize=8)
        axes[8].set_title("Vertical velocity and alt-hold output "
                          "(corr flat at 0 = alt-hold off or altitude data invalid)")

    axes[-1].set_xlabel("time (s)")

    fig.tight_layout()
    out = save_plot or "blackbox_plot.png"
    fig.savefig(out, dpi=110)
    print(f"已保存曲线图: {out}")


if __name__ == "__main__":
    sys.exit(main())
