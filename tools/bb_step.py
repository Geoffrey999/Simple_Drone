# -*- coding: utf-8 -*-
"""阶跃响应 / 回正响应分析。

用途：从黑匣子数据里量出闭环动态特性，把"手感调参"变成"按指标调参"。

两类事件
--------
1. **设定值阶跃**（打杆）：摇杆突变 -> 目标角度突变，衡量**跟踪性能**
   （上升时间、超调、稳定时间、稳态误差）。
2. **回正**（松杆 / 被打偏后自主回正）：设定值不变而角度大，衡量**回正性能**
   ——这是"小角度回正慢"这类抱怨的量化对象。

时间基准
--------
用**数据帧在数据流中的序号 / 1000**作为时间轴，而不是帧里的 seq。
原因：实测每个扇区多消耗 8 个 seq（120 帧用掉 128 个），
按 seq 计时会有 6.7% 的系统误差；而报告显示"丢帧 0"，说明数据帧本身
严格 1kHz 连续，用序号计时是精确的。

设定值的局限
------------
固件里 `target = (摇杆-1500)/9 + (roll_offset-1500)`，而 `roll_offset`
来自接收机微调、**未记入黑匣子**。本模块按 offset=1500（无微调）换算，
若你在遥控器上设过微调，设定值曲线会有一个恒定的垂直偏移，
**阶跃的幅度和时刻仍然准确，只有绝对角度值会整体平移**。
"""

from __future__ import annotations

import numpy as np

from bb_format import TYPE_RC, RC_FMT_TARGET

STICK_CENTER = 1500.0
STICK_TO_DEG = 1.0 / 9.0

# 判定门限
STEP_MIN_DEG = 5.0        # 设定值变化小于此值不算阶跃（3° 以下多是操作抖动）
STEP_PRE_S = 0.20         # 阶跃前取多长的基线
STEP_POST_S = 2.00        # 阶跃后观察多长
SETTLE_BAND = 0.05        # 稳定判据：进入终值的 ±5%
RETURN_START_DEG = 3.0    # 角度超过此值算一次"偏了"
RETURN_LEVELS = (5.0, 3.0, 2.0, 1.0, 0.5)   # 回正过程要跨过的角度台阶


# --------------------------------------------------------------------------
# 时间轴与设定值
# --------------------------------------------------------------------------
def build_axis(log, run=None):
    """构造统一时间轴，并返回数据帧数组与设定值数组（均为 1kHz）。

    返回 dict：t / angle_roll / angle_pitch / gyro / rc_sp_roll / rc_sp_pitch
    读不到摇杆帧时 rc_sp_* 为 None。
    """
    if run is None:
        run = log.all_frames()
    if run.size < 32:
        return None

    is_rc = log.ftype[run] == TYPE_RC
    data_mask = ~is_rc

    # 数据帧在数据流中的位置 = 时间(ms)，严格 1kHz
    n_data = int(data_mask.sum())
    if n_data < 32:
        return None

    out = {
        "n_data": n_data,
        "t": np.arange(n_data, dtype=np.float64) / 1000.0,
        "angle_roll": log.angle[run[data_mask], 0].astype(np.float64),
        "angle_pitch": log.angle[run[data_mask], 1].astype(np.float64),
        "gyro_roll": log.gyro[run[data_mask], 1].astype(np.float64),   # gyro_y = roll rate
        "gyro_pitch": log.gyro[run[data_mask], 0].astype(np.float64),  # gyro_x = pitch rate
    }

    # 摇杆帧在数据流中的位置（每帧摇杆覆盖其后 32ms）
    rc_pos = (np.cumsum(data_mask) - 1)[is_rc].astype(np.float64)
    if rc_pos.size >= 4:
        idx = np.arange(n_data, dtype=np.float64)

        # ★ 优先用日志里记的**实际目标角**（帧内 reserved=1）：
        #   它是送进控制器的设定值本身，不需要换算，也不受接收机
        #   微调(roll_offset)影响，BB:STEP 注入时也能正确取到。
        #   旧日志（reserved=0）那个槽位存的是加计推算倾角，不能当设定值，
        #   这时回落到从摇杆换算。
        fmt = getattr(log, "rc_fmt", None)
        sub = run[is_rc]
        has_target = bool(fmt is not None and np.all(fmt[sub] == RC_FMT_TARGET))
        if has_target:
            # 值域校验：目标角不可能超过 ±45°
            _r = log.rc[sub, 4]; _p = log.rc[sub, 5]
            if (np.max(np.abs(_r)) > 45.0) or (np.max(np.abs(_p)) > 45.0):
                has_target = False

        if has_target:
            sp_roll  = log.rc[sub, 4].astype(np.float64)
            sp_pitch = log.rc[sub, 5].astype(np.float64)
            out["sp_source"] = "target"
        else:
            sp_roll  = (log.rc[sub, 0] - STICK_CENTER) * STICK_TO_DEG
            sp_pitch = (log.rc[sub, 1] - STICK_CENTER) * STICK_TO_DEG
            out["sp_source"] = "stick"

        # 零阶保持：每个摇杆帧的值一直有效到下一个摇杆帧
        out["rc_sp_roll"] = _zoh(rc_pos, sp_roll, idx)
        out["rc_sp_pitch"] = _zoh(rc_pos, sp_pitch, idx)
        out["rc_dt"] = float(np.median(np.diff(rc_pos))) / 1000.0
    else:
        out["rc_sp_roll"] = None
        out["rc_sp_pitch"] = None
        out["rc_dt"] = None
        out["sp_source"] = None

    return out


def _zoh(px, py, x):
    """零阶保持采样：x 处的值取 x 之前最近一个样本的值。"""
    if px.size == 0:
        return np.full(x.shape, np.nan)
    j = np.searchsorted(px, x, side="right") - 1
    j = np.clip(j, 0, px.size - 1)
    return py[j]


# --------------------------------------------------------------------------
# 自带拟合：一阶模型 + 延迟
# --------------------------------------------------------------------------
def fit_first_order(t, y, i0, i1, y_inf):
    """对 y 的 [i0,i1) 段拟合 y(t) = y_inf + (y0-y_inf)*exp(-(t-t0)/tau)。

    只做一次网格搜索 + 线性最小二乘，不依赖 scipy。
    返回 (tau, t0, rms)。
    """
    tt = t[i0:i1]
    yy = y[i0:i1]
    if tt.size < 20 or not np.isfinite(yy).all():
        return float("nan"), float("nan"), float("nan")

    best = (float("nan"), float("nan"), float("inf"))
    # 网格搜索 tau 与 t0，y_inf 固定为终值
    for tau in np.geomspace(0.01, 2.0, 60):
        base = np.exp(-(tt - tt[0]) / tau)
        for delay in (0.0, 0.005, 0.010, 0.015, 0.020, 0.030):
            shifted = np.maximum(tt - tt[0] - delay, 0.0)
            base = np.exp(-shifted / tau)
            # yy = y_inf + A*base  ->  最小二乘求 A
            denom = float(np.dot(base, base))
            if denom <= 0:
                continue
            a = float(np.dot(base, yy - y_inf)) / denom
            pred = y_inf + a * base
            rms = float(np.sqrt(np.mean((pred - yy) ** 2)))
            if rms < best[2]:
                best = (float(tau), float(delay), rms)
    return best


# --------------------------------------------------------------------------
# 1. 设定值阶跃
# --------------------------------------------------------------------------
STEP_HOLD_S = 0.80        # 设定值必须**保持**这么久才算"真阶跃"，否则算点动


def find_steps(sp, min_delta=STEP_MIN_DEG, hold_s=STEP_HOLD_S):
    """返回真阶跃 [(index, delta_deg), ...]。

    区分"阶跃"和"点动"：打杆后设定值必须在新值上**保持 hold_s**，
    否则它只是脉冲激励，用它算上升时间/稳态误差没有意义
    （2 秒后飞机已经跟着回中了，终值自然是 0）。
    """
    if sp is None:
        return []
    d = np.diff(sp)
    hits = np.where(np.abs(d) >= min_delta)[0] + 1
    hold_n = int(hold_s * 1000)

    out = []
    for i in hits:
        new_val = sp[i]
        end = min(i + hold_n, sp.size)
        if end <= i:
            continue
        seg = sp[i:end]
        # 保持期间偏离新值不超过阶跃幅度的 30% 才算稳定
        tol = max(0.5, 0.30 * abs(d[i - 1]))
        if np.all(np.abs(seg - new_val) <= tol):
            out.append((int(i), float(d[i - 1])))
    return out


def find_taps(sp, min_delta=STEP_MIN_DEG):
    """返回"点动"事件（设定值突变但很快回中），仅用于统计，不做阶跃指标。"""
    if sp is None:
        return []
    allhits = find_steps(sp, min_delta, hold_s=0.0)
    real = {i for i, _ in find_steps(sp, min_delta)}
    return [(i, d) for i, d in allhits if i not in real]


def analyse_steps(axis, which="roll", min_delta=STEP_MIN_DEG):
    """对每个真阶跃算跟踪指标。

    ★ 稳态值必须在**保持段内**取，不能用固定 2 秒窗口的末尾：
    摇杆往往保持 0.8~1.5 秒就回中，2 秒后飞机已经跟着回到 0，
    在窗口末尾取"终值"会得到 ≈0，超调和稳态误差都会变成垃圾数字。
    """
    sp = axis["rc_sp_" + which]
    ang = axis["angle_" + which]
    t = axis["t"]
    if sp is None:
        return []

    n_pre = int(STEP_PRE_S * 1000)
    rows = []

    for i, delta in find_steps(sp, min_delta):
        tol = max(0.5, 0.30 * abs(delta))

        # 保持段终点：设定值偏离新值超过容差的那一刻（或数据结束）
        hold_end = int(sp.size)
        for k in range(i + 1, sp.size):
            if abs(sp[k] - sp[i]) > tol:
                hold_end = int(k)
                break
        if hold_end - i < 300:          # 保持不足 300ms，无法量稳态
            continue

        seg = ang[i:hold_end]
        if seg.size < 100 or not np.isfinite(seg).any():
            continue

        # 基线（阶跃前）与稳态（保持段最后 200ms）
        base = float(np.nanmean(ang[max(0, i - n_pre):i]))
        ss0 = max(i, hold_end - 200)
        tail = ang[ss0:hold_end]
        final = float(np.nanmean(tail))
        amp = final - base
        if abs(amp) < 2.5:              # 姿态跟上不足 2.5°，做不出有意义的指标
            continue

        # 上升时间 10% -> 90%（只看保持段内）
        lo, hi = base + 0.1 * amp, base + 0.9 * amp
        t_lo = t_hi = None
        for k in range(seg.size):
            v = seg[k]
            if t_lo is None and ((amp > 0 and v >= lo) or (amp < 0 and v <= lo)):
                t_lo = t[i + k]
            if t_lo is not None and ((amp > 0 and v >= hi) or (amp < 0 and v <= hi)):
                t_hi = t[i + k]
                break
        rise = (t_hi - t_lo) if (t_lo is not None and t_hi is not None) else float("nan")

        # 超调与峰值时刻（只看保持段内）
        if amp > 0:
            k_peak = int(np.nanargmax(seg))
        else:
            k_peak = int(np.nanargmin(seg))
        peak = float(seg[k_peak])
        overshoot = max(0.0, (peak - final) / amp * 100.0)
        t_peak = t[i + k_peak] - t[i]

        # 稳定时间：最后一次离开 ±5% 带的时刻
        band = SETTLE_BAND * abs(amp)
        outside = np.where(np.abs(seg - final) > band)[0]
        settle = (t[i + int(outside[-1]) + 1] - t[i]) if (outside.size and outside[-1] + 1 < seg.size) else 0.0

        tau, delay, rms = fit_first_order(t, ang, i, hold_end, final)

        rows.append({
            "t_step_ms": t[i] * 1000.0,
            "hold_ms": (hold_end - i),
            "axis": which,
            "delta_sp": delta,
            "from_deg": base,
            "to_deg": final,
            "sp_deg": float(sp[i]),
            "rise_ms": rise * 1000.0 if np.isfinite(rise) else float("nan"),
            "overshoot_pct": overshoot,
            "t_peak_ms": t_peak * 1000.0,
            "settle_ms": settle * 1000.0,
            "ss_err": final - float(sp[i]),
            "tau_ms": tau * 1000.0,
            "delay_ms": delay * 1000.0,
            "fit_rms": rms,
        })
    return rows


# --------------------------------------------------------------------------
# 2. 回正过程（松杆 / 被打偏后自主回正）
# --------------------------------------------------------------------------
def find_returns(axis, which="roll", start_deg=RETURN_START_DEG):
    """找出"角度偏离 > start_deg 之后回到零"的过程。

    返回 [(i_peak, i_zero), ...]：最大偏离点、过零点（或最小绝对值的点）。
    """
    ang = axis["angle_" + which]
    n = ang.size
    if n < 200:
        return []

    a = np.nan_to_num(ang, nan=0.0)
    sp = axis.get("rc_sp_" + which)
    episodes = []
    i = 0
    while i < n:
        if abs(a[i]) < start_deg:
            i += 1
            continue
        # 找偏离段的终点（回到 start_deg 以内）
        j = i
        peak_i = i
        while j < n and abs(a[j]) >= start_deg * 0.5:
            if abs(a[j]) > abs(a[peak_i]):
                peak_i = j
            j += 1
        if j - i >= 100:                       # 至少 100ms
            # ★ 只收"设定值回中时的自由回正"。
            #   否则像 BB:STEP 的方波激励，飞机只是在 ¸° 之间来回，
            #   并没有真正回到水平，算出来的"各台阶耗时"是无意义的。
            if sp is not None:
                # 看**衷减段本身**（峰值到过零）期间设定值是否居中。
                # 用 3° 宽容：遥控器回中本身有几度死区。
                lo = max(0, peak_i)
                hi = min(sp.size, j + 1)
                if hi > lo and np.nanmax(np.abs(sp[lo:hi])) > 3.0:
                    i = max(j, i + 1)
                    continue
            episodes.append((peak_i, j))
        i = max(j, i + 1)
    return episodes


def analyse_returns(axis, which="roll"):
    """量化回正过程：跨过每个角度台阶各花多少时间。

    判据：若是指数收敛，跨越 5->3->2->1->0.5 各段的时间应大致**相同**
    （每段角度比都接近 1.6~2，故对数时间接近）；
    若越接近水平越慢（时间显著拉长），说明存在刹车/死区。
    """
    ang = axis["angle_" + which]
    t = axis["t"]
    a = np.nan_to_num(ang, nan=0.0)
    rows = []

    for peak_i, end_i in find_returns(axis, which):
        sign = 1.0 if a[peak_i] > 0 else -1.0
        mag = a * sign                     # 统一成正向衰减
        peak_mag = float(mag[peak_i])

        # 从峰值往后，依次找跨过每个台阶的时刻（单调下降才算）
        seg = {}
        prev_t = t[peak_i]
        for lvl in RETURN_LEVELS:
            hit = None
            for k in range(peak_i, min(end_i + 500, mag.size)):
                if mag[k] <= lvl:
                    hit = k
                    break
            if hit is None:
                continue
            seg[lvl] = (t[hit], t[hit] - prev_t)
            prev_t = t[hit]

        row = {
            "t_peak_ms": t[peak_i] * 1000.0,
            "axis": which,
            "peak_deg": peak_mag,
            "sign": sign,
        }
        for lvl in RETURN_LEVELS:
            row["t_%g" % lvl] = seg[lvl][0] if lvl in seg else float("nan")
            row["d_%g" % lvl] = seg[lvl][1] if lvl in seg else float("nan")
        rows.append(row)

    return rows


def summarise_returns(rows):
    """汇总多段回正：取各段的**中位数**，避免单次异常。"""
    out = {}
    if not rows:
        return out
    out["n"] = len(rows)
    for lvl in RETURN_LEVELS:
        key = "d_%g" % lvl
        vals = [r[key] for r in rows if np.isfinite(r.get(key, float("nan")))]
        if vals:
            out[key] = float(np.median(vals)) * 1000.0
    peaks = [r["peak_deg"] for r in rows]
    out["peak_deg"] = float(np.median(peaks))
    return out


# --------------------------------------------------------------------------
# 3. 诊断与建议
# --------------------------------------------------------------------------
def diagnose_return_tail(summary):
    """根据回正分段耗时判断是否存在"小角度拖尾"。

    指数收敛下，从 5° 到 1° 用时 = tau*ln(5)，从 1° 到 0.5° 用时 = tau*ln(2)。
    两者理论比 = ln(5)/ln(2) = 2.32。若实测比值明显大于此值，说明尾段被拖慢。
    """
    notes = []
    d3 = summary.get("d_3")
    d2 = summary.get("d_2")
    d1 = summary.get("d_1")
    d05 = summary.get("d_0.5")
    # 5->1 的耗时只包含 5->3、3->2、2->1 三段；
    # d_5 是"峰值->5°"，不属于这个区间（原实现把它算进去，会虚高）。
    if d3 and d2 and d1 and d05 and d05 > 0:
        big = d3 + d2 + d1
        ratio = big / d05
        theo = np.log(5.0) / np.log(2.0)
        notes.append(("5->1deg 用时 %.0f ms，1->0.5deg 用时 %.0f ms，比值 %.2f"
                      % (big, d05, ratio),
                      "理论指数收敛比值应为 %.2f" % theo,
                      ratio > theo * 1.8))
    return notes


def report(axis, rows_step, rows_ret, which="roll"):
    """打印文本报告。"""
    print()
    print("=" * 72)
    print("阶跃 / 回正响应分析（%s 轴）" % which)
    print("=" * 72)

    sp = axis["rc_sp_" + which]
    if sp is None:
        print("  本段没有摇杆帧，无法还原设定值 —— 无法分析阶跃。")
        print("  （固件需含 BB_TYPE_RC 记录；若第 6 节摇杆帧为 0 说明固件是旧版）")
        return

    print("  摇杆帧间隔 %.0f ms" % (axis["rc_dt"] * 1000))
    if axis.get("sp_source") == "target":
        print("  设定值来源：日志里记的**实际目标角**（固件新格式）——绝对准确")
    else:
        print("  设定值来源：摇杆按 (stick-1500)/9 换算（旧格式日志）")
        print("  注意：接收机微调(roll_offset)未记录，若设过微调则设定值有恒定偏移；")
        print("        阶跃的**幅度和时刻**仍然准确。")
    print()

    # ---- 设定值阶跃 ----
    print("-- 设定值阶跃（打杆跟踪）---------------------------------------")
    taps = find_taps(sp)
    if not rows_step:
        if taps:
            print("  检测到 %d 次**点动**（打杆后很快回中），但没有 >=%.0f° 且保持 %.1f 秒的真阶跃。"
                  % (len(taps), STEP_MIN_DEG, STEP_HOLD_S))
            print("  点动只有脉冲激励，2 秒后飞机已跟着回中，终值是 0，")
            print("  用它算上升时间/超调/稳态误差没有意义，所以不做指标。")
        else:
            print("  未检测到 >=%.0f° 的设定值变化。" % STEP_MIN_DEG)
        print()
        print("  想测阶跃响应，请这样飞（可在地面用手代替，但要有真实推力）：")
        print("    1) 悬停稳定 2 秒（摇杆回中）")
        print("    2) %s 杆**快速打到固定位置并保持 1.5 秒**（别回中）" % which)
        print("    3) 再回中，停 1 秒")
        print("    4) 重复 3~5 次，幅度用 %d%% / %d%% 各做一次" % (30, 60))
    else:
        print("   #  时刻ms  保持ms  设定值   实际到达   稳态误差  上升10-90  超调   峰值ms  稳定±5%  一阶tau")
        for k, r in enumerate(rows_step, 1):
            print("  %2d  %7.0f  %5.0f  %+7.2f  %+7.2f  %+7.2f   %6s   %5.1f%%  %6.0f  %6s  %6s"
                  % (k, r["t_step_ms"], r["hold_ms"], r["sp_deg"], r["to_deg"], r["ss_err"],
                     ("%.0f" % r["rise_ms"]) if np.isfinite(r["rise_ms"]) else "-",
                     r["overshoot_pct"], r["t_peak_ms"],
                     ("%.0f" % r["settle_ms"]) if np.isfinite(r["settle_ms"]) else "-",
                     ("%.0f" % r["tau_ms"]) if np.isfinite(r["tau_ms"]) else "-"))
        # 汇总
        taus = [r["tau_ms"] for r in rows_step if np.isfinite(r["tau_ms"])]
        if taus:
            print("  一阶时间常数中位数: %.0f ms  (对应闭环带宽约 %.2f Hz)"
                  % (np.median(taus), 1000.0 / (2 * np.pi * np.median(taus))))
    print()

    # ---- 回正过程 ----
    print("-- 回正过程（松杆/被扰后自主回正）-------------------------------")
    if not rows_ret:
        print("  未检测到 >=%.0f° 的自由回正过程。" % RETURN_START_DEG)
        print("  （本项只分析“摇杆回中、飞机自己往回扳”的那段；")
        print("     打杆期间的姿态变化不算。要测它请这样飞：")
        print("       打杆让飞机偏 8~15° -> **摇杆完全松开** -> 等 2 秒让它自己回正")
        print("     每次都要真的松手，否则测不到回正特性。）")
    else:
        s = summarise_returns(rows_ret)
        print("  共 %d 段回正，典型峰值 %.1f°" % (s.get("n", 0), s.get("peak_deg", 0)))
        print("  跨越各角度台阶的耗时（中位数）:")
        v = s.get("d_%g" % RETURN_LEVELS[0])
        if v:
            print("    峰值 %.1f° -> %.1f°      %6.0f ms  （起始段，含加速）"
                  % (s.get("peak_deg", 0), RETURN_LEVELS[0], v))
        for a, b in zip(RETURN_LEVELS[:-1], RETURN_LEVELS[1:]):
            # d_X 是"从上一个台阶降到 X°"的耗时，所以
            # "a -> b"这一行取的是 d_b（目标台阶），不是 d_a。
            # 原来取错成 d_a，数值全部错位一行。
            v = s.get("d_%g" % b)
            if v:
                print("    %.1f° -> %.1f°            %6.0f ms" % (a, b, v))
        print()
        for text, extra, bad in diagnose_return_tail(s):
            print("  %s %s（%s）" % ("[!]" if bad else "[OK]", text, extra))
            if bad:
                print("        -> 尾段明显被拖慢，符合'小角度回正慢'；"
                      "优先检查 outer.kd 是否在刹车、inner.ki 是否在拖尾。")
    print()


def _next_level(lvl):
    for a, b in zip(RETURN_LEVELS[:-1], RETURN_LEVELS[1:]):
        if a == lvl:
            return b
    return 0.0


def export_csv(path, axis, rows_step, rows_ret):
    """把阶跃窗口与回正统计导出为 CSV，便于外部画图。"""
    lines = []
    lines.append("# step_response")
    lines.append("kind,t_ms,setpoint_deg,roll_deg,pitch_deg,gyro_roll_dps,gyro_pitch_dps")
    for r in rows_step:
        i = int(r["t_step_ms"])
        a0, a1 = max(0, i - int(STEP_PRE_S * 1000)), min(axis["n_data"], i + int(STEP_POST_S * 1000))
        for k in range(a0, a1):
            sp = axis["rc_sp_roll"][k] if axis["rc_sp_roll"] is not None else float("nan")
            lines.append("step,%d,%.4f,%.4f,%.4f,%.4f,%.4f" % (
                k, sp, axis["angle_roll"][k], axis["angle_pitch"][k],
                axis["gyro_roll"][k], axis["gyro_pitch"][k]))
    lines.append("")
    lines.append("# return_summary")
    lines.append("t_peak_ms,peak_deg," + ",".join("d_%g" % l for l in RETURN_LEVELS))
    for r in rows_ret:
        lines.append("%.0f,%.4f," % (r["t_peak_ms"], r["peak_deg"]) +
                     ",".join(("%.4f" % r["d_%g" % l]) if np.isfinite(r["d_%g" % l]) else ""
                              for l in RETURN_LEVELS))
    with open(path, "w", encoding="utf-8", newline="") as fp:
        fp.write("\n".join(lines) + "\n")
    return len(lines)
