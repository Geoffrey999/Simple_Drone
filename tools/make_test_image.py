"""合成一个"左偏"飞行数据镜像，用于端到端验证上位机管线。

模拟的物理场景：
  * 四轴缓慢左倾（roll 向负方向漂），40 秒累计约 -8deg
  * 同时 accel_confidence 从 1.0 逐渐掉到 0（模拟振动污染重力参考）
  * 电机差动随之增大（控制器在努力往回掰）

生成后可直接分析：
    python bb.py analyze synthetic_flash.bin --plot
"""

from __future__ import annotations

import argparse
import math
import os

import numpy as np

from bb_format import (
    FRAME_SIZE, FRAMES_PER_SECTOR, HEADER_STRUCT, MAGIC_FRAME, MAGIC_HEADER,
    SECTOR_COUNT, SECTOR_DATA_OFF, SECTOR_SIZE, TYPE_DATA, TYPE_DIAG,
)
from selftest_format import build_frame, build_header

SECONDS = 40
RATE_HZ = 1000
DIAG_DIV = 64


def synth(roll_end_deg=-8.0, conf_end=0.0, seed=1):
    """按 1kHz 生成模拟数据；返回 (frames, meta)。"""
    rng = np.random.default_rng(seed)
    n = SECONDS * RATE_HZ
    t = np.arange(n) / float(RATE_HZ)

    # --- 姿态：roll 缓慢左倾（负方向），pitch 基本不动 ---
    roll = roll_end_deg * (t / t[-1])
    pitch = 0.6 * np.sin(2 * math.pi * 0.15 * t)          # 轻微摆动
    yaw = 1.2 * (t / t[-1])                               # 缓慢偏航

    # --- 陀螺：真实角速度 + 一个随时间增长的零偏 ---
    true_roll_rate = np.gradient(roll, t)
    bias_y = 0.05 + 0.30 * (t / t[-1])                    # 0.05 -> 0.35 deg/s
    gyro_y = true_roll_rate + bias_y + rng.normal(0, 0.15, n)
    gyro_x = rng.normal(0, 0.15, n)
    gyro_z = np.gradient(yaw, t) + rng.normal(0, 0.15, n)

    # --- 加速度计：重力在机体轴上的投影 + 噪声 ---
    rr = np.radians(roll)
    pp = np.radians(pitch)
    ax = -np.sin(rr) * np.cos(pp) + rng.normal(0, 0.012, n)
    ay = np.sin(pp) + rng.normal(0, 0.012, n)
    az = np.cos(rr) * np.cos(pp) + rng.normal(0, 0.012, n)

    # --- 电机：维持高度 + 对抗左倾的差动（左侧加大） ---
    hover = 1800.0
    diff = 260.0 * (t / t[-1])                            # 差动随时间增大
    m = np.empty((n, 4))
    m[:, 0] = hover + diff + rng.normal(0, 8, n)          # m1 左后
    m[:, 1] = hover + diff + rng.normal(0, 8, n)          # m2 左前
    m[:, 2] = hover - diff + rng.normal(0, 8, n)          # m3 右前
    m[:, 3] = hover - diff + rng.normal(0, 8, n)          # m4 右后
    throttle = np.full(n, hover)

    # --- 置信度：从 1.0 掉到 conf_end ---
    conf = 1.0 + (conf_end - 1.0) * np.clip((t - 8.0) / 20.0, 0.0, 1.0)

    frames = []
    for i in range(n):
        is_diag = (i % DIAG_DIV) == 0
        if is_diag:
            # 诊断帧槽位 = conf 档位 | period(100us) << 8
            conf_code = int(round(conf[i] * 3.0))
            conf_code = max(0, min(3, conf_code))
            period_code = int(round(1150.0 / 100.0))      # 1150us
            slot = (conf_code & 0xFF) | ((period_code & 0xFF) << 8)
            ftype = TYPE_DIAG
        else:
            # 序号是 16bit 无符号，必须用 & 0xFFFF；
            # 用 & 0x7FFF 会把它截成 15 位，32880 会变成 112（曾经踩过）
            slot = i & 0xFFFF
            ftype = TYPE_DATA

        frames.append(build_frame(
            ftype=ftype, slot=slot, flags=0x03,
            roll=roll[i], pitch=pitch[i], yaw=yaw[i],
            gyro=(gyro_x[i], gyro_y[i], gyro_z[i]),
            accel=(ax[i], ay[i], az[i]),
            motors=tuple(m[i]), throttle=throttle[i],
            battery=95 - int(20 * t[i] / t[-1]),
        ))

    meta = {"roll_end": roll[-1], "bias_end": bias_y[-1], "conf_end": conf[-1],
            "diff_end": diff[-1]}
    return frames, meta


def write_image(path: str, frames, first_sector_logical: int = 100):
    """写成完整 16MB 镜像：把帧铺满若干连续扇区，其余保持 0xFF。"""
    img = bytearray(b"\xFF" * (SECTOR_SIZE * SECTOR_COUNT))
    per = FRAMES_PER_SECTOR
    n_sectors_used = (len(frames) + per - 1) // per

    for s in range(n_sectors_used):
        base = s * SECTOR_SIZE
        chunk = frames[s * per:(s + 1) * per]
        img[base:base + HEADER_STRUCT.size] = build_header(
            first_sector_logical + s, len(chunk))
        d0 = base + SECTOR_DATA_OFF
        for k, fr in enumerate(chunk):
            if len(fr) != FRAME_SIZE:
                raise ValueError(
                    f"第 {s*per+k} 帧长度 {len(fr)} != {FRAME_SIZE}，"
                    f"生成器与格式定义不一致")
            img[d0 + k * FRAME_SIZE: d0 + (k + 1) * FRAME_SIZE] = fr

    # 回读自检：逐扇区确认首帧能正确解出，避免"写了但写错位置"静默失败
    written = 0
    for s in range(n_sectors_used):
        off = s * SECTOR_SIZE + SECTOR_DATA_OFF
        got = bytes(img[off:off + 2])
        want = frames[s * per][:2]
        if got != want:
            raise ValueError(f"扇区 {s} 首帧写入位置校验失败：{got.hex()} != {want.hex()}")
        written += len(frames[s * per:(s + 1) * per])

    with open(path, "wb") as fp:
        fp.write(img)

    return n_sectors_used


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="合成黑匣子测试镜像")
    ap.add_argument("-o", "--output", default="synthetic_flash.bin")
    ap.add_argument("--roll-end", type=float, default=-8.0, help="结束时的 roll(度)")
    ap.add_argument("--conf-end", type=float, default=0.0, help="结束时的置信度")
    args = ap.parse_args(argv)

    frames, meta = synth(args.roll_end, args.conf_end)
    used = write_image(args.output, frames)

    size = os.path.getsize(args.output)
    print(f"已生成 {args.output}  ({size/1048576:.1f} MB, {used} 个扇区)")
    print(f"  帧数       : {len(frames)} ({len(frames)/RATE_HZ:.0f} s @1kHz)")
    print(f"  roll 终值  : {meta['roll_end']:+.2f}deg")
    print(f"  陀螺偏置终值: {meta['bias_end']:.3f} deg/s")
    print(f"  置信度终值 : {meta['conf_end']:.2f}")
    print(f"  电机差动终值: {meta['diff_end']:.0f} PWM")
    print()
    print(f"下一步： python bb.py analyze {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
