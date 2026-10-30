"""格式环回自检：按 C 侧的写法合成 Flash 镜像，再用解析器解回来比对。

只要固件与上位机的字段顺序/缩放/位打包有任何不一致，这里就会失败。
用法： python selftest_format.py
"""

import struct
import sys

import numpy as np

from bb_format import (
    FRAME_SIZE, FRAMES_PER_SECTOR, HEADER_STRUCT, MAGIC_FRAME, MAGIC_HEADER,
    SECTOR_COUNT, SECTOR_DATA_OFF, SECTOR_SIZE, TYPE_DATA, TYPE_DIAG, TYPE_RC,
    TYPE_ALT, CONF_LEVELS, LSB_ACCEL, LSB_ANGLE, LSB_GYRO, MOTOR_FULL,
    MOTOR_SCALE, THROTTLE_SCALE, _decode_frame, parse_flash, unpack_motors,
)

FRAME_STRUCT = struct.Struct("<HBBHhhhhhhhhhIBBxx")


def encode_motors(motors_pwm, throttle_pwm):
    """与 blackbox.c 的打包逻辑逐位一致。"""
    packed = 0
    for i, v in enumerate(motors_pwm):
        v = min(int(v), 4200)
        v = (v * 127 + 2100) // 4200          # 四舍五入到 0..127
        v = min(v, 127)
        packed |= (v << (7 * i))
    th = max(0.0, min(float(throttle_pwm), 4200.0))
    m = int(th * 15.0 / 4200.0 + 0.5)
    m = min(m, 15)
    packed |= (m << 28)
    return packed


def build_frame(*, magic=MAGIC_FRAME, ftype=TYPE_DATA, flags=0, slot=0,
                roll=0.0, pitch=0.0, yaw=0.0,
                gyro=(0.0, 0.0, 0.0), accel=(0.0, 0.0, 1.0),
                motors=(0, 0, 0, 0), throttle=0.0, battery=0, reserved=0):
    """按 C 侧 netint16 转换(带饱和与四舍五入)编码一帧。

    seq 字段是无符号 16bit：这里显式做 & 0xFFFF，否则 seq 超过 32767 时
    struct.pack 会因越界抛异常或写出错误字节（这正是曾经踩过的坑）。
    """
    def i16(v, lsb):
        x = v / lsb
        x = max(-32768.0, min(32767.0, x))
        return int(x + 0.5) if x >= 0 else int(x - 0.5)

    return FRAME_STRUCT.pack(
        magic & 0xFFFF, ftype & 0xFF, flags & 0xFF, int(slot) & 0xFFFF,
        i16(roll, LSB_ANGLE), i16(pitch, LSB_ANGLE), i16(yaw, LSB_ANGLE),
        i16(gyro[0], LSB_GYRO), i16(gyro[1], LSB_GYRO), i16(gyro[2], LSB_GYRO),
        i16(accel[0], LSB_ACCEL), i16(accel[1], LSB_ACCEL), i16(accel[2], LSB_ACCEL),
        encode_motors(motors, throttle), battery & 0xFF, reserved & 0xFF,
    )


def build_header(sector_index, frame_offset, boot_count=7):
    return HEADER_STRUCT.pack(MAGIC_HEADER, 0xA5, sector_index,
                              frame_offset, 0xFFFF, boot_count,
                              0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF)


def build_flash(n_sectors=SECTOR_COUNT, filled_sectors=2, frames_in_last=37):
    """造一个"烧过两三个扇区"的镜像。

    序号 seq 跨扇区连续递增（与固件 bb_frame_seq 的行为一致），
    这是上位机判断"同一次记录是否横跨扇区"的依据。
    """
    img = bytearray(b"\xFF" * (SECTOR_SIZE * n_sectors))
    seq = 0
    for s in range(filled_sectors):
        base = s * SECTOR_SIZE
        n_frames = FRAMES_PER_SECTOR if s < filled_sectors - 1 else frames_in_last
        img[base:base + HEADER_STRUCT.size] = build_header(s, n_frames)
        d0 = base + SECTOR_DATA_OFF
        for k in range(n_frames):
            # roll 随帧号增长，模拟"飞得越久越左偏"
            slot = seq if (seq % 64) != 0 else ((seq // 64 % 4) << 0)  # 诊断帧槽位另作他用
            ftype = TYPE_DATA if (seq % 64) != 0 else TYPE_DIAG
            payload = build_frame(
                ftype=ftype, slot=slot,
                roll=0.05 * seq, pitch=-0.02 * seq, yaw=1.5 * seq,
                gyro=(0.1, -0.2, 0.3), accel=(0.01, -0.02, 0.999),
                motors=(1200 + k, 1300 + k, 1400 + k, 1500 + k),
                throttle=1800.0, battery=88,
            )
            img[d0 + k * FRAME_SIZE: d0 + (k + 1) * FRAME_SIZE] = payload
            seq += 1
    return bytes(img)


def approx(a, b, tol):
    return abs(float(a) - float(b)) <= tol


def main():
    failures = []

    def check(name, cond, detail=""):
        if cond:
            print(f"  [OK]   {name}")
        else:
            print(f"  [FAIL] {name}  {detail}")
            failures.append(name)

    print("== 1. 位打包/解包一致性 ==")
    for motors, thr in [((0, 0, 0, 0), 0.0),
                        ((4200, 0, 2100, 1050), 4200.0),
                        ((1234, 2345, 3456, 4199), 1900.0)]:
        word = encode_motors(motors, thr)
        dec = unpack_motors(word)
        got = dec[:4]
        ok = all(approx(g, m, MOTOR_SCALE) for g, m in zip(got, motors))
        ok = ok and approx(dec[4], thr, THROTTLE_SCALE)
        check(f"motors={motors} thr={thr} -> {[round(x) for x in got]} "
              f"thr'={round(dec[4])}", ok,
              f"word=0x{word:08X}")
        # 确认 4bit 油门字段没有挤掉第 4 路电机
        check("  motor4 未被油门字段覆盖",
              (word >> 21) & 0x7F == min(int(motors[3] * 127 + 2100) // 4200, 127),
              f"motor4_field={(word >> 21) & 0x7F}")

    print("== 2. 单帧编解码 ==")
    payload = build_frame(slot=1234, roll=12.34, pitch=-5.67, yaw=170.25,
                          gyro=(1.5, -2.5, 3.5), accel=(0.1, -0.2, 0.97),
                          motors=(1000, 2000, 3000, 4000), throttle=2500.0,
                          battery=73, flags=0x03)
    f = _decode_frame(payload, 0)
    check("magic/type", f and f["type"] == TYPE_DATA)
    check("roll", approx(f["angle"][0], 12.34, LSB_ANGLE))
    check("pitch", approx(f["angle"][1], -5.67, LSB_ANGLE))
    check("yaw", approx(f["angle"][2], 170.25, LSB_ANGLE))
    check("gyro_y", approx(f["gyro"][1], -2.5, LSB_GYRO))
    check("accel_z", approx(f["accel"][2], 0.97, LSB_ACCEL))
    check("motors[3]", approx(f["motor"][3], 4000, MOTOR_SCALE))
    check("throttle", approx(f["throttle"], 2500, THROTTLE_SCALE))
    check("battery", f["battery"] == 73)
    check("flags", f["flags"] == 0x03)

    print("== 3. 诊断帧 ==")
    # 诊断帧: slot 低字节 = conf 档位, 高字节 = period(100us)
    dpayload = build_frame(ftype=TYPE_DIAG, slot=(12 << 8) | 3,
                           roll=1.0, pitch=2.0, yaw=3.0)
    df = _decode_frame(dpayload, 0)
    check("type=DIAG", df and df["type"] == TYPE_DIAG)
    check(f"conf -> {df['conf']}", approx(df["conf"], CONF_LEVELS[3], 1e-6))
    check(f"period -> {df['period']}us", approx(df["period"], 12 * 100.0, 1e-6))

    print("== 3b. 遥控摇杆帧 ==")
    # 摇杆帧的 int16 槽位是**原始值**（不按物理量缩放），所以直接打包
    rcpayload = FRAME_STRUCT.pack(
        MAGIC_FRAME, TYPE_RC, 0, 77,
        1600, 1400, 1500,      # roll / pitch / yaw 摇杆
        1700,                  # throttle 摇杆
        2000, 2000,            # arm / mode
        3,                     # flight_status
        117, -120,             # 目标角 roll/pitch（×100 存）
        encode_motors((1500, 1500, 1500, 1500), 1700.0),
        80,
        0xB7,                  # reserved=0xB7 -> 新格式（槽位存目标角）
    )
    rf = _decode_frame(rcpayload, 0)
    check("type=RC", rf and rf["type"] == TYPE_RC)
    check(f"摇杆 roll -> {rf['rc']['roll']}", rf["rc"]["roll"] == 1600)
    check(f"摇杆 pitch -> {rf['rc']['pitch']}", rf["rc"]["pitch"] == 1400)
    check(f"摇杆 throttle -> {rf['rc']['throttle']}", rf["rc"]["throttle"] == 1700)
    check(f"目标角 roll -> {rf['rc']['target_roll']:.2f}",
          approx(rf["rc"]["target_roll"], 1.17, 1e-6))
    check(f"目标角 pitch -> {rf['rc']['target_pitch']:.2f}",
          approx(rf["rc"]["target_pitch"], -1.20, 1e-6))
    check(f"格式标记 reserved -> 0x{rf['rc']['rres']:02X}",
          rf["rc"]["rres"] == 0xB7)

    print("== 3c. 高度帧（TYPE_ALT，50Hz）==")
    # 槽位含义：roll=baro×0.01m pitch=alt×0.01m yaw=vvel×0.01m/s
    #           gyro_x=innov×1e-4m gyro_y=bias×1e-4m/s² gyro_z=baro_dt×0.1ms
    #           accel_x=alt_ref×0.01m accel_y=气压×10Pa accel_z=thr_corr×1
    # ★ 气压槽是 10 Pa 一档（=0.1 hPa），不是 Pa 也不是 hPa 直存：
    #   固件里 bmp_data.pressure 的单位是 **Pa**（典型 101325），
    #   按 10Pa 存进 int16 得 10132，解回来再 /10 就是 1013.2 hPa。
    altpayload = FRAME_STRUCT.pack(
        MAGIC_FRAME, TYPE_ALT, 0x09, 4242,
        123, -45, 78,          # baro 1.23m / alt -0.45m / vvel 0.78m/s
        5000, 120, 200,        # innov 0.5m / bias 0.012 / baro_dt 20ms
        7, 10132, -35,         # alt_ref 0.07m / 101320Pa(=1013.2hPa) / thr_corr -35
        # motors 槽 = 定高基准油门：低16位 1703 PWM，bit31=hover_valid
        1703 | (1 << 31),
        66,
        0xC3,                  # reserved=0xC3 -> 高度帧格式标记
    )
    af = _decode_frame(altpayload, 0)
    check("type=ALT", af and af["type"] == TYPE_ALT and af["alt"] is not None)
    if af and af["alt"]:
        a = af["alt"]
        check(f"baro -> {a['baro']:.2f}m", approx(a["baro"], 1.23, 2 * LSB_ANGLE))
        check(f"alt  -> {a['alt']:.2f}m", approx(a["alt"], -0.45, 2 * LSB_ANGLE))
        check(f"vvel -> {a['vvel']:.2f}m/s", approx(a["vvel"], 0.78, 2 * LSB_ANGLE))
        check(f"innov -> {a['innov']:.4f}m", approx(a["innov"], 0.5, 1e-4))
        check(f"bias -> {a['bias']:.4f}", approx(a["bias"], 0.012, 1e-4))
        check(f"baro_dt -> {a['baro_dt_ms']:.1f}ms", approx(a["baro_dt_ms"], 20.0, 0.11))
        check(f"pressure -> {a['pressure_hpa']:.1f}hPa",
              approx(a["pressure_hpa"], 1013.2, 0.11))
        check(f"thr_corr -> {a['thr_corr']:.0f}", approx(a["thr_corr"], -35, 1.0))
        check(f"基准油门 -> {a['hover_base']:.0f}", approx(a["hover_base"], 1703, 1.0))
        check("hover_valid 位", a["hover_valid"] == 1)
        check(f"格式标记 -> 0x{a['mark']:02X}", a["mark"] == 0xC3)
        check("status 原样透传", a["status"] == 0x09)

    print("== 4. 空白帧识别 ==")
    blank = b"\xFF" * FRAME_SIZE
    check("全 0xFF 判为空", _decode_frame(blank, 0) is None)

    print("== 5. 整片解析与定位最近记录 ==")
    img = build_flash()
    log = parse_flash(img)
    check("帧总数", log.seq.size == SECTOR_COUNT * FRAMES_PER_SECTOR,
          f"got {log.seq.size}")
    check("有效扇区头 2 个",
          len([h for h in log.headers if h.valid]) == 2)
    run = log.latest_run()
    check(f"最近一段帧数 = {run.size} (期望 120+37=157)", run.size == 157,
          f"got {run.size}")
    if run.size:
        check("首帧 roll=0.00", approx(log.roll[run[0]], 0.0, LSB_ANGLE + 1e-6))
        # 量化后 7.80 会落在 780 或 781 个 LSB 上，用 2 个 LSB 容差
        check("末帧 roll≈7.80", approx(log.roll[run[-1]], 7.80, 2 * LSB_ANGLE),
              f"got {log.roll[run[-1]]}")
        # 跨扇区连续性：第 121 帧(索引120)应是第 2 个扇区的第 1 帧，序号=120
        if run.size > FRAMES_PER_SECTOR:
            check("跨扇区序号连续(第121帧 seq=120)",
                  log.seq[run[FRAMES_PER_SECTOR]] == 120,
                  f"got {log.seq[run[FRAMES_PER_SECTOR]]}")

    print("== 6. 截断镜像不应崩溃 ==")
    try:
        log2 = parse_flash(img[:SECTOR_SIZE * 3 // 2])
        check("截断解析", log2.seq.size > 0)
    except Exception as exc:                      # noqa: BLE001
        check("截断解析", False, repr(exc))

    print("== 7. 序号超过 32767 时的连续性(无符号回绕) ==")
    # 这是曾经踩过的坑：seq 若按有符号解析，32767 之后变负，
    # 跨扇区连续性判断会整段失效，只能解析出最后一个扇区。
    # 注意必须构造完整的 16MB 镜像（parse_flash 靠长度区分"整片镜像"与"纯帧流"）。
    sectors_needed = 280                       # 280*120 = 33600 > 32767，跨过回绕点
    img2 = bytearray(b"\xFF" * (SECTOR_SIZE * SECTOR_COUNT))
    seq = 0
    for s in range(sectors_needed):
        base = s * SECTOR_SIZE
        img2[base:base + HEADER_STRUCT.size] = build_header(s, FRAMES_PER_SECTOR)
        d0 = base + SECTOR_DATA_OFF
        for k in range(FRAMES_PER_SECTOR):
            ftype = TYPE_DATA if (seq % 64) != 0 else TYPE_DIAG
            slot = (seq & 0xFFFF) if ftype == TYPE_DATA else ((seq // 64 % 4) & 0xFF)
            payload = build_frame(ftype=ftype, slot=slot,
                                  roll=0.0, pitch=0.0, yaw=0.0,
                                  motors=(1000, 1000, 1000, 1000), throttle=1500.0)
            img2[d0 + k * FRAME_SIZE: d0 + (k + 1) * FRAME_SIZE] = payload
            seq += 1

    log3 = parse_flash(bytes(img2))
    run3 = log3.latest_run()
    expect = sectors_needed * FRAMES_PER_SECTOR
    check(f"跨过 32767 后仍能连续解析 (期望 {expect} 帧)", run3.size == expect,
          f"got {run3.size}")
    if run3.size:
        check("最高序号 > 32767", int(log3.seq[run3[-1]]) > 32767,
              f"got {log3.seq[run3[-1]]}")

    print("== 8. 高度帧(ALT)不破坏跨扇区序号连续性 ==")
    # 高度帧每 20 个槽位插一帧，且**不递增 bb_frame_seq**（借用当前序号）。
    # 如果上位机还按"槽位位置差"倒推序号，每扇区就会差 6 帧，
    # _is_same_run() 会失败 → latest_run() 每次只返回最后一个扇区。
    # 这里用 2 个满扇区锁住该行为。
    img4 = bytearray(b"\xFF" * (SECTOR_SIZE * SECTOR_COUNT))
    seq = 0
    alt_slots = 0
    for s in range(2):
        base = s * SECTOR_SIZE
        img4[base:base + HEADER_STRUCT.size] = build_header(s, FRAMES_PER_SECTOR)
        d0 = base + SECTOR_DATA_OFF
        for k in range(FRAMES_PER_SECTOR):
            if k % 20 == 0:                      # 高度帧：6 帧/扇区
                # build_frame 会按 LSB 缩放，所以这里直接给物理量
                payload = build_frame(ftype=TYPE_ALT, slot=seq, flags=0x09,
                                      roll=1.23, pitch=-0.45, yaw=0.78,
                                      reserved=0xC3)
                img4[d0 + k * FRAME_SIZE: d0 + (k + 1) * FRAME_SIZE] = payload
                alt_slots += 1
                continue                         # ★ 不递增 seq
            if k % 32 == 8:                      # 摇杆帧：2 帧/扇区，占序号
                payload = build_frame(ftype=TYPE_RC, slot=seq, roll=1500,
                                      pitch=1500, yaw=1500,
                                      reserved=0xB7)
                img4[d0 + k * FRAME_SIZE: d0 + (k + 1) * FRAME_SIZE] = payload
                seq += 1
                continue
            ftype = TYPE_DATA if (seq % 64) != 0 else TYPE_DIAG
            slot = (seq & 0xFFFF) if ftype == TYPE_DATA else ((seq // 64 % 4) & 0xFF)
            payload = build_frame(ftype=ftype, slot=slot, roll=0.0,
                                  motors=(1000, 1000, 1000, 1000), throttle=1500.0)
            img4[d0 + k * FRAME_SIZE: d0 + (k + 1) * FRAME_SIZE] = payload
            seq += 1

    log4 = parse_flash(bytes(img4))
    run4 = log4.latest_run()
    check(f"高度帧数 = {alt_slots} (期望 12)", alt_slots == 12, f"got {alt_slots}")
    check(f"两扇区仍被判为同一次记录 (期望 {2 * FRAMES_PER_SECTOR} 帧)",
          run4.size == 2 * FRAMES_PER_SECTOR, f"got {run4.size}")
    alt_idx = run4[log4.ftype[run4] == TYPE_ALT]
    check(f"高度帧被解析出来 ({alt_idx.size} 帧)", alt_idx.size == 12,
          f"got {alt_idx.size}")
    if alt_idx.size:
        check("融合高度解码正确",
              approx(log4.alt_fused[alt_idx[0]], -0.45, 2 * LSB_ANGLE),
              f"got {log4.alt_fused[alt_idx[0]]}")
        check("气压高度解码正确",
              approx(log4.alt_baro[alt_idx[0]], 1.23, 2 * LSB_ANGLE),
              f"got {log4.alt_baro[alt_idx[0]]}")
        check("baro_ok 状态位可读", bool(log4.alt_ok()[alt_idx[0]]))
        # 高度帧绝不能污染姿态/陀螺统计
        check("高度帧没写进 angle 数组",
              np.all(np.isnan(log4.angle[alt_idx])), "angle 被 ALT 槽位污染")
        data_idx = run4[log4.ftype[run4] == TYPE_DATA]
        check("数据帧统计未被高度帧影响",
              data_idx.size > 0 and abs(float(np.nanmean(log4.roll[data_idx]))) < 0.01,
              f"mean roll={np.nanmean(log4.roll[data_idx])}")

    print()
    if failures:
        print(f"[FAIL] {len(failures)} 项失败：{failures}")
        return 1
    print("[PASS] 全部通过：固件与上位机格式一致")
    return 0


if __name__ == "__main__":
    sys.exit(main())
