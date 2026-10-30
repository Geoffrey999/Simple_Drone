"""黑匣子上位机命令行工具。

用法示例：
    # 查看可用串口
    python bb.py ports

    # 查询设备状态（是否识别到 Flash、当前写指针、丢帧数...）
    python bb.py status --port COM3

    # 全片擦除（约 200s，需处于未解锁状态）
    python bb.py erase --port COM3

    # 把整片 16MB 拉到本地
    python bb.py dump --port COM3 -o flash.bin

    # 扫描扇区索引头
    python bb.py scan --port COM3

    # 离线解析 + 分析 + 出图（不需要串口/pyserial）
    python bb.py analyze flash.bin --plot --csv flight.csv
  python bb.py step    flash.bin            # 阶跃/回正响应（调 PID）
  python bb.py stepgo --axis roll --amp 8   # 注入方波做阶跃测试

依赖：串口相关命令需要 pyserial；绘图需要 matplotlib。
    pip install pyserial matplotlib
"""

from __future__ import annotations

import argparse
import os
import re
import sys

# --- 控制台编码安全网 -------------------------------------------------------
# 中文 Windows 控制台是 GBK(cp936)，打印 checkmark/warning-sign/micro 这类字符会抛
# UnicodeEncodeError，表现为"脚本直接闪退、连错误信息都看不到"。
# 这里把所有输出流改成 errors="replace"：不支持的字符降级成 '?'
# 而不是崩溃。源码里也已统一改用 ASCII 标记。
for _stream in (sys.stdout, sys.stderr):
    try:
        _stream.reconfigure(errors="replace")     # type: ignore[attr-defined]
    except Exception:
        pass

# 允许以 tools/ 为工作目录或从项目根目录调用
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bb_format  # noqa: E402
from bb_format import FLASH_BYTES  # noqa: E402


def cmd_ports(args) -> int:
    from bb_serial import available_ports, HAVE_SERIAL
    if not HAVE_SERIAL:
        print("未安装 pyserial。请执行： pip install pyserial")
        return 1
    ports = available_ports()
    if not ports:
        print("未发现任何串口。")
        return 1
    print("可用串口：")
    for p in ports:
        print("  " + p)
    print("\n提示：飞控的黑匣子通道是 USART3(PB10=TX, PB11=RX)，默认 460800 8N1。")
    print(f"下一步： python bb.py status --port {ports[0]}")
    return 0


def _print_banner() -> None:
    """不带参数时：打印帮助，并尽量把"下一步该敲什么"直接给出来。"""
    parser = build_parser()
    parser.print_help()

    print()
    print("=" * 66)
    print("下一步怎么走")
    print("=" * 66)

    from bb_serial import HAVE_SERIAL
    if not HAVE_SERIAL:
        print("串口功能需要 pyserial：  pip install pyserial")
        print("离线分析不需要串口，可以直接用：")
        print("  python bb.py analyze <flash镜像.bin> --plot")
        print("  python bb.py selftest        # 不接硬件的格式自检")
        return

    from bb_serial import available_ports
    ports = available_ports()
    if not ports:
        print("没有检测到串口。请确认 USB 转串口已插好、驱动已装。")
        print("也可以插好后重新运行： python bb.py ports")
        return

    print("检测到以下串口：")
    for p in ports:
        print("  " + p)
    if len(ports) == 1:
        print(f"\n只有一路串口，可以直接省略 --port：")
        print("  python bb.py status")
    else:
        print(f"\n用其中一个（不确定哪个是飞控就逐个试 status）：")
        print(f"  python bb.py status --port {ports[0]}")
    print()
    print("完整流程：")
    print("  python bb.py status          # 1. 确认 Flash 识别到 (ok=1)")
    print("  python bb.py dump -o flash.bin   # 2. 飞完拉数据（需未解锁）")
    print("  python bb.py analyze flash.bin --plot   # 3. 分析")


# 别名 -> 标准命令名。argparse 会把别名原样存进 args.cmd，
# 提示用户时最好给标准写法。
_ALIASES = {"st": "status", "d": "dump", "a": "analyze"}


def _resolve_port(args) -> str:
    """--port 缺省时：只有一路串口就自动用它，多路则要求明确指定。"""
    if getattr(args, "port", None):
        return args.port

    from bb_serial import available_ports, HAVE_SERIAL
    if not HAVE_SERIAL:
        raise SystemExit("未安装 pyserial。请执行： pip install pyserial")

    ports = available_ports()
    if not ports:
        raise SystemExit("未检测到串口。插好 USB 转串口后重试，或用 --port 指定。")
    if len(ports) == 1:
        print(f"[自动选择唯一串口 {ports[0]}]")
        return ports[0]

    # 多路串口里优先挑不是 COM1 的（COM1 通常是主板自带串口，不是飞控）
    from bb_serial import pick_default_port
    pick = pick_default_port(ports)
    if pick and pick.upper() != "COM1":
        skipped = [p for p in ports if p != pick]
        print(f"[自动选择 {pick}（跳过 {', '.join(skipped)}；不对请用 --port 指定）]")
        return pick

    cmd = _ALIASES.get(getattr(args, "cmd", ""), getattr(args, "cmd", ""))
    raise SystemExit(
        "检测到多路串口，请用 --port 指定：\n  " + "\n  ".join(
            f"python bb.py {cmd} --port {p}" for p in ports))


def _open(args):
    """按 --port / --baud 打开串口（缺省参数走自动解析）。"""
    from bb_serial import DEFAULT_BAUD, open_port
    port = _resolve_port(args)
    baud = getattr(args, "baud", None) or DEFAULT_BAUD
    return open_port(port, baud=baud)


def cmd_status(args) -> int:
    from bb_serial import parse_stat_line, send_command
    with _open(args) as ser:
        lines = send_command(ser, "BB:STAT", timeout=5.0)

    st = {}
    for ln in lines:
        st = parse_stat_line(ln)
        if st:
            break

    if st:
        if st.get("ok") == 1:
            print("\n[[OK]] 黑匣子就绪：Flash 已识别")
        else:
            print("\n[[X]] ok=0 —— W25Q128 未被识别到，检查 SPI3 接线：")
            print("      PA15=NSS  PC10=SCK  PC11=MISO  PC12=MOSI")
            print("      JEDEC ID 应为 EF 40 18（Winbond W25Q128）")
        if st.get("erasing") == 1:
            print("[i] 正在后台擦除 Flash（约 200s）...")
        elif st.get("eseq", 0) > 0:
            print(f"[i] 擦除已完成过 {st.get('eseq')} 次")

    # ---- 记录质量：丢帧 / RAM 缓冲余量 ----
    # hwm(high water mark) 是固件里环形缓冲的历史最高占用帧数。
    # 它是"Logger 跟不跟得上"最直接的指标：个位数=健康，接近 RING_FRAMES=要丢帧了。
    if st:
        if "hwm" not in st:
            print("\n[i] 固件是旧版（BB:STAT 里没有 hwm 字段），升级后才能看缓冲余量")
        else:
            from bb_format import RING_FRAMES
            rec = int(st.get("rec", 0))
            drop = int(st.get("drop", 0))
            hwm = int(st.get("hwm", 0))
            pct = 100.0 * hwm / float(RING_FRAMES)

            print("\n--- 记录质量（本次解锁以来）---")
            print(f"  已记录 {rec} 帧   丢帧 {drop}   缓冲最高水位 {hwm}/{RING_FRAMES} ({pct:.0f}%)")

            if drop > 0 and pct >= 75.0:
                print("  => [X] 缓冲被打满过，Logger 跟不上 Flash，丢帧已经发生")
                print("     先看上面的 BB:JITTER 有没有被拖慢；必要时降采样率")
            elif drop > 0:
                print("  => [!] 有丢帧但水位不高：偶发长停顿（多半是扇区擦除超时），盯一次飞行")
            elif pct >= 75.0:
                print("  => [!] 还没丢帧但余量很小，随时可能开始丢")
            elif pct >= 25.0:
                print("  => [OK] 余量充足")
            else:
                print("  => [OK] 余量非常充足，Logger 完全跟得上")

    # ---- 时序抖动：判断黑匣子有没有拖慢控制环 / IMU 采样 ----
    jt = {}
    for ln in lines:
        if ln.startswith("BB:JITTER"):
            jt = parse_stat_line(ln.replace("BB:JITTER", "BB:STAT", 1))
            break

    if jt:
        imu_max = int(jt.get("imu_max", 0))
        fc_max = int(jt.get("fc_max", 0))
        imu_ov = int(jt.get("imu_ov1", 0)) + int(jt.get("imu_ov2", 0))
        fc_ov = int(jt.get("fc_ov1", 0)) + int(jt.get("fc_ov2", 0))

        print("\n--- 时序抖动（理想值：max 约 1000~1100us，丢拍次数为 0）---")
        print(f"  IMU 采样周期   最大 {imu_max:5d} us   丢拍 {imu_ov}")
        print(f"  FlyControl 周期 最大 {fc_max:5d} us   丢拍 {fc_ov}")

        if imu_ov == 0 and fc_ov == 0 and max(imu_max, fc_max) < 1500:
            print("  => [OK] 控制环与 IMU 采样都很稳，黑匣子没有拖慢它们")
        elif imu_ov > 0 or fc_ov > 0:
            who = []
            if imu_ov:
                who.append(f"IMU采样丢{imu_ov}次")
            if fc_ov:
                who.append(f"控制环丢{fc_ov}次")
            print(f"  => [X] 确实被拖过：{'，'.join(who)}")
            print("     下一步：先删掉 BlackBox_Queue 里的 __disable_irq()")
        else:
            print("  => [!] 最大周期偏大但没到丢拍，可能只是偶发抖动")
    return 0


def cmd_cal(args) -> int:
    """查询上电时的陀螺静止标定质量。起飞前必看。"""
    from bb_serial import send_command
    with _open(args) as ser:
        lines = send_command(ser, "BB:CAL", timeout=5.0)

    cal = {}
    acc = {}
    for ln in lines:
        for tag, dst in (("BB:CAL", cal), ("BB:ACC", acc)):
            if ln.startswith(tag):
                for field in ln.split()[1:]:
                    if "=" in field:
                        k, v = field.split("=", 1)
                        try:
                            dst[k] = float(v)
                        except ValueError:
                            # 去掉 "0.09deg" 这类带单位的后缀再试一次
                            num = v.rstrip("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ")
                            try:
                                dst[k] = float(num)
                            except ValueError:
                                dst[k] = v

    if not cal:
        print("[X] 没收到 BB:CAL —— 固件是旧版（重新烧录后才有这个命令）")
        return 1

    gx = cal.get("gx", float("nan"))
    gy = cal.get("gy", float("nan"))
    gz = cal.get("gz", float("nan"))
    std = cal.get("std", float("nan"))
    worst = max(abs(gx), abs(gy), abs(gz))

    print("\n上电静止标定结果（陀螺零偏，deg/s）：")
    print(f"  gx {gx:+8.3f}   gy {gy:+8.3f}   gz {gz:+8.3f}")
    print(f"  三轴最大 {worst:.3f}   标定期间标准差之和 {std:.3f}")

    if std > 1.5 or worst > 3.0:
        print("\n[X] 标定不可信 —— 标定时飞机被碰过/在动。")
        print("    后果：恒定方向的偏飞（摇杆回中也压不住）。")
        print("    处理：断电重来，上电后别碰飞机，等约 4 秒启动完成。")
        return 1
    if std > 0.5 or worst > 1.0:
        print("\n[!] 边缘 —— 偏差偏大，建议重新标定一次。")
        return 0

    if acc:
        ax = acc.get("ax", float("nan")); ay = acc.get("ay", float("nan"))
        az = acc.get("az", float("nan"))
        tilt = acc.get("tilt", float("nan"))
        print("\n加计读数（飞机放平静止时应为 0,0,1）：")
        print(f"  ax {ax:+.4f}   ay {ay:+.4f}   az {az:+.4f} g")
        print(f"  合成倾角 {tilt:.2f}°")
        if tilt > 2.0:
            print("\n[X] 加计零偏太大 —— 姿态估计的“水平”会整体歪掉这个角度，")
            print("    控制器就带着它飞 → 恒定方向偏飞（beta 再大也无效）。")
            print("    处理：确认飞机真的放平；重新上电让它重新标定。")
            return 1
        if tilt > 1.0:
            print("\n[!] 加计零偏偏大，建议检查放平后重新上电。")
        else:
            print("  [OK] 加计水平基准正常")

    print("\n[OK] 标定正常，可以起飞")
    return 0


def cmd_ping(args) -> int:
    from bb_serial import send_command
    with _open(args) as ser:
        lines = send_command(ser, "BB:PING", timeout=3.0)
    if any(l.startswith("BB:OK") for l in lines):
        print("\n[[OK]] 握手成功：飞控在线，双向通信正常")
        print(f"    下一步： python bb.py status --port {_resolve_port(args)}")
    return 0


def cmd_listen(args) -> int:
    from bb_serial import listen
    with _open(args) as ser:
        listen(ser, seconds=args.seconds, show_raw=args.raw)
    return 0


def cmd_scan(args) -> int:
    from bb_serial import send_command
    with _open(args) as ser:
        lines = send_command(ser, "BB:SCAN", timeout=120.0)
    hdrs = [l for l in lines if l.startswith("BB:HDR")]
    if hdrs:
        print(f"\n[[OK]] 共 {len(hdrs)} 个有效扇区索引头（即录过 {len(hdrs)} 个 4KB 扇区）")
    return 0


def cmd_erase(args) -> int:
    from bb_serial import send_command, wait_erase_done
    port = _resolve_port(args)
    if not args.yes:
        print("全片擦除会清掉所有飞行数据，后台执行约 200 秒。")
        ans = input("确认继续？输入 yes 回车： ").strip().lower()
        if ans != "yes":
            print("已取消。")
            return 1
    with _open(args) as ser:
        lines = send_command(ser, "BB:ERASE", timeout=10.0)
        started = any("ERASING" in l for l in lines)
        if not started:
            print("[[X]] 固件未受理擦除（可能处于解锁状态，或 Flash 未识别）")
            return 1
        if args.no_wait:
            print("\n[已下发] 擦除在后台执行（约 200s）。")
            print(f"    稍后用 `python bb.py status --port {port}` 查看：")
            print("      erasing=1 表示还在擦；eseq= 增大且 erasing=0 表示已完成。")
            return 0
        # 默认：轮询状态，直到完成 —— 不硬等固定 200s
        ok = wait_erase_done(ser, timeout_s=args.timeout)
    return 0 if ok else 1


def _rebuild_full_image(path: str, start_sector: int, sectors: int,
                        hdr_len: int = 0) -> bool:
    """把"紧凑格式"的 dump 文件还原成完整的 16MB 镜像（原地覆盖）。

    固件按扇区发送：[hdr_len 字节索引头][3840 字节数据区]。
    - hdr_len = 32 时（新固件）：索引头会被放到扇区最开头，解析器能拿到
      逻辑序号/已写帧数/上电次数，才能正确分段和挑出最新一次飞行。
    - hdr_len = 0 时（旧固件，漏发了索引头）：只还原数据区，
      索引头区域填 0xFF，parse_flash 仍然能靠帧自同步读出数据，
      但会丢失扇区索引信息。

    空洞填 0xFF，还原成完整 16MB 镜像，
    这样 bb_format.parse_flash() 一行都不用改（它本来就是按整片镜像写的）。

    返回 True 表示做了转换。
    """
    from bb_format import (SECTOR_COUNT, SECTOR_DATA_LEN, SECTOR_DATA_OFF,
                           SECTOR_SIZE, FLASH_BYTES)

    with open(path, "rb") as fp:
        compact = fp.read()

    per_sector = hdr_len + SECTOR_DATA_LEN
    expect = sectors * per_sector
    if len(compact) < expect:
        print(f"[!] 文件不完整：{len(compact)} / {expect} 字节，按已有部分还原")
    compact = compact[:expect]
    got_sectors = (len(compact) + per_sector - 1) // per_sector

    if start_sector == 0 and got_sectors >= SECTOR_COUNT and len(compact) >= expect:
        # 已经是完整镜像（旧式整片 dump），不用动
        if len(compact) >= FLASH_BYTES:
            return False

    img = bytearray(b"\xFF" * FLASH_BYTES)
    for i in range(got_sectors):
        src = i * per_sector
        base = (start_sector + i) * SECTOR_SIZE

        if hdr_len > 0:
            h = compact[src:src + hdr_len]
            img[base:base + len(h)] = h
            src += hdr_len

        d = compact[src:src + SECTOR_DATA_LEN]
        img[base + SECTOR_DATA_OFF:base + SECTOR_DATA_OFF + len(d)] = d

    with open(path, "wb") as fp:
        fp.write(img)
    tag = f"含索引头 {hdr_len}B/扇区" if hdr_len else "无索引头(旧固件)"
    print(f"[已还原为完整 16MB 镜像：{start_sector} 起 {got_sectors} 个扇区，{tag}]")
    return True


def cmd_dump(args) -> int:
    from bb_serial import dump_flash
    port = _resolve_port(args)
    out = args.output or "flash.bin"
    if args.full:
        print(f"开始倾倒**整片** Flash 到 {out}（16 MB）...")
    else:
        print(f"开始倾倒**有数据的扇区**到 {out}（固件只发已写入的部分）...")
    print("注意：飞机必须处于**未解锁**状态，否则固件会拒绝。")
    with _open(args) as ser:
        n, info = dump_flash(ser, out, full=args.full)

    written = info["bytes"] if args.full else min(n, info["bytes"])
    if written > 0 and not args.full:
        _rebuild_full_image(out, info["start_sector"], info["sectors"],
                            int(info.get("hdr", 0)))
        written = FLASH_BYTES

    print(f"\n已接收 {n} 字节"
          f"（{info['sectors']} 个扇区 / {info['frames']} 帧，起始扇区 {info['start_sector']}"
          f"，索引头 {info.get('hdr', 0)}B/扇区）")
    if n < info["bytes"]:
        print("[!] 数据不完整：可能是串口断开或被中途打断。解析仍可进行，但末尾会缺失。")
    print(f"下一步： python bb.py analyze {out} --plot")
    return 0


def cmd_parse(args) -> int:
    with open(args.image, "rb") as fp:
        raw = fp.read()
    log = bb_format.parse_flash(raw)
    print(log.describe_headers())
    run = log.latest_run()
    print(f"\n最近一次记录：{run.size} 帧（约 {run.size/1000.0:.1f} 秒）")
    if run.size:
        r = log.roll[run]
        print(f"  roll: 首 {r[:100].mean():+.2f}deg  末 {r[-100:].mean():+.2f}deg  "
              f"范围 [{r.min():+.2f}, {r.max():+.2f}]")
    return 0


def cmd_calib(args) -> int:
    """CFG:CAL —— 让飞机重新标定零偏（地面、静止约 2 秒），并把结果写进待生效参数。"""
    import bb_cfg

    _cfg_guard()
    with _open(args) as ser:
        lines, ok = bb_cfg.cfg_cal(ser)
        for ln in lines:
            print("  " + ln)
        if not ok:
            print("[X] 标定失败（解锁中会被拒绝；也可能是超时）")
            return 1

        vals = {}
        for ln in lines:
            for m in re.finditer(r"\b(gx|gy|gz|ax|ay|az)=([-+]?\d+(?:\.\d+)?)", ln):
                vals[m.group(1)] = float(m.group(2))
            m = re.search(r"std=([-+]?\d+(?:\.\d+)?)", ln)
            if m:
                vals["std"] = float(m.group(1))

    print("\n--- 标定结果 ---")
    print(f"  陀螺零偏  gx={vals.get('gx', 0):+.3f}  gy={vals.get('gy', 0):+.3f}  "
          f"gz={vals.get('gz', 0):+.3f} deg/s")
    print(f"  加计零偏  ax={vals.get('ax', 0):+.4f}  ay={vals.get('ay', 0):+.4f}  "
          f"az={vals.get('az', 0):+.4f} g")
    std = vals.get("std")
    if std is not None:
        print(f"  标定质量  std={std:.3f} deg/s " +
              ("[ok]" if std <= 0.5 else "[偏大，飞机当时被碰过？重标]"))
    print("\n已写入待生效值。落盘并生效： python bb.py cfg save && python bb.py cfg reboot")
    return 0


def cmd_analyze(args) -> int:
    import bb_analyze
    argv = [args.image]
    if args.plot:
        argv.append("--plot")
    if args.save_plot:
        argv += ["--save-plot", args.save_plot]
    if args.csv is not None:
        # --csv 不带文件名（args.csv == ""）时，按镜像名导出：flash.bin -> flash.csv
        name = args.csv
        if not name:
            import os
            name = os.path.splitext(os.path.basename(args.image))[0] + ".csv"
        argv += ["--csv", name]
    if getattr(args, "run", -1) >= 0:
        argv += ["--run", str(args.run)]
    if getattr(args, "trim_roll", 0.0):
        argv += ["--trim-roll", str(args.trim_roll)]
    if getattr(args, "trim_pitch", 0.0):
        argv += ["--trim-pitch", str(args.trim_pitch)]
    if getattr(args, "list_runs", False):
        argv.append("--list-runs")
    return bb_analyze.main(argv)


def cmd_stepgo(args) -> int:
    """发送 BB:STEP，让固件向目标角注入方波（测闭环阶跃响应用）。"""
    from bb_serial import send_command

    if args.off:
        cmd = "BB:STEP OFF"
    else:
        if args.axis is None or args.amp is None:
            print("[X] 需要 --axis 和 --amp（或用 --off 取消）")
            return 2
        cmd = f"BB:STEP {args.axis} {args.amp:g} {args.period} {args.cycles}"

    with _open(args) as ser:
        lines = send_command(ser, cmd, timeout=5.0)

    ok = False
    for ln in lines:
        print("  " + ln)
        if ln.startswith("BB:STEP"):
            ok = True

    if not ok:
        print()
        print("[X] 固件没回 BB:STEP —— 可能还是旧版（重新烧录后再试）")
        return 1

    if not args.off:
        half = max(32, args.period // 2)
        print()
        print(f"  半周期 {half} ms，共 {args.cycles} 个周期 = "
              f"{args.period * args.cycles / 1000.0:.1f} 秒")
        print()
        print("  ★ 不需要飞行中连串口：它现在处于“待触发”状态。")
        print("    满足下面全部条件并保持 0.5 秒后自动开始：")
        print("      · 已解锁    · 油门 >= 1300    · 姿态倾角 < 12°")
        print("      · roll/pitch 摇杆都在中位±50 以内")
        print("    也就是：你正常起飞、悬停稳定后它自己开始。")
        print()
        print("  取消方式（任一）：动一下摇杆 / 上锁 /")
        print("                        超过 120 秒未触发则自动作废")
        print(f"  幅度提示：实际注入 ±{args.amp}deg（峰峰 {2*args.amp:g}deg）")
        if half < 800:
            print(f"  [!] 半周期 {half} ms 短于 800 ms，上位机可能识别不出阶跃")
            print("      建议 --period 2048（半周期 1024ms）")
        print()
        print("  飞完降落上锁后：python bb.py dump -o fN.bin && python bb.py step fN.bin")
    return 0


def cmd_step(args) -> int:
    """阶跃 / 回正响应分析。"""
    import bb_step
    import bb_format

    with open(args.image, "rb") as fp:
        log = bb_format.parse_flash(fp.read())

    run = log.all_frames()
    if getattr(args, "run", -1) >= 0:
        runs = log.runs()
        if not runs:
            print("[!] 没有可分段的记录")
            return 1
        k = args.run if args.run >= 0 else 0
        if k >= len(runs):
            print(f"[!] 只有 {len(runs)} 段，--run {k} 超范围")
            return 1
        run = runs[k]

    axis = bb_step.build_axis(log, run)
    if axis is None:
        print("[X] 有效帧太少，无法分析")
        return 1

    rows_step, rows_ret = [], []
    for which in args.axis:
        st = bb_step.analyse_steps(axis, which)
        rt = bb_step.analyse_returns(axis, which)
        bb_step.report(axis, st, rt, which)
        rows_step += st
        rows_ret += rt

    if args.csv:
        n = bb_step.export_csv(args.csv, axis, rows_step, rows_ret)
        print(f"[step] 导出 CSV → {args.csv}（{n} 行）")
    return 0


def cmd_selftest(args) -> int:
    import selftest_format
    return selftest_format.main()


# ==================================================================
#  配置存储（内部 Flash 模拟 EEPROM）：bb.py cfg / preset
# ==================================================================
def _cfg_guard() -> None:
    """本地常量与固件是否一致。不一致只警告，不阻塞（读写走的是参数名）。"""
    try:
        import bb_cfg
        problems = bb_cfg.check_firmware_constants()
    except Exception as exc:                       # noqa: BLE001
        print(f"[!] 无法核对固件常量（{exc}），继续。")
        return
    if problems:
        print("[!] 上位机常量与固件不一致（按名字读写仍可用，但离线解析会错）：")
        for p in problems:
            print("      " + p)
        print("      -> 改完 tools/bb_cfg.py 后重跑： python ee_parse.py --selftest")


def _print_cfg_info(st: dict) -> None:
    src = st.get("src", "?")
    dirty = st.get("dirty", 0)
    tbl = st.get("tbl", 1)
    ovl = st.get("ovl", 0)

    print("\n--- 参数 ---")
    print(f"  参数项数       {st.get('n')}   (tbl={tbl}: 1=参数表正常)")
    print(f"  启动时来源     {src}   (ee=从 Flash 载入, def=用编译期默认值)")
    print(f"  待生效改动     {dirty}   (1=改过但还没重启，需要 SAVE + REBOOT)")

    print("\n--- 存储 ---")
    print(f"  最新记录序号   seq={st.get('seq')}")
    print(f"  位置           Bank {st.get('bank')}  下一个空槽 {st.get('next')}  "
          f"已用 {st.get('used')}")
    print(f"  CRC 失败/镜像越界  crc_fail={st.get('crc_fail')}  ovl={ovl}")

    print("\n--- 实测 ---")
    print(f"  上次写入       {st.get('wr_us')} us")
    print(f"  上次擦除       {st.get('er_us')} us   (约 953 ms = 一个 128KB 扇区)")
    print(f"  镜像末尾       {st.get('img')}   (必须小于 080C0000)")
    print(f"  本次开机       {st.get('up')} ms")

    print()
    if ovl:
        print("[[X]] 镜像越界：配置区已锁成只读，检查 Keil 的 IROM 是否为 0xC0000")
    elif tbl != 1:
        print("[[X]] 参数表顺序不对：已退回默认值，检查 param.c 的表格顺序")
    elif src != "ee":
        print("[[i]] 还没有参数记录：现在用的是编译期默认值（改完存盘一次就会变成 ee）")
    elif dirty:
        print("[[i]] 有未生效的改动：CFG:SAVE 之后重启才会作用到飞行")
    else:
        print("[[OK]] 参数与 Flash 一致，无待生效改动")


def _do_save(ser) -> bool:
    import bb_cfg
    ok, line = bb_cfg.cfg_save(ser)
    print("  " + bb_cfg.explain(line))
    if ok and "SAVED" in line:
        print("      -> 已落盘。要生效还需要重启：CFG:REBOOT 或断电")
    return ok


def _do_reboot(ser) -> bool:
    import time as _time
    import bb_cfg
    ok, lines = bb_cfg.cfg_simple(ser, "CFG:REBOOT", "reboot")
    for ln in lines:
        print("  " + ln)
    if ok:
        print("      -> 已下发软复位；等 2 秒后可用 `bb.py cfg info` 确认参数已生效")
        _time.sleep(2.0)
    return ok


def cmd_cfg(args) -> int:
    import bb_cfg

    action = args.action
    rest = list(args.rest or [])
    _cfg_guard()

    with _open(args) as ser:
        # ---------- 只读 ----------
        if action == "info":
            st, _ok = bb_cfg.cfg_info(ser)
            if not st:
                print("[X] 没收到 CFG:INFO 回包（固件是 P2 之前的版本？）")
                return 1
            _print_cfg_info(st)
            return 0

        if action == "list":
            plist, _ok = bb_cfg.cfg_list(ser)
            if not plist:
                print("[X] 没收到参数列表")
                return 1
            shown = [p for p in plist if p["dirty"]] if args.dirty_only else plist
            width = max(len(p["name"]) for p in plist)
            for p in shown:
                print(f"  0x{p['id']:04X}  {p['name']:<{width}}  {p['value']:>10.3f}"
                      + ("  *" if p["dirty"] else ""))
            n_dirty = sum(1 for p in plist if p["dirty"])
            print(f"\n共 {len(plist)} 项；待生效未重启 {n_dirty} 项（* 标记）")
            if args.json:
                preset = bb_cfg.preset_build({p["name"]: p["value"] for p in plist},
                                             name=os.path.splitext(
                                                 os.path.basename(args.json))[0])
                with open(args.json, "w", encoding="utf-8") as fp:
                    import json as _json
                    _json.dump(preset, fp, ensure_ascii=False, indent=2)
                    fp.write("\n")
                print(f"[已导出 {args.json}]")
            return 0

        if action == "get":
            if not rest:
                raise SystemExit("用法： bb.py cfg get <参数名|ID>")
            p, lines, _ok = bb_cfg.cfg_get(ser, rest[0])
            if not p:
                for ln in lines:
                    print("  " + bb_cfg.explain(ln))
                return 1
            line = f"  0x{p['id']:04X}  {p['name']} = {p['value']:.3f}"
            if p["applied"] is not None:
                line += f"   (生效值 {p['applied']:.3f})"
            print(line)
            if p["dirty"]:
                print("      -> 改过但还没重启，重启后才生效")
            return 0

        # ---------- 写 ----------
        if action == "set":
            if len(rest) < 2:
                raise SystemExit("用法： bb.py cfg set <参数名|ID> <数值> [--save] [--reboot]")
            key, sval = rest[0], rest[1]
            try:
                value = float(sval)
            except ValueError:
                raise SystemExit(f"数值 {sval!r} 不是数字（例：12 / -0.3 / 0.008）")
            ok, line, _lines = bb_cfg.cfg_set(ser, key, value)
            print("  " + bb_cfg.explain(line))
            if not ok:
                return 1
            if "clamped" in line:
                print("      [!] 超出范围，被固件限制到允许区间了（看括号里的原值）")
            if args.save and not _do_save(ser):
                return 1
            if args.reboot and not _do_reboot(ser):
                return 1
            return 0

        if action == "save":
            return 0 if _do_save(ser) else 1

        if action == "reboot":
            return 0 if _do_reboot(ser) else 1

        if action in ("load", "def"):
            cmd = "CFG:LOAD" if action == "load" else "CFG:DEF"
            ok, lines = bb_cfg.cfg_simple(ser, cmd, action)
            for ln in lines:
                print("  " + bb_cfg.explain(ln))
            return 0 if ok else 1

        if action == "hist":
            n = 5
            if rest:
                try:
                    n = int(rest[0])
                except ValueError:
                    raise SystemExit("用法： bb.py cfg hist [条数]")
            lines, tail, _ok = bb_cfg.cfg_hist(ser, n)
            if not lines:
                print("  （没有历史记录）")
            for ln in lines:
                print("  " + ln)
            if tail:
                print(f"\n  深度 depth={tail.get('depth')}，本次列出 {tail.get('n')} 条")
                print("  回滚： bb.py cfg rollback <序号> --reboot")
            return 0

        if action in ("loadhist", "rollback"):
            if not rest:
                raise SystemExit(f"用法： bb.py cfg {action} <序号>")
            try:
                idx = int(rest[0])
            except ValueError:
                raise SystemExit("序号必须是整数（从 0 开始，0 是最新一条）")
            cmd = (f"CFG:ROLLBACK:{idx}" if action == "rollback" else f"CFG:LOAD:{idx}")
            ok, lines = bb_cfg.cfg_simple(ser, cmd, action)
            for ln in lines:
                print("  " + bb_cfg.explain(ln))
            if not ok:
                return 1
            if action == "loadhist":
                print("      -> 只改内存，没写 Flash；要保留请再 `bb.py cfg save`")
            else:
                print("      -> 已作为最新记录存盘，重启后生效")
                if args.reboot and not _do_reboot(ser):
                    return 1
            return 0

        if action == "raw":
            data, lines = bb_cfg.cfg_raw(ser)
            if data is None:
                for ln in lines:
                    print("  " + ln)
                print("[X] 没拿到原始记录（还没有记录？）")
                return 1
            print(f"  收到 {len(data)} 字节：" + " ".join(f"{b:02X}" for b in data[:16]) + " ...")
            if args.out:
                with open(args.out, "wb") as fp:
                    fp.write(data)
                print(f"  [已写入 {args.out}]")
            else:
                for i in range(0, len(data), 16):
                    row = data[i:i + 16]
                    print(f"  {i:04X}  " + " ".join(f"{b:02X}" for b in row))
            return 0

        if action == "erase":
            if not args.yes:
                print("这会擦掉两个 bank 里**全部**配置记录（约 2 秒，整机停顿）。")
                print("当前内存里的参数不会变，但断电后就会回到编译期默认值。")
                ans = input("确认擦除？输入 yes 继续：").strip().lower()
                if ans != "yes":
                    print("已取消。")
                    return 1
            ok, lines = bb_cfg.cfg_simple(ser, "CFG:ERASE", "erase")
            for ln in lines:
                print("  " + bb_cfg.explain(ln))
            return 0 if ok else 1

    raise SystemExit(f"未知的 cfg 动作：{action}")


def cmd_preset(args) -> int:
    import json as _json
    import bb_cfg

    _cfg_guard()

    # ---------- 导出 ----------
    if args.action == "export":
        with _open(args) as ser:
            plist, _ok = bb_cfg.cfg_list(ser)
        if not plist:
            print("[X] 没收到参数列表")
            return 1
        name = args.name or os.path.splitext(os.path.basename(args.file))[0]
        preset = bb_cfg.preset_build({p["name"]: p["value"] for p in plist},
                                     name=name, note=args.note or "")
        with open(args.file, "w", encoding="utf-8") as fp:
            _json.dump(preset, fp, ensure_ascii=False, indent=2)
            fp.write("\n")
        print(f"[已导出 {args.file}]  {len(preset['params'])} 项参数")
        print("  这个文件可以直接进 git，下次 `bb.py preset apply` 一键刷回去。")
        return 0

    # ---------- 导入 ----------
    data = bb_cfg.preset_load(args.file)
    params = data["params"]
    print(f"[预设 {data.get('name', '?')}] {len(params)} 项"
          + (f"   备注：{data['note']}" if data.get("note") else ""))

    if args.dry_run:
        print("\n(dry-run，只打印将要发送的命令)")
        for k, v in params.items():
            print(f"  CFG:SET {k} {bb_cfg.fmt_value(float(v))}")
        if args.save:
            print("  CFG:SAVE")
        if args.reboot:
            print("  CFG:REBOOT")
        return 0

    with _open(args) as ser:
        cur, _ok = bb_cfg.cfg_list(ser)
        curmap = {p["name"]: p["value"] for p in cur}
        changed = [(k, float(v)) for k, v in params.items()
                   if k not in curmap or abs(curmap[k] - float(v)) > 1e-6]
        print(f"  其中 {len(changed)} 项与飞机上当前值不同"
              + ("" if len(changed) <= 8 else "（只列前 8 项）"))
        for k, v in changed[:8]:
            print(f"    {k}: {curmap.get(k, '（未知）')} -> {v}")

        fails = []
        for k, v in params.items():
            ok, line, _lines = bb_cfg.cfg_set(ser, k, float(v))
            if not ok:
                fails.append((k, v, line))
                if len(fails) >= 5:
                    break
        if fails:
            print(f"\n[X] {len(fails)} 项设置失败：")
            for k, v, line in fails:
                print(f"    {k}={v}: {bb_cfg.explain(line)}")
            print("\n  此时飞机内存里是**部分应用**的状态：")
            print("    - 什么都不做，重启一次 -> 未保存的改动全部丢弃（回到盘里的值）")
            print("    - 或立刻执行 `bb.py cfg load` 丢弃未保存改动")
            return 1

        print(f"\n[OK] {len(params)} 项已写入待生效表（还没生效）")
        if args.save and not _do_save(ser):
            return 1
        if args.reboot and not _do_reboot(ser):
            return 1
        if not args.save:
            print("      -> 别忘了存盘： `bb.py cfg save`，然后重启才生效")
        return 0


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        prog="bb.py",
        description="W25Q128 黑匣子上位机（提取 / 解析 / 分析）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = ap.add_subparsers(dest="cmd")      # 不带子命令时打印引导，而不是报错

    def _add_serial_opts(p):
        """网络类命令共用的串口参数。"""
        p.add_argument("--port", help="串口号（只有一路串口时可省略）")
        p.add_argument("--baud", type=int,
                       help="波特率（默认 460800；握手不通报错时可试 115200/921600）")

    p = sub.add_parser("ports", help="列出可用串口")
    p.set_defaults(func=cmd_ports)

    p = sub.add_parser("ping", help="连通性测试（最简单的握手）")
    _add_serial_opts(p)
    p.set_defaults(func=cmd_ping)

    p = sub.add_parser("cal", help="查上电陀螺标定质量（起飞前必看）")
    _add_serial_opts(p)
    p.set_defaults(func=cmd_cal)

    p = sub.add_parser("calib", help="重新标定零偏并写入 cal.* 参数（地面、静止 2 秒）")
    _add_serial_opts(p)
    p.set_defaults(func=cmd_calib)

    p = sub.add_parser("status", aliases=["st"], help="查询黑匣子状态")
    _add_serial_opts(p)
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("scan", help="扫描扇区索引头")
    _add_serial_opts(p)
    p.set_defaults(func=cmd_scan)

    p = sub.add_parser("listen", help="只监听不发送（抓开机横幅 BB:BOOT）")
    _add_serial_opts(p)
    p.add_argument("-s", "--seconds", type=float, default=8.0, help="监听时长（默认 8 秒）")
    p.add_argument("--raw", action="store_true", help="以十六进制显示原始字节")
    p.set_defaults(func=cmd_listen)

    p = sub.add_parser("erase", help="全片擦除（约 200s，需未解锁）")
    _add_serial_opts(p)
    p.add_argument("-y", "--yes", action="store_true", help="跳过确认")
    p.add_argument("--no-wait", action="store_true",
                   help="下发后立即返回，不等待完成（之后用 status 查 erasing/eseq）")
    p.add_argument("--timeout", type=float, default=400.0,
                   help="等待擦除完成的超时秒数（默认 400）")
    p.set_defaults(func=cmd_erase)

    p = sub.add_parser("dump", aliases=["d"], help="倾倒 Flash 到本地文件（默认只发有数据的部分）")
    _add_serial_opts(p)
    p.add_argument("-o", "--output", help="输出文件（默认 flash.bin）")
    p.add_argument("--full", action="store_true",
                   help="整片 16MB（默认只发有数据的扇区，快得多）")
    p.set_defaults(func=cmd_dump)

    p = sub.add_parser("parse", help="离线解析（只看概览）")
    p.add_argument("image")
    p.set_defaults(func=cmd_parse)

    p = sub.add_parser("analyze", aliases=["a"], help="离线分析 + 出图")
    p.add_argument("image")
    p.add_argument("--plot", action="store_true")
    p.add_argument("--save-plot")
    p.add_argument("--csv", nargs="?", const="", metavar="FILE",
                   help="导出 CSV；不给文件名就导出成 镜像名.csv")
    p.add_argument("--run", type=int, default=-1,
                   help="只分析第几段记录（0=最新）。默认分析整片全部数据")
    p.add_argument("--trim-roll", type=float, default=0.0,
                   help="实测姿态与设定值都减去它(deg)，把 ctrl.roll_offset_deg 当零点")
    p.add_argument("--trim-pitch", type=float, default=0.0,
                   help="同上，pitch 方向")
    p.add_argument("--list-runs", action="store_true",
                   help="只列出所有分段（不分析），配合 --run 使用")
    p.set_defaults(func=cmd_analyze)

    p = sub.add_parser("step", aliases=["s"],
                       help="阶跃/回正响应分析（调 PID 用）")
    p.add_argument("image")
    p.add_argument("--axis", nargs="+", default=["roll", "pitch"],
                   choices=["roll", "pitch"], help="分析哪个轴，默认两个都做")
    p.add_argument("--csv", help="把阶跃窗口与回正统计导出为 CSV")
    p.add_argument("--run", type=int, default=-1,
                   help="只分析第几段记录（0=最新）")
    p.set_defaults(func=cmd_step)

    p = sub.add_parser("stepgo", help="发送 BB:STEP 注入方波（测阶跃响应）")
    _add_serial_opts(p)
    p.add_argument("--off", action="store_true", help="取消注入")
    p.add_argument("--axis", choices=["roll", "pitch"])
    p.add_argument("--amp", type=float, help="幅度 deg（2~25）")
    p.add_argument("--period", type=int, default=2048,
                   help="方波周期 ms（默认 2048；半周期需 >=800 才可被识别）")
    p.add_argument("--cycles", type=int, default=4, help="周期数 1~10")
    p.set_defaults(func=cmd_stepgo)

    p = sub.add_parser("cfg", help="配置存储：查看/修改参数、存盘、回滚（内部 Flash 模拟 EEPROM）")
    _add_serial_opts(p)
    p.add_argument("action",
                   choices=["info", "list", "get", "set", "save", "load", "def",
                            "hist", "loadhist", "rollback", "reboot", "raw", "erase"])
    p.add_argument("rest", nargs="*",
                   help="get/set 的参数名或 ID；set 的数值；hist/rollback 的序号")
    p.add_argument("--save", action="store_true", help="set 之后立刻存盘")
    p.add_argument("--reboot", action="store_true", help="存盘/回滚之后软复位让参数生效")
    p.add_argument("--dirty-only", action="store_true", help="list 只显示待生效的改动")
    p.add_argument("--json", help="list 的结果导出成预设 JSON")
    p.add_argument("--out", help="raw 的二进制输出文件")
    p.add_argument("-y", "--yes", action="store_true", help="erase 跳过确认")
    p.set_defaults(func=cmd_cfg)

    p = sub.add_parser("preset", help="参数预设：apply 一键刷入 / export 导出")
    _add_serial_opts(p)
    p.add_argument("action", choices=["apply", "export"])
    p.add_argument("file", help="预设 JSON 文件")
    p.add_argument("--save", action="store_true", help="apply 之后存盘")
    p.add_argument("--reboot", action="store_true", help="apply 之后软复位")
    p.add_argument("--dry-run", action="store_true", help="apply 只打印要发的命令，不连飞机")
    p.add_argument("--name", help="export 时写进文件的名字")
    p.add_argument("--note", help="export 时写进文件的备注")
    p.set_defaults(func=cmd_preset)

    p = sub.add_parser("selftest", help="格式环回自检（不接硬件）")
    p.set_defaults(func=cmd_selftest)

    return ap


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    if getattr(args, "cmd", None) is None:
        _print_banner()
        return 0
    try:
        return args.func(args)
    except SystemExit:
        raise
    except KeyboardInterrupt:
        print("\n[已中断]")
        return 130
    except Exception as exc:                       # noqa: BLE001
        # 兜底：任何意外错误都给出可读信息，而不是抛 traceback 造成"闪退"
        import traceback
        print(f"\n[!] 执行出错：{type(exc).__name__}: {exc}")
        print("    如果这是 dump/erase：可能串口被占用或飞控中途断开。")
        print("    详细信息：")
        for ln in traceback.format_exc().splitlines()[-8:]:
            print("      " + ln)
        return 1


if __name__ == "__main__":
    sys.exit(main())
