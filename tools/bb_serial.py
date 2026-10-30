"""黑匣子串口通信：指令收发与整片倾倒。

依赖 pyserial（未安装时会给出明确提示，不影响离线解析功能）：
    pip install pyserial

串口参数：USART3，默认 460800 8N1（与固件一致）。
    ※ 固件侧是 460800 @APB1=42MHz（16 倍过采样，实际 461538 Bd，偏差 +0.16%）。
      如果 PC 端串口芯片不支持 460800，可在固件里用 BB:BAUD:115200 临时降速。
固件指令：
    BB:PING            连通性测试
    BB:STAT            查询状态(是否识别到 Flash、当前扇区、已记录帧数、丢帧数...)
    BB:SCAN            扫描所有扇区索引头
    BB:ERASE           全片擦除（约 200s，必须处于未解锁状态）
    BB:DUMP            倾倒整片 16MB 原始数据（必须处于未解锁状态）

BB:DUMP 的输出序列：
    "BB:DUMP start=0 bytes=16777216\\r\\n" + <16MB 原始字节> + "\\r\\nBB:DUMPEND\\r\\n"
其中每个 4KB 扇区的前 256 字节是索引头区域，固件**原样发送**（不剔除），
因此上位机拿到的就是完整的 16MB Flash 镜像。
"""

from __future__ import annotations

import sys
import time
from typing import Optional

try:
    import serial  # type: ignore
    from serial.tools import list_ports  # type: ignore
    HAVE_SERIAL = True
except Exception:  # noqa: BLE001
    serial = None
    list_ports = None
    HAVE_SERIAL = False

from bb_format import FLASH_BYTES

DEFAULT_BAUD = 460800
DUMP_TIMEOUT_S = 600.0


def require_serial():
    if not HAVE_SERIAL:
        raise SystemExit(
            "未安装 pyserial，无法进行串口操作。\n"
            "请先执行：  pip install pyserial\n"
            "（离线解析/分析功能不需要 pyserial）"
        )


def available_ports() -> list[str]:
    if not HAVE_SERIAL:
        return []
    return [p.device for p in list_ports.comports()]


def pick_default_port(ports: list[str]) -> str:
    """从检测到的串口里挑一个默认值。

    规则：优先选**不是 COM1** 的。原因：COM1 通常是主板自带的 9 针串口，
    或者蓝牙/虚拟串口，飞控插的是 USB 转串口（CH340/CP210x/FTDI），
    一般落在 COM3 以上。只有 COM1 时才退回 COM1。

    返回值可能是空串（没有串口）。
    """
    for p in ports:
        if p.strip().upper() != "COM1":
            return p
    return ports[0] if ports else ""


def open_port(port: str, baud: int = DEFAULT_BAUD, timeout: float = 1.0, quiet: bool = False):
    """打开串口。失败时抛出带排查建议的异常，而不是静默返回。"""
    require_serial()
    try:
        ser = serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N",
                            stopbits=1, timeout=timeout)
    except Exception as exc:                      # noqa: BLE001
        raise SystemExit(
            f"打不开串口 {port} @{baud}：{exc}\n"
            f"  可能原因：\n"
            f"    - 串口号不对（用 `python bb.py ports` 查）\n"
            f"    - 该串口被别的程序占用（串口助手/Keil 的调试串口要关掉）\n"
            f"    - USB 转串口没插好或驱动没装\n"
            f"    - 波特率不被该芯片支持（可加 --baud 试别的值）"
        ) from exc

    # 显式把 DTR/RTS 置为有效稳态（True/True）。
    # 部分 USB 转串口芯片在打开瞬间会拉低这两根线，形成电平跳变；
    # 这里明确设成稳定值，避免"打开端口"这个动作本身干扰链路。
    for name in ("dtr", "rts"):
        try:
            setattr(ser, name, True)
        except Exception:                          # noqa: BLE001
            pass

    time.sleep(0.2)          # 给驱动和对方一点时间稳定，别紧接着就发

    if not quiet:
        chip = ""
        if HAVE_SERIAL and list_ports is not None:
            for info in list_ports.comports():
                if info.device == port:
                    chip = f" [{info.description}]"
                    break
        print(f"[已打开 {port} @{baud} 8N1{chip}]")
    return ser


def read_line(ser, timeout: float = 3.0) -> str:
    """读一行（以 \\n 结束）。超时返回空串。"""
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            break
        if b != b"\r":
            buf += b
    return buf.decode("ascii", errors="replace")


def send_command(ser, cmd: str, timeout: float = 3.0, echo: bool = True,
                 quiet: bool = False) -> list[str]:
    """发送一条指令并收集回包行，直到 BB:OK / BB:ERR / 超时。

    收不到回包时会打印实际收到的原始字节 —— 没有这个信息就无法判断
    是"链路上完全没有数据"还是"有数据但没有换行导致 read_line 一直等"。
    """
    wire = cmd.encode("ascii") + b"\r\n"
    ser.reset_input_buffer()
    ser.write(wire)
    ser.flush()
    if not quiet:
        print(f"[已发送 {cmd!r}，实际发出字节 {wire!r}，等待回包...]")

    lines: list[str] = []
    raw_seen = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(ser, timeout=max(0.2, deadline - time.time()))
        if not line:
            continue
        if echo:
            print(line)
        lines.append(line)
        if line.startswith("BB:OK") or line.startswith("BB:ERR"):
            break
        if line.startswith("BB:ERASING"):
            break

    if not lines:
        print(f"[{timeout:.0f}s 内没有收到任何完整回包]")
        # 再抓一次原始字节：即使没有换行也能看到到底有没有数据
        leftover = ser.read(256)
        raw_seen += leftover
        if raw_seen:
            print(f"[但串口上收到了 {len(raw_seen)} 字节原始数据]")
            print(f"    HEX : {raw_seen[:64].hex(' ')}")
            print(f"    ASCII: {raw_seen[:64].decode('ascii', errors='replace')!r}")
            print("  -> 有数据但没被识别为完整回包，多半是波特率不对（数据是乱码）")
            print("     或回包里没有换行符（可加 --raw 观察）")
        else:
            print("[串口上**一个字节都没有**：飞控没有回话]")
            print("  排查建议：")
            print("    1. 先用串口助手确认：发 'BB:PING' + 回车，能否收到 'BB:OK'")
            print("       -> 若串口助手能收到而本脚本收不到，问题在脚本侧（把结果告诉我）")
            print("    2. 飞控 PB11(USART3_RX) 是否接到了转串口的 TX？")
            print("       ※ 见过一种接法：只接了 TX 没接 RX，表现为'能收到飞控输出、")
            print("         但发什么它都不理' —— 这时必须补上 RX 那根线")
            print("    3. GND 是否共地？")
            print("    4. 波特率是否一致？固件默认 460800；可试 --baud 115200 / 921600")
    return lines


def parse_stat_line(line: str) -> dict:
    """把 'BB:STAT ok=1 boot=3 sec=12 ...' 解析成字典。"""
    out: dict = {}
    if not line.startswith("BB:STAT"):
        return out
    for field in line.split()[1:]:
        if "=" in field:
            k, v = field.split("=", 1)
            try:
                out[k] = int(v, 0)
            except ValueError:
                out[k] = v
    return out


def wait_erase_done(ser, timeout_s: float = 400.0, poll_s: float = 3.0) -> bool:
    """轮询 BB:STAT，直到擦除完成（eseq 增大且 erasing=0）。

    固件的 BB:ERASE 是异步的（后台约 200s），这里不硬等，而是定期查状态。
    返回 True 表示完成；False 表示超时。
    """
    # 先取一次基准：记录当前 eseq
    lines = send_command(ser, "BB:STAT", timeout=5.0, echo=False)
    base = {}
    for ln in lines:
        base = parse_stat_line(ln)
        if base:
            break
    base_seq = int(base.get("eseq", 0))
    print(f"[擦除进行中... 基准 eseq={base_seq}，每 {poll_s:.0f}s 查一次状态]")

    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < timeout_s:
        time.sleep(poll_s)
        lines = send_command(ser, "BB:STAT", timeout=5.0, echo=False)
        st = {}
        for ln in lines:
            st = parse_stat_line(ln)
            if st:
                break
        if not st:
            print("  (未收到状态，重试...)")
            continue

        elapsed = time.time() - t0
        erasing = int(st.get("erasing", 0))
        seq = int(st.get("eseq", 0))

        if seq > base_seq and erasing == 0:
            print(f"\n[[OK]] 擦除完成，用时约 {elapsed:.0f}s（eseq={seq}）")
            return True

        if elapsed - last_print >= 15.0:
            print(f"  {elapsed:5.0f}s  erasing={erasing} eseq={seq}")
            last_print = elapsed

    print(f"\n[[X]] 等待 {timeout_s:.0f}s 仍未检测到擦除完成。")
    print("    可手动确认： python bb.py status --port ...  看 erasing= 与 eseq=")
    return False


def listen(ser, seconds: float = 8.0, show_raw: bool = False) -> int:
    """只监听不发送，返回收到的字节数。

    用途：抓固件的开机横幅（BB:BOOT ...）。看到横幅即说明固件在跑且 USART3 通了。
    """
    total = 0
    t0 = time.time()
    print(f"[监听 {seconds:.0f} 秒... 现在给飞控重新上电，观察是否出现 BB:BOOT]")
    while time.time() - t0 < seconds:
        chunk = ser.read(256)
        if not chunk:
            continue
        total += len(chunk)
        if show_raw:
            sys.stdout.write(chunk.hex(" ") + "\n")
        else:
            sys.stdout.write(chunk.decode("ascii", errors="replace"))
        sys.stdout.flush()

    print()
    if total == 0:
        print("[0 字节] 串口上完全没有数据。说明飞控没有往这条线发东西：")
        print("    - 固件是否已烧录本次带黑匣子的版本？旧固件不会输出任何东西")
        print("    - 飞控 PB10(USART3_TX) 是否接到了转串口的 RX？")
        print("    - GND 是否共地？")
        print("    - 飞控是否已上电/复位？（横幅只在启动后 2.5 秒左右发一次）")
    else:
        print(f"[共收到 {total} 字节]")
    return total


def dump_flash(ser, out_path: str, progress: bool = True, full: bool = False,
               on_progress=None) -> tuple[int, dict]:
    """执行 BB:DUMP（或 BB:DUMPALL），把固件发来的帧数据写到 out_path。

    固件只发**有数据**的扇区（`BB:DUMP`），每个 4KB 扇区跳过前 256 字节的索引头，
    所以每扇区实际 3840 字节。写出的是紧凑格式，解析时由 bb_format.parse_flash
    按 `sectors` 参数还原物理排布。

    progress=False 时完全不打印；on_progress(done, total) 每秒回调一次
    （GUI 用它更新进度条，命令行不传）。

    返回 (实际写入字节数, 头部信息 dict)。
    """
    cmd = "BB:DUMPALL" if full else "BB:DUMP"
    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()

    # 注意：固件在回表头之前可能要先扫描扇区头，耗时可达数秒，所以超时给宽一点。
    # 另外容忍前面夹带其它行（历史版本会先发一行 BB:NOTE），跳过它去找真正的表头。
    header = ""
    for _ in range(6):
        line = read_line(ser, timeout=10.0)
        if not line:
            break
        if progress and not line.startswith("BB:DUMP"):
            print(f"[忽略] {line}")
        if line.startswith("BB:DUMP"):
            header = line
            break

    if progress and header:
        print(header)
    if not header:
        raise SystemExit(
            "设备未进入倾倒模式（没等到 'BB:DUMP' 表头）。\n"
            "  可能是：飞机处于解锁状态 / Flash 未识别(ok=0) / 固件版本较旧。\n"
            "  先用 `python bb.py status` 看一眼 ok= 和 armed=。")

    # 解析 "BB:DUMP start=<s> sectors=<n> frames=<f> bytes=<b> hdr=<h> empty=<0|1>"
    info = {"start_sector": 0, "sectors": 0, "frames": 0, "bytes": 0,
            "hdr": 0, "empty": 0}
    for field in header.split():
        if "=" not in field:
            continue
        k, v = field.split("=", 1)
        if k in ("start", "sectors", "frames", "bytes", "hdr", "empty"):
            key = "start_sector" if k == "start" else k
            try:
                info[key] = int(v)
            except ValueError:
                pass

    total = info["bytes"]
    if total <= 0:
        raise SystemExit(f"固件报告的长度无效：{header!r}")
    if info.get("empty"):
        print("[i] 固件报告这几次上电没录到数据（整片空白）。"
              "仍会下载 1 个扇区，解出来是空的。")

    written = 0
    t0 = time.time()
    last_report = t0
    trailer = bytearray()

    with open(out_path, "wb") as fp:
        while written < total:
            # 要考虑到夹杂的进度行：只在没有进度输出时才纯读二进制
            chunk = ser.read(min(65536, total - written))
            if not chunk:
                trailer += ser.read(64) or b""
                break
            fp.write(chunk)
            written += len(chunk)
            now = time.time()
            if now - last_report >= 1.0:
                if on_progress is not None:
                    on_progress(written, total)
                if progress:
                    pct = 100.0 * written / total
                    rate = written / max(1e-6, now - t0)
                    print(f"\r  已接收 {written/1024:.0f}/{total/1024:.0f} KB "
                          f"({pct:5.1f}%)  {rate/1024:.0f} KB/s", end="", flush=True)
                last_report = now

    if progress:
        print()
    for _ in range(4):
        line = read_line(ser, timeout=2.0)
        if not line:
            break
        if progress:
            print(line)
        if "DUMPEND" in line:
            break

    return written, info
