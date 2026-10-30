"""配置存储（内部 Flash 模拟 EEPROM）上位机协议层。

固件侧权威定义：
    Core/Inc/flash_ee.h      存储几何（bank/记录/魔数）
    Core/Inc/param.h         参数 ID 与数量
    Core/Src/param.c         参数表（名字/默认值/范围）
    Core/Src/flash_ee.c      记录格式（16 B 头 + 载荷 + CRC32）

⚠ 本文件里的常量与固件是**两份拷贝**（跟 bb_format.py 一样的处理方式）。
  改任意一侧都必须同步另一侧。用 `python ee_parse.py --selftest` 可以逐项核对，
  它会把固件头文件读出来跟这里的值比，对不上直接报错。

串口协议（USART3，460800 8N1）—— 注意回包前缀是 CFG:，不是 BB:，
所以不能用 bb_serial.send_command()（它等的是 BB:OK/BB:ERR），这里自己收行。
"""

from __future__ import annotations

import json
import os
import re
import time
from typing import Optional

# ============================ 存储几何 ============================
EE_BANK0_BASE = 0x080C0000
EE_BANK1_BASE = 0x080E0000
EE_BANK_SIZE = 0x00020000          # 128 KB
EE_RECORD_SIZE = 1024              # 每条记录 1 KB
EE_HDR_SIZE = 16
EE_SLOT_COUNT = EE_BANK_SIZE // EE_RECORD_SIZE      # 128
EE_PAYLOAD_MAX = EE_RECORD_SIZE - EE_HDR_SIZE       # 1008
EE_MAGIC = 0x45454346              # "FCEE"
EE_VER = 1

# 参数载荷（TLV）
PARAM_MAGIC = b"AP"
PARAM_ITEM_SIZE = 6                # uint16 id + float32

# P1 时期的自检载荷，解析时单独标出来（不是参数）
P1_TEST_MAGIC = 0x50315430

# ============================ 命令回包格式 ============================
# 每条命令的"结束标志"：收到以这些开头的行就算收完
TERMS = {
    "info":     ("CFG:INFO",),
    "list":     ("CFG:LISTEND",),
    "get":      ("CFG:P", "CFG:ERR"),
    "set":      ("CFG:OK", "CFG:ERR"),
    "save":     ("CFG:SAVED", "CFG:OK", "CFG:ERR"),
    "load":     ("CFG:OK", "CFG:ERR"),
    "def":      ("CFG:DEF", "CFG:ERR"),
    "hist":     ("CFG:HEND",),
    "loadhist": ("CFG:LOADED", "CFG:ERR"),
    "rollback": ("CFG:ROLLED", "CFG:ERR"),
    "reboot":   ("CFG:REBOOTING", "CFG:ERR"),
    "erase":    ("CFG:ERASED", "CFG:ERR"),
    "raw":      ("CFG:RAW", "CFG:ERR"),
    "cal":      ("CFG:CALEND", "CFG:ERR"),
}

# 超时（秒）。保存可能触发一次 bank 切换（擦除 ~1s），擦除命令要擦两块（~2s）
TIMEOUTS = {
    "info": 5.0, "list": 10.0, "get": 5.0, "set": 5.0,
    "save": 15.0, "load": 5.0, "def": 5.0, "hist": 15.0,
    "loadhist": 5.0, "rollback": 15.0, "reboot": 3.0,
    "erase": 20.0, "raw": 5.0,
    # 标定：机上要静止采样 2 秒，多给一点
    "cal": 20.0,
}


# ==================================================================
#  一、固件常量核对（防止两边不同步）
# ==================================================================
def repo_root() -> str:
    """tools/ 的上一级 = 工程根目录。"""
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _read(path: str) -> str:
    with open(path, "r", encoding="utf-8", errors="replace") as fp:
        return fp.read()


def _parse_defines(text: str) -> dict:
    out = {}
    for m in re.finditer(r"^#define\s+([A-Z0-9_]+)\s+(0x[0-9A-Fa-f]+|\d+)[UL]*\b",
                         text, re.M):
        out[m.group(1)] = int(m.group(2), 0)
    return out


def check_firmware_constants(root: Optional[str] = None) -> list[str]:
    """把固件头文件里的常量读出来，和本文件的值逐项比较。

    返回不一致项的文字列表；空列表 = 完全一致。
    """
    root = root or repo_root()
    problems: list[str] = []
    hdr = os.path.join(root, "Core", "Inc", "flash_ee.h")
    try:
        d = _parse_defines(_read(hdr))
    except OSError:
        return [f"读不到固件头文件：{hdr}"]

    want = {
        "EE_BANK0_BASE": EE_BANK0_BASE,
        "EE_BANK1_BASE": EE_BANK1_BASE,
        "EE_BANK_SIZE": EE_BANK_SIZE,
        "EE_RECORD_SIZE": EE_RECORD_SIZE,
        "EE_HDR_SIZE": EE_HDR_SIZE,
        "EE_MAGIC": EE_MAGIC,
        "EE_VER": EE_VER,
    }
    for k, v in want.items():
        got = d.get(k)
        if got is None:
            problems.append(f"{k}: 头文件里没找到（本文件={v:#x}）")
        elif got != v:
            problems.append(f"{k}: 固件={got:#x} 本文件={v:#x}")

    # 派生值（头文件里是表达式，不是字面量）
    if d.get("EE_BANK_SIZE") and d.get("EE_RECORD_SIZE"):
        if d["EE_BANK_SIZE"] // d["EE_RECORD_SIZE"] != EE_SLOT_COUNT:
            problems.append(
                f"EE_SLOT_COUNT: 固件推出 {d['EE_BANK_SIZE'] // d['EE_RECORD_SIZE']} "
                f"本文件={EE_SLOT_COUNT}")

    # 参数数量
    try:
        ph = _parse_defines(_read(os.path.join(root, "Core", "Inc", "param.h")))
        if "PARAM_COUNT" in ph and ph["PARAM_COUNT"] != PARAM_COUNT:
            problems.append(f"PARAM_COUNT: 固件={ph['PARAM_COUNT']} 本文件={PARAM_COUNT}")
    except OSError:
        problems.append("读不到 Core/Inc/param.h")

    return problems


# ==================================================================
#  二、参数表（从 param.h / param.c 解析，避免手工抄名字）
# ==================================================================
PARAM_COUNT = 82          # 与固件 PARAM_COUNT 一致，selftest 会核对


def _strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def parse_param_ids(param_h_path: str) -> dict:
    """解析 param.h 的 enum，返回 {符号名: ID}。

    处理 `P_X = 0x0100, P_Y, P_Z,` 这种"只给第一个赋初值、后面递增"的写法。
    """
    body = _strip_comments(_read(param_h_path))
    m = re.search(r"typedef\s+enum\s*\{(.*?)\}\s*ParamId\s*;", body, re.S)
    if not m:
        raise ValueError(f"{param_h_path}: 找不到 typedef enum ... ParamId")

    ids: dict[str, int] = {}
    cur: Optional[int] = None
    for item in m.group(1).split(","):
        item = item.strip()
        if not item:
            continue
        if "=" in item:
            name, val = item.split("=", 1)
            name = name.strip()
            val = val.strip()
            try:
                cur = int(val, 0)
            except ValueError:
                raise ValueError(f"{param_h_path}: 认不出的枚举值 {name} = {val}")
        else:
            name = item
            cur = 0 if cur is None else cur + 1
        ids[name] = cur
    return ids


ROW_RE = re.compile(
    r'P_\(\s*([A-Za-z0-9_]+)\s*,\s*"([^"]+)"\s*,\s*'
    r"([-+]?[0-9.]+)f?\s*,\s*([-+]?[0-9.]+)f?\s*,\s*([-+]?[0-9.]+)f?\s*\)")


def load_param_table(root: Optional[str] = None) -> list[dict]:
    """返回参数表：[{id, sym, name, default, lo, hi}, ...]（按 ID 升序）。"""
    root = root or repo_root()
    ids = parse_param_ids(os.path.join(root, "Core", "Inc", "param.h"))
    text = _strip_comments(_read(os.path.join(root, "Core", "Src", "param.c")))

    rows: list[dict] = []
    for m in ROW_RE.finditer(text):
        sym, name, dv, lo, hi = m.groups()
        if sym not in ids:
            raise ValueError(f"param.c 里的 {sym} 在 param.h 里找不到 ID")
        rows.append({
            "id": ids[sym],
            "sym": sym,
            "name": name,
            "default": float(dv),
            "lo": float(lo),
            "hi": float(hi),
        })

    rows.sort(key=lambda r: r["id"])
    if len(rows) != PARAM_COUNT:
        raise ValueError(f"参数表解析出 {len(rows)} 项，期望 {PARAM_COUNT} 项"
                         f"（param.c 的表格结构改了吗？）")
    for i in range(1, len(rows)):
        if rows[i - 1]["id"] >= rows[i]["id"]:
            raise ValueError(f"参数表没按 ID 升序：{rows[i - 1]['name']} >= {rows[i]['name']}")
    return rows


def table_maps(root: Optional[str] = None):
    """(id→行, name→行) 两个字典。"""
    rows = load_param_table(root)
    return ({r["id"]: r for r in rows}, {r["name"]: r for r in rows})


# ==================================================================
#  三、数值格式化 / 预设文件
# ==================================================================
def fmt_value(v: float) -> str:
    """把 float 变成固件能解析的十进制串（不用科学计数法）。"""
    s = f"{v:.6f}".rstrip("0").rstrip(".")
    return s if s not in ("", "-") else "0"


def preset_load(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as fp:
        data = json.load(fp)
    if not isinstance(data, dict) or "params" not in data:
        raise SystemExit(f"{path}: 不是预设文件（缺少 params 字段）")
    if not isinstance(data["params"], dict):
        raise SystemExit(f"{path}: params 必须是 {{参数名: 数值}} 的形式")
    for k, v in data["params"].items():
        if not isinstance(v, (int, float)) or isinstance(v, bool):
            raise SystemExit(f"{path}: 参数 {k} 的值 {v!r} 不是数字")
    return data


def preset_build(params: dict, name: str, note: str = "") -> dict:
    return {
        "name": name,
        "note": note,
        "created": time.strftime("%Y-%m-%d"),
        "fw": "flash_ee v1 / param " + str(PARAM_COUNT),
        "params": {k: round(float(v), 6) for k, v in sorted(params.items())},
    }


# ==================================================================
#  四、串口收发
# ==================================================================
def _send(ser, cmd: str, action: str, echo: bool = False):
    """发一条 CFG: 命令并收行，直到出现该命令的结束标志。

    返回 (lines, ok)。ok=False 表示超时（没等到结束标志）。
    """
    from bb_serial import read_line

    terms = TERMS[action]
    timeout = TIMEOUTS[action]

    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()

    lines: list[str] = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(ser, timeout=max(0.2, deadline - time.time()))
        if not line:
            continue
        if echo:
            print(f"  <- {line}")
        lines.append(line)
        if line.startswith(terms):
            return lines, True
    return lines, False


def _to_num(v: str):
    """尽量把 '1234' / '0x1F' / '1234ms' 变成 int，否则原样返回字符串。

    为什么要处理单位后缀：固件有些字段带单位（`up=1234ms`），
    直接 int() 会失败退化成字符串，调用方做算术时就炸了。
    （`used=47/128` 这种保留字符串，调用方只拿来显示。）
    """
    try:
        return int(v, 0)
    except ValueError:
        pass
    m = re.match(r"^(-?\d+)(?:ms|us|s|%)$", v)
    if m:
        return int(m.group(1))
    return v


def _kv(line: str) -> dict:
    """把 'CFG:INFO a=1 b=2' 变成 {'a':1,'b':2}（认不出的值保留字符串）。"""
    out = {}
    for field in line.split()[1:]:
        if "=" not in field:
            continue
        k, v = field.split("=", 1)
        out[k] = _to_num(v)
    return out


# ---------- 各命令 ----------
def cfg_info(ser) -> tuple[dict, bool]:
    lines, ok = _send(ser, "CFG:INFO", "info")
    for ln in lines:
        if ln.startswith("CFG:INFO"):
            return _kv(ln), ok
    return {}, False


PARAM_LINE_RE = re.compile(
    r"^CFG:P\s+(?:0x)?([0-9A-Fa-f]{1,4})\s+(\S+)\s+([-+0-9.]+)\s*"
    r"(applied=([-+0-9.]+))?\s*(\*?)\s*$")


def _parse_param_line(line: str) -> Optional[dict]:
    m = PARAM_LINE_RE.match(line.strip())
    if not m:
        return None
    return {
        "id": int(m.group(1), 16),
        "name": m.group(2),
        "value": float(m.group(3)),
        "applied": float(m.group(5)) if m.group(5) else None,
        "dirty": m.group(6) == "*",
    }


def cfg_list(ser) -> tuple[list[dict], bool]:
    lines, ok = _send(ser, "CFG:LIST", "list")
    out = []
    for ln in lines:
        p = _parse_param_line(ln)
        if p:
            out.append(p)
    return out, ok


def cfg_get(ser, key: str) -> tuple[Optional[dict], list[str], bool]:
    lines, ok = _send(ser, f"CFG:GET {key}", "get")
    for ln in lines:
        p = _parse_param_line(ln)
        if p:
            return p, lines, ok
    return None, lines, ok


def cfg_set(ser, key: str, value: float) -> tuple[bool, str, list[str]]:
    cmd = f"CFG:SET {key} {fmt_value(value)}"
    lines, ok = _send(ser, cmd, "set")
    for ln in lines:
        if ln.startswith("CFG:OK"):
            return True, ln, lines
        if ln.startswith("CFG:ERR"):
            return False, ln, lines
    return False, f"超时（{TIMEOUTS['set']}s 内没有回包）", lines


def cfg_save(ser) -> tuple[bool, str]:
    lines, ok = _send(ser, "CFG:SAVE", "save")
    for ln in lines:
        if ln.startswith(("CFG:SAVED", "CFG:OK", "CFG:ERR")):
            return (not ln.startswith("CFG:ERR")), ln
    return False, f"超时（{TIMEOUTS['save']}s；可能是 bank 写满正在擦除，可重试）"


def cfg_simple(ser, cmd: str, action: str) -> tuple[bool, list[str]]:
    lines, ok = _send(ser, cmd, action)
    good = any(ln.startswith(TERMS[action][0]) for ln in lines) and \
        not any(ln.startswith("CFG:ERR") for ln in lines)
    return good, lines


def cfg_hist(ser, n: int) -> tuple[list[str], dict, bool]:
    lines, ok = _send(ser, f"CFG:HIST {n}", "hist")
    tail = {}
    for ln in lines:
        if ln.startswith("CFG:HEND"):
            tail = _kv(ln)
    return [ln for ln in lines if ln.startswith("CFG:H ")], tail, ok


def cfg_cal(ser) -> tuple[list[str], bool]:
    """CFG:CAL —— 让飞机重新标定零偏（地面、静止约 2 秒）。

    固件会在 IMU 任务里采样 2 秒，然后把 6 个结果写进**待生效**参数表
    （还要 CFG:SAVE + CFG:REBOOT 才真正用上）。回包形如：
        CFG:CAL start (keep still ~2s)
        CFG:CAL gx=+0.421 gy=-0.114 gz=+0.008 dps
        CFG:CAL ax=-0.0027 ay=-0.0034 az=-0.0016 g std=0.081
        CFG:CALEND items=6 dirty=1
    """
    return _send(ser, "CFG:CAL", "cal")


def cfg_raw(ser) -> tuple[Optional[bytes], list[str]]:
    """CFG:RAW：一行表头 + ceil(len/16) 行十六进制。

    固件没有给这条命令加结束标志（P2 时刻意省掉了），所以按表头里的 len
    算出要收多少行。收不齐就返回已经拿到的部分。
    """
    from bb_serial import read_line

    ser.reset_input_buffer()
    ser.write(b"CFG:RAW\r\n")
    ser.flush()

    lines: list[str] = []
    hdr: Optional[dict] = None
    deadline = time.time() + TIMEOUTS["raw"]
    while time.time() < deadline:
        line = read_line(ser, timeout=2.0)
        if not line:
            continue
        lines.append(line)
        if line.startswith("CFG:ERR"):
            return None, lines
        if line.startswith("CFG:RAW"):
            hdr = _kv(line)
            break
    if hdr is None or "len" not in hdr:
        return None, lines

    need = (int(hdr["len"]) + 15) // 16
    data = bytearray()
    for _ in range(need):
        line = read_line(ser, timeout=2.0)
        if not line:
            break
        lines.append(line)
        m = re.match(r"^[0-9A-Fa-f]{8}:((?:\s+[0-9A-Fa-f]{2})*)\s*$", line.strip())
        if not m:
            break
        data += bytes(int(b, 16) for b in m.group(1).split())
    return bytes(data), lines


# ==================================================================
#  五、友好化：把固件的错误码翻译成人话
# ==================================================================
ERROR_HINTS = {
    "armed":     "飞机处于解锁状态。先上锁再存盘/擦除/重启。",
    "img_overlap": "镜像长进了保留区，配置区已锁成只读。检查 Keil 的 IROM 是否为 0xC0000。",
    "name/id":   "参数名或 ID 不存在。用 `bb.py cfg list` 看可用的名字；"
                 "ID 必须带 0x 前缀（如 0x0118），直接写 0118 会被当成十进制 118。",
    "value":     "数值解析失败（只接受十进制，如 12 / -0.3 / 0.008）。",
    "range":     "序号超出可回滚范围。先 `bb.py cfg hist` 看有哪些。",
    "flash":     "Flash 擦写失败，本次上电已锁成只读，重新上电再试。",
    "verify":    "写入后回读校验失败，本次上电已锁成只读，重新上电再试。",
    "locked":    "本次上电已锁成只读（之前的擦写出过错），重新上电。",
    "save":      "存盘被拒绝（可能是参数表未初始化或 Flash 不可写）。",
    "rollback":  "回滚失败（记录解不出来或写入失败）。",
    "decode":    "历史记录解不出来（格式不符）。",
    "norec":     "还没有任何有效记录。",
    "unknown":   "命令不认识。`bb.py cfg help` 看不到（help 走固件 CFG:HELP）。",
}


def explain(line: str) -> str:
    """把 'CFG:ERR armed' 变成带建议的多行文本。"""
    if not line.startswith("CFG:ERR"):
        return line
    code = line[len("CFG:ERR"):].strip().split(" ")[0] if line.strip() != "CFG:ERR" else ""
    hint = ERROR_HINTS.get(code, "")
    return f"{line}" + (f"\n         -> {hint}" if hint else "")
