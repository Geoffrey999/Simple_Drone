#!/usr/bin/env python3
"""飞控参数面板（配置存储 B 档核心 GUI）。

覆盖范围（B 档核心）：
    · 串口连接 / 断开，状态栏（src / dirty / seq / 开机时长）
    · 68 项参数表：搜索、只看改动、选中改值、一键应用
    · 保存到 Flash / 重启生效 / 重新载入 / 恢复默认
    · 历史列表 + 一键回滚（加载到内存 / 回滚并保存）+ 擦除配置区
    · 预设文件：预览差异、一键刷入、从飞机导出
    · 日志页：所有命令与回包原文

不在本档里（C 档）：黑匣子 dump / 离线分析 / 出图。

运行：
    python bb_gui.py                 # 打开窗口
    python bb_gui.py --selftest      # 不接硬件、不开窗口，自检解析与命令构造

设计要点：
    · **串口只归 worker 线程所有**。GUI 线程绝不直接 read/write —— USART3 上
      文本和二进制混跑，两边同时读会抢字节。所有操作排队交给唯一的工作线程。
    · 任何"打印"都进日志页，不留隐形输出。
"""

from __future__ import annotations

import os
import queue
import re
import struct
import sys
import threading
import time
import traceback

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import bb_cfg  # noqa: E402   纯逻辑，可离线导入

APP_TITLE = "飞控地面站"
POLL_MS = 60                 # GUI 轮询 worker 的间隔
BAUD_CHOICES = ["460800", "115200", "921600", "230400"]
PRESET_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "presets")
HIST_ROWS = 10

# 固件倾倒格式：每个扇区 32B 索引头 + 3840B 数据区 = 3872B
DUMP_HDR_LEN = 32
SECTOR_DATA_LEN = 3840
PER_SECTOR = DUMP_HDR_LEN + SECTOR_DATA_LEN
FRAME_MAGIC = 0x4246         # 'BF'
FRAME_SIZE = 32
TYPE_DATA = 1                # 1=飞行数据帧 2=诊断帧 3=摇杆帧
DUMP_CHUNK = 64              # 一次 BB:DUMP 要多少个扇区（往回倒的步长）


# ==================================================================
#  一、串口工作线程：串口的唯一持有者
# ==================================================================
class Worker(threading.Thread):
    """单线程串口执行器。

    GUI 通过 submit(fn, tag) 投递任务；fn(worker, log) 在工作线程里跑，
    可以阻塞、可以直接用 worker.ser。结果通过 out 队列回给 GUI。
    """

    def __init__(self) -> None:
        super().__init__(daemon=True, name="bb-serial-worker")
        self.jobs: queue.Queue = queue.Queue()
        self.out: queue.Queue = queue.Queue()
        self.ser = None
        self.port: str | None = None
        self.baud: int = 460800
        self.t0 = time.time()

    # ---- GUI 侧调用 ----
    def submit(self, fn, tag: str) -> None:
        self.jobs.put((fn, tag))

    def stop(self) -> None:
        self.jobs.put((None, "stop"))

    # ---- 工作线程 ----
    def run(self) -> None:
        while True:
            fn, tag = self.jobs.get()
            if fn is None:
                return
            try:
                result = fn(self, self._log)
            except Exception as exc:                       # noqa: BLE001
                self.out.put(("err", tag, exc, traceback.format_exc()))
            else:
                self.out.put(("ok", tag, result, ""))

    def _log(self, text: str) -> None:
        """线程安全的日志投递（任意线程可调）。"""
        self.out.put(("log", "", text, ""))


# ==================================================================
#  二、小工具
# ==================================================================
def _fmt(v: float) -> str:
    return f"{v:.3f}"


def _parse_hist_line(line: str) -> dict | None:
    """把 'CFG:H 0 seq=395 slot=47 diff: ...' 拆成字段。"""
    import re
    m = re.match(r"^CFG:H\s+(\d+)\s+seq=(\d+)\s+slot=(\d+)\s*(.*)$", line.strip())
    if not m:
        return None
    return {"idx": int(m.group(1)), "seq": int(m.group(2)),
            "slot": int(m.group(3)), "detail": m.group(4).strip()}


def _list_presets() -> list[str]:
    if not os.path.isdir(PRESET_DIR):
        return []
    return sorted(f for f in os.listdir(PRESET_DIR) if f.lower().endswith(".json"))


# ==================================================================
#  黑匣子倾倒：按扇区范围取，只取"最新一段飞行"
# ==================================================================
def _contig_runs(phys_list: list[int]) -> list[tuple[int, int]]:
    """把物理扇区号列表切成 [(起始, 个数)]；不连续或回绕就断开。

    固件 BB:DUMP:<start>:<count> 只接受升序、且 start+count <= 4096 的范围，
    所以环形缓冲区跨过 0 号扇区时必须拆成两段。
    """
    out: list[tuple[int, int]] = []
    if not phys_list:
        return out
    start = prev = phys_list[0]
    for p in phys_list[1:]:
        if p == prev + 1:
            prev = p
        else:
            out.append((start, prev - start + 1))
            start = prev = p
    out.append((start, prev - start + 1))
    return out


def _sector_frames(blob: bytes):
    """遍历一个扇区原始块里的帧，产出 (槽号, 类型, seq)。"""
    n = min(SECTOR_DATA_LEN // FRAME_SIZE,
            max(0, (len(blob) - DUMP_HDR_LEN) // FRAME_SIZE))
    for k in range(n):
        off = DUMP_HDR_LEN + k * FRAME_SIZE
        magic, ftype = struct.unpack_from("<HB", blob, off)
        if magic != FRAME_MAGIC:
            continue
        yield k, ftype, struct.unpack_from("<H", blob, off + 4)[0]


def _sector_run_start(blob: bytes):
    """这个扇区里是否存在 seq 1..8 的**数据帧** —— 有就说明它是某次飞行的起点。

    ⚠ 只能看 type==1 的飞行数据帧：实测每个扇区的第 0 个槽位常常是
    诊断帧/摇杆帧（type 2/3），而它们那个"seq"槽位存的是 conf/period
    （会读出 2563 这种值），直接读第一个帧会把起点判断全部带偏。
    """
    for _k, ftype, seq in _sector_frames(blob):
        if ftype == TYPE_DATA and 1 <= seq <= 8:
            return seq
    return None


def _sector_last_seq(blob: bytes):
    """扇区里最后一个**数据帧**的 seq（没有数据帧返回 None）。"""
    last = None
    for _k, ftype, seq in _sector_frames(blob):
        if ftype == TYPE_DATA:
            last = seq
    return last


def _assemble_image(pieces: dict[int, bytes]) -> bytearray:
    """把 {物理扇区: 3872B 原始块} 拼成完整 16MB 镜像（空洞填 0xFF）。

    为什么要拼成 16MB 而不是直接存紧凑流：bb_format.parse_flash() 是按
    整片镜像写的，紧凑流只有在"从 0 号扇区开始"时才会碰巧解析正确。
    """
    from bb_format import FLASH_BYTES, SECTOR_DATA_OFF, SECTOR_SIZE

    img = bytearray(b"\xFF" * FLASH_BYTES)
    for phys, blob in pieces.items():
        base = phys * SECTOR_SIZE
        hdr = blob[:DUMP_HDR_LEN]
        data = blob[DUMP_HDR_LEN:DUMP_HDR_LEN + SECTOR_DATA_LEN]
        img[base:base + len(hdr)] = hdr
        img[base + SECTOR_DATA_OFF:base + SECTOR_DATA_OFF + len(data)] = data
    return img


def _bb_dump_range(ser, start: int, count: int) -> bytes:
    """下发 BB:DUMP:<起始物理扇区>:<扇区数>，收完原始字节。"""
    from bb_serial import read_line

    ser.reset_input_buffer()
    ser.write(f"BB:DUMP:{start}:{count}\r\n".encode("ascii"))
    ser.flush()

    header = ""
    for _ in range(10):
        line = read_line(ser, timeout=15.0)
        if not line:
            continue
        if line.startswith("BB:ERR"):
            raise RuntimeError(f"倾倒被拒绝：{line}")
        if line.startswith("BB:DUMP"):
            header = line
            break
    if not header:
        raise RuntimeError("没等到 BB:DUMP 表头（飞机解锁了？Flash 没识别？）")

    info = {}
    for field in header.split():
        if "=" in field:
            k, v = field.split("=", 1)
            try:
                info[k] = int(v)
            except ValueError:
                info[k] = v
    total = int(info.get("bytes", 0))
    if total <= 0:
        raise RuntimeError(f"固件报告的长度无效：{header!r}")

    buf = bytearray()
    while len(buf) < total:
        chunk = ser.read(min(65536, total - len(buf)))
        if not chunk:
            break
        buf += chunk

    for _ in range(6):                      # 吃掉 \r\nBB:DUMPEND\r\n
        line = read_line(ser, timeout=2.0)
        if not line or "DUMPEND" in line:
            break
    return bytes(buf)


def _bb_send(ser, cmd: str, terms, timeout: float = 8.0):
    """发一条黑匣子指令并收行，直到出现指定结束标志。

    BB 系列命令的收尾不统一：BB:STAT/CAL/PING 以 BB:OK 结尾，
    BB:SCAN 以 BB:SCANDONE 结尾，BB:ERASE 立即回 BB:ERASE START。
    所以结束标志必须逐条给，不能像 cfg 那样查表。
    """
    from bb_serial import read_line

    ser.reset_input_buffer()
    ser.write((cmd + "\r\n").encode("ascii"))
    ser.flush()

    lines: list[str] = []
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = read_line(ser, timeout=max(0.2, deadline - time.time()))
        if not line:
            continue
        lines.append(line)
        if any(line.startswith(t) for t in terms):
            return lines, True
    return lines, False


# ==================================================================
#  三、主窗口
# ==================================================================
class App:
    def __init__(self, root) -> None:
        self.root = root
        self.worker = Worker()
        self.worker.start()

        self.busy = 0
        self.callbacks: dict[str, object] = {}
        self.params: list[dict] = []          # 最近一次 cfg list
        self.info: dict = {}
        self.connected = False
        self.closing = False

        # 黑匣子页的状态
        self.bb_info: dict = {}
        self.bb_tab = None
        self._erase_base = 0
        self._erase_t0 = 0.0
        self._erase_stop = False
        self._bb_plot_path: str | None = None
        self._trim_auto = True          # trim 框是否跟着飞机参数自动填

        self._build()
        self._tick_poll()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- 界面 ----------------
    def _build(self) -> None:
        self.root.title(APP_TITLE)
        self.root.geometry("1080x780")
        self.root.minsize(920, 640)

        self._build_status()
        self._build_top()
        self._build_notebook()

        self.bottom = tk.Label(self.root, anchor="w", text="未连接",
                               relief="sunken", bd=1, padx=6)
        self.bottom.pack(side="bottom", fill="x")

        self._update_actions()
        self.on_scan()

    def _build_top(self) -> None:
        f = ttk.Frame(self.root, padding=(8, 6, 8, 0))
        f.pack(side="top", fill="x")

        ttk.Label(f, text="串口").pack(side="left")
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(f, textvariable=self.port_var, width=12,
                                       values=[])
        self.port_combo.pack(side="left", padx=(4, 10))

        ttk.Label(f, text="波特率").pack(side="left")
        self.baud_var = tk.StringVar(value=BAUD_CHOICES[0])
        self.baud_combo = ttk.Combobox(f, textvariable=self.baud_var, width=9,
                                       values=BAUD_CHOICES)
        self.baud_combo.pack(side="left", padx=(4, 10))

        self.btn_scan = ttk.Button(f, text="扫描串口", command=self.on_scan)
        self.btn_scan.pack(side="left", padx=2)

        self.btn_conn = ttk.Button(f, text="连接", command=self.on_connect)
        self.btn_conn.pack(side="left", padx=2)

        self.conn_label = ttk.Label(f, text="● 未连接", foreground="#a00")
        self.conn_label.pack(side="left", padx=12)

    def _build_status(self) -> None:
        f = ttk.Frame(self.root, padding=(8, 4, 8, 4))
        f.pack(side="top", fill="x")
        self.status_var = tk.StringVar(value="—")
        ttk.Label(f, textvariable=self.status_var, anchor="w").pack(side="left")
        self.btn_refresh = ttk.Button(f, text="刷新状态 (F5)", command=self.on_refresh)
        self.btn_refresh.pack(side="right")

    def _build_notebook(self) -> None:
        self.nb = ttk.Notebook(self.root)
        self.nb.pack(side="top", fill="both", expand=True, padx=8, pady=(0, 4))
        self._build_tab_params()
        self._build_tab_curve()
        self._build_tab_hist()
        self._build_tab_preset()
        self._build_tab_log()
        self._build_tab_blackbox()
        self._build_tab_calib()
        self.nb.bind("<<NotebookTabChanged>>", self._on_tab_changed)
        self.root.bind("<F5>", lambda _e: self.on_refresh())
        self.root.bind("<Control-s>", lambda _e: self.on_save())
        self.root.bind("<Control-r>", lambda _e: self.on_reboot())

    # ---- 参数页 ----
    def _build_tab_params(self) -> None:
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="参数")

        bar = ttk.Frame(tab)
        bar.pack(fill="x", padx=4, pady=(6, 0))

        ttk.Label(bar, text="搜索").pack(side="left")
        self.filter_var = tk.StringVar()
        e = ttk.Entry(bar, textvariable=self.filter_var, width=24)
        e.pack(side="left", padx=(4, 10))
        e.bind("<KeyRelease>", lambda _e: self._fill_tree())

        self.only_dirty = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="只看改动", variable=self.only_dirty,
                        command=self._fill_tree).pack(side="left", padx=(0, 12))

        self.btn_load = ttk.Button(bar, text="重新载入", command=self.on_load)
        self.btn_load.pack(side="left", padx=2)
        self.btn_def = ttk.Button(bar, text="恢复默认", command=self.on_def)
        self.btn_def.pack(side="left", padx=2)

        # 参数表
        cols = ("id", "name", "value", "state")
        self.tree = ttk.Treeview(tab, columns=cols, show="headings", height=18)
        for c, txt, w, anchor in (("id", "ID", 70, "w"),
                                  ("name", "名称", 330, "w"),
                                  ("value", "待生效值", 110, "e"),
                                  ("state", "状态", 90, "w")):
            self.tree.heading(c, text=txt)
            self.tree.column(c, width=w, anchor=anchor, stretch=(c == "name"))
        self.tree.tag_configure("dirty", foreground="#c60")
        self.tree.tag_configure("hover", foreground="#06c")
        self.tree.pack(fill="both", expand=True, padx=4, pady=4)
        self.tree.bind("<<TreeviewSelect>>", self._on_select)
        self.tree.bind("<Double-1>", lambda _e: self.value_entry.focus_set())

        # 编辑区
        ed = ttk.LabelFrame(tab, text="修改参数", padding=6)
        ed.pack(fill="x", padx=4, pady=(0, 4))

        self.sel_var = tk.StringVar(value="（未选中）")
        ttk.Label(ed, textvariable=self.sel_var, width=42).pack(side="left")

        ttk.Label(ed, text="新值").pack(side="left", padx=(8, 2))
        self.value_var = tk.StringVar()
        self.value_entry = ttk.Entry(ed, textvariable=self.value_var, width=12)
        self.value_entry.pack(side="left")
        self.value_entry.bind("<Return>", lambda _e: self.on_apply_value())

        self.btn_apply = ttk.Button(ed, text="应用", command=self.on_apply_value)
        self.btn_apply.pack(side="left", padx=6)
        ttk.Label(ed, text="（应用只改内存，要 SAVE + 重启才生效）",
                  foreground="#666").pack(side="left")

        # 生效动作
        act = ttk.Frame(tab)
        act.pack(fill="x", padx=4, pady=(0, 8))
        self.btn_save = ttk.Button(act, text="保存到 Flash (Ctrl+S)", command=self.on_save)
        self.btn_save.pack(side="left")
        self.btn_reboot = ttk.Button(act, text="重启生效 (Ctrl+R)", command=self.on_reboot)
        self.btn_reboot.pack(side="left", padx=6)

        self.dirty_label = ttk.Label(act, text="", foreground="#c60")
        self.dirty_label.pack(side="left", padx=12)

    # ---- 油门曲线页（手动模式专用）----
    #
    # 固件侧：freertos.c 的 FlyControl_ThrottleCurve()，只作用于 HORIZON。
    #   摇杆 1000..2000 → 输出比例 0..1，四点 (1000,0)(x1,y1)(x2,y2)(2000,1) 分段线性。
    # 这一页只做两件事：把曲线画出来（改滑块立刻重画）+ 把四个值写进参数表。
    CURVE_NAMES = ("thr.cv_x1", "thr.cv_y1", "thr.cv_x2", "thr.cv_y2")
    CURVE_DEFAULT = (1500.0, 0.50, 1700.0, 0.70)

    def _build_tab_curve(self) -> None:
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="油门曲线")

        ttk.Label(tab, justify="left", foreground="#555",
                  text="手动模式（HORIZON）专用曲线：把摇杆 1000~2000 先映射成油门输出比例 0~100%。\n"
                       "x1~x2 是“最缓”的一段（默认 1500~1700，正好罩住悬停区间）——"
                       "把 y1/y2 设得接近就能把这段压平，悬停附近更好微调；代价是 1700 以上变陡。\n"
                       "默认 (1500, 50%) (1700, 70%) 时曲线与直线重合，手动行为与以前完全一样。\n"
                       "纵轴是输出比例；实际油门 PWM ≈ 比例 × 1000 × thr.scale × 4.2 ÷ (电量×thr.comp_a + thr.comp_b)\n"
                       "（thr.scale=3、电量 70% 时，比例 0.5 → 约 1590 PWM，正好是这台机的悬停点）"
                  ).pack(anchor="w", padx=8, pady=(6, 2))

        self.cv_canvas = tk.Canvas(tab, width=580, height=330, bg="white",
                                  highlightthickness=1, highlightbackground="#ccc")
        self.cv_canvas.pack(padx=8, pady=4)
        self.cv_canvas.bind("<Configure>", lambda _e: self._draw_curve())

        self.cv_vars = {n: tk.DoubleVar(value=v)
                        for n, v in zip(self.CURVE_NAMES, self.CURVE_DEFAULT)}

        ctl = ttk.LabelFrame(tab, text="断点（改完点“写入参数表”，再 保存到 Flash + 重启 才生效）",
                             padding=6)
        ctl.pack(fill="x", padx=8, pady=(2, 4))
        ctl.columnconfigure(1, weight=1)

        self.cv_readout = {}
        rows = (("thr.cv_x1", "缓区起点 x1（摇杆）", 1000.0, 1900.0, 10.0, "{:.0f}"),
                ("thr.cv_y1", "该点输出 y1（比例）", 0.00, 0.99, 0.01, "{:.2f}"),
                ("thr.cv_x2", "缓区终点 x2（摇杆）", 1050.0, 2000.0, 10.0, "{:.0f}"),
                ("thr.cv_y2", "该点输出 y2（比例）", 0.01, 1.00, 0.01, "{:.2f}"))
        for r, (name, label, lo, hi, res, fmt) in enumerate(rows):
            ttk.Label(ctl, text=label, width=20).grid(row=r, column=0, sticky="w", pady=1)
            ttk.Scale(ctl, from_=lo, to=hi, orient="horizontal",
                      variable=self.cv_vars[name],
                      command=lambda _v: self._draw_curve()
                      ).grid(row=r, column=1, sticky="ew", padx=6)
            ro = ttk.Label(ctl, width=8, anchor="e")
            ro.grid(row=r, column=2, sticky="e")
            self.cv_readout[name] = (ro, fmt)

        act = ttk.Frame(tab)
        act.pack(fill="x", padx=8, pady=(0, 6))
        ttk.Button(act, text="写入参数表", command=self.on_apply_curve).pack(side="left")
        ttk.Button(act, text="从参数表读取", command=self.on_read_curve).pack(side="left", padx=6)
        ttk.Button(act, text="恢复线性", command=self.on_reset_curve).pack(side="left")
        ttk.Button(act, text="保存到 Flash", command=self.on_save).pack(side="left", padx=(18, 0))
        self.cv_status = ttk.Label(act, text="", foreground="#666")
        self.cv_status.pack(side="left", padx=10)

        self._draw_curve()

    # ---- 曲线计算与绘制 ----
    def _curve_points(self):
        """返回 (x1, y1, x2, y2)，已经按固件的合法性规则夹过。"""
        v = [self.cv_vars[n].get() for n in self.CURVE_NAMES]
        x1, y1, x2, y2 = v
        x1 = max(1000.0, min(1900.0, x1))
        x2 = max(1050.0, min(2000.0, x2))
        if x2 <= x1 + 20.0:            # 和固件一样：写坏就退回线性
            return 1500.0, 0.50, 1700.0, 0.70
        y1 = max(0.0, min(0.99, y1))
        y2 = max(0.01, min(1.0, y2))
        if y2 <= y1 + 0.005:
            return 1500.0, 0.50, 1700.0, 0.70
        return x1, y1, x2, y2

    @staticmethod
    def _curve_eval(stick, x1, y1, x2, y2):
        """与固件 FlyControl_ThrottleCurve() 完全一致的分段线性映射。"""
        if stick <= 1000.0:
            return 0.0
        if stick >= 2000.0:
            return 1.0
        if stick < x1:
            span = x1 - 1000.0
            return y1 * (stick - 1000.0) / span if span > 1.0 else y1
        if stick < x2:
            span = x2 - x1
            return y1 + (y2 - y1) * (stick - x1) / span if span > 1.0 else y2
        span = 2000.0 - x2
        return y2 + (1.0 - y2) * (stick - x2) / span if span > 1.0 else 1.0

    def _draw_curve(self) -> None:
        """画曲线：**横轴=摇杆（1000~2000），纵轴=油门输出比例（0~100%）**。"""
        x1, y1, x2, y2 = self._curve_points()
        for name, (lbl, fmt) in self.cv_readout.items():
            idx = self.CURVE_NAMES.index(name)
            lbl.configure(text=fmt.format((x1, y1, x2, y2)[idx]))

        cv = self.cv_canvas
        cv.delete("all")
        w = max(360, cv.winfo_width())
        h = max(240, cv.winfo_height())
        ml, mr, mt, mb = 56, 18, 16, 40
        pw, ph = w - ml - mr, h - mt - mb

        def px(stick):
            """摇杆 1000..2000 → 横坐标"""
            return ml + (stick - 1000.0) / 1000.0 * pw

        def py(val):
            """油门输出比例 0..1 → 纵坐标"""
            return mt + (1.0 - val) * ph

        # 缓区背景：竖条（摇杆在 x1..x2 之间）
        cv.create_rectangle(px(x1), mt, px(x2), mt + ph,
                            fill="#fff7e0", outline="")

        # 网格：竖线=摇杆每 100 计数，横线=输出每 20%
        for s in range(1000, 2001, 100):
            cv.create_line(px(s), mt, px(s), mt + ph, fill="#eee")
            if s % 200 == 0:
                cv.create_text(px(s), mt + ph + 6, text=str(s), anchor="n",
                               fill="#888", font=("TkDefaultFont", 8))
        for pct in range(0, 101, 20):
            v = pct / 100.0
            cv.create_line(ml, py(v), ml + pw, py(v), fill="#eee")
            cv.create_text(ml - 6, py(v), text=f"{pct}%", anchor="e",
                           fill="#888", font=("TkDefaultFont", 8))

        cv.create_text(ml - 34, mt - 10, text="油门输出", anchor="w",
                       fill="#555", font=("TkDefaultFont", 8))
        cv.create_text(ml + pw, mt + ph + 22, text="摇杆 →", anchor="e",
                       fill="#555", font=("TkDefaultFont", 8))

        # 原来的线性映射（参考虚线）
        cv.create_line(px(1000.0), py(0.0), px(2000.0), py(1.0),
                       fill="#bbb", dash=(4, 4))
        cv.create_text(px(1240.0), py(0.10), text="原线性", anchor="w",
                       fill="#aaa", font=("TkDefaultFont", 8))

        # 曲线本体
        pts = []
        for i in range(0, 101):
            s = 1000.0 + i * 10.0
            pts += [px(s), py(self._curve_eval(s, x1, y1, x2, y2))]
        cv.create_line(*pts, fill="#1f6feb", width=2)

        # 四个断点
        for sx, sy, col, tag in ((1000.0, 0.0, "#1f6feb", "1000"),
                                 (x1, y1, "#d97706", "x1"),
                                 (x2, y2, "#d97706", "x2"),
                                 (2000.0, 1.0, "#1f6feb", "2000")):
            cv.create_oval(px(sx) - 4, py(sy) - 4, px(sx) + 4, py(sy) + 4,
                           fill=col, outline="")
            cv.create_text(px(sx), py(sy) - 12, text=tag, fill=col,
                           font=("TkDefaultFont", 8, "bold"))

        # 缓区斜率读数
        seg_slope = (y2 - y1) / max(1.0, (x2 - x1)) * 100.0          # %/100 计数
        finer = (0.1 / seg_slope) if seg_slope > 1e-6 else 0.0       # 相对线性细多少倍
        cv.create_text(px((x1 + x2) / 2.0), mt + ph - 10,
                       text=f"缓区 {seg_slope:.3f}%/100 → 比线性细 {finer:.1f} 倍",
                       fill="#b45309", font=("TkDefaultFont", 8))

    # ---- 曲线页的两个动作 ----
    def on_apply_curve(self) -> None:
        vals = list(self._curve_points())
        names = list(self.CURVE_NAMES)

        def task(w, log):
            ok_all = True
            for nm, v in zip(names, vals):
                ok, line, _ = bb_cfg.cfg_set(w.ser, nm, float(v))
                log(f"  {nm} = {v:g}  {bb_cfg.explain(line) if not ok else 'OK'}")
                ok_all = ok_all and ok
            params, _ = bb_cfg.cfg_list(w.ser)
            info, _ = bb_cfg.cfg_info(w.ser)
            return ok_all, params, info

        self.cv_status.configure(text="写入中…")
        self._submit(task, "curve_set", self._after_curve_set, "写入油门曲线参数…")

    def _after_curve_set(self, payload) -> None:
        ok, params, info = payload
        self._apply_info(info)
        self.params = params
        self._fill_tree()
        if ok:
            self.cv_status.configure(text="已写入内存（还需 保存到 Flash + 重启）")
            self._set_status("油门曲线参数已写入内存")
        else:
            self.cv_status.configure(text="写入失败，看日志")
            self._set_status("油门曲线参数写入失败")

    def on_read_curve(self) -> None:
        if self._sync_curve_from_params():
            self.cv_status.configure(text="已从参数表读取")
        else:
            self.cv_status.configure(text="参数表里没有 thr.cv_*（固件是旧版？）")

    def on_reset_curve(self) -> None:
        for n, v in zip(self.CURVE_NAMES, self.CURVE_DEFAULT):
            self.cv_vars[n].set(v)
        self.cv_status.configure(text="已恢复线性（未写入）")
        self._draw_curve()

    def _sync_curve_from_params(self) -> bool:
        """把参数表里的 thr.cv_* 读进滑块。返回是否全部找到。"""
        got = {}
        for p in self.params:
            if p["name"] in self.CURVE_NAMES:
                got[p["name"]] = float(p["value"])
        if len(got) != len(self.CURVE_NAMES):
            return False
        for n, v in zip(self.CURVE_NAMES, self.CURVE_DEFAULT):
            self.cv_vars[n].set(got.get(n, v))
        self._draw_curve()
        return True

    # ---- 历史页 ----
    def _build_tab_hist(self) -> None:
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="历史 / 回滚")

        cols = ("idx", "seq", "slot", "detail")
        self.hist_tree = ttk.Treeview(tab, columns=cols, show="headings", height=16)
        for c, txt, w, anchor in (("idx", "序号", 60, "center"),
                                  ("seq", "Seq", 80, "center"),
                                  ("slot", "槽位", 70, "center"),
                                  ("detail", "该版本与上一条的差异", 620, "w")):
            self.hist_tree.heading(c, text=txt)
            self.hist_tree.column(c, width=w, anchor=anchor, stretch=(c == "detail"))
        self.hist_tree.pack(fill="both", expand=True, padx=4, pady=6)

        bar = ttk.Frame(tab)
        bar.pack(fill="x", padx=4, pady=(0, 6))
        ttk.Button(bar, text="刷新历史", command=self.on_hist_refresh).pack(side="left")
        self.btn_hist_load = ttk.Button(bar, text="加载到内存（不写盘）",
                                        command=lambda: self.on_hist_restore(False))
        self.btn_hist_load.pack(side="left", padx=6)
        self.btn_hist_roll = ttk.Button(bar, text="回滚并保存",
                                        command=lambda: self.on_hist_restore(True))
        self.btn_hist_roll.pack(side="left")
        self.btn_hist_reboot = ttk.Button(bar, text="重启生效", command=self.on_reboot)
        self.btn_hist_reboot.pack(side="left", padx=6)

        self.btn_erase = ttk.Button(bar, text="擦除配置区…", command=self.on_erase)
        self.btn_erase.pack(side="right")

        ttk.Label(tab, foreground="#666",
                  text="说明：序号 0 是最新一条。回滚也要写 Flash，所以同样要求未解锁；"
                       "回滚后需要重启才生效。").pack(anchor="w", padx=6, pady=(0, 6))

    # ---- 预设页 ----
    def _build_tab_preset(self) -> None:
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="预设")

        left = ttk.Frame(tab)
        left.pack(side="left", fill="y", padx=(4, 8), pady=6)

        ttk.Label(left, text=f"目录：{PRESET_DIR}", foreground="#666").pack(anchor="w")
        self.preset_list = tk.Listbox(left, width=34, height=16)
        self.preset_list.pack(fill="y", expand=True, pady=4)
        self.preset_list.bind("<<ListboxSelect>>", lambda _e: self.on_preset_preview())

        row = ttk.Frame(left)
        row.pack(fill="x")
        ttk.Button(row, text="刷新", command=self.on_preset_refresh).pack(side="left")
        ttk.Button(row, text="打开目录", command=self.on_open_presets).pack(side="left", padx=4)

        right = ttk.Frame(tab)
        right.pack(side="left", fill="both", expand=True, padx=(0, 4), pady=6)

        ttk.Label(right, text="与飞机当前值的差异：").pack(anchor="w")
        self.preset_diff = tk.Text(right, height=14, wrap="none")
        self.preset_diff.pack(fill="both", expand=True, pady=4)
        self.preset_diff.configure(state="disabled")

        act = ttk.Frame(right)
        act.pack(fill="x")
        self.preset_save = tk.BooleanVar(value=True)
        self.preset_reboot = tk.BooleanVar(value=True)
        ttk.Checkbutton(act, text="应用后保存", variable=self.preset_save).pack(side="left")
        ttk.Checkbutton(act, text="保存后重启", variable=self.preset_reboot).pack(side="left", padx=8)
        self.btn_preset_apply = ttk.Button(act, text="应用到飞机",
                                           command=self.on_preset_apply)
        self.btn_preset_apply.pack(side="left", padx=8)
        ttk.Button(act, text="从飞机导出…", command=self.on_preset_export).pack(side="right")

        self.on_preset_refresh()

    # ---- 日志页 ----
    def _build_tab_log(self) -> None:
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="日志")

        bar = ttk.Frame(tab)
        bar.pack(fill="x", padx=4, pady=4)
        ttk.Button(bar, text="清空", command=self._clear_log).pack(side="left")
        ttk.Label(bar, text="所有命令与回包原文（出问题先看这里）",
                  foreground="#666").pack(side="left", padx=8)

        self.log = tk.Text(tab, wrap="none", height=20)
        self.log.pack(fill="both", expand=True, padx=4, pady=(0, 6))
        self.log.configure(state="disabled")

    # ---- 黑匣子页 ----
    def _build_tab_blackbox(self) -> None:
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text="黑匣子")
        self.bb_tab = tab

        # 顶部：状态 + 常用动作
        top = ttk.Frame(tab)
        top.pack(fill="x", padx=4, pady=(6, 2))

        self.btn_bb_status = ttk.Button(top, text="刷新状态", command=self.on_bb_status)
        self.btn_bb_status.pack(side="left")
        self.btn_bb_scan = ttk.Button(top, text="扫描扇区", command=self.on_bb_scan)
        self.btn_bb_scan.pack(side="left", padx=4)
        self.btn_bb_cal = ttk.Button(top, text="校准报告", command=self.on_bb_cal)
        self.btn_bb_cal.pack(side="left")
        self.btn_bb_erase = ttk.Button(top, text="擦除全片…", command=self.on_bb_erase)
        self.btn_bb_erase.pack(side="left", padx=4)
        self.btn_bb_erase_stop = ttk.Button(top, text="停止等待", command=self.on_bb_erase_stop,
                                            state="disabled")
        self.btn_bb_erase_stop.pack(side="left")

        self.bb_status_var = tk.StringVar(value="（未查询）")
        ttk.Label(top, textvariable=self.bb_status_var,
                  foreground="#060").pack(side="left", padx=10)

        # 倾倒
        dump = ttk.LabelFrame(tab, text="倾倒 Flash 到本地文件（未解锁才受理）", padding=6)
        dump.pack(fill="x", padx=4, pady=4)

        row = ttk.Frame(dump)
        row.pack(fill="x")
        ttk.Label(row, text="输出").pack(side="left")
        self.bb_path_var = tk.StringVar(
            value=os.path.join(os.path.dirname(os.path.abspath(__file__)), "flash.bin"))
        ttk.Entry(row, textvariable=self.bb_path_var, width=44).pack(side="left", padx=4)
        ttk.Button(row, text="浏览…", command=self.on_bb_browse).pack(side="left")

        opt = ttk.Frame(dump)
        opt.pack(fill="x", pady=(6, 0))
        self.bb_mode = tk.StringVar(value="run")
        ttk.Radiobutton(opt, text="最近", value="run",
                        variable=self.bb_mode).pack(side="left")
        self.bb_runs_var = tk.StringVar(value="1")
        ttk.Spinbox(opt, from_=1, to=20, width=3,
                    textvariable=self.bb_runs_var).pack(side="left", padx=2)
        ttk.Label(opt, text="段飞行").pack(side="left")
        ttk.Radiobutton(opt, text="全部已记录", value="all",
                        variable=self.bb_mode).pack(side="left", padx=(12, 0))
        self.btn_bb_dump = ttk.Button(opt, text="开始倾倒", command=self.on_bb_dump)
        self.btn_bb_dump.pack(side="left", padx=10)

        prog = ttk.Frame(dump)
        prog.pack(fill="x", pady=(6, 0))
        self.bb_pb = ttk.Progressbar(prog, mode="determinate", maximum=100, length=320)
        self.bb_pb.pack(side="left")
        self.bb_prog_var = tk.StringVar(value="")
        ttk.Label(prog, textvariable=self.bb_prog_var).pack(side="left", padx=8)

        # 下半：扇区索引 | 离线分析
        pane = ttk.Panedwindow(tab, orient="horizontal")
        pane.pack(fill="both", expand=True, padx=4, pady=(0, 6))

        left = ttk.Frame(pane)
        ttk.Label(left, text="扇区索引（扫描能看出录了几段）").pack(anchor="w")
        cols = ("idx", "phys", "off")
        self.bb_tree = ttk.Treeview(left, columns=cols, show="headings", height=8)
        for c, txt, w, anchor in (("idx", "序号", 60, "center"),
                                  ("phys", "物理扇区", 90, "center"),
                                  ("off", "已写帧数", 90, "center")):
            self.bb_tree.heading(c, text=txt)
            self.bb_tree.column(c, width=w, anchor=anchor)
        self.bb_tree.pack(fill="both", expand=True, pady=4)
        pane.add(left, weight=1)

        right = ttk.Frame(pane)
        ttk.Label(right, text="离线分析（本地文件，不占用串口）").pack(anchor="w")
        arow = ttk.Frame(right)
        arow.pack(fill="x")
        self.bb_plot = tk.BooleanVar(value=True)
        ttk.Checkbutton(arow, text="出图", variable=self.bb_plot).pack(side="left")
        self.btn_bb_analyze = ttk.Button(arow, text="分析这个文件", command=self.on_bb_analyze)
        self.btn_bb_analyze.pack(side="left", padx=6)
        self.btn_bb_csv = ttk.Button(arow, text="导出 CSV…", command=self.on_bb_csv)
        self.btn_bb_csv.pack(side="left")
        self.btn_bb_openplot = ttk.Button(arow, text="打开曲线图",
                                          command=self.on_bb_open_plot, state="disabled")
        self.btn_bb_openplot.pack(side="left", padx=6)

        # trim 归零：把 ctrl.*_offset_deg 当零点（默认**不勾** = 保留 offset 显示）
        trow = ttk.Frame(right)
        trow.pack(fill="x", pady=(4, 0))
        self.trim_on = tk.BooleanVar(value=False)
        ttk.Checkbutton(trow, text="减去 trim（把 offset 当零点）",
                        variable=self.trim_on).pack(side="left")
        ttk.Label(trow, text="roll").pack(side="left", padx=(10, 2))
        self.trim_roll_var = tk.StringVar(value="0.000")
        e1 = ttk.Entry(trow, textvariable=self.trim_roll_var, width=8)
        e1.pack(side="left")
        ttk.Label(trow, text="pitch").pack(side="left", padx=(8, 2))
        self.trim_pitch_var = tk.StringVar(value="0.000")
        e2 = ttk.Entry(trow, textvariable=self.trim_pitch_var, width=8)
        e2.pack(side="left")
        for e in (e1, e2):
            e.bind("<KeyRelease>", self._on_trim_edited)
        ttk.Button(trow, text="从飞机读", command=self.on_trim_from_fc).pack(side="left", padx=6)

        self.bb_report = tk.Text(right, wrap="none", height=10)
        self.bb_report.pack(fill="both", expand=True, pady=4)
        self.bb_report.configure(state="disabled")
        pane.add(right, weight=2)

    # ---- 校准页 ----
    def _build_tab_calib(self) -> None:
        tab = ttk.Frame(self.nb, padding=10)
        self.nb.add(tab, text="校准")
        self.cal_tab = tab

        ttk.Label(tab, text="传感器校准（档 2：只标零偏）",
                  font=("Segoe UI", 11, "bold")).pack(anchor="w")
        ttk.Label(tab, wraplength=780, justify="left",
                  text="实测陀螺零偏和加计零偏并存进配置，下次上电直接减掉，"
                       "不必再要求“上电后保持静止 2 秒”。刻度(scale)保持 1.0 —— "
                       "六面标定留到以后（代码已在 ICM42605_SixSideCalibrate）。"
                  ).pack(anchor="w", pady=(2, 10))

        grid = ttk.Frame(tab)
        grid.pack(anchor="w")
        self.cal_vars: dict[str, tk.StringVar] = {}
        rows = (("陀螺零偏 (deg/s)",
                 (("cal.gyro_x_bias", "gx"), ("cal.gyro_y_bias", "gy"), ("cal.gyro_z_bias", "gz"))),
                ("加计零偏 (g)",
                 (("cal.accel_x_off", "ax"), ("cal.accel_y_off", "ay"), ("cal.accel_z_off", "az"))))
        for r, (title, cols) in enumerate(rows):
            ttk.Label(grid, text=title, width=16).grid(row=r, column=0, sticky="w", pady=3)
            for c, (name, short) in enumerate(cols):
                ttk.Label(grid, text=short).grid(row=r, column=1 + c * 2, padx=(10, 2))
                var = tk.StringVar(value="-")
                self.cal_vars[name] = var
                ttk.Entry(grid, textvariable=var, width=10,
                          state="readonly").grid(row=r, column=2 + c * 2)

        ttk.Label(grid, text="标定质量", width=16).grid(row=2, column=0, sticky="w", pady=(10, 3))
        self.cal_std_var = tk.StringVar(value="-")
        ttk.Label(grid, textvariable=self.cal_std_var).grid(row=2, column=1, columnspan=3, sticky="w")
        ttk.Label(grid, text="（std < 0.5 dps 才可靠；偏大说明标定时被碰过）"
                  ).grid(row=2, column=4, columnspan=3, sticky="w")

        btn = ttk.Frame(tab)
        btn.pack(fill="x", pady=(14, 6))
        self.btn_cal_run = ttk.Button(btn, text="标定（静止 2 秒）", command=self.on_cal_run)
        self.btn_cal_run.pack(side="left")
        ttk.Button(btn, text="刷新", command=self.on_refresh).pack(side="left", padx=6)
        ttk.Button(btn, text="保存到 Flash", command=self.on_save).pack(side="left")
        ttk.Button(btn, text="重启生效", command=self.on_reboot).pack(side="left", padx=6)

        ttk.Label(tab, justify="left", wraplength=780,
                  text="① 飞机放平、别动 → ② 点「标定（静止 2 秒）」→ ③ 看标定质量 std < 0.5\n"
                       "→ ④ 保存到 Flash → ⑤ 重启生效。值是“待生效”时后面会带 *。\n"
                       "解锁状态下会被拒绝（标定要占 IMU 2 秒，空中做等于失控）。"
                  ).pack(anchor="w", pady=(8, 0))

    def _update_cal_view(self) -> None:
        """把 cal.* 参数显示到校准页（待生效的加 *）。"""
        vals = {p["name"]: p for p in self.params}
        for name, var in getattr(self, "cal_vars", {}).items():
            p = vals.get(name)
            var.set("-" if not p else _fmt(p["value"]) + (" *" if p["dirty"] else ""))

    def on_cal_run(self) -> None:
        if not self._bb_require():
            return

        def task(w, log):
            return _bb_send(w.ser, "CFG:CAL", ("CFG:CALEND", "CFG:ERR"), 15.0)

        self._submit(task, "cal", self._after_cal, "标定中（保持静止约 2 秒）…")

    def _after_cal(self, payload) -> None:
        lines, ok = payload
        for ln in lines:
            self._drain_log("  " + ln)
        if not ok:
            self._set_status("标定失败/被拒绝（解锁中不能标定），看日志页")
            return
        std = None
        for ln in lines:
            m = re.search(r"std=([-+]?\d+(?:\.\d+)?)", ln)
            if m:
                std = float(m.group(1))
        if std is None:
            self.cal_std_var.set("未知")
        else:
            self.cal_std_var.set(f"{std:.3f} dps" + ("（可用）" if std <= 0.5 else "（偏大，重标）"))
        self._set_status("标定完成，已写入待生效值 —— 记得「保存到 Flash」+「重启生效」")
        self.on_refresh()

    def _on_tab_changed(self, _evt=None) -> None:
        """切到黑匣子页且还没查过状态时，自动查一次。"""
        if self.bb_tab is None or not self.connected or self.busy:
            return
        try:
            cur = self.nb.nametowidget(self.nb.select())
        except Exception:                                  # noqa: BLE001
            return
        if cur is self.bb_tab and not self.bb_info:
            self.on_bb_status()

    # ---------------- 黑匣子操作 ----------------
    def _bb_require(self) -> bool:
        if not self.connected:
            self._set_status("先连接串口")
            return False
        return True

    # ---------------- trim 归零 ----------------
    def _on_trim_edited(self, _evt=None) -> None:
        self._trim_auto = False

    def on_trim_from_fc(self) -> None:
        if not self.params:
            self._set_status("还没从飞机读到参数：先连接并刷新，或直接手填两个数")
            return
        self._trim_auto = True
        self._sync_trim_from_params(force=True)
        self._set_status(f"trim 已按飞机参数填入：roll={self.trim_roll_var.get()}  "
                         f"pitch={self.trim_pitch_var.get()}")

    def _sync_trim_from_params(self, force: bool = False) -> None:
        """把飞机上的 ctrl.*_offset_deg 填进 trim 框（用户手改过就不覆盖）。"""
        if not self.params or ((not self._trim_auto) and (not force)):
            return
        vals = {p["name"]: p["value"] for p in self.params}
        for name, var in (("ctrl.roll_offset_deg", self.trim_roll_var),
                          ("ctrl.pitch_offset_deg", self.trim_pitch_var)):
            if name in vals:
                var.set(f"{vals[name]:.3f}")

    def _trim_args(self) -> list[str]:
        """勾了「减去 trim」才返回 --trim-roll/--trim-pitch（默认不加 = 保留 offset）。"""
        if not self.trim_on.get():
            return []
        try:
            r = float(self.trim_roll_var.get() or 0.0)
            p = float(self.trim_pitch_var.get() or 0.0)
        except ValueError:
            self._set_status("trim 框里不是数字，按 0 处理")
            r = p = 0.0
        out: list[str] = []
        if abs(r) > 1e-9:
            out += ["--trim-roll", f"{r:.6g}"]
        if abs(p) > 1e-9:
            out += ["--trim-pitch", f"{p:.6g}"]
        return out

    def on_bb_status(self) -> None:
        if not self._bb_require():
            return

        def task(w, log):
            from bb_serial import parse_stat_line
            lines, ok = _bb_send(w.ser, "BB:STAT", ("BB:OK", "BB:ERR"), 8.0)
            st = {}
            for ln in lines:
                log("  " + ln)
                if ln.startswith("BB:STAT"):
                    st = parse_stat_line(ln)
            return st, ok

        self._submit(task, "bbstat", self._after_bb_status, "查询黑匣子状态…")

    def _after_bb_status(self, payload) -> None:
        st, ok = payload
        if not st:
            self._set_status("黑匣子没有回包（Flash 未识别？看日志里的 BB:FLASH 行）")
            return
        self.bb_info = st
        rec_on = st.get("rec_on")
        rec_txt = {1: "记录=开", 0: "记录=关"}.get(rec_on, "记录=?")   # 旧固件没这个字段
        self.bb_status_var.set(
            f"ok={st.get('ok')}  boot={st.get('boot')}  扇区={st.get('sec')}  "
            f"已记录={st.get('rec')} 丢帧={st.get('drop')}  "
            f"解锁={st.get('armed')}  擦除中={st.get('erasing')}  {rec_txt}")
        if not st.get("ok"):
            self._set_status("黑匣子 ok=0：W25Q128 没识别到，检查 SPI3 接线")
        else:
            self._set_status("黑匣子状态已刷新")

    def on_bb_scan(self) -> None:
        if not self._bb_require():
            return

        def task(w, log):
            lines, ok = _bb_send(w.ser, "BB:SCAN", ("BB:SCANDONE", "BB:ERR"), 25.0)
            rows = []
            for ln in lines:
                m = re.match(r"^BB:HDR\s+idx=(\d+)\s+phys=(\d+)\s+off=(\d+)", ln)
                if m:
                    rows.append(tuple(int(g) for g in m.groups()))
                else:
                    log("  " + ln)
            return rows, ok

        self._submit(task, "bbscan", self._after_bb_scan, "扫描扇区索引…")

    def _after_bb_scan(self, payload) -> None:
        rows, _ok = payload
        self.bb_tree.delete(*self.bb_tree.get_children())
        for i, (idx, phys, off) in enumerate(rows):
            self.bb_tree.insert("", "end", iid=str(i), values=(idx, phys, off))
        self._set_status(f"扫描到 {len(rows)} 个有效扇区索引头")

    def on_bb_cal(self) -> None:
        if not self._bb_require():
            return

        def task(w, log):
            lines, ok = _bb_send(w.ser, "BB:CAL", ("BB:OK", "BB:ERR"), 8.0)
            for ln in lines:
                log("  " + ln)
            return lines, ok

        self._submit(task, "bbcal", lambda r: self._set_status(
            "校准报告已输出到日志页" if r[1] else "校准查询没有回包"),
            "查询标定质量…")

    def on_bb_erase(self) -> None:
        if not self._bb_require():
            return
        if not _ask(self.root, "擦除整片 Flash？",
                    "把 16MB 全片擦掉（后台约 200 秒），黑匣子历史全部消失。\n\n"
                    "必须处于未解锁状态；擦除期间不要断电。\n\n"
                    "配置存储（内部 Flash 里的参数）不受影响。\n确定继续吗？"):
            return

        def task(w, log):
            lines, ok = _bb_send(w.ser, "BB:ERASE",
                                 ("BB:ERASE START", "BB:ERASING", "BB:ERR"), 8.0)
            for ln in lines:
                log("  " + ln)
            return ok

        self._submit(task, "bberase", self._after_bb_erase, "下发擦除…")

    def _after_bb_erase(self, ok) -> None:
        if not ok:
            self._set_status("擦除被拒绝（解锁状态？）")
            return
        self._erase_base = int(self.bb_info.get("eseq", 0) or 0)
        self._erase_t0 = time.time()
        self._erase_stop = False
        self.btn_bb_erase_stop.configure(state="normal")
        self._set_status("擦除中…（每 3 秒查一次状态，可点“停止等待”）")
        self.root.after(3000, self._erase_tick)

    def on_bb_erase_stop(self) -> None:
        self._erase_stop = True
        self.btn_bb_erase_stop.configure(state="disabled")

    def _erase_tick(self) -> None:
        if self.closing:
            return
        if self._erase_stop or not self.connected:
            self._erase_stop = False
            self.btn_bb_erase_stop.configure(state="disabled")
            self._set_status("已停止等待（擦除可能仍在飞控后台进行，点“刷新状态”可查看）")
            return

        def task(w, log):
            from bb_serial import parse_stat_line
            lines, ok = _bb_send(w.ser, "BB:STAT", ("BB:OK", "BB:ERR"), 8.0)
            st = {}
            for ln in lines:
                if ln.startswith("BB:STAT"):
                    st = parse_stat_line(ln)
            return st, ok

        self._submit(task, "erasetick", self._after_erase_tick, "等待擦除完成…")

    def _after_erase_tick(self, payload) -> None:
        st, _ok = payload
        if st:
            self.bb_info = st
        el = time.time() - self._erase_t0
        eseq = int(st.get("eseq", 0) or 0) if st else 0
        erasing = int(st.get("erasing", 0) or 0) if st else 0

        if st and eseq > self._erase_base and not erasing:
            self.btn_bb_erase_stop.configure(state="disabled")
            self._set_status(f"擦除完成，用时 {el:.0f}s")
            self.on_bb_status()
            return
        if el > 320.0:
            self.btn_bb_erase_stop.configure(state="disabled")
            self._set_status("等了 320s 还没擦完，已停止轮询（点“刷新状态”可继续看）")
            return
        self._set_status(f"擦除中… {el:.0f}s（eseq={eseq}），可点“停止等待”")
        self.root.after(3000, self._erase_tick)

    def on_bb_browse(self) -> None:
        path = filedialog.asksaveasfilename(
            title="倾倒目标文件", defaultextension=".bin",
            initialdir=os.path.dirname(self.bb_path_var.get()) or ".",
            filetypes=[("Flash 镜像", "*.bin"), ("所有文件", "*.*")],
            initialfile=os.path.basename(self.bb_path_var.get()) or "flash.bin")
        if path:
            self.bb_path_var.set(path)

    def on_bb_dump(self) -> None:
        if not self._bb_require():
            return
        path = self.bb_path_var.get().strip()
        if not path:
            self._set_status("先填输出文件路径")
            return
        mode = self.bb_mode.get()
        want = 1
        if mode == "run":
            try:
                want = max(1, min(20, int(self.bb_runs_var.get())))
            except (TypeError, ValueError):
                want = 1
                self.bb_runs_var.set("1")

        self.bb_pb["value"] = 0
        self.bb_prog_var.set("准备中…")

        def task(w, log):
            if mode == "run":
                img, info = self._dump_newest_run(w, log, want)
            else:
                img, info = self._dump_all(w, log)
            with open(path, "wb") as fp:
                fp.write(img)
            log(f"  [已写入 {path}：{len(img) / 1048576.0:.1f} MB 镜像]")
            return info

        txt = (f"倾倒最近 {want} 段飞行…" if mode == "run" else "倾倒全部已记录数据…")
        self._submit(task, "bbdump", self._after_bb_dump, txt)

    # ---- 倾倒实现（都在 worker 线程里跑）----
    def _scan_headers(self, w, log) -> list[dict]:
        lines, _ok = _bb_send(w.ser, "BB:SCAN", ("BB:SCANDONE", "BB:ERR"), 25.0)
        hdrs = []
        for ln in lines:
            m = re.match(r"^BB:HDR\s+idx=(\d+)\s+phys=(\d+)\s+off=(\d+)", ln)
            if m:
                hdrs.append({"idx": int(m.group(1)),
                             "phys": int(m.group(2)),
                             "off": int(m.group(3))})
        hdrs.sort(key=lambda h: h["idx"])        # 逻辑序号 = 写入顺序
        return hdrs

    def _fetch(self, w, log, phys_list, pieces: dict, total_bytes: int) -> None:
        """按物理连续段把扇区拉回来，塞进 pieces。"""
        for start, cnt in _contig_runs(phys_list):
            log(f"    倾倒物理扇区 {start}..{start + cnt - 1}（{cnt} 个 ≈ {cnt * PER_SECTOR / 1024:.0f} KB）")
            raw = _bb_dump_range(w.ser, start, cnt)
            for k in range(cnt):
                pieces[start + k] = raw[k * PER_SECTOR:(k + 1) * PER_SECTOR]
            w.out.put(("progress", "bbdump",
                       (len(pieces) * PER_SECTOR, max(total_bytes, 1)), ""))

    def _fetch_one(self, w, phys: int, pieces: dict) -> bytes | None:
        if phys in pieces:
            return pieces[phys]
        raw = _bb_dump_range(w.ser, phys, 1)
        blob = raw[:PER_SECTOR]
        pieces[phys] = blob
        return blob

    def _count_frames(self, pieces: dict) -> int:
        n = 0
        for blob in pieces.values():
            for i in range(SECTOR_DATA_LEN // FRAME_SIZE):
                off = DUMP_HDR_LEN + i * FRAME_SIZE
                if off + FRAME_SIZE <= len(blob) and \
                        struct.unpack_from("<H", blob, off)[0] == FRAME_MAGIC:
                    n += 1
        return n

    def _dump_newest_run(self, w, log, want_runs: int = 1):
        """只取最近 N 段飞行：从最新扇区往回分块倒，数到 N 个"起点"就停。

        判据：固件在每次解锁（arm 上升沿）会切到一个**新扇区**并把帧序号归零，
        所以某段飞行的起点扇区里必然存在一个 seq 在 1..8 之间的**数据帧**。
        注意排除序号回绕（16 位序号每 65536 帧归零一次，约 65 秒）：
        回绕时前一个扇区的末序号会接近 65535，那种不算新的一段。
        """
        hdrs = self._scan_headers(w, log)
        if not hdrs:
            raise RuntimeError("没有扫描到扇区索引头 —— 还没录过飞行？")
        total_bytes = len(hdrs) * PER_SECTOR
        log(f"  扇区索引 {len(hdrs)} 个（{total_bytes / 1048576.0:.2f} MB），"
            f"最新写到物理扇区 {hdrs[-1]['phys']}；从新往旧找飞行起点…")

        pieces: dict[int, bytes] = {}
        starts: list[int] = []          # 起点位置（新→旧）
        cursor = len(hdrs) - 1

        while cursor >= 0 and len(starts) < want_runs:
            take = min(DUMP_CHUNK, cursor + 1)
            group_start = cursor - take + 1
            group = hdrs[group_start: cursor + 1]
            self._fetch(w, log, [h["phys"] for h in group], pieces, total_bytes)

            for j in range(len(group) - 1, -1, -1):
                if len(starts) >= want_runs:
                    break
                phys = group[j]["phys"]
                blob = pieces.get(phys)
                if not blob:
                    continue
                seq = _sector_run_start(blob)
                if seq is None:
                    continue

                # 找到"疑似起点"，先用前一个扇区的末序号排除序号回绕
                pos = group_start + j
                prev_seq = None
                if pos > 0:
                    prev_blob = self._fetch_one(w, hdrs[pos - 1]["phys"], pieces)
                    prev_seq = _sector_last_seq(prev_blob) if prev_blob else None

                if prev_seq is not None and prev_seq >= 60000:
                    log(f"    扇区 {phys} 的 seq 归零是**序号回绕**"
                        f"（上一扇区末 seq={prev_seq}），不是新的一段，跳过")
                    continue

                starts.append(pos)
                log(f"  找到第 {len(starts)} 段起点：物理扇区 {phys}（该扇区有 seq={seq} 的数据帧）")

            if group_start == 0:
                break
            cursor = group_start - 1

        if starts:
            run_idx = min(starts)                    # 最老的那个 = 包含最近 N 段
            if len(starts) < want_runs:
                log(f"  只找到 {len(starts)} 段（要 {want_runs} 段），从最早那段开始")
        else:
            # 一路倒到头都没有起点：更早的数据被环形覆盖了，从最早的有数据处开始
            run_idx = None
            for j, h in enumerate(hdrs):
                blob = pieces.get(h["phys"])
                if blob is None:
                    blob = self._fetch_one(w, h["phys"], pieces)
                if blob and _sector_last_seq(blob) is not None:
                    run_idx = j
                    break
            log("  已到最早的数据，从这里开始（更早的被环形覆盖了）")
            if run_idx is None:
                raise RuntimeError("没找到可用的数据段（都是空扇区？）")

        # 起点可能落在更早的块里，那些扇区还没倒回来 —— 缺的就补上
        need = [h["phys"] for h in hdrs[run_idx:] if h["phys"] not in pieces]
        if need:
            log(f"  补取起点之前的 {len(need)} 个扇区…")
            self._fetch(w, log, need, pieces, total_bytes)

        used = hdrs[run_idx:]
        keep = {h["phys"]: pieces[h["phys"]] for h in used if h["phys"] in pieces}
        log(f"  本段 {len(keep)} 个扇区（全部数据 {len(hdrs)} 个），拼装镜像…")
        img = _assemble_image(keep)
        frames = self._count_frames(keep)
        return img, {"sectors": len(keep), "frames": frames, "total_sectors": len(hdrs),
                     "first_phys": used[0]["phys"], "last_phys": used[-1]["phys"],
                     "runs": len(starts), "mode": "run"}

    def _dump_all(self, w, log):
        """全部已记录数据：从 0 号扇区一直取到最后一个有数据的扇区。"""
        hdrs = self._scan_headers(w, log)
        if not hdrs:
            raise RuntimeError("没有扫描到扇区索引头 —— 还没录过飞行？")
        last = max(h["phys"] for h in hdrs)
        total_bytes = (last + 1) * PER_SECTOR
        log(f"  扇区索引 {len(hdrs)} 个，最后一个有数据的物理扇区 {last}")

        pieces: dict[int, bytes] = {}
        step = 512
        for start in range(0, last + 1, step):
            cnt = min(step, last + 1 - start)
            self._fetch(w, log, list(range(start, start + cnt)), pieces, total_bytes)

        log(f"  共 {len(pieces)} 个扇区，拼装镜像…")
        img = _assemble_image(pieces)
        frames = self._count_frames(pieces)
        return img, {"sectors": len(pieces), "frames": frames,
                     "total_sectors": len(pieces),
                     "first_phys": 0, "last_phys": last, "mode": "all"}

    def _after_bb_dump(self, info) -> None:
        self.bb_pb["value"] = 100
        n = info.get("sectors", 0)
        total = info.get("total_sectors", n)
        frames = info.get("frames", 0)
        dur = frames / 1000.0
        self.bb_prog_var.set(
            f"完成：{n}/{total} 个扇区 / {frames} 帧"
            + (f"（约 {dur:.1f} 秒）" if frames else ""))

        if info.get("mode") == "run" and n >= total:
            tail = "整片只有这一段飞行（或更早的被覆盖），所以等于全部"
        elif info.get("mode") == "run":
            nruns = info.get("runs") or 1
            tail = (f"只取了最近 {nruns} 段" if nruns > 1 else "只取了最新一段")
            tail += f"（全部数据 {total} 个扇区）"
        else:
            tail = "全部已记录数据"
        self._set_status(
            f"已保存 {n} 个扇区（物理 {info.get('first_phys')}..{info.get('last_phys')}）"
            f"到 {self.bb_path_var.get()} —— {tail}")

    def on_bb_analyze(self) -> None:
        path = self.bb_path_var.get().strip()
        if not path or not os.path.isfile(path):
            self._set_status(f"文件不存在：{path}（先倾倒，或手动选一个 .bin）")
            return
        do_plot = self.bb_plot.get()
        trim_args = self._trim_args()          # 只能在 GUI 线程里读 Tk 变量

        def task(w, log):
            import contextlib
            import io

            import bb_analyze
            argv = [path]
            base = None
            if do_plot:
                base = os.path.splitext(path)[0] + "_bb.png"
                argv += ["--plot", "--save-plot", base]
            if trim_args:
                argv += trim_args
                log(f"  [trim 归零] {' '.join(trim_args)}")
            buf = io.StringIO()
            # bb_analyze 是"边算边打印"的老式实现，这里把它的 stdout 收进来显示。
            # 只有 worker 一个线程在干活，重定向不会抢到别人的输出。
            with contextlib.redirect_stdout(buf):
                rc = bb_analyze.main(argv)
            return rc, buf.getvalue(), base

        self._submit(task, "bbanalyze", self._after_bb_analyze,
                     "分析中…（16MB 可能要几十秒）")

    def _after_bb_analyze(self, payload) -> None:
        rc, text, base = payload
        self.bb_report.configure(state="normal")
        self.bb_report.delete("1.0", "end")
        self.bb_report.insert("1.0", text or "(没有输出)")
        self.bb_report.see("1.0")
        self.bb_report.configure(state="disabled")

        if base and os.path.isfile(base):
            self._bb_plot_path = base
            self.btn_bb_openplot.configure(state="normal")
        else:
            self._bb_plot_path = None
            self.btn_bb_openplot.configure(state="disabled")
        self._set_status("分析完成" if rc == 0 else f"分析返回 {rc}，看报告里的提示")

    def on_bb_csv(self) -> None:
        """把倾倒下来的数据导出成 CSV（逐帧一行，方便丢进 Excel/Matlab）。"""
        path = self.bb_path_var.get().strip()
        if not path or not os.path.isfile(path):
            self._set_status(f"文件不存在：{path}（先倾倒，或手动选一个 .bin）")
            return

        base = os.path.splitext(path)[0]
        csv_path = filedialog.asksaveasfilename(
            title="导出 CSV（连续数据段逐帧）", defaultextension=".csv",
            initialdir=os.path.dirname(path) or ".",
            initialfile=os.path.basename(base) + ".csv",
            filetypes=[("CSV", "*.csv"), ("所有文件", "*.*")])
        if not csv_path:
            return
        trim_args = self._trim_args()

        def task(w, log):
            import contextlib
            import io

            import bb_analyze
            argv = [path, "--csv", csv_path] + trim_args
            if trim_args:
                log(f"  [trim 归零] {' '.join(trim_args)}（CSV 里的角度也一起减去）")
            buf = io.StringIO()
            # bb_analyze 的 --csv 走的是同一套解析，这里把它的输出收进日志页
            with contextlib.redirect_stdout(buf):
                rc = bb_analyze.main(argv)
            return rc, buf.getvalue(), csv_path

        self._submit(task, "bbcsv", self._after_bb_csv, "导出 CSV…")

    def _after_bb_csv(self, payload) -> None:
        rc, text, csv_path = payload
        if text:
            for ln in text.splitlines():
                self._drain_log("  " + ln)
        if os.path.isfile(csv_path):
            size = os.path.getsize(csv_path)
            rows = 0
            try:
                with open(csv_path, "r", encoding="utf-8", errors="replace") as fp:
                    rows = max(0, sum(1 for _ in fp) - 1)
            except OSError:
                pass
            self._set_status(f"已导出 CSV：{csv_path}（{rows} 行 / {size / 1024:.0f} KB）")
        else:
            self._set_status(f"CSV 没有生成（返回 {rc}）：文件里可能没有飞行数据，看日志页")

    def on_bb_open_plot(self) -> None:
        if not self._bb_plot_path:
            return
        for p in (self._bb_plot_path,
                  self._bb_plot_path.replace(".png", "_step.png")):
            if os.path.isfile(p):
                try:
                    if sys.platform.startswith("win"):
                        os.startfile(p)                    # type: ignore[attr-defined]
                    else:
                        import subprocess
                        subprocess.Popen(["xdg-open", p])
                except Exception as exc:                   # noqa: BLE001
                    self._set_status(f"打不开图片：{exc}")

    # ---------------- 基础设施 ----------------
    def _set_busy(self, busy: bool, text: str = "") -> None:
        self.busy = max(0, self.busy + (1 if busy else -1))
        self.bottom.configure(text=text if self.busy else
                              ("就绪" if self.connected else "未连接"))
        self._update_actions()

    def _set_progress(self, pair) -> None:
        """倾倒进度（worker 每 ~1 秒发一次）。"""
        done, total = pair
        pct = 100.0 * done / max(1, total)
        self.bb_pb["value"] = pct
        self.bb_prog_var.set(f"{done / 1048576.0:.2f} / {total / 1048576.0:.2f} MB"
                             f"  ({pct:.1f}%)")

    def _update_actions(self) -> None:
        idle = (not self.busy) and self.connected
        idle_any = not self.busy
        for w in (self.btn_refresh, self.btn_load, self.btn_def, self.btn_apply,
                  self.btn_save, self.btn_reboot, self.btn_hist_load,
                  self.btn_hist_roll, self.btn_hist_reboot, self.btn_erase,
                  self.btn_preset_apply,
                  self.btn_bb_status, self.btn_bb_scan, self.btn_bb_cal,
                  self.btn_bb_erase, self.btn_bb_dump, self.btn_cal_run):
            try:
                w.configure(state="normal" if idle else "disabled")
            except Exception:                              # noqa: BLE001
                pass
        # 离线分析/导出只读本地文件，不占串口 —— 没连接也能用
        for w in (self.btn_bb_analyze, self.btn_bb_csv):
            try:
                w.configure(state="normal" if idle_any else "disabled")
            except Exception:                              # noqa: BLE001
                pass
        self.btn_conn.configure(state="normal" if idle_any else "disabled")
        self.btn_scan.configure(state="normal" if idle_any else "disabled")

        dirty = bool(self.info.get("dirty"))
        self.dirty_label.configure(
            text="⚠ 有改动还没重启（先保存再重启）" if dirty else "")

    def _log(self, text: str) -> None:
        self.worker.out.put(("log", "", text, ""))

    def _drain_log(self, text: str) -> None:
        self.log.configure(state="normal")
        self.log.insert("end", text.rstrip("\n") + "\n")
        # 控制日志长度，避免长时间运行后卡顿
        if int(self.log.index("end-1c").split(".")[0]) > 4000:
            self.log.delete("1.0", "1000.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def _clear_log(self) -> None:
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def _submit(self, fn, tag: str, on_ok, busy_text: str = "通讯中…") -> None:
        if self.busy:
            return                      # 串口独占：忙时直接丢弃，不排队堆积
        self.callbacks[tag] = on_ok
        self._set_busy(True, busy_text)
        self.worker.submit(fn, tag)

    def _tick_poll(self) -> None:
        """GUI 线程轮询 worker 的结果队列。"""
        try:
            for _ in range(200):
                kind, tag, payload, tb = self.worker.out.get_nowait()
                if kind == "progress":
                    self._set_progress(payload)
                    continue
                if kind == "log":
                    self._drain_log(payload)
                    continue
                cb = self.callbacks.pop(tag, None)
                self._set_busy(False)
                if kind == "ok":
                    if cb:
                        cb(payload)
                else:
                    self._drain_log(f"[!] {tag} 出错：{payload}")
                    for ln in (tb or "").splitlines()[-6:]:
                        self._drain_log("    " + ln)
                    self._set_status(f"出错：{payload}")
        except queue.Empty:
            pass
        if not self.closing:
            self.root.after(POLL_MS, self._tick_poll)

    def _set_status(self, text: str) -> None:
        self.status_var.set(text)

    # ---------------- 连接 ----------------
    def on_scan(self) -> None:
        from bb_serial import available_ports, pick_default_port, HAVE_SERIAL
        if not HAVE_SERIAL:
            self.port_combo["values"] = []
            self._set_status("未安装 pyserial：pip install pyserial")
            return
        ports = available_ports()
        self.port_combo["values"] = ports
        # 当前选的口不在了（或者还没选）就重新挑：优先非 COM1 的那个
        if (self.port_var.get() not in ports) or not self.port_var.get():
            self.port_var.set(pick_default_port(ports))
        if ports:
            self._set_status(f"检测到 {len(ports)} 个串口：{', '.join(ports)}"
                             f"（默认不用 COM1）")
        else:
            self._set_status("没有检测到串口")

    def on_connect(self) -> None:
        if self.connected:
            self.on_disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self._set_status("先选一个串口（点“扫描串口”）")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            self._set_status("波特率不是数字")
            return

        def task(w, log):
            from bb_serial import open_port
            if w.ser is not None:
                try:
                    w.ser.close()
                except Exception:                          # noqa: BLE001
                    pass
                w.ser = None
            w.port, w.baud = port, baud
            w.ser = open_port(port, baud=baud, quiet=True)
            log(f"[已连接 {port} @{baud}]")
            info, _ = bb_cfg.cfg_info(w.ser)
            if not info:
                raise RuntimeError("连上了，但 CFG:INFO 没有回包 —— "
                                   "固件可能是 P2 之前的版本？")
            params, _ok = bb_cfg.cfg_list(w.ser)
            log(f"[参数 {len(params)} 项]")
            return info, params

        self._submit(task, "connect", self._after_connect, "连接中…")

    def _after_connect(self, payload) -> None:
        info, params = payload
        self.connected = True
        self.conn_label.configure(text="● 已连接", foreground="#070")
        self.btn_conn.configure(text="断开")
        self._apply_info(info)
        self.params = params
        self._sync_trim_from_params()
        self._fill_tree()
        self.on_hist_refresh()
        self._set_status("连接成功")

    def on_disconnect(self) -> None:
        def task(w, log):
            if w.ser is not None:
                try:
                    w.ser.close()
                except Exception:                          # noqa: BLE001
                    pass
                w.ser = None
            return True

        self._submit(task, "disconnect", self._after_disconnect, "断开中…")

    def _after_disconnect(self, _r) -> None:
        self.connected = False
        self.params = []
        self.info = {}
        self.tree.delete(*self.tree.get_children())
        self.hist_tree.delete(*self.hist_tree.get_children())
        self.conn_label.configure(text="● 未连接", foreground="#a00")
        self.btn_conn.configure(text="连接")
        self._set_status("已断开")
        self._update_actions()

    # ---------------- 状态 / 参数表 ----------------
    def _apply_info(self, info: dict) -> None:
        self.info = info or {}
        i = self.info

        def _int(key: str) -> int:
            """容错取整：字段可能带单位（up=1234ms），也可能压根没有。"""
            v = i.get(key, 0)
            if isinstance(v, int):
                return v
            if isinstance(v, float):
                return int(v)
            digits = "".join(ch for ch in str(v) if ch.isdigit())
            return int(digits) if digits else 0

        up_s = _int("up") / 1000.0
        self._set_status(
            f"src={i.get('src')}  dirty={i.get('dirty')}  "
            f"seq={i.get('seq')}  used={i.get('used')}  bank={i.get('bank')}  "
            f"写入 {_int('wr_us')}us  擦除 {_int('er_us') / 1000:.0f}ms  "
            f"开机 {up_s:.1f}s")
        self._update_actions()

    def _fill_tree(self) -> None:
        pat = self.filter_var.get().strip().lower()
        only_dirty = self.only_dirty.get()
        sel = self._selected_name()

        self.tree.delete(*self.tree.get_children())
        for p in self.params:
            if pat and pat not in p["name"].lower():
                continue
            if only_dirty and not p["dirty"]:
                continue
            state = "* 待重启" if p["dirty"] else "已生效"
            tags = ("dirty",) if p["dirty"] else ()
            self.tree.insert("", "end", iid=p["name"],
                             values=(f"0x{p['id']:04X}", p["name"],
                                     _fmt(p["value"]), state),
                             tags=tags)
        if sel and self.tree.exists(sel):
            self.tree.selection_set(sel)
            self.tree.see(sel)
        self._update_cal_view()

    def _selected_name(self) -> str | None:
        s = self.tree.selection()
        return s[0] if s else None

    def _on_select(self, _evt=None) -> None:
        name = self._selected_name()
        if not name:
            self.sel_var.set("（未选中）")
            return
        p = next((x for x in self.params if x["name"] == name), None)
        if p is None:
            return
        self.sel_var.set(f"0x{p['id']:04X}  {p['name']}")
        self.value_var.set(bb_cfg.fmt_value(p["value"]))
        self.value_entry.focus_set()
        self.value_entry.selection_range(0, "end")

    def on_refresh(self) -> None:
        if not self.connected:
            return

        def task(w, log):
            info, _ = bb_cfg.cfg_info(w.ser)
            params, _ok = bb_cfg.cfg_list(w.ser)
            return info, params

        self._submit(task, "refresh", self._after_refresh, "读取参数…")

    def _after_refresh(self, payload) -> None:
        info, params = payload
        self._apply_info(info)
        self.params = params
        self._sync_trim_from_params()
        self._fill_tree()
        self._set_status(self.status_var.get())

    def on_apply_value(self) -> None:
        name = self._selected_name()
        if not name:
            self._set_status("先在表里选一个参数")
            return
        sval = self.value_var.get().strip()
        try:
            value = float(sval)
        except ValueError:
            self._set_status(f"“{sval}”不是数字（例：12 / -0.3 / 0.008）")
            return

        def task(w, log):
            ok, line, _lines = bb_cfg.cfg_set(w.ser, name, value)
            log(("  " if ok else "  ") + bb_cfg.explain(line))
            info, _ = bb_cfg.cfg_info(w.ser)
            params, _ = bb_cfg.cfg_list(w.ser)
            return ok, line, info, params

        self._submit(task, "set", self._after_set, f"写入 {name}={sval} …")

    def _after_set(self, payload) -> None:
        ok, line, info, params = payload
        self._apply_info(info)
        self.params = params
        self._sync_trim_from_params()
        self._fill_tree()
        if not ok:
            self._set_status("设置失败：" + line)
            return
        if "clamped" in line:
            self._set_status("已写入，但数值超出范围被限制（看日志里的原值）")
        else:
            self._set_status("已写入待生效表 —— 记得 SAVE + 重启")
        self._on_select()

    # ---------------- 生效动作 ----------------
    def on_save(self) -> None:
        if not self.connected:
            return

        def task(w, log):
            ok, line = bb_cfg.cfg_save(w.ser)
            log("  " + bb_cfg.explain(line))
            info, _ = bb_cfg.cfg_info(w.ser)
            return ok, line, info

        self._submit(task, "save", self._after_save, "写 Flash…")

    def _after_save(self, payload) -> None:
        ok, line, info = payload
        self._apply_info(info)
        if ok:
            self._set_status("已落盘 —— 要生效还需要重启（Ctrl+R 或重启按钮）")
        else:
            self._set_status("保存失败：" + line)

    def on_reboot(self) -> None:
        if not self.connected:
            return
        if not _ask(self.root, "确认重启？", "飞控会软复位。\n\n"
                    "未保存的改动会丢弃；保存过的改动会在这时生效。\n"
                    "（解锁状态下会被固件拒绝）"):
            return

        def task(w, log):
            ok, lines = bb_cfg.cfg_simple(w.ser, "CFG:REBOOT", "reboot")
            for ln in lines:
                log("  " + ln)
            if not ok:
                return False, lines, None
            time.sleep(3.0)                     # 等它重启完
            try:
                w.ser.close()
            except Exception:                   # noqa: BLE001
                pass
            from bb_serial import open_port
            w.ser = open_port(w.port, baud=w.baud, quiet=True)
            log("[已重新连接]")
            info, _ = bb_cfg.cfg_info(w.ser)
            params, _ = bb_cfg.cfg_list(w.ser)
            return True, lines, (info, params)

        self._submit(task, "reboot", self._after_reboot, "重启并重连…")

    def _after_reboot(self, payload) -> None:
        ok, lines, extra = payload
        if not ok or not extra:
            self._set_status("重启被拒绝（解锁状态？）或重连失败，试试点“连接”")
            self.connected = False
            self.conn_label.configure(text="● 已断开", foreground="#a00")
            self.btn_conn.configure(text="连接")
            return
        info, params = extra
        self._apply_info(info)
        self.params = params
        self._sync_trim_from_params()
        self._fill_tree()
        self._set_status("已重启并重连，参数已生效")

    def on_load(self) -> None:
        def task(w, log):
            ok, lines = bb_cfg.cfg_simple(w.ser, "CFG:LOAD", "load")
            params, _ = bb_cfg.cfg_list(w.ser)
            info, _ = bb_cfg.cfg_info(w.ser)
            return ok, lines, params, info

        self._submit(task, "load", self._after_load, "重新载入…")

    def _after_load(self, payload) -> None:
        ok, lines, params, info = payload
        self._apply_info(info)
        self.params = params
        self._sync_trim_from_params()
        self._fill_tree()
        self._set_status("已从 Flash 重新载入（未保存的改动已丢弃）" if ok else "载入失败")

    def on_def(self) -> None:
        if not _ask(self.root, "恢复默认？",
                    "把 68 项全部改回编译期默认值（只改内存，不写 Flash）。\n\n"
                    "要保留请接着点“保存到 Flash”。"):
            return

        def task(w, log):
            ok, lines = bb_cfg.cfg_simple(w.ser, "CFG:DEF", "def")
            params, _ = bb_cfg.cfg_list(w.ser)
            info, _ = bb_cfg.cfg_info(w.ser)
            return ok, lines, params, info

        self._submit(task, "def", self._after_load, "恢复默认…")

    # ---------------- 历史 ----------------
    def on_hist_refresh(self) -> None:
        if not self.connected:
            return

        def task(w, log):
            lines, tail, _ok = bb_cfg.cfg_hist(w.ser, HIST_ROWS)
            return [ln for ln in lines], tail

        self._submit(task, "hist", self._after_hist, "读取历史…")

    def _after_hist(self, payload) -> None:
        lines, tail = payload
        self.hist_tree.delete(*self.hist_tree.get_children())
        for ln in lines:
            h = _parse_hist_line(ln)
            if not h:
                continue
            self.hist_tree.insert("", "end", iid=str(h["idx"]),
                                  values=(h["idx"], h["seq"], h["slot"], h["detail"]))
        depth = tail.get("depth", "?") if tail else "?"
        self._set_status(f"历史：可回滚 {depth} 条（当前显示 {len(lines)} 条）")

    def on_hist_restore(self, save: bool) -> None:
        sel = self.hist_tree.selection()
        if not sel:
            self._set_status("先在历史列表里选一条")
            return
        idx = int(sel[0])
        if save:
            if not _ask(self.root, "确认回滚并保存？",
                        f"把第 {idx} 条历史（0=最新）写回待生效表，并且立刻存盘。\n\n"
                        "回滚后需要重启才生效。被回滚掉的版本仍留在历史里。"):
                return
        cmd = f"CFG:ROLLBACK:{idx}" if save else f"CFG:LOAD:{idx}"
        action = "rollback" if save else "loadhist"

        def task(w, log):
            ok, lines = bb_cfg.cfg_simple(w.ser, cmd, action)
            for ln in lines:
                log("  " + bb_cfg.explain(ln))
            params, _ = bb_cfg.cfg_list(w.ser)
            info, _ = bb_cfg.cfg_info(w.ser)
            return ok, params, info

        text = "回滚并保存…" if save else "加载历史…"
        self._submit(task, "histrestore", self._after_hist_restore, text)

    def _after_hist_restore(self, payload) -> None:
        ok, params, info = payload
        self._apply_info(info)
        self.params = params
        self._sync_trim_from_params()
        self._fill_tree()
        self.on_hist_refresh()
        self._set_status("已回滚到内存，重启后生效" if ok else "回滚失败（看日志）")

    def on_erase(self) -> None:
        if not _ask(self.root, "擦除配置区？",
                    "会擦掉内部 Flash 里的两个 bank（约 2 秒，整机停顿）。\n"
                    "历史全部消失；内存里的参数不变，但断电后回到默认值。\n\n"
                    "确定要继续吗？"):
            return

        def task(w, log):
            ok, lines = bb_cfg.cfg_simple(w.ser, "CFG:ERASE", "erase")
            for ln in lines:
                log("  " + bb_cfg.explain(ln))
            info, _ = bb_cfg.cfg_info(w.ser)
            params, _ = bb_cfg.cfg_list(w.ser)
            return ok, params, info

        self._submit(task, "erase", self._after_erase, "擦除中（约 2 秒）…")

    def _after_erase(self, payload) -> None:
        ok, params, info = payload
        self._apply_info(info)
        self.params = params
        self._sync_trim_from_params()
        self._fill_tree()
        self.hist_tree.delete(*self.hist_tree.get_children())
        self._set_status("配置区已擦除（内存里的参数还在，断电即回默认）" if ok else "擦除失败")

    # ---------------- 预设 ----------------
    def on_preset_refresh(self) -> None:
        self.preset_list.delete(0, "end")
        for f in _list_presets():
            self.preset_list.insert("end", f)

    def on_open_presets(self) -> None:
        try:
            if sys.platform.startswith("win"):
                os.startfile(PRESET_DIR)                  # type: ignore[attr-defined]
            else:
                import subprocess
                subprocess.Popen(["xdg-open", PRESET_DIR])
        except Exception as exc:                          # noqa: BLE001
            self._set_status(f"打不开目录：{exc}")

    def _selected_preset(self) -> str | None:
        sel = self.preset_list.curselection()
        if not sel:
            self._set_status("先在左边选一个预设文件")
            return None
        return os.path.join(PRESET_DIR, self.preset_list.get(sel[0]))

    def on_preset_preview(self) -> None:
        path = self._selected_preset()
        if not path or not self.connected:
            return
        try:
            data = bb_cfg.preset_load(path)
        except SystemExit as exc:
            self._set_status(str(exc))
            return

        def task(w, log):
            cur, _ok = bb_cfg.cfg_list(w.ser)
            return cur

        def done(cur) -> None:
            curmap = {p["name"]: p["value"] for p in cur}
            lines = []
            for k, v in sorted(data["params"].items()):
                old = curmap.get(k)
                if old is None:
                    lines.append(f"  {k}: （飞机上没有这个参数） -> {v}")
                elif abs(old - float(v)) > 1e-6:
                    lines.append(f"  {k}: {old:.3f} -> {float(v):.3f}")
            txt = "\n".join(lines) if lines else "  与飞机当前值完全一致"
            self.preset_diff.configure(state="normal")
            self.preset_diff.delete("1.0", "end")
            self.preset_diff.insert("1.0",
                                    f"{data.get('name', '?')}  ({len(data['params'])} 项)\n"
                                    + (f"备注：{data['note']}\n" if data.get("note") else "")
                                    + f"\n差异 {len(lines)} 项：\n" + txt)
            self.preset_diff.configure(state="disabled")

        self._submit(task, "presetpreview", done, "对比预设…")

    def on_preset_apply(self) -> None:
        path = self._selected_preset()
        if not path or not self.connected:
            return
        try:
            data = bb_cfg.preset_load(path)
        except SystemExit as exc:
            self._set_status(str(exc))
            return
        params = data["params"]
        if not _ask(self.root, "应用预设？",
                    f"把 {len(params)} 项参数写入待生效表。\n\n"
                    + ("写完后会保存到 Flash。\n" if self.preset_save.get() else "")
                    + ("保存后会重启并重连。\n" if self.preset_reboot.get() else "")
                    + "\n任何一项失败会立刻停下并报告。"):
            return

        do_save = self.preset_save.get()
        do_reboot = self.preset_reboot.get()

        def task(w, log):
            fails = []
            for i, (k, v) in enumerate(sorted(params.items())):
                ok, line, _lines = bb_cfg.cfg_set(w.ser, k, float(v))
                if not ok:
                    fails.append((k, v, line))
                    log(f"  [X] {k}={v}: {bb_cfg.explain(line)}")
                    if len(fails) >= 5:
                        break
                elif i % 15 == 0:
                    log(f"  … 已写入 {i + 1}/{len(params)}")
            if fails:
                return {"fails": fails, "saved": False, "rebooted": False}
            log(f"  [OK] {len(params)} 项已写入待生效表")

            saved = rebooted = False
            if do_save:
                ok, line = bb_cfg.cfg_save(w.ser)
                log("  " + bb_cfg.explain(line))
                saved = ok
            if saved and do_reboot:
                okr, lines = bb_cfg.cfg_simple(w.ser, "CFG:REBOOT", "reboot")
                log("  " + (lines[-1] if lines else ""))
                rebooted = okr
                if okr:
                    time.sleep(3.0)
                    try:
                        w.ser.close()
                    except Exception:                    # noqa: BLE001
                        pass
                    from bb_serial import open_port
                    w.ser = open_port(w.port, baud=w.baud, quiet=True)
                    log("[已重新连接]")
            info, _ = bb_cfg.cfg_info(w.ser)
            plist, _ = bb_cfg.cfg_list(w.ser)
            return {"fails": [], "saved": saved, "rebooted": rebooted,
                    "info": info, "params": plist}

        self._submit(task, "presetapply", self._after_preset_apply, "应用预设…")

    def _after_preset_apply(self, r) -> None:
        if r.get("fails"):
            self._set_status(f"{len(r['fails'])} 项失败 —— 内存里是部分应用状态，"
                             "可点“重新载入”丢弃")
            return
        if "info" in r:
            self._apply_info(r["info"])
            self.params = r["params"]
            self._fill_tree()
        if r["saved"] and r["rebooted"]:
            self._set_status("预设已应用、保存并重启，参数已生效")
        elif r["saved"]:
            self._set_status("预设已应用并保存 —— 还需要重启才生效")
        else:
            self._set_status("预设已写入待生效表 —— 还需要 SAVE + 重启")

    def on_preset_export(self) -> None:
        if not self.connected:
            return
        path = filedialog.asksaveasfilename(
            title="导出当前参数为预设",
            initialdir=PRESET_DIR, defaultextension=".json",
            filetypes=[("JSON", "*.json")], initialfile="my_tune.json")
        if not path:
            return

        def task(w, log):
            plist, _ok = bb_cfg.cfg_list(w.ser)
            return plist

        def done(plist) -> None:
            import json
            name = os.path.splitext(os.path.basename(path))[0]
            preset = bb_cfg.preset_build({p["name"]: p["value"] for p in plist},
                                         name=name)
            with open(path, "w", encoding="utf-8") as fp:
                json.dump(preset, fp, ensure_ascii=False, indent=2)
                fp.write("\n")
            self.on_preset_refresh()
            self._set_status(f"已导出 {os.path.basename(path)}（{len(plist)} 项）")

        self._submit(task, "presetexport", done, "导出预设…")

    # ---------------- 关闭 ----------------
    def on_close(self) -> None:
        self.closing = True
        self.worker.stop()
        try:
            if self.worker.ser is not None:
                self.worker.ser.close()
        except Exception:                                  # noqa: BLE001
            pass
        self.root.destroy()


def _ask(root, title: str, msg: str) -> bool:
    from tkinter import messagebox
    return bool(messagebox.askyesno(title, msg, parent=root))


# ==================================================================
#  四、不接硬件的自检
# ==================================================================
def selftest() -> int:
    import bb_serial

    fails = 0

    def check(cond, what, extra=""):
        nonlocal fails
        if cond:
            print(f"[OK] {what}")
        else:
            fails += 1
            print(f"[X ] {what}" + (f"   {extra}" if extra else ""))

    print("=== 1. 常量与固件一致 ===")
    problems = bb_cfg.check_firmware_constants()
    check(not problems, "flash_ee.h / param.h 与 bb_cfg.py 一致", "; ".join(problems))

    print("\n=== 2. 用假串口跑一遍命令构造与解析 ===")

    class FakeSer:
        """按脚本回包的假串口：read(1) 从缓冲区吐字节。"""

        def __init__(self) -> None:
            self.written: list[str] = []
            self.buf = b""
            self.replies: list[list[str]] = []

        def expect(self, *lines: str) -> "FakeSer":
            self.replies.append(list(lines))
            return self

        def _pop_reply(self) -> None:
            if not self.replies:
                self.buf = b""
                return
            self.buf = ("\r\n".join(self.replies.pop(0)) + "\r\n").encode("ascii", "replace")

        def write(self, data) -> None:
            self.written.append(data.decode("ascii", "replace").strip())
            self._pop_reply()

        def flush(self) -> None:
            pass

        def reset_input_buffer(self) -> None:
            pass

        def read(self, n: int = 1):
            if not self.buf:
                return b""
            byte, self.buf = self.buf[:1], self.buf[1:]
            return byte

    def fast_read_line(ser, timeout: float = 3.0) -> str:
        out = bytearray()
        for _ in range(2000):
            b = ser.read(1)
            if not b:
                break
            if b == b"\n":
                break
            if b != b"\r":
                out += b
        return out.decode("ascii", "replace")

    real_read_line = bb_serial.read_line
    bb_serial.read_line = fast_read_line          # monkeypatch：跳过真超时等待
    try:
        ser = FakeSer()
        ser.expect("CFG:INFO n=68 tbl=1 src=ee dirty=0 seq=7 next=8 used=8/128 "
                   "bank=A crc_fail=0 erases=0 wr_us=1700 er_us=953350 "
                   "img=080135DC ovl=0 up=1234ms")
        info, ok = bb_cfg.cfg_info(ser)
        check(ok and info.get("src") == "ee" and info.get("seq") == 7,
              "CFG:INFO 解析", str(info))

        ser.expect("CFG:P 0118 pid.roll_inner.kp 12.000 *",
                   "CFG:LISTEND n=68 dirty=1")
        plist, ok = bb_cfg.cfg_list(ser)
        check(ok and len(plist) == 1 and plist[0]["dirty"] and plist[0]["value"] == 12.0,
              "CFG:LIST 解析 + 脏标记", str(plist))

        ser.expect("CFG:OK 0118 pid.roll_inner.kp=13.000 (save+reboot)")
        ok, line, _ = bb_cfg.cfg_set(ser, "pid.roll_inner.kp", 13.0)
        check(ok and ser.written[-1] == "CFG:SET pid.roll_inner.kp 13",
              "CFG:SET 命令写法", ser.written[-1])

        ser.expect("CFG:P 0118 pid.roll_inner.kp 13.000 applied=12.000")
        p, _lines, _ok = bb_cfg.cfg_get(ser, "0x0118")
        check(p is not None and p["applied"] == 12.0,
              "CFG:GET 解析 applied", str(p))

        ser.expect("CFG:SAVED seq=8 slot=8 dirty=0 wr_us=1700")
        ok, line = bb_cfg.cfg_save(ser)
        check(ok and "SAVED" in line, "CFG:SAVE 解析", line)

        ser.expect("CFG:H 0 seq=8 slot=8 same",
                   "CFG:H 1 seq=7 slot=7 diff: pid.roll_inner.kp 12.000->13.000",
                   "CFG:HEND n=2 depth=8")
        lines, tail, ok = bb_cfg.cfg_hist(ser, 5)
        check(ok and len(lines) == 2 and tail.get("depth") == 8,
              "CFG:HIST 解析", f"{len(lines)} / {tail}")

        h = _parse_hist_line("CFG:H 1 seq=7 slot=7 diff: pid.roll_inner.kp 12.000->13.000")
        check(h is not None and h["idx"] == 1 and "12.000->13.000" in h["detail"],
              "历史行拆分", str(h))

        ser.expect("CFG:ERR armed")
        ok, lines = bb_cfg.cfg_simple(ser, "CFG:SAVE", "save")
        check(not ok, "CFG:ERR 能被识别为失败")

        check("解锁" in bb_cfg.explain("CFG:ERR armed"),
              "错误码翻译成人话")
    finally:
        bb_serial.read_line = real_read_line

    print("\n=== 3. 预设文件 ===")
    presets = _list_presets()
    # 早期叫 base_default.json，后来改名为 Default.json；两者认一个
    check(("base_default.json" in presets) or ("Default.json" in presets),
          "presets/ 里有默认预设", str(presets))
    if presets:
        data = bb_cfg.preset_load(os.path.join(PRESET_DIR, presets[0]))
        check("params" in data and data["params"], "预设文件能读", presets[0])

    print()
    if fails == 0:
        print("全部通过。")
        return 0
    print(f"自检失败 {fails} 项。")
    return 1


# ==================================================================
#  五、入口
# ==================================================================
def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    if "--selftest" in argv:
        return selftest()

    # 可选：启动时就选好串口（bb_gui.bat --port COM8）
    port = None
    if "--port" in argv:
        idx = argv.index("--port")
        if idx + 1 >= len(argv):
            print("--port 后面要跟串口号，例如：--port COM8")
            return 2
        port = argv[idx + 1]

    try:                                   # 高 DPI 下字体不至于太小
        from ctypes import windll
        windll.shcore.SetProcessDpiAwareness(1)
    except Exception:                      # noqa: BLE001
        pass

    try:
        root = tk.Tk()
    except tk.TclError as exc:
        print(f"[X] 打不开窗口：{exc}")
        return 1
    style = ttk.Style()
    try:
        style.theme_use("vista" if sys.platform.startswith("win") else "clam")
    except Exception:                      # noqa: BLE001
        pass
    app = App(root)
    if port:
        app.port_var.set(port)
    root.mainloop()
    return 0


def _crash_log() -> None:
    """pythonw 启动时没有控制台，未捕获异常会静默消失 —— 落日志 + 弹窗。"""
    tb = traceback.format_exc()
    log = os.path.join(os.path.dirname(os.path.abspath(__file__)), "gui_error.log")
    try:
        with open(log, "a", encoding="utf-8") as fp:
            fp.write(f"\n===== {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n{tb}\n")
    except OSError:
        log = "(写日志失败)"
    try:
        r = tk.Tk()
        r.withdraw()
        messagebox.showerror("GUI 启动失败",
                             f"详细堆栈已写入：\n{log}\n\n{tb[-700:]}")
    except Exception:                      # noqa: BLE001
        print(tb)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except BaseException:                  # noqa: BLE001
        _crash_log()
        raise
