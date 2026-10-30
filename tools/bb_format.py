"""W25Q128 黑匣子 —— 二进制格式定义与解析核心。

本文件是**固件与上位机的唯一格式契约**。下面每个字段的偏移/类型/缩放都必须与
Core/Inc/blackbox.h 里的 C 结构体严格一致，改动任意一侧都要同步另一侧。

固件侧结构体（实测偏移，ARM ABI 会把总长向上取整到 4 的倍数）：

    typedef struct {                     /* 共 32 字节 */
        uint16_t magic;                  /*  0  0x4246 ('BF')            */
        uint8_t  type;                   /*  2  1=数据帧 2=诊断帧         */
        uint8_t  flags;                  /*  3  bit0 高度有效, bit1 定高  */
        uint16_t seq;                    /*  4  数据帧: 1kHz 序号(无符号!) */
                                         /*     诊断帧: 低字节 conf 档位,  */
                                         /*             高字节 period/100us */
        int16_t  roll;                   /*  6  x100   deg               */
        int16_t  pitch;                  /*  8  x100   deg               */
        int16_t  yaw;                    /* 10  x100   deg               */
        int16_t  gyro_x;                 /* 12  x10    deg/s             */
        int16_t  gyro_y;                 /* 14  x10    deg/s             */
        int16_t  gyro_z;                 /* 16  x10    deg/s             */
        int16_t  accel_x;                /* 18  x1000  g                 */
        int16_t  accel_y;                /* 20  x1000  g                 */
        int16_t  accel_z;                /* 22  x1000  g                 */
        uint32_t motors;                 /* 24  4x10bit 电机 + 6bit 油门  */
        uint8_t  battery;                /* 28  0..100                   */
        uint8_t  reserved[3];            /* 29                           */
    } bb_frame_t;
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

import numpy as np

# ================================================================ 几何参数
SECTOR_SIZE = 4096
SECTOR_COUNT = 4096
PAGE_SIZE = 256
SECTOR_DATA_OFF = PAGE_SIZE                        # 数据从 256 字节对齐处开始
SECTOR_DATA_LEN = SECTOR_SIZE - SECTOR_DATA_OFF    # 3840
FRAME_SIZE = 32
FRAMES_PER_PAGE = PAGE_SIZE // FRAME_SIZE          # 8
PAGES_PER_SECTOR = SECTOR_DATA_LEN // PAGE_SIZE    # 15
FRAMES_PER_SECTOR = SECTOR_DATA_LEN // FRAME_SIZE  # 120

FLASH_BYTES = SECTOR_SIZE * SECTOR_COUNT           # 16 MB
TOTAL_FRAMES = SECTOR_COUNT * FRAMES_PER_SECTOR    # 491520 帧 -> 约 8.2 分钟
DUMP_BYTES = TOTAL_FRAMES * FRAME_SIZE             # 纯帧字节数 = 15 MB

# 固件 RAM 环形缓冲的帧数（Core/Src/blackbox.c 的 BB_RING_FRAMES）。
# 只用于解读 BB:STAT 里的 hwm=：hwm/这个数就是"缓冲被占满的比例"。
# 判读：稳态是个位数帧，换扇区擦 Flash 时会短时涨到几十帧，都正常；
#       接近 RING_FRAMES 说明 Logger 跟不上，随之而来的就是丢帧。
RING_FRAMES = 1024

# ================================================================ 帧格式
# 字段顺序：magic(H) type(B) flags(B) seq(H) roll..accel_z(9×h) motors(I) battery(B) pad(x)
#  seq 必须是 H(无符号)：固件用 uint16_t，数到 65535 才回绕；
#  早期版本误用 h(有符号)，32767 之后变负，会破坏跨扇区连续性判断。
FRAME_FIELDS = (
    "magic", "type", "flags", "seq",
    "roll", "pitch", "yaw",
    "gyro_x", "gyro_y", "gyro_z",
    "accel_x", "accel_y", "accel_z",
    "motors", "battery", "reserved",
)
FRAME_STRUCT = struct.Struct("<HBBHhhhhhhhhhIBBxx")
assert FRAME_STRUCT.size == FRAME_SIZE, (
    f"帧格式 {FRAME_STRUCT.size} 字节，应为 {FRAME_SIZE}")
assert len(FRAME_FIELDS) == len(FRAME_STRUCT.unpack(b"\x00" * FRAME_SIZE)), (
    "FRAME_FIELDS 与格式串字段数不一致（格式串写错时靠这里兜住）")

MAGIC_FRAME = 0x4246

TYPE_DATA = 0x01
TYPE_DIAG = 0x02
TYPE_RC   = 0x03
TYPE_ALT  = 0x04
# 摇杆帧 reserved 字节的格式特征值（与固件 BB_RC_FMT_TARGET 一致）
RC_FMT_TARGET = 0xB7
# 高度帧 reserved 字节的格式特征值（与固件 BB_ALT_FMT_MARK 一致）
ALT_FMT_MARK = 0xC3

# 固件每 64 帧插一帧诊断帧（与 blackbox.h 的 BB_DIAG_INTERVAL 一致）
DIAG_DIV = 64

# 扇区索引头（对应 bb_header_t，共 32 字节）。
# 字段：magic(u32) ftype(u8) 3x pad sector_index(u32) frame_offset(u16) pad2(u16)
#       boot_count(u32) reserved3[3](u32 x3)
HEADER_STRUCT = struct.Struct("<IB3xIHHIIII")
assert HEADER_STRUCT.size == 32, "扇区头格式必须是 32 字节"
MAGIC_HEADER = 0x42480000
TYPE_HEADER = 0xA5

# ================================================================ 缩放系数
LSB_ANGLE = 0.01      # deg
LSB_GYRO = 0.1        # deg/s
LSB_ACCEL = 0.001     # g

MOTOR_FULL = 4200.0                 # PWM 满量程
MOTOR_SCALE = MOTOR_FULL / 127.0    # 7bit  -> PWM
THROTTLE_SCALE = MOTOR_FULL / 15.0  # 4bit  -> PWM

# 诊断帧 conf 档位 0..3 对应的近似权重（固件把 0..1 量化成 4 档）
CONF_LEVELS = (0.0, 0.25, 0.5, 1.0)
PERIOD_UNIT_US = 100.0              # 诊断周期单位

# 空 Flash 擦除后是 0xFF，按本格式读出 yaw = -1（int16 的 0xFFFF）。
# 因此 yaw == -1 可作为"空白/未写"的可靠标记：真实偏航角被限制在 ±327.67deg，
# 不可能恰好落在 -0.01deg...精确等于 LSB 边界值上（且我们要的就是"是否写过"）。
BLANK_YAW = -1


def unpack_motors(word: int):
    """打包的电机/油门字 -> (m1, m2, m3, m4, throttle)，单位都是 PWM(0..4200)。"""
    m1 = (word & 0x7F) * MOTOR_SCALE
    m2 = ((word >> 7) & 0x7F) * MOTOR_SCALE
    m3 = ((word >> 14) & 0x7F) * MOTOR_SCALE
    m4 = ((word >> 21) & 0x7F) * MOTOR_SCALE
    thr = ((word >> 28) & 0x0F) * THROTTLE_SCALE
    return m1, m2, m3, m4, thr


@dataclass
class SectorHeader:
    phys_sector: int
    magic: int
    ftype: int
    sector_index: int
    frame_offset: int
    boot_count: int

    @property
    def valid(self) -> bool:
        return (self.magic == MAGIC_HEADER
                and self.ftype == TYPE_HEADER
                and self.frame_offset <= FRAMES_PER_SECTOR)


class FlightLog:
    """整片 Flash 解析结果（逐帧数组 + 扇区头列表）。"""

    def __init__(self, n_sectors: int):
        n = n_sectors * FRAMES_PER_SECTOR
        self.n_sectors = n_sectors
        self.seq = np.full(n, -1, dtype=np.int32)
        self.ftype = np.zeros(n, dtype=np.uint8)
        self.flags = np.zeros(n, dtype=np.uint8)
        self.angle = np.full((n, 3), np.nan, dtype=np.float32)
        self.gyro = np.full((n, 3), np.nan, dtype=np.float32)
        self.accel = np.full((n, 3), np.nan, dtype=np.float32)
        self.motor = np.full((n, 4), np.nan, dtype=np.float32)
        self.throttle = np.full(n, np.nan, dtype=np.float32)
        self.battery = np.zeros(n, dtype=np.uint8)
        self.diag_conf = np.full(n, np.nan, dtype=np.float32)
        self.diag_period = np.full(n, np.nan, dtype=np.float32)
        # 遥控摇杆帧（每 32ms 一帧）：r=摇杆 roll, p=pitch, y=yaw, t=throttle,
        # ro=外环 roll 输出, po=外环 pitch 输出。NaN 表示该位置不是摇杆帧。
        self.rc = np.full((n, 6), np.nan, dtype=np.float32)
        # 摇杆帧槽位格式版本：1=实际目标角，0=旧格式加计倾角
        self.rc_fmt = np.zeros(n, dtype=np.uint8)
        # 摇杆帧借用的 gyro_x/gyro_y/accel_x 槽：[arm, mode, flight_status]。
        # 判"为什么电机不转"必须能看 flight_status（0=LANDED 1=LANDING
        # 2=HORIZON 3=ALT_HOLD），所以单独存下来。
        self.rc_aux = np.full((n, 3), np.nan, dtype=np.float32)
        # 高度帧（每 20ms 一帧，类型 0x04）：
        #   [0]气压高度 [1]融合高度 [2]垂直速度 [3]创新 [4]加计偏置
        #   [5]气压更新间隔(ms) [6]地面参考 [7]原始气压(hPa) [8]定高油门修正
        # NaN 表示该位置不是高度帧。
        self.alt = np.full((n, 9), np.nan, dtype=np.float32)
        # 高度帧的状态位：bit0 baro_ok, bit1 创新被限幅, bit2 增益被拒, bit3 气压有效
        self.alt_status = np.zeros(n, dtype=np.uint8)
        # 高度帧 motors 槽里顺带记的"定高基准油门"(学到的悬停油门, PWM) 和有效标志
        self.alt_hover = np.zeros(n, dtype=np.float32)
        self.alt_hover_valid = np.zeros(n, dtype=np.uint8)
        self.sector_of = np.repeat(np.arange(n_sectors, dtype=np.int32), FRAMES_PER_SECTOR)
        self.headers: list[SectorHeader] = []
        self.blank = np.ones(n, dtype=bool)

    # ------------------------------------------------------------ 便捷访问
    @property
    def roll(self):
        return self.angle[:, 0]

    @property
    def pitch(self):
        return self.angle[:, 1]

    @property
    def yaw(self):
        return self.angle[:, 2]

    @property
    def accel_norm(self):
        return np.linalg.norm(self.accel, axis=1)

    # ---- 高度帧（类型 0x04）的便捷访问 ----
    @property
    def alt_baro(self):
        """气压高度 (m)，也就是融合器的输入"""
        return self.alt[:, 0]

    @property
    def alt_fused(self):
        """融合高度 (m)"""
        return self.alt[:, 1]

    @property
    def alt_vvel(self):
        """垂直速度 (m/s)"""
        return self.alt[:, 2]

    def alt_ok(self):
        """哪些高度帧的 baro_ok 位是 1"""
        return (self.alt_status & 0x01) != 0

    # ------------------------------------------------------------ 提取最近一段
    def all_frames(self):
        """按"写入顺序"返回整片里所有非空帧的索引。

        这是**最可靠**的取数方式：不依赖 seq 连续性推断"这是不是同一次记录"。
        固件是按扇区顺序依次写满的，所以按逻辑 sector_index 升序拼起来
        就是真实的时间顺序（环形回绕也成立，因为 sector_index 是单调递增的）。

        什么时候需要它：靠 seq 差值分段很容易被"每扇区实际消耗的 seq 数
        与存储帧数不等"这种情况破坏，结果是每 120 帧就被切成一段。
        与其猜，不如把所有数据按顺序全给出来。

        历史原因（已修）：旧固件换扇区时会把页缓冲里已攒好的 8 帧静默丢弃，
        于是"相邻扇区序号差 128 而每扇区只存 120 帧"（120+8）。现在固件不再丢，
        且 _sector_first_seq() 会把"不消耗序号"的高度帧排除掉，差值应当对得上。
        """
        hs = sorted([h for h in self.headers if h.valid],
                    key=lambda h: h.sector_index)
        parts = []
        for h in hs:
            lo = h.phys_sector * FRAMES_PER_SECTOR
            hi = lo + FRAMES_PER_SECTOR
            nz = np.flatnonzero(~self.blank[lo:hi])
            if nz.size:
                parts.append(nz + lo)
        if not parts:
            return np.flatnonzero(~self.blank)      # 无索引头时退化为全扫描
        return np.concatenate(parts)

    def latest_run(self):
        """定位"最近一次记录"的连续帧索引。

        环形缓冲里同时存在新旧多次记录，因此：
          1. 用扇区索引头里逻辑 sector_index 最大的扇区作为"最新写入点"；
          2. 从该扇区向后收集，直到遇到空白扇区；
          3. 再向前收集（回绕），直到空白扇区或遇到更早的索引头为止 ——
             同一次飞行常常横跨两个扇区，漏掉前半段会导致曲线缺一截。
        """
        return self._scan_run(None)

    def runs(self, max_runs: int = 12):
        """把整片 Flash 里所有记录切成若干段，按"从新到旧"返回。

        为什么需要它：环形缓冲里往往并存多次上电/多次解锁的记录，
        而"最新一段"可能只是落地后短暂解锁的几帧（甚至不到 0.2 秒）。
        只给最新一段会让人误以为"数据丢了"，其实那次飞行还在更早的扇区里。

        返回 [[帧索引数组], ...]，第 0 段是最新的。
        """
        exclude = np.zeros(self.blank.shape[0], dtype=bool)
        out = []
        for _ in range(max_runs):
            r = self._scan_run(exclude)
            if r.size == 0:
                break
            out.append(r)
            exclude[r] = True
        return out

    def _scan_run(self, exclude):
        """扫描一段最"新"的连续记录。exclude 里的帧视为已消费(等同空白)。"""
        blank = self.blank if exclude is None else (self.blank | exclude)

        valid_headers = [h for h in self.headers if h.valid]
        if not valid_headers:
            # 兜底：没有可用索引头时，退化为"最后一段连续非空帧"
            nz = np.flatnonzero(~blank)
            if nz.size == 0:
                return np.array([], dtype=int)
            # 从末尾往前找连续块（允许扇区之间无空洞）
            end = nz[-1]
            start = end
            while start - 1 >= 0 and not blank[start - 1]:
                start -= 1
            return np.arange(start, end + 1, dtype=int)

        newest = max(valid_headers, key=lambda h: h.sector_index)
        start_sector = newest.phys_sector
        newest_written = newest.frame_offset if newest.frame_offset > 0 else FRAMES_PER_SECTOR

        # ── 修正：最新扇区很可能是"刚建好、还没写数据"的那个 ──
        # 固件在解锁时会立刻建一个新扇区（frame_offset=0，数据区全 0xFF），
        # 若直接拿它当锚点，下面的 while 第一步就因 blank 而 break，
        # 结果就是明明有大量数据却报"没有找到任何有效飞行数据"。
        # 这里按逻辑序号从新到旧，跳过全空的扇区，找到真正有数据的最新扇区。
        for h in sorted(valid_headers, key=lambda x: x.sector_index, reverse=True):
            lo = h.phys_sector * FRAMES_PER_SECTOR
            hi = lo + FRAMES_PER_SECTOR
            if not blank[lo:hi].all():
                newest = h
                start_sector = h.phys_sector
                newest_written = h.frame_offset if h.frame_offset > 0 else FRAMES_PER_SECTOR
                break

        parts = []

        # ---- 向后：从最新扇区开始收集，直到空白扇区 ----
        anchor = start_sector                 # 固定锚点，回溯时不能改它
        s = start_sector
        while True:
            lo = s * FRAMES_PER_SECTOR
            hi = lo + FRAMES_PER_SECTOR
            if blank[lo:hi].all():
                break
            parts.append(np.flatnonzero(~blank[lo:hi]) + lo)
            s = (s + 1) % self.n_sectors
            if s == anchor:
                break

        # ---- 向前（回绕）：同一次记录常横跨多个扇区，必须补上前半段。
        #      判据：前一扇区首帧序号 + 该扇区已写帧数 == 本扇区首帧序号。
        #      注意循环条件是 s != anchor（固定值），否则条件会跟着走导致提前退出。
        s = (anchor - 1) % self.n_sectors
        while s != anchor:
            lo = s * FRAMES_PER_SECTOR
            hi = lo + FRAMES_PER_SECTOR
            nz = np.flatnonzero(~blank[lo:hi])
            if nz.size == 0:
                break

            if not self._is_same_run(s, (s + 1) % self.n_sectors, blank):
                break     # 序号接不上 -> 属于更早的一次记录

            parts.append(nz + lo)
            s = (s - 1) % self.n_sectors

        if not parts:
            return np.array([], dtype=int)
        parts.reverse()
        return np.concatenate(parts)

    def _sector_first_seq(self, s: int, blank=None):
        """求某物理扇区第一帧的(真实)序号；无法判定时返回 None。

        扇区首帧常常正好是诊断帧（诊断帧出现在 seq % DIAG_DIV == 0 处），
        此时它的槽位存的是 conf/period 而不是序号，于是向后找第一个数据帧，
        再用"位置差"把序号倒推回来。
        """
        blank = self.blank if blank is None else blank
        lo = s * FRAMES_PER_SECTOR
        hi = lo + FRAMES_PER_SECTOR
        for k in range(hi - lo):
            i = lo + k
            if blank[i]:
                continue
            if self.ftype[i] == TYPE_DATA:
                # 位置差 k 要从"消耗序号的槽位"来算：高度帧(ALT)不递增
                # bb_frame_seq（它借用当前序号），必须从 k 里扣掉。
                # 历史坑：固件曾经在换扇区时把页缓冲里已攒好的 8 帧静默丢弃，
                # 导致"相邻扇区序号差 128 而每扇区只存 120 帧"，序号连续性判据
                # 因此永远失败、每 120 帧就被切成一段。那个 bug 已修，这里再扣掉
                # ALT 帧之后，差值才真正等于扇区内的有效槽位数。
                consumed = int(np.count_nonzero(
                    (~blank[lo:i]) & (self.ftype[lo:i] != TYPE_ALT)))
                return (int(self.seq[i]) - consumed) % 65536
        return None

    def _is_same_run(self, prev_sector: int, cur_sector: int, blank=None) -> bool:
        """判断 cur_sector 是否紧接在 prev_sector 之后（同一次记录横跨扇区）。"""
        blank = self.blank if blank is None else blank
        prev_first = self._sector_first_seq(prev_sector, blank)
        cur_first = self._sector_first_seq(cur_sector, blank)
        if prev_first is None or cur_first is None:
            return False

        # 前一扇区"消耗了多少个序号"：只能是数实际槽位，
        #   ① 索引头的 frame_offset 记的是**写了几帧**（含不占序号的高度帧），
        #      和序号差不是一回事；而且当前固件的索引头原地重写写不进去，实测恒为 0。
        #   ② 所以一律用实际非空且"占序号"的槽位数（排除高度帧）。
        lo = prev_sector * FRAMES_PER_SECTOR
        hi = lo + FRAMES_PER_SECTOR
        written = int(np.count_nonzero(~blank[lo:hi] & (self.ftype[lo:hi] != TYPE_ALT)))
        if written <= 0:
            return False

        return ((cur_first - prev_first) % 65536) == (written % 65536)

    def data_frames(self, run):
        """从连续段中筛出普通数据帧（诊断帧的序号槽被复用，不可用于计时）。"""
        return run[self.ftype[run] == TYPE_DATA]

    def describe_headers(self) -> str:
        valid = [h for h in self.headers if h.valid]
        if not valid:
            return "未发现有效的扇区索引头（Flash 可能为空或未格式化）"
        valid.sort(key=lambda h: h.sector_index)
        lines = [f"发现 {len(valid)} 个有效扇区索引头："]
        for h in valid[-12:]:
            lines.append(f"  逻辑#{h.sector_index:<6} 物理扇区={h.phys_sector:<5} "
                         f"已写帧={h.frame_offset:<4} boot={h.boot_count}")
        if len(valid) > 12:
            lines.append(f"  ...（仅显示最后 12 个，共 {len(valid)} 个）")
        return "\n".join(lines)


def _looks_like_image(raw: bytes) -> bool:
    """判断这是"整片镜像"(含扇区头空洞)还是"纯帧流"。

    比凭长度判断可靠：看首个扇区前 32 字节是否像合法的索引头，
    或者扇区数据区起始处是否像一帧。
    """
    if len(raw) < SECTOR_DATA_OFF + FRAME_SIZE:
        return False
    # 情况 A：开头是合法索引头
    h = _parse_header(raw[:HEADER_STRUCT.size], 0)
    if h.valid:
        return True
    # 情况 B：索引头区域是空白(0xFF)但数据区有帧 -> 仍是镜像
    if raw[:HEADER_STRUCT.size] == b"\xFF" * HEADER_STRUCT.size:
        return True
    # 情况 C：开头直接就是帧 -> 纯帧流
    return False


def parse_flash(raw: bytes) -> FlightLog:
    """解析整片 Flash 转储。

    raw 为固件 BB:DUMP 输出的原始字节（含每个扇区开头的 256 字节索引头区域），
    也兼容已剔除空洞的纯帧流。
    """
    if _looks_like_image(raw):
        n_sectors = min(SECTOR_COUNT, len(raw) // SECTOR_SIZE)
        if n_sectors == 0:
            n_sectors = 1
        image = raw[:n_sectors * SECTOR_SIZE]
        log = FlightLog(n_sectors)

        for s in range(n_sectors):
            base = s * SECTOR_SIZE
            log.headers.append(_parse_header(image[base:base + 32], s))

            d0 = base + SECTOR_DATA_OFF
            lo = s * FRAMES_PER_SECTOR
            _fill_frames(log, lo, image, d0, FRAMES_PER_SECTOR)
        return log

    # 纯帧流：按扇区粒度还原物理排布
    n_frames = len(raw) // FRAME_SIZE
    n_sectors = max(1, (n_frames + FRAMES_PER_SECTOR - 1) // FRAMES_PER_SECTOR)
    log = FlightLog(n_sectors)
    for s in range(n_sectors):
        lo = s * FRAMES_PER_SECTOR
        _fill_frames_flat(log, lo, raw, s * FRAMES_PER_SECTOR * FRAME_SIZE,
                          min(FRAMES_PER_SECTOR, n_frames - s * FRAMES_PER_SECTOR))
    return log


def _parse_header(buf: bytes, phys: int) -> SectorHeader:
    if len(buf) < HEADER_STRUCT.size:
        buf = buf + b"\x00" * (HEADER_STRUCT.size - len(buf))
    (magic, ftype, sector_index, frame_offset,
     _pad2, boot_count, _r0, _r1, _r2) = HEADER_STRUCT.unpack_from(buf, 0)
    return SectorHeader(phys, magic, ftype, sector_index, frame_offset, boot_count)


def _decode_frame(buf: bytes, off: int):
    """解出一帧，返回 dict；空白帧返回 None。"""
    (magic, ftype, flags, slot, roll, pitch, yaw,
     gx, gy, gz, ax, ay, az, motors, bat, reserved) = FRAME_STRUCT.unpack_from(buf, off)

    if magic != MAGIC_FRAME:
        return None
    # 空白 Flash 全 0xFF：magic 恰好等于 0xFFFF != MAGIC_FRAME，已被上面挡掉，
    # 这里再用 yaw == BLANK_YAW 做二次确认。
    if yaw == BLANK_YAW and roll == BLANK_YAW and pitch == BLANK_YAW:
        return None

    m1, m2, m3, m4, thr = unpack_motors(motors)
    conf = period = None
    rc = None
    alt = None
    if ftype == TYPE_DIAG:
        conf_code = slot & 0xFF
        period_code = (slot >> 8) & 0xFF
        conf = CONF_LEVELS[conf_code] if conf_code < 4 else None
        period = period_code * PERIOD_UNIT_US
    elif ftype == TYPE_RC:
        # 遥控摇杆帧：int16 槽位含义与数据帧不同（见 blackbox.h 注释）
        rc = {
            "roll":  roll, "pitch": pitch, "yaw": yaw, "throttle": gx,
            "arm":   gy,   "mode":  gz,
            "status": ax,
            # 这两个槽位的含义随固件版本变过，靠帧内 reserved 字节区分：
            #   reserved=0xB7 -> 实际目标角（新格式，可当设定值用）
            #   其他值 -> 旧格式。旧固件没写过这个字节，它是栈垃圾，
            #   所以不能用 0/1 判断（容易碰巧），必须比对特征值。
            "rres": reserved,
            "target_roll":  ay * 0.01,
            "target_pitch": az * 0.01,
        }
    elif ftype == TYPE_ALT:
        # 高度帧（50Hz）：槽位含义见 blackbox.h 的 BB_TYPE_ALT 说明。
        # 解码后放在单独字段里 —— 绝不能把 roll/pitch 当角度、gyro_x 当角速度，
        # 否则会凭空造出几十 deg/s 的"假角速度"（和摇杆帧那个坑一模一样）。
        alt = {
            "mark":         reserved,
            "baro":         roll * 0.01,       # 气压高度 (m)，融合器输入
            "alt":          pitch * 0.01,      # 融合高度 (m)
            "vvel":         yaw * 0.01,        # 垂直速度 (m/s)
            "innov":        gx * 0.0001,       # 创新 (m)
            "bias":         gy * 0.0001,       # 加计偏置 (m/s^2)
            "baro_dt_ms":   gz * 0.1,          # 气压更新间隔 (ms)
            "alt_ref":      ax * 0.01,         # 地面参考 (m)
            # 气压槽 LSB = 10 Pa = 0.1 hPa（固件里气压是 Pa，按 10Pa 一档存），
            # 这里换算成人看着舒服的 hPa：101325 Pa -> 10132 -> 1013.2 hPa
            "pressure_hpa": ay * 0.1,          # 原始气压 (hPa)
            "thr_corr":     az * 1.0,          # 定高油门修正量 (PWM)
            # flags: bit0 baro_ok, bit1 创新被限幅, bit2 增益被拒, bit3 气压有效
            "status":       flags,
            "seq_tag":      slot,
            # motors 槽在高度帧里存"定高基准油门"：低16位=PWM，bit31=hover_valid
            "hover_base":   float(motors & 0xFFFF),
            "hover_valid":  1 if (motors >> 31) & 1 else 0,
        }

    return {
        "magic": magic, "type": ftype, "flags": flags, "slot": slot,
        "angle": (roll * LSB_ANGLE, pitch * LSB_ANGLE, yaw * LSB_ANGLE),
        "gyro": (gx * LSB_GYRO, gy * LSB_GYRO, gz * LSB_GYRO),
        "accel": (ax * LSB_ACCEL, ay * LSB_ACCEL, az * LSB_ACCEL),
        "motor": (m1, m2, m3, m4), "throttle": thr,
        "battery": bat, "conf": conf, "period": period, "rc": rc, "alt": alt,
    }


def _fill_frames(log: FlightLog, lo: int, buf: bytes, off: int, count: int):
    for k in range(count):
        f = _decode_frame(buf, off + k * FRAME_SIZE)
        if f is None:
            continue
        i = lo + k
        # 时间轴只认数据帧/诊断帧的 seq：摇杆帧和高度帧借用当前序号（不递增），
        # 留着会造成"重复序号"，破坏跨扇区连续性判断。
        log.seq[i] = f["slot"] if f["type"] not in (TYPE_DIAG, TYPE_RC, TYPE_ALT) else -1
        log.ftype[i] = f["type"]
        log.flags[i] = f["flags"]
        log.battery[i] = f["battery"]

        if f["type"] == TYPE_RC:
            # ★ 摇杆帧与数据帧共用同一个 32 字节结构，但槽位含义不同：
            #   roll/pitch/yaw 里放的是摇杆(1500)、gyro_y 里放的是 arm 值(2000)…
            # 如果把摇杆帧也写进 angle/gyro/accel 数组，就会凭空造出
            #   gyro_y = 2000*0.1 = 200 deg/s、roll = 1500*0.01 = 15deg
            # 的假数据。按 3% 的帧占比摊到均值上正好是 +6 deg/s 的"假零偏"，
            # 曾因此误判陀螺有问题。所以摇杆帧**只**写 rc，不碰其它数组。
            r = f["rc"]
            log.rc[i] = (r["roll"], r["pitch"], r["yaw"], r["throttle"],
                         r["target_roll"], r["target_pitch"])
            # 记下这一帧的槽位格式版本，供上层判断能不能当设定值用
            log.rc_fmt[i] = r["rres"]
            log.rc_aux[i] = (r["arm"], r["mode"], r["status"])
        elif f["type"] == TYPE_ALT:
            # 高度帧同理：槽位里放的是高度/创新/偏置，不能进 angle/gyro/accel。
            a = f["alt"]
            log.alt[i] = (a["baro"], a["alt"], a["vvel"], a["innov"], a["bias"],
                          a["baro_dt_ms"], a["alt_ref"], a["pressure_hpa"],
                          a["thr_corr"])
            log.alt_status[i] = a["status"]
            log.alt_hover[i] = a["hover_base"]
            log.alt_hover_valid[i] = a["hover_valid"]
        else:
            log.angle[i] = f["angle"]
            log.gyro[i] = f["gyro"]
            log.accel[i] = f["accel"]
            log.motor[i] = f["motor"]
            log.throttle[i] = f["throttle"]
            if f["conf"] is not None:
                log.diag_conf[i] = f["conf"]
            if f["period"] is not None:
                log.diag_period[i] = f["period"]

        log.blank[i] = False


def _fill_frames_flat(log: FlightLog, lo: int, buf: bytes, off: int, count: int):
    _fill_frames(log, lo, buf, off, count)
