/**
  ******************************************************************************
  * @file    param.c
  * @brief   参数表实现：存取、序列化、历史回滚、CFG: 控制台命令
  *
  * ── 两层数据（"重启生效"的物理保证）──
  *   s_val[]     待生效：CFG:SET 改它、CFG:SAVE 存它
  *   s_applied[] 生效  ：启动时从 s_val 拷一次，飞行代码只读它
  *
  * ── Flash 载荷格式（TLV，自描述）──
  *   offset 0 : 'A' 'P'                      参数体标识
  *   offset 2 : count (uint16, 小端)
  *   offset 4 : count × { uint16 id; float value; }   共 6 B/项
  *   尾部补 0 到 4 字节整数倍（EE_Save 要求）
  *
  *   为什么用 TLV 而不是直接 memcpy 结构体：
  *     · 以后加参数，老记录里没有 → 该项拿编译期默认值，不用迁移函数；
  *     · 删参数，老记录里多出来 → id 查不到直接忽略；
  *     · 结构体布局/padding 变化不会悄悄破坏数据。
  *
  * ── 输出为什么不直接用 printf("%f") ──
  *   本工程用 MicroLIB（useUlib=1），它对浮点格式化的支持不可靠。
  *   所以参数值一律用自己写的定点转换打印，解析也用自己写的，
  *   避免哪天换个库版本就变成打印一堆乱码。
  ******************************************************************************
  */

#include "param.h"
#include "flash_ee.h"
#include "blackbox.h"
#include "usart.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* ==================================================================
 *  一、参数表
 *     ⚠ 必须按 id 升序排列（Param_IndexOfId 用二分查找）
 * ================================================================== */
typedef struct {
    uint16_t    id;
    const char *name;
    float       def;
    float       lo;
    float       hi;
} param_def_t;

#define P_(id, nm, dv, lo_, hi_)   { (id), (nm), (dv), (lo_), (hi_) }

static const param_def_t s_defs[] = {
    /* ── 外环（输出是角速度指令 deg/s）── */
    P_(P_ROLL_OUTER_KP,   "pid.roll_outer.kp",           9.00f,   0.00f,  50.0f),
    P_(P_ROLL_OUTER_KI,   "pid.roll_outer.ki",           1.00f,   0.00f,  10.0f),
    P_(P_ROLL_OUTER_KD,   "pid.roll_outer.kd",           0.12f,   0.00f,   5.0f),
    P_(P_ROLL_OUTER_IMAX, "pid.roll_outer.imax",        20.00f,   0.00f,1000.0f),
    P_(P_ROLL_OUTER_DLPF, "pid.roll_outer.dlpf",         0.16f,   0.01f,   1.0f),
    P_(P_PITCH_OUTER_KP,  "pid.pitch_outer.kp",          9.00f,   0.00f,  50.0f),
    P_(P_PITCH_OUTER_KI,  "pid.pitch_outer.ki",          1.00f,   0.00f,  10.0f),
    P_(P_PITCH_OUTER_KD,  "pid.pitch_outer.kd",          0.12f,   0.00f,   5.0f),
    P_(P_PITCH_OUTER_IMAX,"pid.pitch_outer.imax",       20.00f,   0.00f,1000.0f),
    P_(P_PITCH_OUTER_DLPF,"pid.pitch_outer.dlpf",        0.16f,   0.01f,   1.0f),
    P_(P_YAW_OUTER_KP,    "pid.yaw_outer.kp",            2.00f,   0.00f,  50.0f),
    P_(P_YAW_OUTER_KI,    "pid.yaw_outer.ki",            0.10f,   0.00f,  10.0f),
    P_(P_YAW_OUTER_KD,    "pid.yaw_outer.kd",            0.02f,   0.00f,   5.0f),
    P_(P_YAW_OUTER_IMAX,  "pid.yaw_outer.imax",         20.00f,   0.00f,1000.0f),
    P_(P_YAW_OUTER_DLPF,  "pid.yaw_outer.dlpf",          0.16f,   0.01f,   1.0f),
    /* ── 内环（角速率环）── */
    P_(P_ROLL_INNER_KP,   "pid.roll_inner.kp",          10.00f,   0.00f,  50.0f),
    P_(P_ROLL_INNER_KI,   "pid.roll_inner.ki",           0.25f,   0.00f,  10.0f),
    P_(P_ROLL_INNER_KD,   "pid.roll_inner.kd",           0.16f,   0.00f,   5.0f),
    P_(P_ROLL_INNER_IMAX, "pid.roll_inner.imax",         8.00f,   0.00f,1000.0f),
    P_(P_ROLL_INNER_DLPF, "pid.roll_inner.dlpf",         0.20f,   0.01f,   1.0f),
    P_(P_PITCH_INNER_KP,  "pid.pitch_inner.kp",         10.00f,   0.00f,  50.0f),
    P_(P_PITCH_INNER_KI,  "pid.pitch_inner.ki",          0.25f,   0.00f,  10.0f),
    P_(P_PITCH_INNER_KD,  "pid.pitch_inner.kd",          0.16f,   0.00f,   5.0f),
    P_(P_PITCH_INNER_IMAX,"pid.pitch_inner.imax",        8.00f,   0.00f,1000.0f),
    P_(P_PITCH_INNER_DLPF,"pid.pitch_inner.dlpf",        0.20f,   0.01f,   1.0f),
    P_(P_YAW_INNER_KP,    "pid.yaw_inner.kp",            6.00f,   0.00f,  50.0f),
    P_(P_YAW_INNER_KI,    "pid.yaw_inner.ki",            0.30f,   0.00f,  10.0f),
    P_(P_YAW_INNER_KD,    "pid.yaw_inner.kd",            0.06f,   0.00f,   5.0f),
    P_(P_YAW_INNER_IMAX,  "pid.yaw_inner.imax",          8.00f,   0.00f,1000.0f),
    P_(P_YAW_INNER_DLPF,  "pid.yaw_inner.dlpf",          0.16f,   0.01f,   1.0f),
    /* ── 定高 ── */
    P_(P_ALT_KP,          "pid.altitude.kp",            30.00f,   0.00f, 500.0f),
    P_(P_ALT_KI,          "pid.altitude.ki",             3.00f,   0.00f, 100.0f),
    P_(P_ALT_KD,          "pid.altitude.kd",             3.00f,   0.00f, 500.0f),
    P_(P_ALT_IMAX,        "pid.altitude.imax",         300.00f,   0.00f,2000.0f),
    P_(P_ALT_DLPF,        "pid.altitude.dlpf",           1.00f,   0.01f,   1.0f),
    P_(P_ALT_VEL_KP,      "pid.altitude_velocity.kp",    4.00f,   0.00f, 200.0f),
    P_(P_ALT_VEL_KI,      "pid.altitude_velocity.ki",    0.00f,   0.00f, 100.0f),
    P_(P_ALT_VEL_KD,      "pid.altitude_velocity.kd",    2.00f,   0.00f, 200.0f),
    P_(P_ALT_VEL_IMAX,    "pid.altitude_velocity.imax",100.00f,   0.00f,2000.0f),
    P_(P_ALT_VEL_DLPF,    "pid.altitude_velocity.dlpf",  1.00f,   0.01f,   1.0f),

    P_(P_PID_RATE_LIMIT,  "pid.outer_rate_limit",      250.00f,  50.00f, 500.0f),

    /* ── 滤波 / 姿态估计 ── */
    P_(P_GYRO_LPF_X,      "gyro.lpf_x",                  0.70f,   0.05f,  0.99f),
    P_(P_GYRO_LPF_Y,      "gyro.lpf_y",                  0.70f,   0.05f,  0.99f),
    P_(P_GYRO_LPF_Z,      "gyro.lpf_z",                  0.70f,   0.05f,  0.99f),
    P_(P_ACCEL_LPF_VERT,  "accel.lpf_vert",              0.50f,   0.05f,  0.99f),
    P_(P_NOTCH_FREQ,      "notch.freq",                315.00f,  50.00f, 480.0f),
    P_(P_NOTCH_BW,        "notch.bw",                    1.00f,   0.10f,   4.0f),
    P_(P_AHRS_BETA,       "ahrs.beta",                   0.20f,   0.01f,   2.0f),

    /* ── 高度互补滤波增益（三阶观测器，重启生效）──
     * 闭环 s³ + Kb·s² + Kp·s + Kv，稳定要求 Kb·Kp > Kv（SetGains 会检查）。
     * 教科书式"三重极点 1/T"：Kb=3/T, Kp=3/T², Kv=1/T³。T=5s → 0.6/0.12/0.008。
     *
     * ⚠ 下面这组默认值是 T=5s 那组**整体 ×20**，为什么：
     *   修复前修正步长错用了 baro_dt(20ms) 而不是 dt(1ms)，等效增益正好放大 20 倍，
     *   实际跑的就是这组。保持默认 = 保持修复前的飞行手感（逐拍离散值完全相同），
     *   不会因为一次固件升级就让定高环变慢 20 倍而失稳。
     *   想回到教科书式的 T=5s，把三项改成 0.12 / 0.008 / 0.6 即可。 */
    P_(P_EST_KP,          "est.Kp",                      2.40f,   0.01f,  30.0f),
    P_(P_EST_KV,          "est.Kv",                      0.16f,   0.001f,  5.0f),
    P_(P_EST_KB,          "est.Kb",                     12.00f,   0.05f,  60.0f),

    /* ── 摇杆 / 油门 / 定高 ── */
    P_(P_STICK_DEG,       "stick.deg_per_unit",          9.00f,   1.00f,  30.0f),
    P_(P_STICK_YAW_SCALE, "stick.yaw_rate_scale",        1.00f,   0.10f,   5.0f),
    P_(P_CTRL_ROLL_OFF,   "ctrl.roll_offset_deg",        0.00f, -10.00f,  10.0f),
    P_(P_CTRL_PITCH_OFF,  "ctrl.pitch_offset_deg",       0.00f, -10.00f,  10.0f),
    P_(P_THR_SCALE,       "thr.scale",                   3.00f,   1.00f,  10.0f),
    P_(P_THR_COMP_A,      "thr.comp_a",                  0.008f,  0.00f,   0.1f),
    P_(P_THR_COMP_B,      "thr.comp_b",                  3.40f,   1.00f,   5.0f),
    P_(P_THR_THO_H,       "thr.tho_h",                 380.00f,   0.00f,2000.0f),
    P_(P_THR_THO_L,       "thr.tho_l",                 440.00f,   0.00f,2000.0f),
    P_(P_THR_VOL_H,       "thr.vol_h",                  60.00f,   0.00f, 100.0f),
    P_(P_THR_VOL_L,       "thr.vol_l",                  26.00f,   0.00f, 100.0f),
    /* 【现在真的在用】悬停油门学习值的"参考点"：ref_throttle = 学习时的悬停油门，
     * ref_pct = 当时的电量。只存绝对油门无法表达"同机不同电量需要不同油门"
     * （实测满电悬停 1500、快没电 1700），换电池后就会一直冲。
     * 固件每次 disarm 写这两个值，上电时按电量差折算回来。
     * 出厂默认 537 远低于任何真实悬停油门，所以没存过的板子走"只按绝对油门"的老路径。 */
    P_(P_ALT_REF_THR,     "alt.ref_throttle",          537.00f,   0.00f,4200.0f),
    P_(P_ALT_REF_VOLT,    "alt.ref_volt",                3.648f,  0.00f,   5.0f),   /* 预留未用 */
    P_(P_ALT_REF_PCT,     "alt.ref_pct",                31.00f,   0.00f, 100.0f),
    P_(P_ALT_STICK_BASE,  "alt.stick_base",              1.00f,   0.00f,   5.0f),
    P_(P_ALT_STICK_RANGE, "alt.stick_range",           500.00f,  50.00f,1000.0f),
    /* 定高串级环的**目标垂速限幅**(m/s)：上升和下降共用。
     * 为什么必须有：位置误差 × kp_a 就是目标垂速，误差 3m 时能给出 4.5m/s，
     * 内环再乘 kp_v 就是 600 PWM 的油门突跳 —— 定高起飞那一段"窜"就是这么来的。
     * 限到 1.0 之后，从地面爬到 1m 用 1 秒、到 3m 用 3 秒，油门突跳约 150 PWM。
     * 嫌慢就往 1.5 调，嫌急就往 0.6 调。 */
    P_(P_ALT_RATE_MAX,    "alt.rate_max",                1.00f,   0.20f,   3.0f),
    /* 降落近地减速的下限（m/s）。飞行器离地越低，目标下降率越小：
     *   2m→1.6  1m→0.8  0.4m→0.32  再低→这个下限
     * 调大=落得快、对脚架冲击大；调小=最后一段很磨蹭。
     * 触地判据要求 |垂速|<0.25，所以这个值建议 ≤0.3，否则落地断电会等到撞地之后。 */
    P_(P_ALT_LAND_FLARE,  "alt.land_flare",              0.25f,  0.05f,  1.0f),
    /* 降落超时（s）：到点强制断电。防止气压漂移导致"永远到不了 0.1m"而一直往地里顶。 */
    P_(P_ALT_LAND_TIMEOUT,"alt.land_timeout",           40.00f,   5.00f,  60.0f),

    /* ── 手动模式油门曲线 ──
     * 只在 HORIZON（手动）用：摇杆 1000..2000 先映射成输出比例 0..1，再乘到
     * 原来的油门上。四个断点 (1000,0) (x1,y1) (x2,y2) (2000,1) 分段线性。
     * x1..x2 就是"最缓"的那一段 —— 默认 1500~1700，正好罩住悬停区间，
     * 把 y1/y2 设得接近就能把这段压平，手在悬停附近微调好调得多。
     * 默认 (1500,0.50) (1700,0.70) 时曲线与直线完全重合 → 行为和以前一样。 */
    P_(P_THR_CV_X1,       "thr.cv_x1",                1500.00f,1000.0f,1900.0f),
    P_(P_THR_CV_Y1,       "thr.cv_y1",                   0.50f,   0.00f,   0.99f),
    P_(P_THR_CV_X2,       "thr.cv_x2",                1700.00f,1050.0f,2000.0f),
    P_(P_THR_CV_Y2,       "thr.cv_y2",                   0.70f,   0.01f,   1.00f),
    /* 悬停油门学习值（PWM）。
     * 0 = 还没学到过；>0 = 上电时直接当定高基准的起点。
     * 固件在每次 disarm 时把学到的值写进来（只在未解锁时写 Flash），
     * 所以正常飞行几架次之后这里会是一个 1400~1800 的实数。
     * 想恢复"每次上电重新学"就把它设回 0。 */
    P_(P_ALT_HOVER_THR,   "alt.hover_throttle",          0.00f,   0.00f,4200.0f),

    /* ── 传感器校准（由 CFG:CAL 实测写入；默认全 0 = "还没标过"）──
     * 全 0 时上电仍走原来那套 1 秒自标定，行为与加参数之前一致。
     * 只要有一项非 0，就认为存过标定结果，上电直接用、不再要求静止。 */
    P_(P_CAL_GYRO_X,      "cal.gyro_x_bias",             0.00f, -15.00f,  15.0f),
    P_(P_CAL_GYRO_Y,      "cal.gyro_y_bias",             0.00f, -15.00f,  15.0f),
    P_(P_CAL_GYRO_Z,      "cal.gyro_z_bias",             0.00f, -15.00f,  15.0f),
    P_(P_CAL_ACCEL_X,     "cal.accel_x_off",             0.00f,  -0.50f,   0.50f),
    P_(P_CAL_ACCEL_Y,     "cal.accel_y_off",             0.00f,  -0.50f,   0.50f),
    P_(P_CAL_ACCEL_Z,     "cal.accel_z_off",             0.00f,  -0.50f,   0.50f),

    /* ── 功能开关（0/1）── */
    /* 黑匣子记录总开关：0 = 完全不动 Flash（连解锁时切扇区都不做），1 = 正常记录。
     * 默认 1，与加这个参数之前的行为一致。重启生效。 */
    P_(P_BB_RECORDING,    "bb.recording",                1.00f,   0.00f,   1.0f)
};

#define PARAM_STATIC_ASSERT(name, cond)  typedef char name[((cond) ? 1 : -1)]
PARAM_STATIC_ASSERT(param_table_size, ((sizeof(s_defs) / sizeof(s_defs[0])) == PARAM_COUNT));

/* 载荷长度：4 B 头 + 每项 6 B，**向上补到 4 的整数倍**。
 * EE_Save 要求长度是 4 的倍数，而 6B/项 不保证对齐：
 * 参数个数是偶数时自然对齐，奇数时会多出 2 字节填充（补 0）。
 * 这里用宏统一收敛，加/减参数都不用再算一遍。 */
#define PARAM_BLOB_SIZE   (((4U + (PARAM_COUNT * 6U)) + 3U) & ~3U)
PARAM_STATIC_ASSERT(param_blob_align, ((PARAM_BLOB_SIZE % 4U) == 0U));
PARAM_STATIC_ASSERT(param_blob_fit,   (PARAM_BLOB_SIZE <= 1008U));

/* ==================================================================
 *  二、状态
 * ================================================================== */
static float   s_val[PARAM_COUNT];        /* 待生效 */
static float   s_applied[PARAM_COUNT];    /* 生效（启动时快照，之后只读） */
static uint8_t s_ready;
static uint8_t s_table_ok = 1U;           /* 表是否按 id 升序（启动时校验） */
static uint8_t s_src_ee;                  /* 1 = 启动时从记录载入；0 = 用默认值 */

/* 复用缓冲（省栈：logger 任务只有 2KB 栈） */
static uint8_t  s_blob[PARAM_BLOB_SIZE];
static ParamSet s_set_a;
static ParamSet s_set_b;

/* ==================================================================
 *  三、小工具
 * ================================================================== */
static void p_send(const char *s)
{
    if (s != NULL)
    {
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 200);
    }
}

static void p_printf(const char *fmt, ...)
{
    char    buf[192];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    p_send(buf);
}

/* 浮点 → "12.345" 定点字符串。
 * 不用 %f：本工程用 MicroLIB，浮点格式化支持不可靠。
 * 参数范围都在 ±5000 内，×1000 用 int32 不会溢出。 */
static const char *p_fmt(float v, char *buf)
{
    int32_t  x;
    char    *p = buf;

    if (v != v)                     /* NaN */
    {
        (void)strcpy(buf, "nan");
        return buf;
    }
    if (v < 0.0f)
    {
        *p++ = '-';
        v = -v;
    }
    if (v > 100000.0f)
    {
        v = 100000.0f;              /* 防止整数部分溢出（正常不会到这） */
    }
    x = (int32_t)(v * 1000.0f + 0.5f);
    (void)sprintf(p, "%ld.%03ld", (long)(x / 1000), (long)(x % 1000));
    return buf;
}

/* 自写浮点解析：支持 [-+]数字[.数字]，其它一律失败。
 * 不用 atof/strtod：一是 Microlib 的浮点输入同样不可靠，
 * 二是它们对 "abc"/"12abc" 都返回 0，分不清"用户输入 0"和"输入非法"。 */
static uint8_t p_parse_float(const char *s, float *out)
{
    int32_t   sign = 1;
    uint32_t  ip = 0;
    uint32_t  fp = 0;
    uint32_t  div = 1;
    uint8_t   any = 0;
    uint8_t   frac = 0;

    if (s == NULL)
    {
        return 0;
    }
    if (*s == '-') { sign = -1; s++; }
    else if (*s == '+') { s++; }

    while (*s != '\0')
    {
        if ((*s >= '0') && (*s <= '9'))
        {
            any = 1;
            if (frac == 0U)
            {
                if (ip > 100000U) { return 0; }          /* 明显超出范围 */
                ip = (ip * 10U) + (uint32_t)(*s - '0');
            }
            else if (div < 100000U)
            {
                fp  = (fp * 10U) + (uint32_t)(*s - '0');
                div = div * 10U;
            }
            s++;
        }
        else if ((*s == '.') && (frac == 0U))
        {
            frac = 1U;
            s++;
        }
        else
        {
            return 0;
        }
    }

    if (any == 0U)
    {
        return 0;
    }
    *out = (float)sign * ((float)ip + ((float)fp / (float)div));
    return 1;
}

static uint8_t p_finite(float v)
{
    return (uint8_t)((v == v) && (v > -1.0e6f) && (v < 1.0e6f));
}

static float p_clamp(float v, float lo, float hi)
{
    if (v < lo) { return lo; }
    if (v > hi) { return hi; }
    return v;
}

/* 取下一个空格分隔的 token，返回剩余串 */
static const char *p_token(const char *s, char *dst, uint16_t dst_size)
{
    uint16_t n = 0;

    while (*s == ' ')
    {
        s++;
    }
    if (*s == '\0')
    {
        dst[0] = '\0';
        return s;
    }
    while ((*s != ' ') && (*s != '\0') && (n + 1U < dst_size))
    {
        dst[n++] = *s++;
    }
    dst[n] = '\0';
    return s;
}

/* ==================================================================
 *  四、表查询
 * ================================================================== */
uint16_t Param_IndexOfId(uint16_t id)
{
    uint16_t lo = 0;
    uint16_t hi = PARAM_COUNT;

    while (lo < hi)
    {
        uint16_t mid = (uint16_t)((lo + hi) / 2U);

        if (s_defs[mid].id == id)
        {
            return mid;
        }
        if (s_defs[mid].id < id)
        {
            lo = (uint16_t)(mid + 1U);
        }
        else
        {
            hi = mid;
        }
    }
    return PARAM_ID_INVALID;
}

uint16_t Param_IdAt(uint16_t idx)
{
    return (idx < PARAM_COUNT) ? s_defs[idx].id : PARAM_ID_INVALID;
}

const char *Param_NameAt(uint16_t idx)
{
    return (idx < PARAM_COUNT) ? s_defs[idx].name : "?";
}

uint16_t Param_FindByName(const char *name)
{
    uint16_t i;

    if (name == NULL)
    {
        return PARAM_ID_INVALID;
    }
    for (i = 0; i < PARAM_COUNT; i++)
    {
        if (strcmp(s_defs[i].name, name) == 0)
        {
            return s_defs[i].id;
        }
    }
    return PARAM_ID_INVALID;
}

/* ==================================================================
 *  五、取值 / 编辑
 * ================================================================== */
float Param_GetApplied(uint16_t id)
{
    uint16_t idx = Param_IndexOfId(id);

    return (idx == PARAM_ID_INVALID) ? 0.0f : s_applied[idx];
}

float Param_Get(uint16_t id)
{
    uint16_t idx = Param_IndexOfId(id);

    return (idx == PARAM_ID_INVALID) ? 0.0f : s_val[idx];
}

uint8_t Param_Set(uint16_t id, float v)
{
    uint16_t idx = Param_IndexOfId(id);

    if ((idx == PARAM_ID_INVALID) || (p_finite(v) == 0U))
    {
        return 0;
    }
    s_val[idx] = p_clamp(v, s_defs[idx].lo, s_defs[idx].hi);
    return 1;
}

uint8_t Param_ResetDefaults(void)
{
    uint16_t i;

    for (i = 0; i < PARAM_COUNT; i++)
    {
        s_val[i] = s_defs[i].def;
    }
    return 1;
}

uint8_t Param_IsDirty(void)
{
    uint16_t i;

    for (i = 0; i < PARAM_COUNT; i++)
    {
        if (s_val[i] != s_applied[i])
        {
            return 1;
        }
    }
    return 0;
}

/* ==================================================================
 *  六、序列化
 * ================================================================== */
uint16_t Param_Serialize(uint8_t *buf, uint16_t buf_size)
{
    uint16_t i;
    uint16_t off;

    if ((buf == NULL) || (buf_size < PARAM_BLOB_SIZE))
    {
        return 0;
    }

    buf[0] = (uint8_t)'A';
    buf[1] = (uint8_t)'P';
    buf[2] = (uint8_t)(PARAM_COUNT & 0xFFU);
    buf[3] = (uint8_t)((PARAM_COUNT >> 8) & 0xFFU);

    off = 4;
    for (i = 0; i < PARAM_COUNT; i++)
    {
        buf[off]      = (uint8_t)(s_defs[i].id & 0xFFU);
        buf[off + 1U] = (uint8_t)((s_defs[i].id >> 8) & 0xFFU);
        (void)memcpy(&buf[off + 2U], &s_val[i], 4U);
        off = (uint16_t)(off + 6U);
    }
    while (off < PARAM_BLOB_SIZE)
    {
        buf[off++] = 0U;
    }
    return (uint16_t)PARAM_BLOB_SIZE;
}

uint8_t Param_Deserialize(const uint8_t *buf, uint16_t len, ParamSet *out)
{
    uint16_t n;
    uint16_t i;
    uint16_t off;

    if ((buf == NULL) || (out == NULL) || (len < 4U))
    {
        return 0;
    }
    if ((buf[0] != (uint8_t)'A') || (buf[1] != (uint8_t)'P'))
    {
        return 0;                       /* 不是参数体（例如 P1 的测试记录）*/
    }

    n = (uint16_t)((uint16_t)buf[2] | ((uint16_t)buf[3] << 8));
    if (((uint32_t)4U + ((uint32_t)n * 6U)) > (uint32_t)len)
    {
        return 0;
    }

    /* 先全部填默认值：老记录里缺的参数自动拿默认值 */
    for (i = 0; i < PARAM_COUNT; i++)
    {
        out->v[i] = s_defs[i].def;
    }

    off = 4;
    for (i = 0; i < n; i++)
    {
        uint16_t id = (uint16_t)((uint16_t)buf[off] | ((uint16_t)buf[off + 1U] << 8));
        float    v;
        uint16_t idx;

        (void)memcpy(&v, &buf[off + 2U], 4U);
        off = (uint16_t)(off + 6U);

        idx = Param_IndexOfId(id);
        if (idx == PARAM_ID_INVALID)
        {
            continue;                   /* 未知 id（删掉的参数）：忽略 */
        }
        if (p_finite(v) == 0U)
        {
            continue;                   /* 坏值：保留默认值 */
        }
        out->v[idx] = p_clamp(v, s_defs[idx].lo, s_defs[idx].hi);
    }
    return 1;
}

/* ==================================================================
 *  七、生命周期
 * ================================================================== */
void Param_Init(void)
{
    uint16_t len = 0;
    uint16_t i;

    /* 0. 表必须按 id 升序（二分查找的前提）。顺序错了会让查找静默失效，
     *    所以在启动时校验一次：不升序就退回全默认值并标记 tbl=0，
     *    这样 CFG:INFO 一眼就能看出来，而不是某些参数莫名其妙改不动。 */
    for (i = 1; i < PARAM_COUNT; i++)
    {
        if (s_defs[i - 1U].id >= s_defs[i].id)
        {
            s_table_ok = 0U;
            break;
        }
    }

    /* 1. 默认值兜底 */
    for (i = 0; i < PARAM_COUNT; i++)
    {
        s_set_a.v[i] = s_defs[i].def;
    }

    /* 2. 有记录就覆盖（读失败/格式不对 → 保持默认值） */
    s_src_ee = 0U;
    if ((s_table_ok != 0U) &&
        (EE_Read(s_blob, (uint16_t)sizeof(s_blob), &len) == (uint8_t)EE_OK))
    {
        s_src_ee = Param_Deserialize(s_blob, len, &s_set_a);
    }

    /* 3. 待生效 = 生效 = 载入值（此刻还没起飞，两边一致） */
    for (i = 0; i < PARAM_COUNT; i++)
    {
        s_val[i]     = s_set_a.v[i];
        s_applied[i] = s_set_a.v[i];
    }
    s_ready = 1U;
}

uint8_t Param_ReloadFromEE(void)
{
    uint16_t len = 0;
    uint16_t i;
    uint8_t  ok = 0;

    for (i = 0; i < PARAM_COUNT; i++)
    {
        s_set_a.v[i] = s_defs[i].def;
    }
    if (EE_Read(s_blob, (uint16_t)sizeof(s_blob), &len) == (uint8_t)EE_OK)
    {
        ok = Param_Deserialize(s_blob, len, &s_set_a);
    }
    for (i = 0; i < PARAM_COUNT; i++)
    {
        s_val[i] = s_set_a.v[i];
    }
    return ok;
}

/* 返回：0=失败，1=已写入，2=内容未变化 */
uint8_t Param_Save(void)
{
    uint16_t len;
    uint8_t  r;

    if (s_ready == 0U)          /* 没跑过 Param_Init：不允许把未初始化的表写进 Flash */
    {
        return 0;
    }
    len = Param_Serialize(s_blob, (uint16_t)sizeof(s_blob));
    if (len == 0U)
    {
        return 0;
    }
    r = EE_Save(s_blob, len);

    if (r == (uint8_t)EE_OK)
    {
        return 1;
    }
    if (r == (uint8_t)EE_OK_UNCHANGED)
    {
        return 2;
    }
    return 0;
}

/* ==================================================================
 *  八、历史回滚
 * ================================================================== */
uint8_t Param_ReadHistory(uint16_t idx, ParamSet *out, uint32_t *seq, uint16_t *slot)
{
    uint16_t len = 0;
    uint32_t s   = 0;
    uint16_t sl  = 0;

    if (out == NULL)
    {
        return 0;
    }
    if (EE_ReadHistory(idx, s_blob, (uint16_t)sizeof(s_blob), &len, &s, &sl) != (uint8_t)EE_OK)
    {
        return 0;
    }
    if (Param_Deserialize(s_blob, len, out) == 0U)
    {
        return 0;
    }
    if (seq  != NULL) { *seq  = s;  }
    if (slot != NULL) { *slot = sl; }
    return 1;
}

uint8_t Param_LoadHistory(uint16_t idx)
{
    uint16_t i;

    if (Param_ReadHistory(idx, &s_set_a, NULL, NULL) == 0U)
    {
        return 0;
    }
    for (i = 0; i < PARAM_COUNT; i++)
    {
        s_val[i] = s_set_a.v[i];
    }
    return 1;
}

uint8_t Param_Rollback(uint16_t idx)
{
    if (Param_LoadHistory(idx) == 0U)
    {
        return 0;
    }
    return (Param_Save() != 0U) ? 1U : 0U;
}

/* ==================================================================
 *  九、控制台命令
 * ================================================================== */
static void p_print_param(uint16_t idx, uint8_t with_applied)
{
    char buf[20];

    if (with_applied != 0U)
    {
        char buf2[20];

        p_printf("CFG:P %04X %s %s applied=%s\r\n",
                 (unsigned)s_defs[idx].id,
                 s_defs[idx].name,
                 p_fmt(s_val[idx], buf),
                 p_fmt(s_applied[idx], buf2));
    }
    else
    {
        p_printf("CFG:P %04X %s %s%s\r\n",
                 (unsigned)s_defs[idx].id,
                 s_defs[idx].name,
                 p_fmt(s_val[idx], buf),
                 (s_val[idx] != s_applied[idx]) ? " *" : "");
    }
}

static void p_cmd_info(void)
{
    FlashEE_Status_t st;

    EE_GetStatus(&st);
    p_printf("CFG:INFO n=%u tbl=%u src=%s dirty=%u seq=%lu next=%u used=%u/%u bank=%c "
             "crc_fail=%lu erases=%lu wr_us=%lu er_us=%lu img=%08lX ovl=%u up=%lums\r\n",
             (unsigned)PARAM_COUNT,
             (unsigned)s_table_ok,
             (s_src_ee != 0U) ? "ee" : "def",
             (unsigned)Param_IsDirty(),
             (unsigned long)st.seq,
             (unsigned)st.next_slot,
             (unsigned)st.used_slots,
             (unsigned)EE_SLOT_COUNT,
             (st.bank == 0U) ? 'A' : 'B',
             (unsigned long)st.crc_fail,
             (unsigned long)st.erases,
             (unsigned long)st.wr_us,
             (unsigned long)st.er_us,
             (unsigned long)st.image_end,
             (unsigned)st.img_overlap,
             (unsigned long)HAL_GetTick());
}

static uint16_t p_lookup_id(const char *tok)
{
    uint32_t v;

    if (tok[0] == '\0')
    {
        return PARAM_ID_INVALID;
    }
    if ((tok[0] >= '0') && (tok[0] <= '9'))
    {
        if ((tok[0] == '0') && ((tok[1] == 'x') || (tok[1] == 'X')))
        {
            v = (uint32_t)strtoul(&tok[2], NULL, 16);
        }
        else
        {
            v = (uint32_t)strtoul(tok, NULL, 10);
        }
        return (v > 0xFFFFU) ? PARAM_ID_INVALID : (uint16_t)v;
    }
    return Param_FindByName(tok);
}

static void p_cmd_get(const char *arg)
{
    char     tok[PARAM_NAME_MAX + 4U];
    uint16_t id;
    uint16_t idx;

    (void)p_token(arg, tok, (uint16_t)sizeof(tok));
    id  = p_lookup_id(tok);
    idx = (id == PARAM_ID_INVALID) ? PARAM_ID_INVALID : Param_IndexOfId(id);

    if (idx == PARAM_ID_INVALID)
    {
        p_send("CFG:ERR name/id\r\n");
        return;
    }
    p_print_param(idx, 1U);
}

static void p_cmd_set(const char *arg)
{
    char     tok[PARAM_NAME_MAX + 4U];
    char     buf[20];
    char     buf2[20];
    const char *rest;
    float    v;
    float    clamped;
    uint16_t id;
    uint16_t idx;

    rest = p_token(arg, tok, (uint16_t)sizeof(tok));
    id   = p_lookup_id(tok);
    idx  = (id == PARAM_ID_INVALID) ? PARAM_ID_INVALID : Param_IndexOfId(id);

    if (idx == PARAM_ID_INVALID)
    {
        p_send("CFG:ERR name/id\r\n");
        return;
    }
    (void)p_token(rest, tok, (uint16_t)sizeof(tok));
    if (p_parse_float(tok, &v) == 0U)
    {
        p_send("CFG:ERR value\r\n");
        return;
    }
    if (p_finite(v) == 0U)
    {
        p_send("CFG:ERR value (nan/inf)\r\n");
        return;
    }

    clamped = p_clamp(v, s_defs[idx].lo, s_defs[idx].hi);
    s_val[idx] = clamped;

    if (clamped != v)
    {
        p_printf("CFG:OK %04X %s=%s (clamped from %s)\r\n",
                 (unsigned)s_defs[idx].id, s_defs[idx].name,
                 p_fmt(clamped, buf), p_fmt(v, buf2));
    }
    else
    {
        p_printf("CFG:OK %04X %s=%s (save+reboot)\r\n",
                 (unsigned)s_defs[idx].id, s_defs[idx].name, p_fmt(clamped, buf));
    }
}

static void p_cmd_list(void)
{
    uint16_t i;

    for (i = 0; i < PARAM_COUNT; i++)
    {
        p_print_param(i, 0U);
    }
    p_printf("CFG:LISTEND n=%u dirty=%u\r\n", (unsigned)PARAM_COUNT, (unsigned)Param_IsDirty());
}

static void p_cmd_save(void)
{
    uint8_t  r;
    FlashEE_Status_t st;

    r = Param_Save();
    EE_GetStatus(&st);

    if (r == 1U)
    {
        p_printf("CFG:SAVED seq=%lu slot=%u dirty=0 wr_us=%lu\r\n",
                 (unsigned long)st.seq,
                 (unsigned)((st.next_slot > 0U) ? (st.next_slot - 1U) : 0U),
                 (unsigned long)st.wr_us);
    }
    else if (r == 2U)
    {
        p_send("CFG:OK unchanged\r\n");
    }
    else if (st.img_overlap != 0U)
    {
        p_send("CFG:ERR img_overlap\r\n");
    }
    else if (BlackBox_IsArmed() != 0U)
    {
        p_send("CFG:ERR armed\r\n");
    }
    else
    {
        p_printf("CFG:ERR save (ready=%u)\r\n", (unsigned)st.ready);
    }
}

static void p_cmd_load(void)
{
    if (Param_ReloadFromEE() != 0U)
    {
        p_send("CFG:OK reloaded from flash\r\n");
    }
    else
    {
        p_send("CFG:OK no record, defaults\r\n");
    }
}

static void p_cmd_def(void)
{
    (void)Param_ResetDefaults();
    p_printf("CFG:DEF n=%u (save+reboot)\r\n", (unsigned)PARAM_COUNT);
}

/* 把 a→b 之间变化的参数拼进一行（a 较早，b 较新） */
static void p_diff(const char *prefix, const float *a, const float *b)
{
    char     body[136];
    char     line[200];
    uint16_t i;
    uint16_t shown = 0;
    uint16_t total = 0;
    uint16_t pos = 0;

    body[0] = '\0';

    for (i = 0; i < PARAM_COUNT; i++)
    {
        char item[64];
        char va[20];
        char vb[20];
        int  n;

        if (a[i] == b[i])
        {
            continue;
        }
        total++;
        if (shown >= 3U)
        {
            continue;
        }
        n = sprintf(item, "%s%s %s->%s",
                    (shown == 0U) ? "" : ", ",
                    s_defs[i].name,
                    p_fmt(a[i], va), p_fmt(b[i], vb));
        if ((n <= 0) || ((pos + (uint16_t)n) >= (uint16_t)(sizeof(body) - 1U)))
        {
            break;
        }
        (void)memcpy(&body[pos], item, (size_t)n + 1U);
        pos  = (uint16_t)(pos + (uint16_t)n);
        shown++;
    }

    if (total == 0U)
    {
        (void)snprintf(line, sizeof(line), "%ssame\r\n", prefix);
    }
    else if (total > shown)
    {
        (void)snprintf(line, sizeof(line), "%sdiff: %s (+%u more)\r\n",
                       prefix, body, (unsigned)(total - shown));
    }
    else
    {
        (void)snprintf(line, sizeof(line), "%sdiff: %s\r\n", prefix, body);
    }
    p_send(line);
}

static void p_cmd_hist(const char *arg)
{
    char     tok[8];
    char     head[72];
    uint16_t n = 5U;
    uint16_t depth;
    uint16_t idx;
    uint32_t seq;
    uint16_t slot;
    ParamSet *cur  = &s_set_a;
    ParamSet *prev = &s_set_b;
    ParamSet *tmp;
    uint8_t  have_prev = 0U;

    (void)p_token(arg, tok, (uint16_t)sizeof(tok));
    if (tok[0] != '\0')
    {
        uint32_t v = (uint32_t)strtoul(tok, NULL, 10);

        if (v > 0U)
        {
            n = (uint16_t)((v > 16U) ? 16U : v);
        }
    }

    depth = EE_HistoryCount();
    if (n > depth)
    {
        n = depth;
    }

    for (idx = 0; idx < n; idx++)
    {
        if (Param_ReadHistory(idx, cur, &seq, &slot) == 0U)
        {
            break;
        }

        (void)sprintf(head, "CFG:H %u seq=%lu slot=%u ",
                      (unsigned)idx, (unsigned long)seq, (unsigned)slot);

        if (have_prev == 0U)
        {
            /* 最新一条：和当前待生效表比较，一眼看出"盘上的"和"改过的"差在哪 */
            p_diff(head, s_val, cur->v);
        }
        else
        {
            /* 更旧的：和上一条更新的记录比较 */
            p_diff(head, prev->v, cur->v);
        }

        tmp  = prev;
        prev = cur;
        cur  = tmp;
        have_prev = 1U;
    }

    p_printf("CFG:HEND n=%u depth=%u\r\n", (unsigned)n, (unsigned)depth);
}

static void p_cmd_hist_index(const char *arg, uint8_t do_save)
{
    float    v;
    uint16_t idx;

    if (p_parse_float(arg, &v) == 0U)
    {
        p_send("CFG:ERR index\r\n");
        return;
    }
    idx = (uint16_t)v;
    if (idx >= EE_HistoryCount())
    {
        p_send("CFG:ERR range\r\n");
        return;
    }

    if (do_save == 0U)
    {
        if (Param_LoadHistory(idx) != 0U)
        {
            p_printf("CFG:LOADED idx=%u (ram only, save+reboot)\r\n", (unsigned)idx);
        }
        else
        {
            p_send("CFG:ERR decode\r\n");
        }
        return;
    }

    if (BlackBox_IsArmed() != 0U)
    {
        p_send("CFG:ERR armed\r\n");
        return;
    }
    if (Param_Rollback(idx) != 0U)
    {
        FlashEE_Status_t st;

        EE_GetStatus(&st);
        p_printf("CFG:ROLLED idx=%u new_seq=%lu (reboot to apply)\r\n",
                 (unsigned)idx, (unsigned long)st.seq);
    }
    else
    {
        p_send("CFG:ERR rollback\r\n");
    }
}

static void p_cmd_reboot(void)
{
    if (BlackBox_IsArmed() != 0U)
    {
        p_send("CFG:ERR armed\r\n");
        return;
    }
    p_send("CFG:REBOOTING\r\n");
    HAL_Delay(50);              /* 让这行字发完再复位 */
    NVIC_SystemReset();
}

static void p_cmd_help(void)
{
    p_send("CFG:INFO                status (dirty/seq/up)\r\n"
           "CFG:LIST                all params\r\n"
           "CFG:GET <id|name>       read one\r\n"
           "CFG:SET <id|name> <v>   edit (save+reboot to apply)\r\n"
           "CFG:SAVE                write to flash (disarmed only)\r\n"
           "CFG:LOAD                discard edits, reload from flash\r\n"
           "CFG:DEF                 restore compile-time defaults\r\n"
           "CFG:HIST [n]            recent saves + diff\r\n"
           "CFG:LOAD:<n>            load history n into ram\r\n"
           "CFG:ROLLBACK:<n>        load history n and save\r\n"
           "CFG:REBOOT              soft reset (disarmed only)\r\n"
           "CFG:RAW                 hexdump newest record\r\n"
           "CFG:ERASE               erase both banks (disarmed only)\r\n"
           "CFG:CAL                 re-calibrate gyro/accel bias (still 2s, disarmed)\r\n"
           "CFG:HELP                this help\r\n");
}

uint8_t Param_Console(const char *cmd)
{
    if (cmd == NULL)
    {
        return 0U;
    }

    if (strcmp(cmd, "CFG:INFO") == 0)          { p_cmd_info();     return 1U; }
    if (strcmp(cmd, "CFG:LIST") == 0)          { p_cmd_list();     return 1U; }
    if (strncmp(cmd, "CFG:GET", 7) == 0)       { p_cmd_get(&cmd[7]);  return 1U; }
    if (strncmp(cmd, "CFG:SET", 7) == 0)       { p_cmd_set(&cmd[7]);  return 1U; }
    if (strcmp(cmd, "CFG:SAVE") == 0)          { p_cmd_save();     return 1U; }
    if (strcmp(cmd, "CFG:LOAD") == 0)          { p_cmd_load();     return 1U; }
    if (strncmp(cmd, "CFG:LOAD:", 9) == 0)     { p_cmd_hist_index(&cmd[9], 0U); return 1U; }
    if (strncmp(cmd, "CFG:ROLLBACK:", 13) == 0){ p_cmd_hist_index(&cmd[13], 1U); return 1U; }
    if (strncmp(cmd, "CFG:HIST", 8) == 0)      { p_cmd_hist(&cmd[8]); return 1U; }
    if (strcmp(cmd, "CFG:DEF") == 0)           { p_cmd_def();      return 1U; }
    if (strcmp(cmd, "CFG:REBOOT") == 0)        { p_cmd_reboot();   return 1U; }
    if (strcmp(cmd, "CFG:HELP") == 0)          { p_cmd_help();     return 1U; }

    return 0U;      /* 交给 flash_ee 处理 RAW / ERASE */
}
