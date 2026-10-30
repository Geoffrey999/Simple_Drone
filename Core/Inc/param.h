/**
  ******************************************************************************
  * @file    param.h
  * @brief   飞控参数表：ID/名字/默认值/范围 + 存取 + 控制台命令
  *
  * 设计（为什么要分"待生效"和"生效"两份）：
  *
  *   · g_val[]     —— 待生效值。CFG:SET 改的是它，CFG:SAVE 存的也是它。
  *   · g_applied[] —— 生效值。**只在启动时**从 g_val 拷一次，飞行代码只读它。
  *
  *   这样"改了参数 → 重启才生效"是物理保证的：即使飞行代码里有人手滑直接调
  *   Param_Get()，跑的也不是待生效表；而改参数绝不会让正在飞的飞机突然换性格。
  *
  * Flash 侧格式（TLV，见 param.c）：新固件加参数不会让旧记录失效（缺的项用
  * 编译期默认值），删参数也不会（多出来的 id 直接忽略）。
  *
  * ID 规则：高字节分组，低字节序号；**一旦发布就不能改数字**（记录里存的就是它）。
  *   0x01xx  PID          0x02xx  滤波/姿态估计
  *   0x03xx  摇杆/油门/定高   0x04xx  传感器校准（P3 预留，本版不序列化）
  ******************************************************************************
  */

#ifndef __PARAM_H__
#define __PARAM_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define PARAM_COUNT          82U      /* 表项数，与 param.c 里的 s_defs[] 一致 */
#define PARAM_NAME_MAX       28U      /* 最长名字 26 字符 + NUL + 余量 */
#define PARAM_ID_INVALID     0xFFFFU

/* 序列化后的一整份参数（68 项 × 6 B + 4 B 头 = 412 B） */
typedef struct {
    float v[PARAM_COUNT];
} ParamSet;

/* ============================ 参数 ID ============================ */
typedef enum {
    /* ── PID：0x0100 + 环×8 + 项(0=kp 1=ki 2=kd 3=integral_max 4=dterm_lpf) ── */
    P_ROLL_OUTER_KP    = 0x0100, P_ROLL_OUTER_KI,  P_ROLL_OUTER_KD,
    P_ROLL_OUTER_IMAX,           P_ROLL_OUTER_DLPF,
    P_PITCH_OUTER_KP   = 0x0108, P_PITCH_OUTER_KI, P_PITCH_OUTER_KD,
    P_PITCH_OUTER_IMAX,          P_PITCH_OUTER_DLPF,
    P_YAW_OUTER_KP     = 0x0110, P_YAW_OUTER_KI,   P_YAW_OUTER_KD,
    P_YAW_OUTER_IMAX,            P_YAW_OUTER_DLPF,
    P_ROLL_INNER_KP    = 0x0118, P_ROLL_INNER_KI,  P_ROLL_INNER_KD,
    P_ROLL_INNER_IMAX,           P_ROLL_INNER_DLPF,
    P_PITCH_INNER_KP   = 0x0120, P_PITCH_INNER_KI, P_PITCH_INNER_KD,
    P_PITCH_INNER_IMAX,          P_PITCH_INNER_DLPF,
    P_YAW_INNER_KP     = 0x0128, P_YAW_INNER_KI,   P_YAW_INNER_KD,
    P_YAW_INNER_IMAX,            P_YAW_INNER_DLPF,
    P_ALT_KP           = 0x0130, P_ALT_KI,         P_ALT_KD,
    P_ALT_IMAX,                  P_ALT_DLPF,
    P_ALT_VEL_KP       = 0x0138, P_ALT_VEL_KI,     P_ALT_VEL_KD,
    P_ALT_VEL_IMAX,              P_ALT_VEL_DLPF,

    P_PID_RATE_LIMIT   = 0x0180,   /* 外环输出限幅(deg/s) */

    /* ── 滤波 / 姿态 ── */
    P_GYRO_LPF_X       = 0x0200, P_GYRO_LPF_Y, P_GYRO_LPF_Z,
    P_ACCEL_LPF_VERT   = 0x0210,
    P_NOTCH_FREQ       = 0x0220, P_NOTCH_BW,
    P_AHRS_BETA        = 0x0230,
    P_EST_KP           = 0x0240, P_EST_KV, P_EST_KB,

    /* ── 摇杆 / 油门 / 定高 ── */
    P_STICK_DEG        = 0x0300,   /* (rc-1500)/deg → 目标角 */
    P_STICK_YAW_SCALE  = 0x0301,
    P_CTRL_ROLL_OFF    = 0x0302,   /* 目标角常驻微调(deg)，等价于遥控器微调 */
    P_CTRL_PITCH_OFF   = 0x0303,
    P_THR_SCALE        = 0x0310,
    P_THR_COMP_A       = 0x0311,   /* 电压补偿：/ (bat*a + b) */
    P_THR_COMP_B       = 0x0312,
    P_THR_THO_H        = 0x0313,   /* 定高模式油门高/低参考 */
    P_THR_THO_L        = 0x0314,
    P_THR_VOL_H        = 0x0315,
    P_THR_VOL_L        = 0x0316,
    P_ALT_REF_THR      = 0x0320,
    P_ALT_REF_VOLT     = 0x0321,
    P_ALT_REF_PCT      = 0x0322,
    P_ALT_STICK_BASE   = 0x0323,   /* 定高油门→目标高度：base + (thr-1000)/range */
    P_ALT_STICK_RANGE  = 0x0324,
    P_ALT_RATE_MAX     = 0x0325,   /* 定高目标垂速限幅 (m/s)，上升/下降共用 */
    P_ALT_LAND_FLARE   = 0x0326,   /* 降落：近地减速的下限下降率 (m/s) */
    P_ALT_LAND_TIMEOUT = 0x0327,   /* 降落：超时强制断电 (s) */
    /* 手动模式油门曲线（分段线性，四点 (1000,0)(x1,y1)(x2,y2)(2000,1)） */
    P_THR_CV_X1        = 0x0328,   /* 缓区起点（摇杆值） */
    P_THR_CV_Y1        = 0x0329,   /* 该点的输出比例 0..1 */
    P_THR_CV_X2        = 0x032A,   /* 缓区终点（摇杆值） */
    P_THR_CV_Y2        = 0x032B,   /* 该点的输出比例 0..1 */
    P_ALT_HOVER_THR    = 0x0330,   /* 悬停油门学习值（当前代码未启用，预留） */

    /* ── 传感器校准（0x04xx）── 参数只用来"记住"标定结果，值由 CFG:CAL 实测写入
     *    档 2 = 只做零偏：上电直接减掉，不需要每次都跑 2 秒自标定。
     *    scale 保持 1.0（六面标定留到以后，代码已在 ICM42605_SixSideCalibrate）。 */
    P_CAL_GYRO_X       = 0x0401,   /* 陀螺零偏 (deg/s) */
    P_CAL_GYRO_Y       = 0x0402,
    P_CAL_GYRO_Z       = 0x0403,
    P_CAL_ACCEL_X      = 0x0411,   /* 加计零偏 (g) */
    P_CAL_ACCEL_Y      = 0x0412,
    P_CAL_ACCEL_Z      = 0x0413,   /* 水平放置时该值为负表示 az 偏大 */

    /* ── 功能开关（0/1）── */
    P_BB_RECORDING     = 0x0500    /* 1=把飞行数据记进黑匣子（默认）；0=完全不记录 */
} ParamId;

/* ============================ 生命周期 ============================ */

/* 上电调用（调度器启动前）：EE → 待生效表；再快照一份给飞行代码。
 * 没有任何记录时全部用编译期默认值。 */
void Param_Init(void);

/* ============================ 取值 ============================ */

/* 生效值：飞行代码用这个。只有重启才会变。 */
float Param_GetApplied(uint16_t id);

/* 待生效值：控制台用这个。 */
float Param_Get(uint16_t id);

/* ============================ 编辑 ============================ */
uint8_t  Param_Set(uint16_t id, float v);        /* 会 clamp/拒绝 NaN，返回 1=成功 */
uint8_t  Param_ResetDefaults(void);              /* 待生效表恢复编译期默认（不写 Flash） */
uint8_t  Param_ReloadFromEE(void);               /* 丢弃未保存改动，从最新记录重载 */
uint8_t  Param_Save(void);                       /* 待生效表落盘 */
uint8_t  Param_IsDirty(void);                    /* 1 = 有待生效但未生效的改动 */

/* ============================ 表查询 ============================ */
uint16_t    Param_IndexOfId(uint16_t id);        /* 找不到返回 PARAM_ID_INVALID */
uint16_t    Param_IdAt(uint16_t idx);
uint16_t    Param_FindByName(const char *name);  /* 返回 ID，找不到 PARAM_ID_INVALID */
const char *Param_NameAt(uint16_t idx);

/* ============================ 序列化 ============================ */
uint16_t Param_Serialize(uint8_t *buf, uint16_t buf_size);          /* 返回字节数，0=失败 */
uint8_t  Param_Deserialize(const uint8_t *buf, uint16_t len, ParamSet *out);

/* ============================ 历史回滚 ============================ */
/* idx=0 是最新一条。返回 1=成功。slot 可为 NULL。 */
uint8_t Param_ReadHistory(uint16_t idx, ParamSet *out, uint32_t *seq, uint16_t *slot);
uint8_t Param_LoadHistory(uint16_t idx);         /* 读进待生效表（不写 Flash） */
uint8_t Param_Rollback(uint16_t idx);            /* 读进待生效表并立即保存 */

/* ============================ 控制台 ============================ */
/* 处理 "CFG:..." 命令。返回 1 = 已处理；0 = 不认识（交给 flash_ee 的 RAW/ERASE）。 */
uint8_t Param_Console(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __PARAM_H__ */
