/**
  ******************************************************************************
  * @file    blackbox.h
  * @brief   W25Q128 黑匣子：飞行数据记录 + 串口上位机接口
  *
  * 硬件
  *   W25Q128JVSIQ (16MB, 4KB 扇区)
  *   NSS  : PA15 (软件控制, 低有效)
  *   SCK  : PC10  (SPI3)
  *   MISO : PC11  (SPI3)
  *   MOSI : PC12  (SPI3)
  *   指令/取数通道: USART3 (PB10=TX / PB11=RX, 默认 460800 8N1)
  *     ※ 不占用 USART1(遥控接收, 定长二进制帧)
  *
  * 设计要点
  *   1) 记录频率 1000Hz，数据帧固定 32 字节;
  *   2) 只在"已解锁"时记录，未解锁完全不写 Flash;
  *   3) 采样在 FlyControl 任务(1kHz)里只做一次内存拷贝，绝不阻塞;
  *      真正的 Flash 写入由独立的 Logger 任务分批完成，避免拖慢控制环;
  *   4) Flash 按"扇区"组成环形缓冲，写满自动回到 0 号扇区重新擦写，
  *      因此永远保留最近一段飞行数据(约 4.3 分钟);
  *   5) 每个扇区开头写一个 16 字节索引头(记录扇区号)，上电靠它恢复写指针;
  *   6) 每个帧自带 32bit 递增序号，Python 侧靠序号连续性判断丢帧。
  ******************************************************************************
  */

#ifndef __BLACKBOX_H__
#define __BLACKBOX_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "height_estimator.h"   /* BB_TYPE_ALT 帧要用到 HeightEstimator_View */

/* ============================ 几何参数 ============================
 * W25Q128 的"页"只有 256 字节，且页写入不能跨页边界(超出部分会回卷到本页开头)，
 * 所以数据区必须从 256 字节整数倍地址开始，且每次写恰好一整页。
 *
 * 每个扇区(4096B)布局：
 *   [0x000 .. 0x0FF] 扇区索引头(只用前 32B，其余留空，保证数据从 256 对齐处开始)
 *   [0x100 .. 0xFFF] 3840B 数据区 = 15 页 × 256B = 120 帧 × 32B
 * ================================================================ */
#define BB_SECTOR_SIZE        4096U                              /* W25Q128 扇区 4KB          */
#define BB_SECTOR_COUNT       4096U                              /* 16MB / 4KB                */
#define BB_PAGE_SIZE          256U                               /* W25Q128 页                */
#define BB_SECTOR_DATA_OFF    BB_PAGE_SIZE                       /* 数据区起点，256B 对齐      */
#define BB_SECTOR_DATA_LEN    (BB_SECTOR_SIZE - BB_SECTOR_DATA_OFF)   /* 3840B                  */
#define BB_FRAME_SIZE         32U                                /* 数据帧固定 32B            */
#define BB_FRAMES_PER_PAGE    (BB_PAGE_SIZE / BB_FRAME_SIZE)          /* 每页 8 帧              */
#define BB_PAGES_PER_SECTOR   (BB_SECTOR_DATA_LEN / BB_PAGE_SIZE)     /* 每扇区 15 页           */
#define BB_FRAMES_PER_SECTOR  (BB_SECTOR_DATA_LEN / BB_FRAME_SIZE)    /* 每扇区 120 帧          */
#define BB_RECORD_SECONDS     ((float)BB_SECTOR_COUNT * (float)BB_FRAMES_PER_SECTOR / 1000.0f)

/* 采样触发间隔：每 64 帧插一帧诊断帧(含 accel_confidence / sample_period / 垂直加速度) */
#define BB_DIAG_INTERVAL      64U

/* ============================ 帧定义 ============================ */
/* 'BH' —— 扇区索引头 magic(32bit) */
#define BB_MAGIC_HEADER       0x42480000U
/* 'BF' —— 数据帧 magic(16bit，与帧内 magic 字段宽度一致，避免截断) */
#define BB_MAGIC_FRAME        0x4246U

#define BB_TYPE_HEADER        0xA5U         /* 扇区索引头              */
#define BB_TYPE_FRAME         0x01U         /* 常规飞行数据帧          */
#define BB_TYPE_DIAG          0x02U         /* 诊断帧                  */
#define BB_TYPE_RC            0x03U         /* 遥控摇杆帧              */
#define BB_TYPE_ALT           0x04U         /* 高度帧(50Hz，高度融合内部量) */

/* 遥控摇杆帧的采样间隔（帧数）。32ms 一次：足够看出打杆趋势，
 * 又只占约 3% 的 Flash 空间。
 * 为什么要单独记它：判断"飞机往左飞"到底是
 *   · 操作者在打杆修正（机械/重心偏置），还是
 *   · 操作者回中但控制器自己把飞机推走了（姿态估计有偏），
 * 没有摇杆量就永远分不清这两种。 */
#define BB_RC_INTERVAL        32U
/* 摇杆帧内 reserved 字节的格式特征值。
 * 旧固件没写过这个字节，它是未初始化的栈垃圾，
 * 所以不能用 0/1 这种小数值判断（容易碰巧）。
 *   等于 0xB7 -> accel_y/accel_z 存的是**实际目标角**（×100 deg）
 *   其他任何值 -> 旧格式，不能当设定值用 */
#define BB_RC_FMT_TARGET      0xB7U
/* 高度帧(0x04) reserved 字节的特征值，用于和摇杆帧一样做格式识别 */
#define BB_ALT_FMT_MARK       0xC3U

/* 注意：不使用 #pragma pack(1)。Cortex-M4 对非 4 字节对齐地址做浮点访问会 HardFault，
 * 而下面两个结构体的字段顺序本身就满足自然对齐且总长恰好 32 字节，
 * 由编译期断言(SIZE_CHECK)保证，一旦被改坏会直接编译报错。 */

/* 扇区索引头：固定 32 字节，占用每个扇区的前 128 字节区域 */
typedef struct {
    uint32_t magic;          /* BB_MAGIC_HEADER                       */
    uint8_t  type;           /* BB_TYPE_HEADER                        */
    uint8_t  reserved[3];
    uint32_t sector_index;   /* 本扇区在环形缓冲中的逻辑序号(单调递增)  */
    uint16_t frame_offset;   /* 本扇区已写入帧数(上电恢复写指针用)      */
    uint16_t reserved2;
    uint32_t boot_count;     /* 上电次数                                */
    uint32_t reserved3[3];   /* 补齐到 32 字节                          */
} bb_header_t;

/* 编译期尺寸断言：C99 可用，Keil(ARMCC5/ARMCLANG) 与 GCC 通吃。
 * 若结构体尺寸不等于期望值，typedef 的数组长度会是负数从而编译报错。 */
#define BB_SIZE_CHECK(name, type, expect) \
    typedef char name[(sizeof(type) == (expect)) ? 1 : -1]

/* 飞行数据帧：固定 32 字节。
 *
 * 为什么要用 int16 定点而不是 float：诊断"左偏"最要紧的是姿态角(3) + 陀螺三轴(3)
 * + 加计三轴(3) 共 9 个量，用 float32 就要 36 字节，装不进 32 字节的帧。
 * 定点后 9 个量只占 18 字节，且精度完全够用：
 *   姿态角 0.01° / 陀螺 0.1°/s / 加计 0.001g
 */
#define BB_ANGLE_LSB      0.01f      /* deg    */
#define BB_GYRO_LSB       0.1f       /* deg/s  */
#define BB_ACCEL_LSB      0.001f     /* g      */

/* 飞行数据帧：固定 32 字节（实测偏移，改字段顺序前请重新核对）。
 *
 * 为什么用 int16 定点而不是 float：诊断"左偏"最要紧的是姿态角(3) + 陀螺三轴(3)
 * + 加计三轴(3) 共 9 个量，float32 要 36 字节，装不进 32 字节。定点后只占 18 字节：
 *   姿态角 0.01° / 陀螺 0.1°/s / 加计 0.001g
 *
 * ⚠ 踩过的坑：ARM ABI 会把结构体总长向上取整到 4 的倍数，所以"有效负载"必须
 *   正好停在 32 字节，否则尾部填充会把总长顶到 36。下面注释里的数字都是实测值。
 */
typedef struct {
    uint16_t magic;          /*  0  BB_MAGIC_FRAME(16bit，误判率 1/65536) */
    uint8_t  type;           /*  2  BB_TYPE_FRAME / BB_TYPE_DIAG / BB_TYPE_RC */
    uint8_t  flags;          /*  3  bit0 高度有效, bit1 定高模式, bit2 混控被抗饱和缩放 */
    /* seq 必须是**无符号**：它要数到 65535。用 int16 会在 32767 处回绕成负数，
     * 上位机的跨扇区连续性判断依赖它，会因此整段失效。
     * 诊断帧时本槽改存 conf/period(见下)。 */
    uint16_t seq;            /*  4  本次解锁内帧序号(1kHz, 16bit)       */
    int16_t  roll;           /*  6  ×100  deg                         */
    int16_t  pitch;          /*  8  ×100  deg                         */
    int16_t  yaw;            /* 10  ×100  deg                         */
    int16_t  gyro_x;         /* 12  ×10   deg/s                       */
    int16_t  gyro_y;         /* 14  ×10   deg/s                       */
    int16_t  gyro_z;         /* 16  ×10   deg/s                       */
    int16_t  accel_x;        /* 18  ×1000 g                           */
    int16_t  accel_y;        /* 20  ×1000 g                           */
    int16_t  accel_z;        /* 22  ×1000 g                           */
    /* 四路电机各 7bit(0..127 映射 PWM 0..4200，步进 33)，油门 4bit(0..15 同量程)。
     * 4*7+4 = 32，正好占满一个字；用 10bit 会溢出 32 位。解包方式：
     *   motor1 =  word        & 0x7F
     *   motor2 = (word >>  7) & 0x7F
     *   motor3 = (word >> 14) & 0x7F
     *   motor4 = (word >> 21) & 0x7F
     *   thr4   = (word >> 28) & 0x0F   -> 油门 = thr4 * (4200/15) */
    uint32_t motors;         /* 24                                    */
    uint8_t  battery;        /* 28  电量百分比 0..100                  */
    uint8_t  reserved;       /* 29  预留(补齐到 32，ARM ABI 要求总长为 4 的倍数) */
} bb_frame_t;

#ifndef BB_SKIP_SIZE_CHECK
BB_SIZE_CHECK(bb_header_size_check, bb_header_t, 32);
BB_SIZE_CHECK(bb_frame_size_check,  bb_frame_t,  BB_FRAME_SIZE);
#endif

/* ── BB_TYPE_RC（遥控摇杆帧）的槽位复用 ──
 * 与数据帧共用同一个 32 字节结构，只是 int16 槽的含义不同：
 *   roll   : 摇杆 roll    (1000..2000 原始值)
 *   pitch  : 摇杆 pitch
 *   yaw    : 摇杆 yaw
 *   gyro_x : 摇杆 throttle
 *   gyro_y : arm 值  (1000/2000)
 *   gyro_z : mode 值 (1000/1500/2000)
 *   accel_x: flight_status
 *   accel_y: **实际目标 roll** ×100 deg（就是送进控制器的设定值）
 *   accel_z: **实际目标 pitch** ×100 deg
 *   motors / battery 含义不变。
 *
 * 为什么要记它：判断“姿态估计是否偏”时，一直是用上位机另算一份加计倾角去对比，
 * 但上位机的角度约定不一定和固件一致，结果可能是“算法偏差”而非“姿态偏差”。
 * 在固件里用**同一套角度约定**算一份存下来，两个数一定同源，不再有争议。
 */
void     BlackBox_QueueRc(const uint16_t rc[4], uint16_t arm, uint16_t mode,
                          uint8_t flight_status, float target_roll_deg,
                          float target_pitch_deg, const uint16_t motor[4],
                          uint8_t battery_percent);

/* ── BB_TYPE_ALT（高度帧，50Hz，与气压样本同频）的槽位复用 ──
 * 数据帧的 32 字节已经排满，塞不进"融合高度/垂速/创新/偏置"这一组诊断量，
 * 所以照抄摇杆帧/诊断帧那套做法：同一个 32 字节结构，按 type 区分含义。
 * 每 20 个控制周期插一帧 = 50Hz，占 5% Flash（录制时长 8.2 → 7.8 分钟）。
 *
 *   roll   : 气压高度 baro      ×0.01 m   （已减地面参考，也就是融合器的输入）
 *   pitch  : 融合高度 alt       ×0.01 m
 *   yaw    : 垂直速度 vel       ×0.01 m/s
 *   gyro_x : 创新 innov         ×0.0001 m （baro-alt，看修正强度）
 *   gyro_y : 加速度偏置 bias    ×0.0001 m/s²
 *   gyro_z : baro_dt            ×0.1 ms   （看气压更新间隔，≥3.2s 就是时钟出事了）
 *   accel_x: 地面参考 alt_ref   ×0.01 m
 *   accel_y: 原始气压           ×10 Pa (= 0.1 hPa；固件里气压单位是 Pa)
 *   accel_z: 定高油门修正量     ×1 PWM   （flycontrol 的 altitude_output）
 *   motors : 低 16 位 = 定高基准油门（学到的悬停油门，PWM）；bit31 = hover_valid
 *   flags  : bit0 baro_ok, bit1 创新被限幅, bit2 增益参数被拒, bit3 气压值有效
 *   reserved: BB_ALT_FMT_MARK (0xC3)
 *   battery: 电量 %
 *
 * 为什么值得：判断高度算得对不对必须同时看到"输入(baro) / 输出(alt,vel) /
 * 修正量(innov,bias)"三组量，只记输出永远分不清是气压计坏了还是滤波器坏了。
 * 基准油门也记下来，是因为"修正量长期偏离 0"最常见的原因就是基准给错了
 * （悬停油门学习没生效 / 还没学会）。 */
void     BlackBox_QueueAlt(const HeightEstimator_View *v, float throttle_correction,
                           float hover_base, uint8_t hover_valid);

/* 请求把"学到的悬停油门"落盘到参数 alt.hover_throttle，并把
 * "悬停油门 + 当时电量"存进 alt.ref_throttle / alt.ref_pct（换电池后折算用）。
 * 由 FlyControl 任务在 disarm 时调用；真正的 Flash 写由 Logger 任务执行
 * （参数/Flash 域归它独占），并且只在未解锁时才会真的写。 */
void     BlackBox_RequestHoverThrottleSave(float pwm, float bat_pct);

/* --------------------------------------------------------------------------
 * 阶跃注入（BB:STEP）——专门用于量闭环阶跃响应
 *
 * 为什么需要它：手动打杆做阶跃有三个硬伤——摇杆量化、
 * 遥控帧只有 32ms 分辨率、每次幅度不一致。固件注入可做到
 * 幅度、时序完全可重复，且目标角直接记进摇杆帧，上位机零歧义对齐。
 *
 * ★ 不需要飞行中连串口：地面上发一次命令，它进入"待触发"状态，
 *   等飞机**解锁 + 油门超过 1300 + 姿态平稳 + 摇杆回中**持续一段时间后
 *   自动开始注入；命令若 120 秒内未被触发则自动作废。
 *
 * 安全设计：
 *   · 仅在已解锁时生效
 *   · 摇杆一动（偏离中位 >40）立即自动取消，飞手随时接管
 *   · 幅度/周期/次数都有上限，到期自动停止
 * -------------------------------------------------------------------------- */
void     BlackBox_StepStop(void);

/* 取当前应该叠加到目标角上的偏移（deg）。未激活时输出 0。
 * 需传入摇杆值，用于判定飞手是否接管。 */
void     BlackBox_GetStepOffset(float *roll_off_deg, float *pitch_off_deg,
                                uint16_t stick_roll, uint16_t stick_pitch,
                                uint8_t flight_ready);

/* ============================ 状态信息 ============================ */
typedef struct {
    uint8_t  flash_ok;           /* Flash 自检通过                     */
    uint8_t  jedec_id[3];        /* 厂商/型号 ID                       */
    uint8_t  jedec_raw[3];       /* 实际读到的原始值(诊断：为什么 ok=0) */
    uint8_t  init_step;          /* 自检进行到第几步(诊断用)            */
    uint32_t boot_count;
    uint32_t sector_index;       /* 当前逻辑扇区号                     */
    uint16_t frame_offset;       /* 当前扇区内已写帧数                 */
    uint32_t flight_id;
    uint32_t recorded_frames;    /* 本次上电累计记录帧数               */
    uint32_t dropped_frames;     /* 因缓冲满被丢弃的帧数               */
    /* RAM 环形缓冲的历史最高水位(帧)，每次解锁清零。判断"还剩多少余量"：
     *   个位数    -> Logger 跟得上，健康
     *   接近上限  -> Logger 已饱和，drop 马上就会涨（该查 Flash 或继续加大缓冲） */
    uint32_t ring_hwm;
    uint8_t  armed;
} BlackBox_Status_t;

/* ============================ API ============================ */
void     BlackBox_Init(void);                    /* 上电初始化：自检 + 恢复写指针  */
void     BlackBox_LoggerTask(void *argument);    /* Logger 任务主体                */

void     BlackBox_SetArmed(uint8_t armed);       /* 解锁/上锁，内部自行处理边界     */
/* 查询解锁状态：配置存储（内部 Flash 擦写）前必须确认未解锁。
 * 内部 Flash 扇区擦除是秒级的，解锁时执行会让控制环停顿上秒。 */
uint8_t  BlackBox_IsArmed(void);

/* 记录总开关（参数 bb.recording，重启生效）。
 * 0 = 不排队、不写页、解锁时也不切扇区 —— 飞行中完全不碰 SPI3/Flash。
 * 只影响"记录"，BB:STAT / BB:DUMP / BB:ERASE 等命令照常可用。 */
void     BlackBox_SetRecording(uint8_t on);
uint8_t  BlackBox_IsRecording(void);
void     BlackBox_Queue(float roll, float pitch, float yaw,
                        float gyro_x, float gyro_y, float gyro_z,
                        float accel_x, float accel_y, float accel_z,
                        float altitude, float vert_velocity, float vert_accel,
                        const uint16_t motor[4], float throttle,
                        uint8_t alt_valid, uint8_t alt_hold_mode,
                        uint8_t mix_saturated,
                        uint8_t land_cut_reason,
                        uint8_t battery_percent);

void     BlackBox_GetStatus(BlackBox_Status_t *st);

/* 串口上位机指令：喂入一个字节，内部组行、执行、回包（非阻塞） */
void     BlackBox_FeedCommandByte(uint8_t byte);
void     BlackBox_Poll(void);                    /* 在 Logger 任务里周期性调用    */

/* 启动一次数据倾倒(异步，由 Logger 任务完成)，start_frame/count 以帧为单位 */
uint8_t  BlackBox_RequestDump(void);

#ifdef __cplusplus
}
#endif

#endif /* __BLACKBOX_H__ */
