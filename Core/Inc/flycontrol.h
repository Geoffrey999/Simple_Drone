#ifndef __FLYCONTROL_H__
#define __FLYCONTROL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "icm42605.h"

#define MOTOR_MIN 0
#define MOTOR_MAX 4200
#define	THROTTLE_TO_PWM_MULTIPLE_SCALE	3.0f
#define 	THO_H		380.0f	//ALT_STABLE_REF_THROTTLE_HIGH
#define 	VOL_H		60.0f		//ALT_STABLE_REF_VOLTAGE_PERCENTAGE_HIGH
#define 	THO_L		440.0f	//ALT_STABLE_REF_THROTTLE_LOW
#define 	VOL_L		26.0f		//ALT_STABLE_REF_VOLTAGE_PERCENTAGE_LOW

#define ALT_STABLE_REF_THROTTLE							537.0f
#define ALT_STABLE_REF_VOLTAGE							3.648f
#define ALT_STABLE_REF_VOLTAGE_PERCENTAGE		31.0f
#define	BATTERY_STEP												0.008f

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
		float integral_max;
    float prev_error;
    float output;
    float dterm_lpf;
    float dterm_filtered;
} PID;

typedef enum{
	LANDED=0,
	LANDING,
	HORIZON_FLYING,
	ALTITUDE_HOLD_FLYING
}Drone_Flying_Status;

typedef struct {
    PID roll_outer;
    PID roll_inner;
    PID pitch_outer;
    PID pitch_inner;
    PID yaw_outer;
    PID yaw_inner;
    PID altitude;
    PID altitude_velocity;

    float target_roll;
    float target_pitch;
    float target_yaw;
    float throttle;
    float target_altitude;
    float current_altitude;
    float prev_altitude;
    float current_vertical_velocity;

    /* 高度样本的实际间隔(s)。定高串级环的 dt 必须用它，不能写死。
     * 曾经写死 0.02（当时气压是 50Hz）；后来把气压读取周期改成 50ms(20Hz)，
     * 写死 0.02 让 D 项 =(Δerr/0.02) 直接放大 2.5 倍：
     * 气压的 ±15cm 波动被放大成目标垂速 ±75m/s → 内环 ±600 满舵，
     * 电机在 1050~2265 PWM 之间 20Hz 抖（实测 corr 饱和占比 22%）。
     * 由任务在收到新高度样本时更新。 */
    float alt_dt;

    /* 定高目标垂速限幅(m/s)，来自 alt.rate_max。上升/下降共用。
     * 没有它的话：位置误差 3m × kp_a(1.5) = 4.5m/s 的爬升命令 → 内环
     * kp_v(150) 翻译成 675 PWM（饱和）→ 定高起飞那一段"窜"得厉害。 */
    float alt_rate_max;

    /* 降落断电原因（只用于记录，0=没断 1=贴地高度 2=垂速塌陷 3=超时） */
    uint8_t land_cut_reason;

    /* 混控抗饱和的缩放系数：1.0 = 差动放得下、没动过；<1 = 这一拍把差动按比例
     * 缩小过（剧烈机动、四路要顶到限位时）。只用于记录/诊断。 */
    float mix_scale;

    /* 降落近地减速的下限下降率(m/s)，来自 alt.land_flare。
     * flying_status == LANDING 时，目标垂速由"离地高度 × 系数"给出并夹在
     * [-rate_max, -land_flare_rate] 之间 —— 见 FlyController_Update 里的 flare 段。 */
    float land_flare_rate;

    /* ── 定高串级环输出的油门修正量（原来藏在 Update 里的 static）── */
    float altitude_output;

    /* ── 悬停油门学习 ──
     * 学的是**绝对油门**(PWM)，不是比例。原来学的是"总油门/电压补偿值"的比例，
     * 但电压补偿直接吃遥控报的电量百分比，而那个值在飞行中会从 88% 跳到 37%
     * （电机加载 → 电压跌落），结果基准油门跟着抖好几百 PWM —— 等于给整机
     * 塞了一个巨大的假扰动。改成绝对量之后，电压补偿只作为"还没学会时的起点"。
     *   hover_throttle: 学到的悬停油门 (PWM)
     *   hover_learn_s : 累计"满足学习条件"的时长，达到门限才 hover_valid
     *   hover_valid   : 1 = 可以拿来当定高基准（上电时会从 alt.hover_throttle 恢复）
     */
    float hover_throttle;
    float hover_learn_s;
    uint8_t hover_valid;

    /* 学习时的"参考点"：hover_ref_pwm 是当时的悬停油门，hover_ref_pct 是当时
     * 遥控报的电量。只存绝对油门是不够的 —— 同一架机满电悬停 1500、快没电要 1700，
     * 换块满电电池上来还按 1700 给油门，就会一直往上冲。
     * 落盘时一起存（用 alt.ref_throttle / alt.ref_pct 这两个预留参数），
     * 上电时按电量差折算回来：hover = ref_pwm − 斜率×(当前电量 − ref_pct)。
     * 这样换电池后第一次切定高就是准的，不用等它重新学。 */
    float hover_ref_pwm;
    float hover_ref_pct;

    /* 本拍电量百分比（由任务每拍写入，供电压补偿/学习用）。
     * bat_percent 是原始值（只用于记录），bat_filt 是低通后的值（控制用）：
     * 实测同一架次里遥控报的电量会从 83% 抖到 50%（电机一加载电压就跌），
     * 直接拿去算电压补偿会让基准油门跟着抖 ±170 PWM（≈±1 m/s²），
     * 定高环就得一直追这个假扰动。 */
    uint16_t bat_percent;
    float    bat_filt;

		Drone_Flying_Status flying_status;
    
    uint16_t motor1;
    uint16_t motor2;
    uint16_t motor3;
    uint16_t motor4;
		
		/* 本拍是否收到了**新的**高度样本（队列非空） */
		uint8_t altitude_ready;
		/* 高度数据源是否**健康**（气压计存在且新鲜）。
		 * 0 时禁止定高修正、并把修正量清零、油门交回飞手 ——
		 * 和 altitude_ready 的区别：后者只表示"这一拍有没有新数据"，
		 * 正常情况下 20 拍里只有 1 拍是 1，不能拿它判健康。 */
		uint8_t alt_source_ok;
} FlyController;

void FlyController_Init(FlyController *fc);
/* 用参数表里的值覆盖 Init 设的编译期默认值。
 * 只在任务启动时调用一次 —— 飞行中不重新应用，保证"改参数必须重启才生效"。 */
void FlyController_ApplyGains(FlyController *fc);
void FlyController_Update(FlyController *fc, ICM42605_Data *imu_data);
void FlyController_SetTarget(FlyController *fc, float roll, float pitch, float yaw, float throttle);
void FlyController_SetTargetAltitude(FlyController *fc, float target_altitude);
void FlyController_SetCurrentAltitude(FlyController *fc, float current_altitude);
void FlyController_SetCurrentVerticalVelocity(FlyController *fc, float current_vertical_velocity);
void FlyController_SetBatteryPercent(FlyController *fc, uint16_t bat_percentage);
void FlyController_Reset(FlyController *fc);
float Throttle_Voltage_Compensation(uint16_t bat_percentage);

/* ── 悬停油门学习 ──
 * 把"平稳悬停时真正需要的总油门"学出来，作为定高模式的基准油门，
 * 让定高 PID 只做小量修正（而不是长期扛着几十%的油门缺口）。
 *
 * FlyController_UpdateHoverLearn 每拍(1kHz)调用；只有同时满足
 *   · 已解锁且不在 LANDED
 *   · 高度数据可用（否则垂速是冻结值）
 *   · |天向加速度| < 0.03g 且 |垂速| < 0.2m/s      ← 真的在平稳悬停
 *   · 倾角 < 8°、总油门在合理区间
 * 才认为样本有效。有效样本先快速收敛（~0.5s），再慢速精修（~2s）。
 * 旧实现的问题见 flycontrol.c 里的注释。 */
void FlyController_UpdateHoverLearn(FlyController *fc, float throttle_total,
                                    float a_up_g, float tilt_deg, float dt);

/* 定高模式该用的基准油门：学到就用学到的绝对油门，没学到退回电压补偿(旧行为)。
 * 上电时可以用 alt.hover_throttle 里存的历史学习值做起点。 */
float FlyController_GetHoverBase(FlyController *fc);

/* 上电时把 EEPROM 里学到的悬停油门灌进来（pwm<=0 表示没有存过） */
void FlyController_SeedHoverThrottle(FlyController *fc, float pwm);

/* 设置学习时的参考点（悬停油门 + 当时电量）。pwm<=0 表示没有参考点，
 * 之后只按绝对油门走（老行为）。 */
void FlyController_SetHoverReference(FlyController *fc, float ref_pwm, float ref_pct);

/* 按当前电量把参考点折算成"这块电池该用的悬停油门"。
 * 只在**未解锁**时调用：解锁后学习器接管，这里不再插手。
 * 电量用内部低通后的值（见 SetBatteryPercent）。 */
void FlyController_UpdateHoverForBattery(FlyController *fc);

/* 当前低通后的电量百分比（供落盘时记录参考点用） */
float FlyController_GetBatteryPercent(FlyController *fc);
#ifdef __cplusplus
}
#endif

#endif
