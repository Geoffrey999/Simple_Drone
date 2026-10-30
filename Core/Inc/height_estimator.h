#ifndef __HEIGHT_ESTIMATOR_H__
#define __HEIGHT_ESTIMATOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "filter.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <math.h>

/* ============================================================================
 *  高度估计：三阶互补滤波器（= 一个三阶观测器）
 *  融合加速度计（高频、快但要积分）与气压计（低频、绝对但要滤波）
 *
 *  状态: [altitude, velocity, accel_bias]
 *
 *  预测（IMU 频率 1kHz）:
 *      velocity += (a_up - bias) * dt
 *      altitude += velocity * dt
 *
 *  修正（与预测同一拍、**同一个 dt**）:
 *      innov = baro_alt - altitude
 *      velocity += Kp * innov * dt     ← ★ Kp 作用在**速度**上
 *      bias     -= Kv * innov * dt     ← ★ Kv 作用在**偏置**上
 *      altitude += Kb * innov * dt     ← ★ Kb 作用在**位置**上
 *
 *  闭环特征多项式: s³ + Kb·s² + Kp·s + Kv
 *  稳定充要条件 : Kb>0, Kp>0, Kv>0 且 Kb·Kp > Kv
 *  三重极点 1/T : Kb=3/T, Kp=3/T², Kv=1/T³   （T=5s → 0.6 / 0.12 / 0.008）
 *
 *  ⚠ 历史坑（已修）：修正步长原来用 baro_dt(≈20ms) 而不是 dt(1ms)，
 *    等于每个气压样本的修正被连续执行 20 次 → 等效增益整整放大 20 倍，
 *    设计上的 T=5s 三重极点实际变成了"85ms 跟随气压计"。
 *    现在步长统一用 dt；想让滤波强度和修复前完全一致，把三个增益一起 ×20
 *    （0.12/0.008/0.6 → 2.4/0.16/12.0，逐拍离散值完全相同）。
 * ========================================================================== */

/* 诊断快照：一次加锁拷出来，供黑匣子 ALT 帧使用 */
typedef struct {
    float altitude;         /* 融合高度 (m)                 */
    float velocity;         /* 垂直速度 (m/s，向上为正)      */
    float accel_bias;       /* 加速度偏置估计 (m/s²)         */
    float baro_altitude;    /* 最近一次有效气压高度 (m)      */
    float pressure_pa;      /* 最近一次原始气压 (**Pa**)     */
    float innov;            /* 最近一次创新 baro-alt (m)     */
    float baro_dt;          /* 实测气压更新间隔 (s，已限幅)  */
    float alt_ref;          /* Reset 时的参考面 (m)          */
    uint8_t baro_ok;        /* 1=气压数据新鲜可用            */
    uint8_t innov_clamped;  /* 最近一次创新被限幅            */
    uint8_t gain_rejected;  /* 1=参数越出稳定域，用了默认值  */
} HeightEstimator_View;

typedef struct {
    /* 估计状态 */
    float altitude;             /* 高度 (m)，相对参考面 */
    float velocity;             /* 垂直速度 (m/s)，向上为正 */
    float accel_bias;           /* 加速度偏置 (m/s^2) */

    /* 互补滤波增益 (连续时间增益，内部乘以 dt) */
    float Kp;                   /* 速度修正增益 (1/s) */
    float Kv;                   /* 偏置修正增益 (1/s^2) */
    float Kb;                   /* 位置修正增益 (1/s^3) */

    /* 气压计输入（受 baro_sem 保护） */
    float baro_altitude;
    float baro_pressure;        /* **Pa**，仅诊断 */
    float innov;                /* 仅诊断 */
    float alt_ref;              /* Reset 的参考面，仅诊断 */
    float baro_dt;              /* 实测更新间隔，仅诊断（已限幅） */
    uint16_t ms_since_baro;     /* 距上次有效气压样本的毫秒数 */
    uint16_t clamp_ms;          /* 创新连续被限幅的时长(ms) → 发散兜底用 */
    uint8_t baro_ok;            /* 1=气压新鲜；超时/从未收到则 0 */
    uint8_t innov_clamped;
    uint8_t gain_rejected;      /* 1=参数不满足稳定域，用了默认值 */
    uint8_t initialized;        /* 1=已有参考面，Predict 可以开始修正 */
    uint8_t got_sample;         /* 1=本次 Reset 之后已收到过合格气压样本 */

    SemaphoreHandle_t baro_sem;

    float dt;                   /* 最近一次预测的步长 (s) */
} HeightEstimator;

/* 气压样本多久没更新就认为失效（ms）。50Hz 正常是 20ms，留 10 倍余量。 */
#define HEIGHT_EST_BARO_TIMEOUT_MS   200U
/* 单次创新限幅（m）。正常只有几 cm，超过这个值一定是坏样本/长时间丢数。 */
#define HEIGHT_EST_INNOV_LIMIT_M     2.0f
/* 相邻两次气压样本的最大合理跳变（m）。20ms 内不可能变化更多。 */
#define HEIGHT_EST_BARO_MAX_STEP_M   100.0f
/* 创新**连续**被限幅超过这么久(ms)，就认为估计器已经跟不上真实高度
 * （典型场景：飞机以 3m/s 下坠而滤波器只有 T=5s 的带宽），
 * 直接把位置拉回气压计、速度清零重新收敛。
 * 取值要大于气压计单次毛刺的持续时间（实测起飞毛刺约 200ms），
 * 又要小于"发散到没法用"的时间，300ms 是个折中。 */
#define HEIGHT_EST_RELOCK_MS         300U

void HeightEstimator_Init(HeightEstimator *est);
void HeightEstimator_SetGains(HeightEstimator *est, float kp, float kv, float kb);
void HeightEstimator_Reset(HeightEstimator *est, float baro_altitude);

/* vert_accel_g : 天向运动加速度，单位 g，已扣重力（向上为正）
 * dt           : 本次积分步长 (s) —— 修正也用这个步长 */
void HeightEstimator_Predict(HeightEstimator *est, float vert_accel_g, float dt);

/* baro_altitude: 已减去地面参考的气压高度 (m)
 * pressure_pa  : 原始气压，单位 **Pa**（不是 hPa！BMP280 的 64 位补偿
 *                结果 /256 之后就是 Pa，典型值 101325），仅用于诊断。
 *                非法值可传 NAN。
 * 返回 0=接受，1=丢弃（NaN / 超范围 / 跳变过大 → 会自然触发超时失效） */
uint8_t HeightEstimator_UpdateBaro(HeightEstimator *est, float baro_altitude,
                                   float pressure_pa);

float HeightEstimator_GetHeight(HeightEstimator *est);
float HeightEstimator_GetVelocity(HeightEstimator *est);
uint8_t HeightEstimator_IsBaroOk(HeightEstimator *est);
void HeightEstimator_GetView(HeightEstimator *est, HeightEstimator_View *v);

#ifdef __cplusplus
}
#endif

#endif /* __HEIGHT_ESTIMATOR_H__ */
