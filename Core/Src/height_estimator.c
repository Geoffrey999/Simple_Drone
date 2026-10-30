#include "height_estimator.h"
#include "dwt.h"

#define GRAVITY 9.80665f

/* 默认三重极点的时间常数(s)。参数表里的 est.Kp/Kv/Kb 会覆盖它，见 SetGains。 */
#define HEIGHT_EST_DEFAULT_T   5.0f

/* ---------------- 互斥量小工具 ----------------
 * 三个任务会碰这个结构：
 *   IMURead(prio 40)  : Predict —— 唯一写 altitude/velocity/bias 的任务
 *   BaroRead(prio 9)  : UpdateBaro / Reset / 各种读取
 *   FlyControl(prio 10): 只读(经 GetView 或 GetHeight)
 * Predict 用 0 超时取锁：拿不到就跳过这一拍（1kHz 下丢一拍积分的影响是 0.1%），
 * 绝不让高优先级的 IMU 任务为一个气压任务阻塞 —— 否则会拖慢姿态解算。 */
static uint8_t est_lock(HeightEstimator *est, TickType_t timeout)
{
    if (est->baro_sem == NULL)
    {
        return 1U;      /* 互斥量创建失败：退化成无锁，至少功能还在 */
    }
    return (xSemaphoreTake(est->baro_sem, timeout) == pdTRUE) ? 1U : 0U;
}

static void est_unlock(HeightEstimator *est)
{
    if (est->baro_sem != NULL)
    {
        (void)xSemaphoreGive(est->baro_sem);
    }
}

/* ============================ 初始化 ============================ */

void HeightEstimator_Init(HeightEstimator *est)
{
    est->altitude = 0.0f;
    est->velocity = 0.0f;
    est->accel_bias = 0.0f;

    /* 默认三重极点 1/T（Kb=3/T, Kp=3/T², Kv=1/T³）。
     * 参数表会用 SetGains 再覆盖一次。 */
    est->Kp = 3.0f / (HEIGHT_EST_DEFAULT_T * HEIGHT_EST_DEFAULT_T);
    est->Kv = 1.0f / (HEIGHT_EST_DEFAULT_T * HEIGHT_EST_DEFAULT_T * HEIGHT_EST_DEFAULT_T);
    est->Kb = 3.0f / HEIGHT_EST_DEFAULT_T;

    est->baro_altitude = 0.0f;
    est->baro_pressure = 0.0f;
    est->innov = 0.0f;
    est->alt_ref = 0.0f;
    est->baro_dt = 0.02f;           /* 50Hz，仅诊断 */
    est->ms_since_baro = 0xFFFFU;   /* 一开始就当作"没有气压" */
    est->clamp_ms = 0U;
    est->baro_ok = 0U;
    est->innov_clamped = 0U;
    est->gain_rejected = 0U;
    est->initialized = 0U;
    est->got_sample = 0U;

    est->baro_sem = xSemaphoreCreateMutex();

    est->dt = 0.001f;
}

/* 应用参数表里的增益，并检查稳定域。
 * s³ + Kb·s² + Kp·s + Kv 稳定的充要条件：Kb>0, Kp>0, Kv>0 且 Kb·Kp > Kv。
 * 参数表允许的范围比这个宽得多（比如 est.Kb=0 或者 Kv 给大了），
 * 一旦越界就**保留默认值**并置 gain_rejected，避免"高度自己越晃越大"却查不出原因。 */
void HeightEstimator_SetGains(HeightEstimator *est, float kp, float kv, float kb)
{
    /* NaN 自身不等于自身 */
    if ((kp != kp) || (kv != kv) || (kb != kb))
    {
        est->gain_rejected = 1U;
        return;
    }
    if ((kp <= 0.0f) || (kv <= 0.0f) || (kb <= 0.0f))
    {
        est->gain_rejected = 1U;
        return;
    }
    if ((kb * kp) <= kv)
    {
        est->gain_rejected = 1U;
        return;
    }

    if (est_lock(est, portMAX_DELAY) == 0U)
    {
        return;
    }
    est->Kp = kp;
    est->Kv = kv;
    est->Kb = kb;
    est->gain_rejected = 0U;
    est_unlock(est);
}

/* 把状态锁定到参考面（地面/起飞点）。baro_altitude 是此刻的气压高度，
 * 通常传 0（调用方已经把地面 offset 减掉了）。 */
void HeightEstimator_Reset(HeightEstimator *est, float baro_altitude)
{
    if (est_lock(est, portMAX_DELAY) == 0U)
    {
        return;
    }

    est->altitude       = baro_altitude;
    est->velocity       = 0.0f;
    est->accel_bias     = 0.0f;
    est->baro_altitude  = baro_altitude;
    est->alt_ref        = baro_altitude;
    est->innov          = 0.0f;
    est->ms_since_baro  = 0U;
    est->clamp_ms       = 0U;
    est->baro_ok        = 1U;       /* 参考面是可信的，先当作可用 */
    est->initialized    = 1U;
    est->got_sample     = 0U;       /* 下一帧气压直接采纳为参考，不做跳变检查 */

    est_unlock(est);
}

/* ============================ 预测 + 修正 ============================ */

void HeightEstimator_Predict(HeightEstimator *est, float vert_accel_g, float dt)
{
    float innov;
    float accel_mps2;

    /* 步长防守：0 或异常的 dt 会让积分/修正都失真。
     * 注意限幅要和调用方(freertos.c 的 200µs~5ms)保持一致，别一个 5ms 一个 100ms。 */
    if ((dt <= 0.0f) || (dt > 0.05f))
    {
        return;
    }
    /* 加速度本身也该是有限值（NaN 一旦进状态就永久污染） */
    if (vert_accel_g != vert_accel_g)
    {
        return;
    }

    /* 拿不到锁就跳过这一拍：宁可少积 1ms，也不要让 IMU 任务等气压任务 */
    if (est_lock(est, 0) == 0U)
    {
        return;
    }

    est->dt = dt;

    /* 1. 加速度积分 -> 速度（扣掉估计的偏置） */
    accel_mps2 = vert_accel_g * GRAVITY;
    est->velocity += (accel_mps2 - est->accel_bias) * dt;

    /* 2. 速度积分 -> 高度 */
    est->altitude += est->velocity * dt;

    /* 3. 气压超时计时：没有任何有效样本就撤销 baro_ok，
     *    上层据此禁止定高（不能拿"恒为 0 的假气压"当真值融合）。 */
    if (est->ms_since_baro < 1000U)
    {
        uint32_t add = (uint32_t)(dt * 1000.0f + 0.5f);

        if (add == 0U)
        {
            add = 1U;
        }
        est->ms_since_baro = (uint16_t)(est->ms_since_baro + add);
        if (est->ms_since_baro >= 1000U)
        {
            est->ms_since_baro = 1000U;
        }
    }
    if (est->ms_since_baro > HEIGHT_EST_BARO_TIMEOUT_MS)
    {
        est->baro_ok = 0U;
    }

    /* 4. 互补修正 —— ★ 步长必须是 dt（本次积分的时间），不是 baro_dt。
     *    原来用 baro_dt(≈20ms) 而本函数是 1kHz 调的，等于每个气压样本的修正
     *    被重复做了 20 次，等效增益放大 20 倍。详见 height_estimator.h 的说明。 */
    if ((est->initialized != 0U) && (est->baro_ok != 0U))
    {
        innov = est->baro_altitude - est->altitude;

        /* 创新限幅：一个坏气压样本(比如 I2C 干扰导致压力=0 → 高度 44330m)
         * 会在 1ms 内把状态推到天文数字并且再也回不来。 */
        if (innov > HEIGHT_EST_INNOV_LIMIT_M)
        {
            innov = HEIGHT_EST_INNOV_LIMIT_M;
            est->innov_clamped = 1U;
        }
        else if (innov < -HEIGHT_EST_INNOV_LIMIT_M)
        {
            innov = -HEIGHT_EST_INNOV_LIMIT_M;
            est->innov_clamped = 1U;
        }
        else
        {
            est->innov_clamped = 0U;
        }

        est->velocity    += est->Kp * innov * dt;   /* 速度 */
        est->accel_bias  -= est->Kv * innov * dt;   /* 偏置（负号是稳定所需的） */
        est->altitude    += est->Kb * innov * dt;   /* 位置 */
        est->innov        = innov;

        /* ── 发散兜底 ──
         * 创新**连续**顶在限幅上，说明估计值和气压计的差已经超过 2m 且迟迟拉不回来。
         * 实测过一次：降落模式把目标设成 -3m，定高环全力下压，飞机以 3m/s 下坠，
         * 而滤波器（T=5s 那组增益）的速度修正只有 0.24 m/s²，10 秒才能纠正 2.4m/s，
         * 结果融合高度一路冲到 -7m、偏置被推到 -0.07，定高环反而按 -7m 拼命加油门。
         * 这里把"位置"直接拉回气压计、"速度"清零（偏置保留），让它重新收敛。
         * 短暂的气压毛刺（<300ms）不会触发，所以不会被尖峰带跑。 */
        if (est->innov_clamped != 0U)
        {
            est->clamp_ms = (uint16_t)(est->clamp_ms + (uint16_t)(dt * 1000.0f + 0.5f));
            if (est->clamp_ms >= HEIGHT_EST_RELOCK_MS)
            {
                est->altitude    = est->baro_altitude;
                est->velocity    = 0.0f;
                est->innov       = 0.0f;
                est->innov_clamped = 0U;
                est->clamp_ms    = 0U;
            }
        }
        else
        {
            est->clamp_ms = 0U;
        }

        /* 状态的数值防护：万一被外部搞坏，至少不要变成 inf/NaN 传下去 */
        if ((est->altitude != est->altitude) || (est->velocity != est->velocity) ||
            (est->altitude > 100000.0f) || (est->altitude < -100000.0f) ||
            (est->velocity > 10000.0f)  || (est->velocity < -10000.0f))
        {
            est->altitude    = 0.0f;
            est->velocity    = 0.0f;
            est->accel_bias  = 0.0f;
            est->initialized = 0U;      /* 等下一次气压样本重新锁参考面 */
            est->got_sample  = 0U;
            est->baro_ok     = 0U;
        }
    }

    est_unlock(est);
}

/* ============================ 气压更新 ============================ */

uint8_t HeightEstimator_UpdateBaro(HeightEstimator *est, float baro_altitude,
                                   float pressure_pa)
{
    static uint32_t last_us = 0;
    uint32_t        now_us;
    uint32_t        d_us;

    /* 采样间隔只用于诊断，但**必须限幅**：DWT 时钟是跨任务共享的，
     * 偶发一次假的"溢出"会让差值跳 +25.6 秒。以前这个值直接当环路增益用，
     * 一跳就会把状态打到 1e22；现在它只是诊断量，仍然限幅以免写进黑匣子后误导判读。 */
    now_us = DWT_GetUS();
    if (last_us == 0U)
    {
        last_us = now_us;
        d_us    = 20000U;               /* 首拍按 50Hz 记 */
    }
    else
    {
        d_us = now_us - last_us;
        last_us = now_us;
    }
    if (d_us < 2000U)   { d_us = 2000U; }      /* 500Hz 上限 */
    if (d_us > 100000U) { d_us = 100000U; }    /* 10Hz  下限 */

    /* ---- 样本合理性检查：不合格直接丢弃，不更新 baro_altitude ----
     * 丢弃时**不**清 ms_since_baro，于是连续丢样本会在 200ms 后自动置 baro_ok=0，
     * 上层就会禁止定高，而不是拿着一份冻结的旧数据继续飞。 */
    if (baro_altitude != baro_altitude)                     /* NaN */
    {
        return 1U;
    }
    if ((baro_altitude > 20000.0f) || (baro_altitude < -5000.0f))
    {
        return 1U;
    }
    /* ★ 单位是 Pa，不是 hPa！
     *   BMP280 的 64 位补偿公式返回 Q24.8（Pa×256），bmp280.c 里 /256 之后
     *   得到的 data->pressure 就是 Pa，典型值 101325。
     *   曾经这里按 hPa 写成 300~1200，结果每一拍气压都被判为非法：
     *   队列永远不推数据 → altitude_ready 恒 0 → 定高完全不能用。
     *   范围取 300~1200 hPa 对应的 30000~120000 Pa，和 BMP280_CalcAltitude 一致。 */
    if ((pressure_pa == pressure_pa) &&
        ((pressure_pa > 120000.0f) || (pressure_pa < 30000.0f)))
    {
        return 1U;
    }

    if (est_lock(est, portMAX_DELAY) == 0U)
    {
        return 1U;
    }

    /* Reset 之后（或从来没 Reset 过）的第一帧合格样本：直接采纳为参考面，
     * 不做跳变检查。原因：Reset 常常传 0（调用方已经把地面 offset 减掉了），
     * 万一 offset 没算对，第一帧可能会和 0 差很远；如果这里按"跳变过大"拒掉，
     * 后面每一帧都会拿这个旧参考去比，等于把定高永久废掉。
     * NaN / 超范围 / 非法气压已经在上面挡掉了，所以采纳是安全的。 */
    if (est->got_sample == 0U)
    {
        est->altitude      = baro_altitude;
        est->alt_ref       = baro_altitude;
        est->velocity      = 0.0f;
        est->accel_bias    = 0.0f;
        est->initialized   = 1U;
        est->got_sample    = 1U;
    }
    else if (fabsf(baro_altitude - est->baro_altitude) > HEIGHT_EST_BARO_MAX_STEP_M)
    {
        /* 20ms 内不可能跳 100m：判为坏样本 */
        est_unlock(est);
        return 1U;
    }
    else
    {
        /* 正常样本 */
    }

    est->baro_altitude = baro_altitude;
    est->baro_pressure = pressure_pa;
    est->baro_dt       = (float)d_us * 1e-6f;
    est->ms_since_baro = 0U;
    est->baro_ok       = 1U;

    est_unlock(est);
    return 0U;
}

/* ============================ 读取 ============================ */

float HeightEstimator_GetHeight(HeightEstimator *est)
{
    float h;

    if (est_lock(est, portMAX_DELAY) == 0U)
    {
        return 0.0f;
    }
    h = est->altitude;
    est_unlock(est);
    return h;
}

float HeightEstimator_GetVelocity(HeightEstimator *est)
{
    float v;

    if (est_lock(est, portMAX_DELAY) == 0U)
    {
        return 0.0f;
    }
    v = est->velocity;
    est_unlock(est);
    return v;
}

uint8_t HeightEstimator_IsBaroOk(HeightEstimator *est)
{
    uint8_t ok;

    if (est_lock(est, portMAX_DELAY) == 0U)
    {
        return 0U;
    }
    ok = est->baro_ok;
    est_unlock(est);
    return ok;
}

/* 一次加锁拷出全部诊断量，供黑匣子 ALT 帧使用（避免 8 次单独取锁） */
void HeightEstimator_GetView(HeightEstimator *est, HeightEstimator_View *v)
{
    if (v == NULL)
    {
        return;
    }

    if (est_lock(est, portMAX_DELAY) == 0U)
    {
        return;
    }

    v->altitude      = est->altitude;
    v->velocity      = est->velocity;
    v->accel_bias    = est->accel_bias;
    v->baro_altitude = est->baro_altitude;
    v->pressure_pa   = est->baro_pressure;
    v->innov         = est->innov;
    v->baro_dt       = est->baro_dt;
    v->alt_ref       = est->alt_ref;
    v->baro_ok       = est->baro_ok;
    v->innov_clamped = est->innov_clamped;
    v->gain_rejected = est->gain_rejected;

    est_unlock(est);
}
