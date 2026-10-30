#include "icm42605.h"
#include "spi.h"
#include "blackbox.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "filter.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;

/* 全局 IMU 数据实例定义在 freertos.c，用于对外提供当前解算周期 */
extern ICM42605_Data imu_data;


#define ICM42605_REG_BANK_SEL  0x76

static uint8_t current_bank = 0xFF;

static SemaphoreHandle_t spi_dma_txrx_sem;

static uint8_t dma_tx_buffer[16] __attribute__((aligned(4)));
static uint8_t dma_rx_buffer[16] __attribute__((aligned(4)));

/* ==================== 加速度计三轴低通 (修复 3.1) ====================
 * 原先只有 vert_accel 过了低通，x/y 是裸数据直喂 Madgwick，
 * 飞行时振动直接调制重力方向，把姿态慢慢"拉偏"。
 * alpha=0.15 @1kHz → 截止约 26Hz，相位滞后约 4ms（对重力参考可忽略）。
 */
/* 加计三轴低通系数。fc = -ln(1-alpha)*fs/(2pi)，1kHz：
 *   alpha 0.15 -> fc ≈ 26Hz（原值，为 0.7~0.9g 的强振动而设）
 *   alpha 0.30 -> fc ≈ 60Hz（现用）
 * 为什么要放宽：这个低通在 10Hz 处带来 21°、20Hz 处 38° 相位滞后，
 * 而 Madgwick 是靠加计修正姿态方向的——加计迟了，估计就被“过时的方向”
 * 往回拽，表现就是“姿态估计跟不上飞机实际角度”。
 * 现在振动已压下来（加计水平 AC-RMS 仅 0.021g），不再需要那么重的滤波。
 * 若姿态开始拖着噪声抖动，往 0.22 回调。 */
#define ACCEL_LPF_ALPHA   1.0f

static LowPassFilter_t accel_lpf_x;
static LowPassFilter_t accel_lpf_y;
static LowPassFilter_t accel_lpf_z;
/* |a| 的低通（~3Hz）：仅用于 accel_confidence 判据，见 UpdateData 里的说明。
 * 振动是高频零均值的，滤掉它之后 |a| 才反映真实的重力/机动大小。 */
static LowPassFilter_t accel_norm_lpf;
static uint8_t accel_lpf_ready = 0;

void ICM42605_InitAccelLPF(void)
{
    LowPass_Init(&accel_lpf_x, ACCEL_LPF_ALPHA);
    LowPass_Init(&accel_lpf_y, ACCEL_LPF_ALPHA);
    LowPass_Init(&accel_lpf_z, ACCEL_LPF_ALPHA);
    /* alpha=0.02 @1kHz -> fc ≈ 3.2Hz，用于"要不要信任重力参考"的判断 */
    LowPass_Init(&accel_norm_lpf, 0.02f);
    accel_lpf_ready = 1;
}

/* ==================== 陀螺零偏在线估计 (修复 2) ====================
 * 上电只标定一次，起飞后 IMU 自发热 + 电机热辐射会让零偏持续变化，
 * yaw 因此线性漂移（无磁力计则永不回正）。
 * 这里在"静止且未解锁"时持续跟踪零偏，用慢速 EMA，运动时冻结。
 */
/* 判定"静止"用的门限。★ 注意这里比较的是**修正后**的陀螺值：
 * 一旦标定残留超过门限，在线跟踪就会被永久禁止，误差再也修不回来
 * （实测出现过残留 6.2 deg/s 卡死的情况，因为门限只有 3.0）。
 * 所以门限放宽到 12 deg/s —— 飞机静止放桌上时不可能有 12 deg/s 的真实转动，
 * 而"标定没减干净"的残留可以靠它自愈。
 * 防误学的另一道保险是下面的加计条件（|a|-1g < 0.03），
 * 真正的机动/搬运会先把这道闸挡住。 */
#define BIAS_LEARN_GYRO_DPS      12.0f   /* 三轴角速度都小于此值才认为静止 */
#define BIAS_LEARN_ACCEL_TOL     0.03f  /* |a|-1g 容差 */
#define BIAS_FAST_SAMPLES        2000U  /* 前 2 秒用均值快速收敛 */
#define BIAS_EMA_WINDOW          2000.0f/* 之后 EMA 等效窗口 ~1.4 分钟 */
#define BIAS_MAX_CORRECTION_DPS  10.0f  /* 单次学习允许的最大修正量，防误学 */

static uint8_t  bias_learning_enabled = 1;
static volatile uint8_t sample_disarmed = 0;
static uint8_t  bias_fast_done = 0;
static uint32_t bias_sample_cnt = 0;
static float    bias_acc_x = 0.0f;
static float    bias_acc_y = 0.0f;
static float    bias_acc_z = 0.0f;
/* 被跟踪的零偏（rad/s），与 data->gyro_offset_* 分开保存，便于慢速收敛 */
static float    bias_track_x = 0.0f;
static float    bias_track_y = 0.0f;
static float    bias_track_z = 0.0f;

/* 重力修正权重：0=完全信任陀螺(重力参考失效)，1=完全信任加速度计。
 * 原先是 UpdateData() 内部的 block-static，为便于黑匣子观测提升为文件级。 */
static float    accel_confidence = 1.0f;

/* 标定期间三轴陀螺标准差之和（deg/s）。静止标定时应 < 0.5；
 * 明显偏大说明这 2 秒里飞机被碰过/在动，标出的零偏不可信。
 * 通过 BB:CAL 命令输出，起飞前检查。 */
static float    calib_gyro_std_dps = 0.0f;

float ICM42605_GetCalibStd(void)
{
    return calib_gyro_std_dps;
}

/* ==================================================================
 *  校准参数化（档 2：只做零偏）
 * ================================================================== */
static volatile uint8_t cal_req   = 0U;      /* 1 = 有标定请求          */
static volatile uint8_t cal_state = 0U;      /* 见 icm42605.h 的注释    */
static float cal_dps[3] = {0.0f, 0.0f, 0.0f};   /* 最近一次标定的陀螺零偏 */
static float cal_g[3]   = {0.0f, 0.0f, 0.0f};   /* 最近一次标定的加计零偏 */

/* 用存过的标定结果直接起机（不测量、不要求静止）。 */
void ICM42605_SetCalibration(ICM42605_Data *data,
                             float gx_dps, float gy_dps, float gz_dps,
                             float ax_g, float ay_g, float az_g)
{
    data->gyro_offset_x = gx_dps * DEG_TO_RAD;
    data->gyro_offset_y = gy_dps * DEG_TO_RAD;
    data->gyro_offset_z = gz_dps * DEG_TO_RAD;
    data->accel_offset_x = ax_g;
    data->accel_offset_y = ay_g;
    data->accel_offset_z = az_g;
    /* 档 2 不做刻度标定：保持 1.0（六面标定的路径在 SixSideCalibrate 里） */
    data->accel_scale_x = 1.0f;
    data->accel_scale_y = 1.0f;
    data->accel_scale_z = 1.0f;

    cal_dps[0] = gx_dps; cal_dps[1] = gy_dps; cal_dps[2] = gz_dps;
    cal_g[0]   = ax_g;   cal_g[1]   = ay_g;   cal_g[2]   = az_g;

    /* 在线零偏跟踪以存下来的值为初值，避免它再从 0 慢慢爬 */
    ICM42605_ResetBiasTracker(data);
}

void ICM42605_RequestCalibration(void)
{
    cal_state = 1U;
    cal_req   = 1U;
}

uint8_t ICM42605_CalibrationState(void)
{
    return cal_state;
}

void ICM42605_GetCalibration(float *gx_dps, float *gy_dps, float *gz_dps,
                             float *ax_g, float *ay_g, float *az_g)
{
    if (gx_dps != NULL) { *gx_dps = cal_dps[0]; }
    if (gy_dps != NULL) { *gy_dps = cal_dps[1]; }
    if (gz_dps != NULL) { *gz_dps = cal_dps[2]; }
    if (ax_g   != NULL) { *ax_g   = cal_g[0];   }
    if (ay_g   != NULL) { *ay_g   = cal_g[1];   }
    if (az_g   != NULL) { *az_g   = cal_g[2];   }
}

/* 由 IMU 任务每周期调用：有请求且未解锁时，就地做一次 2 秒标定。
 * 返回 1 表示这一拍刚做完标定（调用方照常继续采样即可）。 */
uint8_t ICM42605_ServiceCalibration(ICM42605_Data *data, uint8_t armed)
{
    if (cal_req == 0U)
    {
        return 0U;
    }

    /* 解锁中拒绝：标定要阻塞 2 秒，绝不能在飞的时候做 */
    if (armed != 0U)
    {
        cal_req   = 0U;
        cal_state = 4U;
        return 0U;
    }

    cal_req   = 0U;
    cal_state = 2U;

    ICM42605_Calibrate(data, 1000);      /* 内部 osDelay(2)，约 2 秒；
                                          * 结果和标定质量由 Calibrate 自己记下 */

    cal_state = 3U;
    return 1U;
}

float ICM42605_GetAccelConfidence(void)
{
    return accel_confidence;
}

/* 当前姿态解算周期只需从 imu_data.sample_period 读取，不再另存一份，
 * 避免两处状态不同步。 */
float ICM42605_GetSamplePeriod(void)
{
    return imu_data.sample_period;
}

void ICM42605_SetArmed(uint8_t armed)
{
    sample_disarmed = (armed == 0) ? 1 : 0;
}

void ICM42605_ResetBiasTracker(ICM42605_Data *data)
{
    bias_fast_done  = 0;
    bias_sample_cnt = 0;
    bias_acc_x = 0.0f;
    bias_acc_y = 0.0f;
    bias_acc_z = 0.0f;

    /* 用已有的标定值做初值，学习过程从这里平滑过渡 */
    /* ★ bias_track_* 的单位是 deg/s，而 gyro_offset_* 是 rad/s，必须换算。
     * 原来直接赋值，单位差了 57.3 倍的因子，会让第一轮快速学习
     * 从错误初值开始，容易把标定值冲掉。 */
    bias_track_x = (data != NULL) ? (data->gyro_offset_x * RAD_TO_DEG) : 0.0f;
    bias_track_y = (data != NULL) ? (data->gyro_offset_y * RAD_TO_DEG) : 0.0f;
    bias_track_z = (data != NULL) ? (data->gyro_offset_z * RAD_TO_DEG) : 0.0f;
}

/* HAL 的 SPI 完成回调在整工程里**只能有一处定义**（否则 L6200E 重复符号）。
 * 目前只有 SPI1(IMU, DMA 模式) 需要它；SPI3 的黑匣子改用阻塞式传输，不再走中断。
 * 新增走中断/DMA 的 SPI 外设时，在这里加分支，不要另建同名回调。 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(spi_dma_txrx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
    else
    {
        /* 其它 SPI：暂无使用者 */
    }
}


static void ICM42605_SelectBank(uint8_t bank)
{
    if (current_bank != bank)
    {
        uint8_t tx_data[2];
        tx_data[0] = ICM42605_REG_BANK_SEL;
        tx_data[1] = bank;
        
        ICM42605_CS_Low();
        HAL_SPI_Transmit(&hspi1, tx_data, 2, ICM42605_SPI_TIMEOUT);
        ICM42605_CS_High();
        
        current_bank = bank;
    }
}

void ICM42605_CS_Low(void)
{
    HAL_GPIO_WritePin(ICM42605_CS_PORT, ICM42605_CS_PIN, GPIO_PIN_RESET);
}

void ICM42605_CS_High(void)
{
    HAL_GPIO_WritePin(ICM42605_CS_PORT, ICM42605_CS_PIN, GPIO_PIN_SET);
}

uint8_t ICM42605_ReadReg(uint8_t reg)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[2];
    
    tx_buf[0] = reg | ICM42605_READ_FLAG;
    tx_buf[1] = 0x00;
    
    ICM42605_CS_Low();
    HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2, ICM42605_SPI_TIMEOUT);
    ICM42605_CS_High();
    
    return rx_buf[1];
}

void ICM42605_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t tx_data[2];
    
    tx_data[0] = reg;
    tx_data[1] = data;
    
    ICM42605_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx_data, 2, ICM42605_SPI_TIMEOUT);
    ICM42605_CS_High();
}

void ICM42605_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    dma_tx_buffer[0] = reg | ICM42605_READ_FLAG;
    memset(dma_rx_buffer, 0, len + 1);

    ICM42605_CS_Low();
    
    HAL_SPI_TransmitReceive_DMA(&hspi1, dma_tx_buffer, dma_rx_buffer, len + 1);
    
    if (xSemaphoreTake(spi_dma_txrx_sem, pdMS_TO_TICKS(ICM42605_SPI_TIMEOUT)) != pdTRUE)
    {
        HAL_SPI_DMAStop(&hspi1);
    }
    
    ICM42605_CS_High();
    
    memcpy(buf, &dma_rx_buffer[1], len);
}


uint8_t ICM42605_Init(void)
{
    current_bank = 0xFF;
    
    spi_dma_txrx_sem = xSemaphoreCreateBinary();
    
    if (spi_dma_txrx_sem == NULL)
    {
        return 0;
    }
    
    ICM42605_CS_High();
    osDelay(50);
    
    ICM42605_SelectBank(0);
    osDelay(10);


    uint8_t who_am_i = ICM42605_ReadReg(ICM42605_REG_WHO_AM_I);
//    if (who_am_i != ICM42605_WHO_AM_I_VALUE)
//    {
//        return 0;
//    }
    
    ICM42605_WriteReg(ICM42605_REG_PWR_MGMT0, 0x0F);
    osDelay(10);
    
    ICM42605_WriteReg(ICM42605_REG_DEVICE_CONFIG, 0x01);
    osDelay(1);
    
    ICM42605_WriteReg(ICM42605_REG_INTF_CONFIG0, 0x58);
    osDelay(1);
    
    ICM42605_WriteReg(ICM42605_REG_GYRO_CONFIG0, 
                      (ICM42605_GYRO_FS_2000DPS << 5) | ICM42605_GYRO_ODR_1KHZ);
    osDelay(1);
    
    ICM42605_WriteReg(ICM42605_REG_ACCEL_CONFIG0, 
                      (ICM42605_ACCEL_FS_16G << 5) | ICM42605_ACCEL_ODR_1KHZ);
    osDelay(1);
    
    /* ── UI 低通带宽（GYRO_ACCEL_CONFIG0 = 0x52）──
     * 原值 0x00：高低半字节都是 0，即 BW = ODR/2 = 500Hz —— 等于不做任何滤波。
     *
     * 实测频谱（空心杯有刷电机 + 二叶桨，ODR=1kHz）：
     *   gyro_x 主峰 310.9Hz，>100Hz 占 85% 能量，RMS 34 deg/s
     *   gyro_y 主峰 320.0Hz，>100Hz 占 81% 能量，RMS 42 deg/s
     * 即陀螺 80~93% 的能量是这个 310~320Hz 的电机/桨振动。
     *
     * BW=500Hz 时这部分原封不动进入采样，之后靠软件陷波压不干净，
     * 而且 500Hz 以上的噪声（有刷电机换向尖峰是宽带的）会**混叠**到基带里，
     * 采样后无法再分离 —— 所以必须在这一层先压。
     *
     * 索引 3 => BW = max(82Hz, ODR/8) = 125Hz @1kHz ODR，对 310Hz 约 -18dB。
     * 加计同样设 125Hz（后面还有 26Hz 的 ACCEL_LPF，延时可接受）。
     *
     * 若手感变钝（内环相位滞后感变强），把索引降到 2（200Hz）折中；
     * 若振动仍大，升到 4（100Hz）进一步压。
     */
    ICM42605_WriteReg(ICM42605_REG_GYRO_ACCEL_CONFIG0,
                      (uint8_t)((ICM42605_ACCEL_UI_BW_IDX << 4) | ICM42605_GYRO_UI_BW_IDX));
    osDelay(1);
    
    ICM42605_WriteReg(ICM42605_REG_PWR_MGMT0, 0x0F);
    osDelay(50);
    
		
    return 1;
}

void ICM42605_ReadRawData(ICM42605_Data *data)
{
    uint8_t buf[14];
    
    ICM42605_ReadMultiRegs(0x1D, buf, 14);
    
    data->temp_raw    = (int16_t)((buf[0] << 8) | buf[1]);
    data->accel_x_raw = (int16_t)((buf[2] << 8) | buf[3]);
    data->accel_y_raw = (int16_t)((buf[4] << 8) | buf[5]);
    data->accel_z_raw = (int16_t)((buf[6] << 8) | buf[7]);
    data->gyro_x_raw  = (int16_t)((buf[8] << 8) | buf[9]);
    data->gyro_y_raw  = (int16_t)((buf[10] << 8) | buf[11]);
    data->gyro_z_raw  = (int16_t)((buf[12] << 8) | buf[13]);
}

void ICM42605_UpdateData(ICM42605_Data *data)
{
    ICM42605_ReadRawData(data);

    data->gyro_x = ((float)data->gyro_x_raw / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_y = ((float)data->gyro_y_raw / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_z = ((float)data->gyro_z_raw / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    
    data->gyro_x -= data->gyro_offset_x;
    data->gyro_y -= data->gyro_offset_y;
    data->gyro_z -= data->gyro_offset_z;
	
    float accel_x_g = (float)data->accel_x_raw / ACCEL_SENSITIVITY_16G;
    float accel_y_g = (float)data->accel_y_raw / ACCEL_SENSITIVITY_16G;
    float accel_z_g = (float)data->accel_z_raw / ACCEL_SENSITIVITY_16G;

    /* ---- 修复 3.1：三轴低通在去偏置/刻度之前做，避免偏置被反复滤波拖尾 ---- */
    if (accel_lpf_ready)
    {
        accel_x_g = LowPass_Update(&accel_lpf_x, accel_x_g);
        accel_y_g = LowPass_Update(&accel_lpf_y, accel_y_g);
        accel_z_g = LowPass_Update(&accel_lpf_z, accel_z_g);
    }

    data->accel_x = (accel_x_g - data->accel_offset_x) / data->accel_scale_x;
    data->accel_y = (accel_y_g - data->accel_offset_y) / data->accel_scale_y;
    data->accel_z = (accel_z_g - data->accel_offset_z) / data->accel_scale_z;

    
    data->temp = ((float)data->temp_raw / 132.48f) + 25.0f;
    
    float accel_norm = sqrtf(data->accel_x * data->accel_x + 
                             data->accel_y * data->accel_y + 
                             data->accel_z * data->accel_z);

    /* ---- 修复 2：静止且未解锁时在线跟踪陀螺零偏 ---- */
    float gyro_dps_max = 0.0f;
    float gx_tmp = data->gyro_x * RAD_TO_DEG;
    float gy_tmp = data->gyro_y * RAD_TO_DEG;
    float gz_tmp = data->gyro_z * RAD_TO_DEG;
    gyro_dps_max = fabsf(gx_tmp);
    if (fabsf(gy_tmp) > gyro_dps_max) gyro_dps_max = fabsf(gy_tmp);
    if (fabsf(gz_tmp) > gyro_dps_max) gyro_dps_max = fabsf(gz_tmp);

    if (bias_learning_enabled && (sample_disarmed == 1) &&
        (gyro_dps_max < BIAS_LEARN_GYRO_DPS) &&
        (fabsf(accel_norm - 1.0f) < BIAS_LEARN_ACCEL_TOL))
    {
        /* ★ 必须累加**原始**值（已修正值 + 当前零偏），不能累加已修正值。
         * 原来累加 gx_tmp（已减过零偏），算出的是"残余"，然后执行
         *     data->gyro_offset_x = 残余
         * 这等于用残余**覆盖**掉标定值，标定基准当场丢失。
         * 正确关系是 offset = 原始均值，所以这里先把零偏加回去。 */
        bias_acc_x += gx_tmp + data->gyro_offset_x * RAD_TO_DEG;
        bias_acc_y += gy_tmp + data->gyro_offset_y * RAD_TO_DEG;
        bias_acc_z += gz_tmp + data->gyro_offset_z * RAD_TO_DEG;
        bias_sample_cnt++;

        if (bias_sample_cnt >= BIAS_FAST_SAMPLES)
        {
            float inv = 1.0f / (float)bias_sample_cnt;
            float learned_x = bias_acc_x * inv;
            float learned_y = bias_acc_y * inv;
            float learned_z = bias_acc_z * inv;

            if (!bias_fast_done)
            {
                /* 快速阶段：直接采用均值（约 2 秒收敛） */
                bias_track_x = learned_x;
                bias_track_y = learned_y;
                bias_track_z = learned_z;
                bias_fast_done = 1;
            }
            else
            {
                /* 慢速阶段：EMA 持续跟踪温漂，并限制单步修正量防误学 */
                float alpha  = (float)bias_sample_cnt / (BIAS_EMA_WINDOW + (float)bias_sample_cnt);
                float step_x = alpha * (learned_x - bias_track_x);
                float step_y = alpha * (learned_y - bias_track_y);
                float step_z = alpha * (learned_z - bias_track_z);
                float step_max = BIAS_MAX_CORRECTION_DPS;

                if (step_x >  step_max) step_x =  step_max;
                if (step_x < -step_max) step_x = -step_max;
                if (step_y >  step_max) step_y =  step_max;
                if (step_y < -step_max) step_y = -step_max;
                if (step_z >  step_max) step_z =  step_max;
                if (step_z < -step_max) step_z = -step_max;

                bias_track_x += step_x;
                bias_track_y += step_y;
                bias_track_z += step_z;
            }

            /* 回写到唯一被使用的那份零偏，下一拍立即生效 */
            data->gyro_offset_x = bias_track_x * DEG_TO_RAD;
            data->gyro_offset_y = bias_track_y * DEG_TO_RAD;
            data->gyro_offset_z = bias_track_z * DEG_TO_RAD;

            bias_acc_x = 0.0f;
            bias_acc_y = 0.0f;
            bias_acc_z = 0.0f;
            bias_sample_cnt = 0;
        }
    }
    else
    {
        /* 运动中：清空累积，避免把机动/振动当成零偏 */
        bias_acc_x = 0.0f;
        bias_acc_y = 0.0f;
        bias_acc_z = 0.0f;
        bias_sample_cnt = 0;
    }

    if (accel_norm < 0.1f || accel_norm > 50.0f)
    {
        return;
    }
    
    /* ---- 修复 4(配套)：自适应权重，加速度计不可信时关掉重力修正 ----
     * Madgwick 的前提是加速度矢量方向等于重力方向。机动/强振动时该前提不成立，
     * 此时继续用重力纠偏会把姿态拉向错误方向（这就是"越飞越偏"的推手）。
     *
     * ★ 关键修正：判据必须用**低通后的 |a|**，不能用瞬时值。
     *   原因：这个机架（空心杯有刷 + 二叶桨）的振动 AC-RMS 达 0.7~0.9g，
     *   瞬时 |a| 被振动抬到 1.56g，accel_dev≈0.66 恒定超过 0.30 阈值，
     *   结果 accel_confidence 长期被压到 0（实测占 95.7% 时间），
     *   重力修正几乎全程关闭 —— 姿态退化为纯陀螺积分，
     *   而陀螺又有 310Hz 振动整流产生的零偏漂移，姿态必然越飞越歪。
     *
     *   振动的本质是高频、零均值，低通后幅值会回到真实重力大小；
     *   真正需要关掉重力修正的场合（持续机动）是低频的，低通不影响判断。
     */
    {
        /* ~3Hz 低通（alpha=0.02 @1kHz）：滤掉振动，保留真实的机动变化 */
        float accel_norm_f = LowPass_Update(&accel_norm_lpf, accel_norm);
        float accel_dev = fabsf(accel_norm_f - 1.0f);
        float conf_target;

        if (accel_dev <= 0.10f)       conf_target = 1.0f;
        else if (accel_dev >= 0.30f)  conf_target = 0.0f;
        else                          conf_target = (0.30f - accel_dev) / 0.20f;

        /* 置信度平滑过渡，避免 beta 突变引起姿态抖动。
         * accel_confidence 为文件级变量，黑匣子会周期性把它记下来。 */
        accel_confidence += 0.005f * (conf_target - accel_confidence);

        Vector3f gyro = {data->gyro_x, data->gyro_y, data->gyro_z};
        Vector3f accel = {data->accel_x, data->accel_y, data->accel_z};

        MadgwickAHRSUpdate(&data->quat, &gyro, &accel, data->sample_period,
                           data->beta * accel_confidence);
//		MahonyAHRSUpdate(&data->quat, &gyro, &accel,data->sample_period, 1.0f, 0.0f);		
		data->vert_accel = ICM42605_Calculate_Vertical_Acceleration_Quaternion(&accel,&data->quat,data);
        Quaternion_ToEulerAngles(&data->quat, &data->angle);
    }
    
    data->tilt_total = ICM42605_CalculateTotalTilt(data);
}

void ICM42605_Calibrate(ICM42605_Data *data, uint16_t samples)
{
    int32_t gyro_x_sum = 0;
    int32_t gyro_y_sum = 0;
    int32_t gyro_z_sum = 0;

    int32_t accel_x_sum = 0;
    int32_t accel_y_sum = 0;
    int32_t accel_z_sum = 0;
    /* 陀螺平方和（deg/s）²，用于算标定期间的标准差 */
    float   gx_sq_sum = 0.0f;
    float   gy_sq_sum = 0.0f;
    float   gz_sq_sum = 0.0f;
    
    data->gyro_offset_x = 0.0f;
    data->gyro_offset_y = 0.0f;
    data->gyro_offset_z = 0.0f;
    data->accel_offset_x = 0.0f;
    data->accel_offset_y = 0.0f;
    data->accel_offset_z = 0.0f;
    data->accel_scale_x = 1.0f;
    data->accel_scale_y = 1.0f;
    data->accel_scale_z = 1.0f;
    
    for (uint16_t i = 0; i < samples; i++)
    {
        ICM42605_ReadRawData(data);
        gyro_x_sum += data->gyro_x_raw;
        gyro_y_sum += data->gyro_y_raw;
        gyro_z_sum += data->gyro_z_raw;
        accel_x_sum += data->accel_x_raw;
        accel_y_sum += data->accel_y_raw;
        accel_z_sum += data->accel_z_raw;

        /* 同时累加平方，用于判断"标定期间飞机是否被扰动"。
         * 陀螺在静止时读数应几乎不变，标准差 >0.5 deg/s 基本可以判定
         * 这 2 秒里飞机被碰过/在动 —— 那样标出来的零偏是错的，
         * 会直接表现为"恒定方向的偏飞"（实测出现过 6.3 deg/s 的假零偏）。 */
        {
            float gx = (float)data->gyro_x_raw / GYRO_SENSITIVITY_2000DPS;
            float gy = (float)data->gyro_y_raw / GYRO_SENSITIVITY_2000DPS;
            float gz = (float)data->gyro_z_raw / GYRO_SENSITIVITY_2000DPS;
            gx_sq_sum += gx * gx;
            gy_sq_sum += gy * gy;
            gz_sq_sum += gz * gz;
        }

        osDelay(2);
    }

    data->gyro_offset_x = ((float)gyro_x_sum / samples / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_offset_y = ((float)gyro_y_sum / samples / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_offset_z = ((float)gyro_z_sum / samples / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->accel_offset_x = (float)accel_x_sum / samples / ACCEL_SENSITIVITY_16G;
    data->accel_offset_y = (float)accel_y_sum / samples / ACCEL_SENSITIVITY_16G;
    data->accel_offset_z = (float)(accel_z_sum / samples / ACCEL_SENSITIVITY_16G - 1.0f);

    /* 标定质量：三轴标准差之和（deg/s）。静止时应远小于 0.5。
     * 供 BB:CAL 命令输出，起飞前就能看出这次标定靠不靠谱。 */
    {
        float inv = 1.0f / (float)samples;
        float mx = data->gyro_offset_x * RAD_TO_DEG;
        float my = data->gyro_offset_y * RAD_TO_DEG;
        float mz = data->gyro_offset_z * RAD_TO_DEG;
        float vx = gx_sq_sum * inv - mx * mx;
        float vy = gy_sq_sum * inv - my * my;
        float vz = gz_sq_sum * inv - mz * mz;
        if (vx < 0.0f) { vx = 0.0f; }
        if (vy < 0.0f) { vy = 0.0f; }
        if (vz < 0.0f) { vz = 0.0f; }
        calib_gyro_std_dps = sqrtf(vx) + sqrtf(vy) + sqrtf(vz);
    }
		
    /* ★ 原本这里用一组**硬编码常量**覆盖了上面刚算出的加计零偏：
     *     accel_offset_x = -0.002730f;  accel_offset_y = -0.003430f; ...
     * 那些常量来自某一次旧标定，不会随板子/安装变化而更新。
     *
     * 实测后果：四次飞行的 accel_x 均值恒为 -0.021 g，
     * 是硬编码值(-0.00273)的 7.7 倍。若这部分属于零偏而非真实倾角，
     * 姿态估计的“水平”就会偏 atan(0.021/1.0) ≈ 1.2°，
     * 控制器会带着这个倾角飞 -> 恒定方向偏飞，
     * **与 beta 无关**（这正是 beta 加到 1.0 仍然偏的原因）。
     *
     * 现在改为使用本次上电实测的零偏（上面已算好）。
     * 前提：上电时飞机放平且不动，与陀螺标定的要求一致。
     * 刻度因子保留原硬编码（都接近 1，影响 <2%）。 */
    /* ★ 刻度因子**归一**（不再用硬编码常量）。
     *
     * 原来这里覆盖成 0.988330 / 0.995882 / 0.983333，来自某次六面标定。
     * 但那条路径 `ICM42605_SixSideCalibrate` 在 freertos.c 里是**注释掉的**，
     * 永远不会重新测量，于是它成了一个不可维护的魔数。
     *
     * 而且它会让诊断报告长期误报：
     *   应用方式是 accel = (raw - offset) / scale，除以 <1 的 scale 等于放大。
     *   上电标定令水平时 (raw-offset)=1.0，再 /0.983333 = **1.017 g**，
     *   于是 "加速度计合成幅值偏离1g" 这条判读永远丢不掉，
     *   实测 f1=1.0313、f2=1.0266 与 1.017 完全对得上。
     *
     * 影响量化：非等比 scale 对姿态角的影响仅 <=0.13°（Madgwick 会归一化，
     * 等比部分自动抵消），所以归一对控制质量无害；但能让诊断数字变干净。
     *
     * 若以后真要标定刻度：解开 freertos.c 里的 ICM42605_SixSideCalibrate，
     * 它会把真实的 scale 写回来（上面 line 493-495 已经先置 1.0f）。 */

    /* 把这次的结果记下来，供 CFG:CAL 和上位机读取。
     * 放在这里而不是只放在 ServiceCalibration 里：上电那次自标定也能被看到。 */
    cal_dps[0] = data->gyro_offset_x * RAD_TO_DEG;
    cal_dps[1] = data->gyro_offset_y * RAD_TO_DEG;
    cal_dps[2] = data->gyro_offset_z * RAD_TO_DEG;
    cal_g[0]   = data->accel_offset_x;
    cal_g[1]   = data->accel_offset_y;
    cal_g[2]   = data->accel_offset_z;

    /* 以本次标定结果为初值启动在线零偏跟踪（修复 2） */
    ICM42605_ResetBiasTracker(data);
}

void ICM42605_SixSideCalibrate(ICM42605_Data *data, uint16_t samples_per_face)
{
    float face_data[6][3] = {0};

    printf("\r\n[Calib] === Six-Sided Accelerometer Calibration ===\r\n");
    printf("[Calib] Performing initial gyro calibration...\r\n");
    ICM42605_Calibrate(data, samples_per_face);

    const char *face_names[6] = {
        "Z-axis UP (normal level position, IC facing up)",
        "Z-axis DOWN (inverted, IC facing down)",
        "X-axis UP (right edge down, X arrow pointing up)",
        "X-axis DOWN (left edge down, X arrow pointing down)",
        "Y-axis UP (front edge down, Y arrow pointing up)",
        "Y-axis DOWN (rear edge down, Y arrow pointing down)"
    };

    for (int face = 0; face < 6; face++)
    {
        printf("\r\n[Calib] Face %d/6: %s\r\n", face + 1, face_names[face]);
        printf("[Calib] Hold steady... ");

        for (int delay = 12; delay > 0; delay--)
        {
            printf("%d... ", delay);
            osDelay(1000);
        }
        printf("\r\n");

        float sum_x = 0, sum_y = 0, sum_z = 0;
        for (uint16_t i = 0; i < samples_per_face; i++)
        {
            ICM42605_ReadRawData(data);
            sum_x += (float)data->accel_x_raw;
            sum_y += (float)data->accel_y_raw;
            sum_z += (float)data->accel_z_raw;
            osDelay(3);
        }

        face_data[face][0] = sum_x / samples_per_face;
        face_data[face][1] = sum_y / samples_per_face;
        face_data[face][2] = sum_z / samples_per_face;

        printf("[Calib]   Raw avg: X=%.1f  Y=%.1f  Z=%.1f\r\n",
               face_data[face][0], face_data[face][1], face_data[face][2]);
    }

    float x_plus_g  = face_data[2][0] / ACCEL_SENSITIVITY_16G;
    float x_minus_g = face_data[3][0] / ACCEL_SENSITIVITY_16G;
    float y_plus_g  = face_data[4][1] / ACCEL_SENSITIVITY_16G;
    float y_minus_g = face_data[5][1] / ACCEL_SENSITIVITY_16G;
    float z_plus_g  = face_data[0][2] / ACCEL_SENSITIVITY_16G;
    float z_minus_g = face_data[1][2] / ACCEL_SENSITIVITY_16G;

    data->accel_offset_x = (x_plus_g + x_minus_g) / 2.0f;
    data->accel_offset_y = (y_plus_g + y_minus_g) / 2.0f;
    data->accel_offset_z = (z_plus_g + z_minus_g) / 2.0f;

    data->accel_scale_x = (x_plus_g - x_minus_g) / 2.0f;
    data->accel_scale_y = (y_plus_g - y_minus_g) / 2.0f;
    data->accel_scale_z = (z_plus_g - z_minus_g) / 2.0f;

    printf("\r\n[Calib] === Calibration Complete ===\r\n");
    printf("[Calib] Accel offsets (g): X=%+.6f  Y=%+.6f  Z=%+.6f\r\n",
           data->accel_offset_x, data->accel_offset_y, data->accel_offset_z);
    printf("[Calib] Accel scales:      X=%+.6f  Y=%+.6f  Z=%+.6f\r\n",
           data->accel_scale_x, data->accel_scale_y, data->accel_scale_z);

    float x_err = fabsf(data->accel_scale_x - 1.0f) * 100.0f;
    float y_err = fabsf(data->accel_scale_y - 1.0f) * 100.0f;
    float z_err = fabsf(data->accel_scale_z - 1.0f) * 100.0f;
    printf("[Calib] Scale error: X=%.2f%%  Y=%.2f%%  Z=%.2f%%\r\n", x_err, y_err, z_err);
}

void Quaternion_Init(Quaternion *q)
{
    q->q0 = 1.0f;
    q->q1 = 0.0f;
    q->q2 = 0.0f;
    q->q3 = 0.0f;
}

void Quaternion_Normalize(Quaternion *q)
{
    float norm = sqrtf(q->q0 * q->q0 + q->q1 * q->q1 + q->q2 * q->q2 + q->q3 * q->q3);
    
    if (norm > 0.0001f)
    {
        float inv_norm = 1.0f / norm;
        q->q0 *= inv_norm;
        q->q1 *= inv_norm;
        q->q2 *= inv_norm;
        q->q3 *= inv_norm;
    }
}

void Quaternion_ToEulerAngles(Quaternion *q, EulerAngles *angle)
{
    float sinr_cosp = 2.0f * (q->q0 * q->q1 + q->q2 * q->q3);
    float cosr_cosp = 1.0f - 2.0f * (q->q1 * q->q1 + q->q2 * q->q2);
    angle->pitch = -atan2f(sinr_cosp, cosr_cosp) * RAD_TO_DEG;
    
    float sinp = 2.0f * (q->q0 * q->q2 - q->q3 * q->q1);
    if (fabsf(sinp) >= 1.0f)
    {
        angle->roll = copysignf(90.0f, sinp);
    }
    else
    {
        angle->roll = asinf(sinp) * RAD_TO_DEG;
    }
    
    float siny_cosp = 2.0f * (q->q0 * q->q3 + q->q1 * q->q2);
    float cosy_cosp = 1.0f - 2.0f * (q->q2 * q->q2 + q->q3 * q->q3);
    angle->yaw = -atan2f(siny_cosp, cosy_cosp) * RAD_TO_DEG;
}

void MadgwickAHRSUpdate(Quaternion *q, Vector3f *gyro, Vector3f *accel, float dt, float beta)
{
    float q0 = q->q0;
    float q1 = q->q1;
    float q2 = q->q2;
    float q3 = q->q3;
    
    float ax = accel->x;
    float ay = accel->y;
    float az = accel->z;
    
    float gx = gyro->x;
    float gy = gyro->y;
    float gz = gyro->z;
    
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    
    if (norm < 0.0001f)
    {
        return;
    }
    
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
    
    norm = sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);

    if (norm < 0.0001f)
    {
        /* ★ 两个重力方向完美对齐时梯度为 0。
         * 原来这里直接 return，**连陀螺积分都跳过了**，
         * 一旦真的落到这个状态，姿态会永久冻结在当前值。
         * 正确做法：只去掉梯度修正项，陀螺积分照常进行。 */
        float d0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        float d1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        float d2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        float d3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        q0 += d0 * dt;
        q1 += d1 * dt;
        q2 += d2 * dt;
        q3 += d3 * dt;

        norm = sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        if (norm > 0.0001f)
        {
            norm = 1.0f / norm;
            q->q0 = q0 * norm;
            q->q1 = q1 * norm;
            q->q2 = q2 * norm;
            q->q3 = q3 * norm;
        }
        return;
    }
    
    norm = 1.0f / norm;
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;
    
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    
    q->q0 = q0;
    q->q1 = q1;
    q->q2 = q2;
    q->q3 = q3;
    
    Quaternion_Normalize(q);
}

void MahonyAHRSUpdate(Quaternion *q, Vector3f *gyro, Vector3f *accel, float dt, float kp, float ki)
{
    static float integralFBx = 0.0f;
    static float integralFBy = 0.0f;
    static float integralFBz = 0.0f;
    
    float q0 = q->q0;
    float q1 = q->q1;
    float q2 = q->q2;
    float q3 = q->q3;
    
    float ax = accel->x;
    float ay = accel->y;
    float az = accel->z;
    
    float gx = gyro->x;
    float gy = gyro->y;
    float gz = gyro->z;
    
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    
    if (norm < 0.0001f)
    {
        return;
    }
    
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;
    
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;
    
    float vx = 2.0f * (q1 * q3 - q0 * q2);
    float vy = 2.0f * (q0 * q1 + q2 * q3);
    float vz = q0q0 - q1q1 - q2q2 + q3q3;
    
    float ex = ay * vz - az * vy;
    float ey = az * vx - ax * vz;
    float ez = ax * vy - ay * vx;
    
    if (ki > 0.0f)
    {
        integralFBx += ex * dt;
        integralFBy += ey * dt;
        integralFBz += ez * dt;
        gx += kp * ex + ki * integralFBx;
        gy += kp * ey + ki * integralFBy;
        gz += kp * ez + ki * integralFBz;
    }
    else
    {
        gx += kp * ex;
        gy += kp * ey;
        gz += kp * ez;
    }
    
    float qDot0 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    float qDot1 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    float qDot2 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    float qDot3 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;
    
    q->q0 = q0;
    q->q1 = q1;
    q->q2 = q2;
    q->q3 = q3;
    
    Quaternion_Normalize(q);
}

float ICM42605_CalculateTotalTilt(ICM42605_Data *data)
{
    float roll_rad = data->angle.roll * DEG_TO_RAD;
    float pitch_rad = data->angle.pitch * DEG_TO_RAD;
    
    float cos_roll = cosf(roll_rad);
    float cos_pitch = cosf(pitch_rad);
    
    float cos_tilt = cos_roll * cos_pitch;
    float tilt_rad = acosf(cos_tilt);
    
    return tilt_rad * RAD_TO_DEG;
}

float ICM42605_Calculate_Vertical_Acceleration_Quaternion(Vector3f* accel, Quaternion* q,ICM42605_Data *data)
{
    float q0 = q->q0, q1 = q->q1, q2 = q->q2, q3 = q->q3;
    float ax = accel->x, ay = accel->y, az = accel->z;
    
    float a_down_earth = 
        2.0f * (q1*q3 - q0*q2) * ax +
        2.0f * (q0*q1 + q2*q3) * ay +
        (q0*q0 - q1*q1 - q2*q2 + q3*q3) * az;
    
    float a_vertical_motion = a_down_earth - 1.0f;
    
    return a_vertical_motion;
}


