#ifndef __ICM42605_H__
#define __ICM42605_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <math.h>

#define ICM42605_CS_PIN GPIO_PIN_4
#define ICM42605_CS_PORT GPIOA

#define ICM42605_SPI_TIMEOUT 100

#define ICM42605_REG_DEVICE_CONFIG      0x11
#define ICM42605_REG_DRIVE_CONFIG       0x13
#define ICM42605_REG_INT_CONFIG         0x14
#define ICM42605_REG_FIFO_CONFIG        0x16
#define ICM42605_REG_TEMP_DATA1         0x1D
#define ICM42605_REG_TEMP_DATA0         0x1E
#define ICM42605_REG_ACCEL_DATA_X1      0x1F
#define ICM42605_REG_ACCEL_DATA_X0      0x20
#define ICM42605_REG_ACCEL_DATA_Y1      0x21
#define ICM42605_REG_ACCEL_DATA_Y0      0x22
#define ICM42605_REG_ACCEL_DATA_Z1      0x23
#define ICM42605_REG_ACCEL_DATA_Z0      0x24
#define ICM42605_REG_GYRO_DATA_X1       0x25
#define ICM42605_REG_GYRO_DATA_X0       0x26
#define ICM42605_REG_GYRO_DATA_Y1       0x27
#define ICM42605_REG_GYRO_DATA_Y0       0x28
#define ICM42605_REG_GYRO_DATA_Z1       0x29
#define ICM42605_REG_GYRO_DATA_Z0       0x2A
#define ICM42605_REG_TMST_FSYNCH        0x2B
#define ICM42605_REG_TMST_FSYNCL        0x2C
#define ICM42605_REG_INT_STATUS         0x2D
#define ICM42605_REG_FIFO_COUNTH        0x2E
#define ICM42605_REG_FIFO_COUNTL        0x2F
#define ICM42605_REG_FIFO_DATA          0x30
#define ICM42605_REG_APEX_DATA0         0x31
#define ICM42605_REG_APEX_DATA1         0x32
#define ICM42605_REG_APEX_DATA2         0x33
#define ICM42605_REG_APEX_DATA3         0x34
#define ICM42605_REG_APEX_DATA4         0x35
#define ICM42605_REG_APEX_DATA5         0x36
#define ICM42605_REG_INT_STATUS2        0x37
#define ICM42605_REG_INT_STATUS3        0x38
#define ICM42605_REG_SIGNAL_PATH_RESET  0x4B
#define ICM42605_REG_INTF_CONFIG0       0x4C
#define ICM42605_REG_INTF_CONFIG1       0x4D
#define ICM42605_REG_PWR_MGMT0          0x4E
#define ICM42605_REG_GYRO_CONFIG0       0x4F
#define ICM42605_REG_ACCEL_CONFIG0      0x50
#define ICM42605_REG_GYRO_CONFIG1       0x51
#define ICM42605_REG_GYRO_ACCEL_CONFIG0 0x52

/* UI 低通带宽索引（写进 GYRO_ACCEL_CONFIG0 的高低半字节）。
 * 取值含义（ICM42605 数据手册，ODR=1kHz 时换算）：
 *   0: BW = ODR/2  = 500Hz   ← 等于不滤波
 *   1: BW = 400Hz
 *   2: BW = 200Hz   ← 陀螺当前选用
 *   3: BW = 125Hz
 *   4: BW = 100Hz
 *   5: BW = 62.5Hz
 *   6: BW = 50Hz
 *   7: BW = 25Hz
 *
 * 调参记录（都基于黑匣子实测）：
 *   · 索引 0（原值）：310~320Hz 振动完全通过，陀螺 RMS 42 deg/s。
 *   · 索引 3（125Hz）：振动压得很好（>100Hz 能量降 11.6 倍），
 *     但相位滞后把内环自激从 10Hz/RMS11 放大到 11.5Hz/RMS28，出现明显抖动。
 *   · 索引 2（200Hz，当前）：折中。310Hz 主要交给 315Hz 陷波 + 软件低通，
 *     内部只保留必要的抗混叠（挡住 500Hz 以上的有刷电机换向噪声）。
 *   振动回升 → 往 3/4 调；仍有自激 → 往 1 调（400Hz，几乎无滞后）。
 */
#define ICM42605_GYRO_UI_BW_IDX         1U
#define ICM42605_ACCEL_UI_BW_IDX        3U
#define ICM42605_REG_ACCEL_CONFIG1      0x53
#define ICM42605_REG_TMST_CONFIG        0x54
#define ICM42605_REG_APEX_CONFIG0       0x56
#define ICM42605_REG_SMD_CONFIG         0x57
#define ICM42605_REG_FIFO_CONFIG1       0x5F
#define ICM42605_REG_FIFO_CONFIG2       0x60
#define ICM42605_REG_FIFO_CONFIG3       0x61
#define ICM42605_REG_FSYNC_CONFIG       0x62
#define ICM42605_REG_INT_CONFIG0        0x63
#define ICM42605_REG_INT_CONFIG1        0x64
#define ICM42605_REG_INT_SOURCE0        0x65
#define ICM42605_REG_INT_SOURCE1        0x66
#define ICM42605_REG_INT_SOURCE3        0x68
#define ICM42605_REG_INT_SOURCE4        0x69
#define ICM42605_REG_FIFO_LOST_PKT0     0x6C
#define ICM42605_REG_FIFO_LOST_PKT1     0x6D
#define ICM42605_REG_SELF_TEST_CONFIG   0x70
#define ICM42605_REG_WHO_AM_I           0x75

#define ICM42605_WHO_AM_I_VALUE         0x42

#define ICM42605_GYRO_FS_2000DPS        0x00
#define ICM42605_GYRO_FS_1000DPS        0x01
#define ICM42605_GYRO_FS_500DPS         0x02
#define ICM42605_GYRO_FS_250DPS         0x03

#define ICM42605_ACCEL_FS_16G           0x00
#define ICM42605_ACCEL_FS_8G            0x01
#define ICM42605_ACCEL_FS_4G            0x02
#define ICM42605_ACCEL_FS_2G            0x03

#define ICM42605_GYRO_ODR_32KHZ         0x01
#define ICM42605_GYRO_ODR_16KHZ         0x02
#define ICM42605_GYRO_ODR_8KHZ          0x03
#define ICM42605_GYRO_ODR_4KHZ          0x04
#define ICM42605_GYRO_ODR_2KHZ          0x05
#define ICM42605_GYRO_ODR_1KHZ          0x06
#define ICM42605_GYRO_ODR_200HZ         0x07
#define ICM42605_GYRO_ODR_100HZ         0x08
#define ICM42605_GYRO_ODR_50HZ          0x09
#define ICM42605_GYRO_ODR_25HZ          0x0A
#define ICM42605_GYRO_ODR_12_5HZ        0x0B
#define ICM42605_GYRO_ODR_500HZ         0x0F

#define ICM42605_ACCEL_ODR_32KHZ        0x01
#define ICM42605_ACCEL_ODR_16KHZ        0x02
#define ICM42605_ACCEL_ODR_8KHZ         0x03
#define ICM42605_ACCEL_ODR_4KHZ         0x04
#define ICM42605_ACCEL_ODR_2KHZ         0x05
#define ICM42605_ACCEL_ODR_1KHZ         0x06
#define ICM42605_ACCEL_ODR_200HZ        0x07
#define ICM42605_ACCEL_ODR_100HZ        0x08
#define ICM42605_ACCEL_ODR_50HZ         0x09
#define ICM42605_ACCEL_ODR_25HZ         0x0A
#define ICM42605_ACCEL_ODR_12_5HZ       0x0B
#define ICM42605_ACCEL_ODR_6_25HZ       0x0C
#define ICM42605_ACCEL_ODR_3_125HZ      0x0D
#define ICM42605_ACCEL_ODR_1_5625HZ     0x0E
#define ICM42605_ACCEL_ODR_500HZ        0x0F

#define ICM42605_READ_FLAG              0x80

#define GYRO_SENSITIVITY_2000DPS        (16.4f)
#define ACCEL_SENSITIVITY_16G           (2048.0f)

#define DEG_TO_RAD                      (0.017453292519943295f)
#define RAD_TO_DEG                      (57.29577951308232f)

typedef struct {
    float x;
    float y;
    float z;
} Vector3f;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion;

typedef struct {
    float roll;
    float pitch;
    float yaw;
} EulerAngles;

typedef struct {
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temp_raw;
    
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
	
		float filtered_gyro_x;
		float filtered_gyro_y;
		float filtered_gyro_z;
    
    Quaternion quat;
    EulerAngles angle;
    
    float gyro_offset_x;
    float gyro_offset_y;
    float gyro_offset_z;
    
    float accel_offset_x;
    float accel_offset_y;
    float accel_offset_z;

    float accel_scale_x;
    float accel_scale_y;
    float accel_scale_z;
    
    float sample_period;
    float beta;
    
    float tilt_total;
		float vert_accel;
		float filtered_vert_accel;
} ICM42605_Data;

void ICM42605_CS_Low(void);
void ICM42605_CS_High(void);
uint8_t ICM42605_ReadReg(uint8_t reg);
void ICM42605_WriteReg(uint8_t reg, uint8_t data);
void ICM42605_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t ICM42605_Init(void);
void ICM42605_ReadRawData(ICM42605_Data *data);
void ICM42605_UpdateData(ICM42605_Data *data);
void ICM42605_Calibrate(ICM42605_Data *data, uint16_t samples);
void ICM42605_SixSideCalibrate(ICM42605_Data *data, uint16_t samples_per_face);

/* 加速度计三轴低通（修复 3.1）：使用前必须先调用一次 */
void ICM42605_InitAccelLPF(void);/* 陀螺零偏在线估计（修复 2） */
void ICM42605_SetArmed(uint8_t armed);
void ICM42605_ResetBiasTracker(ICM42605_Data *data);

/* 供黑匣子记录的解算内部量 */
float ICM42605_GetAccelConfidence(void);
float ICM42605_GetSamplePeriod(void);
/* 上电标定期间三轴陀螺标准差之和（deg/s）。静止标定应 < 0.5，
 * 偏大说明标定时飞机被扰动，零偏不可信（会造成恒定方向偏飞）。 */
float ICM42605_GetCalibStd(void);

/* ── 校准参数化（档 2：只做零偏）────────────────────────────────
 * 上电时如果有存过的标定结果（cal.* 参数非 0），用 ICM42605_SetCalibration()
 * 直接灌进去，不再要求"上电后静止 1 秒"；没有就退回 ICM42605_Calibrate()。
 *
 * 重新标定由控制台触发：ICM42605_RequestCalibration() 只置一个请求，
 * 真正的测量在 IMU 任务里做（ICM42605_ServiceCalibration()），
 * 避免控制台任务和 IMU 任务同时抢 SPI。 */
void    ICM42605_SetCalibration(ICM42605_Data *data,
                                float gx_dps, float gy_dps, float gz_dps,
                                float ax_g, float ay_g, float az_g);
void    ICM42605_RequestCalibration(void);
uint8_t ICM42605_ServiceCalibration(ICM42605_Data *data, uint8_t armed);
/* 0=空闲 1=已请求 2=正在标定 3=完成 4=被拒绝(解锁中) */
uint8_t ICM42605_CalibrationState(void);
void    ICM42605_GetCalibration(float *gx_dps, float *gy_dps, float *gz_dps,
                                float *ax_g, float *ay_g, float *az_g);

void Quaternion_Init(Quaternion *q);
void Quaternion_Update(Quaternion *q, Vector3f *gyro, Vector3f *accel, float dt, float beta);
void Quaternion_ToEulerAngles(Quaternion *q, EulerAngles *angle);
void Quaternion_Normalize(Quaternion *q);

void MahonyAHRSUpdate(Quaternion *q, Vector3f *gyro, Vector3f *accel, float dt, float kp, float ki);
void MadgwickAHRSUpdate(Quaternion *q, Vector3f *gyro, Vector3f *accel, float dt, float beta);

float ICM42605_CalculateTotalTilt(ICM42605_Data *data);
float ICM42605_Calculate_Vertical_Acceleration_Quaternion(Vector3f* accel, Quaternion* q,ICM42605_Data *data);
#ifdef __cplusplus
}
#endif

#endif
