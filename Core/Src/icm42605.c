#include "icm42605.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "cmsis_os.h"
#include "filter.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;


#define ICM42605_REG_BANK_SEL  0x76

static uint8_t current_bank = 0xFF;

static SemaphoreHandle_t spi_dma_txrx_sem;

static uint8_t dma_tx_buffer[16] __attribute__((aligned(4)));
static uint8_t dma_rx_buffer[16] __attribute__((aligned(4)));

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(spi_dma_txrx_sem, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    
    ICM42605_WriteReg(ICM42605_REG_GYRO_ACCEL_CONFIG0, 0x00);
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
    
    data->accel_x = (float)data->accel_x_raw / ACCEL_SENSITIVITY_16G;
    data->accel_y = (float)data->accel_y_raw / ACCEL_SENSITIVITY_16G;
    data->accel_z = (float)data->accel_z_raw / ACCEL_SENSITIVITY_16G;
    
    data->gyro_x = ((float)data->gyro_x_raw / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_y = ((float)data->gyro_y_raw / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_z = ((float)data->gyro_z_raw / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    
    data->gyro_x -= data->gyro_offset_x;
    data->gyro_y -= data->gyro_offset_y;
    data->gyro_z -= data->gyro_offset_z;
	
    data->accel_x -= data->accel_offset_x;
    data->accel_y -= data->accel_offset_y;
    data->accel_z -= data->accel_offset_z;

    
    data->temp = ((float)data->temp_raw / 132.48f) + 25.0f;
    
    float accel_norm = sqrtf(data->accel_x * data->accel_x + 
                             data->accel_y * data->accel_y + 
                             data->accel_z * data->accel_z);
    
    if (accel_norm < 0.1f || accel_norm > 50.0f)
    {
        return;
    }
    
		
    Vector3f gyro = {data->gyro_x, data->gyro_y, data->gyro_z};
    Vector3f accel = {data->accel_x, data->accel_y, data->accel_z};

		MadgwickAHRSUpdate(&data->quat, &gyro, &accel, data->sample_period, data->beta);
//		MahonyAHRSUpdate(&data->quat, &gyro, &accel,data->sample_period, 1.0f, 0.0f);		

    Quaternion_ToEulerAngles(&data->quat, &data->angle);
    
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
    
    data->gyro_offset_x = 0.0f;
    data->gyro_offset_y = 0.0f;
    data->gyro_offset_z = 0.0f;
    data->accel_offset_x = 0.0f;
    data->accel_offset_y = 0.0f;
    data->accel_offset_z = 0.0f;
    
    for (uint16_t i = 0; i < samples; i++)
    {
        ICM42605_ReadRawData(data);
        gyro_x_sum += data->gyro_x_raw;
        gyro_y_sum += data->gyro_y_raw;
        gyro_z_sum += data->gyro_z_raw;
        accel_x_sum += data->accel_x_raw;
        accel_y_sum += data->accel_y_raw;
        accel_z_sum += data->accel_z_raw;
        osDelay(2);
    }
    
    data->gyro_offset_x = ((float)gyro_x_sum / samples / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_offset_y = ((float)gyro_y_sum / samples / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->gyro_offset_z = ((float)gyro_z_sum / samples / GYRO_SENSITIVITY_2000DPS) * DEG_TO_RAD;
    data->accel_offset_x = (float)accel_x_sum / samples / ACCEL_SENSITIVITY_16G;
    data->accel_offset_y = (float)accel_y_sum / samples / ACCEL_SENSITIVITY_16G;
    data->accel_offset_z = (float)(accel_z_sum / samples / ACCEL_SENSITIVITY_16G - 1.0f);
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


