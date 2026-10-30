#ifndef __BMP280_H__
#define __BMP280_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "soft_iic.h"
#include <math.h>

#define BMP280_ADDR        0xEC

#define BMP280_REG_ID      0xD0
#define BMP280_REG_RESET   0xE0
#define BMP280_REG_STATUS  0xF3
#define BMP280_REG_CTRL    0xF4
#define BMP280_REG_CONFIG  0xF5
#define BMP280_REG_PRESS   0xF7
#define BMP280_REG_TEMP    0xFA
#define BMP280_REG_CAL     0x88

#define BMP280_ID          0x58

#define BMP280_MODE_SLEEP  0x00
#define BMP280_MODE_FORCE  0x01
#define BMP280_MODE_NORMAL 0x03

#define BMP280_OS_NONE     0x00
#define BMP280_OS_X1       0x01
#define BMP280_OS_X2       0x02
#define BMP280_OS_X4       0x03
#define BMP280_OS_X8       0x04
#define BMP280_OS_X16      0x05

/* CONFIG 寄存器(0xF5)的 IIR 系数编码（bit4:2），系数越大越平越慢 */
#define BMP280_IIR_OFF     0x00
#define BMP280_IIR_COEF_2  0x04
#define BMP280_IIR_COEF_4  0x08
#define BMP280_IIR_COEF_8  0x0C
#define BMP280_IIR_COEF_16 0x10

/* 气压计读取周期(ms)。×16 过采样单次转换约 40ms，必须大于它才不会读到重复样本。 */
#define BMP280_READ_PERIOD_MS   50U

#define WINDOW_SIZE 32

typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;

typedef struct {
    float temperature;   /* °C */
    /* ★ 单位是 Pa，不是 hPa！Bosch 的 64 位补偿返回 Q24.8(=Pa×256)，
     *   bmp280.c 里 /256 之后就得到 Pa，海平面典型值 101325。
     *   曾经把它当 hPa 用过，直接把整条高度链路判废。 */
    float pressure;      /* Pa */
    float altitude;      /* m（ISA 标准大气，基准 101325Pa；压力非法时是 NaN） */

    int32_t raw_pressure;
    int32_t raw_temperature;

    BMP280_CalibData calib;

    int32_t t_fine;
} BMP280_Data;


uint8_t BMP280_Init(BMP280_Data *data);
void BMP280_ReadCalibData(BMP280_Data *data);
void BMP280_ReadRawData(BMP280_Data *data);
void BMP280_CompensateData(BMP280_Data *data);
/* pressure 单位 **Pa**；超出 30000~120000 返回 NaN */
float BMP280_CalcAltitude(float pressure);



#ifdef __cplusplus
}
#endif

#endif
