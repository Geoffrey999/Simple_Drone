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
    float temperature;
    float pressure;
    float altitude;

    int32_t raw_pressure;
    int32_t raw_temperature;

    BMP280_CalibData calib;

    int32_t t_fine;
} BMP280_Data;


uint8_t BMP280_Init(BMP280_Data *data);
void BMP280_ReadCalibData(BMP280_Data *data);
void BMP280_ReadRawData(BMP280_Data *data);
void BMP280_CompensateData(BMP280_Data *data);
float BMP280_CalcAltitude(float pressure);



#ifdef __cplusplus
}
#endif

#endif
