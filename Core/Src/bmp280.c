#include "bmp280.h"
#include "cmsis_os.h"



static void BMP280_WriteReg(uint8_t reg, uint8_t data)
{
	IIC_Start();
	IIC_Send_Byte(BMP280_ADDR);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();
	IIC_Send_Byte(data);
	IIC_Wait_Ack();
	IIC_Stop();
}

static uint8_t BMP280_ReadReg(uint8_t reg)
{
	uint8_t data;

	IIC_Start();
	IIC_Send_Byte(BMP280_ADDR);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Send_Byte(BMP280_ADDR | 0x01);
	IIC_Wait_Ack();
	data = IIC_Read_Byte(0);
	IIC_Stop();

	return data;
}

static void BMP280_ReadMultiRegs(uint8_t reg, uint8_t *buf, uint16_t len)
{
	uint16_t i;

	IIC_Start();
	IIC_Send_Byte(BMP280_ADDR);
	IIC_Wait_Ack();
	IIC_Send_Byte(reg);
	IIC_Wait_Ack();

	IIC_Start();
	IIC_Send_Byte(BMP280_ADDR | 0x01);
	IIC_Wait_Ack();

	for (i = 0; i < len - 1; i++)
	{
			buf[i] = IIC_Read_Byte(1);
	}
	buf[len - 1] = IIC_Read_Byte(0);

	IIC_Stop();
}

uint8_t BMP280_Init(BMP280_Data *data)
{
	uint8_t id;

	IIC_Init();

	id = BMP280_ReadReg(BMP280_REG_ID);

	if (id != BMP280_ID)
	{
		return 0;
	}

	BMP280_WriteReg(BMP280_REG_RESET, 0xB6);

	osDelay(10);

	BMP280_WriteReg(BMP280_REG_CONFIG, 0x00);

	uint8_t ctrl = (BMP280_OS_X2 << 5) | (BMP280_OS_X16 << 2) | BMP280_MODE_NORMAL;
	BMP280_WriteReg(BMP280_REG_CTRL, ctrl);

	osDelay(100);

	BMP280_ReadCalibData(data);

	return 1;
}

void BMP280_ReadCalibData(BMP280_Data *data)
{
	uint8_t buf[24];

	BMP280_ReadMultiRegs(BMP280_REG_CAL, buf, 24);

	data->calib.dig_T1 = (uint16_t)((uint16_t)buf[1] << 8) | buf[0];
	data->calib.dig_T2 = (int16_t)((int16_t)buf[3] << 8) | buf[2];
	data->calib.dig_T3 = (int16_t)((int16_t)buf[5] << 8) | buf[4];
	data->calib.dig_P1 = (uint16_t)((uint16_t)buf[7] << 8) | buf[6];
	data->calib.dig_P2 = (int16_t)((int16_t)buf[9] << 8) | buf[8];
	data->calib.dig_P3 = (int16_t)((int16_t)buf[11] << 8) | buf[10];
	data->calib.dig_P4 = (int16_t)((int16_t)buf[13] << 8) | buf[12];
	data->calib.dig_P5 = (int16_t)((int16_t)buf[15] << 8) | buf[14];
	data->calib.dig_P6 = (int16_t)((int16_t)buf[17] << 8) | buf[16];
	data->calib.dig_P7 = (int16_t)((int16_t)buf[19] << 8) | buf[18];
	data->calib.dig_P8 = (int16_t)((int16_t)buf[21] << 8) | buf[20];
	data->calib.dig_P9 = (int16_t)((int16_t)buf[23] << 8) | buf[22];
}

void BMP280_ReadRawData(BMP280_Data *data)
{
    uint8_t buf[6];

    BMP280_ReadMultiRegs(BMP280_REG_PRESS, buf, 6);

    data->raw_pressure   = ((int32_t)buf[0] << 12) | ((int32_t)buf[1] << 4) | ((int32_t)buf[2] >> 4);
    data->raw_temperature = ((int32_t)buf[3] << 12) | ((int32_t)buf[4] << 4) | ((int32_t)buf[5] >> 4);
}

void BMP280_CompensateData(BMP280_Data *data)
{
	int32_t var1, var2, t;
	int64_t var1l, var2l, p;

	var1 = ((((data->raw_temperature >> 3) - ((int32_t)data->calib.dig_T1 << 1))) *
					((int32_t)data->calib.dig_T2)) >> 11;
	var2 = (((((data->raw_temperature >> 4) - ((int32_t)data->calib.dig_T1)) *
						((data->raw_temperature >> 4) - ((int32_t)data->calib.dig_T1))) >> 12) *
					((int32_t)data->calib.dig_T3)) >> 14;
	data->t_fine = var1 + var2;
	t = (data->t_fine * 5 + 128) >> 8;
	data->temperature = (float)t / 100.0f;

	var1l = ((int64_t)data->t_fine) - 128000;
	var2l = var1l * var1l * (int64_t)data->calib.dig_P6;
	var2l = var2l + ((var1l * (int64_t)data->calib.dig_P5) << 17);
	var2l = var2l + (((int64_t)data->calib.dig_P4) << 35);
	var1l = ((var1l * var1l * (int64_t)data->calib.dig_P3) >> 8) +
					((var1l * (int64_t)data->calib.dig_P2) << 12);
	var1l = (((((int64_t)1) << 47) + var1l)) * ((int64_t)data->calib.dig_P1) >> 33;
	if (var1l == 0)
	{
		data->pressure = 0;
	}
	else
	{
		p = 1048576 - data->raw_pressure;
		p = (((p << 31) - var2l) * 3125) / var1l;
		var1l = (((int64_t)data->calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
		var2l = (((int64_t)data->calib.dig_P8) * p) >> 19;
		p = ((p + var1l + var2l) >> 8) + (((int64_t)data->calib.dig_P7) << 4);
		data->pressure = (float)p / 256.0f;
	}

	data->altitude = BMP280_CalcAltitude(data->pressure);
}

float BMP280_CalcAltitude(float pressure)
{
    return 44330.0f * (1.0f - powf(pressure / 101325.0f, 1.0f / 5.255f));
}


