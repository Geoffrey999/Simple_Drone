#ifndef __IIC_H__
#define __IIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define IIC_SCL_PIN    GPIO_PIN_6
#define IIC_SCL_PORT   GPIOB
#define IIC_SDA_PIN    GPIO_PIN_7
#define IIC_SDA_PORT   GPIOB

#define IIC_SCL_HIGH()  HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_SET)
#define IIC_SCL_LOW()   HAL_GPIO_WritePin(IIC_SCL_PORT, IIC_SCL_PIN, GPIO_PIN_RESET)
#define IIC_SDA_HIGH()  HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_SET)
#define IIC_SDA_LOW()   HAL_GPIO_WritePin(IIC_SDA_PORT, IIC_SDA_PIN, GPIO_PIN_RESET)
#define IIC_SDA_READ()  HAL_GPIO_ReadPin(IIC_SDA_PORT, IIC_SDA_PIN)

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t data);
uint8_t IIC_Read_Byte(uint8_t ack);

#ifdef __cplusplus
}
#endif

#endif
