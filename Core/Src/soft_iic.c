#include "soft_iic.h"
#include "dwt.h"
#include <stdio.h>
static void IIC_Delay(void)
{
		uint32_t start;
		start = DWT->CYCCNT;
    while((DWT->CYCCNT - start)<168);
}

void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = IIC_SCL_PIN | IIC_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IIC_SCL_PORT, &GPIO_InitStruct);

    IIC_SCL_HIGH();
    IIC_SDA_HIGH();
}

void IIC_Start(void)
{
    IIC_SDA_HIGH();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SDA_LOW();
    IIC_Delay();
    IIC_SCL_LOW();
}

void IIC_Stop(void)
{
    IIC_SCL_LOW();
    IIC_SDA_LOW();
    IIC_Delay();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SDA_HIGH();
    IIC_Delay();
}

uint8_t IIC_Wait_Ack(void)
{
    uint8_t ack_time = 0;

    IIC_SDA_HIGH();
    IIC_Delay();
    IIC_SCL_HIGH();
    IIC_Delay();

    while (IIC_SDA_READ())
    {
        ack_time++;
        if (ack_time > 250)
        {
            IIC_Stop();
            return 1;
        }
    }

    IIC_SCL_LOW();
    return 0;
}

void IIC_Ack(void)
{
    IIC_SCL_LOW();
    IIC_SDA_LOW();
    IIC_Delay();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SCL_LOW();
}

void IIC_NAck(void)
{
    IIC_SCL_LOW();
    IIC_SDA_HIGH();
    IIC_Delay();
    IIC_SCL_HIGH();
    IIC_Delay();
    IIC_SCL_LOW();
}

void IIC_Send_Byte(uint8_t data)
{
    uint8_t i;

    IIC_SCL_LOW();

    for (i = 0; i < 8; i++)
    {
        if (data & 0x80)
            IIC_SDA_HIGH();
        else
            IIC_SDA_LOW();

        data <<= 1;
        IIC_Delay();
        IIC_SCL_HIGH();
        IIC_Delay();
        IIC_SCL_LOW();
        IIC_Delay();
    }
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, data = 0;

    IIC_SDA_HIGH();

    for (i = 0; i < 8; i++)
    {
        data <<= 1;
        IIC_SCL_HIGH();
        IIC_Delay();
        if (IIC_SDA_READ())
            data |= 0x01;
        IIC_SCL_LOW();
        IIC_Delay();
    }

    if (ack)
        IIC_Ack();
    else
        IIC_NAck();

    return data;
}
