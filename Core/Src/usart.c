/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include <stdio.h>
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;

/* 黑匣子指令通道(USART3)：DMA + 环形缓冲接收。
 * 460800 波特率下每字节仅 21.7µs，任务轮询（ms 级）必然丢字节，
 * 必须由 DMA 自动搬运到环形缓冲，任务只需按"计数器差值"取走新增部分。 */
uint8_t  Usart3RxBuf[USART3_RX_BUF_SIZE];
static uint16_t usart3_rx_last = 0;      /* 上次读到环形缓冲的哪个位置 */
static uint32_t usart3_rx_total = 0;     /* 累计收到的字节数（诊断用） */
static uint32_t usart3_ore_cnt = 0;      /* 溢出错误次数（诊断用） */
static uint32_t usart3_dma_err = 0;      /* DMA 传输错误次数（诊断用） */
static uint32_t usart3_last_rx_tick = 0; /* 上次收到字节的时刻（自愈用） */

/* 清掉 UART 的陈旧接收状态。
 * 关键：启动 DMA 之前必须做，否则上电瞬间因引脚未稳定而产生的一个陈旧字节
 * 会占住接收寄存器，被 DMA 当成"第 1 个数据字节"搬进缓冲，造成数据错位。
 * 清除方式是"先读 SR 再读 DR"，这个顺序不能反。 */
static void usart3_clear_stale_rx(void)
{
  volatile uint32_t tmp;

  tmp = USART3->SR;      /* 读 SR */
  tmp = USART3->DR;      /* 再读 DR */
  (void)tmp;
}

uint32_t USART3_GetRxTotal(void) { return usart3_rx_total; }
uint32_t USART3_GetOreCount(void) { return usart3_ore_cnt; }
uint32_t USART3_GetDmaErr(void)  { return usart3_dma_err; }

/* 重新武装 DMA 接收。用于自愈：一旦接收停摆，隔一会儿自动重启一次。 */
void USART3_RestartRx(void)
{
  (void)HAL_UART_DMAStop(&huart3);
  usart3_clear_stale_rx();
  usart3_rx_last = 0;
  (void)HAL_UART_Receive_DMA(&huart3, Usart3RxBuf, USART3_RX_BUF_SIZE);
}

/* 供 DMA1_Stream1 中断在检测到传输错误时调用 */
void USART3_OnDmaError(void)
{
  usart3_dma_err++;
}

void USART3_DMA_Rx_Init(void)
{
  /* ★ 必须先使能 DMA1 时钟！
   * MX_DMA_Init() 是 CubeMX 生成的，它只为"已配置 DMA 的外设"使能时钟——
   * 本工程里只有 SPI1/USART1 用了 DMA2，所以那里只有 __HAL_RCC_DMA2_CLK_ENABLE()。
   * USART3 走的是 DMA1_Stream1，DMA1 的时钟必须在这里自己开。
   * 否则：DMA1 寄存器读写全部无效，HAL_DMA_Init / HAL_UART_Receive_DMA 会"假装成功"
   *      （HAL 不检查寄存器是否真的写进去），但接收完全不工作。 */
  __HAL_RCC_DMA1_CLK_ENABLE();

  hdma_usart3_rx.Instance = DMA1_Stream1;
  hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;          /* USART3_RX = DMA1 CH4 */
  hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;              /* 循环：写满自动回卷 */
  hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
  hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
  {
    Error_Handler();
  }

  __HAL_LINKDMA(&huart3, hdmarx, hdma_usart3_rx);

  (void)HAL_UART_DMAStop(&huart3);   /* 若之前有残留接收，先停掉 */
  usart3_clear_stale_rx();           /* 清陈旧字节/错误标志，再启动 */

  usart3_rx_last  = 0;
  usart3_rx_total = 0;
  usart3_ore_cnt  = 0;
  usart3_dma_err  = 0;
  usart3_last_rx_tick = HAL_GetTick();

  (void)HAL_UART_Receive_DMA(&huart3, Usart3RxBuf, USART3_RX_BUF_SIZE);
}

/* 取走环形缓冲里新增的字节。
 *
 * 原理：循环 DMA 模式下，CNDTR 从 USART3_RX_BUF_SIZE 递减到 0 再回卷，
 *       所以 (SIZE - CNDTR) 就是 DMA 累计写入的位置。与上次位置相减，
 *       得到的就是这期间新到的字节。
 *
 * 返回值 = 本次取到的字节数。 */
uint16_t USART3_PollRx(uint8_t *out, uint16_t max)
{
  uint16_t pos;
  uint16_t n = 0;

  if ((out == NULL) || (max == 0))
  {
    return 0;
  }

  /* ---- 必须处理 ORE（溢出错误）----
   * STM32F4 上 ORE 置位后可能一直堵住 DMA 请求，接收会"静默死掉"，
   * 现象和"没接线"完全一样。所以每次进来都检查并清除。
   * 清除方式是"先读 SR 再读 DR"，这个顺序不能反。 */
  if ((USART3->SR & USART_SR_ORE) != 0U)
  {
    usart3_clear_stale_rx();
    usart3_ore_cnt++;
  }

  pos = (uint16_t)(USART3_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx));

  while ((usart3_rx_last != pos) && (n < max))
  {
    out[n++] = Usart3RxBuf[usart3_rx_last];
    usart3_rx_last++;
    if (usart3_rx_last >= USART3_RX_BUF_SIZE)
    {
      usart3_rx_last = 0;
    }
  }

  usart3_rx_total += n;
  if (n > 0U)
  {
    usart3_last_rx_tick = HAL_GetTick();
  }
  else if ((HAL_GetTick() - usart3_last_rx_tick) > 1000U)
  {
    /* 自愈：连续 1 秒一个字节都没收到，就重新武装一次 DMA。
     * 覆盖 ORE 卡死、DMA 传输错误等"静默停摆"的情况——
     * 这类故障的现象和"没接线"一样，靠这个兜底能自己恢复。 */
    USART3_RestartRx();
    usart3_last_rx_tick = HAL_GetTick();
  }

  return n;
}

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  /* 黑匣子指令/取数通道。460800 @APB1=42MHz，16 倍过采样下
   * USARTDIV=5.6962 -> 5+11/16，实际 461538 Bd，偏差 +0.16%（远小于 UART 容限）。 */
  huart3.Init.BaudRate = 460800;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA2_Stream2;
    hdma_usart1_rx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA2_Stream7;
    hdma_usart1_tx.Init.Channel = DMA_CHANNEL_4;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */
    /* USART3 指令接收的 DMA。优先级 6 >= 5，允许在其中调用 FreeRTOS 的 FromISR API。
     * （SPI3 的 NVIC 使能已移到 spi.c 的 HAL_SPI_MspInit 里，那儿才是它该在的地方） */
    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* USER CODE END USART3_MspInit 1 */  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PA0-WKUP     ------> UART4_TX
    PA1     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);

  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
int fputc(int ch, FILE *f)
{
	/* USART3 现在同时是黑匣子的指令/取数通道：倾倒二进制数据期间绝不能插入
	 * printf 内容，否则上位机会解析错位。黑匣子忙时直接丢弃字符。 */
	extern volatile uint8_t bb_busy;
	if (bb_busy != 0)
	{
		return (ch);
	}
	HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 100);
	return (ch);
}

#pragma import(__use_no_semihosting)  
int _ttywrch(int ch)    
{
    ch=ch;
	return ch;
}         
struct __FILE 
{ 
	int handle; 

}; 
FILE __stdout;       

void _sys_exit(int x) 
{ 
	x = x; 
} 
/* USER CODE END 1 */
