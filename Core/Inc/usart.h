/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart4;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart3;

/* USER CODE BEGIN Private defines */
#define USART3_RX_BUF_SIZE    256U   /* 黑匣子指令 DMA 环形缓冲大小 */
/* DMA1_Stream1 传输错误标志。HAL 的 DMA_FLAG_* 按 stream 分组命名，
 * 这里需要一个"Stream1"的别名，实际位定义与 TEIF1 相同。 */
#ifndef DMA_FLAG_TEIF1_5
#define DMA_FLAG_TEIF1_5      DMA_FLAG_TEIF1
#endif

extern uint8_t Usart3RxBuf[USART3_RX_BUF_SIZE];
/* USER CODE END Private defines */

void MX_UART4_Init(void);
void MX_USART1_UART_Init(void);
void MX_USART3_UART_Init(void);

/* USER CODE BEGIN Prototypes */
/* 黑匣子指令通道(USART3)：DMA + 环形缓冲接收 */
void     USART3_DMA_Rx_Init(void);
uint16_t USART3_PollRx(uint8_t *out, uint16_t max);
/* 诊断：累计收到的字节数 / 溢出错误次数。
 * 用途：如果"发了指令没回复"，先看这两个数——
 *   rx_total 不涨  -> 字节根本没进 MCU（接线/DMA 未启动）
 *   ore_cnt 在涨   -> 接收在溢出，需查 DMA 是否被别的外设抢占 */
uint32_t USART3_GetRxTotal(void);
uint32_t USART3_GetOreCount(void);
uint32_t USART3_GetDmaErr(void);
/* 重新武装 DMA 接收（自愈用） */
void     USART3_RestartRx(void);
/* 由 DMA1_Stream1_IRQHandler 在检测到传输错误时调用 */
void     USART3_OnDmaError(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

