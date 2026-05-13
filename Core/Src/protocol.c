#include "protocol.h"

uint8_t Usart1DmaBuf[RX_FRAME_SIZE];
volatile uint16_t Usart1RxLen = 0;
uint16_t Throttle = 0;

void USART1_DMA_Rx_Init(void)
{
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, Usart1DmaBuf, RX_FRAME_SIZE);
}

uint16_t Calculate_Checksum(const uint8_t* data, uint16_t len) {
    uint16_t sum = 0;
    for (uint16_t i = 0; i < len; i++) {
        sum += data[i];
    }
	sum = 0xffff - sum;
    return sum;
}




