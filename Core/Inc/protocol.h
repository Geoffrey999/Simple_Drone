#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "usart.h"
#include <string.h>

#define RX_FRAME_SIZE          32
#define TX_FRAME_SIZE          24
#define FRAME_HEADER        0x2040
#define BATTERY_MIN_VOLTAGE	2820.0f		//3.4V
#define BATTERY_MAX_VOLTAGE	3490.0f		//4.2V
#define SCALING_FACTOR			6.7f		  //(BATTERY_MAX_VOLTAGE-BATTERY_MIN_VOLTAGE)/100

struct euler_frame{
	float tran_buf[3];
	uint32_t tail;
};

struct quat_frame{
	float quat[4];
	uint32_t tail;
};

typedef struct{
	uint16_t head;
	uint16_t bat;
	float roll;
	float pitch;
	float yaw;
	float altitude;
	uint16_t rssi;
	uint16_t checksum;
}Tx_Frame_t;

typedef struct {
    uint16_t head;
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t arm;
    uint16_t mode;
    uint16_t angle_p;
    uint16_t angle_i;
    uint16_t angle_d;
    uint16_t rate_p;
    uint16_t rate_i;
    uint16_t rate_d;
    uint16_t roll_offset;
    uint16_t pitch_offset;
    uint16_t checksum;
} Rx_Frame_t;

extern uint8_t Usart1DmaBuf[RX_FRAME_SIZE];
extern volatile uint16_t Usart1RxLen;
extern uint16_t Throttle;

void USART1_DMA_Rx_Init(void);
uint16_t Calculate_Checksum(const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif
