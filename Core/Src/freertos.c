/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dwt.h"
#include "gpio.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "icm42605.h"
#include "bmp280.h"
#include "protocol.h"
#include "flycontrol.h"
#include "filter.h"
#include "stream_buffer.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
#define ARM_UNLOCKED    2000
#define MODE_ALT_HOLD   1000
#define MODE_HORIZON   	2000
#define MODE_LANDING  	1500
Tx_Frame_t tx_frame_buf;
ICM42605_Data imu_data;
FlyController fc;

static uint16_t prev_arm_status = 0;

/* ─── Inter-task communication data structures ─── */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_z;
    float tilt_total;
    float sample_period;
} IMU_Packet_t;

typedef struct {
    uint16_t roll;
    uint16_t pitch;
    uint16_t throttle;
    uint16_t yaw;
    uint16_t arm;
    uint16_t mode;
    uint16_t roll_offset;
    uint16_t pitch_offset;
    uint16_t battery_percent;
} RC_Packet_t;

/* ─── IPC handles ─── */
osMessageQueueId_t rcFrameQueueHandle;
osMessageQueueId_t imuFlyQueueHandle;
osMessageQueueId_t imuUsartQueueHandle;
osMessageQueueId_t altFlyQueueHandle;
osMessageQueueId_t altUsartQueueHandle;

StreamBufferHandle_t usart1RxStreamBuf;
SemaphoreHandle_t uart1TxSem;
/* USER CODE END Variables */
/* Definitions for BaroRead */
osThreadId_t BaroReadHandle;
const osThreadAttr_t BaroRead_attributes = {
  .name = "BaroRead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for IMURead */
osThreadId_t IMUReadHandle;
const osThreadAttr_t IMURead_attributes = {
  .name = "IMURead",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Usart */
osThreadId_t UsartHandle;
const osThreadAttr_t Usart_attributes = {
  .name = "Usart",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for FlyControl */
osThreadId_t FlyControlHandle;
const osThreadAttr_t FlyControl_attributes = {
  .name = "FlyControl",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityLow2,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBaroReadTask(void *argument);
void StartIMUReadTask(void *argument);
void StartUsartTask(void *argument);
void StartFlyControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationIdleHook(void);
void vApplicationTickHook(void);

/* USER CODE BEGIN 2 */
void vApplicationIdleHook( void )
{
   /* vApplicationIdleHook() will only be called if configUSE_IDLE_HOOK is set
   to 1 in FreeRTOSConfig.h. It will be called on each iteration of the idle
   task. It is essential that code added to this hook function never attempts
   to block in any way (for example, call xQueueReceive() with a block time
   specified, or call vTaskDelay()). If the application makes use of the
   vTaskDelete() API function (as this demo application does) then it is also
   important that vApplicationIdleHook() is permitted to return to its calling
   function, because it is the responsibility of the idle task to clean up
   memory allocated by the kernel to any task that has since been deleted. */
}
/* USER CODE END 2 */

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
}
/* USER CODE END 3 */

/* USER CODE BEGIN 4 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        xSemaphoreGiveFromISR(uart1TxSem, NULL);
    }
}
/* USER CODE END 4 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  uart1TxSem = xSemaphoreCreateBinary();
  xSemaphoreGive(uart1TxSem);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  rcFrameQueueHandle = osMessageQueueNew(3, sizeof(RC_Packet_t), NULL);
  imuFlyQueueHandle  = osMessageQueueNew(1, sizeof(IMU_Packet_t), NULL);
  imuUsartQueueHandle= osMessageQueueNew(1, sizeof(IMU_Packet_t), NULL);
  altFlyQueueHandle  = osMessageQueueNew(1, sizeof(float), NULL);
  altUsartQueueHandle= osMessageQueueNew(1, sizeof(float), NULL);

  usart1RxStreamBuf  = xStreamBufferCreate(256, 1);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of BaroRead */
  BaroReadHandle = osThreadNew(StartBaroReadTask, NULL, &BaroRead_attributes);

  /* creation of IMURead */
  IMUReadHandle = osThreadNew(StartIMUReadTask, NULL, &IMURead_attributes);

  /* creation of Usart */
  UsartHandle = osThreadNew(StartUsartTask, NULL, &Usart_attributes);

  /* creation of FlyControl */
  FlyControlHandle = osThreadNew(StartFlyControlTask, NULL, &FlyControl_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartBaroReadTask */
/**
  * @brief  Function implementing the BaroRead thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBaroReadTask */
void StartBaroReadTask(void *argument)
{
  /* USER CODE BEGIN StartBaroReadTask */
	BMP280_Data bmp_data;
	MovingAvgFilter_t altitude_filter;
	
	float moving_avg_buf[WINDOW_SIZE];
	float altitude_offset;
	float filtered_altitude;
	DWT_Init();
	
	osDelay(2000);
	MovingAvg_Init(&altitude_filter,moving_avg_buf,WINDOW_SIZE);
	if (BMP280_Init(&bmp_data))
	{
		for(int i=0;i<100;i++)
		{
			BMP280_ReadRawData(&bmp_data);
			BMP280_CompensateData(&bmp_data);
			filtered_altitude = MovingAvg_Update(&altitude_filter, bmp_data.altitude);
			if(i>=WINDOW_SIZE)
			{
				altitude_offset+=filtered_altitude;
			}
			osDelay(20);
		}
		altitude_offset = altitude_offset/(100.0f-WINDOW_SIZE);
	}
	TickType_t last_tick;
	
  /* Infinite loop */
  for(;;)
  {
		last_tick = xTaskGetTickCount();
    BMP280_ReadRawData(&bmp_data);
    BMP280_CompensateData(&bmp_data);
		filtered_altitude = MovingAvg_Update(&altitude_filter, bmp_data.altitude) - altitude_offset;

		osMessageQueuePut(altFlyQueueHandle,   &filtered_altitude, 0, 0);
		osMessageQueuePut(altUsartQueueHandle, &filtered_altitude, 0, 0);
		vTaskDelayUntil(&last_tick,20);
  }
  /* USER CODE END StartBaroReadTask */
}

/* USER CODE BEGIN Header_StartIMUReadTask */
/**
* @brief Function implementing the IMURead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartIMUReadTask */
void StartIMUReadTask(void *argument)
{
  /* USER CODE BEGIN StartIMUReadTask */
	uint8_t imu_init_ok = 0;
	LowPassFilter_t gyro_filter;
	LowPass_Init(&gyro_filter, 0.51f);
	osDelay(2000);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);
	imu_data.sample_period = 0.001f;
	imu_data.beta = 0.2f;
	
	Quaternion_Init(&imu_data.quat);
	
	imu_init_ok = ICM42605_Init();
	if (imu_init_ok)
	{
		 ICM42605_Calibrate(&imu_data, 1000);
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	TickType_t last_tick;
		
  /* Infinite loop */
	for(;;)
	{
		last_tick = xTaskGetTickCount();
		ICM42605_UpdateData(&imu_data);
		imu_data.filtered_gyro_x = LowPass_Update(&gyro_filter,imu_data.gyro_x);
		imu_data.filtered_gyro_y = LowPass_Update(&gyro_filter,imu_data.gyro_y);
		imu_data.filtered_gyro_z = LowPass_Update(&gyro_filter,imu_data.gyro_z);
			IMU_Packet_t pkt;
			pkt.roll         = imu_data.angle.roll;
			pkt.pitch        = imu_data.angle.pitch;
			pkt.yaw          = imu_data.angle.yaw;
			pkt.gyro_x       = imu_data.gyro_x;
			pkt.gyro_y       = imu_data.gyro_y;
			pkt.gyro_z       = imu_data.gyro_z;
			pkt.accel_z      = imu_data.accel_z;
			pkt.tilt_total   = imu_data.tilt_total;
			pkt.sample_period= imu_data.sample_period;

			osMessageQueuePut(imuFlyQueueHandle,   &pkt, 0, 0);
			osMessageQueuePut(imuUsartQueueHandle, &pkt, 0, 0);
		vTaskDelayUntil(&last_tick,1);
	}
  /* USER CODE END StartIMUReadTask */
}

/* USER CODE BEGIN Header_StartUsartTask */
/**
* @brief Function implementing the Usart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsartTask */
void StartUsartTask(void *argument)
{
  /* USER CODE BEGIN StartUsartTask */
		tx_frame_buf.head = 0x4020;
		tx_frame_buf.bat = 0;
		tx_frame_buf.roll = 0;
		tx_frame_buf.pitch = 0;
		tx_frame_buf.yaw = 0;
		tx_frame_buf.altitude = 0;
		tx_frame_buf.rssi = 0;
		uint8_t temp_buf[RX_FRAME_SIZE];
		USART1_DMA_Rx_Init();
		TickType_t last_tick;
		uint32_t no_rx_count;
		uint8_t total_rx_count;
		uint8_t successful_rx_count;

		Rx_Frame_t rx_local;
		IMU_Packet_t usart_imu;
		float usart_alt;

	  /* Infinite loop */
	  for(;;)
	  {
		  last_tick = xTaskGetTickCount();

			/* ── 1. Receive RC frame from stream buffer (ISR → task) ── */
			size_t read_len = xStreamBufferReceive(usart1RxStreamBuf, temp_buf, RX_FRAME_SIZE, 0);
			if(read_len == RX_FRAME_SIZE)
			{
				memcpy(&rx_local, temp_buf, RX_FRAME_SIZE);

				RC_Packet_t rc_pkt;
				rc_pkt.roll           = rx_local.roll;
				rc_pkt.pitch          = rx_local.pitch;
				rc_pkt.throttle       = rx_local.throttle;
				rc_pkt.yaw            = rx_local.yaw;
				rc_pkt.arm            = rx_local.arm;
				rc_pkt.mode           = rx_local.mode;
				rc_pkt.roll_offset    = rx_local.roll_offset;
				rc_pkt.pitch_offset   = rx_local.pitch_offset;
				rc_pkt.battery_percent = tx_frame_buf.bat;

				osMessageQueuePut(rcFrameQueueHandle, &rc_pkt, 0, 0);

				Throttle = rx_local.throttle - 1000;
				no_rx_count = 0;
				successful_rx_count++;
			}
			else
			{
				no_rx_count++;
			}

			/* ── 2. RC link lost — send disarm packet ── */
			if(no_rx_count > 30)
			{
				RC_Packet_t rc_pkt = {0};
				osMessageQueuePut(rcFrameQueueHandle, &rc_pkt, 0, 0);

				HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
				HAL_UART_DMAStop(&huart1);
				HAL_UART_Receive_DMA(&huart1, Usart1DmaBuf, RX_FRAME_SIZE);
				no_rx_count = 0;
			}

			/* ── 3. Read battery ADC ── */
			HAL_ADC_Start(&hadc1);
			HAL_ADC_PollForConversion(&hadc1, 1);
			if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
			{
				tx_frame_buf.bat = (HAL_ADC_GetValue(&hadc1)-BATTERY_MIN_VOLTAGE)/SCALING_FACTOR;
			}

			/* ── 4. Read IMU telemetry from queue ── */
			if (osMessageQueueGet(imuUsartQueueHandle, &usart_imu, NULL, 0) == osOK)
			{
				tx_frame_buf.roll  = usart_imu.roll;
				tx_frame_buf.pitch = usart_imu.pitch;
				tx_frame_buf.yaw   = usart_imu.yaw;
			}

			/* ── 5. Read altitude telemetry from queue ── */
			if (osMessageQueueGet(altUsartQueueHandle, &usart_alt, NULL, 0) == osOK)
			{
				tx_frame_buf.altitude = usart_alt;
			}

			/* ── 6. Transmit telemetry (every 4 cycles), DMA-safe ── */
			if(total_rx_count % 4 == 0)
			{
				xSemaphoreTake(uart1TxSem, pdMS_TO_TICKS(100));
				tx_frame_buf.checksum = Calculate_Checksum((uint8_t*)&tx_frame_buf, TX_FRAME_SIZE-2);
	 			HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&tx_frame_buf, TX_FRAME_SIZE);
			}

			/* ── 7. RSSI statistics ── */
			total_rx_count++;
			if(total_rx_count >= 100)
			{
				tx_frame_buf.rssi = (uint16_t)successful_rx_count;
				successful_rx_count = 0;
				total_rx_count = 0;
			}

			vTaskDelayUntil(&last_tick, 16);
	  }
  /* USER CODE END StartUsartTask */
}


/* USER CODE BEGIN Header_StartFlyControlTask */
/**
* @brief Function implementing the FlyControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlyControlTask */
void StartFlyControlTask(void *argument)
{
  /* USER CODE BEGIN StartFlyControlTask */
		osDelay(4100);
		TickType_t last_tick;
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);

		FlyController_Init(&fc);

		/* Local copies updated from queues (persist across cycles) */
		ICM42605_Data imu_local;
		memset(&imu_local, 0, sizeof(imu_local));
		imu_local.sample_period = 0.001f;

		RC_Packet_t current_rc = {0};
		float current_alt = 0.0f;

	  /* Infinite loop */
	  for(;;)
	  {
			last_tick = xTaskGetTickCount();

			/* ── 1. Read latest IMU data from queue (non-blocking) ── */
			IMU_Packet_t imu_pkt;
			if (osMessageQueueGet(imuFlyQueueHandle, &imu_pkt, NULL, 0) == osOK)
			{
				imu_local.angle.roll     = imu_pkt.roll;
				imu_local.angle.pitch    = imu_pkt.pitch;
				imu_local.angle.yaw      = imu_pkt.yaw;
				imu_local.gyro_x         = imu_pkt.gyro_x;
				imu_local.gyro_y         = imu_pkt.gyro_y;
				imu_local.gyro_z         = imu_pkt.gyro_z;
				imu_local.accel_z        = imu_pkt.accel_z;
				imu_local.tilt_total     = imu_pkt.tilt_total;
				imu_local.sample_period  = imu_pkt.sample_period;
			}

			/* ── 2. Read latest RC packet from queue (non-blocking) ── */
			RC_Packet_t rc_pkt;
			if (osMessageQueueGet(rcFrameQueueHandle, &rc_pkt, NULL, 0) == osOK)
			{
				current_rc = rc_pkt;
			}

			/* ── 3. Read latest altitude from queue (non-blocking) ── */
			float alt_val;
			if (osMessageQueueGet(altFlyQueueHandle, &alt_val, NULL, 0) == osOK)
			{
				current_alt = alt_val;
			}

			/* ── 4. Compute control targets ── */
			float target_roll  =  (current_rc.roll - 1500.0f) / 9.0f + (current_rc.roll_offset - 1500.0f);
			float target_pitch =  (current_rc.pitch - 1500.0f) / 9.0f + (current_rc.pitch_offset - 1500.0f);
			float target_yaw   =   current_rc.yaw - 1500.0f;
			float target_throttle;

			FlyController_SetCurrentAltitude(&fc, current_alt);

			if(current_rc.mode == MODE_ALT_HOLD)
			{
				FlyController_SetTargetAltitude(&fc, (1.0f + ((current_rc.throttle - 1000.0f) / 500.0f)));
				target_throttle = Throttle_Voltage_Compensation(current_rc.battery_percent);
				fc.flying_status = ALTITUDE_HOLD_FLYING;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			else if(current_rc.mode == MODE_HORIZON)
			{
				target_throttle = (current_rc.throttle - 1000.0f) * THROTTLE_TO_PWM_MULTIPLE_SCALE;
				fc.flying_status = HORIZON_FLYING;
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
			}
			else
			{
				FlyController_SetTargetAltitude(&fc, -3.0f);
				target_throttle = Throttle_Voltage_Compensation(current_rc.battery_percent);
				if((fc.hover_throttle < Throttle_Voltage_Compensation(current_rc.battery_percent) - 60.0f) && (current_alt < 0.3f))
				{
					fc.flying_status = LANDED;
					target_throttle = 0;
				}
			}

			/* ── 5. Execute control ── */
			if((current_rc.arm == ARM_UNLOCKED) && (fc.flying_status != LANDED))
			{
				if(prev_arm_status != ARM_UNLOCKED)
				{
					FlyController_Reset(&fc);
				}
				FlyController_SetTarget(&fc, target_roll, target_pitch, target_yaw, target_throttle);
				FlyController_Update(&fc, &imu_local);

				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, fc.motor1);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, fc.motor2);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, fc.motor3);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, fc.motor4);
			}
			else
			{
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
			}

			prev_arm_status = current_rc.arm;

			vTaskDelayUntil(&last_tick, 1);
	  }
  /* USER CODE END StartFlyControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

