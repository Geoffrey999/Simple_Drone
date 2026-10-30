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
#include "height_estimator.h"
#include "protocol.h"
#include "flycontrol.h"
#include "filter.h"
#include "blackbox.h"
#include "param.h"
#include "stream_buffer.h"
#include "semphr.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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

/* ── 模式确认时间 ──
 * 遥控是**三位开关**：手动(2000) / 降落(1500) / 定高(1000)。
 * 从手动切定高，开关**必然扫过 1500**。如果按"当前值"直接判定，
 * 那几十毫秒里固件会真的进入降落分支：目标高度变 -3m、基准油门掉到电压补偿值，
 * 若此刻高度 <0.3m 还会直接断电并锁成 LANDED —— 手动起飞后想切定高就会"掉一下"。
 * 所以要求某个模式**连续保持 MODE_CONFIRM_MS** 才真正生效，
 * 扫过中间位的时间(通常 50~150ms)自然被吃掉；顺带也滤掉了模式通道的抖动。 */
#define MODE_CONFIRM_MS   250U
/* ── 降落触地判据（2026-10-26 重写）──
 * 条件必须连续满足这么久才断电（1kHz 调用 → 200ms = 气压 4 个样本）。 */
#define LAND_TOUCH_CONFIRM_MS   200U
/* ⚠ 绝对高度**不能**用来判断"是否贴地"：起降点不等高（降落点比起飞点低/高）时
 * 它根本没有意义。这个门限只做一层极松的兜底（防止在巡航高度就出现"垂速塌陷"
 * 这种离谱情况）：8m 表示"从 8m 以上进入降落时，先降到 8m 以下才允许判触地"。 */
#define LAND_TOUCH_ALT_GATE     8.00f
/* 高度兜底判据：估计高度低于这里**且垂速也已经归零**才认为贴地。
 * 只有高度不够（气压漂移 ±0.5m，且起降点可能不等高）。 */
#define LAND_TOUCH_ALT_MIN      0.08f
/* 垂速塌陷判据：最近 400ms 内垂速曾低于 FROM，而现在**平均**垂速 |v| 小于 TO。
 * 这是"与绝对高度无关"的接触证据 —— 只有地面能让下降中的飞机速度归零。
 * TO 用 0.12 而不是 0.08：贴着地面时融合垂速会抖 ±0.09，门限太紧会让判据闪烁
 * （26 日 21:37 那次飞机已趴地，判据在 1315/1563 之间来回跳，磨了 1 秒才断电）。 */
#define LAND_COLLAPSE_FROM     -0.15f
#define LAND_COLLAPSE_TO        0.12f
/* "现在停了"取最近几个样本的平均：20Hz 下 4 个 = 200ms，噪声 8.9→4.5 cm/s */
#define LAND_VVEL_AVG_N         4U
/* ── 判据 0：加计冲击（最可靠）──
 * 实测触地冲击 2.84~15.8 g（机架硬、飞控硬装），而全飞行空中振动最大 1.66~1.99 g。
 * 2.5 g 门槛在两者之间，两边都有余量。这是唯一直接测量"碰地"的信号：
 * 触地瞬间桨洗气流撞地会把气压打成 +0.28m/s 的假"上升"，垂速判据会被骗，
 * 但加计上的 4.19 g 尖峰骗不了。 */
#define LAND_IMPACT_G           2.5f
#define LAND_IMPACT_CONFIRM_MS  3U
/* 气压跳变识别：桨洗气流撞地会在 50ms 内让气压高度"跳" 1~2m（等效 >20m/s）。
 * 检测到这种非物理跳变后，触地判据冻结 400ms —— 实测一次误断电就是被它害的。 */
#define BARO_JUMP_RATE_MAX      6.0f
#define BARO_JUMP_HOLDOFF_MS    400U
/* 垂速历史长度：气压 20Hz，8 个样本 ≈ 400ms */
#define VVEL_HIST_N             8U
/* 气压门控只能**推迟**断电，不能"否决"：触地条件（不看门控）连续满足这么久
 * 就强制断电。否则气压反复跳变时判据会被永久冻结，只能等 20 秒超时磨桨。 */
#define LAND_TOUCH_ESCALATE_MS  800U
/* 判到触地但还在确认窗里时，把油门压到悬停油门的这个比例。
 * 地面效应下 85% 的悬停油门刚好"贴住地面"，不会把飞机顶起来弹一下。 */
#define LAND_TOUCH_UNLOAD_RATIO   0.85f

/* ── 手动模式油门曲线 ──
 * 摇杆 1000..2000 → 输出比例 0..1，分段线性，四点 (1000,0)(x1,y1)(x2,y2)(2000,1)。
 * x1..x2 是"最缓"的一段（默认 1500~1700，罩住悬停区间）：把 y1/y2 设得接近，
 * 这一段就变平，摇杆在悬停附近微调更细腻；代价是 1700 以上变陡（很少用到）。
 * 默认 (1500,0.50)(1700,0.70) 时与直线重合，行为和以前完全一样。 */
static float FlyControl_ThrottleCurve(float stick,
                                      float x1, float y1, float x2, float y2)
{
	if (stick <= 1000.0f) { return 0.0f; }
	if (stick >= 2000.0f) { return 1.0f; }

	if (stick < x1)
	{
		float span = x1 - 1000.0f;
		return (span > 1.0f) ? (y1 * (stick - 1000.0f) / span) : y1;
	}
	if (stick < x2)
	{
		float span = x2 - x1;
		return (span > 1.0f) ? (y1 + (y2 - y1) * (stick - x1) / span) : y2;
	}
	{
		float span = 2000.0f - x2;
		return (span > 1.0f) ? (y2 + (1.0f - y2) * (stick - x2) / span) : 1.0f;
	}
}
#define MODE_BAND         250U      /* 1000/1500/2000 各自的判定半宽 */

/* 把模式通道归类成三档。用区间而不是精确相等：
 * 遥控通道是模拟量，切换过程/校准偏差会出现 1480、1010 这类值。 */
static uint16_t FlyControl_ClassifyMode(uint16_t mode)
{
    if (mode <= (uint16_t)(MODE_ALT_HOLD + MODE_BAND)) { return MODE_ALT_HOLD; }
    if (mode >= (uint16_t)(MODE_HORIZON  - MODE_BAND)) { return MODE_HORIZON;  }
    return MODE_LANDING;
}

Tx_Frame_t tx_frame_buf;
ICM42605_Data imu_data;
FlyController fc;
HeightEstimator est;

static uint16_t prev_arm_status = 0;

/* ─── Inter-task communication data structures ─── */
typedef struct {
    float roll;
    float pitch;
    float yaw;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float filtered_vert_accel;
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

typedef struct {
    float data1;
    float data2;
//		float data3;
    char tail[4];

} Vofa_t;

/* ─── IPC handles ─── */
osMessageQueueId_t rcFrameQueueHandle;
osMessageQueueId_t imuFlyQueueHandle;
osMessageQueueId_t imuUsartQueueHandle;
osMessageQueueId_t altFlyQueueHandle;
osMessageQueueId_t vertVelocityQueueHandle;
osMessageQueueId_t altUsartQueueHandle;

StreamBufferHandle_t usart1RxStreamBuf;
SemaphoreHandle_t uart1TxSem;

/* ─── 时序抖动统计（诊断用：只累加，绝不参与控制） ───
 * 目的：用数据判断"黑匣子的 Flash 操作有没有拖慢控制环 / IMU 采样"。
 * 由 BB:STAT 命令额外输出一行 BB:JITTER。
 *
 *   ulImuMaxUs     / ulFcMaxUs      : IMU采样 / FlyControl循环 的最大周期(µs)
 *   ulImuOver1500  / ulFcOver1500   : 周期 > 1500µs 的次数(≈丢了一拍)
 *   ulImuOver2500  / ulFcOver2500   : 周期 > 2500µs 的次数(≈丢了两拍)
 *
 * 判读：正常情况下两者都应稳定在 1000~1100µs，Over* 保持 0。
 */
volatile uint32_t ulImuMaxUs    = 0;
volatile uint32_t ulImuOver1500 = 0;
volatile uint32_t ulImuOver2500 = 0;
volatile uint32_t ulFcMaxUs     = 0;
volatile uint32_t ulFcOver1500  = 0;
volatile uint32_t ulFcOver2500  = 0;

/* ─── 高度数据源健康标志（只由 BaroRead 任务写，FlyControl 只读）───
 *   0 = 气压计不存在 / 初始化失败 / 连续 200ms 没有有效样本
 *   1 = 有新鲜可用的气压高度
 * 用途：定高必须看这个，而不是"这一拍队列里有没有新数据"。
 * 原来的写法(只看队列)在气压计根本没识别到时，融合器会拿一个恒为 0 的
 * 假气压当真值，height 永远是 0，而定高会照着这个 0 一路把油门往上推。 */
volatile uint8_t g_alt_ok = 0;
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
/* Definitions for BlackBox（黑匣子：最低优先级，宁可丢数据也绝不抢占控制环） */
osThreadId_t BlackBoxHandle;
const osThreadAttr_t BlackBox_attributes = {
  .name = "BlackBox",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartBaroReadTask(void *argument);
void StartIMUReadTask(void *argument);
void StartUsartTask(void *argument);
void StartFlyControlTask(void *argument);
void StartBlackBoxTask(void *argument);

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
	vertVelocityQueueHandle = osMessageQueueNew(1, sizeof(float), NULL);
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

  /* creation of BlackBox */
  BlackBoxHandle = osThreadNew(StartBlackBoxTask, NULL, &BlackBox_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
Vofa_t vofa_data_frame= {
			.data1 = 0.0f,
			.data2 = 0.0f,
//			.data3 = 0.0f,
			.tail = {0,0,0x80,0x7f}
	};
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
	/* ★ 必须初始化：原来这里是 `float altitude_offset;`，然后直接 += 累加，
	 *   属于读未初始化变量(UB)。更糟的是 BMP280_Init 失败时整段被跳过，
	 *   这个垃圾值还会被拿去当"地面参考"减掉，等于凭空给高度加一个随机偏置。 */
	float altitude_offset = 0.0f;
	float filtered_altitude;
	float vert_velocity;

	DWT_Init();
	HeightEstimator_Init(&est);
	/* 高度互补滤波增益（重启生效，见 param.c 的 est.* 注释）。
	 * SetGains 会检查稳定域 Kb·Kp > Kv，越界就保留默认值并置 gain_rejected。 */
	HeightEstimator_SetGains(&est,
	                         Param_GetApplied(P_EST_KP),
	                         Param_GetApplied(P_EST_KV),
	                         Param_GetApplied(P_EST_KB));

	osDelay(2000);

	/* ── 1. 气压计初始化：失败就每秒重试，但**绝不**把假数据喂给融合算法 ──
	 * 原来失败后照样跑主循环：bmp_data 是未初始化栈内存，calib 全 0/全 0xFF，
	 * 算出来的压力/高度是垃圾(甚至是 NaN)，而 Reset() 已经把 initialized 置 1，
	 * 融合器会拿它当真值 → 定高照着假高度推油门。 */
	while (BMP280_Init(&bmp_data) == 0)
	{
		g_alt_ok = 0;
		osDelay(1000);
	}

	/* ── 2. 地面参考面：约 3 秒(60 次×50ms) 滑动平均，跳过 NaN ──
	 * （读取周期从 20ms 改成 BMP280_READ_PERIOD_MS=50ms：
	 *   ×16 过采样单次转换约 40ms，20ms 读一次约一半是重复样本，实测 48%。） */
	MovingAvg_Init(&altitude_filter, moving_avg_buf, WINDOW_SIZE);
	{
		float    sum = 0.0f;
		float    last_good = 0.0f;
		uint32_t good = 0;
		int i;

		for (i = 0; i < 60; i++)
		{
			BMP280_ReadRawData(&bmp_data);
			BMP280_CompensateData(&bmp_data);

			/* NaN 自身不等于自身。坏样本既不进滑动平均(NaN 会永久污染 sum)，
			 * 也不进累加，只把最后一个好值留作兜底。 */
			if (bmp_data.altitude == bmp_data.altitude)
			{
				last_good = bmp_data.altitude;
				filtered_altitude = MovingAvg_Update(&altitude_filter, bmp_data.altitude);

				if ((i >= (int)WINDOW_SIZE) && (filtered_altitude == filtered_altitude))
				{
					sum += filtered_altitude;
					good++;
				}
			}
			osDelay(BMP280_READ_PERIOD_MS);
		}

		if (good >= 10U)
		{
			altitude_offset = sum / (float)good;
		}
		else if (last_good == last_good)
		{
			altitude_offset = last_good;
		}
		else
		{
			altitude_offset = 0.0f;
		}
	}

	/* 参考面就是此刻的 0 米 */
	HeightEstimator_Reset(&est, 0.0f);

	TickType_t last_tick;

  /* Infinite loop */
  for(;;)
  {
		last_tick = xTaskGetTickCount();
    BMP280_ReadRawData(&bmp_data);
    BMP280_CompensateData(&bmp_data);

		/* 融合。返回值 0 表示样本可信；NaN/超范围/跳变过大都会被拒。
		 * 被拒时不推队列 → 控制侧 altitude_ready=0，定高环这一拍不更新；
		 * 连续被拒超过 200ms 融合器会把 baro_ok 置 0，定高整体退出。 */
		if (HeightEstimator_UpdateBaro(&est,
		                               bmp_data.altitude - altitude_offset,
		                               bmp_data.pressure) == 0U)
		{
			filtered_altitude = HeightEstimator_GetHeight(&est);
			vert_velocity     = HeightEstimator_GetVelocity(&est);

			osMessageQueuePut(vertVelocityQueueHandle,   &vert_velocity, 0, 0);
			osMessageQueuePut(altFlyQueueHandle,   &filtered_altitude, 0, 0);
			osMessageQueuePut(altUsartQueueHandle, &filtered_altitude, 0, 0);
		}

		/* 健康标志：由融合器的新鲜度(200ms 超时)决定，不是"这一拍有没有数据" */
		g_alt_ok = HeightEstimator_IsBaroOk(&est);

		/* 读取周期必须**大于** BMP280 的转换时间（×16 过采样约 40ms），
		 * 否则一半的读回是上一次的重复值（实测 48%）。见 bmp280.h 的
		 * BMP280_READ_PERIOD_MS。 */
		vTaskDelayUntil(&last_tick, BMP280_READ_PERIOD_MS);
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
	LowPassFilter_t gyro_filter_lpf_x, gyro_filter_lpf_y, gyro_filter_lpf_z,accel_filter_vert;

	/* 陀螺软件低通：0.51(≈107Hz) -> 0.7(≈190Hz)。
	 * 原因：107Hz 的低通在 11.5Hz 处贡献 ~6° 相位滞后，叠加 IMU 内部低通后
	 * 把内环相位裕度吃掉，实测出现 11.5Hz 自激（RMS 28 deg/s）。
	 * 振动这件事现在主要由 IMU 内部 200Hz 带宽 + 315Hz 陷波负责，
	 * 软件低通不必再那么狠，放宽以换回相位裕度。
	 *
	 * 系数改由参数表给（gyro.lpf_x/y/z、accel.lpf_vert），重启生效。 */
	LowPass_Init(&gyro_filter_lpf_x,    Param_GetApplied(P_GYRO_LPF_X));
	LowPass_Init(&gyro_filter_lpf_y,    Param_GetApplied(P_GYRO_LPF_Y));
	LowPass_Init(&gyro_filter_lpf_z,    Param_GetApplied(P_GYRO_LPF_Z));
	LowPass_Init(&accel_filter_vert,    Param_GetApplied(P_ACCEL_LPF_VERT));
	
	NotchFilter_t gyro_filter_notch_x,gyro_filter_notch_y,gyro_filter_notch_z,accel_filter_notch_vert;

	/* ── 陷波器频率必须对准实测振动峰 ──
	 * 原设定 180Hz，但用黑匣子实测（f2.bin 频谱分析）振动主峰在：
	 *   gyro_x 310.9Hz / gyro_y 320.0Hz / accel_y 310.9Hz
	 * 180Hz 的陷波对 310Hz 只有边缘处的轻微衰减，等于没起作用。
	 *
	 * 现改为 315Hz、带宽 1 个倍频程（约 223~445Hz 覆盖），
	 * 能覆盖电机转速随油门变化时的峰漂移。
	 * 注意：采样率 1kHz，Nyquist=500Hz，315Hz 已偏高，
	 * 所以带宽不能太窄（Q 太高时系数对浮点误差敏感）。
	 *
	 * ★ 这一层是"补刀"：真正把 310Hz 压下去的主力是 IMU 内部 UI 低通
	 *   （见 icm42605.c 的 GYRO_ACCEL_CONFIG0，已设为 125Hz 带宽）。 */
	/* 陷波参数走参数表（notch.freq / notch.bw），实测振动峰在 311~320Hz，
	 * 默认 315Hz + 1 倍频程带宽。改它需要 SAVE + 重启。 */
	Notch_Init(&gyro_filter_notch_x, 1000.0f,
	           Param_GetApplied(P_NOTCH_FREQ), Param_GetApplied(P_NOTCH_BW));
	Notch_Init(&gyro_filter_notch_y, 1000.0f,
	           Param_GetApplied(P_NOTCH_FREQ), Param_GetApplied(P_NOTCH_BW));
	Notch_Init(&gyro_filter_notch_z, 1000.0f,
	           Param_GetApplied(P_NOTCH_FREQ), Param_GetApplied(P_NOTCH_BW));
	Notch_Init(&accel_filter_notch_vert, 1000.0f,
	           Param_GetApplied(P_NOTCH_FREQ), Param_GetApplied(P_NOTCH_BW));
	osDelay(2000);
	GREEN_LIGHT_ON;
	/* sample_period 仅在首拍/测量失败时作为兜底值，运行中由 DWT 实测周期覆盖 */
	imu_data.sample_period = 0.001f;
	/* Madgwick 的重力修正增益 beta。
	 *
	 * beta 决定“估计多依赖加计、多依赖陀螺”：
	 *   beta 高 -> 向加计方向收敛快，但也会被加计的**相位滞后**拖住，
	 *             表现为“姿态估计跟不上飞机实际角度”；
	 *   beta 低 -> 以快速无滞后的陀螺积分为主，跟随快，
	 *             零偏与温漂靠加计慢慢纠正。
	 *
	 * 历史：曾把 beta 提到 0.4 / 0.6 / 1.0，是为了硬压“加计零偏导致的 -2° 偏差”。
	 * 但那个零偏已在**源头**修复（加计零偏改用每次上电实测，
	 * BB:ACC 已显示 tilt ≈ 0.02°），不再需要用高 beta 硬压。
	 * 高 beta 与加计的 26Hz 低通滞后叠加，反而让估计变慢。
	 *
	 * 现选 0.35：以陀螺为主（快），加计只做慢速修正。
	 * 若长时间飞行后姿态慢慢偏掉（陀螺温漂没被及时纠正）-> 升到 0.5；
	 * 若仍觉得跟不上 -> 降到 0.25。
	 * 现在这个系数由参数表 ahrs.beta 给，重启生效。 */
	imu_data.beta = Param_GetApplied(P_AHRS_BETA);
	
	Quaternion_Init(&imu_data.quat);
	
	/* 修复 3.1：加速度计三轴低通，必须在 UpdateData 之前初始化 */
	ICM42605_InitAccelLPF();
	/* 修复 2：以零偏 0 为初值启动在线跟踪，标定完成后会自动重置为标定值 */
	ICM42605_ResetBiasTracker(&imu_data);
	
	imu_init_ok = ICM42605_Init();
	if (imu_init_ok)
	{
		/* 校准：存过 cal.* 就直接用（不要求上电静止），否则退回 1 秒自标定。
		 * 判据"存过" = 六项里任意一项非 0；默认全 0 表示还没标过，
		 * 此时行为和加这几个参数之前完全一致。 */
		float cgx = Param_GetApplied(P_CAL_GYRO_X);
		float cgy = Param_GetApplied(P_CAL_GYRO_Y);
		float cgz = Param_GetApplied(P_CAL_GYRO_Z);
		float cax = Param_GetApplied(P_CAL_ACCEL_X);
		float cay = Param_GetApplied(P_CAL_ACCEL_Y);
		float caz = Param_GetApplied(P_CAL_ACCEL_Z);

		if ((fabsf(cgx) + fabsf(cgy) + fabsf(cgz) +
			 fabsf(cax) + fabsf(cay) + fabsf(caz)) > 1e-6f)
		{
			ICM42605_SetCalibration(&imu_data, cgx, cgy, cgz, cax, cay, caz);
		}
		else
		{
			ICM42605_Calibrate(&imu_data, 1000);
		}
	}
//	ICM42605_SixSideCalibrate(&imu_data, 1000);
	GREEN_LIGHT_OFF;
	TickType_t last_tick;
	uint32_t last_imu_us = DWT_GetUS();
		
  /* Infinite loop */
	for(;;)
	{
		last_tick = xTaskGetTickCount();

		/* 控制台请求的重新标定（CFG:CAL）：在本任务里做，避免和采样抢 SPI。
		 * 约 2 秒；解锁中会被 ICM42605_ServiceCalibration 直接拒绝。
		 * 做完这一拍 dt 会偏大，下面 200us~5ms 的窗口会把它丢掉，不会污染姿态。 */
		(void)ICM42605_ServiceCalibration(&imu_data, BlackBox_IsArmed());

		/* ── 用 DWT 实测相邻两次 IMU 采样的真实间隔作为姿态解算 dt ──
		 * 原实现把 sample_period 写死 0.001f，但 vTaskDelayUntil 只保证"不早于 1ms"，
		 * 真实周期取决于本任务耗时（SPI DMA + 滤波 + 队列），偏差会让陀螺积分刻度错 k 倍。
		 * 这里在采样点测量，保证 dt 与这份陀螺数据严格配对。 */
		{
			uint32_t now_us = DWT_GetUS();
			uint32_t dt_us  = now_us - last_imu_us;

			/* 限幅：0.2ms(5kHz) ~ 5ms(200Hz)。窗口外(任务被长时间打断等异常拍)
			 * 不更新 dt、也不推进时间戳，避免异常值污染下一拍的测量。 */
			if (dt_us >= 200U && dt_us <= 5000U)
			{
				last_imu_us = now_us;
				imu_data.sample_period = (float)dt_us * 0.000001f;

				/* ── 抖动统计（诊断用，只累加）──
				 * 这三行用来回答"黑匣子的 Flash 操作有没有拖慢 IMU 采样"。
				 * 只看有效窗口内的样本，避免启动/异常拍干扰。 */
				if (dt_us > ulImuMaxUs)    { ulImuMaxUs = dt_us; }
				if (dt_us > 1500U)         { ulImuOver1500++; }
				if (dt_us > 2500U)         { ulImuOver2500++; }
			}
		}

		ICM42605_UpdateData(&imu_data);
		imu_data.filtered_gyro_x = LowPass_Update(&gyro_filter_lpf_x,imu_data.gyro_x);
		imu_data.filtered_gyro_y = LowPass_Update(&gyro_filter_lpf_y,imu_data.gyro_y);
		imu_data.filtered_gyro_z = LowPass_Update(&gyro_filter_lpf_z,imu_data.gyro_z);
		imu_data.filtered_vert_accel = LowPass_Update(&accel_filter_vert,imu_data.vert_accel);
		
		imu_data.filtered_gyro_x = Notch_Update(&gyro_filter_notch_x,imu_data.filtered_gyro_x);
		imu_data.filtered_gyro_y = Notch_Update(&gyro_filter_notch_y,imu_data.filtered_gyro_y);
		imu_data.filtered_gyro_z = Notch_Update(&gyro_filter_notch_z,imu_data.filtered_gyro_z);
		imu_data.filtered_vert_accel = Notch_Update(&accel_filter_notch_vert,imu_data.filtered_vert_accel);
		
		IMU_Packet_t pkt;
		pkt.roll         = imu_data.angle.roll;
		pkt.pitch        = imu_data.angle.pitch;
		pkt.yaw          = imu_data.angle.yaw;
		pkt.gyro_x       = imu_data.filtered_gyro_x;
		pkt.gyro_y       = imu_data.filtered_gyro_y;
		pkt.gyro_z       = imu_data.filtered_gyro_z;
		pkt.accel_x      = imu_data.accel_x;
		pkt.accel_y      = imu_data.accel_y;
		pkt.accel_z      = imu_data.accel_z;
		pkt.filtered_vert_accel      = imu_data.filtered_vert_accel;
		pkt.tilt_total   = imu_data.tilt_total;
		pkt.sample_period= imu_data.sample_period;
		HeightEstimator_Predict(&est,imu_data.filtered_vert_accel,imu_data.sample_period);
		
			vofa_data_frame.data1 = imu_data.gyro_x;
			vofa_data_frame.data2 = imu_data.filtered_gyro_x;
//			vofa_data_frame.data3 = est.altitude;
//			HAL_UART_Transmit(&huart3,(uint8_t *)&vofa_data_frame,sizeof(vofa_data_frame),100);
		
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
		uint32_t no_rx_count = 0;
		uint8_t total_rx_count = 0;
		uint8_t successful_rx_count = 0;

		Rx_Frame_t rx_local;
		IMU_Packet_t usart_imu;
		float usart_alt;
		HAL_ADC_Start(&hadc1);

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

//				Throttle = rx_local.throttle - 1000;
				no_rx_count = 0;
				RED_LIGHT_OFF;
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

				RED_LIGHT_BLINK;
				HAL_UART_DMAStop(&huart1);
				HAL_UART_Receive_DMA(&huart1, Usart1DmaBuf, RX_FRAME_SIZE);
				no_rx_count = 0;
			}

			/* ── 3. Read battery ADC ── */
			HAL_ADC_Start(&hadc1);
			if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK)
			{
				uint32_t adc_raw = HAL_ADC_GetValue(&hadc1);
				float bat_pct = ((float)adc_raw - BATTERY_MIN_VOLTAGE) / SCALING_FACTOR;
				if (bat_pct < 0.0f) bat_pct = 0.0f;
				if (bat_pct > 100.0f) bat_pct = 100.0f;
				tx_frame_buf.bat = (uint16_t)bat_pct;
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

			/* ── 6. Transmit telemetry (every 2 cycles), DMA-safe ── */
			if(total_rx_count % 2 == 0)
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
		osDelay(4200);
		TickType_t last_tick;
		/* FlyControl 循环周期测量基准（诊断用，见 ulFcMaxUs 等注释） */
		uint32_t last_fc_us = DWT_GetUS();
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_3,0);
		__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,0);

		FlyController_Init(&fc);

		/* ── 用参数表覆盖编译期默认值（PID 增益等）──
		 * 只在启动时调用一次：飞行中改参数不会影响正在跑的控制环，
		 * 想让它生效必须 CFG:SAVE + 重启。 */
		FlyController_ApplyGains(&fc);

		/* 恢复上次离地时学到并落盘的悬停油门（alt.hover_throttle，0 = 还没存过）。
		 * 有了它，每次上电第一次切定高就已经是准的，不用再等 1.5s 学习。
		 * 同时读回"学习时的参考点"（alt.ref_throttle/alt.ref_pct）：
		 * 只有它俩才带电量信息，能处理"换电池"这种场景。 */
		FlyController_SetHoverReference(&fc,
		                                Param_GetApplied(P_ALT_REF_THR),
		                                Param_GetApplied(P_ALT_REF_PCT));
		FlyController_SeedHoverThrottle(&fc, Param_GetApplied(P_ALT_HOVER_THR));

		/* 下面几项每个控制周期都要用，缓存到局部变量：
		 * 一是省掉每拍的查表，二是把"重启才变"这件事固化下来。 */
		const float cfg_stick_deg  = Param_GetApplied(P_STICK_DEG);
		const float cfg_yaw_scale  = Param_GetApplied(P_STICK_YAW_SCALE);
		const float cfg_roll_off   = Param_GetApplied(P_CTRL_ROLL_OFF);
		const float cfg_pitch_off  = Param_GetApplied(P_CTRL_PITCH_OFF);
		const float cfg_thr_scale  = Param_GetApplied(P_THR_SCALE);
		const float cfg_comp_a     = Param_GetApplied(P_THR_COMP_A);
		const float cfg_comp_b     = Param_GetApplied(P_THR_COMP_B);
		const float cfg_alt_base   = Param_GetApplied(P_ALT_STICK_BASE);
		const float cfg_alt_range  = Param_GetApplied(P_ALT_STICK_RANGE);
		const float cfg_land_timeout_s = Param_GetApplied(P_ALT_LAND_TIMEOUT);   /* 降落超时(s) */

		/* 手动油门曲线的四个断点。参数写坏（x1>=x2 / y1>=y2）就整体退回线性，
		 * 绝不能让一个坏参数把油门锁死。 */
		float cfg_cv_x1 = Param_GetApplied(P_THR_CV_X1);
		float cfg_cv_y1 = Param_GetApplied(P_THR_CV_Y1);
		float cfg_cv_x2 = Param_GetApplied(P_THR_CV_X2);
		float cfg_cv_y2 = Param_GetApplied(P_THR_CV_Y2);

		if (!((cfg_cv_x1 > 1050.0f) && (cfg_cv_x2 < 1950.0f) &&
		      (cfg_cv_x2 > cfg_cv_x1 + 20.0f) &&
		      (cfg_cv_y1 >= 0.0f) && (cfg_cv_y2 <= 1.0f) &&
		      (cfg_cv_y2 > cfg_cv_y1 + 0.005f)))
		{
			cfg_cv_x1 = 1500.0f;
			cfg_cv_y1 =  0.50f;
			cfg_cv_x2 = 1700.0f;
			cfg_cv_y2 =  0.70f;
		}

		/* 黑匣子记录总开关（参数 bb.recording，重启生效）。
		 * 关掉之后 FlyControl 这边的 Queue 调用会直接返回，Flash 一点不碰。 */
		BlackBox_SetRecording((Param_GetApplied(P_BB_RECORDING) >= 0.5f) ? 1U : 0U);

		/* Local copies updated from queues (persist across cycles) */
		ICM42605_Data imu_local;
		memset(&imu_local, 0, sizeof(imu_local));
		imu_local.sample_period = 0.001f;

		RC_Packet_t current_rc = {0};
		float current_alt = 0.0f;
		float current_vert_velocity = 0.0f;
		/* 模式确认（见 MODE_CONFIRM_MS 的说明）：mode_raw 是刚看到的那一档，
		 * mode_eff 是"连续保持够久、真正生效"的那一档。默认手动最安全。 */
		/* 降落地状态机的计时（1kHz 调用，计数即毫秒） */
		uint16_t land_ms  = 0;
		uint16_t touch_ms = 0;
		uint16_t touch_raw_ms = 0;      /* 不看气压门控的原始触地计时（升级出口用） */
		uint16_t impact_ms    = 0;      /* 加计冲击连续计数（判据 0） */
		/* 触地判据用：垂速历史（20Hz × 8 = 400ms）+ 气压跳变冻结计时 */
		float    vvel_hist[VVEL_HIST_N] = {0};
		uint8_t  vvel_hist_i    = 0U;
		float    baro_guard_last = 0.0f;
		uint8_t  baro_guard_have = 0U;
		uint16_t baro_guard_ms   = 0U;
		uint16_t mode_raw = 0xFFFFU;
		uint16_t mode_eff = MODE_HORIZON;
		TickType_t mode_since = 0;
		/* 摇杆帧的分频计数（每 BB_RC_INTERVAL 帧记一次） */
		static uint32_t bb_rc_divider = 0;
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
				imu_local.accel_x        = imu_pkt.accel_x;
				imu_local.accel_y        = imu_pkt.accel_y;
				imu_local.accel_z        = imu_pkt.accel_z;
				imu_local.vert_accel        = imu_pkt.filtered_vert_accel;
				imu_local.tilt_total     = imu_pkt.tilt_total;
				imu_local.sample_period  = imu_pkt.sample_period;
			}

			/* ── 2. Read latest RC packet from queue (non-blocking) ── */
			RC_Packet_t rc_pkt;
			if (osMessageQueueGet(rcFrameQueueHandle, &rc_pkt, NULL, 0) == osOK)
			{
				current_rc = rc_pkt;
			}

			/* ── 2b. 模式确认：消抖 + "扫过中间位"保护 ──
			 * 三位开关从手动(2000)切定高(1000)必然经过降落(1500)。
			 * 只有某一档连续保持 MODE_CONFIRM_MS 才生效，中间的过渡帧被忽略。 */
			{
				uint16_t mode_cls = FlyControl_ClassifyMode(current_rc.mode);

				if (mode_cls != mode_raw)
				{
					mode_raw   = mode_cls;
					mode_since = xTaskGetTickCount();
				}
				else if ((xTaskGetTickCount() - mode_since) >= pdMS_TO_TICKS(MODE_CONFIRM_MS))
				{
					mode_eff = mode_cls;
				}
			}

			/* ── 3. Read latest altitude from queue (non-blocking) ── */
			float alt_val;
			if (osMessageQueueGet(altFlyQueueHandle, &alt_val, NULL, 0) == osOK)
			{
				static TickType_t alt_last_tick = 0;
				TickType_t alt_now = xTaskGetTickCount();

				current_alt = alt_val;
				fc.altitude_ready = 1;

				/* 高度样本的实际间隔 → 定高 PID 的 dt（见 flycontrol.h 的 alt_dt 说明）。
				 * 气压读取周期是 50ms，但会受任务调度影响而抖动，所以实测后给环用。 */
				if (alt_last_tick != 0)
				{
					uint32_t d_ms = (uint32_t)(alt_now - alt_last_tick);

					if ((d_ms >= 5U) && (d_ms <= 200U))
					{
						fc.alt_dt = (float)d_ms * 0.001f;
					}
				}
				alt_last_tick = alt_now;

				/* ── 高度帧（50Hz，跟着气压样本走）──
				 * 与数据帧同一个生产者(本任务)，所以不会破坏 RAM 环形的单写者假设。
				 * 内容见 blackbox.h 的 BB_TYPE_ALT 槽位表。
				 * alt_view 用 static：本任务栈只有 1KB，别为 40 字节的快照冒险。 */
				{
					static HeightEstimator_View alt_view;
					float baro_now;

					HeightEstimator_GetView(&est, &alt_view);
					BlackBox_QueueAlt(&alt_view, fc.altitude_output,
					                  fc.hover_throttle, fc.hover_valid);

					/* ── 气压卫生（降落触地判据用）──
					 * 桨洗气流撞地会在 50ms 内让气压"跳" 1~2m（等效 >20m/s），
					 * 融合高度会被拖着走 —— 这时候的"高度很小"绝不能当触地证据。
					 * 一旦发现非物理跳变，就冻结触地判据 400ms。 */
					baro_now = alt_view.baro_altitude;
					if (baro_guard_have != 0U)
					{
						float dtb = (alt_view.baro_dt > 0.005f) ? alt_view.baro_dt : 0.05f;

						if ((fabsf(baro_now - baro_guard_last) / dtb) > BARO_JUMP_RATE_MAX)
						{
							baro_guard_ms = BARO_JUMP_HOLDOFF_MS;
						}
					}
					baro_guard_last = baro_now;
					baro_guard_have = 1U;

					/* 垂速历史（20Hz × 8 = 400ms）：判断"刚才在下降、现在停了" */
					vvel_hist[vvel_hist_i] = alt_view.velocity;
					vvel_hist_i = (uint8_t)((vvel_hist_i + 1U) % VVEL_HIST_N);
				}
			}
			else
			{
				fc.altitude_ready = 0;
			}

			/* 气压跳变冻结计时（1kHz 递减） */
			if (baro_guard_ms > 0U)
			{
				baro_guard_ms--;
			}

			/* ★ 高度数据源健康度：0 时禁止定高（见 flycontrol.c 的说明）。
			 * 注意它和 altitude_ready 的区别：后者只看"这一拍有没有新样本"。 */
			fc.alt_source_ok = g_alt_ok;
			
			float vert_vel_val;
			if (osMessageQueueGet(vertVelocityQueueHandle, &vert_vel_val, NULL, 0) == osOK)
			{
				current_vert_velocity = vert_vel_val;
			}

			/* ── 4. Compute control targets ──
			 * 摇杆灵敏度、微调偏置、油门曲线全部来自参数表（重启生效） */
			float target_roll  =  (current_rc.roll - 1500.0f) / cfg_stick_deg + (current_rc.roll_offset - 1500.0f) / cfg_stick_deg + cfg_roll_off;
			float target_pitch =  (current_rc.pitch - 1500.0f) / cfg_stick_deg + (current_rc.pitch_offset - 1500.0f) / cfg_stick_deg + cfg_pitch_off;

			/* ── 阶跃注入（BB:STEP）──
			 * 测闭环阶跃响应用。只在已解锁、且摇杆回中时才叠加；
			 * 飞手一动摇杆它会立即自动取消（实现在 BlackBox_GetStepOffset 里）。
			 * 注意：目标角本身会被记进摇杆帧，所以上位机能看到真实的设定值。 */
			{
				float step_roll_off = 0.0f;
				float step_pitch_off = 0.0f;

				/* 触发条件：已解锁 + 油门已抬起（真的在飞）+ 姿态平稳。
				 * 摇杆回中的判定在 BlackBox_GetStepOffset 里做。
				 * 这样地面上下的命令不会在手持/地面状态下开始摆动，
				 * 而是等你真正悬停稳了才自动开始。 */
				uint8_t step_ready = 0U;
				if ((current_rc.arm == ARM_UNLOCKED) &&
				    (current_rc.throttle >= 1300U) &&
				    ((imu_data.tilt_total * RAD_TO_DEG) < 12.0f))
				{
					step_ready = 1U;
				}

				BlackBox_GetStepOffset(&step_roll_off, &step_pitch_off,
				                       current_rc.roll, current_rc.pitch,
				                       step_ready);
				target_roll  += step_roll_off;
				target_pitch += step_pitch_off;
			}
			float target_yaw   =   (current_rc.yaw - 1500.0f) * cfg_yaw_scale;
			float target_throttle;

			FlyController_SetCurrentAltitude(&fc, current_alt);
			/* 电量每拍喂给控制器：电压补偿 + 悬停油门学习都要用 */
			FlyController_SetBatteryPercent(&fc, current_rc.battery_percent);

			/* 未解锁时按当前电量折算悬停油门基准（换电池的场景）。
			 * 解锁之后不再插手：那时由飞行中的学习器自己收敛，改基准反而会扰动。
			 * 地面上的电量是"不带载"的值，换完电池它立刻就是新电量，所以这里
			 * 每次上电/换电池都能把基准纠正到当前这块电池应有的值。 */
			if (current_rc.arm != ARM_UNLOCKED)
			{
				FlyController_UpdateHoverForBattery(&fc);
			}

			/* ★ 用确认后的模式（mode_eff）而不是遥控原始值：
			 * 三位开关切换过程中会短暂经过"降落"，那段时间必须当作**没换模式**。 */
			if(mode_eff == MODE_ALT_HOLD)
			{
				/* 落地（LANDED）之后拨到定高 = 重新起飞。这是飞手的明确意图
				 * （arm 是独立通道，拨模式开关不会误碰），所以这里不再设"油门杆
				 * 必须推过 1150"之类的门槛 —— 曾经加过，结果落地后切回定高电机
				 * 完全不转，被当成故障。不想再起飞就 disarm。 */
				FlyController_SetTargetAltitude(&fc, (cfg_alt_base + ((current_rc.throttle - 1000.0f) / cfg_alt_range)));
				FlyController_SetCurrentVerticalVelocity(&fc,current_vert_velocity);
				fc.flying_status = ALTITUDE_HOLD_FLYING;
				BLUE_LIGHT_ON;

				if (fc.alt_source_ok != 0)
				{
					/* ★ 基准油门用**学到的悬停油门**（还没学会时自动退回电压补偿）。
					 * 原来固定用 Throttle_Voltage_Compensation()，比这台机的真实悬停
					 * 油门低约 26%，缺口只能靠定高积分长期扛着 —— 结果是上爬没余量
					 * （+600 限幅里已占 +443）、起飞快不起来、外环被积分主导变得迟钝。
					 * 用学习值之后，悬停时修正量应落在 0 附近，上下余量对称。 */
					target_throttle = FlyController_GetHoverBase(&fc);
				}
				else
				{
					/* ★ 气压高度不可用（没装/初始化失败/连续 200ms 无有效样本）：
					 * 定高修正量已被 flycontrol.c 清零，这里把油门**交还飞手**。
					 * 原来无论数据有没有都走 Throttle_Voltage_Compensation()——那是个
					 * 只跟电压有关的常数，等于油门锁死、打杆无效，只能 disarm。 */
					/* 交还飞手时同样走手动曲线，保证和 HORIZON 手感一致 */
					float cv_u = FlyControl_ThrottleCurve(current_rc.throttle,
					                                      cfg_cv_x1, cfg_cv_y1,
					                                      cfg_cv_x2, cfg_cv_y2);
					target_throttle = cv_u * 1000.0f * cfg_thr_scale * 4.2f
					                  / ((float)current_rc.battery_percent * cfg_comp_a + cfg_comp_b);
				}
			}
			else if(mode_eff == MODE_HORIZON)
			{
				/* 手动模式走油门曲线：摇杆 1000..2000 → 输出比例 0..1，再乘原量程。
				 * 曲线只作用于手动（定高/降落用的是目标高度，不吃这条曲线）。 */
				float cv_u = FlyControl_ThrottleCurve(current_rc.throttle,
				                                      cfg_cv_x1, cfg_cv_y1,
				                                      cfg_cv_x2, cfg_cv_y2);

				target_throttle = cv_u * 1000.0f * cfg_thr_scale * 4.2f
				                  / ((float)current_rc.battery_percent * cfg_comp_a + cfg_comp_b);

				fc.flying_status = HORIZON_FLYING;
				BLUE_LIGHT_OFF;
			}
			else
			{
				/* ══════════════════════════════════════════════════════════════════
				 *  MODE_LANDING —— 降落（小状态机）
				 *
				 *  旧实现的三个问题：
				 *   ① 从不设置 flying_status：从**手动**切进来时它还是 HORIZON_FLYING，
				 *      flycontrol 会因此把定高修正量清零 → 油门只剩
				 *      Throttle_Voltage_Compensation（928~1458 PWM，比悬停 1561 亏
				 *      100~633）→ 从 2m 以 3.8 m/s 砸地，而且电量越满砸得越狠。
				 *   ② 下降率不受控，由"基准亏多少"决定，满电/没电手感完全不同。
				 *   ③ 触地判据要求 |垂速|<0.3，而 alt.rate_max 是 2.0 →
				 *      要等撞地之后垂速才会掉下来，等于"撞了才断电"。
				 *
				 *  现在：
				 *   · 基准用**学到的悬停油门**（和定高一致），修正量只负责"降多快"；
				 *   · flying_status = LANDING，下降率由 flycontrol.c 的 flare 剖面
				 *     接管（离地越低越慢，落地速度 ≈ alt.land_flare）；
				 *   · 目标高度贴当前高度 → 外环误差≈0，积分不 windup；
				 *   · 触地三选二 + 250ms 确认；另有超时兜底，避免无限往地里顶。
				 * ══════════════════════════════════════════════════════════════════ */
				FlyController_SetCurrentVerticalVelocity(&fc, current_vert_velocity);

				if (fc.flying_status == LANDED)
				{
					/* 已经断电：保持，直到飞手换模式或 disarm。
					 * （不能在这里重新置 LANDING，否则触地后下一拍又被拉回飞行状态。） */
					target_throttle = 0.0f;
					land_ms      = 0;
					touch_ms     = 0;
					touch_raw_ms = 0;
					impact_ms    = 0;
				}
				else
				{
					if (fc.flying_status != LANDING)
					{
						land_ms      = 0;      /* 刚切进来：计时归零 */
						touch_ms     = 0;
						touch_raw_ms = 0;
						impact_ms    = 0;
					}
					else if (land_ms < 0xFFFEU)
					{
						land_ms++;             /* 1kHz 调用，直接当毫秒 */
					}

					FlyController_SetTargetAltitude(&fc, current_alt);
					fc.flying_status = LANDING;

					if (fc.alt_source_ok != 0U)
					{
						uint8_t touch      = 0U;
						uint8_t reason     = 0U;
						uint8_t impact_hit = 0U;

						target_throttle = FlyController_GetHoverBase(&fc);

						/* ══ 判据 0：加计冲击（最可靠，2026-10-26 加）══
						 * 实测触地冲击 2.84~15.8 g（机架硬、飞控硬装），而全飞行空中
						 * 振动最大只有 1.66~1.99 g → 2.5 g 门槛两边都有余量。
						 * 这是唯一**直接测量碰地**的信号：气压被桨洗撞地打成 +0.28m/s
						 * 的假"上升"时它照样是 4.19 g 的尖峰，不受任何滤波/融合滞后影响。
						 * 26 日 21:47 那次就是靠这条才能解决：气压瞬态让垂速判据以为
						 * "没落地"，油门保持 → 地面效应把飞机顶到 0.4m → 反复弹跳。 */
						{
							float a2 = (imu_local.accel_x * imu_local.accel_x) +
							           (imu_local.accel_y * imu_local.accel_y) +
							           (imu_local.accel_z * imu_local.accel_z);

							if (a2 > (LAND_IMPACT_G * LAND_IMPACT_G))
							{
								if (impact_ms < 0xFFFEU) { impact_ms++; }
							}
							else
							{
								impact_ms = 0;
							}

							if (impact_ms >= LAND_IMPACT_CONFIRM_MS)
							{
								impact_hit = 1U;
							}
						}

						/* ══ 触地判据（2026-10-26 重写）══
						 * 旧版用 `高度<0.28 && |垂速|<0.25`：flare 最后一段目标下降率
						 * 就是 0.25，所以只要进到 0.28m 内这条几乎必然成立 →
						 * 10~20cm 空中断电、掉下来弹跳（实测日志 19.24s 断电时
						 * 估计高度 0.21m，断电后垂速冲到 -1.5m/s 自由落体）。
						 * 现在只用**与绝对高度无关**的动力学证据 + 气压卫生门控：
						 *   ① 垂速塌陷：400ms 内曾 < -0.15，现在 |垂速| < 0.08
						 *      （真接触时唯一的"硬约束"就是地面让速度归零，与参考面无关）
						 *   ② 高度兜底：估计高度 < 0.08 **且** 垂速也已经停住
						 *      （必须垂速也停了才算，否则"起降点不等高"时会提前断电）
						 * 两者都只受一个极松的高度门限（8m）和气压卫生门控约束：
						 * 最近 400ms 有气压跳变时**推迟**断电，但推迟不得超过 800ms。
						 *
						 * ⚠ "现在停了"用**最近 200ms 的平均垂速**判断，不用瞬时值：
						 * 26 日 21:37 那次飞机已经趴在地面上，融合垂速却在 -0.04~-0.13
						 * 之间抖（噪声 8.9cm/s，而旧门限 0.08 只有 1σ），判据忽真忽假 →
						 * 200ms 确认不断清零、油门在 1315/1563 之间来回跳，磨了 1 秒才断电。
						 * 4 样本平均把噪声降到 4.5cm/s，门限放宽到 0.12 后不再抖；
						 * 而空中 flare 底线是 0.25m/s，平均值 0.25 ≫ 0.12，不会误判。 */
						if (current_alt < LAND_TOUCH_ALT_GATE)
						{
							float vmin = vvel_hist[0];
							float vavg = 0.0f;
							uint8_t k;

							for (k = 1U; k < VVEL_HIST_N; k++)
							{
								if (vvel_hist[k] < vmin) { vmin = vvel_hist[k]; }
							}

							/* 最近 4 个样本（20Hz → 200ms）的平均垂速 */
							for (k = 1U; k <= LAND_VVEL_AVG_N; k++)
							{
								vavg += vvel_hist[(uint8_t)((vvel_hist_i + VVEL_HIST_N - k) % VVEL_HIST_N)];
							}
							vavg /= (float)LAND_VVEL_AVG_N;

							if ((vmin < LAND_COLLAPSE_FROM) &&
							    (fabsf(vavg) < LAND_COLLAPSE_TO))
							{
								touch  = 1U;   /* 刚才在下降，现在停了 → 触地 */
								reason = 2U;
							}
							else if ((current_alt < LAND_TOUCH_ALT_MIN) &&
							         (fabsf(vavg) < LAND_COLLAPSE_TO))
							{
								touch  = 1U;   /* 贴地高度兜底（垂速也必须停住） */
								reason = 1U;
							}
						}

						/* 不带门控的原始条件计时：用来做"升级出口" */
						if (touch != 0U)
						{
							if (touch_raw_ms < 0xFFFEU) { touch_raw_ms++; }
						}
						else
						{
							touch_raw_ms = 0;
						}

						/* ── 确认窗内先把油门让下来 ──
						 * 触地条件刚成立时还不能断电（要等 200ms 确认，防误切），
						 * 但这时候地面已经在托着飞机了。实测 26 日 21:28 那次：
						 * 11.2s 就落地，可油门在 1520~1560 又挂了 0.3 秒，而地面效应
						 * 悬停只需要约 1300（悬停 1536×0.85）—— 多出来的推力把飞机
						 * 顶起来"弹了一下"，用户感觉就是落地不干脆。
						 * 所以判到塌陷就立刻把油门压到悬停的 85%：足以贴住地面、不至于被顶起来。
						 * 这只是"少给"，不断电、不切状态；万一真是空中误判，定高环 0.2 秒内自己补回来。 */
						if (touch != 0U)
						{
							float unload = FlyController_GetHoverBase(&fc) * LAND_TOUCH_UNLOAD_RATIO;

							if ((target_throttle > unload) && (unload > 0.0f))
							{
								target_throttle = unload;
							}
						}

						/* 气压被桨洗撞地扰动时**不允许断电**：宁可晚 0.4 秒，
						 * 也不能在 20cm 高度把动力断掉。注意这只是"推迟" —— 见下面的升级出口。 */
						if (baro_guard_ms != 0U)
						{
							touch = 0U;
						}
						/* ↑ 加计冲击**不受**门控影响（见下面的分支）：它是直接测量，
						 *   桨洗撞地造成的恰恰就是气压瞬态，用气压去"否决"冲击检测
						 *   等于把唯一可靠的证据也一起否掉。 */

						if (touch != 0U)
						{
							if (touch_ms < 0xFFFEU) { touch_ms++; }
						}
						else
						{
							touch_ms = 0;
						}

						if ((impact_hit != 0U) ||
						    (touch_ms >= LAND_TOUCH_CONFIRM_MS) ||
						    (touch_raw_ms >= LAND_TOUCH_ESCALATE_MS))
						{
							fc.flying_status = LANDED;

							/* 1=贴地高度 2=垂速塌陷 4=气压门控升级 5=加计冲击 */
							if (impact_hit != 0U)
							{
								fc.land_cut_reason = 5U;
							}
							else
							{
								fc.land_cut_reason = (touch_ms >= LAND_TOUCH_CONFIRM_MS) ? reason : 4U;
							}

							target_throttle = 0.0f;
						}
					}
					else
					{
						/* 高度数据不可用：温和盲降（亏悬停 15% ≈ 1.5 m/s²），
						 * 靠下面的超时断电收尾。此时任何"闭环"都是假的。 */
						target_throttle = FlyController_GetHoverBase(&fc) * 0.85f;
					}

					/* 超时兜底：气压漂移时可能永远等不到触地证据，不能一直往地里顶 */
					if ((fc.flying_status != LANDED) &&
					    (land_ms >= (uint16_t)(cfg_land_timeout_s * 1000.0f)))
					{
						fc.flying_status   = LANDED;
						fc.land_cut_reason = 3U;         /* 3=超时 */
						target_throttle    = 0.0f;
					}
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

				/* ★ 同步清掉"最近一次混控输出"：不这么做的话 fc.motor1..4 会一直
				 * 停在断电前的那组值（实测落地后黑匣子里还是 1554 PWM），
				 * 看日志会以为"落地了电机还在转"，把排查带偏。 */
				fc.motor1 = 0U;
				fc.motor2 = 0U;
				fc.motor3 = 0U;
				fc.motor4 = 0U;
			}

			/* 上锁瞬间：把这次学到的悬停油门 + 学习时的电量一起交给 Logger 落盘
			 * （写 Flash 归 Logger 独占，而且只在未解锁时才允许写 —— 见 BlackBox_LoggerTask）。
			 * 带上电量是为了下次换电池后能按电量折算回来，见 FlyController_UpdateHoverForBattery。 */
			if ((prev_arm_status == ARM_UNLOCKED) && (current_rc.arm != ARM_UNLOCKED) &&
			    (fc.hover_valid != 0U))
			{
				BlackBox_RequestHoverThrottleSave(fc.hover_throttle,
				                                  FlyController_GetBatteryPercent(&fc));
			}

			prev_arm_status = current_rc.arm;

			/* 修复 2：仅未解锁时允许在线学习陀螺零偏，防止飞行中把机动学成零偏 */
			ICM42605_SetArmed((current_rc.arm == ARM_UNLOCKED) ? 1 : 0);

			/* ── 6. 黑匣子：与上面同一份数据，保证记录内容与控制所用完全一致。
			 *     只做一次内存拷贝，绝不阻塞(Flash 写入由 BlackBox 任务完成)。 ── */
			BlackBox_SetArmed((current_rc.arm == ARM_UNLOCKED) ? 1 : 0);
			{
				uint16_t bb_motor[4];
				bb_motor[0] = fc.motor1;
				bb_motor[1] = fc.motor2;
				bb_motor[2] = fc.motor3;
				bb_motor[3] = fc.motor4;

				BlackBox_Queue(imu_local.angle.roll,
				               imu_local.angle.pitch,
				               imu_local.angle.yaw,
				               imu_local.gyro_x,
				               imu_local.gyro_y,
				               imu_local.gyro_z,
				               imu_local.accel_x,
				               imu_local.accel_y,
				               imu_local.accel_z,
				               current_alt,
				               current_vert_velocity,
				               imu_local.vert_accel,
				               bb_motor,
				               fc.throttle,
				               fc.altitude_ready,
				               (fc.flying_status == ALTITUDE_HOLD_FLYING) ? 1U : 0U,
				               (fc.mix_scale < 0.999f) ? 1U : 0U,
				               fc.land_cut_reason,
				               (uint8_t)current_rc.battery_percent);

				/* 每 32ms 再记一帧摇杆量 + 原始陀螺：
				 *  · 摇杆量用来区分"操作者在打杆修正"和"控制器把飞机推走"
				 *  · 原始陀螺用来对照数据帧里的滤波后值，判断恒定偏移
				 *    是传感器本身的还是滤波器引入的 */
				if ((imu_local.sample_period > 0.0f) &&
				    ((bb_rc_divider++ % BB_RC_INTERVAL) == 0U))
				{
					uint16_t rc4[4];
					rc4[0] = current_rc.roll;
					rc4[1] = current_rc.pitch;
					rc4[2] = current_rc.yaw;
					rc4[3] = current_rc.throttle;

					BlackBox_QueueRc(rc4, current_rc.arm, current_rc.mode,
					                 (uint8_t)fc.flying_status,
					                 /* 记**实际目标角**（送进控制器的设定值）。
					                  * 这样阶跃分析不需要从摇杆反推设定值，
					                  * 也不受接收机微调(roll_offset)影响；
					                  * BB:STEP 注入时摇杆没动，也能正确记下。 */
					                 fc.target_roll,
					                 fc.target_pitch,
					                 bb_motor,
					                 (uint8_t)current_rc.battery_percent);
				}
			}

			/* ── FlyControl 循环周期抖动统计（诊断用，只累加）──
			 * 这是"控制环有没有被阻塞"的直接证据：
			 * 若这里出现 >1500µs 的周期，说明本任务确实被拖过一拍。 */
			{
				uint32_t now_us = DWT_GetUS();
				uint32_t fc_dt  = now_us - last_fc_us;

				/* 忽略启动阶段的超长间隔(>20ms)，那只是任务刚起来 */
				if (fc_dt < 20000U)
				{
					last_fc_us = now_us;
					if (fc_dt > ulFcMaxUs) { ulFcMaxUs = fc_dt; }
					if (fc_dt > 1500U)     { ulFcOver1500++; }
					if (fc_dt > 2500U)     { ulFcOver2500++; }
				}
				else
				{
					last_fc_us = now_us;
				}
			}

			vTaskDelayUntil(&last_tick, 1);
	  }
  /* USER CODE END StartFlyControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE BEGIN Header_StartBlackBoxTask */
/**
* @brief 黑匣子任务：独占 W25Q128(SPI3) 与 USART3 指令通道。
*        优先级最低，Flash 访问全部走中断模式 SPI 并主动 delay，
*        因此绝不会拖慢 1kHz 控制环。进入后不再返回。
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlackBoxTask */
void StartBlackBoxTask(void *argument)
{
  /* USER CODE BEGIN StartBlackBoxTask */
	/* USART3 挂 DMA 环形接收：上位机指令通道 */
	USART3_DMA_Rx_Init();

	BlackBox_Init();

	/* 交给黑匣子模块的主循环：内部自行读取 USART3、执行指令、写 Flash */
	BlackBox_LoggerTask(argument);
  /* USER CODE END StartBlackBoxTask */
}

/* ==================== 栈溢出钩子 ====================
 * 对应 FreeRTOSConfig.h 里 configCHECK_FOR_STACK_OVERFLOW = 2。
 * 原来没有开启检测：任务栈用过头只会悄悄破坏相邻内存，
 * 表现就是"偶发、部分的功能异常"（比如控制环有点不受控）。
 *
 * 这里**不停机**——飞行中停机比继续更危险——只记录信息并点灯。
 *
 * 调试时在 Keil 的 Watch 窗口加这三个变量即可判断：
 *   ulStackOverflowCount  溢出次数（>0 就说明确实溢出过）
 *   pcOverflowTaskName    最后溢出的任务名（"FlyControl" / "IMURead" / ...）
 *   xOverflowTaskHandle   对应任务句柄
 * PC13 会被拉低点灯（板上 LED 一般低电平点亮）。
 */
volatile uint32_t       ulStackOverflowCount = 0;
volatile char          *pcOverflowTaskName   = NULL;
volatile TaskHandle_t   xOverflowTaskHandle  = NULL;

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
	ulStackOverflowCount++;
	xOverflowTaskHandle = xTask;
	pcOverflowTaskName  = pcTaskName;   /* 指向任务名，Watch 里可直接看字符串 */

	RED_LIGHT_ON;
}

/* USER CODE END Application */

