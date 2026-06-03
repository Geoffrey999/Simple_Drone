# F405 Drone Flight Controller

基于 STM32F405 的四轴飞行器飞控固件，运行 FreeRTOS 实时操作系统。

## 硬件平台

- **MCU**: STM32F405RGT6 (Cortex-M4F, 168MHz)
- **IMU**: TDK ICM-42605 (6 轴陀螺仪 + 加速度计) — SPI 接口
- **气压计**: Bosch BMP280 — 软件 I2C 接口
- **电池监测**: 板载 ADC 分压采样
- **电机输出**: TIM3 四通道 PWM 
- **遥控器输入**: USART1 — DMA 接收

## 软件架构

### 实时操作系统

FreeRTOS + CMSIS-RTOS2 封装，共 4 个任务：

| 任务 | 优先级 | 周期 | 功能 |
|------|--------|------|------|
| **IMURead** | High | 1 kHz | 读取 ICM-42605，运行 Madgwick 姿态解算 |
| **FlyControl** | Normal | 1 kHz | 级联 PID 控制 + 电机混控 |
| **Usart** | Low | ~60 Hz | RC 信号解析、遥测下发、电池 ADC 读取 |
| **BaroRead** | Low | 50 Hz | 读取 BMP280 气压计，滑动平均滤波 |

### 任务间通信

- `rcFrameQueueHandle` — 遥控器数据队列 (RC → FlyControl)
- `imuFlyQueueHandle` — IMU 数据队列 (IMURead → FlyControl)
- `imuUsartQueueHandle` — IMU 遥测队列 (IMURead → Usart)
- `altFlyQueueHandle` — 高度数据队列 (BaroRead → FlyControl)
- `altUsartQueueHandle` — 高度遥测队列 (BaroRead → Usart)
- `usart1RxStreamBuf` — UART DMA 接收流缓冲区 (ISR → Usart)
- `uart1TxSem` — UART DMA 发送互斥信号量

### 控制算法

**级联 PID 结构** — 外环角度环 + 内环角速率环：

```
目标角度 → [外环 PID] → 目标角速率 → [内环 PID] → 电机混控 → PWM
```

支持三种飞行模式：

- **ANGLE (自稳模式)**: 常规姿态自稳，摇杆控制角度
- **ALTITUDE_HOLD (定高模式)**: 基于气压计的高度锁定
- **LANDING/LANDED**: 自动降落与地面检测

附加功能：
- 电池电压补偿 — 随电压下降自动补偿油门
- 悬停油门自适应学习 — 稳态时跟踪维持悬停所需的油门值
- 倾斜角补偿 — 大倾斜角度时自动补偿推力损失

### 滤波器

- 滑动平均滤波器 (Moving Average)
- 一阶低通滤波器 (Low Pass)
- 二阶陷波滤波器 (Notch Filter)
- 一维卡尔曼滤波器 (Kalman Filter)
- 互补滤波器 (Complementary Filter)
- Alpha-Beta 滤波器

### 姿态解算

- **Madgwick AHRS** 算法 — 默认
- **Mahony AHRS** 算法 — 备选

## 通信协议

### 上行 (遥控 → 飞控) — 32 字节

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0 | head | 帧头 `0x4020` |
| 2 | roll | 横滚通道 |
| 4 | pitch | 俯仰通道 |
| 6 | throttle | 油门通道 |
| 8 | yaw | 偏航通道 |
| 10 | arm | 解锁/上锁 (`2000` = 解锁) |
| 12 | mode | 飞行模式 |
| 14-22 | PID 参数 | 在线调参支持 |
| 28 | roll_offset | 横滚中位偏移 |
| 30 | checksum | 校验和 |

### 下行 (飞控 → 地面站) — 24 字节

| 偏移 | 字段 | 说明 |
|------|------|------|
| 0 | head | 帧头 `0x4020` |
| 2 | bat | 电池百分比 |
| 4 | roll/pitch/yaw | 姿态角 |
| 16 | altitude | 高度 |
| 20 | rssi | 链路接收质量 |
| 22 | checksum | 校验和 |

## 开发环境

- **IDE**: Keil MDK-ARM v5
- **MCU 支持包**: STM32F4xx_DFP
- **HAL 驱动**: STM32F4xx_HAL_Driver
- **工具链**: ARMCC / ARMCLANG


## 工程结构

```
├── Core/                       # 应用代码
│   ├── Inc/                    # 头文件
│   │   ├── main.h
│   │   ├── flycontrol.h        # 飞控 PID 控制
│   │   ├── icm42605.h          # IMU 驱动
│   │   ├── bmp280.h            # 气压计驱动
│   │   ├── protocol.h          # 通信协议
│   │   ├── filter.h            # 数字滤波器
│   │   ├── soft_iic.h          # 软件 I2C
│   │   ├── dwt.h               # DWT 计时
│   │   └── FreeRTOSConfig.h
│   └── Src/
│       ├── main.c
│       ├── freertos.c          # 任务创建 & IPC
│       ├── flycontrol.c        # 级联 PID 实现
│       ├── icm42605.c          # IMU 驱动 & Madgwick/Mahony
│       ├── bmp280.c            # 气压计驱动 & 高度计算
│       ├── protocol.c          # 串口协议解析
│       ├── filter.c            # 各类滤波器实现
│       ├── soft_iic.c          # GPIO 模拟 I2C
│       ├── dwt.c               # DWT 微秒计时
│       └── ...
├── Drivers/                    # STM32 HAL / CMSIS
├── Middlewares/                 # FreeRTOS
├── MDK-ARM/                    # Keil 项目文件
└── f405.ioc                    # CubeMX 配置
```


## License

本项目基于 MIT 协议开源。
