/**
  ******************************************************************************
  * @file    blackbox.c
  * @brief   W25Q128 黑匣子实现：SPI3 驱动 + 1kHz 记录 + 串口指令/取数
  *
  * 硬件连接
  *   W25Q128JVSIQ (16MB, 4KB 扇区, 256B 页)
  *     NSS  PA15 / SCK PC10 / MISO PC11 / MOSI PC12  ---> SPI3
  *   指令与取数通道: USART3 (PB10/PB11, 与 printf 共用, 默认 460800 8N1)
  *     ※ 不占用 USART1(遥控接收, 数据是定长二进制帧, 不能挤占)
  *
  * 分工
  *   FlyControl (1kHz) ---> BlackBox_Queue()    只做一次内存拷贝, 永不阻塞
  *   Logger     (低优先级) ---> BlackBox_LoggerTask()  攒满一页(8帧)再写 Flash
  *   Usart3     (更低)    ---> BlackBox_Poll()  处理上位机指令
  ******************************************************************************
  */

#include "blackbox.h"
#include "icm42605.h"
#include "spi.h"
#include "usart.h"
#include "flash_ee.h"
#include "param.h"
#include "dwt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

extern SPI_HandleTypeDef hspi3;
extern UART_HandleTypeDef huart3;

/* 全局 IMU 数据实例（定义在 freertos.c）：BB:CAL 需要读上电标定的零偏 */
extern ICM42605_Data imu_data;

/* 时序抖动统计（定义在 freertos.c，由 IMU / FlyControl 任务累加）。
 * 只读，黑匣子不修改它们。 */
extern volatile uint32_t ulImuMaxUs;
extern volatile uint32_t ulImuOver1500;
extern volatile uint32_t ulImuOver2500;
extern volatile uint32_t ulFcMaxUs;
extern volatile uint32_t ulFcOver1500;
extern volatile uint32_t ulFcOver2500;

static void bb_dump_range(uint32_t start_sector, uint32_t count_sectors, uint8_t is_empty);
static void bb_dump_used(void);
static void bb_send(const char *s);
static uint8_t bb_begin_sector(void);
static void bb_do_clk(void);

/* ========================= W25Q128 指令集 ========================= */
#define W25_CMD_WRITE_ENABLE      0x06U
#define W25_CMD_READ_STATUS1      0x05U
#define W25_CMD_READ_DATA         0x03U
#define W25_CMD_PAGE_PROGRAM      0x02U
#define W25_CMD_SECTOR_ERASE      0x20U
#define W25_CMD_CHIP_ERASE        0xC7U
#define W25_CMD_JEDEC_ID          0x9FU
#define W25_CMD_RELEASE_POWERDOWN 0xABU

#define W25_STATUS1_BUSY          0x01U
#define W25_STATUS1_WEL           0x02U

#define W25_JEDEC_WINBOND         0xEFU
#define W25_JEDEC_CAPACITY_128M   0x18U

#define W25_SPI_TIMEOUT_MS        100U
#define W25_PAGE_PROGRAM_MS       10U
#define W25_SECTOR_ERASE_MS       600U
#define W25_CHIP_ERASE_MS         300000U   /* 全片擦除手册最长 200s，留足余量 */

#define BB_FLASH_TOTAL_BYTES      (BB_SECTOR_SIZE * BB_SECTOR_COUNT)
#define BB_TOTAL_FRAMES           (BB_SECTOR_COUNT * BB_FRAMES_PER_SECTOR)

/* dump 时每个扇区随数据一起上传的索引头字节数。
 * 必须与 bb_header_t 的实际大小一致(SIZE_CHECK 保证是 32)。 */
#define BB_DUMP_HEADER_LEN        ((uint32_t)sizeof(bb_header_t))

/* ============================ 内部状态 ============================ */
/* RAM 环形缓冲的帧数。它唯一的用途是吸收"Logger 正在写 Flash/擦扇区时"新产生的帧，
 * 所以容量必须覆盖**最坏情况下的一次停顿**：
 *     · 页编程     0.7~3ms      ->    1~3 帧
 *     · 扇区擦除   45ms(typ) / 400ms(max)  ->  45~400 帧   ← 决定性的一项
 *
 * 原来只有 256 帧(256ms)。实测(flash.bin 72 个扇区)稳态下缓冲长期停在 ~85% 满，
 * 每个扇区边界都固定丢 9 帧 —— 因为换扇区那次"擦除+写头"约 48ms，
 * 而缓冲只剩 ~39 帧的空位。这种"刚好卡在临界点"的状态，只要多加一点数据
 * (比如插高度帧)就必然变成大面积丢帧。
 *
 * 1024 帧 = 32KB，能完整吞下 400ms 的最坏擦除时间，留 2.5 倍余量。
 * 代价：掉电时理论上最多丢 1 秒；但改完之后稳态缓冲基本是空的(个位数帧)，
 *       实际掉电损失反而比原来更小。
 * RAM：静态数组从 8KB 涨到 32KB，整机 RW+ZI 从 33.6KB 到 57.6KB，F405 的 128KB 够用。 */
#define BB_RING_FRAMES            1024U     /* RAM 缓冲 1024 帧 = 32KB ≈ 1s */

static SemaphoreHandle_t bb_flash_mutex;

static volatile uint8_t  bb_hw_ok;
static volatile uint8_t  bb_armed;
/* 记录总开关（参数 bb.recording，重启生效）：
 * 0 = 连 RAM 队列都不进，Flash 一个字节都不动（解锁时也不切扇区、不擦除）。
 * 默认 1，与没加这个开关之前的行为完全一致。 */
static volatile uint8_t  bb_recording = 1U;
static volatile uint8_t  bb_arm_edge;      /* 解锁上升沿，交给 Logger 处理 */
static volatile uint8_t  bb_erase_req;
/* 擦除状态：bb_erasing=1 表示正在擦；bb_erase_seq 每完成一次擦除 +1。
 * 上位机轮询 BB:STAT 看这两个值即可判断"擦完了没"，不必硬等 200 秒。 */
volatile uint8_t  bb_erasing = 0;
volatile uint32_t bb_erase_seq = 0;
/* 非 static：usart.c 的 fputc 需要靠它避免在倾倒二进制数据时插入 printf 内容 */
volatile uint8_t  bb_busy;                 /* 指令正在占用 Flash(禁止倾倒)  */

static uint32_t bb_boot_count;
static uint32_t bb_sector_index;           /* 逻辑扇区序号(单调递增)        */
static uint32_t bb_frame_offset;           /* 当前扇区内已写帧数            */
static uint32_t bb_frame_seq;              /* 本次上锁内的帧序号            */
static uint8_t  bb_flight_id;
static uint32_t bb_recorded;
static uint32_t bb_dropped;
static uint32_t bb_sector_in_use;          /* 当前扇区是否已擦除并写好头    */
static uint8_t  bb_jedec_raw[3] = {0xFFU, 0xFFU, 0xFFU};  /* 实际读到的 JEDEC(诊断) */
static uint8_t  bb_init_step = 0;          /* 自检进行到哪一步(诊断)        */

static bb_frame_t bb_ring[BB_RING_FRAMES];
static volatile uint32_t bb_head;
static volatile uint32_t bb_tail;
/* 环形缓冲的历史最高水位(帧)。用来判断"余量还剩多少"：
 * 正常情况下它应该只有个位数；一旦接近 BB_RING_FRAMES 就说明 Logger 跟不上，
 * 丢帧只是时间问题。BB:STAT 里以 hwm= 显示。 */
static volatile uint32_t bb_ring_hwm;

static uint8_t  bb_page_buf[BB_PAGE_SIZE]; /* 攒页缓冲 + 倾倒时的读缓冲      */
static uint32_t bb_page_fill;
static uint32_t bb_page_index;

/* ---------- UART 指令组行 ---------- */
/* 32 字节放不下 CFG: 命令（例如 "CFG:SET pid.altitude_velocity.dlpf 1.0" 就有
 * 40 字节），一旦超长整行会被丢弃，表现为"命令没反应"。这里放宽到 64。 */
#define BB_CMD_LINE_MAX  64U
static char     bb_cmd_line[BB_CMD_LINE_MAX];
static uint32_t bb_cmd_len;
static char     bb_cmd_ready[BB_CMD_LINE_MAX];
static volatile uint8_t bb_cmd_pending;

/* ==================================================================
 *  一、W25Q128 底层驱动（阻塞式 SPI，无中断/无 DMA）
 * ================================================================== */

static void w25_cs_low(void)  { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); }
static void w25_cs_high(void) { HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);   }

static uint8_t w25_tx(const uint8_t *buf, uint16_t len)
{
    /* 阻塞式发送。
     * 为什么不用中断/DMA：Flash 操作本就在最低优先级任务里跑，阻塞它不影响
     * 1kHz 控制环；而 W25Q128 自身耗时(页写 0.7~3ms、扇区擦 45ms)远大于
     * SPI 传几个字节的时间(µs 级)。用阻塞式可省掉中断+信号量的整套异步机制，
     * 少一个故障点。 */
    return (HAL_SPI_Transmit(&hspi3, (uint8_t *)buf, len, W25_SPI_TIMEOUT_MS) == HAL_OK) ? 0U : 1U;
}

static uint8_t w25_rx(uint8_t *buf, uint16_t len)
{
    return (HAL_SPI_Receive(&hspi3, buf, len, W25_SPI_TIMEOUT_MS) == HAL_OK) ? 0U : 1U;
}

static uint8_t w25_read_status(void)
{
    uint8_t cmd = W25_CMD_READ_STATUS1;
    uint8_t st  = 0xFFU;

    w25_cs_low();
    if (w25_tx(&cmd, 1) == 0)
    {
        (void)w25_rx(&st, 1);
    }
    w25_cs_high();
    return st;
}

static uint8_t w25_wait_ready(uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    uint32_t   spins = 0;

    while ((w25_read_status() & W25_STATUS1_BUSY) != 0U)
    {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeout_ms))
        {
            return 1;
        }

        /* 先忙等一小段，再开始 osDelay(1)。
         *
         * 为什么：页编程典型只要 0.7ms，而 osDelay 的最小粒度是 1 个 tick(1ms)。
         * 原来"一进循环就 osDelay(1)"，等于每写一页白等一整拍 —— 一个扇区 15 页
         * 就是 15ms 的纯浪费，而这 15ms 刚好全部加在缓冲最紧张的换扇区阶段。
         *
         * 忙等期间没有关中断，更高优先级的 IMU(40)/FlyControl(10)/Baro(9) 任务
         * 照样能随时抢占本任务；本任务优先级最低(osPriorityLow)，所以这里空转
         * 只会吃掉 idle 的时间，不会影响 1kHz 控制环的时序。
         * 一次 w25_read_status 约 10~20µs，60 次 ≈ 0.6~1.2ms，正好盖住典型页编程。
         * 擦除(45~400ms)这种长操作会自然走到 osDelay 那一支，把 CPU 让出去。 */
        if (++spins < 60U)
        {
            /* 空转 */
        }
        else
        {
            osDelay(1);   /* 长操作(扇区/全片擦除)：绝不空转，把 CPU 让给控制任务 */
        }
    }
    return 0;
}

static uint8_t w25_write_enable(void)
{
    uint8_t cmd = W25_CMD_WRITE_ENABLE;

    w25_cs_low();
    if (w25_tx(&cmd, 1) != 0)
    {
        w25_cs_high();
        return 1;
    }
    w25_cs_high();

    return ((w25_read_status() & W25_STATUS1_WEL) != 0U) ? 0U : 1U;
}

static uint8_t w25_read_jedec(uint8_t id[3])
{
    uint8_t cmd = W25_CMD_JEDEC_ID;

    w25_cs_low();
    if (w25_tx(&cmd, 1) != 0)
    {
        w25_cs_high();
        return 1;
    }
    if (w25_rx(id, 3) != 0)
    {
        w25_cs_high();
        return 1;
    }
    w25_cs_high();
    return 0;
}

static uint8_t w25_read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    uint8_t cmd[4];

    if ((buf == NULL) || (len == 0U) || (len > 0xFFFFU))
    {
        return 1;
    }
    if ((addr + len) > BB_FLASH_TOTAL_BYTES)
    {
        return 1;
    }

    cmd[0] = W25_CMD_READ_DATA;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((addr >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(addr & 0xFFU);

    w25_cs_low();
    if (w25_tx(cmd, 4) != 0)
    {
        w25_cs_high();
        return 1;
    }
    if (w25_rx(buf, (uint16_t)len) != 0)
    {
        w25_cs_high();
        return 1;
    }
    w25_cs_high();
    return 0;
}

/* 写一整页：必须 256B 对齐，长度 1..256（跨页会回卷，务必遵守） */
static uint8_t w25_page_program(uint32_t addr, const uint8_t *data, uint16_t len)
{
    uint8_t cmd[4];

    if ((data == NULL) || (len == 0U) || (len > BB_PAGE_SIZE))
    {
        return 1;
    }
    if ((addr % BB_PAGE_SIZE) != 0U)
    {
        return 1;
    }
    if ((addr + len) > BB_FLASH_TOTAL_BYTES)
    {
        return 1;
    }

    if (w25_write_enable() != 0)
    {
        return 1;
    }

    cmd[0] = W25_CMD_PAGE_PROGRAM;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((addr >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(addr & 0xFFU);

    w25_cs_low();
    if (w25_tx(cmd, 4) != 0)
    {
        w25_cs_high();
        return 1;
    }
    if (w25_tx(data, len) != 0)
    {
        w25_cs_high();
        return 1;
    }
    w25_cs_high();

    return w25_wait_ready(W25_PAGE_PROGRAM_MS);
}

static uint8_t w25_sector_erase(uint32_t addr)
{
    uint8_t cmd[4];

    addr &= ~(BB_SECTOR_SIZE - 1U);

    if (w25_write_enable() != 0)
    {
        return 1;
    }

    cmd[0] = W25_CMD_SECTOR_ERASE;
    cmd[1] = (uint8_t)((addr >> 16) & 0xFFU);
    cmd[2] = (uint8_t)((addr >> 8) & 0xFFU);
    cmd[3] = (uint8_t)(addr & 0xFFU);

    w25_cs_low();
    if (w25_tx(cmd, 4) != 0)
    {
        w25_cs_high();
        return 1;
    }
    w25_cs_high();

    return w25_wait_ready(W25_SECTOR_ERASE_MS);
}

static uint8_t w25_chip_erase(void)
{
    uint8_t cmd = W25_CMD_CHIP_ERASE;

    if (w25_write_enable() != 0)
    {
        return 1;
    }

    w25_cs_low();
    if (w25_tx(&cmd, 1) != 0)
    {
        w25_cs_high();
        return 1;
    }
    w25_cs_high();

    return w25_wait_ready(W25_CHIP_ERASE_MS);
}

/* ==================================================================
 *  二、扇区与写指针管理
 * ================================================================== */

static uint8_t bb_read_sector_header(uint32_t phys_sector, bb_header_t *hdr)
{
    /* 只读头本身(32B)而不是整页，全片扫描时能省下大量 SPI 传输时间 */
    if (w25_read(phys_sector * BB_SECTOR_SIZE, (uint8_t *)hdr, sizeof(bb_header_t)) != 0)
    {
        return 0;
    }
    if ((hdr->magic != BB_MAGIC_HEADER) || (hdr->type != BB_TYPE_HEADER))
    {
        return 0;
    }
    if (hdr->frame_offset > BB_FRAMES_PER_SECTOR)
    {
        return 0;
    }
    return 1;
}

/* 扫描全片，找出逻辑序号最大的扇区（= 最新写入处）。
 * 返回 1 表示找到，out_phys 给出它的物理扇区号、out_offset 给出已写帧数。 */
static uint8_t bb_find_last_written(uint32_t *out_phys, uint32_t *out_offset)
{
    uint32_t i;
    uint32_t best_sector = 0;
    uint32_t best_index  = 0;
    uint32_t best_offset = 0;
    uint8_t  found = 0;
    bb_header_t hdr;

    for (i = 0; i < BB_SECTOR_COUNT; i++)
    {
        if (bb_read_sector_header(i, &hdr) != 0)
        {
            if ((found == 0) || ((int32_t)(hdr.sector_index - best_index) > 0))
            {
                best_sector = i;
                best_index  = hdr.sector_index;
                best_offset = hdr.frame_offset;
                found = 1;
            }
        }
    }

    if (found != 0)
    {
        if (out_phys   != NULL) { *out_phys   = best_sector; }
        if (out_offset != NULL) { *out_offset = best_offset; }
    }
    return found;
}

/* 计算"有数据的物理扇区范围"。
 * 数据是从扇区 0 开始连续写的（环形缓冲写满才回头），所以范围永远是 [0, end]。
 * 返回 1 表示有数据，out_end 给出最后一个有数据的物理扇区号(含)。 */
static uint8_t bb_get_used_range(uint32_t *out_end)
{
    uint32_t phys = 0;
    uint32_t off  = 0;

    if (out_end == NULL)
    {
        return 0;
    }
    if (bb_find_last_written(&phys, &off) == 0)
    {
        return 0;       /* 整片空白 */
    }

    /* 该扇区如果已经写满(BB_FRAMES_PER_SECTOR)，说明它是完整的一段；
     * 否则它本身就是最后一个有数据的扇区。两种情况都取 phys 作为结束。 */
    *out_end = phys;
    return 1;
}

static void bb_scan_write_pointer(void)
{
    uint32_t best_sector = 0;
    uint32_t best_index  = 0;

    if (bb_find_last_written(&best_sector, NULL) != 0)
    {
        /* 取回该扇区的逻辑序号，用于续写 */
        bb_header_t hdr;
        if (bb_read_sector_header(best_sector, &hdr) != 0)
        {
            best_index = hdr.sector_index;
        }

        /* 一律从"新扇区"开始记：避免在扇区中间续写(会破坏已写数据)，
         * 代价只是丢掉上次掉电时未写满的那一个扇区(<=4KB)。 */
        bb_sector_index = best_index + 1U;
        bb_frame_offset = 0U;
        bb_sector_in_use = 0xFFFFFFFFU;
    }
    else
    {
        bb_sector_index = 0U;
        bb_frame_offset = 0U;
        bb_sector_in_use = 0xFFFFFFFFU;
    }
}

static uint8_t bb_write_sector_header(void)
{
    uint8_t buf[BB_PAGE_SIZE];
    uint32_t phys = (bb_sector_index % BB_SECTOR_COUNT) * BB_SECTOR_SIZE;
    bb_header_t hdr;

    memset(buf, 0xFF, sizeof(buf));

    hdr.magic        = BB_MAGIC_HEADER;
    hdr.type         = BB_TYPE_HEADER;
    hdr.reserved[0]  = 0xFFU;
    hdr.reserved[1]  = 0xFFU;
    hdr.reserved[2]  = 0xFFU;
    hdr.sector_index = bb_sector_index;
    hdr.frame_offset = (uint16_t)bb_frame_offset;
    hdr.reserved2    = 0xFFFFU;
    hdr.boot_count   = bb_boot_count;
    hdr.reserved3[0] = 0xFFFFFFFFU;
    hdr.reserved3[1] = 0xFFFFFFFFU;
    hdr.reserved3[2] = 0xFFFFFFFFU;

    memcpy(buf, &hdr, sizeof(hdr));

    return w25_page_program(phys, buf, BB_PAGE_SIZE);
}

/* 进入(或重新进入)当前扇区：需要时先擦除，再写索引头。返回 0 成功 */
static uint8_t bb_begin_sector(void)
{
    uint32_t phys_sector = bb_sector_index % BB_SECTOR_COUNT;

    if (bb_sector_in_use != phys_sector)
    {
        if (w25_sector_erase(phys_sector * BB_SECTOR_SIZE) != 0)
        {
            return 1;
        }
        bb_sector_in_use = phys_sector;
        bb_frame_offset  = 0;
    }

    bb_page_index = bb_frame_offset / BB_FRAMES_PER_PAGE;
    bb_page_fill  = bb_frame_offset % BB_FRAMES_PER_PAGE;

    return bb_write_sector_header();
}

/* 页缓冲攒满后写入 Flash。返回 0 成功 / 1 需要换扇区 / 2 失败 */
static uint8_t bb_write_page(void)
{
    uint32_t phys_sector = bb_sector_index % BB_SECTOR_COUNT;
    uint32_t phys;

    if (bb_page_index >= BB_PAGES_PER_SECTOR)
    {
        return 1;   /* 本扇区已满，交由调用方换扇区 */
    }

    phys = phys_sector * BB_SECTOR_SIZE + BB_SECTOR_DATA_OFF + bb_page_index * BB_PAGE_SIZE;

    if (w25_page_program(phys, bb_page_buf, BB_PAGE_SIZE) != 0)
    {
        return 2;
    }

    bb_frame_offset += bb_page_fill;
    bb_page_index++;
    bb_page_fill = 0;

    /* ★ 原来这里有"每写 4 页刷新一次索引头"，已删除 —— 它是**无效写**：
     *   NOR Flash 的页编程只能把 1 写成 0，同一页不擦除再写，新内容会和旧内容逐位相与。
     *   索引头的 frame_offset 是 0 -> 32 -> 64 … 一路置位的，与完之后永远是 0
     *   （实测 flash.bin 里 72 个扇区的 frame_offset 确实全是 0，印证了这一点），
     *   所以它从来没起过作用，只是每个扇区白白多做 3~4 次页编程(每次 0.7~3ms)，
     *   而且恰好都落在缓冲最紧张的换扇区阶段。
     *   现在索引头只在换扇区时写一次。
     *   掉电续写位置本来也不依赖它：bb_scan_write_pointer() 一律从下一个新扇区开始记。 */
    return 0;
}

/* ==================================================================
 *  三、对外接口
 * ================================================================== */

void BlackBox_Init(void)
{
    uint8_t id[3] = {0};
    uint8_t i;

    bb_head = 0;
    bb_tail = 0;
    bb_ring_hwm = 0;
    bb_page_fill = 0;
    bb_page_index = 0;
    bb_frame_seq = 0;
    bb_flight_id = 0;
    bb_recorded = 0;
    bb_dropped = 0;
    bb_armed = 0;
    bb_arm_edge = 0;
    bb_erase_req = 0;
    bb_busy = 0;
    bb_cmd_len = 0;
    bb_cmd_pending = 0;
    bb_sector_index = 0;
    bb_frame_offset = 0;
    bb_sector_in_use = 0xFFFFFFFFU;
    bb_hw_ok = 0;
    bb_init_step = 1;
    for (i = 0; i < 3U; i++)
    {
        bb_jedec_raw[i] = 0xFFU;    /* 初值设成"没读到"的样子 */
    }

    bb_flash_mutex = xSemaphoreCreateMutex();
    if (bb_flash_mutex == NULL)
    {
        bb_init_step = 0xF0U;       /* 互斥量创建失败 */
        return;
    }

    w25_cs_high();
    osDelay(10);

    {
        uint8_t cmd = W25_CMD_RELEASE_POWERDOWN;
        w25_cs_low();
        (void)w25_tx(&cmd, 1);
        w25_cs_high();
    }
    osDelay(5);
    bb_init_step = 2;

    if (w25_read_jedec(id) != 0)
    {
        bb_init_step = 0xE1U;       /* JEDEC 读取超时/失败(SPI 中断没响应) */
        return;
    }
    bb_init_step = 3;

    for (i = 0; i < 3U; i++)
    {
        bb_jedec_raw[i] = id[i];    /* 记录实际读到的值，供上位机判断 */
    }

    if ((id[0] != W25_JEDEC_WINBOND) || (id[2] != W25_JEDEC_CAPACITY_128M))
    {
        bb_init_step = 0xE2U;       /* ID 读到但与 W25Q128 不符 */
        return;
    }

    bb_hw_ok = 1;

    /* 上电次数：从现存扇区头继承，保证跨重启单调递增 */
    {
        uint32_t i;
        uint32_t best = 0;
        uint8_t  found = 0;
        bb_header_t hdr;

        for (i = 0; i < BB_SECTOR_COUNT; i++)
        {
            if (bb_read_sector_header(i, &hdr) != 0)
            {
                if ((found == 0) || ((int32_t)(hdr.boot_count - best) > 0))
                {
                    best = hdr.boot_count;
                    found = 1;
                }
            }
        }
        bb_boot_count = (found != 0) ? (best + 1U) : 1U;
    }

    bb_scan_write_pointer();
}

void BlackBox_SetArmed(uint8_t armed)
{
    if ((armed != 0U) && (bb_armed == 0U))
    {
        bb_flight_id = (uint8_t)(bb_flight_id + 1U);
        if (bb_flight_id == 0U)
        {
            bb_flight_id = 1U;
        }
        bb_frame_seq = 0;
        bb_armed = 1;
        /* 每次解锁重新统计水位高标，BB:STAT 里的 hwm 反映"这一段飞行"的余量 */
        bb_ring_hwm = 0;
        bb_arm_edge = 1;    /* 真正的扇区切换交给 Logger 做，这里绝不碰 Flash */
    }
    else if ((armed == 0U) && (bb_armed != 0U))
    {
        bb_armed = 0;
    }
    else
    {
        /* 状态无变化 */
    }
}

/* 供配置存储模块用：擦/写内部 Flash 前必须确认未解锁。
 * 内部 Flash 的扇区擦除是秒级的，解锁时执行会让控制环停顿上秒 —— 空中发生就是炸机。 */
uint8_t BlackBox_IsArmed(void)
{
    return bb_armed;
}

/* 黑匣子记录总开关（参数 bb.recording，重启生效）。
 * 关掉之后：不排队、不写页、解锁时也不切扇区 —— 飞行中完全不碰 SPI3/Flash。
 * 注意它只影响"记录"，BB:STAT / BB:DUMP / BB:ERASE 等上位机命令照常可用。 */
void BlackBox_SetRecording(uint8_t on)
{
    bb_recording = (on != 0U) ? 1U : 0U;
}

uint8_t BlackBox_IsRecording(void)
{
    return bb_recording;
}

/* 把浮点量按给定 LSB 转成 int16，超范围时饱和(宁可饱和也不要回卷) */
static int16_t bb_to_i16(float v, float lsb)
{
    float x;

    if (v != v)          /* NaN 自身不等于自身，避免未定义行为 */
    {
        return 0;
    }

    x = v / lsb;

    if (x > 32767.0f)  { x = 32767.0f; }
    if (x < -32768.0f) { x = -32768.0f; }
    return (int16_t)((x >= 0.0f) ? (x + 0.5f) : (x - 0.5f));
}

/* 更新环形缓冲水位高标(帧)。任何一帧入队后调一次。
 * 这是判断"Logger 还有多少余量"最直接的指标：
 *   个位数        -> 稳态健康，Logger 跟得上（换扇区擦除时会短时涨到几十帧，正常）
 *   接近 1024     -> Logger 已经饱和，再看 drop 一定在涨 */
static void bb_ring_note_hwm(void)
{
    uint32_t used = (bb_head + BB_RING_FRAMES - bb_tail) % BB_RING_FRAMES;

    if (used > bb_ring_hwm)
    {
        bb_ring_hwm = used;
    }
}

/* 悬停油门学习值的落盘请求（FlyControl 任务置位，Logger 任务执行） */
static volatile float   bb_hover_req;
static volatile float   bb_hover_req_bat;
static volatile uint8_t bb_hover_pending;

void BlackBox_RequestHoverThrottleSave(float pwm, float bat_pct)
{
    bb_hover_req     = pwm;
    bb_hover_req_bat = bat_pct;
    bb_hover_pending = 1U;
}

void BlackBox_Queue(float roll, float pitch, float yaw,
                    float gyro_x, float gyro_y, float gyro_z,
                    float accel_x, float accel_y, float accel_z,
                    float altitude, float vert_velocity, float vert_accel,
                    const uint16_t motor[4], float throttle,
                    uint8_t alt_valid, uint8_t alt_hold_mode,
                    uint8_t mix_saturated,
                    uint8_t land_cut_reason,
                    uint8_t battery_percent)
{
    bb_frame_t f;
    uint32_t head;
    uint32_t next;
    uint32_t i;

    /* 本版帧布局容不下高度/垂速/垂直加速度(32 字节优先给了姿态与 IMU 原始量)，
     * 参数保留是为了不破坏调用方接口，同时便于将来扩帧。 */
    (void)altitude;
    (void)vert_velocity;
    (void)vert_accel;
    (void)alt_valid;

    if ((bb_hw_ok == 0) || (bb_armed == 0) || (bb_recording == 0U))
    {
        return;
    }

    head = bb_head;
    next = (head + 1U) % BB_RING_FRAMES;
    if (next == bb_tail)
    {
        bb_dropped++;       /* 缓冲满：丢帧，绝不阻塞 1kHz 控制环 */
        return;
    }

    memset(&f, 0, sizeof(f));

    f.magic = BB_MAGIC_FRAME;
    f.seq   = (uint16_t)bb_frame_seq;

    f.roll  = bb_to_i16(roll,  BB_ANGLE_LSB);
    f.pitch = bb_to_i16(pitch, BB_ANGLE_LSB);
    f.yaw   = bb_to_i16(yaw,   BB_ANGLE_LSB);

    /* 陀螺记录的是 rad/s 的原始量，转成 deg/s 后再定点 */
    f.gyro_x = bb_to_i16(gyro_x * RAD_TO_DEG, BB_GYRO_LSB);
    f.gyro_y = bb_to_i16(gyro_y * RAD_TO_DEG, BB_GYRO_LSB);
    f.gyro_z = bb_to_i16(gyro_z * RAD_TO_DEG, BB_GYRO_LSB);

    f.accel_x = bb_to_i16(accel_x, BB_ACCEL_LSB);
    f.accel_y = bb_to_i16(accel_y, BB_ACCEL_LSB);
    f.accel_z = bb_to_i16(accel_z, BB_ACCEL_LSB);

    if ((bb_frame_seq % BB_DIAG_INTERVAL) == 0U)
    {
        /* 诊断帧：把 seq 槽改作诊断量(见 blackbox.h 的帧布局说明)。
         * conf 压成 0..3 档(0.25 分辨率，足够判断"重力参考是否还在工作")；
         * period 以 100µs 为单位(0..255 覆盖 0..25.5ms)。 */
        float c = ICM42605_GetAccelConfidence();
        float p = ICM42605_GetSamplePeriod() * 10000.0f;   /* s -> 100µs 单位 */
        uint8_t c_code;
        uint8_t p_code;

        if (c < 0.0f)   { c = 0.0f; }
        if (c > 1.0f)   { c = 1.0f; }
        c_code = (uint8_t)(c * 3.0f + 0.5f);

        if (p < 0.0f)   { p = 0.0f; }
        if (p > 255.0f) { p = 255.0f; }
        p_code = (uint8_t)(p + 0.5f);

        f.type = BB_TYPE_DIAG;
        f.seq  = (uint16_t)((uint16_t)c_code | ((uint16_t)p_code << 8));
    }
    else
    {
        f.type = BB_TYPE_FRAME;
    }

    /* 四路电机(0..4200 -> 0..127) + 油门(0..4200 -> 0..15) 打包成一个 32bit 字 */
    {
        uint32_t packed = 0;
        uint32_t m;

        for (i = 0; i < 4U; i++)
        {
            uint32_t v = (uint32_t)motor[i];
            if (v > 4200U) { v = 4200U; }
            v = (v * 127U + 2100U) / 4200U;      /* 四舍五入到 0..127 */
            if (v > 127U) { v = 127U; }
            packed |= (v << (7U * i));
        }

        {
            float th = throttle;
            if (th < 0.0f)    { th = 0.0f; }
            if (th > 4200.0f) { th = 4200.0f; }
            m = (uint32_t)((th * 15.0f / 4200.0f) + 0.5f);
            if (m > 15U) { m = 15U; }
            packed |= (m << 28);
        }

        f.motors = packed;
    }

    f.flags   = (uint8_t)(((alt_valid != 0U) ? 0x01U : 0x00U) |
                          ((alt_hold_mode != 0U) ? 0x02U : 0x00U) |
                          ((mix_saturated != 0U) ? 0x04U : 0x00U) |
                          (uint8_t)((land_cut_reason & 0x07U) << 3));  /* bit3-5: 降落断电原因 */
    f.battery = battery_percent;

    memcpy(&bb_ring[head], &f, BB_FRAME_SIZE);

    bb_frame_seq++;
    bb_recorded++;

    __disable_irq();
    bb_head = next;
    __enable_irq();

    bb_ring_note_hwm();
}

/* 遥控摇杆帧：与数据帧共用 32 字节结构，槽位含义见 blackbox.h 的说明。
 * 目的是回答"飞机往左飞是操作者打杆修正，还是控制器自己推走的"。 */
void BlackBox_QueueRc(const uint16_t rc[4], uint16_t arm, uint16_t mode,
                      uint8_t flight_status, float target_roll_deg,
                      float target_pitch_deg, const uint16_t motor[4],
                      uint8_t battery_percent)
{
    bb_frame_t f;
    uint32_t head;
    uint32_t next;
    uint32_t i;

    if ((bb_hw_ok == 0) || (bb_armed == 0) || (bb_recording == 0U) ||
        (rc == NULL) || (motor == NULL))
    {
        return;
    }

    head = bb_head;
    next = (head + 1U) % BB_RING_FRAMES;
    if (next == bb_tail)
    {
        bb_dropped++;
        return;
    }

    memset(&f, 0, sizeof(f));
    f.magic = BB_MAGIC_FRAME;
    f.type  = BB_TYPE_RC;
    f.seq   = (uint16_t)bb_frame_seq;

    f.roll   = (int16_t)((rc[0] > 2000U) ? 2000 : rc[0]);
    f.pitch  = (int16_t)((rc[1] > 2000U) ? 2000 : rc[1]);
    f.yaw    = (int16_t)((rc[2] > 2000U) ? 2000 : rc[2]);
    f.gyro_x = (int16_t)((rc[3] > 2000U) ? 2000 : rc[3]);
    f.gyro_y = (int16_t)arm;
    f.gyro_z = (int16_t)mode;
    f.accel_x = (int16_t)flight_status;
    /* 原始(未滤波)陀螺，×10 存。
     * 用来对照数据帧里的滤波后值：
     * 两者偏离一致 -> 传感器本身就带偏移；
     * 只有滤波后偏 -> 滤波器引入了偏移。 */
    /* **实际目标角**（×100，±327 deg）——就是送进控制器的设定值。
     * 记它而不记摇杆的好处：
     *   · 不需要知道接收机微调(roll_offset)，设定值是绝对准的
     *   · BB:STEP 注入时摇杆没动，只有记目标才能还原设定值
     * 用它做阶跃响应分析时，直接从这里取设定值，无需换算。 */
    f.accel_y = bb_to_i16(target_roll_deg,  0.01f);
    f.accel_z = bb_to_i16(target_pitch_deg, 0.01f);
    /* 格式标记：摇杆帧的 accel_y/accel_z 槽位曾经存的是"加计推算倾角"，
     * 现在存"实际目标角"。两者含义完全不同，必须让上位机能分辨，
     * 否则拿旧日志配新解析器会把加计倾角当成设定值。
     *   ★ 用特征值而不是 1：旧固件的摇杆帧结构体未清零，
     *     reserved 字节是**未初始化的栈垃圾**，恰好等于 1 的概率不小。
     *     用一个不常见的特征值 0xB7 才能可靠地区分新旧日志。 */
    f.reserved = BB_RC_FMT_TARGET;

    {
        uint32_t packed = 0;
        for (i = 0; i < 4U; i++)
        {
            uint32_t v = (uint32_t)motor[i];
            if (v > 4200U) { v = 4200U; }
            v = (v * 127U + 2100U) / 4200U;
            if (v > 127U) { v = 127U; }
            packed |= (v << (7U * i));
        }
        f.motors = packed;
    }

    f.battery = battery_percent;

    memcpy(&bb_ring[head], &f, BB_FRAME_SIZE);

    bb_frame_seq++;
    bb_recorded++;

    __disable_irq();
    bb_head = next;
    __enable_irq();

    bb_ring_note_hwm();
}

/* 高度帧（BB_TYPE_ALT，50Hz）：槽位含义见 blackbox.h 的说明。
 * 由 FlyControl 任务调用（和数据帧/摇杆帧同一个生产者），所以不违反
 * "RAM 环形只有一个写者"的假设 —— 绝不能从气压计任务里直接调。 */
void BlackBox_QueueAlt(const HeightEstimator_View *v, float throttle_correction,
                       float hover_base, uint8_t hover_valid)
{
    bb_frame_t f;
    uint32_t head;
    uint32_t next;
    uint8_t  flags;

    if (v == NULL)
    {
        return;
    }
    if ((bb_hw_ok == 0) || (bb_armed == 0) || (bb_recording == 0U))
    {
        return;
    }

    head = bb_head;
    next = (head + 1U) % BB_RING_FRAMES;
    if (next == bb_tail)
    {
        bb_dropped++;
        return;
    }

    memset(&f, 0, sizeof(f));
    f.magic = BB_MAGIC_FRAME;
    f.type  = BB_TYPE_ALT;
    f.seq   = (uint16_t)bb_frame_seq;

    flags = (uint8_t)(((v->baro_ok       != 0U) ? 0x01U : 0x00U) |
                      ((v->innov_clamped != 0U) ? 0x02U : 0x00U) |
                      ((v->gain_rejected != 0U) ? 0x04U : 0x00U) |
                      (((v->pressure_pa == v->pressure_pa) &&
                        (v->pressure_pa > 1000.0f)) ? 0x08U : 0x00U));
    f.flags = flags;

    f.roll    = bb_to_i16(v->baro_altitude, 0.01f);        /* ×0.01 m   */
    f.pitch   = bb_to_i16(v->altitude,      0.01f);        /* ×0.01 m   */
    f.yaw     = bb_to_i16(v->velocity,      0.01f);        /* ×0.01 m/s */
    f.gyro_x  = bb_to_i16(v->innov,         0.0001f);      /* ×0.1 mm   */
    f.gyro_y  = bb_to_i16(v->accel_bias,    0.0001f);      /* ×1e-4 m/s²*/
    /* baro_dt 单位换成 0.1ms：正常 20ms → 200，int16 上限 3276.7ms */
    f.gyro_z  = bb_to_i16(v->baro_dt * 1000.0f, 0.1f);
    f.accel_x = bb_to_i16(v->alt_ref,       0.01f);        /* ×0.01 m   */
    /* 气压槽：LSB = 10 Pa（= 0.1 hPa）。101325 Pa -> 10132。
     * 注意固件里气压的单位是 **Pa**（BMP280 补偿结果 /256），不是 hPa；
     * int16 上限 3276.7 hPa，够覆盖 300~1200hPa。上位机按 ×0.1 还原成 hPa。 */
    f.accel_y = bb_to_i16(v->pressure_pa,   10.0f);
    f.accel_z = bb_to_i16(throttle_correction, 1.0f);      /* ×1 PWM    */

    f.reserved = BB_ALT_FMT_MARK;
    f.battery  = 0U;

    /* motors 槽在高度帧里没别的用途，用来记"定高基准油门"（学到的悬停油门）：
     *   低 16 位 = 基准油门 PWM，bit31 = hover_valid。
     * 判读：修正量长期不落在 0 附近时，先看这一项有没有在动。 */
    {
        uint32_t hb = (uint32_t)(hover_base + 0.5f);

        if (hb > 0xFFFFU) { hb = 0xFFFFU; }
        f.motors = hb | ((hover_valid != 0U) ? 0x80000000UL : 0UL);
    }

    memcpy(&bb_ring[head], &f, BB_FRAME_SIZE);

    /* 不递增 bb_frame_seq：ALT 帧是"旁挂"的样本，和摇杆帧一样借用当前序号，
     * 上位机靠 type 把它排除在序号时间轴之外（否则会造成重复/回退的假丢帧）。 */
    bb_recorded++;

    __disable_irq();
    bb_head = next;
    __enable_irq();

    bb_ring_note_hwm();
}

void BlackBox_GetStatus(BlackBox_Status_t *st)
{
    if (st == NULL)
    {
        return;
    }

    st->flash_ok      = bb_hw_ok;
    st->jedec_id[0]   = W25_JEDEC_WINBOND;
    st->jedec_id[1]   = 0x40U;
    st->jedec_id[2]   = W25_JEDEC_CAPACITY_128M;
    st->jedec_raw[0]  = bb_jedec_raw[0];
    st->jedec_raw[1]  = bb_jedec_raw[1];
    st->jedec_raw[2]  = bb_jedec_raw[2];
    st->init_step     = bb_init_step;
    st->boot_count    = bb_boot_count;
    st->sector_index  = bb_sector_index;
    st->frame_offset  = (uint16_t)bb_frame_offset;
    st->flight_id     = bb_flight_id;
    st->recorded_frames = bb_recorded;
    st->dropped_frames  = bb_dropped;
    st->ring_hwm      = bb_ring_hwm;
    st->armed         = bb_armed;
}

/* ==================================================================
 *  四、Logger 任务
 * ================================================================== */

void BlackBox_LoggerTask(void *argument)
{
    (void)argument;

    osDelay(2500);

    if (bb_hw_ok != 0)
    {
        if (xSemaphoreTake(bb_flash_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
        {
            if (bb_begin_sector() != 0)
            {
                bb_hw_ok = 0;
            }
            xSemaphoreGive(bb_flash_mutex);
        }
    }

    for (;;)
    {
        uint8_t rc = 0;

        /* ---- 0a. 读取上位机指令(USART3 DMA 环形缓冲) ---- */
        {
            uint8_t  chunk[64];
            uint16_t n = USART3_PollRx(chunk, (uint16_t)sizeof(chunk));
            uint16_t k;

            for (k = 0; k < n; k++)
            {
                BlackBox_FeedCommandByte(chunk[k]);
            }
        }
        BlackBox_Poll();

        /* ---- 0b. 解锁瞬间：把上一段未写满的页冲掉，换到新扇区 ---- */
        if (bb_arm_edge != 0)
        {
            bb_arm_edge = 0;

            /* 记录开关关掉时，连"切扇区"都不做 —— 那一步会擦除/写索引头，
             * 是黑匣子唯一会在飞行中动 Flash 的地方。 */
            if (bb_recording == 0U)
            {
                continue;
            }

            if (xSemaphoreTake(bb_flash_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
            {
                uint32_t carry = 0;

                if ((bb_page_fill > 0U) && (bb_page_index < BB_PAGES_PER_SECTOR))
                {
                    /* 半页残留：补 0xFF 后写下去，保持帧序连续 */
                    uint32_t written = bb_page_fill;
                    memset(&bb_page_buf[written * BB_FRAME_SIZE], 0xFF,
                           BB_PAGE_SIZE - written * BB_FRAME_SIZE);
                    bb_page_fill = written;
                    (void)bb_write_page();
                }
                /* 上面若因"本扇区已写满"没写成功，这半页帧必须带到新扇区去，
                 * 不能清掉（写成功时 bb_page_fill 已被 bb_write_page 清 0）。 */
                carry = bb_page_fill;

                bb_page_fill  = 0;
                bb_page_index = 0;
                bb_sector_index++;
                if (bb_begin_sector() != 0)
                {
                    bb_hw_ok = 0;
                }
                bb_page_fill = carry;
                xSemaphoreGive(bb_flash_mutex);
            }
            continue;
        }

        /* ---- 1. 全片擦除请求（只在未解锁时执行） ---- */
        if ((bb_erase_req != 0) && (bb_armed == 0))
        {
            bb_erase_req = 0;
            bb_busy = 1;
            bb_erasing = 1;
            bb_send("BB:ERASE START (about 200s)\r\n");

            if (xSemaphoreTake(bb_flash_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
            {
                if (w25_chip_erase() == 0)
                {
                    bb_sector_index  = 0;
                    bb_frame_offset  = 0;
                    bb_sector_in_use = 0xFFFFFFFFU;
                    bb_recorded = 0;
                    bb_dropped  = 0;
                    bb_frame_seq = 0;
                    bb_page_fill = 0;
                    bb_page_index = 0;
                    (void)bb_begin_sector();
                }
                xSemaphoreGive(bb_flash_mutex);
            }
            bb_erasing = 0;
            bb_erase_seq++;                 /* 上位机靠这个值变化判断"擦完了" */
            bb_busy = 0;
            bb_send("BB:ERASE DONE\r\n");   /* 主动上报一次(上位机轮询为主，这个是附带) */
            continue;
        }

        /* ---- 1b. 悬停油门学习值落盘 ----
         * Flash / 参数域归 Logger 任务独占，而且只在未解锁时才允许写。
         * 同时存下"悬停油门 + 当时电量"到 alt.ref_throttle / alt.ref_pct：
         * 只存绝对油门的话，换一块满电电池上来还按旧油门给，会一直往上冲。 */
        if ((bb_hover_pending != 0U) && (bb_armed == 0U))
        {
            float pwm = bb_hover_req;
            float bat = bb_hover_req_bat;
            /* 用"待生效表"而不是 GetApplied 做比较基准：Applied 是开机快照，
             * 保存之后不会更新，会导致每次落地都重复写一遍 Flash。 */
            float old = Param_Get(P_ALT_HOVER_THR);
            float old_ref = Param_Get(P_ALT_REF_THR);
            float old_pct = Param_Get(P_ALT_REF_PCT);
            uint8_t changed = 0U;

            bb_hover_pending = 0;

            /* 4200 = 油门满量程（与混控侧 MOTOR_MAX 一致） */
            if ((pwm > 200.0f) && (pwm < 4200.0f))
            {
                if (fabsf(pwm - old) > 5.0f)
                {
                    if (Param_Set(P_ALT_HOVER_THR, pwm) == 0U)
                    {
                        pwm = 0.0f;         /* 写失败就别再动参考点 */
                    }
                    changed = 1U;
                }
                /* 参考点（悬停油门 + 电量）每 5 PWM / 2% 更新一次 */
                if ((pwm > 200.0f) && (bat > 5.0f) && (bat <= 100.0f) &&
                    ((fabsf(pwm - old_ref) > 5.0f) || (fabsf(bat - old_pct) > 2.0f)))
                {
                    (void)Param_Set(P_ALT_REF_THR, pwm);
                    (void)Param_Set(P_ALT_REF_PCT, bat);
                    changed = 1U;
                }
            }

            if (changed != 0U)
            {
                char line[80];

                (void)Param_Save();
                (void)snprintf(line, sizeof(line),
                               "BB:HOVER SAVED %.0f @ %.0f%% (was %.0f @ %.0f%%)\r\n",
                               (double)bb_hover_req, (double)bat,
                               (double)old, (double)old_pct);
                bb_send(line);
            }
        }

        /* ---- 2. 把 RAM 缓冲里的帧攒进页缓冲 ---- */
        while ((bb_page_fill < BB_FRAMES_PER_PAGE) && (bb_tail != bb_head))
        {
            memcpy(&bb_page_buf[bb_page_fill * BB_FRAME_SIZE],
                   &bb_ring[bb_tail], BB_FRAME_SIZE);
            bb_tail = (bb_tail + 1U) % BB_RING_FRAMES;
            bb_page_fill++;
        }

        /* ---- 3. 页满则写 Flash ---- */
        if (bb_page_fill >= BB_FRAMES_PER_PAGE)
        {
            if (xSemaphoreTake(bb_flash_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                rc = bb_write_page();
                xSemaphoreGive(bb_flash_mutex);
            }
        }

        if (rc == 1U)
        {
            /* 扇区写满：换到下一个扇区。
             * ★ 这里原来还要先把 frame_offset=120 写进索引头"记下已写满"，
             *   那也是无效写(同样只能清位、与完还是 0)，纯属在擦除前多花一次
             *   0.7~3ms 的页编程 —— 正好是缓冲最紧张的时刻，已删除。
             *   上位机判断"扇区写满"靠的是 frame_offset==0 就按满扇区处理，
             *   所以去掉之后行为不变。
             *
             * ★★ 另一个必须修的问题：走到这里时 bb_page_fill 已经被"攒"满了
             *   （一般是 8 帧），而 bb_write_page 因为扇区已满并没有把它们写下去。
             *   原来紧接着 bb_page_fill = 0，这 8 帧就被**静默丢弃**了：
             *     · 既没进 Flash，也不计 bb_dropped（所以 BB:STAT 的 drop 是 0）
             *     · 每换一个扇区丢 8 帧 = 6.7%，并且永久消失
             *   这正是代码里那句"实测相邻扇区差 128 而每扇区只存 120 帧"的来源
             *   （120 + 8 = 128）。现在把它们带到新扇区接着写。 */
            if (xSemaphoreTake(bb_flash_mutex, pdMS_TO_TICKS(2000)) == pdTRUE)
            {
                uint32_t carry = bb_page_fill;

                bb_frame_offset = 0;
                bb_page_fill = 0;
                bb_page_index = 0;
                bb_sector_index++;
                if (bb_begin_sector() != 0)
                {
                    bb_hw_ok = 0;
                }
                bb_page_fill = carry;       /* 页缓冲内容没动，直接续写 */
                xSemaphoreGive(bb_flash_mutex);
            }
        }
        else if (rc == 2U)
        {
            bb_hw_ok = 0;   /* 写失败：停记，避免拖累系统 */
        }
        else
        {
            /* ★ 关键修复：只在"攒不满一页"的时候才睡。
             *
             * 原来这里是无条件 osDelay(2)。而这条 else 分支在**每次成功写完一页**之后
             * 也会走到（rc==0 就是成功），于是缓冲里明明还堆着几十帧要落盘，
             * 也要先睡 2ms。8 帧一页 = 2ms 睡眠 + ~1.5ms 实际写页，
             * 一个扇区 15 页白睡 30ms；这 30ms 就是缓冲长期停在 85% 满的根因。
             *
             * 本任务优先级最低(osPriorityLow)，"有活的时候连续干"不会抢控制环的 CPU；
             * 只有真的没活(缓冲里不足一页)才用 osDelay 让出 CPU。 */
            uint32_t pending = (bb_head + BB_RING_FRAMES - bb_tail) % BB_RING_FRAMES;

            if (pending < BB_FRAMES_PER_PAGE)
            {
                osDelay(1);
            }
        }
    }
}

/* ==================================================================
 *  五、串口指令接口（USART3）
 * ================================================================== */

static void bb_send(const char *s)
{
    if (s != NULL)
    {
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 100);
    }
}

static void bb_send_ok(void)
{
    bb_send("BB:OK\r\n");
}

/* 标定质量报告（BB:STAT 内嵌 + BB:CAL 单独查询都用它）。
 *
 * gyro_offset_* 是上电时那 2 秒静止标定得到的零偏（这里换成 deg/s 显示）。
 *   · 正常应 |offset| < 1 deg/s。明显偏大说明标定时飞机被碰过/在动，
 *     会直接造成"恒定方向的偏飞"——实测出现过 6.3 deg/s 的假零偏，
 *     表现就是一直往一个方向飞、摇杆却完全回中。
 *   · std 是三轴标准差之和，静止标定应 < 0.5。
 * 不合格就断电重来（上电后别碰飞机，等 ~4 秒启动完成）。 */
static void bb_send_cal(void)
{
    char line[128];
    float std = ICM42605_GetCalibStd();

    (void)snprintf(line, sizeof(line),
                   "BB:CAL gx=%+.3f gy=%+.3f gz=%+.3f dps std=%.3f %s\r\n",
                   (double)(imu_data.gyro_offset_x * RAD_TO_DEG),
                   (double)(imu_data.gyro_offset_y * RAD_TO_DEG),
                   (double)(imu_data.gyro_offset_z * RAD_TO_DEG),
                   (double)std,
                   (std > 0.5f) ? "BAD" : "ok");
    bb_send(line);

    /* 当前**修正后**的陀螺读数（deg/s）。飞机静止放手时应接近 0。
     * 如果这里是 +6 之类的值，说明零偏没减干净 —— 静置约 2 秒后
     * 在线跟踪会自动把它学进去（见 icm42605.c 的 BIAS_LEARN_GYRO_DPS），
     * 再查一次这条就会变回 0 附近。 */
    (void)snprintf(line, sizeof(line),
                   "BB:GYRO cx=%+.3f cy=%+.3f cz=%+.3f dps %s\r\n",
                   (double)(imu_data.gyro_x * RAD_TO_DEG),
                   (double)(imu_data.gyro_y * RAD_TO_DEG),
                   (double)(imu_data.gyro_z * RAD_TO_DEG),
                   (fabsf(imu_data.gyro_y * RAD_TO_DEG) > 1.0f) ? "RESIDUAL(静置等待自愈)" : "ok");
    bb_send(line);

    /* 加计读数（g）。飞机放平、静止时应为 (0, 0, 1)。
     * 若 ax/ay 明显非 0，说明加计零偏没标定好 —— 那会让姿态估计的"水平"
     * 整体歪掉 atan(ax)，控制器就带着这个倾角飞，造成恒定方向偏飞，
     * 且**与 beta 无关**（这是 beta 加到 1.0 仍然偏的原因）。 */
    {
        float ax = imu_data.accel_x;
        float ay = imu_data.accel_y;
        float az = imu_data.accel_z;
        float tilt_deg = atan2f(sqrtf(ax * ax + ay * ay), az) * RAD_TO_DEG;
        const char *judge;

        if (tilt_deg > 2.0f)      { judge = "BAD";   }
        else if (tilt_deg > 1.0f) { judge = "CHECK"; }
        else                      { judge = "ok";    }

        (void)snprintf(line, sizeof(line),
                       "BB:ACC ax=%+.4f ay=%+.4f az=%+.4f g  tilt=%.2fdeg %s\r\n",
                       (double)ax, (double)ay, (double)az,
                       (double)tilt_deg, judge);
        bb_send(line);
    }
}

static void bb_send_status(void)
{
    char line[320];
    BlackBox_Status_t st;

    BlackBox_GetStatus(&st);

    (void)snprintf(line, sizeof(line),
                   "BB:STAT ok=%u boot=%lu sec=%lu off=%lu fid=%lu rec=%lu drop=%lu hwm=%lu armed=%u total=%lu erasing=%u eseq=%lu rec_on=%u\r\n",
                   (unsigned)st.flash_ok,
                   (unsigned long)st.boot_count,
                   (unsigned long)st.sector_index,
                   (unsigned long)st.frame_offset,
                   (unsigned long)st.flight_id,
                   (unsigned long)st.recorded_frames,
                   (unsigned long)st.dropped_frames,
                   (unsigned long)st.ring_hwm,
                   (unsigned)st.armed,
                   (unsigned long)BB_FLASH_TOTAL_BYTES,
                   (unsigned)bb_erasing,
                   (unsigned long)bb_erase_seq,
                   (unsigned)bb_recording);
    bb_send(line);

    /* 接收诊断：rx_total 不涨=字节没进 MCU；ore 在涨=接收溢出。
     * 本行能被回复本身就说明"收字节->组行->执行->回包"整条链路是通的。 */
    (void)snprintf(line, sizeof(line),
                   "BB:RX total=%lu ore=%lu\r\n",
                   (unsigned long)USART3_GetRxTotal(),
                   (unsigned long)USART3_GetOreCount());
    bb_send(line);

    /* ── 标定质量（起飞前必看）── 也可单独发 BB:CAL 查询 */
    bb_send_cal();

    /* 时序抖动（判断黑匣子有没有拖慢控制环 / IMU 采样）──
     * imu_*: IMU 任务相邻两次采样的间隔；fc_*: FlyControl 循环周期。
     * 正常应该是 ~1000~1100µs，且 ov1/ov2 一直为 0。
     * ov1 = 周期>1500µs 的次数(丢一拍)，ov2 = 周期>2500µs 的次数(丢两拍)。 */
    (void)snprintf(line, sizeof(line),
                   "BB:JITTER imu_max=%lu imu_ov1=%lu imu_ov2=%lu fc_max=%lu fc_ov1=%lu fc_ov2=%lu\r\n",
                   (unsigned long)ulImuMaxUs,
                   (unsigned long)ulImuOver1500,
                   (unsigned long)ulImuOver2500,
                   (unsigned long)ulFcMaxUs,
                   (unsigned long)ulFcOver1500,
                   (unsigned long)ulFcOver2500);
    bb_send(line);

    /* Flash 自检诊断：ok=0 时靠这行定位。
     *   step=0xF0 信号量创建失败
     *   step=0xE1 JEDEC 读取失败(SPI3 完成中断没响应 / 超时)
     *   step=0xE2 JEDEC 读到了但与 W25Q128 不符(看 raw 值)
     *   step>=3    已读通，继续后续初始化 */
    (void)snprintf(line, sizeof(line),
                   "BB:FLASH ok=%u step=0x%02X jedec=%02X%02X%02X want=%02X%02X%02X\r\n",
                   (unsigned)st.flash_ok,
                   (unsigned)st.init_step,
                   st.jedec_raw[0], st.jedec_raw[1], st.jedec_raw[2],
                   st.jedec_id[0], st.jedec_id[1], st.jedec_id[2]);
    bb_send(line);
}

static void bb_do_erase(void)
{
    if (bb_armed != 0)
    {
        bb_send("BB:ERR armed\r\n");
        return;
    }
    bb_erase_req = 1;
    bb_send("BB:ERASING ~200s (check BB:STAT for erasing= and eseq=)\r\n");
}

static void bb_do_scan(void)
{
    uint32_t i;
    uint32_t cnt = 0;
    char line[96];
    bb_header_t hdr;

    if (bb_busy != 0)
    {
        bb_send("BB:ERR busy\r\n");
        return;
    }
    if (bb_armed != 0)
    {
        bb_send("BB:ERR armed\r\n");
        return;
    }

    bb_busy = 1;
    if (xSemaphoreTake(bb_flash_mutex, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        bb_busy = 0;
        bb_send("BB:ERR mutex\r\n");
        return;
    }

    for (i = 0; i < BB_SECTOR_COUNT; i++)
    {
        if (bb_read_sector_header(i, &hdr) != 0)
        {
            cnt++;
            (void)snprintf(line, sizeof(line), "BB:HDR idx=%lu phys=%lu off=%u\r\n",
                           (unsigned long)hdr.sector_index,
                           (unsigned long)i,
                           (unsigned)hdr.frame_offset);
            bb_send(line);
        }
    }

    xSemaphoreGive(bb_flash_mutex);
    bb_busy = 0;

    (void)snprintf(line, sizeof(line), "BB:SCANDONE headers=%lu\r\n", (unsigned long)cnt);
    bb_send(line);
}

/* 从 Flash 读一段并发出去。返回 0 成功、非 0 失败（失败时已发过错误行）。
 * 一次最多一页(256B)，因为 bb_page_buf 就是这么大。 */
static uint8_t bb_dump_send_chunk(uint32_t addr, uint32_t len)
{
    if ((len == 0U) || (len > BB_PAGE_SIZE))
    {
        return 1;
    }
    if (xSemaphoreTake(bb_flash_mutex, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        return 1;
    }
    if (w25_read(addr, bb_page_buf, len) != 0)
    {
        xSemaphoreGive(bb_flash_mutex);
        bb_send("\r\nBB:ERR read\r\n");
        return 1;
    }
    xSemaphoreGive(bb_flash_mutex);

    if (HAL_UART_Transmit(&huart3, bb_page_buf, (uint16_t)len, 2000) != HAL_OK)
    {
        return 1;
    }
    return 0;
}

/* 按扇区范围倾倒：从 start_sector 起，共 count_sectors 个扇区。
 *
 * 每个扇区的发送格式（共 BB_HEADER_LEN + BB_SECTOR_DATA_LEN = 3872 字节）：
 *   [ 32 字节索引头 ][ 3840 字节数据区 ]
 * 中间的 32..256 是保留区，不上传（省时间，内容也不用）。
 *
 * 索引头必须发：里面存着逻辑序号 / 已写帧数 / 上电次数，
 * 上位机靠它把扇区分段、判断哪一段是最新一次飞行。
 * （早期版本漏发了它，导致 analyze 报告"未发现有效的扇区索引头"。）
 *
 * ★ 协议约定：本函数发出的**第一行永远是 "BB:DUMP ..." 表头**。
 *   上位机靠它判断是否进入倾倒模式，所以在此之前不能发任何其它行。 */
static void bb_dump_range(uint32_t start_sector, uint32_t count_sectors, uint8_t is_empty)
{
    uint32_t end_sector;
    uint32_t total_bytes;
    uint32_t sec;
    char head[160];

    if (bb_busy != 0)
    {
        bb_send("BB:ERR busy\r\n");
        return;
    }
    if (bb_armed != 0)
    {
        bb_send("BB:ERR armed\r\n");     /* 飞行中禁止长时间占用 */
        return;
    }
    if ((start_sector >= BB_SECTOR_COUNT) || (count_sectors == 0U))
    {
        bb_send("BB:ERR range\r\n");
        return;
    }

    end_sector = start_sector + count_sectors;
    if (end_sector > BB_SECTOR_COUNT)
    {
        end_sector = BB_SECTOR_COUNT;
    }
    count_sectors = end_sector - start_sector;
    total_bytes = count_sectors * (BB_DUMP_HEADER_LEN + BB_SECTOR_DATA_LEN);

    /* 把范围告知上位机：start=起始扇区 hdr=每扇区索引头字节数 empty=1 表示没录过数据 */
    (void)snprintf(head, sizeof(head),
                   "BB:DUMP start=%lu sectors=%lu frames=%lu bytes=%lu hdr=%u empty=%u\r\n",
                   (unsigned long)start_sector,
                   (unsigned long)count_sectors,
                   (unsigned long)(count_sectors * BB_FRAMES_PER_SECTOR),
                   (unsigned long)total_bytes,
                   (unsigned)BB_DUMP_HEADER_LEN,
                   (unsigned)is_empty);
    bb_send(head);

    bb_busy = 1;

    for (sec = start_sector; sec < end_sector; sec++)
    {
        uint32_t base = sec * BB_SECTOR_SIZE;
        uint32_t off;

        /* 1) 索引头 32 字节（放在扇区最开头） */
        if (bb_dump_send_chunk(base, BB_DUMP_HEADER_LEN) != 0)
        {
            bb_busy = 0;
            return;
        }

        /* 2) 数据区 3840 字节，按 256 字节页发送 */
        for (off = BB_SECTOR_DATA_OFF; off < BB_SECTOR_SIZE; off += BB_PAGE_SIZE)
        {
            uint32_t chunk = BB_PAGE_SIZE;
            if ((off + chunk) > BB_SECTOR_SIZE)
            {
                chunk = BB_SECTOR_SIZE - off;
            }
            if (bb_dump_send_chunk(base + off, chunk) != 0)
            {
                bb_busy = 0;
                return;
            }
        }
    }

    bb_busy = 0;
    bb_send("\r\nBB:DUMPEND\r\n");
}

/* 智能倾倒：只发"有数据"的扇区。
 * 黑匣子的数据是从扇区 0 开始连续写的，所以只需发 [0, 最后一个有数据的扇区]。
 * 没录过数据时，仍发 1 个扇区（上位机拿到一个合法全空镜像），
 * 并在表头里用 empty=1 说明——**绝不额外先发别的行**，否则上位机会认不出表头。 */
static void bb_dump_used(void)
{
    uint32_t last = 0;

    if (bb_get_used_range(&last) == 0)
    {
        bb_dump_range(0U, 1U, 1U);      /* 整片空白 */
        return;
    }
    bb_dump_range(0U, last + 1U, 0U);
}

static void bb_set_baud(uint32_t baud){
    if ((baud < 9600U) || (baud > 6000000U))
    {
        bb_send("BB:ERR baud\r\n");
        return;
    }
    huart3.Init.BaudRate = baud;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        bb_send("BB:ERR baudinit\r\n");
        return;
    }
    bb_send("BB:OK\r\n");
}

/* ==========================================================================
 * 阶跃注入（BB:STEP）
 *
 * 用法：  BB:STEP <roll|pitch> <幅度deg> <周期ms> <次数>
 *        BB:STEP OFF
 *
 * 它把一个"+amp / -amp"方波叠加到目标角上，每个半周期就是一次
 * 完整的阶跃（从 -amp 跳到 +amp，幅度 2*amp）。同时目标角本身会被
 * 记进摇杆帧，上位机直接从里面取设定值，不需要推算时序。
 *
 * 建议周期取 64ms 的整数倍（64/512/1024），这样半周期边沿恰好
 * 落在摇杆帧（每 32ms 一帧）上，边沿时刻无误差。
 * ========================================================================== */
#define BB_STEP_MAX_AMP_DEG     25.0f
#define BB_STEP_MIN_PERIOD_MS   64U
#define BB_STEP_MAX_PERIOD_MS   4000U
#define BB_STEP_MAX_CYCLES      10U
#define BB_STEP_STICK_CANCEL    40U      /* 摇杆偏离中位超过此值就取消（10° 对应 90） */

static volatile uint8_t  bb_step_pending  = 0U;   /* 已下命令、等待飞行状态触发 */
static volatile uint8_t  bb_step_active   = 0U;   /* 正在注入方波 */
static volatile uint8_t  bb_step_axis     = 0U;   /* 0=roll 1=pitch */
static volatile float    bb_step_amp      = 0.0f;
static volatile uint32_t bb_step_half_ms  = 0U;
static volatile uint32_t bb_step_total_ms = 0U;
static volatile uint32_t bb_step_t0       = 0U;
static volatile uint32_t bb_step_pend_t0  = 0U;   /* 下命令的时刻，用于超时作废 */
static volatile uint32_t bb_step_ready_ms = 0U;   /* 飞行状态已连续满足多久 */

#define BB_STEP_PENDING_TIMEOUT_MS  120000U   /* 2 分钟内没飞就作废 */

void BlackBox_StepStop(void)
{
    bb_step_active = 0U;
}

void BlackBox_GetStepOffset(float *roll_off_deg, float *pitch_off_deg,
                            uint16_t stick_roll, uint16_t stick_pitch,
                            uint8_t flight_ready)
{
    float off = 0.0f;
    uint32_t now = (uint32_t)HAL_GetTick();

    if ((roll_off_deg == NULL) || (pitch_off_deg == NULL))
    {
        return;
    }

    if ((bb_step_pending == 0U) && (bb_step_active == 0U))
    {
        return;                                  /* 什么都没开，直接返回 0 */
    }

    /* 飞手接管：摇杆一动立即取消（待触发也一样），不与人抢控制 */
    {
        int32_t dr = (int32_t)stick_roll  - 1500;
        int32_t dp = (int32_t)stick_pitch - 1500;
        if ((dr > (int32_t)BB_STEP_STICK_CANCEL) || (dr < -(int32_t)BB_STEP_STICK_CANCEL) ||
            (dp > (int32_t)BB_STEP_STICK_CANCEL) || (dp < -(int32_t)BB_STEP_STICK_CANCEL))
        {
            bb_step_pending = 0U;
            bb_step_active  = 0U;
            return;
        }
    }

    if ((bb_step_pending != 0U) && (bb_step_active == 0U))
    {
        /* 命令超时：地上下了命令但一直没飞，不让它到下次飞行才突然发作 */
        if ((now - bb_step_pend_t0) >= BB_STEP_PENDING_TIMEOUT_MS)
        {
            bb_step_pending = 0U;
            return;
        }

        /* ★ 触发：飞行状态连续满足 500ms 后开始。
         *   flight_ready 由控制循环算（已解锁 + 油门足够 + 姿态平稳），
         *   加上这里已经确认摇杆回中，所以不会在地面或手上就开始摆。 */
        if (flight_ready != 0U)
        {
            if (bb_step_ready_ms == 0U)
            {
                bb_step_ready_ms = now;
            }
            else if ((now - bb_step_ready_ms) >= 500U)
            {
                bb_step_t0      = now;
                bb_step_active  = 1U;
                bb_step_pending = 0U;
                bb_step_ready_ms = 0U;
            }
        }
        else
        {
            bb_step_ready_ms = 0U;               /* 飞行状态不满足，重新计时 */
        }
    }

    if (bb_step_active != 0U)
    {
        uint32_t el = now - bb_step_t0;
        if (el >= bb_step_total_ms)
        {
            bb_step_active = 0U;                 /* 序列跑完，自动停止 */
        }
        else if (bb_step_half_ms > 0U)
        {
            off = (((el / bb_step_half_ms) & 1U) != 0U) ? -bb_step_amp : bb_step_amp;
        }
    }

    if (bb_step_axis == 0U)
    {
        *roll_off_deg  = off;
        *pitch_off_deg = 0.0f;
    }
    else
    {
        *roll_off_deg  = 0.0f;
        *pitch_off_deg = off;
    }
}


static void bb_do_step(const char *arg)
{
    char line[128];
    char axis[8];
    int  k = 0;
    float amp;
    uint32_t period, cycles;
    const char *p = arg;

    while ((*p == ' ') || (*p == '\t')) { p++; }

    if ((*p == '\0') || (strncmp(p, "OFF", 3U) == 0) || (strncmp(p, "off", 3U) == 0))
    {
        bb_step_pending = 0U;
        bb_step_active  = 0U;
        bb_send("BB:STEP off\r\n");
        return;
    }

    while ((*p != '\0') && (*p != ' ') && (k < (int)sizeof(axis) - 1))
    {
        axis[k++] = *p++;
    }
    axis[k] = '\0';
    while (*p == ' ') { p++; }

    amp = (float)atof(p);
    while ((*p != '\0') && (*p != ' ')) { p++; }
    while (*p == ' ') { p++; }
    period = (uint32_t)strtoul(p, NULL, 10);

    {
        const char *q = strchr(p, ' ');
        cycles = (q != NULL) ? (uint32_t)strtoul(q + 1, NULL, 10) : 1U;
    }

    if ((strcmp(axis, "roll") == 0) || (strcmp(axis, "ROLL") == 0))
    {
        bb_step_axis = 0U;
    }
    else if ((strcmp(axis, "pitch") == 0) || (strcmp(axis, "PITCH") == 0))
    {
        bb_step_axis = 1U;
    }
    else
    {
        bb_send("BB:ERR step axis (roll|pitch)\r\n");
        return;
    }

    if (amp < 0.0f) { amp = -amp; }
    if (amp < 2.0f)   { amp = 2.0f; }     /* 太小的阶跃分辨不出 */
    if (amp > BB_STEP_MAX_AMP_DEG) { amp = BB_STEP_MAX_AMP_DEG; }
    if (period < BB_STEP_MIN_PERIOD_MS) { period = BB_STEP_MIN_PERIOD_MS; }
    if (period > BB_STEP_MAX_PERIOD_MS) { period = BB_STEP_MAX_PERIOD_MS; }
    if (cycles < 1U) { cycles = 1U; }
    if (cycles > BB_STEP_MAX_CYCLES) { cycles = BB_STEP_MAX_CYCLES; }

    bb_step_amp      = amp;
    bb_step_half_ms  = period / 2U;
    if (bb_step_half_ms < 32U) { bb_step_half_ms = 32U; }
    bb_step_total_ms = period * cycles;
    bb_step_t0       = (uint32_t)HAL_GetTick();
    bb_step_pend_t0  = (uint32_t)HAL_GetTick();
    bb_step_ready_ms = 0U;
    bb_step_active   = 0U;
    bb_step_pending  = 1U;      /* 等待飞行状态触发，而不是立即开始 */

    (void)snprintf(line, sizeof(line),
                   "BB:STEP %s amp=%+.1f period=%lums cycles=%lu total=%lums armed\r\n",
                   (bb_step_axis == 0U) ? "roll" : "pitch",
                   (double)bb_step_amp,
                   (unsigned long)period,
                   (unsigned long)cycles,
                   (unsigned long)bb_step_total_ms);
    bb_send(line);
}

/* ==================================================================
 * BB:CLK —— 时基自检
 *
 * 什么时候用：怀疑"时间不对"的时候。本工程有三套互不相干的时间源：
 *   · DWT 周期计数器 → DWT_GetUS()：CPU 周期 / SystemCoreClock，
 *     黑匣子的 dt、配置存储的擦写耗时都靠它；
 *   · HAL_GetTick()：TIM6 中断里累加的 uwTick，所有 HAL 超时、CFG:SAVE
 *     里记录的 up= 都靠它；
 *   · FreeRTOS tick：SysTick 中断，osDelay / vTaskDelayUntil 靠它。
 *
 * 做法：读一次 → 让出 1 秒 → 再读一次，打印三个增量。
 * 三个都应该 ≈1000ms。谁偏得离谱，问题就在那条链上：
 *   d_hal 大而 d_dwt 正常 → TIM6 时基错（PSC/ARR/中断频率）
 *   d_dwt 大而 d_hal 正常 → SystemCoreClock 与实际主频不符
 *   d_rtos 异常          → SysTick 重装值/中断优先级问题
 *
 * 注意：会阻塞 1 秒，只在地面用（解锁时直接拒绝）。
 * ================================================================== */
static void bb_do_clk(void)
{
    char     line[176];
    uint32_t ms0, ms1, us0, us1, rt0, rt1;
    extern TIM_HandleTypeDef htim6;

    if (bb_armed != 0U)
    {
        bb_send("BB:ERR armed\r\n");
        return;
    }

    ms0 = HAL_GetTick();
    us0 = DWT_GetUS();
    rt0 = (uint32_t)xTaskGetTickCount();

    osDelay(1000);

    ms1 = HAL_GetTick();
    us1 = DWT_GetUS();
    rt1 = (uint32_t)xTaskGetTickCount();

    (void)snprintf(line, sizeof(line),
                   "BB:CLK d_hal=%lums d_dwt=%lums d_rtos=%lums (all should be ~1000)\r\n",
                   (unsigned long)(ms1 - ms0),
                   (unsigned long)((us1 - us0) / 1000U),
                   (unsigned long)(rt1 - rt0));
    bb_send(line);

    (void)snprintf(line, sizeof(line),
                   "BB:CLK sysclk=%lu hclk=%lu pclk1=%lu\r\n",
                   (unsigned long)SystemCoreClock,
                   (unsigned long)HAL_RCC_GetHCLKFreq(),
                   (unsigned long)HAL_RCC_GetPCLK1Freq());
    bb_send(line);

    (void)snprintf(line, sizeof(line),
                   "BB:CLK tim6 psc=%lu arr=%lu cr1=%04lX  total hal=%lums dwt=%lums rtos=%lums\r\n",
                   (unsigned long)TIM6->PSC,
                   (unsigned long)TIM6->ARR,
                   (unsigned long)TIM6->CR1,
                   (unsigned long)ms1,
                   (unsigned long)(us1 / 1000U),
                   (unsigned long)rt1);
    bb_send(line);
}

/* 控制台触发的重新标定（档 2：陀螺零偏 + 加计零偏）。
 *
 * 为什么在这里等：真正的测量在 IMU 任务里做（2 秒），这里只负责
 * 发请求、等结果、把结果写进**待生效**参数表。写完还要 CFG:SAVE
 * 才会落 Flash，再 CFG:REBOOT 才生效 —— 与其它参数同一套流程。
 *
 * 安全：解锁中直接拒绝。标定会阻塞 IMU 任务 2 秒，空中做等于失控。 */
static void bb_do_calibrate(void)
{
    char line[176];
    uint32_t t0;
    uint8_t st = 0U;
    float gx = 0.0f, gy = 0.0f, gz = 0.0f;
    float ax = 0.0f, ay = 0.0f, az = 0.0f;

    if (bb_armed != 0U)
    {
        bb_send("CFG:ERR armed\r\n");
        return;
    }

    bb_busy = 1;
    ICM42605_RequestCalibration();
    bb_send("CFG:CAL start (keep still ~2s)\r\n");

    t0 = (uint32_t)HAL_GetTick();
    while ((uint32_t)(HAL_GetTick() - t0) < 8000U)
    {
        st = ICM42605_CalibrationState();
        if ((st == 3U) || (st == 4U))
        {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

    if (st != 3U)
    {
        bb_send((st == 4U) ? "CFG:ERR armed\r\n" : "CFG:ERR cal_timeout\r\n");
        bb_busy = 0;
        return;
    }

    ICM42605_GetCalibration(&gx, &gy, &gz, &ax, &ay, &az);

    (void)snprintf(line, sizeof(line),
                   "CFG:CAL gx=%+.3f gy=%+.3f gz=%+.3f dps\r\n",
                   (double)gx, (double)gy, (double)gz);
    bb_send(line);
    (void)snprintf(line, sizeof(line),
                   "CFG:CAL ax=%+.4f ay=%+.4f az=%+.4f g std=%.3f\r\n",
                   (double)ax, (double)ay, (double)az,
                   (double)ICM42605_GetCalibStd());
    bb_send(line);

    /* 写进待生效表（内存，不写 Flash） */
    (void)Param_Set(P_CAL_GYRO_X, gx);
    (void)Param_Set(P_CAL_GYRO_Y, gy);
    (void)Param_Set(P_CAL_GYRO_Z, gz);
    (void)Param_Set(P_CAL_ACCEL_X, ax);
    (void)Param_Set(P_CAL_ACCEL_Y, ay);
    (void)Param_Set(P_CAL_ACCEL_Z, az);

    (void)snprintf(line, sizeof(line),
                   "CFG:CALEND items=6 dirty=%u\r\n",
                   (unsigned)Param_IsDirty());
    bb_send(line);
    bb_busy = 0;
}

static void bb_execute(const char *cmd)
{
    if (strcmp(cmd, "BB:ERASE") == 0)
    {
        bb_do_erase();
    }
    else if (strcmp(cmd, "BB:DUMP") == 0)
    {
        /* 只发有数据的部分（默认） */
        bb_dump_used();
    }
    else if (strcmp(cmd, "BB:DUMPALL") == 0)
    {
        /* 整片 16MB，用于诊断或数据绕回后的完整备份 */
        bb_dump_range(0U, BB_SECTOR_COUNT, 0U);
    }
    else if (strncmp(cmd, "BB:DUMP:", 8) == 0)
    {
        /* BB:DUMP:<起始扇区>:<扇区数> —— 指定范围 */
        uint32_t s = (uint32_t)strtoul(&cmd[8], NULL, 10);
        const char *p = strchr(&cmd[8], ':');
        if (p != NULL)
        {
            bb_dump_range(s, (uint32_t)strtoul(p + 1, NULL, 10), 0U);
        }
        else
        {
            bb_send("BB:ERR args\r\n");
        }
    }
    else if (strcmp(cmd, "BB:STAT") == 0)
    {
        bb_send_status();
        bb_send_ok();
    }
    else if (strcmp(cmd, "BB:CAL") == 0)
    {
        /* 独立查询上电标定质量：起飞前自检用，不打印其它状态行 */
        bb_send_cal();
        bb_send_ok();
    }
    else if (strcmp(cmd, "BB:SCAN") == 0)
    {
        bb_do_scan();
    }
    else if (strcmp(cmd, "BB:CLK") == 0)
    {
        /* 时基自检：DWT / HAL tick / FreeRTOS tick 三者互相对比 */
        bb_do_clk();
    }
    else if (strcmp(cmd, "BB:PING") == 0)
    {
        bb_send_ok();
    }
    else if (strncmp(cmd, "BB:STEP", 7) == 0)
    {
        /* 阶跃注入：BB:STEP <roll|pitch> <amp> <period_ms> <cycles> | BB:STEP OFF */
        bb_do_step(&cmd[7]);
        bb_send_ok();
    }
    else if (strncmp(cmd, "BB:BAUD:", 8) == 0)
    {
        bb_set_baud((uint32_t)strtoul(&cmd[8], NULL, 10));
    }
    else if (strcmp(cmd, "CFG:CAL") == 0)
    {
        /* 必须排在下面 CFG: 的通配分支之前，否则会被 Param_Console 判成未知命令 */
        bb_do_calibrate();
    }
    else if (strncmp(cmd, "CFG:", 4) == 0)
    {
        /* 配置存储（内部 Flash 模拟 EEPROM）。
         * 为什么在这里分发：这条命令通道本来就在最低优先级的 Logger 任务里跑，
         * 而且擦除会停顿上秒 —— 放在控制任务里是绝对不能接受的。
         *
         * bb_busy 置 1：其它任务（IMU 标定等）的 printf 会被 fputc 丢掉，
         * 保证这一串回包不会被插字。param/flash_ee 的输出直接写 USART3，不受影响。 */
        bb_busy = 1;
        if ((Param_Console(cmd) == 0U) && (EE_Console(cmd) == 0U))
        {
            bb_send("CFG:ERR unknown (try CFG:HELP)\r\n");
        }
        bb_busy = 0;
    }
    else
    {
        bb_send("BB:ERR unknown\r\n");
    }
}

void BlackBox_FeedCommandByte(uint8_t byte)
{
    if ((byte == (uint8_t)'\r') || (byte == (uint8_t)'\n'))
    {
        if ((bb_cmd_len > 0U) && (bb_cmd_pending == 0U))
        {
            bb_cmd_line[bb_cmd_len] = '\0';
            memcpy(bb_cmd_ready, bb_cmd_line, bb_cmd_len + 1U);
            bb_cmd_pending = 1U;
            bb_cmd_len = 0U;
        }
        return;
    }

    if (bb_cmd_len < (BB_CMD_LINE_MAX - 1U))
    {
        bb_cmd_line[bb_cmd_len++] = (char)byte;
    }
    else
    {
        bb_cmd_len = 0U;
    }
}

void BlackBox_Poll(void)
{
    if (bb_cmd_pending != 0U)
    {
        bb_cmd_pending = 0U;
        bb_execute(bb_cmd_ready);
    }
}

uint8_t BlackBox_RequestDump(void)
{
    if (bb_busy != 0)
    {
        return 1;
    }
    bb_dump_used();
    return 0;
}
