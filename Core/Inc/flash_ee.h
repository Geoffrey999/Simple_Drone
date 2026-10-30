/**
  ******************************************************************************
  * @file    flash_ee.h
  * @brief   STM32F405 内部 Flash 模拟 EEPROM（双 bank 追加式日志 + CRC32）
  *
  * 为什么用内部 Flash 而不是板上那颗 W25Q128：
  *   · 配置是"让飞机能飞"的东西，不该依赖黑匣子那颗外部 Flash；
  *   · 上电扫描几毫秒就能同步完成，不用等 SPI3 初始化和芯片自检(2.5s)；
  *   · BB:ERASE(全片擦除)不会连配置一起擦掉。
  *   （代价：128KB 扇区擦除是秒级，见下面"擦除时机"）
  *
  * ── 存储介质 ──
  *   扇区 10 : 0x080C0000, 128KB = Bank A
  *   扇区 11 : 0x080E0000, 128KB = Bank B
  *   ⚠ 必须把 Keil 的 IROM 从 1MB(0x100000) 改成 0xC0000，否则链接器
  *     可能把代码放进这块保留区。EE_Init() 里有运行时守卫
  *     （比较链接器符号 Image$$ER_IROM1$$Limit），一旦越界就自动锁成只读。
  *
  * ── 布局 ──
  *   每个 bank = 128 个 1KB 记录槽，新记录只追加到"擦除态"的槽里。
  *   一条记录 = 16B 头(magic/ver/plen/seq/crc32) + 载荷(≤1008B)。
  *
  * ── 掉电语义 ──
  *   写新记录时旧记录原封不动；一个 bank 写满才去擦"另一个" bank。
  *   所以任何时刻断电，盘上都至少留着一条完整可用的记录。
  *   每次擦除可换 128 次保存 → 10k 次擦除寿命 ≈ 128 万次保存。
  *
  * ── 擦除时机（唯一的硬约束）──
  *   平时保存只要 ~2ms；第 128、256、384… 次保存会触发一次 1~2s 的扇区擦除，
  *   期间 CPU 取指停顿、所有任务冻结。因此只在**未解锁**时允许写。
  *
  * ── P1 阶段说明 ──
  *   本文件当前只提供驱动 + 自检命令（CFG:INFO/SAVE/LOAD/FILL/RAW/ERASE），
  *   载荷是一个内部测试结构，**还没有接任何飞行参数**。
  *   P2 会把参数表接上来，把 CFG:SET/LIST/HIST 等命令挪到 param.c。
  ******************************************************************************
  */

#ifndef __FLASH_EE_H__
#define __FLASH_EE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ============================== 几何参数 ============================== */
#define EE_BANK0_BASE      0x080C0000UL                          /* 扇区 10 */
#define EE_BANK1_BASE      0x080E0000UL                          /* 扇区 11 */
#define EE_BANK_SIZE       0x00020000UL                          /* 128 KB   */
#define EE_RECORD_SIZE     1024UL                                /* 槽 1 KB  */
#define EE_HDR_SIZE        16UL
#define EE_SLOT_COUNT      ((uint16_t)(EE_BANK_SIZE / EE_RECORD_SIZE))   /* 128 */
#define EE_PAYLOAD_MAX     ((uint16_t)(EE_RECORD_SIZE - EE_HDR_SIZE))    /* 1008 */
#define EE_MAGIC           0x45454346UL     /* "FCEE"（擦除态 0xFFFFFFFF 天然不等）*/
#define EE_VER             1U               /* 记录格式版本 */

/* ============================== 返回值 ============================== */
typedef enum {
    EE_OK             = 0,   /* 成功写入                                        */
    EE_OK_UNCHANGED   = 1,   /* 内容和最新记录相同，没有写 Flash                */
    EE_ERR_ARG        = 10,  /* 参数非法（长度非 4 字节整数倍 / 超上限 / NULL） */
    EE_ERR_NOREC      = 11,  /* 没有有效记录                                    */
    EE_ERR_ARMED      = 12,  /* 已解锁，拒绝写/擦                               */
    EE_ERR_IMAGE      = 13,  /* 镜像越过保留区，EE 已锁成只读                   */
    EE_ERR_STATE      = 14,  /* 未初始化 / 写失败后不再尝试                     */
    EE_ERR_FLASH      = 15,  /* 擦除或编程失败                                  */
    EE_ERR_VERIFY     = 16   /* 回读校验失败                                    */
} EE_Result;

/* ============================== 状态 ============================== */
typedef struct {
    uint8_t  ready;         /* 1 = 初始化完成且允许写                    */
    uint8_t  have;          /* 1 = 有一份有效记录                        */
    uint8_t  bank;          /* 当前活动 bank：0=Bank A, 1=Bank B         */
    uint8_t  img_overlap;   /* 1 = 镜像越界（EE 锁死，只读）             */
    uint8_t  last_err;      /* 最近一次错误码（EE_Result）               */
    uint16_t next_slot;     /* 下一次保存要写的槽号                      */
    uint16_t used_slots;    /* 活动 bank 已用槽数（= 历史深度）          */
    uint16_t plen;          /* 最新记录载荷长度                          */
    uint32_t seq;           /* 最新记录序号                              */
    uint32_t image_end;     /* 镜像末尾地址（Image$$ER_IROM1$$Limit）    */
    uint32_t saves;         /* 本次上电成功保存次数                      */
    uint32_t erases;        /* 本次上电扇区擦除次数                      */
    uint32_t crc_fail;      /* 扫描时 CRC 校验失败的记录数               */
    uint32_t bad_hdr;       /* 扫描时头部非法(版本/长度)的记录数         */
    uint32_t wr_us;         /* 上次写入耗时(µs)                          */
    uint32_t er_us;         /* 上次擦除耗时(µs)                          */
} FlashEE_Status_t;

/* ============================== API ============================== */

/* 上电初始化：扫描两个 bank 找最新有效记录。
 * 只做 Flash 读，几毫秒完成，可直接在 main() 里、调度器启动前调用。
 * 返回 1 = 可用；0 = 镜像越界（此时只读，任何写都会返回 EE_ERR_IMAGE）。 */
uint8_t EE_Init(void);

/* 追加保存一条记录。len 必须是 4 的整数倍且 ≤ EE_PAYLOAD_MAX。
 * 内容与最新记录完全相同时直接返回 EE_OK_UNCHANGED，不消耗槽位。 */
uint8_t EE_Save(const uint8_t *payload, uint16_t len);

/* 读出最新一条记录。out_len 可为 NULL。 */
uint8_t EE_Read(uint8_t *buf, uint16_t buf_size, uint16_t *out_len);

/* 历史记录（供参数回滚）。idx = 0 是最新一条。
 * 会跳过写入中途掉电留下的半条记录，所以读到的都是 CRC 通过的。 */
uint16_t EE_HistoryCount(void);
uint8_t  EE_ReadHistory(uint16_t idx, uint8_t *buf, uint16_t buf_size,
                        uint16_t *len, uint32_t *seq, uint16_t *slot);

/* 擦除两个 bank（仅未解锁）。RAM 里的状态会复位成"无记录"。 */
uint8_t EE_EraseAll(void);

void        EE_GetStatus(FlashEE_Status_t *st);
const char *EE_ResultStr(uint8_t r);

/* 控制台：只处理与"原始记录"有关的 CFG:RAW / CFG:ERASE（P2 起参数类命令
 * 归 param.c）。返回 1 = 已处理，0 = 不认识。
 * 输出直接走 USART3（绕过 printf），这样上层用 bb_busy 屏蔽 printf 时
 * 不会连自己的回包一起丢掉。 */
uint8_t EE_Console(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_EE_H__ */
