/**
  ******************************************************************************
  * @file    flash_ee.c
  * @brief   STM32F405 内部 Flash 模拟 EEPROM 实现（P1：驱动 + 自检命令）
  *
  * 设计要点（为什么要这么绕）：
  *
  * 1) Flash 不能原地改字节，只能"整扇区擦 + 往空格里写"。
  *    所以一次保存 = 往下一个空槽**追加**一条新记录，不是覆盖旧值。
  *    好处：写的时候旧记录一个字节都没动，掉电最多丢正在写的那条。
  *
  * 2) 双 bank 交替。一个 bank 写满 128 条才去擦**另一个** bank，
  *    保证任何时刻都有一份完整可用的配置。
  *    代价：那次擦除 1~2s，CPU 取指全程停顿 —— 所以只许未解锁时写。
  *
  * 3) 固定 1KB 槽位（而不是变长记录）。扫描时按常数步进跳槽，
  *    一条坏记录不会让后面全部错位。这是这类实现最容易踩的坑。
  *
  * 4) 头部带 seq（单调递增）和 CRC32。seq 用来判断"谁是新的"，
  *    CRC 用来判断"这条是不是完整的"。两者都不能省：
  *    只看 magic，半条记录也会被当成有效；只看 CRC，空槽 0xFFFFFFFF
  *    的 CRC 也可能碰巧对上（概率不为 0）。
  *
  * 5) 镜像越界守卫。若工程被 CubeMX 重新生成导致 IROM 又变回 1MB，
  *    代码有可能被链接进保留区，此时擦除会把自己的代码擦掉。
  *    EE_Init() 用链接器符号 Image$$ER_IROM1$$Limit 检查，
  *    越界就把 EE 锁成只读，宁可用默认参数也不能擦掉固件。
  ******************************************************************************
  */

#include "flash_ee.h"
#include "usart.h"
#include "blackbox.h"
#include "dwt.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

/* ==================================================================
 *  编译期断言：结构体/几何参数被改坏时直接编译报错
 * ================================================================== */
#define EE_SIZE_CHECK(name, cond) typedef char name[((cond) ? 1 : -1)]

/* 一条记录（1KB）的头部，字段顺序即 Flash 里的字节顺序（逐字段序列化，不依赖打包） */
typedef struct {
    uint32_t magic;   /*  0  EE_MAGIC                                  */
    uint16_t ver;     /*  4  记录格式版本                              */
    uint16_t plen;    /*  6  载荷长度（4 字节整数倍）                  */
    uint32_t seq;     /*  8  保存序号（单调递增，0xFFFFFFFF 保留）     */
    uint32_t crc;     /* 12  CRC32(bytes[4..12) ‖ 载荷)                */
} ee_hdr_t;           /* = 16 B                                        */

EE_SIZE_CHECK(ee_hdr_size_check,  (sizeof(ee_hdr_t) == EE_HDR_SIZE));
EE_SIZE_CHECK(ee_record_align,    ((EE_RECORD_SIZE % 4U) == 0U));
EE_SIZE_CHECK(ee_bank_fit,        (((uint32_t)EE_SLOT_COUNT * EE_RECORD_SIZE) <= EE_BANK_SIZE));
EE_SIZE_CHECK(ee_payload_calc,    ((EE_HDR_SIZE + (uint32_t)EE_PAYLOAD_MAX) == EE_RECORD_SIZE));
EE_SIZE_CHECK(ee_magic_not_erased, (EE_MAGIC != 0xFFFFFFFFUL));

/* ==================================================================
 *  内部状态
 * ================================================================== */
static uint32_t s_rec_w[EE_RECORD_SIZE / 4U];      /* 1KB 写缓冲，天然 4 字节对齐 */
#define S_REC_BYTE ((uint8_t *)s_rec_w)

static FlashEE_Status_t s_ee;

/* 镜像末尾（Keil/ARMCC 链接器符号）。若区域被改名，链接阶段会报未定义符号，
 * 属于"编译期就能发现"的问题，不会留到飞行时。 */
extern uint32_t Image$$ER_IROM1$$Limit;

/* ==================================================================
 *  一、小工具
 * ================================================================== */
static uint32_t ee_bank_base(uint8_t bank)
{
    return (bank == 0U) ? EE_BANK0_BASE : EE_BANK1_BASE;
}

static uint32_t ee_slot_addr(uint8_t bank, uint16_t slot)
{
    return ee_bank_base(bank) + ((uint32_t)slot * EE_RECORD_SIZE);
}

/* 读槽位第一个字：0xFFFFFFFF 表示这个槽是空的（可以写） */
static uint32_t ee_slot_first_word(uint8_t bank, uint16_t slot)
{
    return *(const volatile uint32_t *)ee_slot_addr(bank, slot);
}

/* seq 的大小比较：用有符号差处理回绕（a 比 b 新返回 1） */
static uint8_t ee_seq_newer(uint32_t a, uint32_t b)
{
    return (uint8_t)(((int32_t)(a - b)) > 0);
}

/* 致命错误：把 EE 锁成只读，本次上电不再尝试写。
 * 为什么要锁：擦/写失败后继续反复擦写只会让坏块更坏，
 * 而且这类故障在空中出现时，反复停顿比直接放弃更危险。
 * 上电重新扫描会自己恢复（前提是盘上的旧记录还完好）。 */
static uint8_t ee_fatal(uint8_t err)
{
    s_ee.last_err = err;
    s_ee.ready    = 0U;
    return err;
}

/* ---- USART3 输出：故意不用 printf ----
 * 上层（blackbox.c）在 CFG: 命令执行期间会把 bb_busy 置 1 以屏蔽其它任务的
 * printf，而 usart.c 的 fputc 一旦看到 bb_busy 就丢字符。所以这里直接走
 * HAL_UART_Transmit，绕过 fputc。 */
static void ee_send(const char *s)
{
    if (s != NULL)
    {
        (void)HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 200);
    }
}

static void ee_printf(const char *fmt, ...)
{
    char buf[192];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    ee_send(buf);
}

/* ==================================================================
 *  二、CRC32（软件按位实现，不占 1KB 查表空间）
 * ================================================================== */
static uint32_t ee_crc_core(uint32_t crc, const volatile uint8_t *p, uint32_t n)
{
    uint32_t i;
    uint8_t  b;

    for (i = 0; i < n; i++)
    {
        crc ^= (uint32_t)p[i];
        for (b = 0; b < 8U; b++)
        {
            crc = (crc >> 1) ^ (0xEDB88320UL & (uint32_t)(-(int32_t)(crc & 1U)));
        }
    }
    return crc;
}

/* 算一条记录的 CRC：覆盖头部 ver/plen/seq（8 字节）+ 载荷 */
static uint32_t ee_crc_record(const ee_hdr_t *h, const volatile uint8_t *payload)
{
    uint32_t crc = 0xFFFFFFFFUL;

    crc = ee_crc_core(crc, (const volatile uint8_t *)h + 4, 8U);
    if (h->plen != 0U)
    {
        crc = ee_crc_core(crc, payload, h->plen);
    }
    return crc ^ 0xFFFFFFFFUL;
}

/* ==================================================================
 *  三、Flash 低层：擦扇区 / 按字编程
 * ================================================================== */
static void ee_clear_err_flags(void)
{
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

/* 擦一个 bank（= 一个 128KB 扇区）。
 * ★ 这个函数会阻塞 1~2s（手册典型 2s），期间取指停顿、所有任务冻结。
 *   这是 flash 的物理特性，不是 bug —— 所以调用点必须确认"未解锁"。 */
static uint8_t ee_erase_bank(uint8_t bank)
{
    FLASH_EraseInitTypeDef init;
    uint32_t sector_err = 0;
    uint32_t t0;
    uint8_t  rc = 0;

    t0 = DWT_GetUS();

    HAL_FLASH_Unlock();
    ee_clear_err_flags();

    memset(&init, 0, sizeof(init));
    init.TypeErase    = FLASH_TYPEERASE_SECTORS;
    init.VoltageRange = FLASH_VOLTAGE_RANGE_3;               /* 2.7~3.6V */
    init.Sector       = (bank == 0U) ? FLASH_SECTOR_10 : FLASH_SECTOR_11;
    init.NbSectors    = 1;

    if (HAL_FLASHEx_Erase(&init, &sector_err) != HAL_OK)
    {
        rc = 1;
    }

    HAL_FLASH_Lock();

    s_ee.er_us = DWT_GetUS() - t0;

    /* 回读确认：擦完第一个字必须是 0xFFFFFFFF，否则说明扇区没擦干净 */
    if ((rc == 0U) && (ee_slot_first_word(bank, 0) != 0xFFFFFFFFUL))
    {
        rc = 1;
    }
    if (rc == 0U)
    {
        s_ee.erases++;
    }
    return rc;
}

/* 从 addr 起按 32bit 字编程 n 个字（调用者保证 4 字节对齐） */
static uint8_t ee_program_words(uint32_t addr, const uint32_t *src, uint32_t n)
{
    uint32_t i;
    uint8_t  rc = 0;

    HAL_FLASH_Unlock();
    ee_clear_err_flags();

    for (i = 0; i < n; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + (i * 4U),
                              (uint64_t)src[i]) != HAL_OK)
        {
            rc = 1;
            break;
        }
    }

    HAL_FLASH_Lock();
    return rc;
}

/* ==================================================================
 *  四、扫描：找出最新一条有效记录
 * ================================================================== */
static void ee_scan(void)
{
    uint8_t  bank;
    uint16_t slot;
    uint8_t  best_ok = 0;
    uint8_t  best_bank = 0;
    uint16_t best_slot = 0;
    uint32_t best_seq = 0;
    uint16_t best_plen = 0;

    s_ee.crc_fail = 0;
    s_ee.bad_hdr  = 0;

    for (bank = 0; bank < 2U; bank++)
    {
        for (slot = 0; slot < EE_SLOT_COUNT; slot++)
        {
            const volatile ee_hdr_t *vh = (const volatile ee_hdr_t *)ee_slot_addr(bank, slot);
            ee_hdr_t h;

            h.magic = vh->magic;
            if (h.magic != EE_MAGIC)
            {
                /* 绝大多数槽都是擦除态(0xFFFFFFFF)，这里直接跳过，扫描很快 */
                continue;
            }

            h.ver  = vh->ver;
            h.plen = vh->plen;
            h.seq  = vh->seq;
            h.crc  = vh->crc;

            if ((h.ver != EE_VER) || (h.plen == 0U) ||
                (h.plen > EE_PAYLOAD_MAX) || ((h.plen & 3U) != 0U))
            {
                s_ee.bad_hdr++;
                continue;
            }

            if (ee_crc_record(&h, (const volatile uint8_t *)ee_slot_addr(bank, slot) + EE_HDR_SIZE) != h.crc)
            {
                s_ee.crc_fail++;
                continue;
            }

            if ((best_ok == 0U) || ee_seq_newer(h.seq, best_seq))
            {
                best_ok   = 1U;
                best_bank = bank;
                best_slot = slot;
                best_seq  = h.seq;
                best_plen = h.plen;
            }
        }
    }

    if (best_ok != 0U)
    {
        s_ee.have       = 1U;
        s_ee.bank       = best_bank;
        s_ee.next_slot  = (uint16_t)(best_slot + 1U);
        s_ee.used_slots = (uint16_t)(best_slot + 1U);
        s_ee.seq        = best_seq;
        s_ee.plen       = best_plen;
    }
    else
    {
        s_ee.have       = 0U;
        s_ee.bank       = 0U;
        s_ee.next_slot  = 0U;
        s_ee.used_slots = 0U;
        s_ee.seq        = 0U;
        s_ee.plen       = 0U;
    }
}

/* 从 from 起找当前 bank 里第一个空槽；找不到返回 -1（bank 已满） */
static int16_t ee_find_free_slot(uint8_t bank, uint16_t from)
{
    uint16_t slot;

    for (slot = from; slot < EE_SLOT_COUNT; slot++)
    {
        if (ee_slot_first_word(bank, slot) == 0xFFFFFFFFUL)
        {
            return (int16_t)slot;
        }
    }
    return -1;
}

/* ==================================================================
 *  五、对外 API
 * ================================================================== */
uint8_t EE_Init(void)
{
    memset(&s_ee, 0, sizeof(s_ee));

    /* 时间测量用（DWT 计数器）。这里最早初始化一次，后面测擦/写耗时才有数 */
    DWT_Init();

    s_ee.image_end = (uint32_t)(uintptr_t)&Image$$ER_IROM1$$Limit;

    if (s_ee.image_end > EE_BANK0_BASE)
    {
        /* 镜像已经长进保留区：绝对不能再擦写，否则会擦掉自己的代码。
         * 典型场景：CubeMX 重新生成工程把 IROM 从 0xC0000 改回了 0x100000。 */
        s_ee.img_overlap = 1U;
        s_ee.last_err    = (uint8_t)EE_ERR_IMAGE;
        s_ee.ready       = 0U;
        return 0U;
    }

    ee_scan();

    s_ee.ready = 1U;
    return 1U;
}

uint8_t EE_Save(const uint8_t *payload, uint16_t len)
{
    ee_hdr_t *h;
    int16_t   slot;
    uint8_t   bank;
    uint32_t  seq;
    uint32_t  t0;
    uint16_t  words;

    if (s_ee.ready == 0U)
    {
        return (s_ee.img_overlap != 0U) ? (uint8_t)EE_ERR_IMAGE : (uint8_t)EE_ERR_STATE;
    }
    if ((payload == NULL) || (len == 0U) || (len > EE_PAYLOAD_MAX) || ((len & 3U) != 0U))
    {
        return (uint8_t)EE_ERR_ARG;
    }
    if (BlackBox_IsArmed() != 0U)
    {
        return (uint8_t)EE_ERR_ARMED;
    }

    /* 内容没变就不写：省擦写寿命，也避免 seq 无意义地涨 */
    if ((s_ee.have != 0U) && (len == s_ee.plen))
    {
        const volatile uint8_t *old = (const volatile uint8_t *)ee_slot_addr(s_ee.bank, (uint16_t)(s_ee.next_slot - 1U)) + EE_HDR_SIZE;
        uint16_t i;
        uint8_t  same = 1U;

        for (i = 0; i < len; i++)
        {
            if (old[i] != payload[i])
            {
                same = 0U;
                break;
            }
        }
        if (same != 0U)
        {
            return (uint8_t)EE_OK_UNCHANGED;
        }
    }

    bank = s_ee.bank;
    slot = ee_find_free_slot(bank, s_ee.next_slot);

    if (slot < 0)
    {
        /* 当前 bank 已满 → 擦另一个 bank，从它的 0 号槽重新开始。
         * 注意擦的是"另一块"，此刻最新记录还在当前 bank 里，掉电也没事。 */
        uint8_t other = (uint8_t)(bank ^ 1U);

        if (ee_erase_bank(other) != 0U)
        {
            return ee_fatal((uint8_t)EE_ERR_FLASH);
        }
        bank = other;
        slot = 0;
    }

    seq = s_ee.seq + 1U;
    if (seq == 0xFFFFFFFFUL)
    {
        seq = 1U;                       /* 0xFFFFFFFF 是擦除态，跳过 */
    }

    memset(s_rec_w, 0xFF, sizeof(s_rec_w));

    h = (ee_hdr_t *)S_REC_BYTE;
    h->magic = EE_MAGIC;
    h->ver   = (uint16_t)EE_VER;
    h->plen  = len;
    h->seq   = seq;
    h->crc   = 0U;
    memcpy(&S_REC_BYTE[EE_HDR_SIZE], payload, len);
    h->crc   = ee_crc_record(h, (const volatile uint8_t *)&S_REC_BYTE[EE_HDR_SIZE]);

    words = (uint16_t)((EE_HDR_SIZE + (uint32_t)len) / 4U);

    t0 = DWT_GetUS();
    if (ee_program_words(ee_slot_addr(bank, (uint16_t)slot), s_rec_w, words) != 0U)
    {
        return ee_fatal((uint8_t)EE_ERR_FLASH);
    }
    s_ee.wr_us = DWT_GetUS() - t0;

    /* 回读校验：不信任"写成功"，只信任"读回来对得上" */
    {
        const volatile ee_hdr_t *vp = (const volatile ee_hdr_t *)ee_slot_addr(bank, (uint16_t)slot);
        ee_hdr_t vh;

        vh.magic = vp->magic;
        vh.ver   = vp->ver;
        vh.plen  = vp->plen;
        vh.seq   = vp->seq;
        vh.crc   = vp->crc;

        if ((vh.magic != EE_MAGIC) || (vh.seq != seq) || (vh.plen != len) ||
            (ee_crc_record(&vh, (const volatile uint8_t *)vp + EE_HDR_SIZE) != h->crc))
        {
            return ee_fatal((uint8_t)EE_ERR_VERIFY);
        }
    }

    s_ee.bank       = bank;
    s_ee.next_slot  = (uint16_t)(slot + 1);
    s_ee.used_slots = (uint16_t)(slot + 1);
    s_ee.seq        = seq;
    s_ee.plen       = len;
    s_ee.have       = 1U;
    s_ee.saves++;
    s_ee.last_err   = (uint8_t)EE_OK;

    return (uint8_t)EE_OK;
}

uint8_t EE_Read(uint8_t *buf, uint16_t buf_size, uint16_t *out_len)
{
    const volatile uint8_t *src;
    uint16_t i;

    if (buf == NULL)
    {
        return (uint8_t)EE_ERR_ARG;
    }
    if (s_ee.have == 0U)
    {
        return (uint8_t)EE_ERR_NOREC;
    }
    if (s_ee.plen > buf_size)
    {
        return (uint8_t)EE_ERR_ARG;
    }

    src = (const volatile uint8_t *)ee_slot_addr(s_ee.bank, (uint16_t)(s_ee.next_slot - 1U)) + EE_HDR_SIZE;
    for (i = 0; i < s_ee.plen; i++)
    {
        buf[i] = src[i];
    }
    if (out_len != NULL)
    {
        *out_len = s_ee.plen;
    }
    return (uint8_t)EE_OK;
}

uint8_t EE_EraseAll(void)
{
    if (s_ee.ready == 0U)
    {
        return (s_ee.img_overlap != 0U) ? (uint8_t)EE_ERR_IMAGE : (uint8_t)EE_ERR_STATE;
    }
    if (BlackBox_IsArmed() != 0U)
    {
        return (uint8_t)EE_ERR_ARMED;
    }

    if ((ee_erase_bank(0U) != 0U) || (ee_erase_bank(1U) != 0U))
    {
        return ee_fatal((uint8_t)EE_ERR_FLASH);
    }

    s_ee.have       = 0U;
    s_ee.bank       = 0U;
    s_ee.next_slot  = 0U;
    s_ee.used_slots = 0U;
    s_ee.seq        = 0U;
    s_ee.plen       = 0U;
    s_ee.last_err   = (uint8_t)EE_OK;

    return (uint8_t)EE_OK;
}

void EE_GetStatus(FlashEE_Status_t *st)
{
    if (st != NULL)
    {
        *st = s_ee;
    }
}

const char *EE_ResultStr(uint8_t r)
{
    switch (r)
    {
        case EE_OK:            return "ok";
        case EE_OK_UNCHANGED:  return "unchanged";
        case EE_ERR_ARG:       return "arg";
        case EE_ERR_NOREC:     return "norec";
        case EE_ERR_ARMED:     return "armed";
        case EE_ERR_IMAGE:     return "img_overlap";
        case EE_ERR_STATE:     return "state";
        case EE_ERR_FLASH:     return "flash";
        case EE_ERR_VERIFY:    return "verify";
        default:               return "?";
    }
}

/* ==================================================================
 *  六、历史记录读取（供参数回滚用）
 * ================================================================== */

/* 当前可回滚的记录数。等于活动 bank 里已用的槽数（最多 128）：
 * 换 bank 时另一块会被擦掉，所以历史深度会跟着"回零重来"。 */
uint16_t EE_HistoryCount(void)
{
    return (s_ee.have != 0U) ? s_ee.used_slots : (uint16_t)0U;
}

/* 读第 idx 条历史记录（idx = 0 是最新一条）。
 *
 * 为什么不直接算 slot = next_slot - 1 - idx：
 *   写入过程中掉过电会留下"头写了、载荷没写完"的半条记录，
 *   那些槽必须跳过，否则回滚会拿到一条 CRC 不过的垃圾。
 *   所以这里从最新往回逐槽校验，只数真正有效的记录。
 * 最多扫 128 个槽，开销可以忽略（只在地面命令里调）。 */
uint8_t EE_ReadHistory(uint16_t idx, uint8_t *buf, uint16_t buf_size,
                       uint16_t *len, uint32_t *seq, uint16_t *slot)
{
    uint16_t i;
    uint16_t count = 0;

    if ((buf == NULL) || (s_ee.have == 0U))
    {
        return (uint8_t)EE_ERR_NOREC;
    }

    for (i = s_ee.next_slot; i > 0U; i--)
    {
        uint16_t                 s  = (uint16_t)(i - 1U);
        const volatile ee_hdr_t *vp = (const volatile ee_hdr_t *)ee_slot_addr(s_ee.bank, s);
        const volatile uint8_t  *src = (const volatile uint8_t *)vp + EE_HDR_SIZE;
        ee_hdr_t h;
        uint16_t k;

        h.magic = vp->magic;
        if (h.magic != EE_MAGIC)
        {
            continue;
        }
        h.ver  = vp->ver;
        h.plen = vp->plen;
        h.seq  = vp->seq;
        h.crc  = vp->crc;

        if ((h.ver != EE_VER) || (h.plen == 0U) ||
            (h.plen > EE_PAYLOAD_MAX) || ((h.plen & 3U) != 0U))
        {
            continue;
        }
        if (ee_crc_record(&h, src) != h.crc)
        {
            continue;
        }

        if (count == idx)
        {
            if (h.plen > buf_size)
            {
                return (uint8_t)EE_ERR_ARG;
            }
            for (k = 0; k < h.plen; k++)
            {
                buf[k] = src[k];
            }
            if (len  != NULL) { *len  = h.plen; }
            if (seq  != NULL) { *seq  = h.seq;  }
            if (slot != NULL) { *slot = s;      }
            return (uint8_t)EE_OK;
        }
        count++;
    }

    return (uint8_t)EE_ERR_NOREC;
}

/* ==================================================================
 *  七、控制台（只保留"原始记录"相关的命令）
 *
 *  参数相关的 CFG:INFO / LIST / GET / SET / SAVE / LOAD / DEF / HIST /
 *  ROLLBACK / REBOOT 都在 param.c 里；这里只剩 RAW 和 ERASE。
 * ================================================================== */
static void ee_cmd_raw(void)
{
    uint32_t addr;
    uint32_t i;
    uint32_t n;
    uint32_t plen;

    if (s_ee.have == 0U)
    {
        ee_send("CFG:ERR norec\r\n");
        return;
    }

    addr = ee_slot_addr(s_ee.bank, (uint16_t)(s_ee.next_slot - 1U));
    plen = s_ee.plen;
    n    = EE_HDR_SIZE + plen;

    ee_printf("CFG:RAW addr=%08lX len=%lu\r\n", (unsigned long)addr, (unsigned long)n);

    for (i = 0; i < n; i += 16U)
    {
        char     line[80];
        int      p;
        uint32_t k;

        p = sprintf(line, "%08lX:", (unsigned long)(addr + i));
        for (k = 0; (k < 16U) && ((i + k) < n); k++)
        {
            p += sprintf(&line[p], " %02X", (unsigned)((const volatile uint8_t *)addr)[i + k]);
        }
        (void)sprintf(&line[p], "\r\n");
        ee_send(line);
    }
}

static void ee_cmd_erase(void)
{
    uint8_t r = EE_EraseAll();

    if (r == (uint8_t)EE_OK)
    {
        ee_send("CFG:ERASED (2 banks)\r\n");
    }
    else
    {
        ee_printf("CFG:ERR erase %s\r\n", EE_ResultStr(r));
    }
}

uint8_t EE_Console(const char *cmd)
{
    if (cmd == NULL)
    {
        return 0U;
    }
    if (strcmp(cmd, "CFG:RAW") == 0)
    {
        ee_cmd_raw();
        return 1U;
    }
    if (strcmp(cmd, "CFG:ERASE") == 0)
    {
        ee_cmd_erase();
        return 1U;
    }
    return 0U;
}
