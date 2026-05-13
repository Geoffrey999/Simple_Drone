#include "dwt.h"

uint32_t previous_count=0;
uint32_t current_count=0;
uint32_t sum_period=0;

static volatile uint32_t overflow_count = 0;
static volatile uint32_t last_cyccnt = 0;
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t get_DWT_count(void)
{
	
	uint32_t period=0;
	current_count=DWT->CYCCNT;
	period = current_count-previous_count;
	sum_period+=period;
	previous_count=current_count;
	
	return sum_period;
}

// 获取扩展的64位计数器值
uint64_t DWT_GetCycles64(void)
{
    uint32_t current_cyccnt = DWT->CYCCNT;
    
    // 检查是否发生了溢出
    if (current_cyccnt < last_cyccnt) {
        // 溢出发生
        overflow_count++;
    }
    
    last_cyccnt = current_cyccnt;
    
    // 组合高位和低位得到64位值
    return ((uint64_t)overflow_count << 32) | current_cyccnt;
}

// 获取毫秒时间
uint32_t DWT_GetMS(void)
{
    uint64_t cycles = DWT_GetCycles64();
    return (uint32_t)(cycles / (SystemCoreClock / 1000));
}

// 获取微秒时间
uint32_t DWT_GetUS(void)
{
    uint64_t cycles = DWT_GetCycles64();
    return (uint32_t)(cycles / (SystemCoreClock / 1000000));
}
