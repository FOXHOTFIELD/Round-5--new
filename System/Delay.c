#include "stm32f10x.h"

/*
 * 使用 DWT->CYCCNT 实现微秒级延时，避免修改 SysTick（FreeRTOS 使用它来驱动时钟）。
 * 若 DWT 不可用则回退到保留/恢复 SysTick 的实现。
 */
static volatile uint8_t dwt_enabled = 0;

static void DWT_Delay_Init(void)
{
	/* 如果已经初始化过，直接返回 */
	if (dwt_enabled) return;

	/* 使能 Trace and Debug block */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

	/* 清零并启用 CYCCNT */
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* 检查是否成功启用 */
	if (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) {
		dwt_enabled = 1;
	} else {
		dwt_enabled = 0;
	}
}

void Delay_us(uint32_t xus)
{
	/* 尝试初始化 DWT */
	DWT_Delay_Init();

	if (dwt_enabled && xus != 0) {
		/* 计算所需周期数（防止乘法溢出） */
		uint32_t cycles = (SystemCoreClock / 1000000UL) * xus;
		uint32_t start = DWT->CYCCNT;

		/* 等待直到 CYCCNT 增加到目标差值（处理溢出通过无符号差值运算） */
		while ((DWT->CYCCNT - start) < cycles) {
			__NOP();
		}
	} else {
		/* 回退：短时间内保存并使用 SysTick（尽量短） */
		uint32_t prev_ctrl = SysTick->CTRL;
		uint32_t prev_load = SysTick->LOAD;
		uint32_t prev_val = SysTick->VAL;

		/* 注意：这会短暂修改 SysTick，勿用于长时间或频繁调用 */
		uint64_t load = (uint64_t)SystemCoreClock * xus / 1000000UL;
		if (load > 0x00FFFFFFU) load = 0x00FFFFFFU;
		SysTick->LOAD = (uint32_t)load;
		SysTick->VAL = 0U;
		/* 使用处理器时钟作为时基，不使能中断 */
		SysTick->CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_CLKSOURCE_Msk;
		while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)) {
			__NOP();
		}

		SysTick->CTRL = prev_ctrl;
		SysTick->LOAD = prev_load;
		SysTick->VAL = prev_val;
	}
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 

