// 编码器(旋转式)读取实现 - 使用 PA0(A通道) 与 PA1(B通道)
// 基于 TIM2 编码器接口模式（硬件正交解码），替代原 EXTI 查表方案
// 说明：
// - TIM2 作为正交解码计数器，硬件完成边沿采样与方向判断，CPU 占用更低、抗抖更好。
// - 由于 STM32F103C8 的 TIM2 计数器为 16 位，驱动通过更新中断做 32 位软件扩展。

#include "Encoder.h"

#define ENC_MAX_RAW_COUNT (740)

/* 速度低通滤波器状态（用于 butterworth2_init/process）*/
static FilterBiquad speed_lp;

static uint8_t speed_lp_ready = 0; /* 延迟到初始化完成再启用滤波 */

static inline float Encoder_FilterPercent(float val)
{
	return speed_lp_ready ? butterworth2_process(&speed_lp, val) : val;

}
// 32 位扩展：以 16 位 CNT 为低 16 位，溢出/下溢在更新中断中累加高位
static volatile int32_t s_ext_count_high = 0; // 以 65536 为步长的高位累计


// 初始化：开启端口时钟，配置 PA0/PA1 为输入，上拉/悬空按传感器而定；配置 TIM2 为编码器模式
void Encoder_Init(void)
{
	// 时钟：GPIOA、AFIO、TIM2
	RCC_APB2PeriphClockCmd(ENC_GPIO_RCC | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(ENC_TIM_RCC, ENABLE);

	// GPIO：PA0/PA1 输入；若传感器开漏输出建议上拉输入
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin   = ENC_CH_A | ENC_CH_B;
	gpio.GPIO_Mode  = GPIO_Mode_IPU; // 上拉输入（可按需改为 GPIO_Mode_IN_FLOATING）
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(ENC_GPIO, &gpio);

	// 定时器基础配置
	TIM_TimeBaseInitTypeDef tb;
	TIM_TimeBaseStructInit(&tb);
	tb.TIM_Prescaler         = 0;          // 不分频，捕获全部边沿
	tb.TIM_Period            = 0xFFFF;     // 16 位计数器最大值
	tb.TIM_ClockDivision     = TIM_CKD_DIV1;
	tb.TIM_CounterMode       = TIM_CounterMode_Up; // 编码器模式下方向由硬件控制
	TIM_TimeBaseInit(ENC_TIM, &tb);

	// 编码器接口：TI1+TI2，双通道，X4 计数；上升沿为例（根据接线可改）
	TIM_EncoderInterfaceConfig(ENC_TIM,
		TIM_EncoderMode_TI12,
		TIM_ICPolarity_Rising,
		TIM_ICPolarity_Rising);

	// 输入滤波：ICFilter 0..15，数值越大滤波越强，响应越慢；经验 6~10
	TIM_ICInitTypeDef ic;
	TIM_ICStructInit(&ic);
	ic.TIM_ICFilter = 10;
	ic.TIM_Channel  = TIM_Channel_1; TIM_ICInit(ENC_TIM, &ic);
	ic.TIM_Channel  = TIM_Channel_2; TIM_ICInit(ENC_TIM, &ic);

	// 清计数与高位扩展
	TIM_SetCounter(ENC_TIM, 0);
	s_ext_count_high = 0;

	// 使能更新中断用于 32 位扩展，并配置 NVIC
	TIM_ClearITPendingBit(ENC_TIM, TIM_IT_Update);
	TIM_ITConfig(ENC_TIM, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = TIM2_IRQn;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority        = 2;
	nvic.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&nvic);

	// 启动计数
	TIM_Cmd(ENC_TIM, ENABLE);

	butterworth2_init(&speed_lp, 10.0f, 1000.0f);
	speed_lp_ready = 1;

	/* 创建编码器后台任务，周期性读取并更新共享值 */
	//(void)xTaskCreate(vEncoderTask, "Encoder", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
}

// 原子读 32 位计数：禁用 TIM2 中断，组合高位与 CNT 后恢复
int32_t Encoder_GetCount(void)
{
	int32_t high;
	uint16_t low;
	NVIC_DisableIRQ(TIM2_IRQn);
	high = s_ext_count_high;
	low  = (uint16_t)TIM_GetCounter(ENC_TIM);
	NVIC_EnableIRQ(TIM2_IRQn);
	return -((high << 16) | low);
}

// 设置/重置累计计数（例如清零）
void Encoder_SetCount(int32_t count)
{
	NVIC_DisableIRQ(TIM2_IRQn);
	s_ext_count_high = (count >> 16);
	TIM_SetCounter(ENC_TIM, (uint16_t)(count & 0xFFFF));
	NVIC_EnableIRQ(TIM2_IRQn);
}

// 返回当前累计计数并清零，便于按固定周期统计转速
int32_t Encoder_GetAndReset(void)
{
	int32_t val = Encoder_GetCount();
	Encoder_SetCount(0);
	return val;
}

// -------------------- 驱动内的 TIM2 更新中断处理入口（供向量表转发调用） --------------------
void Encoder_TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(ENC_TIM, TIM_IT_Update) != RESET)
	{
		// 根据当前计数方向判定是上溢(+65536)还是下溢(-65536)
		if ((ENC_TIM->CR1 & TIM_CR1_DIR) == 0)
		{
			// 向上计数，上溢
			s_ext_count_high += 1;
		}
		else
		{
			// 向下计数，下溢
			s_ext_count_high -= 1;
		}
		TIM_ClearITPendingBit(ENC_TIM, TIM_IT_Update);
	}
}

// 将原始计数（脉冲数/周期）映射为百分比 -100.00 .. 100.00
static float Encoder_RawToPercent(int32_t raw)
{
	if (raw > ENC_MAX_RAW_COUNT) raw = ENC_MAX_RAW_COUNT;
	if (raw < -ENC_MAX_RAW_COUNT) raw = -ENC_MAX_RAW_COUNT;

	// 使用整数计算保留两位小数：p100 = round(raw * 10000 / ENC_MAX_RAW_COUNT)
	int64_t numer = (int64_t)raw * 10000LL;
	int64_t adj = (raw >= 0) ? (ENC_MAX_RAW_COUNT / 2) : -(ENC_MAX_RAW_COUNT / 2);
	int32_t p100 = (int32_t)((numer + adj) / ENC_MAX_RAW_COUNT);

	return (float)p100 / 100.0f;
}

// 读取当前累计计数并返回映射的百分比（不清零）
float Encoder_GetCountPercent(void)
{
	int32_t cnt = Encoder_GetCount();
	float pct = Encoder_RawToPercent(cnt);
	return Encoder_FilterPercent(pct);
}

// 读取并清零累计计数，返回映射后的百分比
float Encoder_GetAndResetPercent(void)
{
	int32_t cnt = Encoder_GetAndReset();
	float pct = Encoder_RawToPercent(cnt);
	return Encoder_FilterPercent(pct);
}

//static void vEncoderTask(void *pv)
//{
//    (void) pv;
//
//	for(;;)
//	{
//		//float pct = Encoder_GetAndResetPercent();
//		float pct = Encoder_GetAndResetPercent();
//		/* 写入共享 encoder 值，vCommTask 会合并发送 */
//		Comm_encoder = pct;
//		vTaskDelay(pdMS_TO_TICKS(100));
//	}
//}
