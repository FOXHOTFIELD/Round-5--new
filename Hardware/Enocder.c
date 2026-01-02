// 编码器(旋转式)读取实现 - 支持两路：
// ENC1：PA0(A通道)/PA1(B通道) -> TIM2
// ENC2：PB6(A通道)/PB7(B通道) -> TIM4
// 基于 TIMx 编码器接口模式（硬件正交解码），替代原 EXTI 查表方案
// 说明：
// - TIMx 作为正交解码计数器，硬件完成边沿采样与方向判断，CPU 占用更低、抗抖更好。
// - 由于 STM32F103C8 的 TIM2/TIM4 计数器为 16 位，驱动通过更新中断做 32 位软件扩展。

#include "Encoder.h"

#define ENC_MAX_RAW_COUNT (740)

typedef struct
{
	GPIO_TypeDef *gpio;
	uint32_t gpio_rcc;
	uint16_t pin_a;
	uint16_t pin_b;
	TIM_TypeDef *tim;
	uint32_t tim_rcc;
	IRQn_Type tim_irq;
} EncoderHw;

static const EncoderHw kEncoderHw[ENCODER_COUNT] =
{
	{ GPIOA, RCC_APB2Periph_GPIOA, GPIO_Pin_0, GPIO_Pin_1, TIM2, RCC_APB1Periph_TIM2, TIM2_IRQn },
	{ GPIOB, RCC_APB2Periph_GPIOB, GPIO_Pin_7, GPIO_Pin_6, TIM4, RCC_APB1Periph_TIM4, TIM4_IRQn },
};

/* 速度低通滤波器状态（用于 butterworth2_init/process）*/
static FilterBiquad speed_lp[ENCODER_COUNT];

static uint8_t speed_lp_ready[ENCODER_COUNT] = {0}; /* 延迟到初始化完成再启用滤波 */

// 32 位扩展：以 16 位 CNT 为低 16 位，溢出/下溢在更新中断中累加高位
static volatile int32_t s_ext_count_high[ENCODER_COUNT] = {0}; // 以 65536 为步长的高位累计

static const EncoderHw *Encoder_GetHw(EncoderId_t id)
{
	if ((uint32_t)id >= ENCODER_COUNT) return NULL;
	return &kEncoderHw[id];
}

static inline float Encoder_FilterPercent(EncoderId_t id, float val)
{
	return speed_lp_ready[id] ? butterworth2_process(&speed_lp[id], val) : val;
}

// 初始化：开启端口时钟，配置引脚为输入，上拉/悬空按传感器而定；配置 TIMx 为编码器模式
void Encoder_Init(EncoderId_t id)
{
	const EncoderHw *hw = Encoder_GetHw(id);
	if (!hw) return;

	// 时钟：GPIOx、AFIO、TIMx
	RCC_APB2PeriphClockCmd(hw->gpio_rcc | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(hw->tim_rcc, ENABLE);

	// GPIO：A/B 输入；若传感器开漏输出建议上拉输入
	GPIO_InitTypeDef gpio;
	gpio.GPIO_Pin   = hw->pin_a | hw->pin_b;
	gpio.GPIO_Mode  = GPIO_Mode_IPU; // 上拉输入（可按需改为 GPIO_Mode_IN_FLOATING）
	gpio.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(hw->gpio, &gpio);

	// 定时器基础配置
	TIM_TimeBaseInitTypeDef tb;
	TIM_TimeBaseStructInit(&tb);
	tb.TIM_Prescaler         = 0;          // 不分频，捕获全部边沿
	tb.TIM_Period            = 0xFFFF;     // 16 位计数器最大值
	tb.TIM_ClockDivision     = TIM_CKD_DIV1;
	tb.TIM_CounterMode       = TIM_CounterMode_Up; // 编码器模式下方向由硬件控制
	TIM_TimeBaseInit(hw->tim, &tb);

	// 编码器接口：TI1+TI2，双通道，X4 计数；上升沿为例（根据接线可改）
	TIM_EncoderInterfaceConfig(hw->tim,
		TIM_EncoderMode_TI12,
		TIM_ICPolarity_Rising,
		TIM_ICPolarity_Rising);

	// 输入滤波：ICFilter 0..15，数值越大滤波越强，响应越慢；经验 6~10
	TIM_ICInitTypeDef ic;
	TIM_ICStructInit(&ic);
	ic.TIM_ICFilter = 0;
	ic.TIM_Channel  = TIM_Channel_1; TIM_ICInit(hw->tim, &ic);
	ic.TIM_Channel  = TIM_Channel_2; TIM_ICInit(hw->tim, &ic);

	// 清计数与高位扩展
	TIM_SetCounter(hw->tim, 0);
	s_ext_count_high[id] = 0;

	// 使能更新中断用于 32 位扩展，并配置 NVIC
	TIM_ClearITPendingBit(hw->tim, TIM_IT_Update);
	TIM_ITConfig(hw->tim, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef nvic;
	nvic.NVIC_IRQChannel = hw->tim_irq;
	nvic.NVIC_IRQChannelPreemptionPriority = 2;
	nvic.NVIC_IRQChannelSubPriority        = 2;
	nvic.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&nvic);

	// 启动计数
	TIM_Cmd(hw->tim, ENABLE);

	butterworth2_init(&speed_lp[id], 10.0f, 1000.0f);
	speed_lp_ready[id] = 1;

	/* 创建编码器后台任务，周期性读取并更新共享值 */
	//(void)xTaskCreate(vEncoderTask, "Encoder", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
}

// 原子读 32 位计数：禁用对应 TIM 中断，组合高位与 CNT 后恢复
int32_t Encoder_GetCount(EncoderId_t id)
{
	const EncoderHw *hw = Encoder_GetHw(id);
	if (!hw) return 0;

	int32_t high;
	uint16_t low;
	NVIC_DisableIRQ(hw->tim_irq);
	high = s_ext_count_high[id];
	low  = (uint16_t)TIM_GetCounter(hw->tim);
	NVIC_EnableIRQ(hw->tim_irq);
	return -((high << 16) | low);
}

// 设置/重置累计计数（例如清零）
void Encoder_SetCount(EncoderId_t id, int32_t count)
{
	const EncoderHw *hw = Encoder_GetHw(id);
	if (!hw) return;

	NVIC_DisableIRQ(hw->tim_irq);
	s_ext_count_high[id] = (count >> 16);
	TIM_SetCounter(hw->tim, (uint16_t)(count & 0xFFFF));
	NVIC_EnableIRQ(hw->tim_irq);
}

// 返回当前累计计数并清零，便于按固定周期统计转速
int32_t Encoder_GetAndReset(EncoderId_t id)
{
	int32_t val = Encoder_GetCount(id);
	Encoder_SetCount(id, 0);
	return val;
}

// -------------------- 驱动内的 TIMx 更新中断处理入口（供向量表转发调用） --------------------
static void Encoder_IRQHandler(EncoderId_t id)
{
	const EncoderHw *hw = Encoder_GetHw(id);
	if (!hw) return;

	if (TIM_GetITStatus(hw->tim, TIM_IT_Update) != RESET)
	{
		// 根据当前计数方向判定是上溢(+65536)还是下溢(-65536)
		if ((hw->tim->CR1 & TIM_CR1_DIR) == 0)
		{
			// 向上计数，上溢
			s_ext_count_high[id] += 1;
		}
		else
		{
			// 向下计数，下溢
			s_ext_count_high[id] -= 1;
		}
		TIM_ClearITPendingBit(hw->tim, TIM_IT_Update);
	}
}

void Encoder_TIM2_IRQHandler(void)
{
	Encoder_IRQHandler(ENCODER_A);
}

void Encoder_TIM4_IRQHandler(void)
{
	Encoder_IRQHandler(ENCODER_B);
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
float Encoder_GetCountPercent(EncoderId_t id)
{
	int32_t cnt = Encoder_GetCount(id);
	float pct = Encoder_RawToPercent(cnt);
	return Encoder_FilterPercent(id, pct);
}

// 读取并清零累计计数，返回映射后的百分比
float Encoder_GetAndResetPercent(EncoderId_t id)
{
	int32_t cnt = Encoder_GetAndReset(id);
	float pct = Encoder_RawToPercent(cnt);
	return Encoder_FilterPercent(id, pct);
}

// 轮询接口保留以兼容旧代码（TIM 编码器模式无需轮询）
void Encoder_Update(EncoderId_t id)
{
	(void)id;
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
//		Comm_encoder_a = pct;
//		vTaskDelay(pdMS_TO_TICKS(100));
//	}
//}
