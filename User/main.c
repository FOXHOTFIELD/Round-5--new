#include "myHeader.h"

//#define TEST

#ifndef TEST
#define LEDtest
/* 从 Menu.c 导出菜单状态（在 Menu.c 中定义） */
extern MenuStatus_t Menu_Status;

#ifdef LEDtest
/* LED 监控任务：当 Menu_Status.Cursor_Index == 1 时点亮 PC13，否则熄灭 */
static void vLedMonitorTask(void *pvParameters)
{
	(void)pvParameters;
    extern uint8_t Menu_Flag;
	for (;;)
	{
		/* PC13 在多数板子上为低电平点亮，故当 Cursor_Index==1 时输出低电平（ResetBits） */
		if (Menu_Flag == 1)
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_13); /* 点亮 */
            vTaskDelay(pdMS_TO_TICKS(100));
            GPIO_SetBits(GPIOC, GPIO_Pin_13);
            Menu_Flag = 0;
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}
#endif

int main(void)
{

	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);            //配置NVIC为分组4

	/* 系统初始化（startup/系统初始化通常已在启动代码中完成） */
	SystemInit();

	/* 初始化 */
	Menu_Init();
	OLED_Init();
	Serial_Init();
	Encoder_Init(ENCODER_A);
	Encoder_Init(ENCODER_B);
	MPU6050_Init();
    MD_Init();
	Comm_Init();
    PID_Init();
    Motor_SetStandbyState(ENABLE);

    #ifdef LEDtest
        /* 使能 GPIOC 时钟并配置 PC13 为推挽输出 */
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
        GPIO_InitTypeDef GPIO_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_Init(GPIOC, &GPIO_InitStructure);
        /* 初始为熄灭（多数开发板 PC13 为低电平点亮） */
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
        /* 创建 LED 监控任务 */
        xTaskCreate(vLedMonitorTask, "LED", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
    #endif

	/* 启动调度 */
	vTaskStartScheduler();

	/* 若调度器返回，进入死循环 */
	while (1)
	{
	}
}

#endif

#ifdef TEST

int main(void)
{
	/* 系统和外设初始化 */
	OLED_Init();
    
    OLED_Clear();
	OLED_ShowChar(0, 0, '1', OLED_6X8);
	OLED_Update();
    

    Serial_Init();
	Encoder_Init(ENCODER_A);
	Encoder_Init(ENCODER_B);
    MD_Init();

    Motor_SetStandbyState(ENABLE);
	/* 使用更高占空比，以 ARR=10000 的尺度为参考 */

    uint8_t flag = 0;

/* 使能 GPIOC 时钟并配置 PC13 为推挽输出 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* 初始为复位（视板子 LED 极性而定） */
	GPIO_ResetBits(GPIOC, GPIO_Pin_13);

	extern uint8_t Serial_RxBuffer[100];
	extern uint16_t Serial_RxLength;
// 使用 Serial_RxBuffer 和 Serial_RxLength
	{
		float speedPercent = 0.0f;
		const float rampDurationMs = 10000.0f; /* 10s */
		const uint32_t loopDelayMs = 20u; /* 与原来延时一致 */
		const uint32_t steps = (uint32_t)(rampDurationMs / loopDelayMs);
		const float stepInc = 100.0f / (float)steps;

		/* 响应优先：在低采样率下使用截止 10Hz 的二阶巴特沃斯低通滤波器
		 * 采样率 fs = 1000 / loopDelayMs = 50 Hz
		 */
		static FilterBiquad speed_lp;
		butterworth2_init(&speed_lp, 10.0f, 50.0f);
		while (1)
		{
			if (speedPercent < 100.0f)
			{
				Motor_SetSpeedPercent(1, speedPercent);
				speedPercent += stepInc;
				if (speedPercent > 100.0f)
					speedPercent = 100.0f;
			}
			else
			{
				Motor_SetSpeedPercent(1, 100.0f);
			}

			float raw = -Encoder_GetAndResetPercent(ENCODER_A);
			float pst = butterworth2_process(&speed_lp, raw);

			OLED_ShowSignedNum(10, 10, pst, 6, OLED_6X8);
			OLED_UpdateArea(10, 10, 128, 8);
			Serial_SendJustFloat(&pst, 1);

			Delay_ms(loopDelayMs);
		}
	}
}

#endif
