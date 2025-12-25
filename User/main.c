#include "myHeader.h"

extern void Delay_ms(uint32_t xms);

#define TEST

#ifndef TEST

/* 从 Menu.c 导出菜单状态（在 Menu.c 中定义） */
extern MenuStatus Menu_Status;

/* LED 监控任务：当 Menu_Status.Cursor_Index == 1 时点亮 PC13，否则熄灭 */
static void vLedMonitorTask(void *pvParameters)
{
	(void)pvParameters;
    extern uint8_t Flag;
	for (;;)
	{
		/* PC13 在多数板子上为低电平点亮，故当 Cursor_Index==1 时输出低电平（ResetBits） */
		if (Flag == 1)
		{
			GPIO_ResetBits(GPIOC, GPIO_Pin_13); /* 点亮 */
		}
		else
		{
			GPIO_SetBits(GPIOC, GPIO_Pin_13); /* 熄灭 */
		}
		vTaskDelay(pdMS_TO_TICKS(100));
	}
}

int main(void)
{

	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);            //配置NVIC为分组4

	/* 系统初始化（startup/系统初始化通常已在启动代码中完成） */
	SystemInit();

	/* 初始化 */
	OLED_Init();
	OLED_ShowChar(0, 0, '1', OLED_6X8);
	OLED_Update();
	Menu_Init();

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
	while (1)
	{
	    
	    if (Serial_GetRxFlag()) {
            OLED_ShowString(0, 20, (char*)Serial_RxBuffer, OLED_6X8);
            OLED_Update();

            if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13) == Bit_RESET) {
                GPIO_SetBits(GPIOC, GPIO_Pin_13);
            } else {
                GPIO_ResetBits(GPIOC, GPIO_Pin_13);
            
            }
            
            Serial_SendString("Ok");
            // 切换LED状态
        }
	}
}



#endif

