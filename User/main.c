#include "myHeader.h"

//#define TEST

#ifndef TEST

int main(void)
{

	/*NVIC中断分组*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);			//配置NVIC为分组4

	/* 系统初始化（startup/系统初始化通常已在启动代码中完成） */
	SystemInit();

	/* 初始化 */
    OLED_Init();
    OLED_ShowChar(0, 0, '1', OLED_6X8);
    OLED_Update();
    Menu_Init();

	/* 创建任务：优先级 */

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

	/* 初始化 */
    OLED_Init();
    
    OLED_ShowChar(0, 0, '1', OLED_6X8);
    OLED_Update();
    
   // Menu_Init();

	/* 创建任务：优先级 */

	/* 启动调度 */
	/* 若调度器返回，进入死循环 */
	while (1)
	{
	}
}




#endif
