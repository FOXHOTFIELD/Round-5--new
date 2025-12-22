#ifndef __MENU_KEYS_H
#define __MENU_KEYS_H

#include "Menu.h"

// 初始化按键硬件与软件防抖/长按处理
// 该模块负责按键的 EXTI 配置、定时器防抖、事件队列
// 菜单层通过 Menu_GetKeyEvent() 获取按键事件
void MenuKeys_Init(void);

// 从按键事件队列获取事件，等待时间以 FreeRTOS tick 为单位
KeyEvent_t Menu_GetKeyEvent(TickType_t xTicksToWait);

#endif
