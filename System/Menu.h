#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stdbool.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "OLED.h"
#include "MotorDrive.h"


#define KEY_Up (GPIO_Pin_7)        //PA7
#define KEY_Down (GPIO_Pin_1)      //PB1
#define KEY_Toggle (GPIO_Pin_0)    //PB0

#define Menu_thrd_correct_port (GPIO_Pin_12) //PB12用于启动input的thrd_correct

#define Menu_Row_1 (16)
#define Menu_Row_2 (32)
#define Menu_Row_3 (48)
#define Menu_Row_Pitch (16)       //行距
#define Menu_Row_Head (8)   //行首

// 按键事件枚举：按下时会把对应事件放入队列
typedef enum {
	KEY_EVENT_NONE = 0,
	KEY_EVENT_Up,
	KEY_EVENT_Down,
    KEY_EVENT_Toggle,
    KEY_EVENT_Toggle_Long,
}KeyEvent_t;

//typedef enum {
    //Edit, //编辑参数
    //Wait, //等待发车
    //Run   //行驶
//} MenuMode;


// 初始化菜单显示及按键轮询任务（调用后会创建任务和队列）
void Menu_Init(void);


/* 设置显示任务句柄，用于通知唤醒显示任务 */
void Menu_SetShowHandle(TaskHandle_t h);

// 从按键事件队列获取一个事件，参数为等待的最大 tick 数，
// 若超时返回 KEY_EVENT_NONE
KeyEvent_t Menu_GetKeyEvent(TickType_t xTicksToWait);

enum PageIndex_t
{
    Main = 0,
    Main_Run,
    Main_Data,
    Main_Control,
};
//菜单页面索引枚举
typedef enum PageIndex_t PageIndex_t;

//菜单光标索引别名
typedef uint8_t CursorIndex_t;    //1, 2 or 3

//菜单功能句柄
typedef void (*MenuHandler_t)(KeyEvent_t ev);

struct MenuFuncs_t
{
    MenuHandler_t Func_Up_Down;
    MenuHandler_t Func_Toggle;
    MenuHandler_t Func_Toggle_Long;
};
//菜单功能结构体
typedef struct MenuFuncs_t MenuFuncs_t;

struct MenuItem_t
{
    char* Title;
    char* Stuff;
    uint8_t OptionCount;
    PageIndex_t Page_Index;
    MenuFuncs_t Funcs;
    const struct MenuItem_t * const *subItem; // 指向子菜单项指针数组（数组元素为指向子项的指针），使用 OptionCount 表示长度
    
};
//菜单项目结构体
typedef struct MenuItem_t MenuItem_t;

struct MenuStatus_t
{
    PageIndex_t Page_Index;
    CursorIndex_t Cursor_Index;   //1, 2 or 3
    const MenuItem_t* curItem;
};
//菜单状态结构体
typedef struct MenuStatus_t MenuStatus_t;

#endif
