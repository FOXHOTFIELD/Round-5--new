#ifndef __MENU_H
#define __MENU_H

#include "stm32f10x.h"
#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

#include "OLED.h"


#define KEY_Up (GPIO_Pin_8)        //PA8
#define KEY_Down (GPIO_Pin_15)     //PB15
#define KEY_Toggle (GPIO_Pin_14)     //PB14

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
} KeyEvent_t;

//typedef enum {
    //Edit, //编辑参数
    //Wait, //等待发车
    //Run   //行驶
//} MenuMode;


// 初始化菜单显示及按键轮询任务（调用后会创建任务和队列）
void Menu_Init(void);

// 从按键事件队列获取一个事件，参数为等待的最大 tick 数，
// 若超时返回 KEY_EVENT_NONE
KeyEvent_t Menu_GetKeyEvent(TickType_t xTicksToWait);
static void Menu_turnEdit(void);
static void Menu_changeSpeed(KeyEvent_t ev);
static void Menu_prepareRun(KeyEvent_t ev);
static void Menu_thrd_correct(void);




enum PageIndex
{
    Main = 0,
    Main_Data,
    Main_Control,
};
//菜单页面索引枚举
typedef enum PageIndex PageIndex;

//菜单光标索引别名
typedef uint8_t CursorIndex;    //1, 2 or 3

//菜单功能句柄
typedef void (*MenuHandler)(KeyEvent_t ev);

struct MenuFuncs
{
    MenuHandler Func_Up_Down;
    MenuHandler Func_Toggle;
    MenuHandler Func_Toggle_Long;
};
//菜单功能结构体
typedef struct MenuFuncs MenuFuncs;

struct MenuItem
{
    char* Title;
    char* Stuff;
    uint8_t OptionCount;
    PageIndex Page_Index;
    MenuFuncs Funcs;
};
//菜单项目结构体
typedef struct MenuItem MenuItem;

struct MenuStatus
{
    PageIndex Page_Index;
    CursorIndex Cursor_Index;   //1, 2 or 3
    const MenuItem* curItem;
};
//菜单状态结构体
typedef struct MenuStatus MenuStatus;

#endif
