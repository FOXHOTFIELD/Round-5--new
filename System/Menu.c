#include "Menu.h"
#include "MenuKeys.h"
#include "comm.h"
#include "PID.h"
#include <string.h>

uint8_t Menu_Flag = 0;
extern Motor_STBY_t Motor_STBY;
/* 功能概要:
 * - 菜单状态管理：定义并初始化 `Menu_Status`（光标与页面索引）。
 * - 主菜单项：提供 `Menu_Main`（标题 "Main"，选项 "Run\\nData"）。
 * - 菜单初始化接口：`Menu_Init` 调用 `MenuKeys_Init` 并向菜单显示任务发送初始通知。
 * - 按键处理：在 `MenuKeys.c` 中使用 EXTI 中断 + 定时器实现防抖、长按识别，并通过 FreeRTOS 队列发送 `KeyEvent_t` 事件。
 * - OLED 更新控制：通过全局 `oled_update_blocked` 允许临时屏蔽自动刷新。
 * - 按键事件 API：`Menu_GetKeyEvent` 提供从队列读取事件的同步接口。
 */

// 菜单相关全局（用于与按键模块协同）
TaskHandle_t xMenuShowHandle = NULL; // 菜单显示任务句柄，用于接收通知唤醒刷新
volatile BaseType_t oled_update_blocked = pdFALSE; // 当某些操作需要临时阻止自动刷新时置位

static void vMenu_ReadCommandFromSerial(void *pv);

static void vMenu_ShowTask(void* pv);
static void vMenuTask(void *pv);
static void Menu_DrawItem(void);
static void Menu_DrawCursor(void);
static void Menu_ShowDebug(void);
static void Menu_MoveCursor(KeyEvent_t ev);
static void Menu_MoveItem(KeyEvent_t ev);
static void Menu_ControlSTBY(KeyEvent_t ev);

// 菜单项：避免与 enum 名称冲突，变量以 "MenuItem_" 前缀命名
const MenuItem_t MenuItem_Main_Run = {
    .Funcs = {
        .Func_Up_Down = NULL,
        .Func_Toggle = Menu_ControlSTBY,
        .Func_Toggle_Long = Menu_MoveItem,
    },
    .Page_Index = Main_Run,
    .OptionCount = 1,
    .Stuff = "Rnu\n       ",
    .Title = "Run",
    .subItem = NULL
};

const MenuItem_t MenuItem_Main_Data = {
    .Funcs = {
        .Func_Up_Down = Menu_MoveCursor,
        .Func_Toggle = NULL,
        .Func_Toggle_Long = Menu_MoveItem
    },
    .Page_Index = Main_Data,
    .OptionCount = 2,
    .Stuff = "Speed\nAngle\n",
    .Title = "Data",
    .subItem = NULL
};

// 子菜单指针数组（初始化为指向上面的命名子项）
const MenuItem_t * const MenuItem_Main_SubItems[] = {
    &MenuItem_Main_Run,
    &MenuItem_Main_Data
};

const MenuItem_t MenuItem_Main = {
    .Funcs = {
        .Func_Up_Down = Menu_MoveCursor,
        .Func_Toggle = Menu_MoveItem,
        .Func_Toggle_Long = NULL
    },
    .Page_Index = Main,
    .OptionCount = 2,
    .Stuff = "Run\nData",
    .Title = "Main",
    .subItem = MenuItem_Main_SubItems
};

//菜单状态变量
MenuStatus_t Menu_Status = {
    .Cursor_Index = 1,
    .Page_Index = Main,
    .curItem = &MenuItem_Main
};

/**
 * 初始化菜单模块
 *
 * 功能：初始化按键硬件与相关任务，创建并启动菜单显示任务，
 *       并向显示任务发送第一次通知以绘制初始界面。
 * 参数：无
 * 返回：无
 */
void Menu_Init(void)
{
    // 初始化按键硬件与事件队列（从 MenuKeys 模块）
    MenuKeys_Init();
    /* 在此创建菜单相关任务，便于模块自包含初始化 */
    (void)xTaskCreate(vMenuTask, "MenuTask", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
    TaskHandle_t hMenuShow = NULL;
    (void)xTaskCreate(vMenu_ShowTask, "MenuShow", 256, NULL, tskIDLE_PRIORITY + 1, &hMenuShow);
    Menu_SetShowHandle(hMenuShow);
    if (hMenuShow != NULL)
        xTaskNotifyGive(hMenuShow); /* 初始通知，刷新首次界面 */

    (void)xTaskCreate(vMenu_ReadCommandFromSerial, "CmdFromSerial", 192, NULL, tskIDLE_PRIORITY + 1, NULL);
}

void Menu_SetShowHandle(TaskHandle_t h)
{
    xMenuShowHandle = h;
}

/**
  菜单主任务（事件处理）（FreeRTOS 任务）
 *
 * 功能：从按键事件队列阻塞读取 `KeyEvent_t`，根据事件更新菜单状态、
 *       调用当前菜单项的回调（若已分配），并通知显示任务刷新。
 * 参数：`pv` - 任务参数（未使用）
 * 返回：此函数不会返回（任务循环）
 */

static void vMenuTask(void *pv)
{
    KeyEvent_t ev;
    for(;;)
    {
        ev = Menu_GetKeyEvent(portMAX_DELAY);

        MenuHandler_t Func = NULL;
        switch (ev)
        {
        case KEY_EVENT_Up:
        case KEY_EVENT_Down:
                Func = Menu_Status.curItem->Funcs.Func_Up_Down;
            break;
        case KEY_EVENT_Toggle:
                Func = Menu_Status.curItem->Funcs.Func_Toggle;
            break;
        case KEY_EVENT_Toggle_Long:
                Func = Menu_Status.curItem->Funcs.Func_Toggle_Long;
            break;
        default:
            break;
        }

        if (Func != NULL)
            Func(ev);
        
        if (xMenuShowHandle != NULL)
            xTaskNotifyGive(xMenuShowHandle);
    }
}

/**
 * 菜单显示任务（FreeRTOS 任务）
 *
 * 功能：等待来自其他任务的通知（ulTaskNotifyTake），在未被
 *       `oled_update_blocked` 阻塞的情况下，调用绘制函数更新 OLED，
 *       并在最后调用 `OLED_Update()` 将缓冲区内容写入屏幕。
 * 参数：`pv` - 任务参数（未使用）
 * 返回：此函数不会返回（任务循环）
 */
static void vMenu_ShowTask(void* pv)
{
    (void) pv;

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //OLED_Clear();

        Menu_DrawItem();
        Menu_DrawCursor();
        Menu_ShowDebug();

        OLED_Update();
    }
}

/**
 * 绘制当前菜单项内容
 *
 * 功能：读取当前菜单项的 `Title` 和 `Stuff`（以 "\n" 分行），
 *       将标题与每行内容绘制到 OLED 的对应行。
 * 参数：无（使用全局 `Menu_Status.curItem`）
 * 返回：无
 */
static void Menu_DrawItem(void)
{
    char* Stuff = Menu_Status.curItem->Stuff;
    char* Title = Menu_Status.curItem->Title;

    OLED_ShowString(0, 0, Title, OLED_6X8);
    OLED_DrawLine(0, 11, 127, 11);

    /* 将 Stuff 按 "\n" 分割，分别显示在第 1、2、3 行（使用 OLED_8X16，高度 16 像素，Y = 16,32,48） */
    char lineBuf[32];
    const char *p = Stuff;
    int lineNo = 1;
    while (p != NULL && *p != '\0' && lineNo <= 3)
    {
        int len = 0;
        while (p[len] != '\0' && p[len] != '\n' && len < (int)sizeof(lineBuf) - 1) len++;
        if (len == 0)
        {
            /* 空行，显示空字符串 */
            lineBuf[0] = '\0';
        }
        else
        {
            memcpy(lineBuf, p, len);
            lineBuf[len] = '\0';
        }

        OLED_ShowString(Menu_Row_Head, lineNo * Menu_Row_Pitch, lineBuf, OLED_8X16);

        p += len;
        if (*p == '\n') p++;
        lineNo++;
    }

}

/**
 * 绘制菜单光标
 *
 * 功能：根据 `Menu_Status.Cursor_Index` 在对应行绘制光标符号（'>'）。
 * 参数：无（使用全局 `Menu_Status.Cursor_Index`）
 * 返回：无
 */
static void Menu_DrawCursor(void)
{
    uint8_t option_count = Menu_Status.curItem->OptionCount;

    OLED_ClearArea(0, Menu_Row_1, 8, Menu_Row_Pitch * option_count);
    CursorIndex_t Cursor_Row = Menu_Status.Cursor_Index * Menu_Row_Pitch;
    OLED_ShowChar(0, Cursor_Row, '>', OLED_8X16);

}

/**
 * 显示调试信息（占位）
 *
 * 功能：用于在调试时显示额外信息，目前为空实现。
 * 参数：无
 * 返回：无
 */
static void Menu_ShowDebug(void)
{
    extern Motor_STBY_t Motor_STBY;
    //OLED_ShowFloatNum(95, 0, (uint8_t), 1, 2, OLED_6X8);
}

/**
 * 移动菜单光标
 *
 * 功能：根据传入的按键事件（Up/Down）调整 `Menu_Status.Cursor_Index`，
 *       并清除对应显示区域以便重绘。
 * 参数：`ev` - 按键事件，期望为 `KEY_EVENT_Up` 或 `KEY_EVENT_Down`
 * 返回：无
 */
static void Menu_MoveCursor(KeyEvent_t ev)
{

    CursorIndex_t cursor_index = Menu_Status.Cursor_Index;
    uint8_t option_count = Menu_Status.curItem->OptionCount;


    switch (ev)
    {
    case KEY_EVENT_Up:
        if (cursor_index <= 1)
            cursor_index = option_count;
        else
            cursor_index -= 1;
        break;
    case KEY_EVENT_Down:
        if (cursor_index >= option_count)
            cursor_index = 1;
        else
            cursor_index += 1;
        break;
    default:
        break;
    }

    // 更新状态结构中的光标索引，显示任务收到通知后会重绘光标
    Menu_Status.Cursor_Index = cursor_index;

}

static void Menu_MoveItem(KeyEvent_t ev)
{

    switch (ev)
    {
    case KEY_EVENT_Toggle:
        {
            /* 进入子菜单：根据当前光标索引选择子项并更新状态 */
            const MenuItem_t *cur = Menu_Status.curItem;
            const MenuItem_t * const *subs = cur->subItem;

            if (subs == NULL || cur->OptionCount == 0)
                return;

            uint8_t idx = Menu_Status.Cursor_Index;
            if (idx < 1 || idx > cur->OptionCount)
                return;

            const MenuItem_t *target = subs[idx - 1];

            Menu_Status.curItem = target;
            Menu_Status.Page_Index = target->Page_Index;
            Menu_Status.Cursor_Index = 1; // 进入新页面时光标回到第 1 项

            break;
        }
    case KEY_EVENT_Toggle_Long:
        {
            /*
            * 长按切回主菜单：
            * - 使用 Page_Index 的值设置光标索引（如果 PageIndex_t 枚举与光标位置有映射关系），
            *   这里通过强制类型转换将页面索引映射为光标索引，保证不越界。
            * - 将 Page_Index 置回 Main，并把当前项指回主菜单对象。
            */
            Menu_Status.Cursor_Index = (CursorIndex_t)Menu_Status.Page_Index;
            Menu_Status.Page_Index = Main;
            Menu_Status.curItem = &MenuItem_Main;

            break;
        }
    default:
        break;
    }

    if (xMenuShowHandle != NULL)
        xTaskNotifyGive(xMenuShowHandle);
}

static void Menu_ControlSTBY(KeyEvent_t ev)
{
    (void) ev;
    
    if (Motor_STBY)
    {
        Motor_SetStandbyState((FunctionalState)Disable);
    }
    else
    {
        Motor_SetStandbyState((FunctionalState)Enable);
    }
}

static void vMenu_ReadCommandFromSerial(void *pv)
{
    (void) pv;

    CommMsg_t msg;
    for (;;)
    {
        if (xQueueReceive(xCommQueue, &msg, portMAX_DELAY) == pdPASS)
        {
            // msg.len 为有效长度，msg.payload 为原始负载
            // 在这里按你的协议解析 payload
            
            PID_Loop_enum idx_loop;
            PID_Param_enum idx_param;
            float val;
            int loop_i, param_i;
            if (sscanf((const char*)msg.payload, "%d,%d,%f", &loop_i, &param_i, &val) == 3) {
                idx_loop = (PID_Loop_enum)loop_i;
                idx_param = (PID_Param_enum)param_i;
                PID_EditConfig(idx_loop, idx_param, val);
            }
        }
    }
}
