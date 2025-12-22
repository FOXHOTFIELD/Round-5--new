#include "MenuKeys.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

extern uint8_t Flag;

// 按键事件队列句柄（模块内部使用）
static QueueHandle_t xKeyQueue = NULL;
// 当某些操作（如准备运行）需要临时阻止自动刷新时置位（菜单层可访问）
extern volatile BaseType_t oled_update_blocked;

// ---------------- 使用 EXTI 中断 + 定时器 的按键处理 ----------------
#define KEY_DEBOUNCE_MS       40
#define TOGGLE_LONG_MS        1000

// 定时器句柄
static TimerHandle_t debounce_timers[3] = {NULL, NULL, NULL}; // 0:Up,1:Down,2:Toggle
static TimerHandle_t toggle_long_timer = NULL;

// key state
static uint8_t key_stable_state[3] = {0,0,0}; // 1 = pressed
static uint8_t toggle_long_fired = 0;

// 索引映射
#define IDX_UP     0
#define IDX_DOWN   1
#define IDX_TOGGLE 2

// forward
static void prv_debounce_timer_cb(TimerHandle_t xTimer);
static void prv_toggle_long_cb(TimerHandle_t xTimer);
// ---------------- 使用 EXTI 中断 + 定时器 的按键处理 ----------------


/**
 * 读取按键电平（根据索引）
 *
 * 描述：根据传入的按键索引返回对应 GPIO 引脚的电平，
 *       约定返回值为 1 表示按下（低电平），0 表示未按下。
 * 参数：`idx` - 按键索引（IDX_UP/IDX_DOWN/IDX_TOGGLE）
 * 返回：uint8_t，1 = 按下，0 = 未按下
 */
static uint8_t prv_read_key_index(uint8_t idx)
{
    if (idx == IDX_UP) return (GPIO_ReadInputDataBit(GPIOA, KEY_Up) == Bit_RESET) ? 1 : 0;
    if (idx == IDX_DOWN) return (GPIO_ReadInputDataBit(GPIOB, KEY_Down) == Bit_RESET) ? 1 : 0;
    if (idx == IDX_TOGGLE) return (GPIO_ReadInputDataBit(GPIOB, KEY_Toggle) == Bit_RESET) ? 1 : 0;
    return 0;
}

/**
 * 在 ISR 中启动防抖定时器并屏蔽对应 EXTI 线
 *
 * 描述：此函数在 EXTI 中断上下文被调用，先屏蔽对应的中断线以防
 *       连续触发，然后通过 `xTimerStartFromISR` 启动去抖定时器。
 * 参数：`line` - EXTI 行掩码（如 EXTI_Line8）
 *       `idx`  - 按键索引（用于选择哪个防抖定时器）
 * 返回：无
 */
static void prv_start_debounce_from_isr(uint32_t line, uint8_t idx)
{
    // 屏蔽中断线，避免连续触发
    EXTI->IMR &= ~line;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (debounce_timers[idx]) {
        xTimerStartFromISR(debounce_timers[idx], &xHigherPriorityTaskWoken);
    }
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * EXTI9_5 中断处理函数（处理 PA8）
 *
 * 描述：检测 PA8 的中断源并清除标志，随后调用防抖启动函数。
 */
void EXTI9_5_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line8) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line8);
        prv_start_debounce_from_isr(EXTI_Line8, IDX_UP);
    }
}

/**
 * EXTI15_10 中断处理函数（处理 PB14/PB15）
 *
 * 描述：检测 PB14（Toggle）和 PB15（Down）对应的中断标志，
 *       清除中断并启动防抖流程。
 */
void EXTI15_10_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line14);
        prv_start_debounce_from_isr(EXTI_Line14, IDX_TOGGLE);
    }
    if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line15);
        prv_start_debounce_from_isr(EXTI_Line15, IDX_DOWN);
    }
}

/**
 * 防抖定时器回调（Timer Service/Timer Task 上下文）
 *
 * 描述：定时器到期后读取按键真实电平，与记录的稳定状态比较，
 *       若状态发生变化则根据按键类型发送相应的 `KeyEvent_t` 到队列，
 *       并在处理完成后重新使能对应的 EXTI 线。
 * 参数：`xTimer` - 触发该回调的定时器句柄（Timer ID 存放按键索引）
 * 返回：无
 */
static void prv_debounce_timer_cb(TimerHandle_t xTimer)
{
    uintptr_t idx = (uintptr_t) pvTimerGetTimerID(xTimer);
    uint8_t i = (uint8_t) idx;
    uint8_t current = prv_read_key_index(i);

    // 如果状态发生变化
    if (current != key_stable_state[i]) {
        // 上升沿（按下）
        if (current == 1) {
            if (i == IDX_UP) {
                KeyEvent_t ev = KEY_EVENT_Up;
                if (xKeyQueue) xQueueSend(xKeyQueue, &ev, 0);
            } else if (i == IDX_DOWN) {
                KeyEvent_t ev = KEY_EVENT_Down;
                if (xKeyQueue) xQueueSend(xKeyQueue, &ev, 0);
            } else if (i == IDX_TOGGLE) {
                // toggle 按下：启动长按定时器，重置标志
                toggle_long_fired = 0;
                if (toggle_long_timer) xTimerStart(toggle_long_timer, 0);
            }
        } else {
            // 下降沿（释放）
            if (i == IDX_TOGGLE) {
                // toggle 释放：根据长按标志判断发送事件
                if (toggle_long_timer) xTimerStop(toggle_long_timer, 0);
                KeyEvent_t ev = toggle_long_fired ? KEY_EVENT_Toggle_Long : KEY_EVENT_Toggle;
                if (xKeyQueue) xQueueSend(xKeyQueue, &ev, 0);
                toggle_long_fired = 0;
            }
        }
        key_stable_state[i] = current;
    }

    // 重新使能对应 EXTI 线
    uint32_t line = (i == IDX_UP) ? EXTI_Line8 : (i == IDX_DOWN ? EXTI_Line15 : EXTI_Line14);
    EXTI->IMR |= line;
}

/**
 * Toggle 长按定时器回调
 *
 * 描述：当 Toggle 键长按计时器到期时设置 `toggle_long_fired` 标志，
 *       表示后续释放事件应被视为长按。
 * 参数：`xTimer` - 触发该回调的定时器句柄
 * 返回：无
 */
static void prv_toggle_long_cb(TimerHandle_t xTimer)
{
    toggle_long_fired = 1;
}

/**
 * 初始化按键硬件与资源
 *
 * 描述：配置按键对应的 GPIO（上拉输入），AFIO/EXTI 线路与 NVIC，
 *       创建防抖定时器与长按定时器，并创建按键事件队列 `xKeyQueue`。
 * 参数：无
 * 返回：无
 */
void MenuKeys_Init(void)
{
    // 打开 GPIO 时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_IS;
    GPIO_IS.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_IS.GPIO_Pin = KEY_Up;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_IS);

    GPIO_IS.GPIO_Pin = KEY_Down | KEY_Toggle;
    GPIO_Init(GPIOB, &GPIO_IS);

    // 配置 EXTI 中断（PA8, PB14, PB15）
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource8);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource14);
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource15);

    {
        EXTI_InitTypeDef EXTI_InitStructure;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        /* Don't enable EXTI lines yet — wait until timers and queue are created */
        EXTI_InitStructure.EXTI_LineCmd = DISABLE;

        EXTI_InitStructure.EXTI_Line = EXTI_Line8;
        EXTI_Init(&EXTI_InitStructure);
        EXTI_InitStructure.EXTI_Line = EXTI_Line14;
        EXTI_Init(&EXTI_InitStructure);
        EXTI_InitStructure.EXTI_Line = EXTI_Line15;
        EXTI_Init(&EXTI_InitStructure);
    }

    // 创建防抖与长按定时器（不启动）
    debounce_timers[IDX_UP] = xTimerCreate("db_up", pdMS_TO_TICKS(KEY_DEBOUNCE_MS), pdFALSE, (void*)(uintptr_t)IDX_UP, prv_debounce_timer_cb);
    debounce_timers[IDX_DOWN] = xTimerCreate("db_dn", pdMS_TO_TICKS(KEY_DEBOUNCE_MS), pdFALSE, (void*)(uintptr_t)IDX_DOWN, prv_debounce_timer_cb);
    debounce_timers[IDX_TOGGLE] = xTimerCreate("db_tg", pdMS_TO_TICKS(KEY_DEBOUNCE_MS), pdFALSE, (void*)(uintptr_t)IDX_TOGGLE, prv_debounce_timer_cb);
    toggle_long_timer = xTimerCreate("tg_long", pdMS_TO_TICKS(TOGGLE_LONG_MS), pdFALSE, NULL, prv_toggle_long_cb);

    // 创建按键事件队列（长度 10），每个元素为 KeyEvent_t
    if (xKeyQueue == NULL) {
        xKeyQueue = xQueueCreate(10, sizeof(KeyEvent_t));
    }

    /* 在启用 EXTI 之前初始化稳定电平并启用 EXTI/NVIC，避免中断在定时器/队列
       尚未就绪时导致 EXTI 被屏蔽且永不恢复的问题。 */
    key_stable_state[IDX_UP] = prv_read_key_index(IDX_UP);
    key_stable_state[IDX_DOWN] = prv_read_key_index(IDX_DOWN);
    key_stable_state[IDX_TOGGLE] = prv_read_key_index(IDX_TOGGLE);

    {
        EXTI_InitTypeDef EXTI_InitStructure;
        EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
        EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
        EXTI_InitStructure.EXTI_LineCmd = ENABLE;

        EXTI_InitStructure.EXTI_Line = EXTI_Line8;
        EXTI_Init(&EXTI_InitStructure);
        EXTI_InitStructure.EXTI_Line = EXTI_Line14;
        EXTI_Init(&EXTI_InitStructure);
        EXTI_InitStructure.EXTI_Line = EXTI_Line15;
        EXTI_Init(&EXTI_InitStructure);

        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

        NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
        NVIC_Init(&NVIC_InitStructure);
    }
}

/**
 * 从按键事件队列获取事件
 *
 * 描述：从模块内部的 `xKeyQueue` 中取出一个 `KeyEvent_t`，若队列为空
 *       则等待 `xTicksToWait` 个 tick；超时返回 `KEY_EVENT_NONE`。
 * 参数：`xTicksToWait` - 最长等待的 tick 数，可使用 `portMAX_DELAY` 阻塞
 * 返回：KeyEvent_t（取到的事件或 `KEY_EVENT_NONE`）
 */
KeyEvent_t Menu_GetKeyEvent(TickType_t xTicksToWait)
{
    KeyEvent_t ev = KEY_EVENT_NONE;
    if (xKeyQueue == NULL) return KEY_EVENT_NONE;
    if (xQueueReceive(xKeyQueue, &ev, xTicksToWait) == pdTRUE) {
        return ev;
    }
    return KEY_EVENT_NONE;
}
