#include "MenuKeys.h"
#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

extern uint8_t Menu_Flag;

// 按键事件队列句柄（模块内部使用）
static QueueHandle_t xKeyQueue = NULL;
// 当某些操作（如准备运行）需要临时阻止自动刷新时置位（菜单层可访问）
extern volatile BaseType_t oled_update_blocked;

// ---------------- 轮询扫描按键（不使用 EXTI） ----------------
#define KEY_DEBOUNCE_MS       40
#define TOGGLE_LONG_MS        700
#define SCAN_PERIOD_MS        10

// 计算防抖需要的扫描次数
#define DEBOUNCE_TICKS        (KEY_DEBOUNCE_MS / SCAN_PERIOD_MS)

// 索引映射
#define IDX_UP     0
#define IDX_DOWN   1
#define IDX_TOGGLE 2

// 单一周期扫描定时器句柄
static TimerHandle_t scan_timer = NULL;

// key state
static uint8_t key_stable_state[3] = {0,0,0}; // 1 = pressed
static uint8_t key_change_count[3] = {0,0,0}; // 连续不同采样计数，用于防抖
static uint8_t key_pressed_flag[3] = {0,0,0}; // 按键处于按下的逻辑标志（用于 toggle）
static uint16_t key_pressed_ms[3] = {0,0,0}; // 按下时间累计（ms）
static uint8_t toggle_long_fired = 0;

// forward
static void prv_scan_timer_cb(TimerHandle_t xTimer);
// ---------------- 轮询扫描按键（不使用 EXTI） ----------------


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

// 已移除 EXTI/ISR 相关代码，改用周期扫描实现（见 prv_scan_timer_cb）

/**
 * 防抖定时器回调（Timer Service/Timer Task 上下文）
 *
 * 描述：定时器到期后读取按键真实电平，与记录的稳定状态比较，
 *       若状态发生变化则根据按键类型发送相应的 `KeyEvent_t` 到队列，
 *       并在处理完成后重新使能对应的 EXTI 线。
 * 参数：`xTimer` - 触发该回调的定时器句柄（Timer ID 存放按键索引）
 * 返回：无
 */
/**
 * 周期扫描定时器回调（每 SCAN_PERIOD_MS）
 *
 * 描述：在 Timer Service/Timer Task 上下文定期采样按键电平，
 *       通过连续不同采样计数实现防抖；对 Toggle 实现长按检测，
 *       在合适时机向 `xKeyQueue` 发送按键事件。
 */
static void prv_scan_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    for (uint8_t i = 0; i < 3; i++) {
        uint8_t current = prv_read_key_index(i);

        if (current != key_stable_state[i]) {
            // 采样值与稳定值不同时，增加不一致计数
            if (++key_change_count[i] >= DEBOUNCE_TICKS) {
                // 经过去抖，确认状态改变
                key_stable_state[i] = current;
                key_change_count[i] = 0;

                if (current == 1) {
                    // 按下已确认
                    if (i == IDX_UP) {
                        KeyEvent_t ev = KEY_EVENT_Up;
                        if (xKeyQueue) xQueueSend(xKeyQueue, &ev, 0);
                    } else if (i == IDX_DOWN) {
                        KeyEvent_t ev = KEY_EVENT_Down;
                        if (xKeyQueue) xQueueSend(xKeyQueue, &ev, 0);
                    } else if (i == IDX_TOGGLE) {
                        // Toggle 按下：开始计时以检测长按
                        key_pressed_flag[i] = 1;
                        key_pressed_ms[i] = 0;
                        toggle_long_fired = 0;
                    }
                } else {
                    // 释放已确认
                    if (i == IDX_TOGGLE) {
                        /* 如果已经触发并发送了长按事件，释放时不再发送普通按键事件，避免重复 */
                        if (!toggle_long_fired) {
                            KeyEvent_t ev = KEY_EVENT_Toggle;
                            if (xKeyQueue) xQueueSend(xKeyQueue, &ev, 0);
                        }
                        key_pressed_flag[i] = 0;
                        key_pressed_ms[i] = 0;
                        toggle_long_fired = 0;
                    }
                }
            }
        } else {
            // 与稳定值一致，重置不一致计数
            key_change_count[i] = 0;
        }

        // 如果 toggle 正在按下，累计时间并判定长按
        if (i == IDX_TOGGLE && key_pressed_flag[IDX_TOGGLE]) {
            key_pressed_ms[IDX_TOGGLE] += SCAN_PERIOD_MS;
            if (!toggle_long_fired && key_pressed_ms[IDX_TOGGLE] >= TOGGLE_LONG_MS) {
                toggle_long_fired = 1;
                /* 在达到长按阈值时立即发送长按事件（Timer 服务/任务上下文） */
                if (xKeyQueue) {
                    KeyEvent_t ev = KEY_EVENT_Toggle_Long;
                    xQueueSend(xKeyQueue, &ev, 0);
                }
            }
        }
    }
}

/* Toggle 长按定时器回调 已移除（改用轮询计时） */

/**
 * 初始化按键硬件与资源
 *
 * 描述：配置按键对应的 GPIO（上拉输入），创建周期扫描定时器和按键事件队列 `xKeyQueue`。
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

    // 创建周期扫描定时器（轮询按键，不使用 EXTI）
    scan_timer = xTimerCreate("key_scan", pdMS_TO_TICKS(SCAN_PERIOD_MS), pdTRUE, NULL, prv_scan_timer_cb);

    // 创建按键事件队列（长度 10），每个元素为 KeyEvent_t
    if (xKeyQueue == NULL) {
        xKeyQueue = xQueueCreate(10, sizeof(KeyEvent_t));
    }

    /* 初始化稳定电平 */
    key_stable_state[IDX_UP] = prv_read_key_index(IDX_UP);
    key_stable_state[IDX_DOWN] = prv_read_key_index(IDX_DOWN);
    key_stable_state[IDX_TOGGLE] = prv_read_key_index(IDX_TOGGLE);
    // 启动扫描定时器
    if (scan_timer) {
        xTimerStart(scan_timer, 0);
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
