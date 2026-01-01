#include "comm.h"
#include "PID.h"

#include "stm32f10x.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"


/* 全局定义 */

QueueHandle_t xCommQueue = NULL; //在CommInit中初始化 在Menu中使用以接收任务

volatile float Comm_pitch = 0.0f;
volatile float Comm_roll = 0.0f;
volatile float Comm_yaw = 0.0f;
volatile float Comm_fps = 0.0f;
volatile float Comm_dt_ms = 0.0f;
volatile float Comm_encoder = 0.0f; //在速度环pid任务中更新
volatile float Comm_out = 0.0f; //在速度环pid任务中更新

/* DWT 工具 */
static inline void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t DWT_GetCycles(void)
{
    return DWT->CYCCNT;
}

/* vDWTTask：周期性计算真实帧间 dt 与 fps（EMA 平滑），写入共享变量 */
static void vDWTTask(void *pv)
{
    (void)pv;
    DWT_Init();
    TickType_t last = xTaskGetTickCount();
    /* 使用与姿态解算相同的频率作为采样参考 */
#ifndef POSE_UPDATE_HZ
#define POSE_UPDATE_HZ 200
#endif
    TickType_t period = (configTICK_RATE_HZ / POSE_UPDATE_HZ);
    if (period == 0) period = 1;
    uint32_t prev_cc = DWT_GetCycles();
    float s_fps_local = 0.0f;
    const float alpha = 0.1f;
    for (;;)
    {
        uint32_t now = DWT_GetCycles();
        uint32_t diff = now - prev_cc;
        prev_cc = now;
        float dt_s = (float)diff / (float)SystemCoreClock;
        float dt_ms = dt_s * 1000.0f;
        float inst_fps = (dt_s > 0.0f) ? (1.0f / dt_s) : 0.0f;
        s_fps_local = (s_fps_local <= 0.0f) ? inst_fps : (s_fps_local * (1.0f - alpha) + inst_fps * alpha);
        Comm_fps = s_fps_local;
        Comm_dt_ms = dt_ms;
        vTaskDelayUntil(&last, period);
    }
}

/* vCommTask：周期性读取最新姿态/转速并通过串口发送固定 6-float 帧 */
static void vCommTask(void *pv)
{
    (void)pv;
    for (;;)
    {
        float out[10];
        out[0] = Comm_pitch;
        out[1] = Comm_roll;
        out[2] = Comm_yaw;
        out[3] = Comm_encoder;
        out[4] = Comm_fps;
        out[5] = Comm_dt_ms;
        out[6] = Comm_out;
        PID_GetConfigParam(0, 0, out + 7);
        PID_GetConfigParam(0, 1, out + 8);
        PID_GetConfigParam(0, 2, out + 9);

        Serial_SendJustFloat(out, 10);

        Serial_ProcessRxPackets();

        vTaskDelay(pdMS_TO_TICKS(COMM_UPLOAD_INTERVAL_MS));
    }
}

void Comm_Init(void)
{
    /* 创建队列（如果已经创建则忽略） */
    if (xCommQueue == NULL) {
        xCommQueue = xQueueCreate(8, sizeof(CommMsg_t));
    }
    /* 创建任务：DWT 优先级略高以保证频率测量稳定 */
    if (xTaskCreate(vDWTTask, "DWT", 256, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
    }
    xTaskCreate(vCommTask, "Comm", 256, NULL, tskIDLE_PRIORITY + 1, NULL);
}
