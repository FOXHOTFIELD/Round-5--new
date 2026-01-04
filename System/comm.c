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
volatile float Comm_encoder_a = 0.0f; //在速度环pid任务中更新
volatile float Comm_encoder_b = 0.0f; //在速度环pid任务中更新
volatile float Comm_out_a = 0.0f; //在速度环pid任务中更新
volatile float Comm_out_b = 0.0f; //在速度环pid任务中更新
volatile float Comm_pose_tag = 0.0f; //在姿态环pid任务中更新

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
    uint32_t prev_pid_cnt = g_pid_loop_counter;
    float s_fps_local = 0.0f;
    const float alpha = 0.1f;
    for (;;)
    {
        uint32_t now = DWT_GetCycles();
        uint32_t diff = now - prev_cc;
        prev_cc = now;
        float dt_s = (float)diff / (float)SystemCoreClock;
        /* 使用 PID 任务计数差分估计实际帧率 */
        uint32_t pid_cnt = g_pid_loop_counter;
        uint32_t pid_diff = pid_cnt - prev_pid_cnt;
        prev_pid_cnt = pid_cnt;

        if (pid_diff > 0 && dt_s > 0.0f) {
            float inst_fps = ((float)pid_diff) / dt_s;
            float inst_dt_ms = (dt_s / (float)pid_diff) * 1000.0f;
            s_fps_local = (s_fps_local <= 0.0f) ? inst_fps : (s_fps_local * (1.0f - alpha) + inst_fps * alpha);
            Comm_fps = s_fps_local;
            Comm_dt_ms = inst_dt_ms;
        }
        vTaskDelayUntil(&last, period);
    }
}

/* vCommTask：周期性读取最新姿态/转速/测得周期并通过串口发送 JustFloat 帧 */
static void vCommTask(void *pv)
{
    (void)pv;
    for (;;)
    {
        float msg[15];
        msg[0] = Comm_pitch;
        msg[1] = Comm_roll;
        msg[2] = Comm_yaw;
        msg[3] = Comm_encoder_a;
        //msg[4] = Comm_fps;
        //msg[5] = Comm_dt_ms;
        msg[4] = Comm_out_a;
        msg[5] = Comm_encoder_b;
        msg[6] = Comm_out_b;
        /* 传输姿态环 PID（KP/KI/KD）参数，供上位机显示/记录 */
        PID_GetConfigParam(PID_LOOP_ATTITUDE, PID_CONFIG_KP, msg + 7);
        PID_GetConfigParam(PID_LOOP_ATTITUDE, PID_CONFIG_KI, msg + 8);
        PID_GetConfigParam(PID_LOOP_ATTITUDE, PID_CONFIG_KD, msg + 9);
        PID_GetParamTarget(PID_LOOP_SPEED, PID_MOTOR_1, msg + 10);
        PID_GetParamTarget(PID_LOOP_SPEED, PID_MOTOR_2, msg + 11);
        msg[12] = Comm_pose_tag;
        msg[13] = Comm_fps;     /* DWT 实测帧率（Hz） */
        msg[14] = Comm_dt_ms;   /* DWT 实测周期（ms） */

        /* 附加: 当前堆剩余与历史最小堆剩余（Bytes） */
//       msg[10] = (float)xPortGetFreeHeapSize();
//        msg[11] = (float)xPortGetMinimumEverFreeHeapSize();
       //msg[10] = (float)xPortGetFreeHeapSize();
        Serial_SendJustFloat(msg, 15);

        Serial_ProcessRxPackets();

        vTaskDelay(pdMS_TO_TICKS(COMM_UPLOAD_INTERVAL_MS));
    }
}

void Comm_Init(void)
{
    /* 创建队列（如果已经创建则忽略） */
    if (xCommQueue == NULL) {
        xCommQueue = xQueueCreate(3, sizeof(CommMsg_t));
    }
    /* 创建任务：DWT 优先级略高以保证频率测量稳定 */
    if (xTaskCreate(vDWTTask, "DWT", 128, NULL, tskIDLE_PRIORITY + 3, NULL) != pdPASS) {
    }
    xTaskCreate(vCommTask, "Comm", 196, NULL, tskIDLE_PRIORITY + 2, NULL);
}
