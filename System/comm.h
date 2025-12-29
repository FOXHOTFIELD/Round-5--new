#pragma onec

#ifndef __SYSTEM_COMM_H
#define __SYSTEM_COMM_H

#include "FreeRTOS.h"
#include "queue.h"

#include "Serial.h"

typedef enum {
    COMM_TYPE_NONE = 0,
    COMM_TYPE_POSE = 1,
    COMM_TYPE_ENCODER = 2
} CommType_t;

typedef struct {
    uint8_t type; /* CommType_t */
    uint8_t len;  /* number of floats in data (max 6) */
    float data[6];
} CommMsg_t;

/* 上传间隔（ms），可编译时通过 -DCOMM_UPLOAD_INTERVAL_MS=... 覆盖 */
#ifndef COMM_UPLOAD_INTERVAL_MS
#define COMM_UPLOAD_INTERVAL_MS 20
#endif

/* 全局队列与共享最新值（由各传感器任务写入，comm.c 中的任务读取并发送） */
extern QueueHandle_t xCommQueue;

extern volatile float Comm_pitch;
extern volatile float Comm_roll;
extern volatile float Comm_yaw;
extern volatile float Comm_fps;
extern volatile float Comm_dt_ms;
extern volatile float Comm_encoder;

/* 初始化通信子系统（创建队列与任务） */
void Comm_Init(void);

#endif
