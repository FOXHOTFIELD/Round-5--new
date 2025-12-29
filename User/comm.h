/* 上位机上传间隔（毫秒），可在编译时通过预定义宏覆盖 */
#ifndef COMM_UPLOAD_INTERVAL_MS
#define COMM_UPLOAD_INTERVAL_MS 20 /* 默认 20ms -> 50Hz */
#endif

/* 统一最新传感器值（由各任务写入，vCommTask 周期性读取并发送合并帧） */
extern volatile float Comm_pitch;
extern volatile float Comm_roll;
extern volatile float Comm_yaw;
extern volatile float Comm_fps;
extern volatile float Comm_dt_ms;
extern volatile float Comm_encoder;
