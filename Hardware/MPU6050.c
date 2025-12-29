#include "MPU6050.h"

#define MPU6050_ADDRESS    0xD0

// 配置：是否启用 DWT 测帧率与是否随串口发送性能数据
#ifndef ENABLE_DWT_FPS
#define ENABLE_DWT_FPS 1
#endif
#ifndef ENABLE_FRAME_RATE_TELEM
#define ENABLE_FRAME_RATE_TELEM 1  // 0=仅发送姿态3浮点；1=发送5浮点(姿态+fps+dt_ms)
// 姿态更新频率（Hz）：可按需提升到 200/250/500
#ifndef POSE_UPDATE_HZ
#define POSE_UPDATE_HZ 200
#endif

// 串口遥测降采样：每多少帧发送一次（1=每帧，2=1/2 频率等）
#ifndef POSE_TELEM_DIV
#define POSE_TELEM_DIV 4  // 在 200Hz 解算下 -> 50Hz 遥测

#endif

#endif

static void vPoseUpdateTask(void *pvParameters);

/* DWT 帧率计算已移至 main.c 的独立任务 vDWTTask */

int16_t AccX = 0, AccY = 0, AccZ = 0, GyroX = 0, GyroY = 0, GyroZ = 0;
int16_t ax_off = 0, ay_off = 0, az_off = 0;
int16_t gx_off = 0, gy_off = 0, gz_off = 0;
uint8_t g_selfcorrect_finished = 0;

void MPU6050_StartPoseTask(void);

void MPU6050_WriteReg(uint8_t RegAddress, uint8_t Data)
{
	Soft_I2C_Start();
	Soft_I2C_SendByte(MPU6050_ADDRESS); // write address + write bit
	Soft_I2C_WaitAck();
	Soft_I2C_SendByte(RegAddress);
	Soft_I2C_WaitAck();
	Soft_I2C_SendByte(Data);
	Soft_I2C_WaitAck();
	Soft_I2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t val;
	Soft_I2C_Start();
	Soft_I2C_SendByte(MPU6050_ADDRESS); // write
	Soft_I2C_WaitAck();
	Soft_I2C_SendByte(RegAddress);
	Soft_I2C_WaitAck();

	Soft_I2C_Start();
	Soft_I2C_SendByte(MPU6050_ADDRESS | 0x01); // read
	Soft_I2C_WaitAck();
	val = Soft_I2C_ReadByte(0); // single byte -> NACK at end
	Soft_I2C_Stop();
	return val;
}

void MPU6050_ReadBytes(uint8_t startReg, uint8_t *buf, uint8_t len)
{
	if (len == 0) return;
	Soft_I2C_Start();
	Soft_I2C_SendByte(MPU6050_ADDRESS); // write
	Soft_I2C_WaitAck();
	Soft_I2C_SendByte(startReg);
	Soft_I2C_WaitAck();

	Soft_I2C_Start();
	Soft_I2C_SendByte(MPU6050_ADDRESS | 0x01); // read
	Soft_I2C_WaitAck();

	for (uint8_t i = 0; i < len; i++) {
		buf[i] = Soft_I2C_ReadByte((i < (len - 1)) ? 1 : 0);
	}
	Soft_I2C_Stop();
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_Init(void)
{
	Soft_I2C_Init();
	Delay_ms(10);

	// 配置姿态解算的三角函数近似模式（可选：TRIG_STD/TRIG_LUT/TRIG_POLY）
	set_trig_approx_mode(TRIG_POLY);

	/* DWT 初始化与帧率计算由 main 中的 vDWTTask 负责 */

	MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x01); // wake, set clock
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2, 0x00);
	/* 采样率：当 DLPF 开启时基准为 1kHz，采样率 = 1000/(1+SMPLRT_DIV)
	 * 目标：尽量与姿态更新频率接近，减少别名效应。
	 */
	{
		uint16_t target_sr = (POSE_UPDATE_HZ > 0) ? POSE_UPDATE_HZ : 100;
		if (target_sr > 1000) target_sr = 1000; // 上限保护
		uint16_t div = (1000 / target_sr);
		if (div == 0) div = 1; // 目标频率>1000时
		/* 采样率 = 1000/(1+div) -> 令 (1+div) ≈ 1000/target_sr */
		if (div > 0) div -= 1;
		if (div > 255) div = 255;
		MPU6050_WriteReg(MPU6050_SMPLRT_DIV, (uint8_t)div);
	}
	MPU6050_WriteReg(MPU6050_CONFIG, 0x03);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG, 0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, 0x00);
	/* 配置 Madgwick 采样频率，与任务循环保持一致 */
	MadgwickAHRS_SetSampleFreq((float)POSE_UPDATE_HZ);

    MPU6050_StartPoseTask();

	MPU6050_SelfCorrect();
}

void MPU6050_GetData(int16_t *pAccX, int16_t *pAccY, int16_t *pAccZ,
					 int16_t *pGyroX, int16_t *pGyroY, int16_t *pGyroZ)
{
	uint8_t raw[14];
	MPU6050_ReadBytes(MPU6050_ACCEL_XOUT_H, raw, 14);

	int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
	int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
	int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

	int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
	int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
	int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

	if (g_selfcorrect_finished) {
		ax -= ax_off; ay -= ay_off; az -= az_off;
		gx -= gx_off; gy -= gy_off; gz -= gz_off;
	}

	if (pAccX) *pAccX = ax;
	if (pAccY) *pAccY = ay;
	if (pAccZ) *pAccZ = az;
	if (pGyroX) *pGyroX = gx;
	if (pGyroY) *pGyroY = gy;
	if (pGyroZ) *pGyroZ = gz;
}

static void MPU6050_SelfCorrectionProgressShow(float prgs)
{
	if (prgs < 1.0f) {
		OLED_ShowNum((prgs < 0.1f ? 58 : 50), 26, (int)(prgs * 100), (prgs < 0.1f ? 1 : 2), OLED_8X16);
		OLED_ShowString(70, 26, "%", OLED_8X16);
		OLED_UpdateArea(26, 26, 70, 8);
	} else {
		OLED_UpdateArea(26, 26, 70, 8);
		OLED_ShowString(45, 26, "fine", OLED_8X16);
		OLED_UpdateArea(26, 26, 70, 8);
		Delay_ms(700);
	}
}

void MPU6050_SelfCorrect(void)
{
	if (g_selfcorrect_finished) return;

	const int warmup = 200;
	const int samples = 3000;
	int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
	int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;

	const int16_t acc_jump = 2000;
	const int16_t gyro_jump = 1500;

	int16_t prev_ax = 0, prev_ay = 0, prev_az = 0;
	int16_t prev_gx = 0, prev_gy = 0, prev_gz = 0;

	for (int i = 0; i < warmup + samples; i++) {
		uint8_t raw[14];
		MPU6050_ReadBytes(MPU6050_ACCEL_XOUT_H, raw, 14);

		int16_t ax = (int16_t)((raw[0] << 8) | raw[1]);
		int16_t ay = (int16_t)((raw[2] << 8) | raw[3]);
		int16_t az = (int16_t)((raw[4] << 8) | raw[5]);

		int16_t gx = (int16_t)((raw[8] << 8) | raw[9]);
		int16_t gy = (int16_t)((raw[10] << 8) | raw[11]);
		int16_t gz = (int16_t)((raw[12] << 8) | raw[13]);

		if (i < warmup) {
			prev_ax = ax; prev_ay = ay; prev_az = az;
			prev_gx = gx; prev_gy = gy; prev_gz = gz;
		} else {
			if ((abs(ax - prev_ax) > acc_jump) || (abs(ay - prev_ay) > acc_jump) || (abs(az - prev_az) > acc_jump)
				|| (abs(gx - prev_gx) > gyro_jump) || (abs(gy - prev_gy) > gyro_jump) || (abs(gz - prev_gz) > gyro_jump)) {
				prev_ax = ax; prev_ay = ay; prev_az = az;
				prev_gx = gx; prev_gy = gy; prev_gz = gz;
			} else {
				ax_sum += ax; ay_sum += ay; az_sum += az;
				gx_sum += gx; gy_sum += gy; gz_sum += gz;
				prev_ax = ax; prev_ay = ay; prev_az = az;
				prev_gx = gx; prev_gy = gy; prev_gz = gz;
			}
		}

		if (i >= warmup && ((i - warmup) % 100 == 0)) {
			float pgrs = (float)(i - warmup + 1) / (float)samples;
			MPU6050_SelfCorrectionProgressShow(pgrs);
		}
	}

	OLED_Clear();

	const int16_t GRAVITY_LSB_1G = 16384;
	int16_t ax_avg = (int16_t)(ax_sum / samples);
	int16_t ay_avg = (int16_t)(ay_sum / samples);
	int16_t az_avg = (int16_t)(az_sum / samples);

	ax_off = (int16_t)(ax_avg - 0);
	ay_off = (int16_t)(ay_avg - 0);
	az_off = (int16_t)(az_avg - GRAVITY_LSB_1G);

	gx_off = (int16_t)(gx_sum / samples);
	gy_off = (int16_t)(gy_sum / samples);
	gz_off = (int16_t)(gz_sum / samples);

	g_selfcorrect_finished = 1;
}

static void vPoseUpdateTask(void *pvParameters)
{
    (void)pvParameters;
	TickType_t last = xTaskGetTickCount();
	TickType_t period = (configTICK_RATE_HZ / POSE_UPDATE_HZ);
	if (period == 0) period = 1; // 保证至少 1 tick
    
	for (;;)
	{
        if(!g_selfcorrect_finished) {
            vTaskDelay(period);
            continue;
        }

        MPU6050_GetData(&AccX, &AccY, &AccZ, &GyroX, &GyroY, &GyroZ);

        // 如果陀螺是度/秒，请先转换：
        const float DEG2RAD = 3.14159265358979323846f / 180.0f;
        // MPU6050 陀螺仪在±2000°/s量程下的灵敏度为 16.4 LSB/(°/s)
        float gx_rad = (GyroX / 16.4f) * DEG2RAD;
        float gy_rad = (GyroY / 16.4f) * DEG2RAD;
        float gz_rad = (GyroZ / 16.4f) * DEG2RAD;

        MadgwickAHRSupdateIMU(gx_rad, gy_rad, gz_rad, AccX, AccY, AccZ);
        // 来自 MadgwickAHRS.c 的四元数转欧拉角函数原型
        quat_to_euler_zyx(q0, q1, q2, q3, &yaw, &pitch, &roll);

        //
        const float RAD2DEG = 180.0f / 3.14159265358979323846f;
        pitch *= RAD2DEG;
        roll  *= RAD2DEG;
        yaw   *= RAD2DEG;

		/* 写入共享最新姿态值，帧率/周期(ms)由 main 中的 vDWTTask 更新 */
		Comm_pitch = pitch;
		Comm_roll = roll;
		Comm_yaw = yaw;

		vTaskDelayUntil(&last, period);

    }
}

void MPU6050_StartPoseTask(void)
{
	const uint16_t stackDepth = 512; // words
	const UBaseType_t priority = tskIDLE_PRIORITY + 2;
	(void)xTaskCreate(vPoseUpdateTask, "Pose", stackDepth, NULL, priority, NULL);
}
