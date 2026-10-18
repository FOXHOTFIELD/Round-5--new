# STM32F103C8 自平衡机器人

基于 **STM32F103C8（Blue Pill）+ FreeRTOS** 的两轮自平衡机器人工程：MPU6050 + Madgwick AHRS 姿态解算，姿态环 + 速度环**双闭环 PID**，编码器测速反馈，OLED + 按键菜单，串口实时遥测。

*A two-wheel self-balancing robot on STM32F103C8 with FreeRTOS, MPU6050/Madgwick AHRS and dual-loop PID control.*

> 更详细的设计与调参说明见 [IFLOW.md](IFLOW.md)。

## 技术栈

| 项 | 内容 |
|---|---|
| 微控制器 | STM32F103C8（Cortex-M3，72MHz） |
| 实时系统 | FreeRTOS（heap_4） |
| 外设库 | STM32F10x 标准外设库 |
| 姿态传感器 | MPU6050（6 轴 IMU，软件 I2C） |
| 解算算法 | Madgwick AHRS（四元数）+ 数字滤波 |
| 执行器 | 双直流电机（PWM + 方向）+ 编码器 |
| 显示/交互 | 0.96" OLED（I2C）+ 按键菜单 |
| 通信 | USART 串口遥测与控制 |
| 开发环境 | Keil uVision 5（ARM Compiler 6） |

## 核心功能

1. **姿态解算**：MPU6050 六轴数据 → 滤波 → Madgwick AHRS → 俯仰/横滚/航向角
2. **双环 PID**：姿态环（外环）给定速度、速度环（内环）给 PWM，实现原地平衡
3. **电机驱动**：PWM 输出 + 方向控制 + 编码器脉冲计数测速
4. **人机交互**：按键 + 菜单 + OLED 显示（可现场改 PID 参数）
5. **串口遥测**：实时输出姿态角、速度、PID 参数，便于上位机观察曲线
6. **性能监控**：用 DWT 周期计数器统计实际控制周期与帧率

## 目录结构

```
Round 5 -new/
├── User/            入口与系统配置：main.c、FreeRTOSConfig.h、FreeRTOSHooks.c、中断服务
├── System/          系统级模块：MadgwickAHRS（姿态解算）、filter（滤波）、Menu（菜单）、
│                    comm（通信）、Delay（延时）、myHeader.h（总头文件）
├── Hardware/        硬件驱动：MPU6050、SoftI2C、MotorDrive（电机）、PID、Encoder（编码器）、
│                    OLED、MenuKeys（按键）、Serial（串口）
├── MiddleWares/     FreeRTOS 内核与移植层
├── Library/         STM32F10x 标准外设库
├── Start/           启动文件与 CMSIS
├── Project.uvprojx  Keil 工程
└── IFLOW.md         项目设计与实现说明
```

## 编译与烧录

1. 安装 Keil uVision 5 + **ARM Compiler 6**，并装好 `STM32F1xx_DFP` 器件包
2. 打开 `Project.uvprojx`，选择目标 `Target 1`
3. `F7` 编译（产物在 `Objects/`，默认不生成 HEX，需要时在 Options → Output 勾选 Create HEX File）
4. 连接 ST-Link，`F8` 下载

## 分支说明

默认分支为 **`pose_pid`**（姿态 PID 调参主线）。

## 已知事项

- 编译产物（`Objects/`、`Listings/`、`*.axf`、`*.hex` 等）已通过 `.gitignore` 排除，仓库只保留源码与工程文件；
- `Hardware/Enocder.c` 的文件名拼写与工程引用保持一致（历史原因，未改名以免破坏工程）；
- 代码注释与文档均为中文；
- 仓库未附带 LICENSE，如需开源分发请自行补充。

## 硬件接线（概要）

| 模块 | 接口 |
|---|---|
| MPU6050 | 软件 I2C（SCL/SDA 任意 GPIO） |
| OLED | 软件 I2C，与 MPU6050 可共用总线 |
| 电机驱动 | 两路 PWM + 方向脚 |
| 编码器 | 定时器编码器模式 / 外部中断计数 |
| 串口 | USART1，115200 8N1 |
| 按键 | GPIO 上拉输入 |

具体引脚以 `System/myHeader.h` 与各驱动文件中的宏定义为准。
