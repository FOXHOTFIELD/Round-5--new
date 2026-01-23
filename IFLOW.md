# STM32F103C8 自平衡机器人项目

## 项目概述

这是一个基于 STM32F103C8（Blue Pill）微控制器的自平衡机器人项目，使用 FreeRTOS 实时操作系统进行任务调度。项目实现了双电机驱动、MPU6050 姿态传感器、编码器反馈、PID 控制以及 OLED 显示等功能。

### 主要技术栈

- **微控制器**: STM32F103C8 (Cortex-M3, 72MHz)
- **开发环境**: Keil uVision 5 (ARM Compiler 6)
- **实时操作系统**: FreeRTOS (heap_4 内存管理)
- **外设库**: STM32F10x 标准外设库
- **传感器**: MPU6050 (6轴 IMU, I2C 接口)
- **执行器**: 双直流电机 + 编码器
- **显示**: 0.96寸 OLED (I2C 接口)
- **通信**: 串口 (USART) 用于遥测和控制

### 核心功能

1. **姿态控制**: 使用 MPU6050 + Madgwick AHRS 算法计算俯仰角、横滚角、偏航角
2. **双环 PID 控制**: 姿态环（外环）+ 速度环（内环）实现自平衡
3. **电机控制**: PWM 输出 + 方向控制 + 编码器反馈
4. **人机交互**: 菜单系统 + 按键 + OLED 显示
5. **串口通信**: 实时遥测姿态、速度、PID 参数等数据
6. **性能监控**: 使用 DWT 计数器测量实际帧率和周期

## 项目结构

```
Round 5 -new/
├── Start/              # 启动文件和 CMSIS 核心
├── Library/            # STM32F10x 标准外设库
├── User/               # 用户代码
│   ├── main.c          # 主程序入口
│   ├── FreeRTOSConfig.h  # FreeRTOS 配置
│   └── stm32f10x_it.c  # 中断服务程序
├── System/             # 系统模块
│   ├── myHeader.h      # 主头文件（包含所有模块）
│   ├── Delay.c/h       # 延时函数
│   ├── Menu.c/h        # 菜单系统
│   ├── comm.c/h        # 通信模块
│   ├── MadgwickAHRS.c/h # 姿态解算算法
│   └── filter.c/h      # 数字滤波器
├── Hardware/           # 硬件驱动
│   ├── MotorDrive.c/h  # 电机驱动
│   ├── Encoder.h       # 编码器接口
│   ├── MPU6050.c/h     # MPU6050 驱动
│   ├── OLED.c/h        # OLED 显示驱动
│   ├── MenuKeys.c/h    # 按键驱动
│   ├── Serial.c/h      # 串口驱动
│   ├── SoftI2C.c/h     # 软件 I2C
│   └── PID.c/h         # PID 控制器
└── MiddleWares/        # FreeRTOS 内核
    ├── include/        # FreeRTOS 头文件
    └── portable/       # 移植层
```

## 构建和运行

### 开发环境要求

- Keil uVision 5.0 或更高版本
- ARM Compiler 6.7 或更高版本
- STM32F1xx_DFP 设备包 (版本 2.4.1)

### 编译步骤

1. 打开 `Project.uvprojx` 项目文件
2. 选择目标 "Target 1"
3. 点击 "Build" (F7) 或使用菜单 Project -> Build Target

### 编译输出

- 输出目录: `./Objects/`
- 可执行文件: `Project.axf`
- 默认不生成 HEX 文件 (如需生成，请在项目设置中启用)

### 调试和烧录

1. 连接 ST-Link 调试器到开发板
2. 点击 "Download" (F8) 烧录程序
3. 点击 "Start/Stop Debug Session" (Ctrl+F5) 进入调试模式

### 运行要求

- STM32F103C8 开发板
- 供电: 5V (USB 或外部电源)
- 外设连接:
  - MPU6050: I2C (PB6=SCL, PB7=SDA)
  - OLED: I2C (PB6=SCL, PB7=SDA)
  - 电机驱动: TIM1 PWM + GPIO 控制
  - 编码器: TIM2/TIM3
  - 串口: USART1 (PA9=TX, PA10=RX)

## 开发约定

### 编码规范

1. **文件命名**: 
   - 头文件使用 `.h` 后缀
   - 源文件使用 `.c` 后缀
   - 驱动文件以功能命名 (如 `MotorDrive.c`)

2. **命名约定**:
   - 宏定义使用全大写加下划线 (如 `PID_UPDATE_MS`)
   - 类型定义使用 `_t` 后缀 (如 `PID_Config_t`)
   - 函数名使用模块前缀 (如 `PID_Init()`)
   - 全局变量使用 `g_` 前缀 (如 `g_pid_loop_counter`)

3. **注释风格**:
   - 函数注释包含功能描述、参数、返回值
   - 关键算法添加详细注释
   - 使用中文注释便于理解

### FreeRTOS 任务配置

项目使用以下 FreeRTOS 任务:

| 任务名称 | 优先级 | 栈大小 | 功能 |
|---------|--------|--------|------|
| PIDTask | tskIDLE_PRIORITY+2 | 160 | PID 控制循环 (1ms) |
| CommTask | tskIDLE_PRIORITY+2 | 196 | 串口通信 (50ms) |
| DWTTask | tskIDLE_PRIORITY+3 | 128 | 性能监控 |
| MenuTask | tskIDLE_PRIORITY+1 | 192 | 菜单处理 |
| MenuShowTask | tskIDLE_PRIORITY+1 | 128 | OLED 显示 |
| 自校正任务 | tskIDLE_PRIORITY+2 | 128 | MPU6050 校准 |
| 姿态更新任务 | tskIDLE_PRIORITY+2 | 256 | 姿态解算 (200Hz) |

### 内存配置

- **总堆大小**: 12KB (configTOTAL_HEAP_SIZE)
- **最小栈大小**: 128 words (configMINIMAL_STACK_SIZE)
- **Tick 频率**: 1000Hz
- **优先级组**: NVIC_PriorityGroup_4 (4位抢占优先级)

### 中断优先级

- FreeRTOS API 调用最高优先级: 5 (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY)
- 最低中断优先级: 15 (configLIBRARY_LOWEST_INTERRUPT_PRIORITY)

### 关键参数

| 参数 | 值 | 说明 |
|-----|---|------|
| CPU 时钟 | 72MHz | SystemCoreClock |
| PWM 频率 | ~25kHz | TIM1, ARR=2879, PSC=0 |
| 姿态更新频率 | 200Hz | POSE_UPDATE_HZ |
| PID 更新周期 | 1ms | PID_UPDATE_MS |
| 通信上传间隔 | 50ms | COMM_UPLOAD_INTERVAL_MS |

### PID 控制配置

**速度环 (PID_Config_Speed)**:
- Kp = 1.2
- Ki = 0.005
- Kd = 1.6
- 前馈增益 = 0.65
- 反向衰减 = 0.6

**姿态环 (PID_Config_Attitude)**:
- Kp = 3.0
- Ki = 0.02
- Kd = 0.0
- 前馈增益 = 0.5
- 反向衰减 = 0.0

### 串口通信协议

使用自定义的 JustFloat 协议发送 17 个浮点数:

| 索引 | 数据 | 说明 |
|-----|------|------|
| 0 | pitch | 俯仰角 (度) |
| 1 | roll | 横滚角 (度) |
| 2 | yaw | 偏航角 (度) |
| 3 | encoder_a | 编码器 A 速度 (%) |
| 4 | out_a | 电机 A 输出 (%) |
| 5 | encoder_b | 编码器 B 速度 (%) |
| 6 | out_b | 电机 B 输出 (%) |
| 7-9 | Kp/Ki/Kd | 姿态环 PID 参数 |
| 10-11 | target_a/b | 速度环目标值 |
| 12 | pose_tag | 姿态标签 |
| 13 | fps | 实际帧率 (Hz) |
| 14 | dt_ms | 实际周期 (ms) |
| 15 | ff_gain | 前馈增益 |
| 16 | rev_decay | 反向衰减系数 |

### 测试模式

项目支持两种编译模式:

1. **正常模式** (默认): 完整的自平衡控制
2. **测试模式** (定义 `TEST` 宏): 
   - 仅测试电机和编码器
   - 速度斜坡控制 (0-100% in 10s)
   - OLED 显示滤波后的速度
   - 串口发送速度数据

在 `main.c` 中通过 `#define TEST` 切换模式。

### 调试技巧

1. **性能监控**: 使用 DWT 计数器测量任务执行时间
2. **栈溢出检测**: 启用 `configCHECK_FOR_STACK_OVERFLOW = 2`
3. **堆使用监控**: 使用 `xPortGetFreeHeapSize()` 检查剩余堆空间
4. **串口调试**: 通过串口实时查看姿态和控制参数

### 注意事项

1. **中断安全**: 在中断中调用 FreeRTOS API 时使用 `FromISR` 版本
2. **临界区**: 访问共享变量时使用 `taskENTER_CRITICAL()` 保护
3. **栈大小**: 根据任务实际需求调整栈大小，避免栈溢出
4. **优先级**: 合理设置任务优先级，避免优先级反转
5. **内存泄漏**: 动态分配的内存需要及时释放

## 常见问题

### 编译错误

- **找不到头文件**: 检查项目设置中的 Include Path
- **链接错误**: 确保所有源文件都已添加到项目中
- **栈溢出**: 增加任务栈大小或减少局部变量

### 运行时问题

- **电机不转**: 检查 STBY 引脚状态和 PWM 配置
- **姿态不稳定**: 调整 PID 参数或检查 MPU6050 校准
- **串口无数据**: 检查波特率和串口初始化
- **OLED 不显示**: 检查 I2C 连接和初始化顺序

### 性能优化

1. **减少浮点运算**: 使用定点数或查表法
2. **优化 DMA**: 使用 DMA 传输数据减少 CPU 占用
3. **降低采样率**: 根据实际需求调整姿态更新频率
4. **代码优化**: 使用 `inline` 和 `const` 关键字

## 扩展建议

1. **添加位置环**: 实现位置控制功能
2. **无线通信**: 添加蓝牙或 WiFi 模块
3. **数据记录**: 使用 SD 卡记录运行数据
4. **参数调节**: 通过串口在线调节 PID 参数
5. **故障检测**: 添加看门狗和异常保护机制

## 相关文档

- [STM32F103 参考手册](https://www.st.com/resource/en/reference_manual/cd00171190.pdf)
- [FreeRTOS 官方文档](https://www.freertos.org/)
- [MPU6050 数据手册](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [Madgwick AHRS 算法](https://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/)

## 版本历史

- Round 5: 当前版本，完整的自平衡控制系统
- 支持双环 PID 控制
- 支持实时遥测
- 支持菜单系统
- 支持性能监控