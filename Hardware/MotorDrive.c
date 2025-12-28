#include "MotorDrive.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

/* STBY 控制辅助（ENABLE=高，DISABLE=低） */
void STBY_cmd(FunctionalState NewState)
{
    if (NewState == ENABLE)
    {
        /* 置高：写 BSRR 低 16 位 */
        STBY_GPIO->BSRR = STBY_PIN;
    }
    else
    {
        /* 置低：写 BSRR 高 16 位 */
        STBY_GPIO->BSRR = (uint32_t)STBY_PIN << 16;
    }
}

Motor_Status motor_status[2] =
{
    {.Mode = Motor_Mode_break, .SpeedInPercent = 0.0f},
    {.Mode = Motor_Mode_break, .SpeedInPercent = 0.0f}
};

void MD_Init(void)
{
    /* PWM 输出：使用 TIM1_CH1 (PA8) 与 TIM1_CH4 (PA11) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // 如需释放调试引脚，可禁用 JTAG 保留 SWD
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    GPIO_InitTypeDef GPIO_IS;
    // PWMA: PA8 -> TIM1_CH1
    GPIO_IS.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_IS.GPIO_Pin = PWMA_PIN;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWMA_GPIO, &GPIO_IS);

    // PWMB: PA11 -> TIM1_CH4
    GPIO_IS.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_IS.GPIO_Pin = PWMB_PIN;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(PWMB_GPIO, &GPIO_IS);

    TIM_TimeBaseInitTypeDef TIM_TBIS;
    TIM_TBIS.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TBIS.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TBIS.TIM_Period = PWM_ARR;                 // 当前 ARR=2879（约25kHz）
    TIM_TBIS.TIM_Prescaler = PWM_PSC_TIM1;         // 72MHz/((PSC+1)*(ARR+1)) ≈ 25kHz（PSC=0, ARR=2879）
    TIM_TBIS.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TBIS);

    TIM_OCInitTypeDef TIM_OCIS;
    TIM_OCStructInit(&TIM_OCIS);
    TIM_OCIS.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCIS.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCIS.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCIS.TIM_Pulse = 0;
    TIM_OC1Init(TIM1, &TIM_OCIS);                  // TIM1_CH1
    TIM_OC4Init(TIM1, &TIM_OCIS);                  // TIM1_CH4

    TIM_Cmd(TIM1, ENABLE);
    // 高级定时器需要显式使能主输出
    TIM_CtrlPWMOutputs(TIM1, ENABLE);


    /* 电机模式输出：改为使用 GPIOB 引脚 */
    // 电机1模式：AIN1 -> PB13, AIN2 -> PB12
    GPIO_IS.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_IS.GPIO_Pin = AIN1_PIN | AIN2_PIN;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(AIN1_GPIO, &GPIO_IS);

    // 电机2模式：BIN1 -> PB14, BIN2 -> PB15
    GPIO_IS.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_IS.GPIO_Pin = BIN1_PIN | BIN2_PIN;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BIN1_GPIO, &GPIO_IS);
    
    Motor_SetMode(1, Motor_Mode_stop);                 //初始化均为高电平 制动模式
    Motor_SetMode(2, Motor_Mode_stop);

    /* STBY 控制：PB5 */
    GPIO_IS.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_IS.GPIO_Pin = STBY_PIN;
    GPIO_IS.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(STBY_GPIO, &GPIO_IS);

    STBY_cmd(DISABLE);


}

/**
    * 函    数：PWM1设置CCR
    * 参    数：Compare 要写入的CCR的值，范围：0~PWM_ARR（当前 ARR=2879，约25kHz）
    * 返 回 值：无
    * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
    *           占空比Duty = CCR / (ARR + 1)
    */
void Motor1_SetCompare(uint16_t Compare)
{
    /* 直写 CCR 提升效率 */
    TIM1->CCR1 = Compare;
}

/**
    * 函    数：PWM2设置CCR
    * 参    数：Compare 要写入的CCR的值，范围：0~PWM_ARR（当前 ARR=2879，约25kHz）
    * 返 回 值：无
    * 注意事项：CCR和ARR共同决定占空比，此函数仅设置CCR的值，并不直接是占空比
    *           占空比Duty = CCR / (ARR + 1)
    */
void Motor2_SetCompare(uint16_t Compare)
{
    /* 直写 CCR 提升效率 */
    TIM1->CCR4 = Compare;
}

void Motor_SetMode(int numMotor, enum Motor_Mode Mode)
{
    if(numMotor == 1){
        switch (Mode)
        {
        case Motor_Mode_break:
            /* AIN1=1, AIN2=1 */
            AIN1_GPIO->BSRR = (uint32_t)AIN1_PIN | (uint32_t)AIN2_PIN;
            break;
        case Motor_Mode_stop:
            /* AIN1=0, AIN2=0 */
            AIN1_GPIO->BSRR = ((uint32_t)AIN1_PIN << 16) | ((uint32_t)AIN2_PIN << 16);
            break;
        case Motor_Mode_frd_rotation:
            /* AIN1=1, AIN2=0 */
            AIN1_GPIO->BSRR = (uint32_t)AIN1_PIN | ((uint32_t)AIN2_PIN << 16);
            break;
        case Motor_Mode_rvs_rotation:
            /* AIN1=0, AIN2=1 */
            AIN1_GPIO->BSRR = ((uint32_t)AIN1_PIN << 16) | (uint32_t)AIN2_PIN;
            break;
        default:
            break;
        }
        motor_status[0].Mode = Mode;
    }

    else if(numMotor == 2){
        switch (Mode)
                {
        case Motor_Mode_break:
            /* BIN1=1, BIN2=1 */
            BIN1_GPIO->BSRR = (uint32_t)BIN1_PIN | (uint32_t)BIN2_PIN;
            break;
        case Motor_Mode_stop:
            /* BIN1=0, BIN2=0 */
            BIN1_GPIO->BSRR = ((uint32_t)BIN1_PIN << 16) | ((uint32_t)BIN2_PIN << 16);
            break;
        case Motor_Mode_frd_rotation:
            /* BIN1=1, BIN2=0 */
            BIN1_GPIO->BSRR = (uint32_t)BIN1_PIN | ((uint32_t)BIN2_PIN << 16);
            break;
        case Motor_Mode_rvs_rotation:
            /* BIN1=0, BIN2=1 */
            BIN1_GPIO->BSRR = ((uint32_t)BIN1_PIN << 16) | (uint32_t)BIN2_PIN;
            break;
        default:
            break;
        }
        motor_status[1].Mode = Mode;
    }
}
/**
    * 函    数：直流电机设置PWM
    * 参    数：index 选择设置哪个电机 1或2
    * 参    数：PWM 要设置的PWM值，范围：-10000~10000（负数为反转），实际饱和到 ARR
    * 返 回 值：无
    */
void Motor_SetPWM(uint8_t index, int16_t PWM)
{
    /* 简洁高效：单分支处理方向，饱和到 [0..PWM_ARR]，减少重复 */
    uint16_t duty;

    if (PWM >= 0) {
        Motor_SetMode(index, Motor_Mode_frd_rotation);
        duty = (PWM > PWM_ARR) ? PWM_ARR : (uint16_t)PWM;
    } else {
        Motor_SetMode(index, Motor_Mode_rvs_rotation);
        int32_t absPWM = -(int32_t)PWM; // 防止 INT16_MIN 溢出
        duty = (absPWM > PWM_ARR) ? PWM_ARR : (uint16_t)absPWM;
    }

    if (index == 1) {
        Motor1_SetCompare(duty);
    } else if (index == 2) {
        Motor2_SetCompare(duty);
    }
}

/**
 * 函    数：按速度百分比设置转速
 * 参    数：index 选择设置哪个电机 1或2
 * 参    数：percent 速度百分比（-100..100；负数为反转）
 * 返 回 值：无
 */
void Motor_SetSpeedPercent(uint8_t index, float percent)
{
    /* 夹紧到 -100..100（支持小数） */
    if (percent > 100.0f) percent = 100.0f;
    if (percent < -100.0f) percent = -100.0f;

    /* 记录状态 */
    if (index == 1) {
        motor_status[0].SpeedInPercent = percent;
    } else if (index == 2) {
        motor_status[1].SpeedInPercent = percent;
    }

    /* 将百分比转换为百分数的整数（保留两位小数），再用整数映射到占空比 */
    int32_t p100 = (int32_t)(percent * 100.0f + (percent >= 0.0f ? 0.5f : -0.5f));
    if (p100 > 10000) p100 = 10000;
    if (p100 < -10000) p100 = -10000;

    uint32_t abs100 = (p100 >= 0) ? (uint32_t)p100 : (uint32_t)(-p100);
    /* 将百分比均匀映射到 0..1439（包含 1439），使用整数四舍五入 */
    const uint32_t PWM_MAP_MAX = 1439U;
    uint16_t duty = (uint16_t)((abs100 * PWM_MAP_MAX + 5000U) / 10000U);

    /* 复用整数 PWM 接口，统一方向与比较寄存器设置 */
    int16_t pwm_value = (p100 >= 0) ? (int16_t)duty : -(int16_t)duty;
    Motor_SetPWM(index, pwm_value);
}
