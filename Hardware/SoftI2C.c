/* 简单软件I2C(bit-bang)实现，基于STM32F10x 标准外设库
 * SCL: PB10
 * SDA: PB11
 */
#include "softi2c.h"
#include "Delay.h"

#define SCL_PIN GPIO_Pin_10
#define SDA_PIN GPIO_Pin_11
#define SCL_PORT GPIOB
#define SDA_PORT GPIOB

static void SDA_Out(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SDA_PORT, &gpio);
}

static void SDA_In(void)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = SDA_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SDA_PORT, &gpio);
}

static void I2C_Delay(void)
{
    Delay_us(5);
}

void Soft_I2C_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SCL_PIN | SDA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(SCL_PORT, &GPIO_InitStructure);

    GPIO_SetBits(SCL_PORT, SCL_PIN);
    GPIO_SetBits(SDA_PORT, SDA_PIN);
    I2C_Delay();
}

void Soft_I2C_Start(void)
{
    SDA_Out();
    GPIO_SetBits(SDA_PORT, SDA_PIN);
    GPIO_SetBits(SCL_PORT, SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(SDA_PORT, SDA_PIN);
    I2C_Delay();
    GPIO_ResetBits(SCL_PORT, SCL_PIN);
}

void Soft_I2C_Stop(void)
{
    SDA_Out();
    GPIO_ResetBits(SDA_PORT, SDA_PIN);
    I2C_Delay();
    GPIO_SetBits(SCL_PORT, SCL_PIN);
    I2C_Delay();
    GPIO_SetBits(SDA_PORT, SDA_PIN);
    I2C_Delay();
}

void Soft_I2C_SendByte(uint8_t byte)
{
    SDA_Out();
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) GPIO_SetBits(SDA_PORT, SDA_PIN);
        else GPIO_ResetBits(SDA_PORT, SDA_PIN);
        byte <<= 1;
        I2C_Delay();
        GPIO_SetBits(SCL_PORT, SCL_PIN);
        I2C_Delay();
        GPIO_ResetBits(SCL_PORT, SCL_PIN);
        I2C_Delay();
    }
}

uint8_t Soft_I2C_WaitAck(void)
{
    uint8_t ack;
    SDA_In();
    I2C_Delay();
    GPIO_SetBits(SCL_PORT, SCL_PIN);
    I2C_Delay();
    ack = GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN);
    GPIO_ResetBits(SCL_PORT, SCL_PIN);
    SDA_Out();
    return ack; // 0 = ACK, 1 = NACK
}

void Soft_I2C_Ack(void)
{
    SDA_Out();
    GPIO_ResetBits(SDA_PORT, SDA_PIN);
    I2C_Delay();
    GPIO_SetBits(SCL_PORT, SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(SCL_PORT, SCL_PIN);
    I2C_Delay();
    GPIO_SetBits(SDA_PORT, SDA_PIN);
}

void Soft_I2C_NAck(void)
{
    SDA_Out();
    GPIO_SetBits(SDA_PORT, SDA_PIN);
    I2C_Delay();
    GPIO_SetBits(SCL_PORT, SCL_PIN);
    I2C_Delay();
    GPIO_ResetBits(SCL_PORT, SCL_PIN);
    I2C_Delay();
}

uint8_t Soft_I2C_ReadByte(uint8_t ack)
{
    uint8_t i, receive = 0;
    SDA_In();
    for (i = 0; i < 8; i++) {
        receive <<= 1;
        GPIO_SetBits(SCL_PORT, SCL_PIN);
        I2C_Delay();
        if (GPIO_ReadInputDataBit(SDA_PORT, SDA_PIN)) receive |= 0x01;
        GPIO_ResetBits(SCL_PORT, SCL_PIN);
        I2C_Delay();
    }
    if (ack) Soft_I2C_Ack();
    else Soft_I2C_NAck();
    return receive;
}
