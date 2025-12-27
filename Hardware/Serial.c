#include "MyHeader.h"
#include "Serial.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "stm32f10x_dma.h"

// 接收缓冲区和标志位
uint8_t Serial_RxBuffer[100];
uint16_t Serial_RxLength;
uint8_t Serial_RxFlag;

/**
  * @brief  串口初始化
  * @param  无
  * @retval 无
  */
void Serial_Init(void)
{
	// 开启时钟（USART2 在 APB1）
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	
	// GPIO初始化
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;		// PA2 TX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;		// PA3 RX
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	// USART初始化
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART2, &USART_InitStructure);
	
	// DMA TX 初始化（USART2_TX -> DMA1_Channel7）
	DMA_InitTypeDef DMA_InitStructure;
	DMA_DeInit(DMA1_Channel7);
	DMA_InitStructure.DMA_BufferSize = 0;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_InitStructure.DMA_MemoryBaseAddr = 0;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
	DMA_Init(DMA1_Channel7, &DMA_InitStructure);
	
	// DMA RX 初始化（USART2_RX -> DMA1_Channel6）
	DMA_DeInit(DMA1_Channel6);
	DMA_InitStructure.DMA_BufferSize = 100;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)Serial_RxBuffer;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;
	DMA_Init(DMA1_Channel6, &DMA_InitStructure);
	DMA_Cmd(DMA1_Channel6, ENABLE);
	
	// 使能USART DMA
	USART_DMACmd(USART2, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);
	
	// 使能USART2 IDLE中断
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
	
	// 配置NVIC
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);
	
	// 使能USART
	USART_Cmd(USART2, ENABLE);
}

/**
  * @brief  串口发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART2, Byte);
	while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET);
}

/**
  * @brief  串口发送一个数组
  * @param  Array 要发送数组的首地址
  * @param  Length 要发送数组的长度
  * @retval 无
  */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	DMA_Cmd(DMA1_Channel7, DISABLE);
	DMA1_Channel7->CNDTR = Length;
	DMA1_Channel7->CMAR = (uint32_t)Array;
	DMA_Cmd(DMA1_Channel7, ENABLE);
	while(DMA_GetFlagStatus(DMA1_FLAG_TC7) == RESET);
	DMA_ClearFlag(DMA1_FLAG_TC7);
}

/**
  * @brief  串口发送一个字符串
  * @param  String 要发送字符串的首地址
  * @retval 无
  */
void Serial_SendString(char *String)
{
	Serial_SendArray((uint8_t *)String, strlen(String));
}

/**
  * @brief  次方函数（内部使用）
  * @retval 返回值等于X的Y次方
  */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  串口发送数字
  * @param  Number 要发送的数字，范围：0~4294967295
  * @param  Length 要发送数字的长度，范围：0~10
  * @retval 无
  */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	uint8_t Array[10];
	for (i = 0; i < Length; i ++)
	{
		Array[i] = Number / Serial_Pow(10, Length - i - 1) % 10 + '0';
	}
	Serial_SendArray(Array, Length);
}

/**
  * @brief  使用printf需要重定向的底层函数
  * @param  保持原始格式即可，无需变动
  * @retval 保持原始格式即可，无需变动
  */
int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

/**
  * @brief  自己封装的prinf函数
  * @param  format 格式化字符串
  * @param  ... 可变的参数列表
  * @retval 无
  */
void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

/**
  * @brief  获取串口接收标志位
  * @param  无
  * @retval 串口接收标志位，范围：0~1，接收到数据后，标志位置1，读取后标志位自动清零
  */
uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1)
	{
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

/**
  * @brief  获取串口接收的数据
  * @param  无
  * @retval 接收的数据，范围：0~255
  */
uint8_t Serial_GetRxData(void)
{
	return Serial_RxBuffer[0];
}

/**
  * @brief  串口发送VOFA+ JustFloat协议格式数据
  * @param  data 浮点型数组的首地址
  * @param  num 浮点型数组的长度（通道数量）
  * @retval 无
  */
void Serial_SendJustFloat(float *data, uint16_t num)
{
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
    Serial_SendArray((uint8_t *)data, num * 4);
    Serial_SendArray(tail, 4);
}

/**
  * @brief  USART1中断函数
  * @param  无
  * @retval 无
  * @note   此函数为中断函数，无需调用，中断触发后自动执行
  *         函数名为预留的指定名称，可以从启动文件复制
  *         请确保函数名正确，不能有任何差异，否则中断函数将不能进入
  */
void USART2_IRQHandler(void)
{
	if (USART_GetITStatus(USART2, USART_IT_IDLE) == SET)
	{
		Serial_RxLength = 100 - DMA_GetCurrDataCounter(DMA1_Channel6);
		Serial_RxFlag = 1;
		USART_ReceiveData(USART2); // 清除IDLE标志
		USART_ClearITPendingBit(USART2, USART_IT_IDLE);
	}
}

/**
  * @brief  按特定格式发送数据包
  * @param  msg 要发送的字符串内容（不包含包头和包尾）
  * @retval 无
  * @note   接收端使用的格式为 "@MSG\r\n"，因此发送端需按如下顺序组帧：
  *         '@' + msg(原样发送) + '\r' + '\n'
  *         为避免过长导致接收缓冲溢出，建议 msg 长度不超过 99 字节
  */
void Serial_SendPacket(const char *msg)
{
    uint8_t packet[102]; // '@' + msg + '\r\n'
    uint16_t len = strlen(msg);
    packet[0] = '@';
    memcpy(&packet[1], msg, len);
    packet[1 + len] = '\r';
    packet[1 + len + 1] = '\n';
    Serial_SendArray(packet, len + 3);
}


