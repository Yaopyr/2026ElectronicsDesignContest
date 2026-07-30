#include "stm32f10x.h"

/**
  * 函    数：循迹传感器初始化
  * 参    数：无
  * 返 回 值：无
  * 说    明：配置8路传感器为输入上拉，PB3/PB4/PA15需禁用JTAG保留SWD
  */
void Tracking_Init(void)
{
	/* 禁用JTAG，保留SWD（释放PB3、PB4、PA15）*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

	/* 开启GPIO时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* GPIOA输入：PA5(S0)、PA3(S1)、PA2(S3)、PA4(S5)、PA15(S7) */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIOB输入：PB3(S2)、PB1(S4)、PB4(S6)、PB15(启动) */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_15;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/**
  * 函    数：读取8路传感器值
  * 参    数：无
  * 返 回 值：8位数据，bit0=S0, bit1=S1, ..., bit7=S7，1=检测到黑线
  */
uint8_t Tracking_Read(void)
{
	uint8_t val = 0;
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5)  == 0) val |= 0x01;	// S0
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_3)  == 0) val |= 0x02;	// S1
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3)  == 0) val |= 0x04;	// S2
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_2)  == 0) val |= 0x08;	// S3
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1)  == 0) val |= 0x10;	// S4
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4)  == 0) val |= 0x20;	// S5
	if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4)  == 0) val |= 0x40;	// S6
	if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0) val |= 0x80;	// S7
	return val;
}

/**
  * 函    数：获取检测到黑线的传感器数量
  * 参    数：无
  * 返 回 值：黑线传感器个数（0~8）
  */
uint8_t Tracking_GetBlackCount(void)
{
	uint8_t val = Tracking_Read();
	uint8_t cnt = 0;
	while (val)
	{
		cnt++;
		val &= val - 1;
	}
	return cnt;
}

/**
  * 函    数：等待启动信号
  * 参    数：无
  * 返 回 值：无
  * 说    明：阻塞等待PB15接收到低电平
  */
void Tracking_WaitStart(void)
{
	while (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 1);
}
