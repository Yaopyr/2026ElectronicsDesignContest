#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值（72MHz HCLK，1 计数 = 1/72µs）
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//0x05 = 时钟源 HCLK(bit2) + 使能(bit0)，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到 0（0x10000 = COUNTFLAG 计数结束标志）
	SysTick->CTRL = 0x00000004;				//关闭定时器（仅清使能位，保留 HCLK 时钟源设置）
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
