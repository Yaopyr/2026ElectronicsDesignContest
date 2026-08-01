#include "UART.h"
#include <stdlib.h>

/* ============================================================================
 * UART.c — USART1 接收 K230 视觉模块的球位置
 *   接线：PA9=TX, PA10=RX；115200-8N1
 *   帧格式："%d\r\n"（ASCII 有符号整数，范围 ±1000，对应 K230 ROI 左右两端）
 *   RXNE 中断逐字节收进 8 字节缓冲，遇 '\n' 用 atoi 解析存入 g_ball_pos
 *   UART_GetBallPosition() 无新帧时返回上一次的值
 * ========================================================================== */

static volatile int16_t g_ball_pos = 0;	// 最新小球位置

/**
  * @brief  USART1 初始化：PA9=TX, PA10=RX, 115200-8N1, RX中断
  */
void UART_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	/* PA9=TX 复用推挽 */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* PA10=RX 浮空输入 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USART1: 115200-8N1 */
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* RXNE 中断使能 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

int16_t UART_GetBallPosition(void)
{
	return g_ball_pos;
}

/**
  * @brief  USART1 RX 中断：接收 "%d\r\n" 格式数据
  */
void USART1_IRQHandler(void)
{
	static char rx_buf[8];
	static uint8_t rx_idx = 0;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		char c = USART_ReceiveData(USART1);

		if (c == '\n')
		{
			rx_buf[rx_idx] = '\0';
			g_ball_pos = (int16_t)atoi(rx_buf);
			rx_idx = 0;
		}
		else if (c != '\r' && rx_idx < sizeof(rx_buf) - 1)
		{
			rx_buf[rx_idx++] = c;
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
