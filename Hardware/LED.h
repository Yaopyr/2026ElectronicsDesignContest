#ifndef __LED_H
#define __LED_H
/* ⚠ 预留模块：板载 LED 驱动（PA1/PA2 低电平点亮），当前未被主程序调用 */
void LED_Init(void);
void LED1_ON(void);
void LED1_OFF(void);
void LED1_Turn(void);
void LED2_ON(void);
void LED2_OFF(void);
void LED2_Turn(void);

#endif
