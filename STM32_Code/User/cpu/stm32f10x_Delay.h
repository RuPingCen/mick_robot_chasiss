#ifndef __stm32f10x_Delay_H
#define	__stm32f10x_Delay_H

#include "stm32f10x.h"

void SysTick_Init(void);//delay延时函数初始化
void Delay_10us(uint32_t nTime); // nTime*10us 延时函数 传入参数为0~2^24(0~16777216)
uint32_t micros(void);
void Initial_micros(void);

#endif  
