#ifndef __stm32f10x_Delay_H
#define	__stm32f10x_Delay_H

#include "stm32f10x.h"

void SysTick_Init(void);//delay��ʱ������ʼ��
void Delay_10us(uint32_t nTime); // nTime*10us ��ʱ���� �������Ϊ0~2^24(0~16777216)
uint32_t micros(void);
void Initial_micros(void);

#endif  
