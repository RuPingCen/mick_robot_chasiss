#ifndef __BSP_SYSTICK_H
#define	__BSP_SYSTICK_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"


 
void SysTick_Init(void);//delay延时函数初始化
void Delay_10us(uint32_t nTime); // nTime*10us 延时函数
void Delay_10ms(__IO u32 nTime);
void delay_ms(__IO u32 nTime);// for mpu6050 DMP

void Initial_micros(void) ;
uint32_t micros(void);
#endif
