#ifndef __BSP_TIMER_H
#define	__BSP_TIMER_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"

void Timer_2to7_Init(TIM_TypeDef* TIMx,u16 u16counter);
void Timer_start(TIM_TypeDef* TIMx);
void Timer_stop(TIM_TypeDef* TIMx);

#endif
