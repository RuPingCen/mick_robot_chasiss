#ifndef __BSP_GPIO_H
#define	__BSP_GPIO_H

#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"


void My_GPIO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mod);
void GPIO_Flip_level(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x);

void My_GPIO_Exit_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x , uint8_t Trigger_mod);




#endif
