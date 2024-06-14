#ifndef __BSP_ADC_H
#define	__BSP_ADC_H

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include <stdio.h>
#include "string.h"





void MY_ADC_DMA_Init(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
unsigned int MY_ADC_Value_Get(ADC_TypeDef* ADCx, uint8_t ADC_Channel);
                            
#endif
