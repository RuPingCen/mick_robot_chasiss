#ifndef __BSP_CAN_H
#define	__BSP_CAN_H

#include "stm32f10x.h"
#include "stm32f10x_can.h"



static void CAN_GPIO_Config(void);
static void CAN_NVIC_Config(void);
static void CAN_Mode_Config(void);
static void CAN_Filter_Config(void);
void CAN_Config(void);
void CAN_SetMsg(void);


#endif
