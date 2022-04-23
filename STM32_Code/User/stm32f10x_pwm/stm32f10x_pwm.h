#ifndef __STM32F10x_PWM_H
#define	__STM32F10x_PWM_H

#include "stm32f10x.h"




 
#define PWM1                                        ((uint8_t)0x00)
#define PWM2                                        ((uint8_t)0x01)
#define PWM3                                        ((uint8_t)0x02)
 

#define PWM_Channel_0                               ((uint8_t)0x00)
#define PWM_Channel_1                               ((uint8_t)0x01)
#define PWM_Channel_2                               ((uint8_t)0x02)
#define PWM_Channel_3                               ((uint8_t)0x03)
 






void PWM_config_Init(u8 u8PWMx , u16 u16channel , u32 u32period , u16 u16duty);
void PWM_Duty_Change(u8 u8PWMx,u16 u16channel , u32 u32period, u16 u16duty);





 

#endif /* __STM32F10x_PWM_H */

