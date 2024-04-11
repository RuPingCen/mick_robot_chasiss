#ifndef __MOTOR_EULER_H
#define	__MOTOR_EULER_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

 

 
/** 
* @brief  remote control information
*/

void MOTOR_EULER_Init(void);
 
void MOTOR_EULER_Set_Speed(int16_t Speed1,int16_t Speed2,uint8_t FuncByte);

 

#ifdef __cplusplus
}
#endif

#endif
