#ifndef __BSP_SPI_H
#define __BSP_SPI_H
#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4xx.h"
#include <stdio.h>

#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"

void SPI1_Init(void);			 //��ʼ��SPI1��
void SPI1_SetSpeed(uint8_t SpeedSet); //����SPI1�ٶ�   
uint8_t SPI1_ReadWriteByte(uint8_t TxData);//SPI1���߶�дһ���ֽ�

#endif
