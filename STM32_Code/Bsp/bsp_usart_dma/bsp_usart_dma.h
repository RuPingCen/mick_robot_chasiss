#ifndef __BSP_USART_DMA_H
#define __BSP_USART_DMA_H

#include "stm32f4xx.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"
#include "string.h"
#include <stdio.h>




#define USART_MAX_TX_LEN             1024
#define USART_MAX_RX_LEN             40

#define USART2_MAX_TX_LEN            25  
#define USART2_MAX_RX_LEN            25   //大疆遥控器是18个字节  乐迪的SBUS是25字节
 

 

void My_USART_DMA_config(USART_TypeDef* USARTx,unsigned int u32_Baud); // 串口初始化函数
 
void USART_DMA_Tx_Data(USART_TypeDef* USARTx,uint8_t *send_buffer, uint32_t send_count); // DMA 数据发送函数




void My_USART1_DMA_TX_Init(void);
void My_USART1_DMA_RX_Init(void);

void My_USART2_DMA_TX_Init(void);
void My_USART2_DMA_RX_Init(void);

void My_USART3_DMA_TX_Init(void);
void My_USART3_DMA_RX_Init(void);

void My_USART6_DMA_TX_Init(void);
void My_USART6_DMA_RX_Init(void);
#endif

