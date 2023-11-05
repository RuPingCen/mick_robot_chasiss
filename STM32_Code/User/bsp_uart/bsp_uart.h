#ifndef __BSP_UART_H
#define	__BSP_UART_H

#ifdef __cplusplus
 extern "C" {
#endif
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h" 


#define USART_REC_LEN  			25// 25/18  	//��������ֽ��� 100



//******************DMA������غ�*******************************************//

#define USART1_DR_Base  0x40013804		// 0x40013800 + 0x04 = 0x40013804   ����1���ͼĴ�����ַ
#define USART2_DR_Base  0x40004404		// 0x40004400 + 0x04 = 0x40004404   ����1���ͼĴ�����ַ
#define USART3_DR_Base  0x40004804		// 0x40004800 + 0x04 = 0x40004804   ����1���ͼĴ�����ַ
#define USART4_DR_Base  0x40004C04		// 0x40004C00 + 0x04 = 0x40004C04   ����1���ͼĴ�����ַ


 void My_Config_USART_Init(USART_TypeDef* USARTx,unsigned int u32_Baud , unsigned char UART_IT_EN);

 void USART1_Config(unsigned int u32_Baud , unsigned char UART_IT_EN);//����1��ʼ������
 void USART2_Config(unsigned int u32_Baud , unsigned char UART_IT_EN);
 void USART3_Config(unsigned int u32_Baud , unsigned char UART_IT_EN);
 void UART4_Config(unsigned int u32_Baud , unsigned char UART_IT_EN);
 void UART5_Config(unsigned int u32_Baud , unsigned char UART_IT_EN);
 void USART1_NVIC_Configuration(void);//�����ж����ú���
 void USART2_NVIC_Configuration(void);//�����ж����ú���
 void USART3_NVIC_Configuration(void);//�����ж����ú���
 void USART4_NVIC_Configuration(void);//�����ж����ú���
 void USART5_NVIC_Configuration(void);//�����ж����ú���
 
 void USART_DMA_Send_Config(USART_TypeDef* USARTx,unsigned char DMA_send_data[],unsigned int u32DMA_size,unsigned char DMA_send_priority);

 
 void USART_DMA_Rec_Config(USART_TypeDef* USARTx,unsigned int u32_Baud,unsigned char lendat);
 void USART1_DMA_Rec_Config(unsigned int u32_Baud);

 void UART_send_string(USART_TypeDef* USARTx,char *buf);//�ַ������ͺ���
 void UART_send_char(USART_TypeDef* USARTx,char buf);//�ַ����ͺ���
 void UART_send_data(USART_TypeDef* USARTx,unsigned int u32tempdat);//�޷������ݷ��ͺ���
 void UART_send_intdata(USART_TypeDef* USARTx,int u32tempdat); //USART1~UART5   u32tempdat��-2147483648~2147483647
 void UART_send_floatdat(USART_TypeDef* USARTx,float floatempdat);//�������ݷ��ͺ���
 void UART_send_buffer(USART_TypeDef* USARTx,unsigned char *buf,unsigned int len);
#ifdef __cplusplus
}
#endif

#endif
