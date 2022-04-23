#ifndef __WS2812_H
#define __WS2812_H
#include "stm32f10x.h"




//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/13 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

 
 				  	    													  
void SPI1_Init(void);			 //��ʼ��SPI��
void SPI1_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�
void SPI2_Init(void);			 //��ʼ��SPI��
void SPI2_SetSpeed(u8 SpeedSet); //����SPI�ٶ�   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI���߶�дһ���ֽ�


void WS2812_RGB_Init(void);

void WS2812_RGB_Rest(void);


void WS2812_RGB_Green(unsigned int j); // ��ɫ�ĵ�
void WS2812_RGB_Red(unsigned int j); // ��ɫ�ĵ�
void WS2812_RGB_Blue(unsigned int j); // ��ɫ�ĵ�
void WS2812_RGB_Yellow(unsigned int j) ; // ��ɫ�ĵ�
void WS2812_RGB_Black(unsigned int j) ; // ��ɫ�ĵ�


void WS2812_Send_Byte(unsigned char dat_color);
void WS2812_RGB_Color(unsigned char R,unsigned char G,unsigned char B,unsigned int j);
void WS2812_RGB_All(void);

void WS2812_RGB_Test(void);
void WS2812_RGB_Test2(void);

#endif
