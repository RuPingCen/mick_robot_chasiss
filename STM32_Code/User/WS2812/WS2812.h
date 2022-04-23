#ifndef __WS2812_H
#define __WS2812_H
#include "stm32f10x.h"




//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//SPI 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/13 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

 
 				  	    													  
void SPI1_Init(void);			 //初始化SPI口
void SPI1_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI1_ReadWriteByte(u8 TxData);//SPI总线读写一个字节
void SPI2_Init(void);			 //初始化SPI口
void SPI2_SetSpeed(u8 SpeedSet); //设置SPI速度   
u8 SPI2_ReadWriteByte(u8 TxData);//SPI总线读写一个字节


void WS2812_RGB_Init(void);

void WS2812_RGB_Rest(void);


void WS2812_RGB_Green(unsigned int j); // 绿色的灯
void WS2812_RGB_Red(unsigned int j); // 红色的灯
void WS2812_RGB_Blue(unsigned int j); // 蓝色的灯
void WS2812_RGB_Yellow(unsigned int j) ; // 黄色的灯
void WS2812_RGB_Black(unsigned int j) ; // 黑色的灯


void WS2812_Send_Byte(unsigned char dat_color);
void WS2812_RGB_Color(unsigned char R,unsigned char G,unsigned char B,unsigned int j);
void WS2812_RGB_All(void);

void WS2812_RGB_Test(void);
void WS2812_RGB_Test2(void);

#endif
