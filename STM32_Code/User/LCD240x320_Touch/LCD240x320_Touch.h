#ifndef __LCD240x320_TOUCH_H
#define	__LCD240x320_TOUCH_H

#include "stm32f10x.h"
 
//**************************************************************************//
//**************************************************************************//
#define	 SPI_CLK_PIN	  GPIO_Pin_7
#define  SPI_CLK_PORT	  GPIOG

#define	 SPI_MISO_PIN	  GPIO_Pin_6
#define	 SPI_MISO_PORT	GPIOF

#define	 SPI_MOSI_PIN	  GPIO_Pin_11
#define	 SPI_MOSI_PORT	GPIOF

#define  SPI_CS_PIN		  GPIO_Pin_10
#define  SPI_CS_PORT		GPIOF

#define  TP_INT_PIN	    GPIO_Pin_9
#define  TP_INT_PORT	  GPIOF



 
#define Touch_CLK_Out(a)	  if (a)\
													  GPIO_SetBits(SPI_CLK_PORT,SPI_CLK_PIN);	\
													  else		\
													  GPIO_ResetBits(SPI_CLK_PORT,SPI_CLK_PIN)
#define Touch_CS_Out(a)	    if (a)	\
														GPIO_SetBits(SPI_CS_PORT,SPI_CS_PIN);	\
														else		\
														GPIO_ResetBits(SPI_CS_PORT,SPI_CS_PIN)
#define Touch_MOSI_Out(a)	  if (a)\
														GPIO_SetBits(SPI_MISO_PORT,SPI_MOSI_PIN);	\
														else		\
														GPIO_ResetBits(SPI_MISO_PORT,SPI_MOSI_PIN)

#define  Touch_MISO_In	  	GPIO_ReadInputDataBit(SPI_MISO_PORT,SPI_MISO_PIN)
#define  INT_IN_2046      	GPIO_ReadInputDataBit(TP_INT_PORT,TP_INT_PIN)  	//读中断引脚状态					
						
						
						
//**************************************************************************//
//***************************自定义宏变量***********************************//						
 
#define LCD240x320_TOUCH_Delay_ms(x)   Delay_10us(100*x)	 //单位ms
						
						
//**************************************************************************//
//************************结构体声明****************************************// 
						
						
typedef	struct POINT /* 液晶坐标结构体 */
{
   uint16_t x;		
     uint16_t y;
}Coordinate;

typedef struct Parameter /*  校正系数结构体 */
{						
	long double An,  			 //注:sizeof(long double) = 8
							Bn,     
							Cn,   
							Dn,    
							En,    
							Fn,     
							Divider ;
}Parameter ;
 
//**************************************************************************// 
//**********************函数声明********************************************//  
 
extern u8 LCD240x320_Touch_Calibrate(void);
extern Coordinate *Read_LCD2046_XY(void);
extern void Touch_GetAdXY(int *x,int *y) ;
extern FunctionalState Calcu_touch_para( Coordinate * displayPtr,Coordinate * screenPtr,Parameter * para);
extern void LCD240x320_Touch_Init(void);
extern unsigned short XPT2046_ReadCMD(void) ;
extern void XPT2046_WriteCMD(unsigned char cmd) ;
extern void LCD240x320_Touch_DelayUS(vu32 cnt);


extern void Palette_draw_point(uint16_t x, uint16_t y);
extern void Palette_Init(void);
extern FunctionalState Get_touch_point(Coordinate * displayPtr);

//**************************************************************************//
//**************************************************************************//

#endif /* __LCD240x320_TOUCH_H */

/**********************************************

void EXTI9_5_IRQHandler(void)//中断处理函数
{ 
  if(EXTI_GetITStatus(EXTI_Line9) != RESET)
  {	
    LCD240x320_touch_flag=1;
    EXTI_ClearITPendingBit(EXTI_Line9);
  }
}
           
***********************************************/
