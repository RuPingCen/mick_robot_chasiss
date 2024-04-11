 /*************************************************************************
说明:		
*  F407有2个高级定时器，10个通用定时器，2个基本定时器(TIM6、TIM7)
*  通用定时器具有功能： 定时，输出比较，输入捕获等功能
*  为避免与PWM以及输入捕捉模块相冲突，因此建议优先使用(TIM6、TIM7)作为定时中断功能
*  保留TIM2-TIM5 以及 TIM8-TIM14作为PWM和输入捕捉功能

主要函数
	void Timer_2to7_Init(TIM_TypeDef* TIMx,u16 u16counter); //定时器初始化功能
	void Timer_start(TIM_TypeDef* TIMx)
	void Timer_stop(TIM_TypeDef* TIMx)

*  作者 :CRP
*  修改时间: 2020-2-3    
*  备    注: 无
*************************************************************************/

#include "bsp_timer.h"
 
  
 
/*************************************************************************
*  Timer_2to7_Init
*  功能说明：把STM32定时器配置为基础定时，即只具备定时产生中断功能
*  参数说明：TIMx        模块号（TIM2-TIM7）
*            u16counter   定时时间参数（定时时间 == 1us * u16counter）
*  函数返回：无
*  修改时间：2023-2-3
*  备    注：CRP   
*************************************************************************/	
 
void Timer_2to7_Init(TIM_TypeDef* TIMx,u16 u16counter)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure; 
	if(TIMx == TIM2)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);/* 设置TIM2CLK 为 84MHZ */		
		//TIM_DeInit(TIM2);	
		/* 自动重装载寄存器周期的值(计数值) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* 时钟预分频数为84 */	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //定时器计数模式为向上计数
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);//清除向上计数中断标志位	
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM2, DISABLE);		/*先关闭等待使用*/  																   
	}
	else if(TIMx == TIM3)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);/* 设置TIM3CLK 为 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* 自动重装载寄存器周期的值(计数值) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* 时钟预分频数为84 */	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //定时器计数模式为向上计数
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);//清除向上计数中断标志位	
		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM3, DISABLE);/*先关闭等待使用*/  
	}
	else if(TIMx == TIM4)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);/* 设置TIM4CLK 为 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* 自动重装载寄存器周期的值(计数值) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* 时钟预分频数为84 */	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //定时器计数模式为向上计数
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);//清除向上计数中断标志位	
		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM4, DISABLE);/*先关闭等待使用*/ 
	}
	else if(TIMx == TIM5)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);/* 设置TIM5CLK 为 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* 自动重装载寄存器周期的值(计数值) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* 时钟预分频数为84 */	
		/* 对外部时钟进行采样的时钟分频,这里没有用到 */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //定时器计数模式为向上计数
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);//清除向上计数中断标志位	
		TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM5, DISABLE);	/*先关闭等待使用*/ 
	}	
	if(TIMx == TIM6)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);/* 设置TIM2CLK 为 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* 自动重装载寄存器周期的值(计数值) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* 时钟预分频数为84  基数为1us*/	
 
		TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);//清除计数中断标志位	
		TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM6, DISABLE); /*先关闭等待使用*/  
	}
	else if(TIMx == TIM7)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);/* 设置TIM3CLK 为 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* 自动重装载寄存器周期的值(计数值) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* 时钟预分频数为72 */	
		 
		TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM7, TIM_FLAG_Update);//清除向上计数中断标志位	
		TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM7, DISABLE);	 /*先关闭等待使用*/  
	}
	else
	{
		while(1);
	}		  
}
void Timer_start(TIM_TypeDef* TIMx)
{
   TIM_Cmd(TIMx, ENABLE); //启动计数函数		 
}
 
void Timer_stop(TIM_TypeDef* TIMx)
{
   TIM_Cmd(TIMx, DISABLE); //STM32定时中断基本功能 停止计数函数
} 
