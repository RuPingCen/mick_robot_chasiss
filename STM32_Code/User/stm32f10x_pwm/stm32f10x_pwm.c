#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h" 
#include "misc.h" 
#include "stm32f10x_tim.h"
 

#include "stm32f10x_pwm.h" 


/*************************************************************************
*  函数名称： PWM_config_Init
*  功能说明： STM32 pwm 模块初始化函数
*  参数说明： u8PWMx (PWM1 PWM2 PWM3)  
*             u16channel ( PWM_Channel_x 0、1、2、3 )
*             u32period  PWm周期参数   PWM周期 = u32period * 1us
*             u16duty ( 1 ~ 1000) PWM占空比参数  输出占空比 = u16duty/1000 * 100%
*
*  引脚说明：        PWM1             PWM2               PWM3
*
*  PWM_Channel_0    PA6               PB6                  PA0                     
*  PWM_Channel_1    PA7               PB7                  PA1                   
*  PWM_Channel_2    PB0               PB8                  PA2                       
*  PWM_Channel_3    PB1               PB9                  PA3                               
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP  PWM模块是使用定时器的输出比较功能产生的PWM
*                  PWM1 模块使用了定时器 (TIM3)
*                  PWM2 模块使用了定时器 (TIM4)
*                  PWM3 模块使用了定时器 (TIM2)
*                  因此使用PWm相应的模块  就不要再次使用定时器模块   避免冲突
*************************************************************************/
void PWM_config_Init(u8 u8PWMx , u16 u16channel , u32 u32period , u16 u16duty)
{
	    GPIO_InitTypeDef GPIO_InitStructure;
	   	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	    TIM_OCInitTypeDef  TIM_OCInitStructure;  
	
			if(u8PWMx == PWM1) //使用定时器3输出比较功能
			{    
						RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); /* 设置TIM3CLK 为 72MHZ */
							/* Time base configuration */		 
						TIM_TimeBaseStructure.TIM_Period = u32period-1;       //当定时器从0计数到999，即为1000次，为一个定时周期
						TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //设置预分频：不预分频，即为72MHz
						TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频(这里用不到)
						TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
						TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
				 
						TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
				
						if(u16channel == PWM_Channel_0)
						{              
								/* GPIOA clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOA, &GPIO_InitStructure);
							
							 /* PWM1 Mode configuration: Channel1 */
							
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
							TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
							TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //使能通道1
							TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
						}
						else if(u16channel == PWM_Channel_1)
						{
								 /* GPIOA clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOA, &GPIO_InitStructure);
														/* PWM1 Mode configuration: Channel2 */
								TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
								TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
								TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //使能通道2
								TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
						}
						else if(u16channel == PWM_Channel_2)
						{
								/*  GPIOB clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOB, &GPIO_InitStructure);
							
												/* PWM1 Mode configuration: Channel3 */
								TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
								TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
								TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //使能通道3
								TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
							
						}
						else if(u16channel == PWM_Channel_3)
						{
								/*  GPIOB clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOB, &GPIO_InitStructure);
							 
												/* PWM1 Mode configuration: Channel4 */
								TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
								TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
								TIM_OC4Init(TIM3, &TIM_OCInitStructure);	//使能通道4
								TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
								TIM_ARRPreloadConfig(TIM3, ENABLE);			 // 使能TIM3重载寄存器ARR
						}
						else  
						{
								while(1);//初始化错误
						}
		
						TIM_Cmd(TIM3, ENABLE);                   //使能定时器3	
				
				
			}
		else if(u8PWMx == PWM2)//使用定时器4 输出比较功能
		{
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); /* 设置TIM4CLK 为 72MHZ */
          TIM_TimeBaseStructure.TIM_Period = u32period-1;       //当定时器从0计数到999，即为1000次，为一个定时周期
					TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //设置预分频：不预分频，即为72MHz
					TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频(这里用不到)
					TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
					TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
			 
			    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
			
          if(u16channel == PWM_Channel_0)
					{              
						 	/* GPIOB clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							/*GPIOB Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
						
						  
						
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
						TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
						TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //使能通道1
						TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_1)
					{
               
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							 
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
													/* PWM1 Mode configuration: Channel2 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
							TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //使能通道2
							TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_2)
					{
               
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							 
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
						
											 
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
							TIM_OC3Init(TIM4, &TIM_OCInitStructure);	  
							TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
              
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							 
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
						  
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
							TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
							TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
							TIM_ARRPreloadConfig(TIM4, ENABLE);			 // 使能TIM4重载寄存器ARR
          }
					else  
					{
              while(1);//初始化错误
          }
  
					TIM_Cmd(TIM4, ENABLE);                   //使能定时器4	
    }
		else if(u8PWMx == PWM3) //使用定时器2输出比较功能
		{
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); /* 设置TIM3CLK 为 72MHZ */
          TIM_TimeBaseStructure.TIM_Period = u32period-1;       //当定时器从0计数到999，即为1000次，为一个定时周期
					TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //设置预分频：不预分频，即为72MHz
					TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//设置时钟分频系数：不分频(这里用不到)
					TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //向上计数模式
					TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure);
			 
			    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1
			
          if(u16channel == PWM_Channel_0)
					{              
						 	/* GPIOA clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
						
						  
						
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
						TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //当定时器计数值小于CCR1_Val时为高电平
						TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //使能通道1
						TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_1)
					{
               /* GPIOA clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
													/* PWM1 Mode configuration: Channel2 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
							TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //使能通道2
							TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_2)
					{
              /*  GPIOB clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
						
											/* PWM1 Mode configuration: Channel3 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
							TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //使能通道3
							TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
              /*  GPIOB clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // 复用推挽输出
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
						 
						 					/* PWM1 Mode configuration: Channel4 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
							TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//使能通道4
							TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
							//TIM_ARRPreloadConfig(TIM1, ENABLE);			 // 使能TIM3重载寄存器ARR
          }
					else  
					{
              while(1);//初始化错误
          }
  
					TIM_Cmd(TIM2, ENABLE);                   //使能定时器1	
    }
		else  
		{
       while(1);  
    }
}
/*************************************************************************
*  函数名称： PWM_Duty_Change
*  功能说明： STM32 pwm 占空比改变函数
*  参数说明： u8PWMx (PWM1 PWM2 PWM3)  
*             u16channel ( PWM_Channel_x 0、1、2、3 )
*             u32period  PWm周期参数  这里的参数必须与初始化PWm的周期一致  
*                                      这里只是为了计算占空比的时候方便  所以再传入一次
*             u16duty ( 1 ~ 1000) PWM占空比参数  输出占空比 = u16duty/1000 * 100%
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP   
*************************************************************************/
void PWM_Duty_Change(u8 u8PWMx,u16 u16channel , u32 u32period, u16 u16duty)
{
	  
//	    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	    TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	     
    if(u8PWMx == PWM1) //使用定时器3输出比较功能
		{ 
			     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1 
			   if(u16channel == PWM_Channel_0)
					{              
						 
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
						TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //使能通道1
						TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
						 	 
          }
					else if(u16channel == PWM_Channel_1)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
							TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //使能通道2
							TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
						   
          }
					else if(u16channel == PWM_Channel_2)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
							TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //使能通道3
							TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
						  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
							TIM_OC4Init(TIM3, &TIM_OCInitStructure);	//使能通道4
							TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
							 
          }
					else  
					{
              while(1);//初始化错误
          }
				 
		}
		else if(u8PWMx == PWM2) //使用定时器3输出比较功能
		{ 
			   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1 
			   if(u16channel == PWM_Channel_0)
					{              
						 
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
						TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //使能通道1
						TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
						 	 
          }
					else if(u16channel == PWM_Channel_1)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
							TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //使能通道2
							TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
						   
          }
					else if(u16channel == PWM_Channel_2)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
							TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //使能通道3
							TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
						  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
							TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//使能通道4
							TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
							 
          }
					else  
					{
              while(1);//初始化错误
          }
		}
		else if(u8PWMx == PWM3) //使用定时器3输出比较功能
		{ 
			   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //配置为PWM模式1 
			   if(u16channel == PWM_Channel_0)
					{              
						 
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //设置跳变值，当计数器计数到这个值时，电平发生跳变
						TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //使能通道1
						TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
						 	 
          }
					else if(u16channel == PWM_Channel_1)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //设置通道2的电平跳变值，输出另外一个占空比的PWM
							TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //使能通道2
							TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
						   
          }
					else if(u16channel == PWM_Channel_2)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道3的电平跳变值，输出另外一个占空比的PWM
							TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //使能通道3
							TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
						  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//设置通道4的电平跳变值，输出另外一个占空比的PWM
							TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//使能通道4
							TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
							 
          }
					else  
					{
              while(1);//初始化错误
          }
			
		}
		else 
			while(1);//初始化错误  


}


/*************************************************************************
功能：			配置定时器为外部计数模式

TIM4      外部计数引脚  PE0 （浮空输入模式）

TIM3      外部计数引脚  PD2  （浮空输入模式）

TIM1      外部计数引脚  PA12 （浮空输入模式）


步骤：
     1、设置定时器计数溢出值（周期） 预分频系数  计数模式    TCR1
		 2、设置SMCR  使能外部时钟
     3、设置中断		 
		 4、启动定时器 
*************************************************************************/
void Time4_ExternPulse_CountMode_Init(void)
{  
			GPIO_InitTypeDef GPIO_InitStructure;
			TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;	   
//			NVIC_InitTypeDef NVIC_InitStructure; 



			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); 

			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 ;  
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
			GPIO_Init(GPIOE, &GPIO_InitStructure);            

			TIM_TimeBaseStructure.TIM_Prescaler = 0x00; 
			TIM_TimeBaseStructure.TIM_Period = 0xFFFF; 
			TIM_TimeBaseStructure.TIM_ClockDivision = 0x0; 
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; 
			TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);  // Time base configuration 

			TIM_ETRClockMode2Config(TIM4, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);  //  TIM4->SMCR=0x4000;   启动外部计数 上升沿有效  关闭预分频  不使用滤波
     

/*
			NVIC_InitStructure.NVIC_IRQChannel = TIM4; //需要使用时 启用     可以放在中断里看看是否计数溢出了
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; 
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
			NVIC_Init(&NVIC_InitStructure);
			TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);  
			
*/			
			
			TIM_SetCounter(TIM4, 0);    
			TIM_Cmd(TIM4, ENABLE); 
}

/*************************************************************************
 函数功能：输入捕捉脉冲的宽度
 
 使用TIM4     CH1~4      PB6、PB7、PB8、PB9
 
*************************************************************************/
void Time4_CaptureMode_Init(void)
{  
	   TIM_ICInitTypeDef TIM4_ICInitStructure; 
		 GPIO_InitTypeDef GPIO_InitStructure;  
     TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;  
     NVIC_InitTypeDef NVIC_InitStructure;  
       
     
     RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);      
     RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
       
    
     GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;  
     GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;  
     GPIO_Init(GPIOB, &GPIO_InitStructure);  
   
     
     TIM_TimeBaseStructure.TIM_Period = 0xffff;    //  
     TIM_TimeBaseStructure.TIM_Prescaler = 71; //  
     TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;//  
     TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 
     TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);    //  
       
      
     TIM4_ICInitStructure.TIM_Channel = TIM_Channel_1 ;  //    
     TIM4_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;  //  
     TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//  
     TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //   
     TIM4_ICInitStructure.TIM_ICFilter = 0x00;  //  
     TIM_ICInit(TIM4, &TIM4_ICInitStructure);  
		 
		 
		 TIM4_ICInitStructure.TIM_Channel = TIM_Channel_2 ;  //    
     TIM4_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;  //  
     TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//  
     TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //   
     TIM4_ICInitStructure.TIM_ICFilter = 0x00;  //  
     TIM_ICInit(TIM4, &TIM4_ICInitStructure);  
		 
		 
		 TIM4_ICInitStructure.TIM_Channel = TIM_Channel_3 ;  //    
     TIM4_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;  //  
     TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//  
     TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //   
     TIM4_ICInitStructure.TIM_ICFilter = 0x00;  //  
     TIM_ICInit(TIM4, &TIM4_ICInitStructure);  
		 
		 
		 
		 TIM4_ICInitStructure.TIM_Channel = TIM_Channel_4 ;  //    
     TIM4_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;  //  
     TIM4_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;//  
     TIM4_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1; //   
     TIM4_ICInitStructure.TIM_ICFilter = 0x00;  //  
     TIM_ICInit(TIM4, &TIM4_ICInitStructure);  
       
      
     NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;    //  
     NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  // 
     NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; //  
     NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;    //  
     NVIC_Init(&NVIC_InitStructure);    
       
     TIM_ClearITPendingBit(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4|TIM_IT_Update);  
		 
     TIM_ITConfig(TIM4, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4, ENABLE);  
		 
     TIM_Cmd(TIM4, ENABLE); 
}
/*********************************************END OF FILE**********************/
