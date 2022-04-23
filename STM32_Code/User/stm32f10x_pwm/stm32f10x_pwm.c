#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h" 
#include "misc.h" 
#include "stm32f10x_tim.h"
 

#include "stm32f10x_pwm.h" 


/*************************************************************************
*  �������ƣ� PWM_config_Init
*  ����˵���� STM32 pwm ģ���ʼ������
*  ����˵���� u8PWMx (PWM1 PWM2 PWM3)  
*             u16channel ( PWM_Channel_x 0��1��2��3 )
*             u32period  PWm���ڲ���   PWM���� = u32period * 1us
*             u16duty ( 1 ~ 1000) PWMռ�ձȲ���  ���ռ�ձ� = u16duty/1000 * 100%
*
*  ����˵����        PWM1             PWM2               PWM3
*
*  PWM_Channel_0    PA6               PB6                  PA0                     
*  PWM_Channel_1    PA7               PB7                  PA1                   
*  PWM_Channel_2    PB0               PB8                  PA2                       
*  PWM_Channel_3    PB1               PB9                  PA3                               
*
*  �������أ� ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP  PWMģ����ʹ�ö�ʱ��������ȽϹ��ܲ�����PWM
*                  PWM1 ģ��ʹ���˶�ʱ�� (TIM3)
*                  PWM2 ģ��ʹ���˶�ʱ�� (TIM4)
*                  PWM3 ģ��ʹ���˶�ʱ�� (TIM2)
*                  ���ʹ��PWm��Ӧ��ģ��  �Ͳ�Ҫ�ٴ�ʹ�ö�ʱ��ģ��   �����ͻ
*************************************************************************/
void PWM_config_Init(u8 u8PWMx , u16 u16channel , u32 u32period , u16 u16duty)
{
	    GPIO_InitTypeDef GPIO_InitStructure;
	   	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	    TIM_OCInitTypeDef  TIM_OCInitStructure;  
	
			if(u8PWMx == PWM1) //ʹ�ö�ʱ��3����ȽϹ���
			{    
						RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); /* ����TIM3CLK Ϊ 72MHZ */
							/* Time base configuration */		 
						TIM_TimeBaseStructure.TIM_Period = u32period-1;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
						TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
						TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
						TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
						TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
				 
						TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
				
						if(u16channel == PWM_Channel_0)
						{              
								/* GPIOA clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOA, &GPIO_InitStructure);
							
							 /* PWM1 Mode configuration: Channel1 */
							
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //��������ֵ�������������������ֵʱ����ƽ��������
							TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
							TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��1
							TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
						}
						else if(u16channel == PWM_Channel_1)
						{
								 /* GPIOA clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOA, &GPIO_InitStructure);
														/* PWM1 Mode configuration: Channel2 */
								TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
								TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
								TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��2
								TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
						}
						else if(u16channel == PWM_Channel_2)
						{
								/*  GPIOB clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOB, &GPIO_InitStructure);
							
												/* PWM1 Mode configuration: Channel3 */
								TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
								TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
								TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��3
								TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
							
						}
						else if(u16channel == PWM_Channel_3)
						{
								/*  GPIOB clock enable */
								RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
								/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
								GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
								GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
								GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
								GPIO_Init(GPIOB, &GPIO_InitStructure);
							 
												/* PWM1 Mode configuration: Channel4 */
								TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
								TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
								TIM_OC4Init(TIM3, &TIM_OCInitStructure);	//ʹ��ͨ��4
								TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
								TIM_ARRPreloadConfig(TIM3, ENABLE);			 // ʹ��TIM3���ؼĴ���ARR
						}
						else  
						{
								while(1);//��ʼ������
						}
		
						TIM_Cmd(TIM3, ENABLE);                   //ʹ�ܶ�ʱ��3	
				
				
			}
		else if(u8PWMx == PWM2)//ʹ�ö�ʱ��4 ����ȽϹ���
		{
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE); /* ����TIM4CLK Ϊ 72MHZ */
          TIM_TimeBaseStructure.TIM_Period = u32period-1;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
					TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
					TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
					TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
					TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
			 
			    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
			
          if(u16channel == PWM_Channel_0)
					{              
						 	/* GPIOB clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							/*GPIOB Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
						
						  
						
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //��������ֵ�������������������ֵʱ����ƽ��������
						TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
						TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
						TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_1)
					{
               
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							 
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
													/* PWM1 Mode configuration: Channel2 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //ʹ��ͨ��2
							TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_2)
					{
               
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							 
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
						
											 
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC3Init(TIM4, &TIM_OCInitStructure);	  
							TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
              
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); 
							 
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOB, &GPIO_InitStructure);
						  
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC4Init(TIM4, &TIM_OCInitStructure);	 
							TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
							TIM_ARRPreloadConfig(TIM4, ENABLE);			 // ʹ��TIM4���ؼĴ���ARR
          }
					else  
					{
              while(1);//��ʼ������
          }
  
					TIM_Cmd(TIM4, ENABLE);                   //ʹ�ܶ�ʱ��4	
    }
		else if(u8PWMx == PWM3) //ʹ�ö�ʱ��2����ȽϹ���
		{
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); /* ����TIM3CLK Ϊ 72MHZ */
          TIM_TimeBaseStructure.TIM_Period = u32period-1;       //����ʱ����0������999����Ϊ1000�Σ�Ϊһ����ʱ����
					TIM_TimeBaseStructure.TIM_Prescaler = 71;	    //����Ԥ��Ƶ����Ԥ��Ƶ����Ϊ72MHz
					TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1 ;	//����ʱ�ӷ�Ƶϵ��������Ƶ(�����ò���)
					TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //���ϼ���ģʽ
					TIM_TimeBaseInit( TIM2, &TIM_TimeBaseStructure);
			 
			    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1
			
          if(u16channel == PWM_Channel_0)
					{              
						 	/* GPIOA clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
						
						  
						
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //��������ֵ�������������������ֵʱ����ƽ��������
						TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;  //����ʱ������ֵС��CCR1_ValʱΪ�ߵ�ƽ
						TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��1
						TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_1)
					{
               /* GPIOA clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
													/* PWM1 Mode configuration: Channel2 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //ʹ��ͨ��2
							TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
          }
					else if(u16channel == PWM_Channel_2)
					{
              /*  GPIOB clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
						
											/* PWM1 Mode configuration: Channel3 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��3
							TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
              /*  GPIOB clock enable */
							RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
							/*GPIOA Configuration: TIM3 channel 1 and 2 as alternate function push-pull */
							GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 ;
							GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		    // �����������
							GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
							GPIO_Init(GPIOA, &GPIO_InitStructure);
						 
						 					/* PWM1 Mode configuration: Channel4 */
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4
							TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
							//TIM_ARRPreloadConfig(TIM1, ENABLE);			 // ʹ��TIM3���ؼĴ���ARR
          }
					else  
					{
              while(1);//��ʼ������
          }
  
					TIM_Cmd(TIM2, ENABLE);                   //ʹ�ܶ�ʱ��1	
    }
		else  
		{
       while(1);  
    }
}
/*************************************************************************
*  �������ƣ� PWM_Duty_Change
*  ����˵���� STM32 pwm ռ�ձȸı亯��
*  ����˵���� u8PWMx (PWM1 PWM2 PWM3)  
*             u16channel ( PWM_Channel_x 0��1��2��3 )
*             u32period  PWm���ڲ���  ����Ĳ����������ʼ��PWm������һ��  
*                                      ����ֻ��Ϊ�˼���ռ�ձȵ�ʱ�򷽱�  �����ٴ���һ��
*             u16duty ( 1 ~ 1000) PWMռ�ձȲ���  ���ռ�ձ� = u16duty/1000 * 100%
*
*  �������أ� ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP   
*************************************************************************/
void PWM_Duty_Change(u8 u8PWMx,u16 u16channel , u32 u32period, u16 u16duty)
{
	  
//	    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	    TIM_OCInitTypeDef  TIM_OCInitStructure; 
	
	     
    if(u8PWMx == PWM1) //ʹ�ö�ʱ��3����ȽϹ���
		{ 
			     TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1 
			   if(u16channel == PWM_Channel_0)
					{              
						 
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //��������ֵ�������������������ֵʱ����ƽ��������
						TIM_OC1Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��1
						TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
						 	 
          }
					else if(u16channel == PWM_Channel_1)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC2Init(TIM3, &TIM_OCInitStructure);	  //ʹ��ͨ��2
							TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Enable);
						   
          }
					else if(u16channel == PWM_Channel_2)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC3Init(TIM3, &TIM_OCInitStructure);	 //ʹ��ͨ��3
							TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
						  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC4Init(TIM3, &TIM_OCInitStructure);	//ʹ��ͨ��4
							TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
							 
          }
					else  
					{
              while(1);//��ʼ������
          }
				 
		}
		else if(u8PWMx == PWM2) //ʹ�ö�ʱ��3����ȽϹ���
		{ 
			   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1 
			   if(u16channel == PWM_Channel_0)
					{              
						 
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //��������ֵ�������������������ֵʱ����ƽ��������
						TIM_OC1Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��1
						TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
						 	 
          }
					else if(u16channel == PWM_Channel_1)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC2Init(TIM4, &TIM_OCInitStructure);	  //ʹ��ͨ��2
							TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
						   
          }
					else if(u16channel == PWM_Channel_2)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC3Init(TIM4, &TIM_OCInitStructure);	 //ʹ��ͨ��3
							TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
						  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC4Init(TIM4, &TIM_OCInitStructure);	//ʹ��ͨ��4
							TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);
							 
          }
					else  
					{
              while(1);//��ʼ������
          }
		}
		else if(u8PWMx == PWM3) //ʹ�ö�ʱ��3����ȽϹ���
		{ 
			   TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;	    //����ΪPWMģʽ1 
			   if(u16channel == PWM_Channel_0)
					{              
						 
						TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
						TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	   //��������ֵ�������������������ֵʱ����ƽ��������
						TIM_OC1Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��1
						TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
						 	 
          }
					else if(u16channel == PWM_Channel_1)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	  //����ͨ��2�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC2Init(TIM2, &TIM_OCInitStructure);	  //ʹ��ͨ��2
							TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
						   
          }
					else if(u16channel == PWM_Channel_2)
					{
							TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��3�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC3Init(TIM2, &TIM_OCInitStructure);	 //ʹ��ͨ��3
							TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
						
          }
					else if(u16channel == PWM_Channel_3)
					{
						  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
							TIM_OCInitStructure.TIM_Pulse = ((u32period/100.0)*u16duty +0.5)/10;	//����ͨ��4�ĵ�ƽ����ֵ���������һ��ռ�ձȵ�PWM
							TIM_OC4Init(TIM2, &TIM_OCInitStructure);	//ʹ��ͨ��4
							TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);
							 
          }
					else  
					{
              while(1);//��ʼ������
          }
			
		}
		else 
			while(1);//��ʼ������  


}


/*************************************************************************
���ܣ�			���ö�ʱ��Ϊ�ⲿ����ģʽ

TIM4      �ⲿ��������  PE0 ����������ģʽ��

TIM3      �ⲿ��������  PD2  ����������ģʽ��

TIM1      �ⲿ��������  PA12 ����������ģʽ��


���裺
     1�����ö�ʱ���������ֵ�����ڣ� Ԥ��Ƶϵ��  ����ģʽ    TCR1
		 2������SMCR  ʹ���ⲿʱ��
     3�������ж�		 
		 4��������ʱ�� 
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

			TIM_ETRClockMode2Config(TIM4, TIM_ExtTRGPSC_OFF, TIM_ExtTRGPolarity_NonInverted, 0);  //  TIM4->SMCR=0x4000;   �����ⲿ���� ��������Ч  �ر�Ԥ��Ƶ  ��ʹ���˲�
     

/*
			NVIC_InitStructure.NVIC_IRQChannel = TIM4; //��Ҫʹ��ʱ ����     ���Է����ж��￴���Ƿ���������
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
 �������ܣ����벶׽����Ŀ��
 
 ʹ��TIM4     CH1~4      PB6��PB7��PB8��PB9
 
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
