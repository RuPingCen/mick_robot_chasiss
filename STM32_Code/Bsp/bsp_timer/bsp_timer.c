 /*************************************************************************
˵��:		
*  F407��2���߼���ʱ����10��ͨ�ö�ʱ����2��������ʱ��(TIM6��TIM7)
*  ͨ�ö�ʱ�����й��ܣ� ��ʱ������Ƚϣ����벶��ȹ���
*  Ϊ������PWM�Լ����벶׽ģ�����ͻ����˽�������ʹ��(TIM6��TIM7)��Ϊ��ʱ�жϹ���
*  ����TIM2-TIM5 �Լ� TIM8-TIM14��ΪPWM�����벶׽����

��Ҫ����
	void Timer_2to7_Init(TIM_TypeDef* TIMx,u16 u16counter); //��ʱ����ʼ������
	void Timer_start(TIM_TypeDef* TIMx)
	void Timer_stop(TIM_TypeDef* TIMx)

*  ���� :�CCRP
*  �޸�ʱ��: 2020-2-3    
*  ��    ע: ��
*************************************************************************/

#include "bsp_timer.h"
 
  
 
/*************************************************************************
*  Timer_2to7_Init
*  ����˵������STM32��ʱ������Ϊ������ʱ����ֻ�߱���ʱ�����жϹ���
*  ����˵����TIMx        ģ��ţ�TIM2-TIM7��
*            u16counter   ��ʱʱ���������ʱʱ�� == 1us * u16counter��
*  �������أ���
*  �޸�ʱ�䣺2023-2-3
*  ��    ע��CRP   
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

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);/* ����TIM2CLK Ϊ 84MHZ */		
		//TIM_DeInit(TIM2);	
		/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* ʱ��Ԥ��Ƶ��Ϊ84 */	
		/* ���ⲿʱ�ӽ��в�����ʱ�ӷ�Ƶ,����û���õ� */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //��ʱ������ģʽΪ���ϼ���
		TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM2, TIM_FLAG_Update);//������ϼ����жϱ�־λ	
		TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM2, DISABLE);		/*�ȹرյȴ�ʹ��*/  																   
	}
	else if(TIMx == TIM3)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);/* ����TIM3CLK Ϊ 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* ʱ��Ԥ��Ƶ��Ϊ84 */	
		/* ���ⲿʱ�ӽ��в�����ʱ�ӷ�Ƶ,����û���õ� */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //��ʱ������ģʽΪ���ϼ���
		TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM3, TIM_FLAG_Update);//������ϼ����жϱ�־λ	
		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM3, DISABLE);/*�ȹرյȴ�ʹ��*/  
	}
	else if(TIMx == TIM4)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4 , ENABLE);/* ����TIM4CLK Ϊ 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* ʱ��Ԥ��Ƶ��Ϊ84 */	
		/* ���ⲿʱ�ӽ��в�����ʱ�ӷ�Ƶ,����û���õ� */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //��ʱ������ģʽΪ���ϼ���
		TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM4, TIM_FLAG_Update);//������ϼ����жϱ�־λ	
		TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM4, DISABLE);/*�ȹرյȴ�ʹ��*/ 
	}
	else if(TIMx == TIM5)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5 , ENABLE);/* ����TIM5CLK Ϊ 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* ʱ��Ԥ��Ƶ��Ϊ84 */	
		/* ���ⲿʱ�ӽ��в�����ʱ�ӷ�Ƶ,����û���õ� */
		TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1;	
		TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //��ʱ������ģʽΪ���ϼ���
		TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM5, TIM_FLAG_Update);//������ϼ����жϱ�־λ	
		TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM5, DISABLE);	/*�ȹرյȴ�ʹ��*/ 
	}	
	if(TIMx == TIM6)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 6;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6 , ENABLE);/* ����TIM2CLK Ϊ 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* ʱ��Ԥ��Ƶ��Ϊ84  ����Ϊ1us*/	
 
		TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM6, TIM_FLAG_Update);//��������жϱ�־λ	
		TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM6, DISABLE); /*�ȹرյȴ�ʹ��*/  
	}
	else if(TIMx == TIM7)
	{
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  													
		NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;	  
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;	
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);	

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7 , ENABLE);/* ����TIM3CLK Ϊ 72MHZ */		
		//TIM_DeInit(TIM2);	
		/* �Զ���װ�ؼĴ������ڵ�ֵ(����ֵ) */
		TIM_TimeBaseStructure.TIM_Period=u16counter-1;//0~65536
		TIM_TimeBaseStructure.TIM_Prescaler= 84-1; /* ʱ��Ԥ��Ƶ��Ϊ72 */	
		 
		TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure);	
		TIM_ClearFlag(TIM7, TIM_FLAG_Update);//������ϼ����жϱ�־λ	
		TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE);	
		TIM_Cmd(TIM7, DISABLE);	 /*�ȹرյȴ�ʹ��*/  
	}
	else
	{
		while(1);
	}		  
}
void Timer_start(TIM_TypeDef* TIMx)
{
   TIM_Cmd(TIMx, ENABLE); //������������		 
}
 
void Timer_stop(TIM_TypeDef* TIMx)
{
   TIM_Cmd(TIMx, DISABLE); //STM32��ʱ�жϻ������� ֹͣ��������
} 
