/************************************************************************
*  ���� :�CCRP
*  ����˵��: ���ٷ����޸��Ժ� ������Լ��Ŀ�
*  ����˵��: uint32_t 32λ��
  
*  �޸�ʱ��: 2014-7-7     
*  ��    ע: ǧ���ܰѱ��� ��TimingDelay��ɾ��
* ��ʹ�ú���:
		void SysTick_Init(void);//delay��ʱ������ʼ��
		void Delay_10us(uint32_t nTime); // nTime*10us ��ʱ���� �������Ϊ0~2^24(0~16777216)
		
************************************************************************/
#include "stm32f10x_Delay.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"


extern  uint32_t TimingDelay;//����delay���Ƶľ�ȷ��ʱ


void SysTick_Init(void)//delay��ʱ������ʼ��
{
	if (SysTick_Config(720))	// ST3.5.0��汾   �ж�ʱ�����720*1/72000000=10us
	{ 
		/* Capture error */ 
		while (1);
	}
		
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;// �رյδ�ʱ��  
}

void Delay_10us(uint32_t nTime) // nTime*10us ��ʱ����
{ 
	TimingDelay = nTime;	
	
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	// ʹ�ܵδ�ʱ��  

	while(TimingDelay != 0);
	
}



/*************************************************************************
*  �������ƣ� Initial_micros
*  ����˵���� ʹ������16λ�Ķ�ʱ�����ڼ�����32λ��ʱ��
*  ����˵���� ��
*
*************************************************************************/
 
void Initial_micros(void) 
{
	
	//		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 

/*    ���ֳ�ʼ����ʽ������
			TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
			TIM_TimeBaseStructure.TIM_Period = 0xffff; //�Զ���װֵ         
			TIM_TimeBaseStructure.TIM_Prescaler = 0x00;       
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;    
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
			TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); 

			TIM_PrescalerConfig(TIM2, 0, TIM_PSCReloadMode_Update);
			TIM_UpdateDisableConfig(TIM2, ENABLE);//Disable the TIM2 Update event


			TIM_SelectInputTrigger(TIM2, TIM_TS_ITR2);//Select the TIM2 Input Trigger: TIM3 TRGO used as Input Trigger for TIM2
			TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_External1);Use the External Clock as TIM2 Slave Mode
			TIM_SelectMasterSlaveMode(TIM2, TIM_MasterSlaveMode_Enable);//Enable the TIM2 Master Slave Mode
			TIM_ARRPreloadConfig(TIM2, ENABLE);	

			TIM_TimeBaseStructure.TIM_Period = 0xffff;     
			TIM_TimeBaseStructure.TIM_Prescaler = 71;	 //1M ��ʱ��  
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
			//Ӧ�����õ�TIM3 
			TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
			TIM_ARRPreloadConfig(TIM3, ENABLE);	// ʹ��TIM3���ؼĴ���ARR

			TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
			TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
			TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);// Use the TIM3 Update evet  as TIM3 Trigger Output(TRGO)
			TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);// Enable the TIM3 Master Slave Mode
*/
	 		TIM2->CR1= 0x0080; //���ƼĴ���1  �����Զ���װ��  
			TIM2->ARR = 0xffff;//�Զ���װ�ؼĴ���
			TIM2->PSC = 0x0000;//��Ƶ�Ĵ���
			TIM2->EGR = 0X0001;//UGλ��1  ���³�ʼ����������������һ�������¼���
			TIM2->SMCR = 0X00A7;//����ģʽʹ�ܣ�MSM=1��  ѡ�Ŵ�������Դ��TIM3   TS[2:0]=010��  	ѡ���ⲿʱ��ģʽ
			
			TIM3->CR1= 0x0084;//�����Զ���װ��      ����Դ����Ϊ�����ʽ    
			TIM3->CR2= 0x0020;//MSN[2:0]  010  ����Ϊ���Ӷ�ʱ��ģʽ
			TIM3->ARR = 0xffff;
			TIM3->PSC = 71;
			TIM3->EGR = 0X0001;
			TIM3->SMCR =0x0080;;//����ģʽʹ�ܣ�MSM=1�� �رմӻ�ģʽ SMS[2:0] = 000  ��Ϊ���Լ����Ǵӻ��� ����û���ٽӴӻ�
 			
			TIM3->CR1|=0X0001;//������ʱ��
			TIM2->CR1|=0X0001;//������ʱ��
			


}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		uint32_t micros(void)
*��������:	  ��ȡϵͳ���е�ʱ�� �����ص�λΪus ��ʱ������	
�����������
�����������������ǰʱ�䣬���ϵ翪ʼ��ʱ  ��λ us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM2->CNT; //����16λʱ��
 	temp = temp<<16;
 	temp += TIM3->CNT; //����16λʱ��
 	return temp;
}
