 /*************************************************************************
*  ����˵��: �δ�ʱ��
*  ���� :�CCRP
*  �޸�ʱ��: 2023-2-3    
*  ��    ע: ��
*************************************************************************/

#include "bsp_systick.h"
#include "bsp_timer.h"


volatile uint32_t TimingDelay;//����delay���Ƶľ�ȷ��ʱ

 
void SysTick_Init(void)//delay��ʱ������ʼ��
{
	/* SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	if(SysTick_Config(SystemCoreClock/100000))	//   
	{ 
		/* Capture error */ 
		while (1);
	}
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;// �رյδ�ʱ��  
}

void Delay_10ms(__IO u32 nTime)
{ 
// Bsp\bsp_eth\stm32f4x7_eth_conf.h ����������õ��������
	Delay_10us(nTime*1000);
}
void delay_ms(__IO u32 nTime)
{ 
// Bsp\bsp_eth\stm32f4x7_eth_conf.h ����������õ��������
	Delay_10us(nTime*100);
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 


	TIM2->CR1= 0x0080; //���ƼĴ���1  �����Զ���װ��  
	TIM2->ARR = 0xffff;//�Զ���װ�ؼĴ���
	TIM2->PSC = 0x0000;//��Ƶ�Ĵ���
	TIM2->EGR = 0X0001;//UGλ��1  ���³�ʼ����������������һ�������¼���
	TIM2->SMCR = 0X00A7;//����ģʽʹ�ܣ�MSM=1��  ѡ�Ŵ�������Դ��TIM3   TS[2:0]=010��  	ѡ���ⲿʱ��ģʽ

	TIM3->CR1= 0x0084;//�����Զ���װ��      ����Դ����Ϊ�����ʽ    
	TIM3->CR2= 0x0020;//MSN[2:0]  010  ����Ϊ���Ӷ�ʱ��ģʽ
	TIM3->ARR = 0xffff;
	TIM3->PSC = 83;
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