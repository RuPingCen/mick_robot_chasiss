/************************************************************************
*  作者 :CRP
*  功能说明: 将官方库修改以后 添加了自己的库
*  参数说明: uint32_t 32位数
  
*  修改时间: 2014-7-7     
*  备    注: 千万不能把变量 “TimingDelay”删掉
* 常使用函数:
		void SysTick_Init(void);//delay延时函数初始化
		void Delay_10us(uint32_t nTime); // nTime*10us 延时函数 传入参数为0~2^24(0~16777216)
		
************************************************************************/
#include "stm32f10x_Delay.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rcc.h"


extern  uint32_t TimingDelay;//用于delay控制的精确延时


void SysTick_Init(void)//delay延时函数初始化
{
	if (SysTick_Config(720))	// ST3.5.0库版本   中断时间等于720*1/72000000=10us
	{ 
		/* Capture error */ 
		while (1);
	}
		
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;// 关闭滴答定时器  
}

void Delay_10us(uint32_t nTime) // nTime*10us 延时函数
{ 
	TimingDelay = nTime;	
	
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;	// 使能滴答定时器  

	while(TimingDelay != 0);
	
}



/*************************************************************************
*  函数名称： Initial_micros
*  功能说明： 使用两个16位的定时器用于级联成32位定时器
*  参数说明： 无
*
*************************************************************************/
 
void Initial_micros(void) 
{
	
	//		TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 

/*    两种初始化方式都可以
			TIM_TimeBaseStructInit(&TIM_TimeBaseStructure); 
			TIM_TimeBaseStructure.TIM_Period = 0xffff; //自动重装值         
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
			TIM_TimeBaseStructure.TIM_Prescaler = 71;	 //1M 的时钟  
			TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;	
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
			//应用配置到TIM3 
			TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);	
			TIM_ARRPreloadConfig(TIM3, ENABLE);	// 使能TIM3重载寄存器ARR

			TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
			TIM_UpdateRequestConfig(TIM3, TIM_UpdateSource_Regular);
			TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);// Use the TIM3 Update evet  as TIM3 Trigger Output(TRGO)
			TIM_SelectMasterSlaveMode(TIM3, TIM_MasterSlaveMode_Enable);// Enable the TIM3 Master Slave Mode
*/
	 		TIM2->CR1= 0x0080; //控制寄存器1  设置自动重装载  
			TIM2->ARR = 0xffff;//自动重装载寄存器
			TIM2->PSC = 0x0000;//分频寄存器
			TIM2->EGR = 0X0001;//UG位置1  重新初始化计数器，并产生一个更新事件。
			TIM2->SMCR = 0X00A7;//主从模式使能（MSM=1）  选着触发输入源（TIM3   TS[2:0]=010）  	选着外部时钟模式
			
			TIM3->CR1= 0x0084;//设置自动重装载      更新源设置为溢出方式    
			TIM3->CR2= 0x0020;//MSN[2:0]  010  设置为主从定时器模式
			TIM3->ARR = 0xffff;
			TIM3->PSC = 71;
			TIM3->EGR = 0X0001;
			TIM3->SMCR =0x0080;;//主从模式使能（MSM=1） 关闭从机模式 SMS[2:0] = 000  因为他自己就是从机了 后面没有再接从机
 			
			TIM3->CR1|=0X0001;//启动定时器
			TIM2->CR1|=0X0001;//启动定时器
			


}
/**************************实现函数********************************************
*函数原型:		uint32_t micros(void)
*功　　能:	  读取系统运行的时间 ，返回单位为us 的时间数。	
输入参数：无
输出参数：处理器当前时间，从上电开始计时  单位 us
*******************************************************************************/
uint32_t micros(void)
{
 	uint32_t temp=0 ;
 	temp = TIM2->CNT; //读高16位时间
 	temp = temp<<16;
 	temp += TIM3->CNT; //读低16位时间
 	return temp;
}
