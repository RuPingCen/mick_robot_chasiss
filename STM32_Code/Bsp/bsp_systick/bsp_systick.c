 /*************************************************************************
*  功能说明: 滴答定时器
*  作者 :CRP
*  修改时间: 2023-2-3    
*  备    注: 无
*************************************************************************/

#include "bsp_systick.h"
#include "bsp_timer.h"


volatile uint32_t TimingDelay;//用于delay控制的精确延时

 
void SysTick_Init(void)//delay延时函数初始化
{
	/* SystemFrequency / 1000    1ms中断一次
	 * SystemFrequency / 100000	 10us中断一次
	 * SystemFrequency / 1000000 1us中断一次
	 */
	if(SysTick_Config(SystemCoreClock/100000))	//   
	{ 
		/* Capture error */ 
		while (1);
	}
	SysTick->CTRL &= ~ SysTick_CTRL_ENABLE_Msk;// 关闭滴答定时器  
}

void Delay_10ms(__IO u32 nTime)
{ 
// Bsp\bsp_eth\stm32f4x7_eth_conf.h 这个函数中用到这个函数
	Delay_10us(nTime*1000);
}
void delay_ms(__IO u32 nTime)
{ 
// Bsp\bsp_eth\stm32f4x7_eth_conf.h 这个函数中用到这个函数
	Delay_10us(nTime*100);
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
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE); 


	TIM2->CR1= 0x0080; //控制寄存器1  设置自动重装载  
	TIM2->ARR = 0xffff;//自动重装载寄存器
	TIM2->PSC = 0x0000;//分频寄存器
	TIM2->EGR = 0X0001;//UG位置1  重新初始化计数器，并产生一个更新事件。
	TIM2->SMCR = 0X00A7;//主从模式使能（MSM=1）  选着触发输入源（TIM3   TS[2:0]=010）  	选着外部时钟模式

	TIM3->CR1= 0x0084;//设置自动重装载      更新源设置为溢出方式    
	TIM3->CR2= 0x0020;//MSN[2:0]  010  设置为主从定时器模式
	TIM3->ARR = 0xffff;
	TIM3->PSC = 83;
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