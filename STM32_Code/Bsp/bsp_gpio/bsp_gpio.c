 /*************************************************************************
*  功能说明: 将官方库修改以后 添加了自己的库
*  参数说明:GPIOx: where x can be (A..G) to select the GPIO peripheral.

			GPIO_Pin_x:  where x can be (0..15)
			
			GPIO_Mod:    GPIO_Mode_IN 			   模拟输入   
						 GPIO_Mode_OUT         输出
						 GPIO_Mode_AF    	  
						 GPIO_Mode_AN   		 

			 GPIO_Speed:		
							GPIO_Speed_2MHz  
							GPIO_Speed_25MHz
							GPIO_Speed_50MHz	
							GPIO_Speed_100MHz
								 
			 Trigger_mod:   0  EXTI_Trigger_Rising; //上升沿中断 
							1  EXTI_Trigger_Falling; //下降沿中断 
							2  EXTI_Trigger_Rising_Falling; //上升沿与下降沿中断	

* 常使用函数:    

		void My_GPIO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mod);//自定义初始化函数
		
		uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);//读取某一个引脚电平
		uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);//读取某一个端口的数据
		
		void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);//置位一个引脚  使引脚输出‘1’
		void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);//复位一个引脚  使引脚输出‘0’
		void GPIO_ToggleBits(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x);//引脚电平翻转
		void GPIO_Flip_level(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x);
		
		void My_GPIO_Exit_Init(GPIO_TypeDef* GPIOx,uint16_t pin_x , uint8_t Trigger_mod);//GPIO外部中断初始化函数
		
		
*  作者 :CRP
*  修改时间: 2020-2-3    
*  备    注: 无
*************************************************************************/

#include "bsp_gpio.h"
 
void My_GPIO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mod)
{	
	GPIO_InitTypeDef GPIO_InitStructure;/*定义一个GPIO_InitTypeDef类型的结构体*/
	uint32_t RCC_temp;/*定义一个uint32_t类型变量*/

	if(GPIOx==GPIOA) RCC_temp=RCC_AHB1Periph_GPIOA;//启动端口时钟
	else if(GPIOx==GPIOB) RCC_temp=RCC_AHB1Periph_GPIOB;
	else if(GPIOx==GPIOC) RCC_temp=RCC_AHB1Periph_GPIOC;
	else if(GPIOx==GPIOD) RCC_temp=RCC_AHB1Periph_GPIOD;
	else if(GPIOx==GPIOE) RCC_temp=RCC_AHB1Periph_GPIOE;
	else if(GPIOx==GPIOF) RCC_temp=RCC_AHB1Periph_GPIOF;
	else if(GPIOx==GPIOG) RCC_temp=RCC_AHB1Periph_GPIOG;
	else if(GPIOx==GPIOH) RCC_temp=RCC_AHB1Periph_GPIOH;
	else if(GPIOx==GPIOI) RCC_temp=RCC_AHB1Periph_GPIOI;
	else if(GPIOx==GPIOJ) RCC_temp=RCC_AHB1Periph_GPIOJ;
	else if(GPIOx==GPIOK) RCC_temp=RCC_AHB1Periph_GPIOK;
	else ;

	RCC_AHB1PeriphClockCmd(RCC_temp, ENABLE);/*开启GPIOX的外设时钟*/ 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;	/*选择要控制的GPIOX引脚*/	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mod; 	/*设置引脚模式*/
  
	if(GPIO_Mod==GPIO_Mode_OUT)
	{
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   /* 默认采用推挽输出*/
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     /* 默认上拉*/
		GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed; 		/*输出模式下需要设置引脚速率  25MHz*/ 
	}	
	else if(GPIO_Mod==GPIO_Mode_IN)
	{
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     /* 默认上拉*/
		GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed; 		/*输出模式下需要设置引脚速率  25MHz*/ 
	}		
	 
	GPIO_Init(GPIOx, &GPIO_InitStructure);			/*调用库函数，初始化GPIO*/

	if(GPIO_Mod==GPIO_Mode_OUT)
		GPIO_SetBits(GPIOx, GPIO_Pin_x);	/*如果为输出模式则初始化输出高电平	*/	 
}

void GPIO_Flip_level(GPIO_TypeDef* GPIOx,uint16_t pin_x)
{
			GPIOx->ODR ^=pin_x;//引脚电平翻转
}
/********************************************************************
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin_0:  where x can be (0..15)
  * @param  Trigger_mod:  0  EXTI_Trigger_Rising; //上升沿中断 
													1  EXTI_Trigger_Falling; //下降沿中断 
												  2  EXTI_Trigger_Rising_Falling; //上升沿与下降沿中断	
   EXTI0-EXTI15用于GPIO外部触发中断
   EXTI15-EXTI20 用于其他外部触发中断
***********************************************************************/
 
void My_GPIO_Exit_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x , uint8_t Trigger_mod)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint32_t RCC_temp;/*定义一个uint32_t类型变量*/
	uint8_t my_GPIO_PortSource;
	uint8_t my_GPIO_PinSource;


	if(GPIOx==GPIOA)      { RCC_temp=RCC_AHB1Periph_GPIOA; my_GPIO_PortSource = EXTI_PortSourceGPIOA;}//启动端口时钟
	else if(GPIOx==GPIOB) { RCC_temp=RCC_AHB1Periph_GPIOB; my_GPIO_PortSource = EXTI_PortSourceGPIOB;}
	else if(GPIOx==GPIOC) { RCC_temp=RCC_AHB1Periph_GPIOC; my_GPIO_PortSource = EXTI_PortSourceGPIOC;}
	else if(GPIOx==GPIOD) { RCC_temp=RCC_AHB1Periph_GPIOD; my_GPIO_PortSource = EXTI_PortSourceGPIOD;}
	else if(GPIOx==GPIOE) { RCC_temp=RCC_AHB1Periph_GPIOE; my_GPIO_PortSource = EXTI_PortSourceGPIOE;}
	else if(GPIOx==GPIOF) { RCC_temp=RCC_AHB1Periph_GPIOF; my_GPIO_PortSource = EXTI_PortSourceGPIOF;}
	else if(GPIOx==GPIOG) { RCC_temp=RCC_AHB1Periph_GPIOG; my_GPIO_PortSource = EXTI_PortSourceGPIOG;}
	else if(GPIOx==GPIOH) { RCC_temp=RCC_AHB1Periph_GPIOH; my_GPIO_PortSource = EXTI_PortSourceGPIOH;}
	else if(GPIOx==GPIOI) { RCC_temp=RCC_AHB1Periph_GPIOI; my_GPIO_PortSource = EXTI_PortSourceGPIOI;}
	else if(GPIOx==GPIOJ) { RCC_temp=RCC_AHB1Periph_GPIOJ; my_GPIO_PortSource = EXTI_PortSourceGPIOJ;}
	else if(GPIOx==GPIOK) { RCC_temp=RCC_AHB1Periph_GPIOK; my_GPIO_PortSource = EXTI_PortSourceGPIOK;}
	else ;

	RCC_AHB1PeriphClockCmd(RCC_temp, ENABLE);/*开启GPIOX的外设时钟*/ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	 if(GPIO_Pin_x==GPIO_Pin_0)      { NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;/* 配置中断源 */my_GPIO_PinSource=EXTI_PinSource0;EXTI_InitStructure.EXTI_Line = EXTI_Line0;}//配置中断线
	 else if(GPIO_Pin_x==GPIO_Pin_1) { NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;my_GPIO_PinSource=EXTI_PinSource1;EXTI_InitStructure.EXTI_Line = EXTI_Line1;}
	 else if(GPIO_Pin_x==GPIO_Pin_2) { NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;my_GPIO_PinSource=EXTI_PinSource2;EXTI_InitStructure.EXTI_Line = EXTI_Line2;}
	 else if(GPIO_Pin_x==GPIO_Pin_3) { NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;my_GPIO_PinSource=EXTI_PinSource3;EXTI_InitStructure.EXTI_Line = EXTI_Line3;}
	 else if(GPIO_Pin_x==GPIO_Pin_4) { NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;my_GPIO_PinSource=EXTI_PinSource4;EXTI_InitStructure.EXTI_Line = EXTI_Line4;}
	 else if(GPIO_Pin_x==GPIO_Pin_5) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=EXTI_PinSource5;EXTI_InitStructure.EXTI_Line = EXTI_Line5;}
	 else if(GPIO_Pin_x==GPIO_Pin_6) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=EXTI_PinSource6;EXTI_InitStructure.EXTI_Line = EXTI_Line6;}
	 else if(GPIO_Pin_x==GPIO_Pin_7) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=EXTI_PinSource7;EXTI_InitStructure.EXTI_Line = EXTI_Line7;}
	 else if(GPIO_Pin_x==GPIO_Pin_8) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=EXTI_PinSource8;EXTI_InitStructure.EXTI_Line = EXTI_Line8;}
	 else if(GPIO_Pin_x==GPIO_Pin_9) { NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=EXTI_PinSource9;EXTI_InitStructure.EXTI_Line = EXTI_Line9;}
	 else if(GPIO_Pin_x==GPIO_Pin_10){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=EXTI_PinSource10;EXTI_InitStructure.EXTI_Line = EXTI_Line10;}
	 else if(GPIO_Pin_x==GPIO_Pin_11){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=EXTI_PinSource11;EXTI_InitStructure.EXTI_Line = EXTI_Line11;}
	 else if(GPIO_Pin_x==GPIO_Pin_12){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=EXTI_PinSource12;EXTI_InitStructure.EXTI_Line = EXTI_Line12;}
	 else if(GPIO_Pin_x==GPIO_Pin_13){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=EXTI_PinSource13;EXTI_InitStructure.EXTI_Line = EXTI_Line13;}
	 else if(GPIO_Pin_x==GPIO_Pin_14){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=EXTI_PinSource14;EXTI_InitStructure.EXTI_Line = EXTI_Line14;}
	 else if(GPIO_Pin_x==GPIO_Pin_15){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=EXTI_PinSource15;EXTI_InitStructure.EXTI_Line = EXTI_Line15;}
	 else ;
	 
	 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); /* Configure one bit for preemption priority */
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//配置抢占优先级
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//配置响应优先级
	 /*************************************************************
		 对于有多个IO中断时 不必再修改这里的中断优先级、相应优先级 
		 在STM32中当中断优先级、相应优先级相同时会按照内部中断向量的序号响应 
	 ************************************************************/
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);


	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;  /* EXTI line gpio config*/	      
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// 无上拉输入
	 GPIO_Init(GPIOx, &GPIO_InitStructure);
	 
		 
	 SYSCFG_EXTILineConfig(my_GPIO_PortSource, my_GPIO_PinSource); 	/* EXTI line mode config */	

	 if(Trigger_mod==0)EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿中断 
	 else if(Trigger_mod==1)EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断 
	 else if(Trigger_mod==2)EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //上升沿与下降沿中断
	 else ;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	//EXTI_InitStructure.EXTI_Trigger = Trigger_mod; //下降沿中断EXTI_Trigger_Falling
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure); 	
}
