 /*************************************************************************
*  作者 :CRP
*  功能说明: 将官方库修改以后 添加了自己的库
*  参数说明:GPIOx: where x can be (A..G) to select the GPIO peripheral.

						GPIO_Pin_x:  where x can be (0..15)
						
						GPIO_Mod:  GPIO_Mode_AIN 			   模拟输入   
											 GPIO_Mode_IN_FLOATING 浮空输入，复位后的状态 
											 GPIO_Mode_IPD 				 下拉输入 
											 GPIO_Mode_IPU  			 上拉输入 
											 GPIO_Mode_Out_OD  	   开漏输出 
											 GPIO_Mode_Out_PP 		 推挽输出 
											 GPIO_Mode_AF_OD    	 复用开漏输出 
											 GPIO_Mode_AF_PP   		 复用推挽输出 
											 
						 GPIO_Speed:		
                       GPIO_Speed_10MHz  
											 GPIO_Speed_2MHz
											 GPIO_Speed_50MHz	
											 
						 Trigger_mod:  0  EXTI_Trigger_Rising; //上升沿中断 
											  	 1  EXTI_Trigger_Falling; //下降沿中断 
											 	   2  EXTI_Trigger_Rising_Falling; //上升沿与下降沿中断	

*  修改时间: 2014-7-7     
*  备    注: 无
* 常使用函数:   My_GPIO_Init(GPIOx,GPIO_Pin_x,GPIO_Mode_IPU,GPIO_Speed_50MHz);//自定义初始化函数

		void My_GPIO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mod,GPIOSpeed_TypeDef GPIO_Speed);//自定义初始化函数
		uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);//读取某一个引脚电平
		uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);//读取某一个端口的数据
		void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);//置位一个引脚  使引脚输出‘1’
		void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);//复位一个引脚  使引脚输出‘0’
		void GPIO_Flip_level(GPIO_TypeDef* GPIOx,uint16_t pin_x);//引脚电平翻转
		
		void My_GPIO_Exit_Init(GPIO_TypeDef* GPIOx,uint16_t pin_x , uint8_t Trigger_mod);//GPIO外部中断初始化函数
  
*************************************************************************/
 








/*****************************函数定义区**************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h" 
#include "misc.h" 
/* ------------ RCC registers bit address in the alias region ----------------*/
#define AFIO_OFFSET                 (AFIO_BASE - PERIPH_BASE)

/* --- EVENTCR Register -----*/

/* Alias word address of EVOE bit */
#define EVCR_OFFSET                 (AFIO_OFFSET + 0x00)
#define EVOE_BitNumber              ((uint8_t)0x07)
#define EVCR_EVOE_BB                (PERIPH_BB_BASE + (EVCR_OFFSET * 32) + (EVOE_BitNumber * 4))


/* ---  MAPR Register ---*/ 
/* Alias word address of MII_RMII_SEL bit */ 
#define MAPR_OFFSET                 (AFIO_OFFSET + 0x04) 
#define MII_RMII_SEL_BitNumber      ((u8)0x17) 
#define MAPR_MII_RMII_SEL_BB        (PERIPH_BB_BASE + (MAPR_OFFSET * 32) + (MII_RMII_SEL_BitNumber * 4))


#define EVCR_PORTPINCONFIG_MASK     ((uint16_t)0xFF80)
#define LSB_MASK                    ((uint16_t)0xFFFF)
#define DBGAFR_POSITION_MASK        ((uint32_t)0x000F0000)
#define DBGAFR_SWJCFG_MASK          ((uint32_t)0xF0FFFFFF)
#define DBGAFR_LOCATION_MASK        ((uint32_t)0x00200000)
#define DBGAFR_NUMBITS_MASK         ((uint32_t)0x00100000)

/********************************************************************
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin_0:  where x can be (0..15)
  * @param  GPIO_Mod:  GPIO_Mode_AIN 			   模拟输入   
											 GPIO_Mode_IN_FLOATING 浮空输入，复位后的状态 
											 GPIO_Mode_IPD 				 下拉输入 
											 GPIO_Mode_IPU  			 上拉输入 
											 GPIO_Mode_Out_OD  	   开漏输出 
											 GPIO_Mode_Out_PP 		 推挽输出 
											 GPIO_Mode_AF_OD    	 复用开漏输出 
											 GPIO_Mode_AF_PP   		 复用推挽输出 
 * @param  GPIO_Speed:		
                       GPIO_Speed_10MHz  
											 GPIO_Speed_2MHz
											 GPIO_Speed_50MHz	
  * @retval The input port pin value.
***********************************************************************/
void My_GPIO_Init(GPIO_TypeDef* GPIOx,uint16_t pin_x,GPIOMode_TypeDef GPIO_Mod,GPIOSpeed_TypeDef GPIO_Speed)
{	
		GPIO_InitTypeDef GPIO_InitStructure;/*定义一个GPIO_InitTypeDef类型的结构体*/
	
	  uint32_t RCC_temp;/*定义一个uint32_t类型变量*/
		
	 if(GPIOx==GPIOA) RCC_temp=RCC_APB2Periph_GPIOA;//启动端口时钟
	 else if(GPIOx==GPIOB) RCC_temp=RCC_APB2Periph_GPIOB;
	 else if(GPIOx==GPIOC) RCC_temp=RCC_APB2Periph_GPIOC;
	 else if(GPIOx==GPIOD) RCC_temp=RCC_APB2Periph_GPIOD;
	 else if(GPIOx==GPIOE) RCC_temp=RCC_APB2Periph_GPIOE;
	 else if(GPIOx==GPIOF) RCC_temp=RCC_APB2Periph_GPIOF;
	 else if(GPIOx==GPIOG) RCC_temp=RCC_APB2Periph_GPIOG;
	 else ;
	
		RCC_APB2PeriphClockCmd( RCC_temp, ENABLE);/*开启GPIOX的外设时钟*/ 
													   
		GPIO_InitStructure.GPIO_Pin = pin_x;	/*选择要控制的GPIOX引脚*/	
	
		GPIO_InitStructure.GPIO_Mode = GPIO_Mod; 	/*设置引脚模式*/  
	
  	if((GPIO_Mod==GPIO_Mode_Out_OD) ||(GPIO_Mod==GPIO_Mode_Out_PP)||(GPIO_Mod==GPIO_Mode_AF_OD)||(GPIO_Mod==GPIO_Mode_AF_PP))
			
	  	 GPIO_InitStructure.GPIO_Speed = GPIO_Speed; 		/*输出模式下需要设置引脚速率*/ 

		GPIO_Init(GPIOx, &GPIO_InitStructure);			/*调用库函数，初始化GPIO*/
		
		if((GPIO_Mod==GPIO_Mode_Out_OD) ||(GPIO_Mod==GPIO_Mode_Out_PP)||(GPIO_Mod==GPIO_Mode_AF_OD)||(GPIO_Mod==GPIO_Mode_AF_PP))
			
		   GPIO_SetBits(GPIOx, pin_x);	/*如果为输出模式则初始化输出高电平	*/	 
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
***********************************************************************/
 
void My_GPIO_Exit_Init(GPIO_TypeDef* GPIOx,uint16_t pin_x , uint8_t Trigger_mod)
{
	   GPIO_InitTypeDef GPIO_InitStructure; 
	   EXTI_InitTypeDef EXTI_InitStructure;
	   NVIC_InitTypeDef NVIC_InitStructure;
	   uint32_t RCC_temp;/*定义一个uint32_t类型变量*/
	   uint8_t my_GPIO_PortSource;
	   uint8_t my_GPIO_PinSource;
	   if(GPIOx==GPIOA) {RCC_temp=RCC_APB2Periph_GPIOA;my_GPIO_PortSource=GPIO_PortSourceGPIOA;}
	   else if(GPIOx==GPIOB) {RCC_temp=RCC_APB2Periph_GPIOB;my_GPIO_PortSource=GPIO_PortSourceGPIOB;}
	   else if(GPIOx==GPIOC) {RCC_temp=RCC_APB2Periph_GPIOC;my_GPIO_PortSource=GPIO_PortSourceGPIOC;}
	   else if(GPIOx==GPIOD) {RCC_temp=RCC_APB2Periph_GPIOD;my_GPIO_PortSource=GPIO_PortSourceGPIOD;}
	   else if(GPIOx==GPIOE) {RCC_temp=RCC_APB2Periph_GPIOE;my_GPIO_PortSource=GPIO_PortSourceGPIOE;}
	   else if(GPIOx==GPIOF) {RCC_temp=RCC_APB2Periph_GPIOF;my_GPIO_PortSource=GPIO_PortSourceGPIOF;}
	   else if(GPIOx==GPIOG) {RCC_temp=RCC_APB2Periph_GPIOG;my_GPIO_PortSource=GPIO_PortSourceGPIOG;}
     else ;	  
		 RCC_APB2PeriphClockCmd(RCC_temp | RCC_APB2Periph_AFIO,ENABLE);


  
     if(pin_x==GPIO_Pin_0){ NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;/* 配置中断源 */my_GPIO_PinSource=GPIO_PinSource0;EXTI_InitStructure.EXTI_Line = EXTI_Line0;}//配置中断线
	   else if(pin_x==GPIO_Pin_1){ NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;my_GPIO_PinSource=GPIO_PinSource1;EXTI_InitStructure.EXTI_Line = EXTI_Line1;}
		 else if(pin_x==GPIO_Pin_2){ NVIC_InitStructure.NVIC_IRQChannel = EXTI2_IRQn;my_GPIO_PinSource=GPIO_PinSource2;EXTI_InitStructure.EXTI_Line = EXTI_Line2;}
		 else if(pin_x==GPIO_Pin_3){ NVIC_InitStructure.NVIC_IRQChannel = EXTI3_IRQn;my_GPIO_PinSource=GPIO_PinSource3;EXTI_InitStructure.EXTI_Line = EXTI_Line3;}
		 else if(pin_x==GPIO_Pin_4){ NVIC_InitStructure.NVIC_IRQChannel = EXTI4_IRQn;my_GPIO_PinSource=GPIO_PinSource4;EXTI_InitStructure.EXTI_Line = EXTI_Line4;}
		 else if(pin_x==GPIO_Pin_5){ NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=GPIO_PinSource5;EXTI_InitStructure.EXTI_Line = EXTI_Line5;}
		 else if(pin_x==GPIO_Pin_6){ NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=GPIO_PinSource6;EXTI_InitStructure.EXTI_Line = EXTI_Line6;}
		 else if(pin_x==GPIO_Pin_7){ NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=GPIO_PinSource7;EXTI_InitStructure.EXTI_Line = EXTI_Line7;}
		 else if(pin_x==GPIO_Pin_8){ NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=GPIO_PinSource8;EXTI_InitStructure.EXTI_Line = EXTI_Line8;}
		 else if(pin_x==GPIO_Pin_9){ NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;my_GPIO_PinSource=GPIO_PinSource9;EXTI_InitStructure.EXTI_Line = EXTI_Line9;}
		 else if(pin_x==GPIO_Pin_10){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=GPIO_PinSource10;EXTI_InitStructure.EXTI_Line = EXTI_Line10;}
		 else if(pin_x==GPIO_Pin_11){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=GPIO_PinSource11;EXTI_InitStructure.EXTI_Line = EXTI_Line11;}
		 else if(pin_x==GPIO_Pin_12){NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=GPIO_PinSource12;EXTI_InitStructure.EXTI_Line = EXTI_Line12;}
		 else if(pin_x==GPIO_Pin_13){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=GPIO_PinSource13;EXTI_InitStructure.EXTI_Line = EXTI_Line13;}
		 else if(pin_x==GPIO_Pin_14){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=GPIO_PinSource14;EXTI_InitStructure.EXTI_Line = EXTI_Line14;}
		 else if(pin_x==GPIO_Pin_15){ NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;my_GPIO_PinSource=GPIO_PinSource15;EXTI_InitStructure.EXTI_Line = EXTI_Line15;}
		 else ;
		 
		 NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); /* Configure one bit for preemption priority */
		
		 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;//配置抢占优先级
		 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;//配置响应优先级
		 /*************************************************************
		   对于有多个IO中断时 不必再修改这里的中断优先级、相应优先级 
		   在STM32中当中断优先级、相应优先级相同时会按照内部中断向量的序号响应 
		 ************************************************************/
		 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		 NVIC_Init(&NVIC_InitStructure);
	
	  
		 GPIO_InitStructure.GPIO_Pin = pin_x;  /* EXTI line gpio config*/	      
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	 // 上拉输入
		 GPIO_Init(GPIOA, &GPIO_InitStructure);
		 
		 
		 
		 
		 
		 
		 GPIO_EXTILineConfig(my_GPIO_PortSource, my_GPIO_PinSource); 	/* EXTI line mode config */	
		
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

void GPIO_DeInit(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  if (GPIOx == GPIOA)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOA, DISABLE);
  }
  else if (GPIOx == GPIOB)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOB, DISABLE);
  }
  else if (GPIOx == GPIOC)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOC, DISABLE);
  }
  else if (GPIOx == GPIOD)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOD, DISABLE);
  }    
  else if (GPIOx == GPIOE)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOE, DISABLE);
  } 
  else if (GPIOx == GPIOF)
  {
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOF, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOF, DISABLE);
  }
  else
  {
    if (GPIOx == GPIOG)
    {
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOG, ENABLE);
      RCC_APB2PeriphResetCmd(RCC_APB2Periph_GPIOG, DISABLE);
    }
  }
}

/**
  * @brief  Deinitializes the Alternate Functions (remap, event control
  *   and EXTI configuration) registers to their default reset values.
  * @param  None
  * @retval None
  */
void GPIO_AFIODeInit(void)
{
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO, DISABLE);
}

/**
  * @brief  Initializes the GPIOx peripheral according to the specified
  *         parameters in the GPIO_InitStruct.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_InitStruct: pointer to a GPIO_InitTypeDef structure that
  *         contains the configuration information for the specified GPIO peripheral.
  * @retval None
  */
void GPIO_Init(GPIO_TypeDef* GPIOx, GPIO_InitTypeDef* GPIO_InitStruct)
{
  uint32_t currentmode = 0x00, currentpin = 0x00, pinpos = 0x00, pos = 0x00;
  uint32_t tmpreg = 0x00, pinmask = 0x00;
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_MODE(GPIO_InitStruct->GPIO_Mode));
  assert_param(IS_GPIO_PIN(GPIO_InitStruct->GPIO_Pin));  
  
/*---------------------------- GPIO Mode Configuration -----------------------*/
  currentmode = ((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x0F);
  if ((((uint32_t)GPIO_InitStruct->GPIO_Mode) & ((uint32_t)0x10)) != 0x00)
  { 
    /* Check the parameters */
    assert_param(IS_GPIO_SPEED(GPIO_InitStruct->GPIO_Speed));
    /* Output mode */
    currentmode |= (uint32_t)GPIO_InitStruct->GPIO_Speed;
  }
/*---------------------------- GPIO CRL Configuration ------------------------*/
  /* Configure the eight low port pins */
  if (((uint32_t)GPIO_InitStruct->GPIO_Pin & ((uint32_t)0x00FF)) != 0x00)
  {
    tmpreg = GPIOx->CRL;
    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = ((uint32_t)0x01) << pinpos;
      /* Get the port pins position */
      currentpin = (GPIO_InitStruct->GPIO_Pin) & pos;
      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding low control register bits */
        pinmask = ((uint32_t)0x0F) << pos;
        tmpreg &= ~pinmask;
        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);
        /* Reset the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
        {
          GPIOx->BRR = (((uint32_t)0x01) << pinpos);
        }
        else
        {
          /* Set the corresponding ODR bit */
          if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
          {
            GPIOx->BSRR = (((uint32_t)0x01) << pinpos);
          }
        }
      }
    }
    GPIOx->CRL = tmpreg;
  }
/*---------------------------- GPIO CRH Configuration ------------------------*/
  /* Configure the eight high port pins */
  if (GPIO_InitStruct->GPIO_Pin > 0x00FF)
  {
    tmpreg = GPIOx->CRH;
    for (pinpos = 0x00; pinpos < 0x08; pinpos++)
    {
      pos = (((uint32_t)0x01) << (pinpos + 0x08));
      /* Get the port pins position */
      currentpin = ((GPIO_InitStruct->GPIO_Pin) & pos);
      if (currentpin == pos)
      {
        pos = pinpos << 2;
        /* Clear the corresponding high control register bits */
        pinmask = ((uint32_t)0x0F) << pos;
        tmpreg &= ~pinmask;
        /* Write the mode configuration in the corresponding bits */
        tmpreg |= (currentmode << pos);
        /* Reset the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPD)
        {
          GPIOx->BRR = (((uint32_t)0x01) << (pinpos + 0x08));
        }
        /* Set the corresponding ODR bit */
        if (GPIO_InitStruct->GPIO_Mode == GPIO_Mode_IPU)
        {
          GPIOx->BSRR = (((uint32_t)0x01) << (pinpos + 0x08));
        }
      }
    }
    GPIOx->CRH = tmpreg;
  }
}

/**
  * @brief  Fills each GPIO_InitStruct member with its default value.
  * @param  GPIO_InitStruct : pointer to a GPIO_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void GPIO_StructInit(GPIO_InitTypeDef* GPIO_InitStruct)
{
  /* Reset GPIO init structure parameters values */
  GPIO_InitStruct->GPIO_Pin  = GPIO_Pin_All;
  GPIO_InitStruct->GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStruct->GPIO_Mode = GPIO_Mode_IN_FLOATING;
}

/**
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin:  specifies the port bit to read.
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The input port pin value.
  */
uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint8_t bitstatus = 0x00;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin)); 
  
  if ((GPIOx->IDR & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

/**
  * @brief  Reads the specified GPIO input data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO input data port value.
  */
uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  return ((uint16_t)GPIOx->IDR);
}

/**
  * @brief  Reads the specified output data port bit.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin:  specifies the port bit to read.
  *   This parameter can be GPIO_Pin_x where x can be (0..15).
  * @retval The output port pin value.
  */
uint8_t GPIO_ReadOutputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint8_t bitstatus = 0x00;
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin)); 
  
  if ((GPIOx->ODR & GPIO_Pin) != (uint32_t)Bit_RESET)
  {
    bitstatus = (uint8_t)Bit_SET;
  }
  else
  {
    bitstatus = (uint8_t)Bit_RESET;
  }
  return bitstatus;
}

/**
  * @brief  Reads the specified GPIO output data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @retval GPIO output data port value.
  */
uint16_t GPIO_ReadOutputData(GPIO_TypeDef* GPIOx)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
    
  return ((uint16_t)GPIOx->ODR);
}

/**
  * @brief  Sets the selected data port bits.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BSRR = GPIO_Pin;
}

/**
  * @brief  Clears the selected data port bits.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bits to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  GPIOx->BRR = GPIO_Pin;
}

/**
  * @brief  Sets or clears the selected data port bit.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *   This parameter can be one of GPIO_Pin_x where x can be (0..15).
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be one of the BitAction enum values:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void GPIO_WriteBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, BitAction BitVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GET_GPIO_PIN(GPIO_Pin));
  assert_param(IS_GPIO_BIT_ACTION(BitVal)); 
  
  if (BitVal != Bit_RESET)
  {
    GPIOx->BSRR = GPIO_Pin;
  }
  else
  {
    GPIOx->BRR = GPIO_Pin;
  }
}

/**
  * @brief  Writes data to the specified GPIO data port.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  PortVal: specifies the value to be written to the port output data register.
  * @retval None
  */
void GPIO_Write(GPIO_TypeDef* GPIOx, uint16_t PortVal)
{
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  
  GPIOx->ODR = PortVal;
}

/**
  * @brief  Locks GPIO Pins configuration registers.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin: specifies the port bit to be written.
  *   This parameter can be any combination of GPIO_Pin_x where x can be (0..15).
  * @retval None
  */
void GPIO_PinLockConfig(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  uint32_t tmp = 0x00010000;
  
  /* Check the parameters */
  assert_param(IS_GPIO_ALL_PERIPH(GPIOx));
  assert_param(IS_GPIO_PIN(GPIO_Pin));
  
  tmp |= GPIO_Pin;
  /* Set LCKK bit */
  GPIOx->LCKR = tmp;
  /* Reset LCKK bit */
  GPIOx->LCKR =  GPIO_Pin;
  /* Set LCKK bit */
  GPIOx->LCKR = tmp;
  /* Read LCKK bit*/
  tmp = GPIOx->LCKR;
  /* Read LCKK bit*/
  tmp = GPIOx->LCKR;
}

/**
  * @brief  Selects the GPIO pin used as Event output.
  * @param  GPIO_PortSource: selects the GPIO port to be used as source
  *   for Event output.
  *   This parameter can be GPIO_PortSourceGPIOx where x can be (A..E).
  * @param  GPIO_PinSource: specifies the pin for the Event output.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @retval None
  */
void GPIO_EventOutputConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)
{
  uint32_t tmpreg = 0x00;
  /* Check the parameters */
  assert_param(IS_GPIO_EVENTOUT_PORT_SOURCE(GPIO_PortSource));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
    
  tmpreg = AFIO->EVCR;
  /* Clear the PORT[6:4] and PIN[3:0] bits */
  tmpreg &= EVCR_PORTPINCONFIG_MASK;
  tmpreg |= (uint32_t)GPIO_PortSource << 0x04;
  tmpreg |= GPIO_PinSource;
  AFIO->EVCR = tmpreg;
}

/**
  * @brief  Enables or disables the Event Output.
  * @param  NewState: new state of the Event output.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void GPIO_EventOutputCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  *(__IO uint32_t *) EVCR_EVOE_BB = (uint32_t)NewState;
}

/**
  * @brief  Changes the mapping of the specified pin.
  * @param  GPIO_Remap: selects the pin to remap.
  *   This parameter can be one of the following values:
  *     @arg GPIO_Remap_SPI1             : SPI1 Alternate Function mapping
  *     @arg GPIO_Remap_I2C1             : I2C1 Alternate Function mapping
  *     @arg GPIO_Remap_USART1           : USART1 Alternate Function mapping
  *     @arg GPIO_Remap_USART2           : USART2 Alternate Function mapping
  *     @arg GPIO_PartialRemap_USART3    : USART3 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_USART3       : USART3 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap_TIM1      : TIM1 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM1         : TIM1 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap1_TIM2     : TIM2 Partial1 Alternate Function mapping
  *     @arg GPIO_PartialRemap2_TIM2     : TIM2 Partial2 Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM2         : TIM2 Full Alternate Function mapping
  *     @arg GPIO_PartialRemap_TIM3      : TIM3 Partial Alternate Function mapping
  *     @arg GPIO_FullRemap_TIM3         : TIM3 Full Alternate Function mapping
  *     @arg GPIO_Remap_TIM4             : TIM4 Alternate Function mapping
  *     @arg GPIO_Remap1_CAN1            : CAN1 Alternate Function mapping
  *     @arg GPIO_Remap2_CAN1            : CAN1 Alternate Function mapping
  *     @arg GPIO_Remap_PD01             : PD01 Alternate Function mapping
  *     @arg GPIO_Remap_TIM5CH4_LSI      : LSI connected to TIM5 Channel4 input capture for calibration
  *     @arg GPIO_Remap_ADC1_ETRGINJ     : ADC1 External Trigger Injected Conversion remapping
  *     @arg GPIO_Remap_ADC1_ETRGREG     : ADC1 External Trigger Regular Conversion remapping
  *     @arg GPIO_Remap_ADC2_ETRGINJ     : ADC2 External Trigger Injected Conversion remapping
  *     @arg GPIO_Remap_ADC2_ETRGREG     : ADC2 External Trigger Regular Conversion remapping
  *     @arg GPIO_Remap_ETH              : Ethernet remapping (only for Connectivity line devices)
  *     @arg GPIO_Remap_CAN2             : CAN2 remapping (only for Connectivity line devices)
  *     @arg GPIO_Remap_SWJ_NoJTRST      : Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST
  *     @arg GPIO_Remap_SWJ_JTAGDisable  : JTAG-DP Disabled and SW-DP Enabled
  *     @arg GPIO_Remap_SWJ_Disable      : Full SWJ Disabled (JTAG-DP + SW-DP)
  *     @arg GPIO_Remap_SPI3             : SPI3/I2S3 Alternate Function mapping (only for Connectivity line devices)
  *                                        When the SPI3/I2S3 is remapped using this function, the SWJ is configured
  *                                        to Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST.   
  *     @arg GPIO_Remap_TIM2ITR1_PTP_SOF : Ethernet PTP output or USB OTG SOF (Start of Frame) connected
  *                                        to TIM2 Internal Trigger 1 for calibration (only for Connectivity line devices)
  *                                        If the GPIO_Remap_TIM2ITR1_PTP_SOF is enabled the TIM2 ITR1 is connected to 
  *                                        Ethernet PTP output. When Reset TIM2 ITR1 is connected to USB OTG SOF output.    
  *     @arg GPIO_Remap_PTP_PPS          : Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices)
  *     @arg GPIO_Remap_TIM15            : TIM15 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM16            : TIM16 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM17            : TIM17 Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_CEC              : CEC Alternate Function mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM1_DMA         : TIM1 DMA requests mapping (only for Value line devices)
  *     @arg GPIO_Remap_TIM9             : TIM9 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM10            : TIM10 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM11            : TIM11 Alternate Function mapping (only for XL-density devices)
  *     @arg GPIO_Remap_TIM13            : TIM13 Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_TIM14            : TIM14 Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_FSMC_NADV        : FSMC_NADV Alternate Function mapping (only for High density Value line and XL-density devices)
  *     @arg GPIO_Remap_TIM67_DAC_DMA    : TIM6/TIM7 and DAC DMA requests remapping (only for High density Value line devices)
  *     @arg GPIO_Remap_TIM12            : TIM12 Alternate Function mapping (only for High density Value line devices)
  *     @arg GPIO_Remap_MISC             : Miscellaneous Remap (DMA2 Channel5 Position and DAC Trigger remapping, 
  *                                        only for High density Value line devices)     
  * @param  NewState: new state of the port pin remapping.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void GPIO_PinRemapConfig(uint32_t GPIO_Remap, FunctionalState NewState)
{
  uint32_t tmp = 0x00, tmp1 = 0x00, tmpreg = 0x00, tmpmask = 0x00;

  /* Check the parameters */
  assert_param(IS_GPIO_REMAP(GPIO_Remap));
  assert_param(IS_FUNCTIONAL_STATE(NewState));  
  
  if((GPIO_Remap & 0x80000000) == 0x80000000)
  {
    tmpreg = AFIO->MAPR2;
  }
  else
  {
    tmpreg = AFIO->MAPR;
  }

  tmpmask = (GPIO_Remap & DBGAFR_POSITION_MASK) >> 0x10;
  tmp = GPIO_Remap & LSB_MASK;

  if ((GPIO_Remap & (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK)) == (DBGAFR_LOCATION_MASK | DBGAFR_NUMBITS_MASK))
  {
    tmpreg &= DBGAFR_SWJCFG_MASK;
    AFIO->MAPR &= DBGAFR_SWJCFG_MASK;
  }
  else if ((GPIO_Remap & DBGAFR_NUMBITS_MASK) == DBGAFR_NUMBITS_MASK)
  {
    tmp1 = ((uint32_t)0x03) << tmpmask;
    tmpreg &= ~tmp1;
    tmpreg |= ~DBGAFR_SWJCFG_MASK;
  }
  else
  {
    tmpreg &= ~(tmp << ((GPIO_Remap >> 0x15)*0x10));
    tmpreg |= ~DBGAFR_SWJCFG_MASK;
  }

  if (NewState != DISABLE)
  {
    tmpreg |= (tmp << ((GPIO_Remap >> 0x15)*0x10));
  }

  if((GPIO_Remap & 0x80000000) == 0x80000000)
  {
    AFIO->MAPR2 = tmpreg;
  }
  else
  {
    AFIO->MAPR = tmpreg;
  }  
}

/**
  * @brief  Selects the GPIO pin used as EXTI Line.
  * @param  GPIO_PortSource: selects the GPIO port to be used as source for EXTI lines.
  *   This parameter can be GPIO_PortSourceGPIOx where x can be (A..G).
  * @param  GPIO_PinSource: specifies the EXTI line to be configured.
  *   This parameter can be GPIO_PinSourcex where x can be (0..15).
  * @retval None
  */
void GPIO_EXTILineConfig(uint8_t GPIO_PortSource, uint8_t GPIO_PinSource)
{
  uint32_t tmp = 0x00;
  /* Check the parameters */
  assert_param(IS_GPIO_EXTI_PORT_SOURCE(GPIO_PortSource));
  assert_param(IS_GPIO_PIN_SOURCE(GPIO_PinSource));
  
  tmp = ((uint32_t)0x0F) << (0x04 * (GPIO_PinSource & (uint8_t)0x03));
  AFIO->EXTICR[GPIO_PinSource >> 0x02] &= ~tmp;
  AFIO->EXTICR[GPIO_PinSource >> 0x02] |= (((uint32_t)GPIO_PortSource) << (0x04 * (GPIO_PinSource & (uint8_t)0x03)));
}

/**
  * @brief  Selects the Ethernet media interface.
  * @note   This function applies only to STM32 Connectivity line devices.  
  * @param  GPIO_ETH_MediaInterface: specifies the Media Interface mode.
  *   This parameter can be one of the following values:
  *     @arg GPIO_ETH_MediaInterface_MII: MII mode
  *     @arg GPIO_ETH_MediaInterface_RMII: RMII mode    
  * @retval None
  */
void GPIO_ETH_MediaInterfaceConfig(uint32_t GPIO_ETH_MediaInterface) 
{ 
  assert_param(IS_GPIO_ETH_MEDIA_INTERFACE(GPIO_ETH_MediaInterface)); 

  /* Configure MII_RMII selection bit */ 
  *(__IO uint32_t *) MAPR_MII_RMII_SEL_BB = GPIO_ETH_MediaInterface; 
}
  
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
