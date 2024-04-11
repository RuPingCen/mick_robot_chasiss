 /*************************************************************************
*  ����˵��: ���ٷ����޸��Ժ� ������Լ��Ŀ�
*  ����˵��:GPIOx: where x can be (A..G) to select the GPIO peripheral.

			GPIO_Pin_x:  where x can be (0..15)
			
			GPIO_Mod:    GPIO_Mode_IN 			   ģ������   
						 GPIO_Mode_OUT         ���
						 GPIO_Mode_AF    	  
						 GPIO_Mode_AN   		 

			 GPIO_Speed:		
							GPIO_Speed_2MHz  
							GPIO_Speed_25MHz
							GPIO_Speed_50MHz	
							GPIO_Speed_100MHz
								 
			 Trigger_mod:   0  EXTI_Trigger_Rising; //�������ж� 
							1  EXTI_Trigger_Falling; //�½����ж� 
							2  EXTI_Trigger_Rising_Falling; //���������½����ж�	

* ��ʹ�ú���:    

		void My_GPIO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mod);//�Զ����ʼ������
		
		uint8_t GPIO_ReadInputDataBit(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);//��ȡĳһ�����ŵ�ƽ
		uint16_t GPIO_ReadInputData(GPIO_TypeDef* GPIOx);//��ȡĳһ���˿ڵ�����
		
		void GPIO_SetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);//��λһ������  ʹ���������1��
		void GPIO_ResetBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin_x);//��λһ������  ʹ���������0��
		void GPIO_ToggleBits(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x);//���ŵ�ƽ��ת
		void GPIO_Flip_level(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x);
		
		void My_GPIO_Exit_Init(GPIO_TypeDef* GPIOx,uint16_t pin_x , uint8_t Trigger_mod);//GPIO�ⲿ�жϳ�ʼ������
		
		
*  ���� :�CCRP
*  �޸�ʱ��: 2020-2-3    
*  ��    ע: ��
*************************************************************************/

#include "bsp_gpio.h"
 
void My_GPIO_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x,GPIOMode_TypeDef GPIO_Mod)
{	
	GPIO_InitTypeDef GPIO_InitStructure;/*����һ��GPIO_InitTypeDef���͵Ľṹ��*/
	uint32_t RCC_temp;/*����һ��uint32_t���ͱ���*/

	if(GPIOx==GPIOA) RCC_temp=RCC_AHB1Periph_GPIOA;//�����˿�ʱ��
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

	RCC_AHB1PeriphClockCmd(RCC_temp, ENABLE);/*����GPIOX������ʱ��*/ 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;	/*ѡ��Ҫ���Ƶ�GPIOX����*/	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mod; 	/*��������ģʽ*/
  
	if(GPIO_Mod==GPIO_Mode_OUT)
	{
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;   /* Ĭ�ϲ����������*/
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     /* Ĭ������*/
		GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed; 		/*���ģʽ����Ҫ������������  25MHz*/ 
	}	
	else if(GPIO_Mod==GPIO_Mode_IN)
	{
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;     /* Ĭ������*/
		GPIO_InitStructure.GPIO_Speed = GPIO_Low_Speed; 		/*���ģʽ����Ҫ������������  25MHz*/ 
	}		
	 
	GPIO_Init(GPIOx, &GPIO_InitStructure);			/*���ÿ⺯������ʼ��GPIO*/

	if(GPIO_Mod==GPIO_Mode_OUT)
		GPIO_SetBits(GPIOx, GPIO_Pin_x);	/*���Ϊ���ģʽ���ʼ������ߵ�ƽ	*/	 
}

void GPIO_Flip_level(GPIO_TypeDef* GPIOx,uint16_t pin_x)
{
			GPIOx->ODR ^=pin_x;//���ŵ�ƽ��ת
}
/********************************************************************
  * @brief  Reads the specified input port pin.
  * @param  GPIOx: where x can be (A..G) to select the GPIO peripheral.
  * @param  GPIO_Pin_0:  where x can be (0..15)
  * @param  Trigger_mod:  0  EXTI_Trigger_Rising; //�������ж� 
													1  EXTI_Trigger_Falling; //�½����ж� 
												  2  EXTI_Trigger_Rising_Falling; //���������½����ж�	
   EXTI0-EXTI15����GPIO�ⲿ�����ж�
   EXTI15-EXTI20 ���������ⲿ�����ж�
***********************************************************************/
 
void My_GPIO_Exit_Init(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin_x , uint8_t Trigger_mod)
{
	GPIO_InitTypeDef GPIO_InitStructure; 
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	uint32_t RCC_temp;/*����һ��uint32_t���ͱ���*/
	uint8_t my_GPIO_PortSource;
	uint8_t my_GPIO_PinSource;


	if(GPIOx==GPIOA)      { RCC_temp=RCC_AHB1Periph_GPIOA; my_GPIO_PortSource = EXTI_PortSourceGPIOA;}//�����˿�ʱ��
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

	RCC_AHB1PeriphClockCmd(RCC_temp, ENABLE);/*����GPIOX������ʱ��*/ 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);


	 if(GPIO_Pin_x==GPIO_Pin_0)      { NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;/* �����ж�Դ */my_GPIO_PinSource=EXTI_PinSource0;EXTI_InitStructure.EXTI_Line = EXTI_Line0;}//�����ж���
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
	 NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//������ռ���ȼ�
	 NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//������Ӧ���ȼ�
	 /*************************************************************
		 �����ж��IO�ж�ʱ �������޸�������ж����ȼ�����Ӧ���ȼ� 
		 ��STM32�е��ж����ȼ�����Ӧ���ȼ���ͬʱ�ᰴ���ڲ��ж������������Ӧ 
	 ************************************************************/
	 NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	 NVIC_Init(&NVIC_InitStructure);


	 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_x;  /* EXTI line gpio config*/	      
	 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;	 
	 GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;// ����������
	 GPIO_Init(GPIOx, &GPIO_InitStructure);
	 
		 
	 SYSCFG_EXTILineConfig(my_GPIO_PortSource, my_GPIO_PinSource); 	/* EXTI line mode config */	

	 if(Trigger_mod==0)EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�������ж� 
	 else if(Trigger_mod==1)EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж� 
	 else if(Trigger_mod==2)EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling; //���������½����ж�
	 else ;
	 EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	//EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	//EXTI_InitStructure.EXTI_Trigger = Trigger_mod; //�½����ж�EXTI_Trigger_Falling
	 EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	 EXTI_Init(&EXTI_InitStructure); 	
}
