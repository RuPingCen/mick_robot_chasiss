 /*******************************************************************************
* ����˵����  USARTx: USART1, USART2, USART3, UART4,  UART5 or USART6. 
*    					u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�    					 
*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
*                                         1�������ڽ����ж� 
*
*    					DMA_send_data�� DMA���ݷ��͵��׵�ַ 
*             u32DMA_size : ���η������ݵĸ��� ��Ҳ��������ĳ��ȣ�
*             DMA_send_priority ��DMA���ȼ���0��1��2��3��  0�����ȼ����    
*             USART_DMAReq �� USART_DMAReq_Tx �����ͣ� ��  USART_DMAReq_Rx �����գ�
*
* ��ע��     	    " PA9 - USART1(Tx)    PA10 - USART1(Rx) "
*                   " PA2 - USART2_TX     PA3 - USART2_RX   "
*					" PB10 - USART3_TX    PB11 - USART3_RX  "
*					" PC10 - UART4_TX     PC11 - UART4_RX   "
*					" PC12 - UART5_TX     PD2  - UART5_RX   "
*                   " PC6 - USART6_TX     PC7  - USART6_RX  "
*  ����1����������ʿɴﵽ4.5Mbit/s  ����2~5���������Ϊ 2.25Mbit/s


 void My_Config_USART_Init(USART_TypeDef* USARTx,unsigned int u32_Baud , unsigned char UART_IT_EN);//���ڳ�ʼ������

 void UART_send_string(USART_TypeDef* USARTx,char *buf);//�ַ�������
 void UART_send_char(USART_TypeDef* USARTx,char buf); //USART1~UART5  �ַ����ͺ���
 void UART_send_data(USART_TypeDef* USARTx,unsigned int u32tempdat);  // 0~4294967296
 void UART_send_intdata(USART_TypeDef* USARTx,int u32tempdat); //-2147483648~2147483647
 void UART_send_floatdat(USART_TypeDef* USARTx,float floatempdat);//���������ͺ��� ��ȷ��С�������λ
 
  //����DMA�������ú���
 void USART_DMA_Config(USART_TypeDef* USARTx,unsigned char DMA_send_data[],unsigned int u32DMA_size,unsigned char DMA_send_priority);
 //����DMA������������
 void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);  
 
 
 //����DMA���չ̶����ȵ�����
 
 
 
 
   * ��ע��     ����DMAֻ��ͨ��1~4����    ����5û��DMA
 ********************************************************************************/
 
 
#include "bsp_uart.h" 



uint8_t USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�


 
 /****************************************************************
*
* ��������    My_Config_USART_Init
* �������ܣ�  ���ڳ�ʼ������
*
* ����˵����  USARTx:  USART1, USART2, USART3, UART4 or UART5.
*
*    					u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�  
*    					 
*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
*                                         1�������ڽ����ж� 
*   
* ��ע��  CRP   	" PA9 - USART1(Tx)    PA10 - USART1(Rx) "
*                   " PA2 - USART2_TX     PA3 - USART2_RX   "
*					" PB10 - USART3_TX    PB11 - USART3_RX  "
*					" PC10 - UART4_TX     PC11 - UART4_RX   "
*					" PC12 - UART5_TX     PD2  - UART5_RX   "
*  ����1����������ʿɴﵽ4.5Mbit/s  ����2~5���������Ϊ 2.25Mbit/s
****************************************************************/
void My_Config_USART_Init(USART_TypeDef* USARTx,unsigned int u32_Baud , unsigned char UART_IT_EN)
{
	if(USARTx == USART1)
	{
		USART1_Config(u32_Baud ,UART_IT_EN);
	}
	else if(USARTx == USART2)
	{
		USART2_Config(u32_Baud ,UART_IT_EN);
	}
	else if(USARTx == USART3)
	{
		USART3_Config(u32_Baud ,UART_IT_EN);
	}
	else if(USARTx == UART4)
	{
		UART4_Config(u32_Baud ,UART_IT_EN);
	}
	else if(USARTx == UART5)
	{
		UART5_Config(u32_Baud ,UART_IT_EN);
	}
	else if(USARTx == USART6)
	{
		USART6_Config(u32_Baud ,UART_IT_EN);
	}
	else  
	{
		while(1);
	}

}


/****************************************************************
  *
  * ��������   USART1_Config
	* �������ܣ� ����ģ��һ��ʼ������
  *
  * ����˵����  u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�  
  *    					 
	*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
  *                                         1�������ڽ����ж�
  * ��ע�� CRP
  *  
  ****************************************************************/
void USART1_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ��һ������ʼ��GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIO���帴�ó�ʲô�ڶ����� */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);/*  ���� PXx �� USARTx__Rx*/



	/* �ڶ��������ô��ڳ�ʼ���ṹ�� */	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* USART1 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); //
	
	
	/* �����������ô��ڵĽ����ж� */
	if(UART_IT_EN==1)
	{
		USART1_NVIC_Configuration();
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);/* ʹ�ܴ���1�����ж� */
	}
	else 
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);/*��ֹ����1�����ж� */

	USART_Cmd(USART1, ENABLE);//
}

void USART1_NVIC_Configuration(void)/// ����USART1�����ж�
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���ж���
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 //�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/****************************************************************
*
* ��������   USART2_Config
* �������ܣ� ����ģ��2��ʼ������
*
* ����˵����  u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�  
*    					 
*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
*                                         1�������ڽ����ж�
* ��ע�� CRP
*  
****************************************************************/
void USART2_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ��һ������ʼ��GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIO���帴�ó�ʲô�ڶ����� */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);/*  ���� PXx �� USARTx__Rx*/
	



	/* �ڶ��������ô��ڳ�ʼ���ṹ�� */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); //
	
	
	/* �����������ô��ڵĽ����ж� */
	if(UART_IT_EN==1)
	{
		USART2_NVIC_Configuration();
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);/* ʹ�ܴ���2�����ж� */
	}
	else 
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);/*��ֹ����2�����ж� */

	USART_Cmd(USART2, ENABLE);//
 
	 
}
void USART2_NVIC_Configuration(void)/// ����USART3�����ж�
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���ж���
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 //�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/****************************************************************
*
* ��������   USART3_Config
* �������ܣ� ����ģ��3��ʼ������
*
* ����˵����  u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�  
*    					 
*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
*                                         1�������ڽ����ж�
*
* ����˵���� PB10   USART3_TX    PB11 USART3_RX
* ��ע�� CRP
*  
****************************************************************/
void USART3_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ��һ������ʼ��GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* GPIO���帴�ó� �ڶ����� */
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);/*  ���� PXx �� USARTx__Rx*/
	 


	/* �ڶ��������ô��ڳ�ʼ���ṹ�� */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); //
	
	
	/* �����������ô��ڵĽ����ж� */
	if(UART_IT_EN==1)
	{
		USART3_NVIC_Configuration();
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);/* ʹ�ܴ���3�����ж� */
	}
	else 
		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);/*��ֹ����3�����ж� */

	USART_Cmd(USART3, ENABLE);//
 
	 
}
void USART3_NVIC_Configuration(void)/// ����USART3�����ж�
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���ж���
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 //�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/****************************************************************
*
* ��������   UART4_Config
* �������ܣ� ����ģ��4��ʼ������
*
* ����˵����  u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�  
*    					 
*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
*                                         1�������ڽ����ж�
*
* ����˵���� PC10   UART4_TX    PC11 UART4_RX 
* ��ע�� CRP
*  
****************************************************************/
 void UART4_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ��һ������ʼ��GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* GPIO���帴�ó�ʲô�ڶ����� */
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);/*  ���� PXx �� USARTx__Rx*/
	



	/* �ڶ��������ô��ڳ�ʼ���ṹ�� */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure); //
	
	
	/* �����������ô��ڵĽ����ж� */
	if(UART_IT_EN==1)
	{
		UART4_NVIC_Configuration();
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);/* ʹ�ܴ���3�����ж� */
	}
	else 
		USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);/*��ֹ����3�����ж� */

	USART_Cmd(UART4, ENABLE);//
 
	 
}
void UART4_NVIC_Configuration(void)/// ����UART4�����ж�
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���ж���
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;	 //�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/****************************************************************
*
* ��������   UART5_Config
* �������ܣ� ����ģ��5��ʼ������
*
* ����˵����  u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�  
*    					 
*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
*                                         1�������ڽ����ж�
*
* ����˵���� PC12   USART5_TX    PD2  USART5_RX
* ��ע�� CRP
*  
****************************************************************/
void UART5_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ��һ������ʼ��GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	/* GPIO���帴�ó�ʲô�ڶ����� */
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);/*  ���� PXx �� USARTx__Rx*/
	
 

	/* �ڶ��������ô��ڳ�ʼ���ṹ�� */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure); //
	
	
	/* �����������ô��ڵĽ����ж� */
	if(UART_IT_EN==1)
	{
		UART5_NVIC_Configuration();
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);/* ʹ�ܴ���3�����ж� */
	}
	else 
		USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);/*��ֹ����3�����ж� */

	USART_Cmd(UART5, ENABLE);//
 
	 
}
void UART5_NVIC_Configuration(void)/// ����UART5�����ж�
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���ж���
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;	 //�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/****************************************************************
*
* ��������   USART6_Config
* �������ܣ� ����ģ��6��ʼ������
*
* ����˵����  u32_Baud: ���ڲ����ʣ�9600 ��115200  �ȣ�  
*    					 
*             UART_IT_EN�� �����ж�ʹ��λ 0����ֹ���ڽ����ж�
*                                         1�������ڽ����ж�
*
* ����˵���� PC6   USART5_TX    PC7  USART5_RX
* ��ע�� CRP
*  
****************************************************************/
void USART6_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* ��һ������ʼ��GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* GPIO���帴�ó�ʲô�ڶ����� */
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);/* ���� PXx �� USARTx_Tx*/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);/*  ���� PXx �� USARTx__Rx*/
	



	/* �ڶ��������ô��ڳ�ʼ���ṹ�� */	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure); //
	
	
	/* �����������ô��ڵĽ����ж� */
	if(UART_IT_EN==1)
	{
		USART6_NVIC_Configuration();
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);/* ʹ�ܴ���3�����ж� */
	}
	else 
		USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);/*��ֹ����3�����ж� */

	USART_Cmd(USART6, ENABLE);//
 
	 
}
void USART6_NVIC_Configuration(void)/// ����USART6�����ж�
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���ж���
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;	 //�ж�Դ
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռʽ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//��Ӧ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/****************************************************************
  *
  * ��������   USART_DMA_Config
	* �������ܣ� ����ģ��DMA���ú���
  *
  * ����˵����  USARTx: USART1, USART2, USART3, UART4 or UART5. 
  *    					DMA_send_data�� DMA���ݷ��͵��׵�ַ 
	*             u32DMA_size : ���η������ݵĸ��� ��Ҳ��������ĳ��ȣ�
  *             DMA_send_priority ��DMA���ȼ���0��1��2��3��  0�����ȼ����    
  *              
  * ��ע�� CRP    ����DMAֻ��ͨ��1~4����    ����5û��DMA
  *  
  ****************************************************************/
 
//void USART_DMA_Send_Config(USART_TypeDef* USARTx,unsigned char DMA_send_data[],unsigned int u32DMA_size,unsigned char DMA_send_priority)
//{
//		DMA_InitTypeDef DMA_InitStructure;		//���ݷ���-------���ڴ��ȡ���ݴ��͵��ⲿ����
//		NVIC_InitTypeDef NVIC_InitStructure;
//	
//	
//		//DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;/*����DMA���͵�Ŀ�ĵ�ַ���������ݼĴ�����ַ*/	   	
//		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_send_data;/*�ģͣ����͵�Դ��ַ���ڴ��ַ(Ҫ����ı�����ָ��)*/		
//		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;/*���򣺴��ڴ浽����*/			
//		DMA_InitStructure.DMA_BufferSize = u32DMA_size;	/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
//		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;/*�����ַ����*/	 
//		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;/*�ڴ��ַ����*/	

//		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	/*�������ݵ�λ*/	
//		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		/*�ڴ����ݵ�λ 8bit*/ 
//	
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;//normal ��ָ������һ�ξͲ��ڷ���
//		//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;/*DMAģʽ������ѭ��*/
//   if(DMA_send_priority==0)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 	/*���ȼ������*/	 		
//   }
//	 else if(DMA_send_priority==1)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_High; 	/*���ȼ�����*/	 	
//   }
//	 else if(DMA_send_priority==2)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 	/*���ȼ��� ��*/	 	
//   }
//		
//	 else if(DMA_send_priority==3)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_Low; 	/*���ȼ� �� */	 	
//   }
//	 
//	 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;/*��ֹ�ڴ浽�ڴ�Ĵ���	*/ 
//	 
//	 
//	 if (USARTx == USART1)
//		{			
//			DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;/*����DMA���͵�Ŀ�ĵ�ַ���������ݼĴ�����ַ*/	
//			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);/*����DMAʱ��*/		
//		
//		 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//			NVIC_Init(&NVIC_InitStructure);
//			
//			DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
//			
//			DMA_Init(DMA1_Channel4, &DMA_InitStructure);/*����DMA1��4ͨ��*/	
//			DMA_Cmd (DMA1_Channel4,ENABLE);/*ʹ��DMA*/
//		}
//		else if (USARTx == USART2)
//		{ 
//			  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;/*����DMA���͵�Ŀ�ĵ�ַ���������ݼĴ�����ַ*/	
//			 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);/*����DMAʱ��*/		
//			 DMA_Init(DMA1_Channel7, &DMA_InitStructure);/*����DMA1��7ͨ��*/		   	   
//			 DMA_Cmd (DMA1_Channel7,ENABLE);/*ʹ��DMA*/					
//		 //DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
//		}
//		else if (USARTx == USART3)
//		{
//			 DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Base;/*����DMA���͵�Ŀ�ĵ�ַ���������ݼĴ�����ַ*/	
//			 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);/*����DMAʱ��*/		
//			 DMA_Init(DMA1_Channel2, &DMA_InitStructure);/*����DMA1��2ͨ��*/		   	   
//			 DMA_Cmd (DMA1_Channel2,ENABLE);/*ʹ��DMA*/					
//		 //DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
//		}    
//		else if (USARTx == UART4)
//		{
//			 DMA_InitStructure.DMA_PeripheralBaseAddr = USART4_DR_Base;/*����DMA���͵�Ŀ�ĵ�ַ���������ݼĴ�����ַ*/	
//			 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);/*����DMAʱ��*/		
//			 DMA_Init(DMA2_Channel5, &DMA_InitStructure);/*����DMA2��5ͨ��*/		   	   
//			 DMA_Cmd (DMA2_Channel5,ENABLE);/*ʹ��DMA*/					
//		 //DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
//		}      
//		else ;
//   
//	 
//}

 




/****************************************************************
  *
  * ��������   USART_DMA_Rec_Config
	* �������ܣ� ����ģ��DMA���չ̶������������ú���
  *
  * ����˵����  USARTx: USART1, USART2, USART3, UART4 . 
  *    					u32_Baud ������  
  *              
  * ��ע�� CRP    ����DMAֻ��ͨ��1~4����    ����5û��DMA
	* 
  *        ����1���գ�Ӧ����DMA1��ͨ��5  DMA1_Channel5_IRQn��
  ****************************************************************/
 
void USART_DMA_Rec_Config(USART_TypeDef* USARTx,unsigned int u32_Baud,unsigned char lendat)
{
//	  DMA_InitTypeDef DMA_InitStructure;
//	  NVIC_InitTypeDef NVIC_InitStructure;
//	
//	
//		USART1_Config(u32_Baud ,0);
//	
//		USART_Cmd(USART1, DISABLE);                    //ʹ�ܴ���1
//		USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);//�رմ��ڽ����ж�
//		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//ʹ��DMA����
//		USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1
//	   
//	
//	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1 ʱ��ʹ��
//	  DMA_DeInit(DMA1_Channel5);//��DMA��ͨ��5�Ĵ�������Ϊȱʡֵ  ����1��Ӧ����DMAͨ��5(����)
//	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;//DMA����UASRT1->DR����ַ
//    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART_RX_BUF;//DMA�ڴ����ַ
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//DMA����  ����->�ڴ�
//    DMA_InitStructure.DMA_BufferSize = lendat;//sizeof(USART_RX_BUF);//DMAͨ����DMA����Ĵ�С
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ������
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����1
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//���ݿ��Ϊ8λ
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//���ݿ��Ϊ8λ
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal��ֻ����һ�Σ�, DMA_Mode_Circular ����ͣ�ش��ͣ�
//    
//    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//(DMA�������ȼ�Ϊ�е�)
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
//    DMA_Init(DMA1_Channel5, &DMA_InitStructure);//��ʼ��DMA1 ͨ��5
//    DMA_Cmd(DMA1_Channel5, ENABLE);  //����DMA����
//	
//	  //DMA�����ж�����
//	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
//	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
//	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
//	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	  NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ��� 	
//		
//		DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);//ʹ��DMA��������ж�
 
}
// ����DMA�������� ����ͨģʽ��ͬ
void USART1_DMA_Rec_Config(unsigned int u32_Baud)
{

//  //GPIO�˿�����
//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ�� USART1��GPIOAʱ��
//  
//	//USART1_TX   GPIOA.9
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
//   
//  //USART1_RX	  GPIOA.10��ʼ��
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  
// 
//  //Usart1 NVIC ����
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//  
//   //USART ��ʼ������
//	USART_InitStructure.USART_BaudRate = u32_Baud;//���ڲ�����
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
//	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
// 
//  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
// // USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//�������ڽ����ж�
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//ʹ��DMA����
//  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1
 
}	



/****************************************************************
  *
  * ��������   UART_send_floatdat
	* �������ܣ� �������ݷ��ͺ��� ��ȷ��С�������λ
  *
  * ����˵����  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
	*             floatempdat�� 
  *   
  * ��ע�� CRP
  *  
  ****************************************************************/
void UART_send_floatdat(USART_TypeDef* USARTx,float floatempdat)
{ 
//      int m,n;
//      float R;
//	    char chartemp[5];
//      R=floatempdat;
//      m=R/1;
//      n=(R-m)*1000; if(n<0) n=-n;
//	
//      UART_send_intdata(USARTx,m);
//      UART_send_char (USARTx,'.');
//	    chartemp[0]=n/100+48;
//	    chartemp[1]=n/10%10+48;
//	    chartemp[2]=n%10+48;
//	    chartemp[3]=' ';
//	    chartemp[4]='\0';
//      UART_send_string(USARTx,chartemp);
//	
			int n,m;
			float dat=floatempdat;
			m=dat/1;
			UART_send_intdata(USARTx,m);
			UART_send_char(USARTx, '.');
			n=((dat-m)*1000)/1;
			if(n<0) n=-n;
			if(n<10)
			{
				UART_send_char(USARTx, '0');
				UART_send_char(USARTx, '0');
				UART_send_data(USARTx,n);
			}
			else if(n<100)
			{
				UART_send_char(USARTx, '0');
				UART_send_data(USARTx,n);
			}
			else if(n<1000)
			{
					UART_send_data(USARTx,n);
			}
 
}

  /****************************************************************
  *
  * ��������   UART_send_intdata
	* �������ܣ� �������������ݷ��ͺ��� 
  *
  * ����˵����  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
	*             u32tempdat��-2147483648~2147483647
  *   
  * ��ע��  CRP
  *  
  ****************************************************************/
void UART_send_intdata(USART_TypeDef* USARTx,int u32tempdat) //USART1~UART5   u32tempdat��-2147483648~2147483647
{
	  unsigned char temp[11],i;
		unsigned int gg;
	 if(u32tempdat < 0)
	 {
				u32tempdat=u32tempdat*(-1);
		    UART_send_char( USARTx,'-');
		 
	 }
		gg=u32tempdat;
	  temp[0]='\0';//Ϊ�˷����ַ����������������
	  i=1;		
		while((gg/10) != 0)
		{
				 temp[i]=gg%10+48;
				 gg=gg/10;
				 i++;
		}
	 temp[i]=gg%10+48; //ASCALL character code conversion  
	
	 for(;i>0;i--)
	 {
	     UART_send_char( USARTx,temp[i]);
   }
}

 
 /****************************************************************
  *
  * ��������   UART_send_data
	* �������ܣ� �޷����������ݷ��ͺ��� 
  *
  * ����˵����  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
	*             u32tempdat��0~4294967296
  *   
  * ��ע�� CRP
  *  
  ****************************************************************/
void UART_send_data(USART_TypeDef* USARTx,unsigned int u32tempdat) //USART1~UART5   u32tempdat��0~4294967296
{
	  unsigned char temp[11],i;
		unsigned int gg;
		gg=u32tempdat;
	  temp[0]='\0';//Ϊ�˷����ַ����������������
	  i=1;		
		while((gg/10) != 0)
		{
				 temp[i]=gg%10+48;
				 gg=gg/10;
				 i++;
		}
	 temp[i]=gg%10+48; //ASCALL character code conversion
	
	 for(;i>0;i--)
	 {
	     UART_send_char(USARTx,temp[i]);
   }
}

/****************************************************************
  *
  * ��������   UART_send_string
	* �������ܣ� �ַ������ͺ���
  *
  * ����˵����  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
  *   
  * ��ע��  CRP
  *  
  ****************************************************************/
void UART_send_string(USART_TypeDef* USARTx,char *buf)
{
	while(*buf!='\0')
	{
		 USART_SendData(USARTx,*buf++);
	   while (USART_GetFlagStatus(USARTx,USART_FLAG_TXE) == RESET); 
	}
}
/****************************************************************
  *
  * ��������   UART_send_buffer
	* �������ܣ� �ַ������ͺ���
  *
  * ����˵����  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
  *   
  * ��ע��  CRP
  *  
  ****************************************************************/
void UART_send_buffer(USART_TypeDef* USARTx,unsigned char *buf,unsigned int len)
{
	unsigned int j=0;
	for(j=0;j<len;j++)
	{
	 USART_SendData(USARTx,*buf++);
	 while (USART_GetFlagStatus(USARTx,USART_FLAG_TXE) == RESET); 
	}
}
/****************************************************************
  *
  * ��������   UART_send_char
	* �������ܣ� �ַ����ͺ���
  *
  * ����˵����  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
  *   
  * ��ע�� CRP
  *  
  ****************************************************************/
void UART_send_char(USART_TypeDef* USARTx,char buf)
{
	USART_SendData(USARTx,buf);
	while (USART_GetFlagStatus(USARTx,USART_FLAG_TXE) == RESET);  
}

//�ض���c�⺯��printf�����ڣ��ض�����ʹ��printf����
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ����� */
		USART_SendData(USART6, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

//�ض���c�⺯��scanf�����ڣ���д����ʹ��scanf��getchar�Ⱥ���
int fgetc(FILE *f)
{
		/* �ȴ������������� */
		while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART6);
}
