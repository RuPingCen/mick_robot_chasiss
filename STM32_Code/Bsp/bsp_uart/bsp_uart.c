 /*******************************************************************************
* 参数说明：  USARTx: USART1, USART2, USART3, UART4,  UART5 or USART6. 
*    					u32_Baud: 串口波特率（9600 、115200  等）    					 
*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
*                                         1：允许串口接收中断 
*
*    					DMA_send_data： DMA数据发送的首地址 
*             u32DMA_size : 单次发送数据的个数 （也就是数组的长度）
*             DMA_send_priority ：DMA优先级（0、1、2、3）  0的优先级最高    
*             USART_DMAReq ： USART_DMAReq_Tx （发送） 或  USART_DMAReq_Rx （接收）
*
* 备注：     	    " PA9 - USART1(Tx)    PA10 - USART1(Rx) "
*                   " PA2 - USART2_TX     PA3 - USART2_RX   "
*					" PB10 - USART3_TX    PB11 - USART3_RX  "
*					" PC10 - UART4_TX     PC11 - UART4_RX   "
*					" PC12 - UART5_TX     PD2  - UART5_RX   "
*                   " PC6 - USART6_TX     PC7  - USART6_RX  "
*  串口1的最大传输速率可达到4.5Mbit/s  串口2~5最大传送速率为 2.25Mbit/s


 void My_Config_USART_Init(USART_TypeDef* USARTx,unsigned int u32_Baud , unsigned char UART_IT_EN);//串口初始化函数

 void UART_send_string(USART_TypeDef* USARTx,char *buf);//字符串函数
 void UART_send_char(USART_TypeDef* USARTx,char buf); //USART1~UART5  字符发送函数
 void UART_send_data(USART_TypeDef* USARTx,unsigned int u32tempdat);  // 0~4294967296
 void UART_send_intdata(USART_TypeDef* USARTx,int u32tempdat); //-2147483648~2147483647
 void UART_send_floatdat(USART_TypeDef* USARTx,float floatempdat);//浮点数发送函数 精确到小数点后三位
 
  //串口DMA传输配置函数
 void USART_DMA_Config(USART_TypeDef* USARTx,unsigned char DMA_send_data[],unsigned int u32DMA_size,unsigned char DMA_send_priority);
 //串口DMA传输启动函数
 void USART_DMACmd(USART_TypeDef* USARTx, uint16_t USART_DMAReq, FunctionalState NewState);  
 
 
 //串口DMA接收固定长度的数据
 
 
 
 
   * 备注：     串口DMA只有通道1~4才有    串口5没有DMA
 ********************************************************************************/
 
 
#include "bsp_uart.h" 



uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节


 
 /****************************************************************
*
* 函数名：    My_Config_USART_Init
* 函数功能：  串口初始换函数
*
* 参数说明：  USARTx:  USART1, USART2, USART3, UART4 or UART5.
*
*    					u32_Baud: 串口波特率（9600 、115200  等）  
*    					 
*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
*                                         1：允许串口接收中断 
*   
* 备注：  CRP   	" PA9 - USART1(Tx)    PA10 - USART1(Rx) "
*                   " PA2 - USART2_TX     PA3 - USART2_RX   "
*					" PB10 - USART3_TX    PB11 - USART3_RX  "
*					" PC10 - UART4_TX     PC11 - UART4_RX   "
*					" PC12 - UART5_TX     PD2  - UART5_RX   "
*  串口1的最大传输速率可达到4.5Mbit/s  串口2~5最大传送速率为 2.25Mbit/s
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
  * 函数名：   USART1_Config
	* 函数功能： 串口模块一初始化函数
  *
  * 参数说明：  u32_Baud: 串口波特率（9600 、115200  等）  
  *    					 
	*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
  *                                         1：允许串口接收中断
  * 备注： CRP
  *  
  ****************************************************************/
void USART1_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 第一步：初始化GPIO */
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

	/* GPIO具体复用成什么第二功能 */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1);/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);/*  连接 PXx 到 USARTx__Rx*/



	/* 第二步：配置串口初始化结构体 */	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* USART1 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure); //
	
	
	/* 第三步：配置串口的接收中断 */
	if(UART_IT_EN==1)
	{
		USART1_NVIC_Configuration();
		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);/* 使能串口1接收中断 */
	}
	else 
		USART_ITConfig(USART1, USART_IT_RXNE, DISABLE);/*禁止串口1接收中断 */

	USART_Cmd(USART1, ENABLE);//
}

void USART1_NVIC_Configuration(void)/// 配置USART1接收中断
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选着中断组
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;	 //中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占式优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//相应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/****************************************************************
*
* 函数名：   USART2_Config
* 函数功能： 串口模块2初始化函数
*
* 参数说明：  u32_Baud: 串口波特率（9600 、115200  等）  
*    					 
*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
*                                         1：允许串口接收中断
* 备注： CRP
*  
****************************************************************/
void USART2_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 第一步：初始化GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* GPIO具体复用成什么第二功能 */
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2);/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2);/*  连接 PXx 到 USARTx__Rx*/
	



	/* 第二步：配置串口初始化结构体 */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure); //
	
	
	/* 第三步：配置串口的接收中断 */
	if(UART_IT_EN==1)
	{
		USART2_NVIC_Configuration();
		USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);/* 使能串口2接收中断 */
	}
	else 
		USART_ITConfig(USART2, USART_IT_RXNE, DISABLE);/*禁止串口2接收中断 */

	USART_Cmd(USART2, ENABLE);//
 
	 
}
void USART2_NVIC_Configuration(void)/// 配置USART3接收中断
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选着中断组
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;	 //中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占式优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//相应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/****************************************************************
*
* 函数名：   USART3_Config
* 函数功能： 串口模块3初始化函数
*
* 参数说明：  u32_Baud: 串口波特率（9600 、115200  等）  
*    					 
*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
*                                         1：允许串口接收中断
*
* 引脚说明： PB10   USART3_TX    PB11 USART3_RX
* 备注： CRP
*  
****************************************************************/
void USART3_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 第一步：初始化GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* GPIO具体复用成 第二功能 */
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3);/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3);/*  连接 PXx 到 USARTx__Rx*/
	 


	/* 第二步：配置串口初始化结构体 */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure); //
	
	
	/* 第三步：配置串口的接收中断 */
	if(UART_IT_EN==1)
	{
		USART3_NVIC_Configuration();
		USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);/* 使能串口3接收中断 */
	}
	else 
		USART_ITConfig(USART3, USART_IT_RXNE, DISABLE);/*禁止串口3接收中断 */

	USART_Cmd(USART3, ENABLE);//
 
	 
}
void USART3_NVIC_Configuration(void)/// 配置USART3接收中断
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选着中断组
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;	 //中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占式优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//相应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/****************************************************************
*
* 函数名：   UART4_Config
* 函数功能： 串口模块4初始化函数
*
* 参数说明：  u32_Baud: 串口波特率（9600 、115200  等）  
*    					 
*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
*                                         1：允许串口接收中断
*
* 引脚说明： PC10   UART4_TX    PC11 UART4_RX 
* 备注： CRP
*  
****************************************************************/
 void UART4_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 第一步：初始化GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* GPIO具体复用成什么第二功能 */
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);/*  连接 PXx 到 USARTx__Rx*/
	



	/* 第二步：配置串口初始化结构体 */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART4, &USART_InitStructure); //
	
	
	/* 第三步：配置串口的接收中断 */
	if(UART_IT_EN==1)
	{
		UART4_NVIC_Configuration();
		USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);/* 使能串口3接收中断 */
	}
	else 
		USART_ITConfig(UART4, USART_IT_RXNE, DISABLE);/*禁止串口3接收中断 */

	USART_Cmd(UART4, ENABLE);//
 
	 
}
void UART4_NVIC_Configuration(void)/// 配置UART4接收中断
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选着中断组
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;	 //中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占式优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//相应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}
/****************************************************************
*
* 函数名：   UART5_Config
* 函数功能： 串口模块5初始化函数
*
* 参数说明：  u32_Baud: 串口波特率（9600 、115200  等）  
*    					 
*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
*                                         1：允许串口接收中断
*
* 引脚说明： PC12   USART5_TX    PD2  USART5_RX
* 备注： CRP
*  
****************************************************************/
void UART5_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 第一步：初始化GPIO */
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
	/* GPIO具体复用成什么第二功能 */
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5);/*  连接 PXx 到 USARTx__Rx*/
	
 

	/* 第二步：配置串口初始化结构体 */	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5, &USART_InitStructure); //
	
	
	/* 第三步：配置串口的接收中断 */
	if(UART_IT_EN==1)
	{
		UART5_NVIC_Configuration();
		USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);/* 使能串口3接收中断 */
	}
	else 
		USART_ITConfig(UART5, USART_IT_RXNE, DISABLE);/*禁止串口3接收中断 */

	USART_Cmd(UART5, ENABLE);//
 
	 
}
void UART5_NVIC_Configuration(void)/// 配置UART5接收中断
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选着中断组
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;	 //中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占式优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//相应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/****************************************************************
*
* 函数名：   USART6_Config
* 函数功能： 串口模块6初始化函数
*
* 参数说明：  u32_Baud: 串口波特率（9600 、115200  等）  
*    					 
*             UART_IT_EN： 串口中断使能位 0：禁止串口接收中断
*                                         1：允许串口接收中断
*
* 引脚说明： PC6   USART5_TX    PC7  USART5_RX
* 备注： CRP
*  
****************************************************************/
void USART6_Config(unsigned int u32_Baud , unsigned char UART_IT_EN)
{
		 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* 第一步：初始化GPIO */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);

	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* GPIO具体复用成什么第二功能 */
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6);/* 连接 PXx 到 USARTx_Tx*/
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6);/*  连接 PXx 到 USARTx__Rx*/
	



	/* 第二步：配置串口初始化结构体 */	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);
	/* USART2 mode config 115200*/
	USART_InitStructure.USART_BaudRate =u32_Baud ;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART6, &USART_InitStructure); //
	
	
	/* 第三步：配置串口的接收中断 */
	if(UART_IT_EN==1)
	{
		USART6_NVIC_Configuration();
		USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);/* 使能串口3接收中断 */
	}
	else 
		USART_ITConfig(USART6, USART_IT_RXNE, DISABLE);/*禁止串口3接收中断 */

	USART_Cmd(USART6, ENABLE);//
 
	 
}
void USART6_NVIC_Configuration(void)/// 配置USART6接收中断
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选着中断组
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;	 //中断源
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占式优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;//相应优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
}

/****************************************************************
  *
  * 函数名：   USART_DMA_Config
	* 函数功能： 串口模块DMA配置函数
  *
  * 参数说明：  USARTx: USART1, USART2, USART3, UART4 or UART5. 
  *    					DMA_send_data： DMA数据发送的首地址 
	*             u32DMA_size : 单次发送数据的个数 （也就是数组的长度）
  *             DMA_send_priority ：DMA优先级（0、1、2、3）  0的优先级最高    
  *              
  * 备注： CRP    串口DMA只有通道1~4才有    串口5没有DMA
  *  
  ****************************************************************/
 
//void USART_DMA_Send_Config(USART_TypeDef* USARTx,unsigned char DMA_send_data[],unsigned int u32DMA_size,unsigned char DMA_send_priority)
//{
//		DMA_InitTypeDef DMA_InitStructure;		//数据方向-------从内存读取数据传送到外部串口
//		NVIC_InitTypeDef NVIC_InitStructure;
//	
//	
//		//DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;/*设置DMA传送的目的地址：串口数据寄存器地址*/	   	
//		DMA_InitStructure.DMA_MemoryBaseAddr = (u32)DMA_send_data;/*ＤＭＡ传送的源地址：内存地址(要传输的变量的指针)*/		
//		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;/*方向：从内存到外设*/			
//		DMA_InitStructure.DMA_BufferSize = u32DMA_size;	/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
//		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;/*外设地址不增*/	 
//		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;/*内存地址自增*/	

//		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	/*外设数据单位*/	
//		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;		/*内存数据单位 8bit*/ 
//	
//		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal ;//normal 是指发送完一次就不在发送
//		//DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;/*DMA模式：不断循环*/
//   if(DMA_send_priority==0)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh; 	/*优先级：最高*/	 		
//   }
//	 else if(DMA_send_priority==1)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_High; 	/*优先级：高*/	 	
//   }
//	 else if(DMA_send_priority==2)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_Medium; 	/*优先级： 中*/	 	
//   }
//		
//	 else if(DMA_send_priority==3)
//	 {
//      DMA_InitStructure.DMA_Priority = DMA_Priority_Low; 	/*优先级 低 */	 	
//   }
//	 
//	 DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;/*禁止内存到内存的传输	*/ 
//	 
//	 
//	 if (USARTx == USART1)
//		{			
//			DMA_InitStructure.DMA_PeripheralBaseAddr = USART1_DR_Base;/*设置DMA传送的目的地址：串口数据寄存器地址*/	
//			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);/*开启DMA时钟*/		
//		
//		 	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel4_IRQn;
//			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//			NVIC_Init(&NVIC_InitStructure);
//			
//			DMA_ITConfig(DMA1_Channel4,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
//			
//			DMA_Init(DMA1_Channel4, &DMA_InitStructure);/*配置DMA1的4通道*/	
//			DMA_Cmd (DMA1_Channel4,ENABLE);/*使能DMA*/
//		}
//		else if (USARTx == USART2)
//		{ 
//			  DMA_InitStructure.DMA_PeripheralBaseAddr = USART2_DR_Base;/*设置DMA传送的目的地址：串口数据寄存器地址*/	
//			 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);/*开启DMA时钟*/		
//			 DMA_Init(DMA1_Channel7, &DMA_InitStructure);/*配置DMA1的7通道*/		   	   
//			 DMA_Cmd (DMA1_Channel7,ENABLE);/*使能DMA*/					
//		 //DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
//		}
//		else if (USARTx == USART3)
//		{
//			 DMA_InitStructure.DMA_PeripheralBaseAddr = USART3_DR_Base;/*设置DMA传送的目的地址：串口数据寄存器地址*/	
//			 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);/*开启DMA时钟*/		
//			 DMA_Init(DMA1_Channel2, &DMA_InitStructure);/*配置DMA1的2通道*/		   	   
//			 DMA_Cmd (DMA1_Channel2,ENABLE);/*使能DMA*/					
//		 //DMA_ITConfig(DMA1_Channel3,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
//		}    
//		else if (USARTx == UART4)
//		{
//			 DMA_InitStructure.DMA_PeripheralBaseAddr = USART4_DR_Base;/*设置DMA传送的目的地址：串口数据寄存器地址*/	
//			 RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);/*开启DMA时钟*/		
//			 DMA_Init(DMA2_Channel5, &DMA_InitStructure);/*配置DMA2的5通道*/		   	   
//			 DMA_Cmd (DMA2_Channel5,ENABLE);/*使能DMA*/					
//		 //DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
//		}      
//		else ;
//   
//	 
//}

 




/****************************************************************
  *
  * 函数名：   USART_DMA_Rec_Config
	* 函数功能： 串口模块DMA接收固定长度数据配置函数
  *
  * 参数说明：  USARTx: USART1, USART2, USART3, UART4 . 
  *    					u32_Baud 波特率  
  *              
  * 备注： CRP    串口DMA只有通道1~4才有    串口5没有DMA
	* 
  *        串口1接收，应该用DMA1的通道5  DMA1_Channel5_IRQn。
  ****************************************************************/
 
void USART_DMA_Rec_Config(USART_TypeDef* USARTx,unsigned int u32_Baud,unsigned char lendat)
{
//	  DMA_InitTypeDef DMA_InitStructure;
//	  NVIC_InitTypeDef NVIC_InitStructure;
//	
//	
//		USART1_Config(u32_Baud ,0);
//	
//		USART_Cmd(USART1, DISABLE);                    //使能串口1
//		USART_ITConfig(USART1, USART_IT_IDLE, DISABLE);//关闭串口接受中断
//		USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能DMA接收
//		USART_Cmd(USART1, ENABLE);                    //使能串口1
//	   
//	
//	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE); //DMA1 时钟使能
//	  DMA_DeInit(DMA1_Channel5);//将DMA的通道5寄存器重设为缺省值  串口1对应的是DMA通道5(接收)
//	  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;//DMA外设UASRT1->DR基地址
//    DMA_InitStructure.DMA_MemoryBaseAddr = (u32)USART_RX_BUF;//DMA内存基地址
//    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//DMA方向  外设->内存
//    DMA_InitStructure.DMA_BufferSize = lendat;//sizeof(USART_RX_BUF);//DMA通道的DMA缓存的大小
//    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址不增加
//    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址自增1
//    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//数据宽度为8位
//    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;//数据宽度为8位
//    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//DMA_Mode_Normal（只传送一次）, DMA_Mode_Circular （不停地传送）
//    
//    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;//(DMA传送优先级为中等)
//    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
//    DMA_Init(DMA1_Channel5, &DMA_InitStructure);//初始化DMA1 通道5
//    DMA_Cmd(DMA1_Channel5, ENABLE);  //启动DMA传输
//	
//	  //DMA发送中断配置
//	  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
//	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
//	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	  NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器 	
//		
//		DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);//使能DMA传输完成中断
 
}
// 配置DMA串口引脚 与普通模式相同
void USART1_DMA_Rec_Config(unsigned int u32_Baud)
{

//  //GPIO端口设置
//  GPIO_InitTypeDef GPIO_InitStructure;
//	USART_InitTypeDef USART_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	 
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能 USART1，GPIOA时钟
//  
//	//USART1_TX   GPIOA.9
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
//   
//  //USART1_RX	  GPIOA.10初始化
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
//  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  
// 
//  //Usart1 NVIC 配置
//  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//  
//   //USART 初始化设置
//	USART_InitStructure.USART_BaudRate = u32_Baud;//串口波特率
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
//	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
// 
//  USART_Init(USART1, &USART_InitStructure); //初始化串口1
// // USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启串口接受中断
//	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);//使能DMA接收
//  USART_Cmd(USART1, ENABLE);                    //使能串口1
 
}	



/****************************************************************
  *
  * 函数名：   UART_send_floatdat
	* 函数功能： 浮点数据发送函数 精确到小数点后三位
  *
  * 参数说明：  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
	*             floatempdat： 
  *   
  * 备注： CRP
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
  * 函数名：   UART_send_intdata
	* 函数功能： 带符号整形数据发送函数 
  *
  * 参数说明：  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
	*             u32tempdat：-2147483648~2147483647
  *   
  * 备注：  CRP
  *  
  ****************************************************************/
void UART_send_intdata(USART_TypeDef* USARTx,int u32tempdat) //USART1~UART5   u32tempdat：-2147483648~2147483647
{
	  unsigned char temp[11],i;
		unsigned int gg;
	 if(u32tempdat < 0)
	 {
				u32tempdat=u32tempdat*(-1);
		    UART_send_char( USARTx,'-');
		 
	 }
		gg=u32tempdat;
	  temp[0]='\0';//为了符合字符串输出函数结束符
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
  * 函数名：   UART_send_data
	* 函数功能： 无符号整形数据发送函数 
  *
  * 参数说明：  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
	*             u32tempdat：0~4294967296
  *   
  * 备注： CRP
  *  
  ****************************************************************/
void UART_send_data(USART_TypeDef* USARTx,unsigned int u32tempdat) //USART1~UART5   u32tempdat：0~4294967296
{
	  unsigned char temp[11],i;
		unsigned int gg;
		gg=u32tempdat;
	  temp[0]='\0';//为了符合字符串输出函数结束符
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
  * 函数名：   UART_send_string
	* 函数功能： 字符串发送函数
  *
  * 参数说明：  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
  *   
  * 备注：  CRP
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
  * 函数名：   UART_send_buffer
	* 函数功能： 字符串发送函数
  *
  * 参数说明：  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
  *   
  * 备注：  CRP
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
  * 函数名：   UART_send_char
	* 函数功能： 字符发送函数
  *
  * 参数说明：  USARTx:  
  *    					USART1, USART2, USART3, UART4 or UART5.
  *   
  * 备注： CRP
  *  
  ****************************************************************/
void UART_send_char(USART_TypeDef* USARTx,char buf)
{
	USART_SendData(USARTx,buf);
	while (USART_GetFlagStatus(USARTx,USART_FLAG_TXE) == RESET);  
}

//重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
		/* 发送一个字节数据到串口 */
		USART_SendData(USART6, (uint8_t) ch);
		
		/* 等待发送完毕 */
		while (USART_GetFlagStatus(USART6, USART_FLAG_TXE) == RESET);		
	
		return (ch);
}

//重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
		while (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(USART6);
}
