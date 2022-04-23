/*********************************************************************************
修改于野火STM32教程的历程
CRP
2019-09-14

使用方法：
	 CAN_Config();   //配置CAN模块 
	 CAN_SetMsg();// 设置要通过CAN发送的信息
	 CAN_Transmit(CAN1, &TxMessage); // 发送消息 “ABCD”
	 
中断服务函数：
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //从邮箱中读出报文
  //  比较ID和数据是否为0x1314及DCBA 
  if((RxMessage.ExtId==0x1314) && (RxMessage.IDE==CAN_ID_EXT)
     && (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDCBA))
  {
    flag = 0; 					       //接收成功  
  }
  else
  {
    flag = 0xff; 					   //接收失败
  }
}

**********************************************************************************/
#include "stm32f10x.h"
#include "bsp_can.h" 


//CanRxMsg RxMessage;				 //接收缓冲区

 

/*
 *  CAN 差分信号的表示
 *  1:隐性电平   H2.5v - L2.5v = 0v
 *  0:显性电平   H3.5v - L1.5v = 2v
 */

/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置,PB8上拉输入，PB9推挽输出
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	
  	/*外设时钟设置*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

//  	/*IO设置*/
//	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
//	/* Configure CAN pin: RX */									               // PB8
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // 上拉输入
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//    /* Configure CAN pin: TX */									               // PB9
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//使能PORTA时钟	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//上拉输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//初始化IO
	
}

/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*中断设置*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //子优先级为0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	 	/************************CAN通信参数设置**********************************/
	/*CAN寄存器初始化*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			//非时间触发通信模式  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//软件自动离线管理	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;			//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW 重新同步跳跃宽度 1个时间单元
	CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;		   //BTR-TS1 时间段1 占用了3个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;		   //BTR-TS1 时间段2 占用了5个时间单元
	CAN_InitStructure.CAN_Prescaler =4;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+5+3)/4=1Mbps
	
	 //   CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
   // CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;		   //BTR-TS1 时间段1 占用了6个时间单元
   // CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 时间段2 占用了3个时间单元
   // CAN_InitStructure.CAN_Prescaler =4;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+6+3)/4=0.9Mbps
		
	CAN_Init(CAN1, &CAN_InitStructure);
}

/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN过滤器初始化*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//过滤器组0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在标识符屏蔽位模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//过滤器位宽为单个32位。
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	//CAN_FilterInitStructure.CAN_FilterIdHigh= (((u32)0x1314<<3)&0xFFFF0000)>>16;				//要过滤的ID高位 
	//CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要过滤的ID低位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//过滤器高16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//过滤器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//过滤器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}


/*
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_Mode_Config();
  CAN_Filter_Config();   
	CAN_NVIC_Config();
}


/*
 * 函数名：CAN_SetMsg
 * 描述  ：CAN通信报文内容设置
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	 
void CAN_SetMsg(void)
{	  
	CanTxMsg TxMessage;			   //发送缓冲区
  //TxMessage.StdId=0x00;						 
  TxMessage.ExtId=0x1314;					 //使用的扩展ID
  TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
  TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
  TxMessage.DLC=2;							 //数据长度为2字节
  TxMessage.Data[0]=0xAB;
  TxMessage.Data[1]=0xCD;
	
	//CAN_Transmit(CAN1, &TxMessage); // 发送消息 “ABCD”
}




/**************************END OF FILE************************************/

