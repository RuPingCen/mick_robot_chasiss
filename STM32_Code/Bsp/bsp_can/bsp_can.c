/*********************************************************************************
修改于野火STM32教程的历程
CRP
2023-08-29

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
#include "stm32f4xx.h"
#include "bsp_can.h" 
#include "stm32f4xx_rcc.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"

uint8_t data_num = 8;
uint8_t data_can[8];
extern __IO uint32_t flag;	


//CanRxMsg RxMessage;				 //接收缓冲区

 

/*
 *  CAN 差分信号的表示
 *  1:隐性电平   H2.5v - L2.5v = 0v
 *  0:显性电平   H3.5v - L1.5v = 2v
 */

/*
 * 函数名：CAN_GPIO_Config
 * 描述  ：CAN的GPIO 配置,PB8上拉输入，PB9推挽输出
 * 输入  ：CAN1或CAN2
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_GPIO_Config(CAN_TypeDef* CANx)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	

    if(CANx == CAN1)
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);//使能CAN1时钟
        /* 使能GPIO时钟 */
        RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK|CAN1_RX_GPIO_CLK, ENABLE);
        
        /* 端口复用 */
        GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
        GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT);
        
        /* 设置CAN1 TX引脚 */
        GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);

        /* 设置CAN1 RX引脚 */
        GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
    }
    else
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);//使能CAN2时钟		
        /* 使能GPIO时钟 */
        RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK|CAN2_RX_GPIO_CLK, ENABLE);
        
        /* 端口复用 */
        GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
        GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT);
        
        /* 设置CAN2 TX引脚 */
        GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);

        /* 设置CAN2 RX引脚 */
        GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);
    }
}

/*
 * 函数名：CAN_NVIC_Config
 * 描述  ：CAN的NVIC 配置,第1优先级组，0，0优先级
 * 输入  ：CAN1或CAN2
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_NVIC_Config(CAN_TypeDef* CANx)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/*中断设置*/
    if(CANx == CAN1) 
	{
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		        //抢占优先级0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			            //子优先级为0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	else 
	{
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		        //抢占优先级0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			            //子优先级为1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}	           
    
}

/*
 * 函数名：CAN_Mode_Config
 * 描述  ：CAN的模式 配置
 * 输入  ：CAN1或CAN2
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Mode_Config(CAN_TypeDef* CANx)
{
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CAN通信参数设置**********************************/
	/* Enable CAN clock */
    if(CANx == CAN1)
    {
        RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);
    }
	/*CAN寄存器初始化*/
	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN单元初始化*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  使用自动唤醒模式
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
	 
	/* ss=1 bs1=4 bs2=2 位时间宽度为(1+4+2) 波特率即为时钟周期tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;		   //BTR-TS1 时间段1 占用了4个时间单元
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;		   //BTR-TS1 时间段2 占用了2个时间单元	
	
	/* CAN Baudrate = 1 MBps (1MBps已为stm32的CAN最高速率) (CAN 时钟频率为 APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =6;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 42/(1+4+2)/6=1 Mbps
	CAN_Init(CANx, &CAN_InitStructure);
}


/*
 * 函数名：CAN_Filter_Config
 * 描述  ：CAN的过滤器 配置
 * 输入  ：CAN1或CAN2
 * 输出  : 无
 * 调用  ：内部调用
 */
static void CAN_Filter_Config(CAN_TypeDef* CANx)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN筛选器初始化*/
    if(CANx == CAN1) {CAN_FilterInitStructure.CAN_FilterNumber=0;}
	else {CAN_FilterInitStructure.CAN_FilterNumber=14;}				//筛选器组14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在掩码模式
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//筛选器位宽为单个32位。
	/* 使能筛选器，按照标志的内容进行比对筛选，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

	CAN_FilterInitStructure.CAN_FilterIdHigh= ((((u32)0x580<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//要筛选的ID高位 
	CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x580<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要筛选的ID低位 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000; //筛选器高16位每位必须匹配 (1表示匹配 0表示忽略该位)
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;//筛选器低16位每位必须匹配
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;	 //筛选器被关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能筛选器
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CAN通信中断使能*/
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}

/*0x580
 * 函数名：CAN_Config
 * 描述  ：完整配置CAN的功能
 * 输入  ：CAN1或CAN2
 * 输出  : 无
 * 调用  ：外部调用
 */
void CAN_Config(CAN_TypeDef* CANx)
{
    CAN_GPIO_Config(CANx);          //GPIO设置
    CAN_Mode_Config(CANx);          //CAN口设置
    CAN_Filter_Config(CANx);        //CAN过滤器设置
    CAN_NVIC_Config(CANx);          //CAN中断设置
	 
}

 
 
/**************************END OF FILE************************************/

