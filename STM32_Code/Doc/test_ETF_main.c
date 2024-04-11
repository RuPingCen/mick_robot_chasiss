#include "stm32f4xx.h"
#include "bsp_systick.h" 
#include "bsp_gpio.h"
#include "bsp_uart.h" 
#include "bsp_timer.h"
#include "bsp_usart_dma.h" 
#include "bsp_can.h" 
#include "key.h" 
#include "led.h" 

#include "lwip/tcp.h"
#include "netconf.h"
#include "tcp_echoclient.h"
#include "stm32f4x7_phy.h"

uint8_t data_buff[1024];
unsigned char test_DMA[50] ={"this is a test example"};

__IO uint32_t flag_can = 0;		 //用于标志是否接收到数据，在中断函数中赋值
CanTxMsg TxMessage;			     //发送缓冲区
CanRxMsg RxMessage;				 //接收缓冲区
extern uint8_t data_can[8];

// 以太网测试变量
extern __IO uint8_t EthLinkStatus;
__IO uint32_t LocalTime = 0;
static void TIM3_Config(uint16_t period,uint16_t prescaler);
void Main_Delay(unsigned int delayvalue);
int main(void)
{
	/* 在这里添加你自己的程序 */
//	int i=0;
	uint8_t flag=0;
	SysTick_Init();
	
	// LED灯
	 My_GPIO_Init( GPIOF, GPIO_Pin_6, GPIO_Mode_OUT);
	 My_GPIO_Init( GPIOF, GPIO_Pin_7, GPIO_Mode_OUT);
	 My_GPIO_Init( GPIOF, GPIO_Pin_8, GPIO_Mode_OUT);
	

	// 按键输入
	//	My_GPIO_Init( GPIOA, GPIO_Pin_0, GPIO_Mode_IN); //输入模式
	//	My_GPIO_Init( GPIOC, GPIO_Pin_13, GPIO_Mode_IN);
	
	//	My_GPIO_Exit_Init(GPIOA,GPIO_Pin_0 ,0); //外部中断触发    上升沿 
	//	My_GPIO_Exit_Init(GPIOC,GPIO_Pin_13 ,0); //外部中断触发   上升沿 
	
	
	//串口测试
	My_Config_USART_Init(USART1,115200,0);
	 //USART3_Config(115200 ,1);
//	UART_send_string(USART1,"USART1 ......");
//	printf("野火STM32F407串口实验\n");
	 UART_send_string(USART1,"UART1 Test......\n");
	
 
 
	//---------------------DMA测试---------------------
//	My_USART_DMA_config(USART1);
//	for(i=0;i<1024;i++)
//	{
//		data_buff[i]	 = 'A';
//	}
//	USART_DMA_Tx_Data(USART1,data_buff, 1024);
//	
//	// DMA接收
//	USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);

	
	//---------------------CAN测试---------------------
//	CAN_Config(CAN1);
//	 for(i=0; i<8; i++)
//    {
//        data_can[i] = i;
//    }
// 
	
	// 定时器初始化测试
//	Timer_2to7_Init(TIM6,50*1000);
//	Timer_start(TIM6);
	
	
	
	TIM3_Config(999,899);//10ms定时器
	printf("以太网通信例程\n");
	
	/* Configure ethernet (GPIOs, clocks, MAC, DMA) */
  ETH_BSP_Config();	
  printf("PHY初始化结束\n");
	
  /* Initilaize the LwIP stack */
  LwIP_Init();	
  
  printf("    KEY1: 启动TCP连接\n");
  printf("    KEY2: 断开TCP连接\n");
  
  /* IP地址和端口可在netconf.h文件修改，或者使用DHCP服务自动获取IP
	(需要路由器支持)*/
  printf("本地IP和端口: %d.%d.%d.%d\n",IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  printf("远端IP和端口: %d.%d.%d.%d:%d\n",DEST_IP_ADDR0, DEST_IP_ADDR1,DEST_IP_ADDR2, DEST_IP_ADDR3,DEST_PORT);
  
	while(1)
	{
//	    /*按一次按键 CAN 发送一次数据*/
//		if(	Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
//		{
//			LED_BLUE;
//			CAN_SetMsg(&TxMessage,data_can,8,0x1315,1,0);
//            printf("set complete\r\n");
//			
//			CAN_Transmit(CAN1, &TxMessage);
//            printf("Transmit complete\r\n");
//			
//			Main_Delay(90);//等待发送完毕，可使用CAN_TransmitStatus查看状态
//	 
//			printf("\r\n ExtId: 0x%x \r\n",TxMessage.ExtId);
//			//CAN_DEBUG_ARRAY(TxMessage.Data,8); 
//		}
//		if(flag_can==1)
//		{		
//			LED_GREEN;
//            //CAN_Receive(CAN1,CAN_FIFO0,&RxMessage);
//			printf("\r\nCAN recived data: \r\n");	
//			printf("\r\n ExtId: 0x%x \r\n",RxMessage.ExtId);
//			printf("\r\n FMI: 0x%x \r\n",RxMessage.FMI);
//		 
//			flag_can=0;
//		}

		if((Key_Scan(KEY1_GPIO_PORT,KEY1_PIN)==KEY_ON) && (flag==0))
		{
			LED2_ON;
			if (EthLinkStatus == 0)
			{
				printf("connect to tcp server\n");
				/*connect to tcp server */ 
				tcp_echoclient_connect();
				flag=1;
			}
		}
		if((Key_Scan(KEY2_GPIO_PORT,KEY2_PIN)==KEY_ON) && flag)
		{
			LED2_OFF;
			tcp_echoclient_disconnect();
			flag=0;
		}
		/* check if any packet received */
		if (ETH_CheckFrameReceived())
		{ 
			/* process received ethernet packet */
			LwIP_Pkt_Handle();
		}
		/* handle periodic timers for LwIP */
		LwIP_Periodic_Handle(LocalTime);
	};
}

 //延时函数 6.3ms
void Main_Delay(unsigned int delayvalue)
{
	unsigned int i;
	while(delayvalue-->0)
	{	
		i=5000;
		while(i-->0);
	}
}
/**
  * @brief  通用定时器3中断初始化
  * @param  period : 自动重装值。
  * @param  prescaler : 时钟预分频数
  * @retval 无
  * @note   定时器溢出时间计算方法:Tout=((period+1)*(prescaler+1))/Ft us.
  *          Ft=定时器工作频率,为SystemCoreClock/2=90,单位:Mhz
  */
static void TIM3_Config(uint16_t period,uint16_t prescaler)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
	TIM_TimeBaseInitStructure.TIM_Prescaler=prescaler;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_Period=period;   //自动重装载值
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/**
  * @brief  定时器3中断服务函数
  * @param  无
  * @retval 无
  */
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		LocalTime+=10;//10ms增量
	}
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}
