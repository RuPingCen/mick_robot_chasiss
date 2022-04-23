/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "stm32f10x_gpio.h"
#include <stdio.h>

#include "IMU.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "DBUS.h"
#include "DJI_Motor.h"
#include "Mick_IO.h"

extern volatile uint32_t TimingDelay;
extern volatile uint32_t Timer2_Counter1;
extern volatile uint32_t Timer2_Counter2;
extern volatile uint32_t Timer2_Counter3;
extern volatile uint32_t Timer2_Counter4;
extern volatile uint32_t Timer2_Counter5;

extern volatile uint8_t UART1_DMA_Flag; //串口1中断标志位
extern volatile uint8_t UART2_Flag; //串口2中断标志位
extern volatile uint8_t CAN1_Flag;  //CAN总线接收中断标志

extern uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节

extern rc_info_t rc;   
extern command_t recived_cmd; //底盘接收上位机命令结构体

extern volatile uint8_t UART2_ReBuff[100];  
extern volatile uint16_t UART2_ReCont;  
extern volatile unsigned char UART2_Reflag; 
extern volatile uint8_t TIM3_Flag;

uint16_t ReCont_2=0;  
unsigned char Reflag_2=0;  
unsigned char U2dat_value=0;
unsigned char U2dat_value_last=0;
unsigned char U1dat_value=0;

 
//extern void Matrix_Keyscan_Analy(unsigned int interupt_times);//矩阵按键读取
//extern void Keyscan_Analy(unsigned int interupt_times);//独立按键读取

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}
 
 
/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}
void TIM2_IRQHandler(void)//定时器2 中断服务函数
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
	  Timer2_Counter1++;
		Timer2_Counter2++;
		Timer2_Counter3++;	
		Timer2_Counter4++;
		Timer2_Counter5++;		
		
		if((Timer2_Counter1>100*10)) // 如果定时计数器操作1s还没有被清零，说明通讯出现了中断
		{			
			if(rc.available == 0x00)
			{
				rc.sw1 = 5; //标记接收数据不可用
				Mecanum_Wheel_Rpm_Model(0,0,0,0);
			}
			else if(recived_cmd.flag ==0x00) //上位机没有持续发送数据超过1s则触发通讯丢失
			{
				Mecanum_Wheel_Rpm_Model(0,0,0,0);
			}
			Timer2_Counter1=0;
		}
		
	  //GPIO_Flip_level(GPIOE,GPIO_Pin_1);
		DJI_Motor_Control(); 
		TIM_ClearITPendingBit(TIM2 , TIM_FLAG_Update);   		
	}
}
void TIM3_IRQHandler(void)//定时器3 中断服务函数   目前未使用
{
	if ( TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET ) 
	{	
	  IMU_Routing();
		TIM3_Flag=0x01;
		TIM_ClearITPendingBit(TIM3, TIM_FLAG_Update);   		
	}
}





void USART1_IRQHandler(void)
{
	uint8_t ch;

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{ 	
		//ch = USART1->DR;
		ch = USART_ReceiveData(USART1);	  	 
		UART_send_char(USART1,ch);
		
	} 
	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}

// USRT2 转发接收到的数据到UART3
void USART2_IRQHandler(void)
{
	uint8_t dat_value;
	U2dat_value_last = U2dat_value;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{ 	
		USART_ClearFlag(USART2, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		dat_value = USART_ReceiveData(USART2);	
		 
	}

		//数据包解析协议
	U2dat_value=dat_value;
	
	if((U2dat_value == 0XEA )&& (U2dat_value_last == 0XAE))
	{
			ReCont_2 =0;
			Reflag_2 = 0x01;
			UART2_ReBuff[ReCont_2++] = U2dat_value_last;
			UART2_ReBuff[ReCont_2++] = U2dat_value; //找到了帧头
		  UART2_Flag=0x01;
		  
	}
	else if(Reflag_2 == 0x01 && ReCont_2>=2) 
	{  
			if(ReCont_2<3)
			{
				UART2_ReBuff[ReCont_2++] = U2dat_value; //找到数据帧长度	
			}
			else if(ReCont_2 < (UART2_ReBuff[2]+2)) 
			{
				UART2_ReBuff[ReCont_2++] = U2dat_value;		  
				//UART_send_char(USART2,U2dat_value);	
			}
			else if((ReCont_2 == (UART2_ReBuff[2]+2)) && (U2dat_value == 0xEF)) 
			{	
				UART2_ReBuff[ReCont_2++] = U2dat_value;
			}
			else if((ReCont_2 == (UART2_ReBuff[2]+3)) && (U2dat_value == 0xFE)) 
			{			
				UART2_ReBuff[ReCont_2++] = U2dat_value;
				UART2_ReCont=ReCont_2;  
        UART2_Reflag=01; 				
				if(DJI_Motor_WriteData_In_Buff(UART2_ReBuff,ReCont_2))
				{
					Timer2_Counter1=0; //清空命令超时计数器	  
				}
				Reflag_2 =0x00; 	
			}
			else
			{
				Reflag_2=0; 
				ReCont_2=0;
			}
	}
	else
	{
			ReCont_2 =0;
			Reflag_2 = 0x00;
	}
	
  if(USART_GetITStatus(USART2, USART_FLAG_PE) != RESET)
	{   
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_PE);
	}

	if(USART_GetITStatus(USART2, USART_FLAG_ORE) != RESET)
	{   
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_ORE);
	}

	if(USART_GetITStatus(USART2, USART_FLAG_FE) != RESET)
	{   
		USART_ReceiveData(USART2);
		USART_ClearFlag(USART2, USART_FLAG_FE);
	}

	USART_ClearITPendingBit(USART2,USART_IT_RXNE);
}

 void USART3_IRQHandler(void)
{
	uint8_t ch;
	LED1_FLIP;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{ 	
		//ch = USART3->DR;
		ch = USART_ReceiveData(USART3);	  	 
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		UART_send_char(USART3,ch);
		
	} 
	if(USART_GetITStatus(USART3, USART_FLAG_PE) != RESET)
	{   
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_PE);
	}

	if(USART_GetITStatus(USART3, USART_FLAG_ORE) != RESET)
	{   
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_ORE);
	}

	if(USART_GetITStatus(USART3, USART_FLAG_FE) != RESET)
	{   
		USART_ReceiveData(USART3);
		USART_ClearFlag(USART3, USART_FLAG_FE);
	}

	USART_ClearITPendingBit(USART3,USART_IT_RXNE);
}
void SysTick_Handler(void)//系统精确延时中断服务函数
{
		if (TimingDelay != 0x00)
		{ 
			 TimingDelay--;
		}
	;
}

 /*外部中断服务函数书写格式*/
//void EXTI15_10_IRQnHandler(void)//PA（15~9）~PG（15~9）中断服务函数
//void EXTI9_5_IRQnHandler(void)//PA（5~9）~PG（5~9）中断服务函数
//void EXTI1_IRQHandler(void)//PA1~PG1 中断服务函数
//void EXTI2_IRQHandler(void)//PA2~PG2 中断服务函数
//void EXTI3_IRQHandler(void)//PA3~PG3 中断服务函数
//void EXTI4_IRQHandler(void)//PA4~PG4 中断服务函数

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) //确保是否产生了EXTI Line中断
	{
	 	//Delay_10us(500);//无法调用系统延时函数
		 
	
		EXTI_ClearITPendingBit(EXTI_Line0);     //清除中断标志位
	}  
	/*****
		if(EXTI_GetITStatus(EXTI_Linex) != RESET) //确保是否产生了EXTI Line中断
	{
	    ///------------------
	     用户函数
      ///-----------------
		EXTI_ClearITPendingBit(EXTI_Linex);     //清除中断标志位
	}  
	*****/
}
// EXTI Line --> PF9
void EXTI9_5_IRQHandler(void)
{ 
  if(EXTI_GetITStatus(EXTI_Line9) != RESET)
  {	
    
    EXTI_ClearITPendingBit(EXTI_Line9);
  }
}




void SDIO_IRQHandler(void) //在SDIO_ITConfig(）这个函数开启了sdio中断	， 数据传输结束时产生中断
{
		
	//SD_ProcessIRQSrc();// Process All SDIO Interrupt Sources 
}
	
	
void DMA1_Channel5_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC5)==SET)
   {
		  DMA_Cmd(DMA1_Channel5, DISABLE); 
			DMA_ClearFlag(DMA1_FLAG_TC5);//清中断标志，否则会一直中断
		  U1dat_value = RC_Callback_Handler(USART_RX_BUF);
			if(U1dat_value == 1)
			{
				UART1_DMA_Flag = 0x01;
				Timer2_Counter1 =0;  //清除通讯异常计数值
			}
			DMA_Cmd(DMA1_Channel5, ENABLE);  //启动DMA传输
   }
}
// CAN1 中断函数
void USB_LP_CAN1_RX0_IRQHandler(void)
 {
  CanRxMsg RxMessage;				 //CAN接收缓冲区
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);
	CAN_RxCpltCallback(&RxMessage);
	CAN1_Flag=0x01;	
 //if((RxMessage.ExtId==0x1234) && (RxMessage.IDE==CAN_ID_EXT)
 //&& (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDECA))

	LED3_FLIP;
}
	
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
