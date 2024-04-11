/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "bsp_systick.h"
#include "bsp_can.h" 
#include "bsp_uart.h"
#include "bsp_usart_dma.h"

#include "Battery/Battery.h" 
#include "DBUS/DBUS.h" 
#include "MOTOR_RMD/MOTOR_RMD.h" 
#include "MOTOR_APSL2DB/MOTOR_APSL2DB.h" 
#include "MOTOR_Control/MOTOR_Control.h"  
#include "Mick_IO/Mick_IO.h"

extern volatile uint32_t TimingDelay;

extern volatile uint8_t flag_can1;	
extern volatile uint8_t flag_can2;	
extern CanRxMsg CAN1_RxMessage;
extern CanRxMsg CAN2_RxMessage;

extern volatile uint32_t Timer2_Counter1;
extern volatile uint32_t Timer2_Counter2;
extern volatile uint32_t Timer2_Counter3;
extern volatile uint32_t Timer2_Counter4;
extern volatile uint32_t Timer2_Counter5;

extern volatile uint32_t Timer6_Counter2; // IMU上传
volatile uint32_t Timer6_Counter1; //电机控制 


extern volatile uint8_t UART2_DMA_Flag; //串口2中断标志位
//extern volatile uint8_t UART1_Flag; //串口2中断标志位
//extern volatile uint8_t CAN1_Flag;  //CAN总线接收中断标志

extern uint8_t USART2_RX_DMA_BUF[USART2_MAX_RX_LEN];  // 串口2 DMA接收缓存
 

volatile uint8_t UART6_last_dat=0,UART6_Re_cnt=0;
extern volatile uint8_t UART3_Flag; //串口6中断标志位
volatile uint8_t UART3_Rec_status; //数据接收状态 0 等待帧头 1表示接收数据 2表示接收完成
extern volatile uint8_t USART3_RX_BUF[RS485_RX_Len];  // 串口6 接收缓存
extern volatile uint8_t RS485_Recv_Data[RS485_RX_Len];//电池接收数据


// 上位机串口数据接收
uint16_t ReCont_2=0;  
unsigned char Reflag_2=0;  
unsigned char U1dat_value=0;
unsigned char U1dat_value_last=0;
extern volatile uint8_t UART1_ReBuff[100];  
extern volatile uint16_t UART1_ReCont;  
extern volatile unsigned char UART1_Reflag; 
extern volatile uint8_t UART1_Flag;
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
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

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)//系统精确延时中断服务函数
{
	if (TimingDelay != 0x00)
	{ 
		 TimingDelay--;
	}
	 
}

void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)!=RESET)
	{		
		//GPIO_ToggleBits(GPIOF,GPIO_Pin_6);
		
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
	}	 
}
void TIM6_DAC_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM6,TIM_IT_Update)!=RESET)
	{		
		Timer6_Counter1++;
		Timer6_Counter2++; //IMU 上传
		ChasissDiffX4_Control_Routing(Timer6_Counter1);
		
		if(Timer6_Counter1 >= 4)
		{
			Timer6_Counter1=0;
			 
		}
			
		TIM_ClearITPendingBit(TIM6,TIM_IT_Update);
	}	 
}
void TIM7_IRQHandler(void)
{	if(TIM_GetITStatus(TIM7,TIM_IT_Update)!=RESET)
	{
		Timer2_Counter1++; //检测遥控器 通讯是否超时
		Timer2_Counter2++; //检测上位机 通讯是否超时
		Timer2_Counter3++; //电池读取
		Timer2_Counter4++; //IO上传任务
		Timer2_Counter5++;  
		Chasiss_Control_Routing();		
		TIM_ClearITPendingBit(TIM7,TIM_IT_Update);
	}	 
}

/**串口中断函数**/ // 上位机指令
void USART1_IRQHandler(void)
{
	uint8_t ucTemp;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)!=RESET)
	{		
		USART_ClearFlag(USART1, USART_FLAG_RXNE);
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
		ucTemp = USART_ReceiveData( USART1 );
		
		LED1_FLIP;
		//USART_SendData(USART1,ucTemp);    
 
		U1dat_value_last = U1dat_value;
		//数据包解析协议
		U1dat_value=ucTemp;
		
		if((U1dat_value == 0XEA )&& (U1dat_value_last == 0XAE))
		{
				ReCont_2 =0;
				Reflag_2 = 0x01;
				UART1_ReBuff[ReCont_2++] = U1dat_value_last;
				UART1_ReBuff[ReCont_2++] = U1dat_value; //找到了帧头
				UART1_Flag=0x01;
			  
		}
		else if(Reflag_2 == 0x01 && ReCont_2>=2) 
		{  
				if(ReCont_2<3)
				{
					UART1_ReBuff[ReCont_2++] = U1dat_value; //找到数据帧长度	
				}
				else if(ReCont_2 < (UART1_ReBuff[2]+2)) 
				{
					UART1_ReBuff[ReCont_2++] = U1dat_value;		  
					//UART_send_char(USART2,U1dat_value);	
				}
				else if((ReCont_2 == (UART1_ReBuff[2]+2)) && (U1dat_value == 0xEF)) 
				{	
					UART1_ReBuff[ReCont_2++] = U1dat_value;
				}
				else if((ReCont_2 == (UART1_ReBuff[2]+3)) && (U1dat_value == 0xFE)) 
				{			
					UART1_ReBuff[ReCont_2++] = U1dat_value;
					UART1_ReCont=ReCont_2;  
					UART1_Reflag=01; 				
					if(Motor_WriteData_In_Buff(UART1_ReBuff,ReCont_2))
					{
						Timer2_Counter2=0;
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
	}
	if(USART_GetITStatus(USART1, USART_FLAG_PE) != RESET)
	{   
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_PE);
	}

	if(USART_GetITStatus(USART1, USART_FLAG_ORE) != RESET)
	{   
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_ORE);
	}

	if(USART_GetITStatus(USART1, USART_FLAG_FE) != RESET)
	{   
		USART_ReceiveData(USART1);
		USART_ClearFlag(USART1, USART_FLAG_FE);
	}

	USART_ClearITPendingBit(USART1,USART_IT_RXNE);
}
void USART2_IRQHandler(void)
{
	//uint8_t ucTemp;
	if(USART_GetITStatus(USART2,USART_IT_RXNE)!=RESET)
	{		
		//ucTemp = USART_ReceiveData( USART2 );
		//GPIO_ToggleBits(GPIOF,GPIO_Pin_7);
		//printf("A");    
	}	 
}

void USART3_IRQHandler(void)
{
	uint8_t ucTemp;
	if(USART_GetITStatus(USART3,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData(USART3);
		//GPIO_ToggleBits(GPIOF,GPIO_Pin_6); 
		if(UART3_Rec_status==0x00) //寻找帧头
		{
			if(UART6_last_dat == 0x01 && ucTemp==0x03)
			{
				UART6_Re_cnt =0;
				RS485_Recv_Data[UART6_Re_cnt++] = UART6_last_dat;
				RS485_Recv_Data[UART6_Re_cnt++] = ucTemp;
				UART3_Rec_status = 0x01;
			}
		}
		else if(UART3_Rec_status == 0x01 && UART6_Re_cnt>=2) //找到帧头
		{
			if(UART6_Re_cnt<3)
			{
				RS485_Recv_Data[UART6_Re_cnt++] = ucTemp; //找到数据帧长度
			}
			else if(UART6_Re_cnt < (RS485_Recv_Data[2]+5)) 
			{
				RS485_Recv_Data[UART6_Re_cnt++] = ucTemp;					
				if(UART6_Re_cnt == (RS485_Recv_Data[2]+5)) 
				{
					
					UART3_Rec_status = 0x02; // 数据接收完成
					UART3_Flag=1;  
					//memcpy(USART3_RX_BUF,RS485_Recv_Data,RS485_RX_Len);
				}
			}
			else
			{
				UART3_Rec_status = 0x00;
				UART6_Re_cnt=0;
			}				
			
			if(UART6_Re_cnt>=RS485_RX_Len)
			{
				UART3_Rec_status = 0x00;
				UART6_Re_cnt=0; 
			}
		}
		else
		{
			UART3_Rec_status = 0x00;
			UART6_Re_cnt =0; 
		}
		
		UART6_last_dat =ucTemp;    
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
void UART4_IRQHandler(void)
{
	uint8_t ucTemp;
	if(USART_GetITStatus(UART4,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData( UART4 );
	 
		USART_SendData(UART4,ucTemp);    
	}	 
}
void UART5_IRQHandler(void)
{
	uint8_t ucTemp;
	if(USART_GetITStatus(UART5,USART_IT_RXNE)!=RESET)
	{		
		ucTemp = USART_ReceiveData( UART5 );
		 
		USART_SendData(UART5,ucTemp);    
	}	 
}
void USART6_IRQHandler(void)
{
	//uint8_t ucTemp;
	if(USART_GetITStatus(USART6,USART_IT_RXNE)!=RESET)
	{		
		//ucTemp = USART_ReceiveData( USART6 );
		
	}	 
	
	if(USART_GetITStatus(USART6, USART_FLAG_PE) != RESET)
	{   
		USART_ReceiveData(USART6);
		USART_ClearFlag(USART6, USART_FLAG_PE);
	}

	if(USART_GetITStatus(USART6, USART_FLAG_ORE) != RESET)
	{   
		USART_ReceiveData(USART6);
		USART_ClearFlag(USART6, USART_FLAG_ORE);
	}

	if(USART_GetITStatus(USART6, USART_FLAG_FE) != RESET)
	{   
		USART_ReceiveData(USART6);
		USART_ClearFlag(USART6, USART_FLAG_FE);
	}

	USART_ClearITPendingBit(USART6,USART_IT_RXNE);
} 

/*  外部中断 回调函数  */
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{	
		//GPIO_ToggleBits(GPIOF,GPIO_Pin_6);
		
		
		EXTI_ClearITPendingBit(EXTI_Line0);
	}

}

void EXTI15_10_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{	
		//GPIO_ToggleBits(GPIOF,GPIO_Pin_7);

		EXTI_ClearITPendingBit(EXTI_Line13);
	}
}


void CAN1_RX_IRQHandler(void)
{
	// CAN_IT_FMP0 标志位通过读取FIFO里面的数值后系统自动进行清除
	if(CAN_GetITStatus(CAN1,CAN_IT_FMP0) == SET)
     {
        CAN_Receive(CAN1,CAN_FIFO0,&CAN1_RxMessage);
		MOTOR_APSL2DB_Measurements_Analy();
        flag_can1 = 1;
     }
}
void CAN2_RX_IRQHandler(void)
{
	// CAN_IT_FMP0 标志位通过读取FIFO里面的数值后系统自动进行清除
	if(CAN_GetITStatus(CAN2,CAN_IT_FMP0) == SET)
     {
        CAN_Receive(CAN2,CAN_FIFO0,&CAN2_RxMessage);
		//MOTOR_APSL2DB_Measurements_Analy();
		MOTOR_RMD_State_Analysis();
        flag_can2 = 1;
     }
}
void DMA1_Stream5_IRQHandler(void) // 串口2  DMA接收 中断
{
	unsigned char U1dat_value=0;
    if(DMA_GetITStatus(DMA1_Stream5,DMA_IT_TCIF5) != RESET)                       //DMA接收完成标志
    {
        //DMA_Cmd(DMA1_Stream5,DISABLE);
        DMA_ClearITPendingBit(DMA1_Stream5,DMA_IT_TCIF5);                         //清除DMA接收中断标志
        DMA1_Stream5->NDTR = USART2_MAX_RX_LEN;
		//DMA_SetCurrDataCounter(DMA1_Stream5, USART_MAX_RX_LEN);
        DMA_Cmd(DMA1_Stream5,ENABLE);
		
		U1dat_value =  RC_Callback_Handler(USART2_RX_DMA_BUF);
		if(U1dat_value)
		{
			UART2_DMA_Flag = 0x01;
			Timer2_Counter1 =0;  //清除通讯异常计数值
		}
    }
	 
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
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


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
