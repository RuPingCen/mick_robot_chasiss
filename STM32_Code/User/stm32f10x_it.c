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

extern volatile uint8_t UART1_DMA_Flag; //����1�жϱ�־λ
extern volatile uint8_t UART2_Flag; //����2�жϱ�־λ
extern volatile uint8_t CAN1_Flag;  //CAN���߽����жϱ�־

extern uint8_t USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�

extern rc_info_t rc;   
extern command_t recived_cmd; //���̽�����λ������ṹ��

extern volatile uint8_t UART2_ReBuff[100];  
extern volatile uint16_t UART2_ReCont;  
extern volatile unsigned char UART2_Reflag; 
extern volatile uint8_t TIM3_Flag;

uint16_t ReCont_2=0;  
unsigned char Reflag_2=0;  
unsigned char U2dat_value=0;
unsigned char U2dat_value_last=0;
unsigned char U1dat_value=0;

 
//extern void Matrix_Keyscan_Analy(unsigned int interupt_times);//���󰴼���ȡ
//extern void Keyscan_Analy(unsigned int interupt_times);//����������ȡ

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
void TIM2_IRQHandler(void)//��ʱ��2 �жϷ�����
{
	if ( TIM_GetITStatus(TIM2 , TIM_IT_Update) != RESET ) 
	{	
	  Timer2_Counter1++;
		Timer2_Counter2++;
		Timer2_Counter3++;	
		Timer2_Counter4++;
		Timer2_Counter5++;		
		
		if((Timer2_Counter1>100*10)) // �����ʱ����������1s��û�б����㣬˵��ͨѶ�������ж�
		{			
			if(rc.available == 0x00)
			{
				rc.sw1 = 5; //��ǽ������ݲ�����
				Mecanum_Wheel_Rpm_Model(0,0,0,0);
			}
			else if(recived_cmd.flag ==0x00) //��λ��û�г����������ݳ���1s�򴥷�ͨѶ��ʧ
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
void TIM3_IRQHandler(void)//��ʱ��3 �жϷ�����   Ŀǰδʹ��
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

// USRT2 ת�����յ������ݵ�UART3
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

		//���ݰ�����Э��
	U2dat_value=dat_value;
	
	if((U2dat_value == 0XEA )&& (U2dat_value_last == 0XAE))
	{
			ReCont_2 =0;
			Reflag_2 = 0x01;
			UART2_ReBuff[ReCont_2++] = U2dat_value_last;
			UART2_ReBuff[ReCont_2++] = U2dat_value; //�ҵ���֡ͷ
		  UART2_Flag=0x01;
		  
	}
	else if(Reflag_2 == 0x01 && ReCont_2>=2) 
	{  
			if(ReCont_2<3)
			{
				UART2_ReBuff[ReCont_2++] = U2dat_value; //�ҵ�����֡����	
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
					Timer2_Counter1=0; //������ʱ������	  
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
void SysTick_Handler(void)//ϵͳ��ȷ��ʱ�жϷ�����
{
		if (TimingDelay != 0x00)
		{ 
			 TimingDelay--;
		}
	;
}

 /*�ⲿ�жϷ�������д��ʽ*/
//void EXTI15_10_IRQnHandler(void)//PA��15~9��~PG��15~9���жϷ�����
//void EXTI9_5_IRQnHandler(void)//PA��5~9��~PG��5~9���жϷ�����
//void EXTI1_IRQHandler(void)//PA1~PG1 �жϷ�����
//void EXTI2_IRQHandler(void)//PA2~PG2 �жϷ�����
//void EXTI3_IRQHandler(void)//PA3~PG3 �жϷ�����
//void EXTI4_IRQHandler(void)//PA4~PG4 �жϷ�����

void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
	 	//Delay_10us(500);//�޷�����ϵͳ��ʱ����
		 
	
		EXTI_ClearITPendingBit(EXTI_Line0);     //����жϱ�־λ
	}  
	/*****
		if(EXTI_GetITStatus(EXTI_Linex) != RESET) //ȷ���Ƿ������EXTI Line�ж�
	{
	    ///------------------
	     �û�����
      ///-----------------
		EXTI_ClearITPendingBit(EXTI_Linex);     //����жϱ�־λ
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




void SDIO_IRQHandler(void) //��SDIO_ITConfig(���������������sdio�ж�	�� ���ݴ������ʱ�����ж�
{
		
	//SD_ProcessIRQSrc();// Process All SDIO Interrupt Sources 
}
	
	
void DMA1_Channel5_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA1_FLAG_TC5)==SET)
   {
		  DMA_Cmd(DMA1_Channel5, DISABLE); 
			DMA_ClearFlag(DMA1_FLAG_TC5);//���жϱ�־�������һֱ�ж�
		  U1dat_value = RC_Callback_Handler(USART_RX_BUF);
			if(U1dat_value == 1)
			{
				UART1_DMA_Flag = 0x01;
				Timer2_Counter1 =0;  //���ͨѶ�쳣����ֵ
			}
			DMA_Cmd(DMA1_Channel5, ENABLE);  //����DMA����
   }
}
// CAN1 �жϺ���
void USB_LP_CAN1_RX0_IRQHandler(void)
 {
  CanRxMsg RxMessage;				 //CAN���ջ�����
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
