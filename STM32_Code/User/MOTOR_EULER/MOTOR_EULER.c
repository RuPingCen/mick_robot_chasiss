/*
* ŷ����챵���������õ��ʹ��CAN��422��UART���ַ�ʽ����
* CAN���߱��ıȽϸ��� ��Ҫʹ�ö�����Ľ��д���
*
*/
#include "stm32f4xx.h"
#include "string.h"
#include "stdlib.h"
 
#include "bsp_uart.h" 
#include "bsp_timer.h"
#include "bsp_usart_dma.h" 

#include "MOTOR_EULER.h" 
 
 
void MOTOR_EULER_Init(void)
{
	My_Config_USART_Init(UART5,115200,1);
	UART_send_string(UART5,"USART5 Chassiss for MixkX4 .....\n");
	MOTOR_EULER_Set_Speed(0,0,0x02);
}
  
void MOTOR_EULER_Set_Speed(int16_t Speed1,int16_t Speed2,uint8_t FuncByte)
{
//						Byte[9]��������λ:Bit0: 1:��̼�����	Bit2&Bit1: 00:���ɻ��� 01:���� 11:ɲ�� 10:����
//						�����Byte[10]����λ����Ϊ0x00,����ᱻʶ���'\0',�����Ͳ��ܷ������������ݰ� 
//						Byte[5],Byte[6]���Speed1	Byte[7],Byte[8]���Speed2	Byte[9]��������λ	Byte[11] CRCУ��
//						  0    1    2    3    4    5    6    7     8   9    10   11   12
	uint8_t Motor_Cmd[13] = {0xAA,0x55,0x01,0x03,0x04,0x00,0x00,0x00,0x00,0x02,0x01,0x00,0xCC};
	unsigned char i;
	Motor_Cmd[5] = (uint8_t)((Speed1 & 0xFF00) >> 8);
	Motor_Cmd[6] = (uint8_t)(Speed1 & 0x00FF);
	Motor_Cmd[7] = (uint8_t)((Speed2 & 0xFF00) >> 8);
	Motor_Cmd[8] = (uint8_t)(Speed2 & 0x00FF);
	Motor_Cmd[9] = FuncByte;
	Motor_Cmd[11] = 0x00;
	for ( uint8_t i = 0 ; i < 11 ; i ++)
	{
		Motor_Cmd[11] += Motor_Cmd[i];
	}
	Motor_Cmd[11] = Motor_Cmd[11] & 0x00FF;
	 

	for ( i = 0 ; i < 13 ; i ++)
	{
		USART_SendData(UART5,Motor_Cmd[i]);
		while ( USART_GetFlagStatus(UART5,USART_FLAG_TXE) == RESET);
	}
	while ( USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
}
