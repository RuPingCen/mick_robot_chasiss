#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "bsp_gpio.h"

#include "Mick_IO/Mick_IO.h"
#include "bsp_uart.h" 

volatile uint8_t Isolated_Output_Value = 0x00; //�洢����˿ڵ�״̬

/***************************************************************************
* @brief       ���ư��ⲿIO��ʼ����LED��KEY�����뿪�ء��������������  MickV3
* @param[out]   ����ֵ 0-15
* @param[in]   ��
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
void Init_Mick_GPIO(void)
{
// led1 = PF13
// led2 = PF14
// led3 = PF15
	
// ���� key1 = PE0  key2 = PE1
	
// sw1 = PE11
// sw2 = PE12
// sw3 = PE13
// sw4 = PE14
	
//  IN1 = PE2
//  IN2 = PE3
//  IN3 = PE4
//  IN4 = PE5

//  Out1 = PF5
//  Out2 = PF4
//  Out3 = PF3
//  Out4 = PF2

 
	// LED��
	My_GPIO_Init(GPIOF, GPIO_Pin_13, GPIO_Mode_OUT);
	My_GPIO_Init(GPIOF, GPIO_Pin_14, GPIO_Mode_OUT);
	My_GPIO_Init(GPIOF, GPIO_Pin_15, GPIO_Mode_OUT);
	
	//�����˿� Ϊ����
	My_GPIO_Init(GPIOE,GPIO_Pin_0,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_1,GPIO_Mode_IN);
	
	//���뿪�ض˿�
	My_GPIO_Init(GPIOE,GPIO_Pin_11,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_12,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_13,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_14,GPIO_Mode_IN);
	
	//�ⲿ��������
	My_GPIO_Init(GPIOE,GPIO_Pin_2,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_3,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_4,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_5,GPIO_Mode_IN);
	
	//�ⲿ�������
	My_GPIO_Init(GPIOF,GPIO_Pin_5,GPIO_Mode_OUT);
	My_GPIO_Init(GPIOF,GPIO_Pin_4,GPIO_Mode_OUT);
	My_GPIO_Init(GPIOF,GPIO_Pin_3,GPIO_Mode_OUT);
	My_GPIO_Init(GPIOF,GPIO_Pin_2,GPIO_Mode_OUT);
	
	
	
	Set_Isolated_Output(1,0);
	Set_Isolated_Output(2,0);
	Set_Isolated_Output(3,0);
	Set_Isolated_Output(4,0);
}

/***************************************************************************
* @brief       ��ȡ4·���뿪�ض�����б���,���뷽ʽ������ 1->4 ��Ӧ��λ����λ
*               1 �����λ   4�����λ
*
* @param[out]   ����ֵ 0-15
* @param[in]   ��
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
uint8_t Read_Code_Switch(void)
{
	uint8_t value=0;
	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_11) == 0)
		value |= 0x08;
	else
		value &= 0xf7;
	
 	if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_12) == 0)
		value |= 0x04;
	else
		value &= 0xfB;
	
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_13) == 0)
		value |= 0x02;
	else
		value &= 0xfD;
	
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14) == 0)
		value |= 0x01;
	else
		value &= 0xfe;
	
	return value;
}
/***************************************************************************
* @brief       ��ȡ���� IO�ڵ�״̬
* @param[out]   1 ��������  0 ����   0xff ��������
* @param[in]   ͨ���� 1 ��ʾ����key1 2��ʾ����key2
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
uint8_t Read_Key(uint8_t ch)
{
	uint8_t key_value=0xff;
	if(ch == 1)
	{
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0) == 0)
			key_value = 1;
		else
			key_value = 0;
	}
	else if(ch == 2)
	{
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_1) == 0)
			key_value = 1;
		else
			key_value = 0;
	}
	else
		;
	
 	return key_value;
}

/***************************************************************************
* @brief       ��ȡ��������ͨ����״̬
* @param[out]   1 �ߵ�λ  0 GNG   0xff ��������
* @param[in]    ͨ���� 1-4 ��Ӧ4��ͨ��
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
uint8_t Read_Isolated_Input(uint8_t ch)
{
	uint8_t key_value=0xff;
	if(ch == 1)
	{    //�ⲿ����VCC ���¹��ͨ��IO��ƽΪ0
			if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_5) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else if(ch == 2)
	{
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else if(ch == 3)
	{
			if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_3) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else if(ch == 4)
	{
		if(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_2) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else
		;
	
 	return key_value;
}
/***************************************************************************
* @brief       ���ø��������״̬
* @param[out]    
* @param[in]    ͨ�� ch�� 1-4 ��Ӧ4��ͨ��    out_value:���ֵ��0��ʾGND 1��ʾ����̬
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
void Set_Isolated_Output(uint8_t ch, uint8_t out_value)
{
	if(ch == 1)
	{    //IO����ߵ�ƽ��1�������¹��ͨ������˽ӵ�GND��  ����Ϊ����̬
			if(out_value)
			{
				GPIO_SetBits(GPIOF,GPIO_Pin_5);
				Isolated_Output_Value |= 0x01;
			}
			else
			{
				GPIO_ResetBits(GPIOF,GPIO_Pin_5);
				Isolated_Output_Value &= 0xfe;
			}
	}
	else if(ch == 2)
	{
			if(out_value)
			{
				GPIO_SetBits(GPIOF,GPIO_Pin_4);
				Isolated_Output_Value |= 0x02;
			}
			else
			{
				GPIO_ResetBits(GPIOF,GPIO_Pin_4);
				Isolated_Output_Value &= 0xfd;
			}
	}
	else if(ch == 3)
	{
			if(out_value)
			{
				GPIO_SetBits(GPIOF,GPIO_Pin_3);
				Isolated_Output_Value |= 0x04;
			}
			else
			{
				GPIO_ResetBits(GPIOF,GPIO_Pin_3);
				Isolated_Output_Value &= 0xfB;
			}
	}
	else if(ch == 4)
	{
			if(out_value)
			{
				GPIO_SetBits(GPIOF,GPIO_Pin_2);
				Isolated_Output_Value |= 0x08;
			}
			else
			{
				GPIO_ResetBits(GPIOF,GPIO_Pin_2);
				Isolated_Output_Value &= 0xf7;
			}
	}
	else
		;

}
 
/***************************************************************************
* @brief       ��ȡ4��IO״̬�ϴ���PC
* @param[out]    
* @param[in]     
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
void Isolated_IO_Upload_Message(void)
{
	unsigned char senddat[35];
	unsigned char i=0,j=0,Isolated_Input_Value;	
	unsigned int sum=0x00;	
 
	senddat[i++]=0xAE;
	senddat[i++]=0xEA;
	senddat[i++]=0x01;//���ݳ����ں��渳ֵ
	senddat[i++]=0xAC; //����λ 0xAC


	senddat[i++] = Isolated_Output_Value;
	
	Isolated_Input_Value = 0x00;
	Isolated_Input_Value = (Read_Isolated_Input(4)<<4)|(Read_Isolated_Input(3)<<3)|(Read_Isolated_Input(2)<<2)|Read_Isolated_Input(1);
	senddat[i++] = Isolated_Input_Value;

	senddat[2]=i-1; //���ݳ���
	for(j=2;j<i;j++)
		sum+=senddat[j];
	senddat[i++]=sum;

	senddat[i++]=0xEF;
	senddat[i++]=0xFE;
	 
	//UART_send_string(USART2,senddat);
	UART_send_buffer(USART1,senddat,i);

}



