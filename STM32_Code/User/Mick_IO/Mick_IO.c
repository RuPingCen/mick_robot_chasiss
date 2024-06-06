#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "bsp_gpio.h"

#include "Mick_IO/Mick_IO.h"
#include "bsp_uart.h" 

volatile uint8_t Isolated_Output_Value = 0x00; //存储输出端口的状态

/***************************************************************************
* @brief       控制板外部IO初始化（LED、KEY、编码开关、隔离输入输出）  MickV3
* @param[out]   编码值 0-15
* @param[in]   无
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
void Init_Mick_GPIO(void)
{
// led1 = PF13
// led2 = PF14
// led3 = PF15
	
// 按键 key1 = PE0  key2 = PE1
	
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

 
	// LED灯
	My_GPIO_Init(GPIOF, GPIO_Pin_13, GPIO_Mode_OUT);
	My_GPIO_Init(GPIOF, GPIO_Pin_14, GPIO_Mode_OUT);
	My_GPIO_Init(GPIOF, GPIO_Pin_15, GPIO_Mode_OUT);
	
	//按键端口 为输入
	My_GPIO_Init(GPIOE,GPIO_Pin_0,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_1,GPIO_Mode_IN);
	
	//拨码开关端口
	My_GPIO_Init(GPIOE,GPIO_Pin_11,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_12,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_13,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_14,GPIO_Mode_IN);
	
	//外部隔离输入
	My_GPIO_Init(GPIOE,GPIO_Pin_2,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_3,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_4,GPIO_Mode_IN);
	My_GPIO_Init(GPIOE,GPIO_Pin_5,GPIO_Mode_IN);
	
	//外部隔离输出
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
* @brief       读取4路编码开关对其进行编码,编码方式从左到右 1->4 对应高位到低位
*               1 是最高位   4是最低位
*
* @param[out]   编码值 0-15
* @param[in]   无
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
* @brief       读取按键 IO口的状态
* @param[out]   1 按键按下  0 按键   0xff 参数有误
* @param[in]   通道： 1 表示按键key1 2表示按键key2
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
* @brief       读取隔离输入通道的状态
* @param[out]   1 高电位  0 GNG   0xff 参数有误
* @param[in]    通道： 1-4 对应4个通道
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
uint8_t Read_Isolated_Input(uint8_t ch)
{
	uint8_t key_value=0xff;
	if(ch == 1)
	{    //外部输入VCC 导致光耦导通，IO电平为0
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
* @brief       设置隔离输出的状态
* @param[out]    
* @param[in]    通道 ch： 1-4 对应4个通道    out_value:输出值，0表示GND 1表示高组态
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
void Set_Isolated_Output(uint8_t ch, uint8_t out_value)
{
	if(ch == 1)
	{    //IO输出高电平（1），导致光耦导通，输出端接到GND上  否则为高阻态
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
* @brief       读取4个IO状态上传到PC
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
	senddat[i++]=0x01;//数据长度在后面赋值
	senddat[i++]=0xAC; //命令位 0xAC


	senddat[i++] = Isolated_Output_Value;
	
	Isolated_Input_Value = 0x00;
	Isolated_Input_Value = (Read_Isolated_Input(4)<<4)|(Read_Isolated_Input(3)<<3)|(Read_Isolated_Input(2)<<2)|Read_Isolated_Input(1);
	senddat[i++] = Isolated_Input_Value;

	senddat[2]=i-1; //数据长度
	for(j=2;j<i;j++)
		sum+=senddat[j];
	senddat[i++]=sum;

	senddat[i++]=0xEF;
	senddat[i++]=0xFE;
	 
	//UART_send_string(USART2,senddat);
	UART_send_buffer(USART1,senddat,i);

}



