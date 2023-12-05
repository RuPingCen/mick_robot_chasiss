#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h" 
#include "stm32f10x_Delay.h"
 
#include "Mick_IO.h"
#include "bsp_uart.h" 

uint8_t Code_Switch_Value = 0x00;

/***************************************************************************
* @brief       控制板外部IO初始化（LED、KEY、编码开关、隔离输入输出）
*
* @param[out]   编码值 0-15
* @param[in]   无
* @retval 
* @maker    crp
* @date 2020-6-1
***************************************************************************/
void Init_Mick_GPIO(void)
{

	My_GPIO_Init(GPIOA,GPIO_Pin_5,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);//初始化LED端口
	My_GPIO_Init(GPIOA,GPIO_Pin_6,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOC,GPIO_Pin_13,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);
	
	//按键端口 为输入
	My_GPIO_Init(GPIOA,GPIO_Pin_7,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_2,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	
	//拨码开关端口
	My_GPIO_Init(GPIOA,GPIO_Pin_15,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_3,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_4,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_5,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	
	//外部隔离输入
	My_GPIO_Init(GPIOB,GPIO_Pin_12,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_13,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_14,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_15,GPIO_Mode_IPU, GPIO_Speed_10MHz);
	
	//外部隔离输出
	My_GPIO_Init(GPIOB,GPIO_Pin_0,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_1,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_8,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);
	My_GPIO_Init(GPIOB,GPIO_Pin_9,GPIO_Mode_Out_PP, GPIO_Speed_10MHz);
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
	if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_15) == 0)
		value |= 0x08;
	else
		value &= 0xf7;
	
 	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_3) == 0)
		value |= 0x04;
	else
		value &= 0xfB;
	
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_4) == 0)
		value |= 0x02;
	else
		value &= 0xfD;
	
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_5) == 0)
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
			if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_7) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else if(ch == 2)
	{
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_2) == 0)
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
			if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else if(ch == 2)
	{
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else if(ch == 3)
	{
			if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13) == 0)
				key_value = 1;
			else
				key_value = 0;
	}
	else if(ch == 4)
	{
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12) == 0)
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
	{    //IO输出高电平，导致光耦导通，输出端接到GND上  否则为高阻态
			if(out_value)
				 GPIO_SetBits(GPIOB,GPIO_Pin_0);
			else
				 GPIO_ResetBits(GPIOB,GPIO_Pin_0);
	}
	else if(ch == 2)
	{
			if(out_value)
				 GPIO_SetBits(GPIOB,GPIO_Pin_1);
			else
				 GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	}
	else if(ch == 3)
	{
			if(out_value)
				 GPIO_SetBits(GPIOB,GPIO_Pin_8);
			else
				 GPIO_ResetBits(GPIOB,GPIO_Pin_8);
	}
	else if(ch == 4)
	{
			if(out_value)
				 GPIO_SetBits(GPIOB,GPIO_Pin_9);
			else
				 GPIO_ResetBits(GPIOB,GPIO_Pin_9);
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
		static  uint32_t IO_upload_counter=0;
		unsigned char senddat[35];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	
		uint8_t ch1,ch2,ch3,ch4;

		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0xAC; //命令位 0x11

		//上传数据帧计数
		senddat[i++]=(IO_upload_counter>>24);
		senddat[i++]=(IO_upload_counter>>16);
		senddat[i++]=(IO_upload_counter>>8);
		senddat[i++]=(IO_upload_counter);
			
		ch1 = Read_Isolated_Input(1);
		ch2 = Read_Isolated_Input(2);
		ch3 = Read_Isolated_Input(3);
		ch4 = Read_Isolated_Input(4);
	
		senddat[i++] = ch1; 
		senddat[i++] = ch2;
		senddat[i++] = ch3; 
		senddat[i++] = ch4;

		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
    senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		IO_upload_counter++;
}
