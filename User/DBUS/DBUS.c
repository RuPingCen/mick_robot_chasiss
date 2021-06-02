#include "stm32f10x.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"

#include "DBUS.h" 
#include "DJI_Motor.h"

unsigned char DBUS_flag=0;
uint32_t ch1_offset_sum=0,ch2_offset_sum=0,ch3_offset_sum=0,ch4_offset_sum=0;
uint32_t rc_counter=0;

rc_info_t dbus_rc;

/***************************************************************************
* @brief       DBUS串口接收回调函数 
* @param[out]  rc:   转换为每个通道的数据
* @param[in]   pData: 输入长度为18字节的数据
* @retval 
* @maker    crp
* @data 2019-9-8
***************************************************************************/
char RC_Callback_Handler(uint8_t *pData)
{
	if(pData == NULL)
	{
		return 0;
	}
	
	dbus_rc.ch1 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	dbus_rc.ch2 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))	& 0x07FF;
	dbus_rc.ch3 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	dbus_rc.ch4 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &0x07FF;
	dbus_rc.sw1 = ((pData[5] >> 4) & 0x000C) >> 2;
	dbus_rc.sw2 = ((pData[5] >> 4) & 0x0003);
	dbus_rc.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	dbus_rc.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	dbus_rc.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
	dbus_rc.press_l = pData[12];
	dbus_rc.press_r = pData[13];
	dbus_rc.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
	
	if(DBUS_flag == DBUS_INIT) 
	{
		dbus_rc.available =0x00;
		if(RC_Offset_Init())
			DBUS_flag = DBUS_RUN;
		else
			DBUS_flag = DBUS_INIT;
	}
	else
	{
		if((dbus_rc.ch1 > 2000) || (dbus_rc.ch1<100) 
			|| (dbus_rc.ch3 > 2000) || (dbus_rc.ch3<100)
		  || (dbus_rc.sw1 > 3) || (dbus_rc.sw1<1)
		  || (dbus_rc.sw2 > 3) || (dbus_rc.sw2<1))
		{
			dbus_rc.available =0x00;
			return 0;
		}
		else
		{
			dbus_rc.available =0x01;
			DBUS_Routing();
		}
		dbus_rc.cnt =dbus_rc.v +1;	
	}
	
	return 1;
}
/***************************************************************************
* @brief       成功接收到一帧DBUS数据以后调用该函数设定PID控制的目标值
* @retval 
* @maker    crp
* @data 2019-9-8
****************************************************************************/
void DBUS_Routing(void)
{
	if((dbus_rc.sw1 !=1) && (dbus_rc.available)) //使能遥控器模式
	{
		if(dbus_rc.sw2 ==1) //1 档模式 最大1m/s
		{
			DiffX4_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.00152,(dbus_rc.ch1-dbus_rc.ch1_offset)*0.00152);
		}
		else if(dbus_rc.sw2 ==3) //2 档模式 最大2m/s
		{
			DiffX4_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.00304,(dbus_rc.ch1-dbus_rc.ch1_offset)*0.00304);
		}
		else if(dbus_rc.sw2 ==2) //3 档模式 最大3.5m/s
		{
			DiffX4_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.0053,(dbus_rc.ch1-dbus_rc.ch1_offset)*0.0053);
		}
		else
			DiffX4_Wheel_Speed_Model(0,0);
	}
}



/***************************************************************************
* @brief       遥控器初始化函数 采集10次数据求取平均值
* @param[out]  校准完成返回1 否则返回0
* @param[in]    
* @retval 
* @maker    crp
* @data 2019-9-8
****************************************************************************/
char RC_Offset_Init(void)
{
	if((dbus_rc.ch1>1000) && (dbus_rc.ch1<1050))
		if((dbus_rc.ch2>1000) && (dbus_rc.ch2<1050))
			if((dbus_rc.ch3>1000) && (dbus_rc.ch3<1050))
				if((dbus_rc.ch4>1000) && (dbus_rc.ch4<1050))
				{
						ch1_offset_sum+=dbus_rc.ch1;
						ch2_offset_sum+=dbus_rc.ch2;
						ch3_offset_sum+=dbus_rc.ch3;
						ch4_offset_sum+=dbus_rc.ch4;
						rc_counter++;
				}

	if(rc_counter>10)
	{
	  ch1_offset_sum = ch1_offset_sum/rc_counter;
		ch2_offset_sum = ch2_offset_sum/rc_counter;
		ch3_offset_sum = ch3_offset_sum/rc_counter;
		ch4_offset_sum = ch4_offset_sum/rc_counter;
		
		dbus_rc.ch1_offset =ch1_offset_sum;
		dbus_rc.ch2_offset =ch2_offset_sum;
		dbus_rc.ch3_offset =ch3_offset_sum;
		dbus_rc.ch4_offset =ch4_offset_sum;
		
		//calibration failed 
		if((dbus_rc.ch1_offset ==0) || (dbus_rc.ch2_offset ==0) || (dbus_rc.ch3_offset ==0) || (dbus_rc.ch4_offset ==0))
		{
			dbus_rc.available =0x00; 
		  rc_counter=0;
			ch1_offset_sum = 0;
			ch2_offset_sum = 0;
			ch3_offset_sum = 0;
			ch4_offset_sum = 0;
			
			return 0;
		}
		else
		{
			dbus_rc.available =0x01; 
			dbus_rc.cnt =rc_counter;
			
			return 1;
		}
	}
	
	dbus_rc.available =0x00;
	dbus_rc.cnt =rc_counter;
	
	return 0;
}

/***************************************************************************
* @brief       试遥控器，打印数据
* @retval 
* @maker    crp
* @data 2019-9-8
****************************************************************************/
void RC_Debug_Message(void)
{
	UART_send_string(USART2,"SBUS:  ch1:");UART_send_data(USART2,dbus_rc.ch1);UART_send_char(USART2,'\t');		
	UART_send_string(USART2,"ch2:");UART_send_data(USART2,dbus_rc.ch2);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch3:");UART_send_data(USART2,dbus_rc.ch3);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch4:");UART_send_data(USART2,dbus_rc.ch4);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"sw1:");UART_send_data(USART2,dbus_rc.sw1);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"sw2:");UART_send_data(USART2,dbus_rc.sw2);UART_send_char(USART2,'\n');	
	
	UART_send_string(USART2,"ch1_offset:");UART_send_data(USART2,dbus_rc.ch1_offset);UART_send_char(USART2,'\t');		
	UART_send_string(USART2,"ch2_offset:");UART_send_data(USART2,dbus_rc.ch2_offset);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch3_offset:");UART_send_data(USART2,dbus_rc.ch3_offset);UART_send_char(USART2,'\t');	
	UART_send_string(USART2,"ch4_offset:");UART_send_data(USART2,dbus_rc.ch4_offset);UART_send_char(USART2,'\n');	
	
	UART_send_char(USART2,'\n');	
}

/***************************************************************************
* @brief       DBUS上传信息到PC上
* @retval 
* @maker    crp
* @data 2019-9-8
****************************************************************************/

void RC_Upload_Message(void)
{
		char senddata[50];
		unsigned char i=0,j=0;	
		unsigned char cmd=0x03;	
		unsigned int sum=0x00;	
		senddata[i++]=0xAE;
		senddata[i++]=0xEA;
		senddata[i++]=0x00;
		senddata[i++]=cmd;
		senddata[i++]=dbus_rc.ch1>>8;
		senddata[i++]=dbus_rc.ch1;
		senddata[i++]=dbus_rc.ch2>>8;
		senddata[i++]=dbus_rc.ch2;
		senddata[i++]=dbus_rc.ch3>>8;
		senddata[i++]=dbus_rc.ch3;
		senddata[i++]=dbus_rc.ch4>>8;
		senddata[i++]=dbus_rc.ch4;
		senddata[i++]=dbus_rc.sw1;
		senddata[i++]=dbus_rc.sw1;
		for(j=2;j<i;j++)
			sum+=senddata[j];
		senddata[i++]=sum;
		senddata[2]=i-2; //数据长度
		senddata[i++]=0xEF;
		senddata[i++]=0xFE;
		senddata[i++]='\0';
		UART_send_string(USART2,senddata);
}
