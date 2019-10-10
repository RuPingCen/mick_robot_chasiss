 
#include "stm32f10x.h"
#include "DBUS.h" 
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h"


unsigned char DBUS_flag=0;
//uint32_t ch1_offset_sum=0,ch2_offset_sum=0,ch3_offset_sum=0,ch4_offset_sum=0;
uint32_t rc_counter=0;

rc_info_t dbus_rc;

/**
  * @brief       DBUS串口接收回调函数 
  * @param[out]  rc:   转换为每个通道的数据
  * @param[in]   pData: 输入长度为18字节的数据
  * @retval 
  */
void rc_callback_handler(uint8_t *pData)
{
	if(pData == NULL)
	{
		return;
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
	
	if(DBUS_flag == DBUS_INIT) //初始化中间位置
	{
	  rc_offset_init();
		DBUS_flag = DBUS_RUN;
	}
	else
	{
		dbus_rc.cnt =dbus_rc.v +1;	
		dbus_rc.available =0x01;
	}
	
	
}
void rc_offset_init(void)
{
//  ch1_offset_sum+=dbus_rc.ch1;
//	ch2_offset_sum+=dbus_rc.ch2;
//	ch3_offset_sum+=dbus_rc.ch3;
//	ch4_offset_sum+=dbus_rc.ch4;
//	rc_counter++;
//	if(rc_counter>3)
//	{
//	  ch1_offset_sum = ch1_offset_sum/rc_counter;
//		ch2_offset_sum = ch2_offset_sum/rc_counter;
//		ch3_offset_sum = ch3_offset_sum/rc_counter;
//		ch4_offset_sum = ch4_offset_sum/rc_counter;
//		
//		dbus_rc.ch1_offset =ch1_offset_sum;
//		dbus_rc.ch2_offset =ch2_offset_sum;
//		dbus_rc.ch3_offset =ch3_offset_sum;
//		dbus_rc.ch4_offset =ch4_offset_sum;
//		
//		dbus_rc.available =0x01;
//		dbus_rc.cnt =rc_counter;
//		
//	}
		dbus_rc.ch1_offset =dbus_rc.ch1;
		dbus_rc.ch2_offset =dbus_rc.ch2;
		dbus_rc.ch3_offset =dbus_rc.ch3;
		dbus_rc.ch4_offset =dbus_rc.ch4;
		
		dbus_rc.available =0x00;
		dbus_rc.cnt =rc_counter;


}
void rc_show_message(void)
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

// DBUS上传信息到PC上
void rc_upload_message(void)
{
		unsigned char senddata[50];
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