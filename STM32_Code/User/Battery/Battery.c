//******************************************************************************
// 描述： 利用 Modbus通信协议读取电池数据
//
//********************************************************************************

#include "Battery/Battery.h"
 
#include "string.h"
#include "bsp_uart.h"
#include <stdio.h>
#include <stdlib.h>
#include "bsp_gpio.h"
 

 

volatile Battery battery;
unsigned char RS485_TX_BUF[8];//电池发送缓存
volatile uint8_t RS485_Recv_Data[RS485_RX_Len];//电池接收数据
uint8_t RS485_Recv_Data_LEN = 0;//电池接收字节数量
 

void Battery_Init(void)
{
	My_Config_USART_Init(USART3,9600,1);
	UART_send_string(USART1,"Battery_Init with USART3 9600\n");
}
/**************************************************************
//函数名：ReadRegs
//描述：读取寄存器
//输入：bDevAddr-设备地址
//      wRegAddr-寄存器首地址
//			wRegNum-寄存器个数
//      pRecvBuf-接收数据缓存的地址指针
//输出：
	发送协议格式：ADR+0x03+起始寄存器高字节+起始寄存器低字节+寄存器数高字节+寄存器数低字节+CRC低字节+CRC高字节
	接收协议格式：ADR+0x03+字节总数+数据高字节+数据低字节+……+CRC低字节+CRC高字节
调用格式
	Battery_Read_Reg(01, 0x0000, 0x002F);//不能以0开头   

01 03 00 00 00 2F 04 16
01 03 00 01 00 2F 55 D6 



**************************************************************/	
void Battery_Read_Reg(uint8_t bDevAddr, uint16_t wRegAddr, uint16_t wRegNum)
{
	uint8_t len =0;
	uint16_t resCRC = 0;//计算的CRC
	RS485_TX_BUF[0] = bDevAddr;//设备地址
	RS485_TX_BUF[1] = 0x03;//功能码0x03FUNC_READMULTIREG
	RS485_TX_BUF[2] = HighByte(wRegAddr); //起始寄存器高字节 
	RS485_TX_BUF[3] = LowByte(wRegAddr); //起始寄存器低字节
	RS485_TX_BUF[4] = HighByte(wRegNum);//寄存器数高字节
	RS485_TX_BUF[5] = LowByte(wRegNum);//寄存器数低字节
	resCRC=Battery_CRC16(RS485_TX_BUF,6);//计算CRC校验值
	RS485_TX_BUF[6] = LowByte(resCRC);//CRC低字节
	RS485_TX_BUF[7] = HighByte(resCRC);//CRC高字节

	while(len<8)
	{
		USART_SendData(USART3,RS485_TX_BUF[len++]);
		while(USART_GetFlagStatus(USART3,USART_FLAG_TXE)==RESET);//等待发送区为空		
	}
//	printf("RS485 send data: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n",
//		RS485_TX_BUF[0],RS485_TX_BUF[1],RS485_TX_BUF[2],RS485_TX_BUF[3],
//		RS485_TX_BUF[4],RS485_TX_BUF[5],RS485_TX_BUF[6],RS485_TX_BUF[7]);
}
 
	
void Battery_Recived_Analy(void)
{
	uint16_t resCRC = 0;//计算的CRC
	uint16_t recvCRC = 0;//接收到的CRC
	int16_t temp_data=0; 

	uint16_t start,num;
	uint8_t i=0,j=0;
 
	if(RS485_Recv_Data[0] !=0x01)//从设备地址正确
	{
		printf("battery device ID is error ! 0x%x\n",RS485_Recv_Data[0]);
		return;
	}

	if(RS485_Recv_Data[1]!= 0x03)//功能码正确
	{
		printf("batterynfuncation ID is error! 0x%x\n",RS485_Recv_Data[1]);
		return;
	}
	
	RS485_Recv_Data_LEN = RS485_Recv_Data[2]+5;
	resCRC = Battery_CRC16(RS485_Recv_Data,RS485_Recv_Data_LEN-2);//计算所接收数据的CRC
	recvCRC = (RS485_Recv_Data[RS485_Recv_Data_LEN-1]<<8)|(RS485_Recv_Data[RS485_Recv_Data_LEN-2]);//接收到的CRC(低字节在前，高字节在后)
	if(resCRC!=recvCRC)//CRC校验正确
	{				
		printf("CRC check is error!\n");
		return;
	}
	
	start = ((RS485_TX_BUF[2]<<8)|RS485_TX_BUF[3]); //从发送信息里面读取开始地址、数据长度
	num = ((RS485_TX_BUF[4]<<8)|RS485_TX_BUF[5]);
 				
	//printf("start:%x num:%x\n",start,num);
	j=0;
	for(i=start;i<num;i++)//从第四个开始为数据
	{    
		temp_data= (RS485_Recv_Data[3+j*2]<<8)|RS485_Recv_Data[3+j*2+1];//数据高字节在前，低字节在后

		if(i==0) 
			battery.MonoOverVol=temp_data;
		else if(i==1)
			battery.OverRelVol=temp_data;
		else if(i==2)
			battery.MonoUnderVol=temp_data;
		else if(i==3)
			battery.UnderRelVol=temp_data;
		else if(i==4)
			battery.GroupOverVol=temp_data/100;
		else if(i==5)
			battery.GroOverRelVol=temp_data/100;
		else if(i==6)
			battery.GroupUnderVol=temp_data/100;
		else if(i==7)
			battery.GroUnderRelVol=temp_data/100;
		else if(i==8)
			battery.ChargeHighTemp=(temp_data-2731)/10;
		else if(i==9)
			battery.ChaHighRelTemp=(temp_data-2731)/10;
		else if(i==10)
			battery.ChargeLowTemp=(temp_data-2731)/10;
		else if(i==11)
			battery.ChaLowRelTemp=(temp_data-2731)/10;
		else if(i==12)
			battery.DischaHighTemp=(temp_data-2731)/10;
		else if(i==13)
			battery.DisHighRelTemp=(temp_data-2731)/10;
		else if(i==14)
			battery.DischaLowTemp=(temp_data-2731)/10;
		else if(i==15)
			battery.DisLowRelTemp=(temp_data-2731)/10;
		else if(i==18)
			battery.TotalVol=temp_data;
		else if(i==20)
			battery.SurplusCapacity=temp_data;
		else if(i==21)
			battery.Battery1Vol=temp_data;
		else if(i==22)
			battery.Battery2Vol=temp_data;
		else if(i==23)
			battery.Battery3Vol=temp_data;
		else if(i==24)
			battery.Battery4Vol=temp_data;
		else if(i==25)
			battery.Battery5Vol=temp_data;
		else if(i==26)
			battery.Battery6Vol=temp_data;
		else if(i==27)
			battery.Battery7Vol=temp_data;
		else if(i==28)
			battery.Battery8Vol=temp_data;
		else if(i==29)
			battery.Battery9Vol=temp_data;
		else if(i==30)
			battery.Battery10Vol=temp_data;
		else if(i==31)
			battery.Battery11Vol=temp_data;
		else if(i==32)
			battery.Battery12Vol=temp_data;
		else if(i==33)
			battery.Battery13Vol=temp_data;
		else if(i==34)
			battery.Battery14Vol=temp_data;
		else if(i==35)
			battery.Battery15Vol=temp_data;
		else if(i==36)
			battery.Battery16Vol=temp_data;//电池只有16个字节
//		else if(i==37)
//			battery.Battery17Vol=temp_data;
//		else if(i==38)
//			battery.Battery18Vol=temp_data;
//		else if(i==39)
//			battery.Battery19Vol=temp_data;
//		else if(i==40)
//			battery.Battery20Vol=temp_data;
		else if(i==37)
			battery.BatteryTemp1=(temp_data-2731)/10;
		else if(i==38)
			battery.BatteryTemp2=(temp_data-2731)/10;
//		else if(i==43)
//			battery.BatteryTemp3=(temp_data-2731)/10;
//		else if(i==44)
//			battery.BatteryTemp4=(temp_data-2731)/10;
		else if(i==39)
			battery.NominalCapacity=temp_data;
		else if(i==40)
			battery.RSOC=temp_data;
		else if(i==16)
		 {
				battery.ChargeOverCur = temp_data;
			  if(temp_data&0x8000)
				{
				  battery.ChargeOverCur =(65535-battery.ChargeOverCur);//放电电流
				}
				else
				{
					battery.ChargeOverCur =(battery.ChargeOverCur);									
				}
		 }										
		else if(i==17)
		 {
				battery.DischaOverCur = temp_data;
			  if(temp_data&0x8000)
				{
				  battery.DischaOverCur =(65535-battery.DischaOverCur);
				}
				else
				{
					battery.DischaOverCur =(battery.DischaOverCur);									
				}
		 }										 
		else if(i==19)
		 {
				battery.ChaDischaCur = temp_data;
			  if(temp_data&0x8000)
				{
				  battery.ChaDischaCur =(65535-(battery.ChaDischaCur));
				}
				else
				{
					battery.ChaDischaCur =(battery.ChaDischaCur);									
				}										
		 }								 
		 j=j+1;
	}
	battery.flag = 0x01;
	RS485_Recv_Data_LEN=0;								
}



/* 函数以 unsigned short 类型返回 CRC */
/* puchMsg 指向需要计算的数组,用于计算 CRC 的报文 */
/* usDataLen报文中的字节数 */
unsigned short Battery_CRC16 (unsigned char *puchMsg,unsigned short usDataLen)  
{
	unsigned char uchCRCHi = 0xFF ;     /* CRC 的高字节初始化 */
	unsigned char uchCRCLo = 0xFF ;     /* CRC 的低字节初始化 */
	unsigned uIndex ;                   /* CRC 查询表索引 */
	while (usDataLen--)  /* 完成整个报文缓冲区 */
	{
		uIndex = uchCRCLo ^ *puchMsg++ ; /* 计算 CRC */
		uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex];
		uchCRCHi = auchCRCLo[uIndex] ;
	}
	return (uchCRCHi << 8 | uchCRCLo) ;
}

void Battery_Debug(void)
{
//	printf("0  单体过压 %d \n",battery.MonoOverVol);
//	printf("1  释放电压 %d \n",battery.OverRelVol);
//	printf("2  单体欠压 %d \n",battery.MonoUnderVol);
//	printf("3  释放电压 %d \n",battery.UnderRelVol);	
//	printf("4  整组过压 %d \n",battery.GroupOverVol);
//	printf("5  释放电压 %d \n",battery.GroOverRelVol);
//	printf("6  整组欠压 %d \n",battery.GroupUnderVol);
//	printf("7  释放电压 %d \n",battery.GroUnderRelVol);	
//	 
//	printf("8  充电高温 %d \n",battery.ChargeHighTemp);
//	printf("9  释放温度 %d \n",battery.ChaHighRelTemp);
//	printf("10 充电低温 %d \n",battery.ChargeLowTemp);
//	printf("11 释放温度 %d \n",battery.ChaLowRelTemp);	
//	printf("12 放电高温 %d \n",battery.DischaHighTemp);
//	printf("13 释放温度 %d \n",battery.DisHighRelTemp);
//	printf("14 放电低温 %d \n",battery.DischaLowTemp);
//	printf("15 释放温度 %d \n",battery.DisLowRelTemp);	
//	 
//	printf("16 充电过流 %d \n",battery.ChargeOverCur);
//	printf("17 放电过流 %d \n",battery.DischaOverCur);
	printf("18 TotalVol: %d  (10mv)\n",battery.TotalVol);
	printf("19 ChaDischaCur %d (10mA)\n",battery.ChaDischaCur);	
	printf("20 SurplusCapacity %d (10mAH)\n",battery.SurplusCapacity);
	printf("21 Battery1Vol %d \n",battery.Battery1Vol);
	printf("22 Battery2Vol %d \n",battery.Battery2Vol);
	printf("23 Battery3Vol %d \n",battery.Battery3Vol);	
	printf("24 Battery4Vol %d \n",battery.Battery4Vol);
	printf("25 Battery5Vol %d \n",battery.Battery5Vol);
	printf("26 Battery6Vol %d \n",battery.Battery6Vol);	
	printf("27 Battery7Vol %d \n",battery.Battery7Vol);
	printf("28 Battery8Vol %d \n",battery.Battery8Vol);
	printf("29 Battery9Vol %d \n",battery.Battery9Vol);	
	 
	printf("30 Battery10Vol %d \n",battery.Battery10Vol);
	printf("31 Battery11Vol %d \n",battery.Battery11Vol);
	printf("32 Battery12Vol %d \n",battery.Battery12Vol);	
	printf("33 Battery13Vol %d \n",battery.Battery13Vol);
	printf("34 Battery14Vol %d \n",battery.Battery14Vol);
	printf("35 Battery15Vol %d \n",battery.Battery15Vol);	
	printf("36 Battery16Vol %d \n",battery.Battery16Vol);
	
	printf("41 BatteryTemp1 %d (0.1K)\n",battery.BatteryTemp1);	
	printf("42 BatteryTemp2 %d (0.1K)\n",battery.BatteryTemp2);	

	printf("45 NominalCapacity %d \n",battery.NominalCapacity);	
	printf("46 CapacityPercent %d \n",battery.RSOC);	
}
/***************************************************************************
* @brief       上传电池信息到PC上
* @retval 
* @maker    crp
* @date 2019-9-8
****************************************************************************/
void Battery_Upload_Message(void)
{
		unsigned char senddata[50];
		unsigned char i=0,j=0;	
		unsigned char cmd=0xA5;	
		unsigned int sum=0x00;	
		senddata[i++]=0xAE;
		senddata[i++]=0xEA;
		senddata[i++]=0x00;
		senddata[i++]=cmd;
		senddata[i++]=battery.TotalVol>>8; //总电压 10mv
		senddata[i++]=battery.TotalVol;
		senddata[i++]=battery.ChaDischaCur>>8;//电流 电流单位采用 10mA，带符号位，充电为正，放电为负
		senddata[i++]=battery.ChaDischaCur;
		senddata[i++]=battery.SurplusCapacity>>8; //剩余容量 10mAH
		senddata[i++]=battery.SurplusCapacity;
		senddata[i++]=battery.NominalCapacity>>8; //标称容量 10mAH
		senddata[i++]=battery.NominalCapacity;
		senddata[i++]=battery.RSOC; //百分比 1% - 100%

		senddata[i++]=battery.BatteryTemp1>>8; //温度1   单位0.1K
		senddata[i++]=battery.BatteryTemp1;
		senddata[i++]=battery.BatteryTemp2>>8; //温度2   单位0.1K
		senddata[i++]=battery.BatteryTemp2;
		
		for(j=2;j<i;j++)
			sum+=senddata[j];
		senddata[i++]=sum;
		senddata[2]=i-2; //数据长度
		senddata[i++]=0xEF;
		senddata[i++]=0xFE;
		senddata[i++]='\0';
		UART_send_buffer(USART1,senddata,i);
}

 //延时函数 6.3ms
void battery_delay(unsigned int delayvalue)
{
	unsigned int i;
	while(delayvalue-->0)
	{	
		i=5000;
		while(i-->0);
	}
}
//uint16_t Convert_16hex_to_10dec(uint16_t hex_num)
//{
//    char hex_str[10];
//    uint16_t dec_num;

//    sprintf(hex_str, "%X", hex_num);  // 使用 sprintf() 将十六进制数转化为字符串 
//    dec_num = strtoul(hex_str, NULL, 16);  // 使用 strtoul() 将字符串转化为十进制数

//    return dec_num;
//}
