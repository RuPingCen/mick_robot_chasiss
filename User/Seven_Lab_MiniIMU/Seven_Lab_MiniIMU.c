#include "stm32f10x.h"
 
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h"
//#include "stm32f10x_usart.h"

#include <stdio.h>
#include "bsp_uart.h" 
//#include "NRF_24L01.h"
#include "Seven_Lab_MiniIMU.h"
#include "IMU.h"


#define  IMU_Reprot_Char(a)     UART_send_char(USART2,a) 

extern imu_Dat IMU_Data;//IMU数据结构体

//extern unsigned char NRF24l01_RX_BUFF[RX_PLOAD_WIDTH];	 // 初始化发送与接收缓冲区
Imu_Report  AHRS_Data;

void IMU_Report_AHRSdata(void)
{
     AHRS_Data_Preper_MPU9250();
     IMU_Report_AHRS_BasicData();
     UART1_ReportAHRS();
}

 /****************************************************************
*
* 函数名：   AHRS_Data_Preper
*
* 函数功能： 对姿态数据进行赋值到结构体中去 
*
* 参数说明：   
*
* 备注： CRP   20150503
*  
****************************************************************/
void AHRS_Data_Preper_NRF24l01(void)
{
	// NRF24l01_RX_BUFF[RX_PLOAD_WIDTH]
	 
//	AHRS_Data.acc[0]=(NRF24l01_RX_BUFF[0]<<8)+ NRF24l01_RX_BUFF[1];
//	AHRS_Data.acc[1]=(NRF24l01_RX_BUFF[2]<<8)+ NRF24l01_RX_BUFF[3];
//	AHRS_Data.acc[2]=(NRF24l01_RX_BUFF[4]<<8)+ NRF24l01_RX_BUFF[5];


//	AHRS_Data.gyro[0]=(NRF24l01_RX_BUFF[6]<<8)+ NRF24l01_RX_BUFF[7];
//	AHRS_Data.gyro[1]=(NRF24l01_RX_BUFF[8]<<8)+ NRF24l01_RX_BUFF[9];
//	AHRS_Data.gyro[2]=(NRF24l01_RX_BUFF[10]<<8)+ NRF24l01_RX_BUFF[11];

//	AHRS_Data.mag[0]=(NRF24l01_RX_BUFF[12]<<8)+ NRF24l01_RX_BUFF[13];
//	AHRS_Data.mag[1]=(NRF24l01_RX_BUFF[14]<<8)+ NRF24l01_RX_BUFF[15];
//	AHRS_Data.mag[2]=(NRF24l01_RX_BUFF[16]<<8)+ NRF24l01_RX_BUFF[17];


//	AHRS_Data.roll=(NRF24l01_RX_BUFF[18]<<8)+ NRF24l01_RX_BUFF[19];
//	AHRS_Data.pitch=(NRF24l01_RX_BUFF[20]<<8)+ NRF24l01_RX_BUFF[21];
//	AHRS_Data.yaw=(NRF24l01_RX_BUFF[22]<<8)+ NRF24l01_RX_BUFF[23];



//	AHRS_Data.altitude=(NRF24l01_RX_BUFF[24]<<8)+ NRF24l01_RX_BUFF[25];   //此处应该吧高度数据扩大十倍后赋值         上位机处自动缩小十倍
//	AHRS_Data.Temperature=(NRF24l01_RX_BUFF[26]<<8)+ NRF24l01_RX_BUFF[27];//此处应该吧温度数据扩大十倍后赋值         上位机处自动缩小十倍
//	AHRS_Data.pressure=(NRF24l01_RX_BUFF[28]<<8)+ NRF24l01_RX_BUFF[29];//此处应该吧压强数据 缩小 十倍后赋值         上位机处自动扩大十倍
//	AHRS_Data.updatafreq=(NRF24l01_RX_BUFF[30]<<8)+ NRF24l01_RX_BUFF[31]; //姿态结算速度  每秒多少次
 
}
void AHRS_Data_Preper_MPU9250(void)
{
	// NRF24l01_RX_BUFF[RX_PLOAD_WIDTH]
	 
	AHRS_Data.acc[0]=IMU_Data.accADC[0];
	AHRS_Data.acc[1]=IMU_Data.accADC[1];
	AHRS_Data.acc[2]=IMU_Data.accADC[2];


	AHRS_Data.gyro[0]=IMU_Data.gyroADC[0];
	AHRS_Data.gyro[1]=IMU_Data.gyroADC[1];
	AHRS_Data.gyro[2]=IMU_Data.gyroADC[2];

	AHRS_Data.mag[0]=IMU_Data.magADC[0];
	AHRS_Data.mag[1]=IMU_Data.magADC[1];
	AHRS_Data.mag[2]=IMU_Data.magADC[2];


	AHRS_Data.roll=IMU_Data.roll*10;
	AHRS_Data.pitch=IMU_Data.pitch*10;
	AHRS_Data.yaw=IMU_Data.yaw*10;



	AHRS_Data.altitude=0;   //此处应该吧高度数据扩大十倍后赋值         上位机处自动缩小十倍
	AHRS_Data.Temperature=200;//此处应该吧温度数据扩大十倍后赋值         上位机处自动缩小十倍
	AHRS_Data.pressure=200;//此处应该吧压强数据 缩小 十倍后赋值         上位机处自动扩大十倍
	AHRS_Data.updatafreq=200; //姿态结算速度  每秒多少次
 
}
void IMU_Report_AHRS_BasicData(void)//上传原始参数
{
 	unsigned int temp=0xaF+9;
	char ctemp;
	IMU_Reprot_Char(0xa5);
	IMU_Reprot_Char(0x5a);
	IMU_Reprot_Char(14+8);
	IMU_Reprot_Char(0xA2);//表示加速度、陀螺仪、罗盘原始数据

	if(AHRS_Data.acc[0]<0)AHRS_Data.acc[0]=32768-AHRS_Data.acc[0];
	ctemp=AHRS_Data.acc[0]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.acc[0];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.acc[1]<0)AHRS_Data.acc[1]=32768-AHRS_Data.acc[1];
	ctemp=AHRS_Data.acc[1]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.acc[1];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.acc[2]<0)AHRS_Data.acc[2]=32768-AHRS_Data.acc[2];
	ctemp=AHRS_Data.acc[2]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.acc[2];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.gyro[0]<0)
		 AHRS_Data.gyro[0]=32768-AHRS_Data.gyro[0];
	ctemp=AHRS_Data.gyro[0]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.gyro[0];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.gyro[1]<0)
		AHRS_Data.gyro[1]=32768-AHRS_Data.gyro[1];
	ctemp=AHRS_Data.gyro[1]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.gyro[1];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
//-------------------------
	if(AHRS_Data.gyro[2]<0)AHRS_Data.gyro[2]=32768-AHRS_Data.gyro[2];
	ctemp=AHRS_Data.gyro[2]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.gyro[2];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp; 

	if(AHRS_Data.mag[0]<0)
		AHRS_Data.mag[0]=32768-AHRS_Data.mag[0];
	ctemp=AHRS_Data.mag[0]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.mag[0];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.mag[1]<0)
		AHRS_Data.mag[1]=32768-AHRS_Data.mag[1];
	ctemp=AHRS_Data.mag[1]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.mag[1];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.mag[2]<0)
		AHRS_Data.mag[2]=32768-AHRS_Data.mag[2];
	ctemp=AHRS_Data.mag[2]>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.mag[2];
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	IMU_Reprot_Char(temp%256);
	IMU_Reprot_Char(0xaa);
}

 
 

void UART1_ReportAHRS(void)
{
 	unsigned int temp=0xaF+2+2;
	char ctemp;
	IMU_Reprot_Char(0xa5);
	IMU_Reprot_Char(0x5a);
	IMU_Reprot_Char(14+4);
	IMU_Reprot_Char(0xA1);//表示结算后的角度数据

	if(AHRS_Data.yaw<0)
		AHRS_Data.yaw=32768-AHRS_Data.yaw;
	ctemp=AHRS_Data.yaw>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.yaw;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.pitch<0)
		AHRS_Data.pitch=32768-AHRS_Data.pitch;
	ctemp=AHRS_Data.pitch>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.pitch;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.roll<0)
		AHRS_Data.roll=32768-AHRS_Data.roll;
	ctemp=AHRS_Data.roll>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.roll;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
 
	if(AHRS_Data.altitude<0)
		AHRS_Data.altitude=32768-AHRS_Data.altitude;
	ctemp=AHRS_Data.altitude>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.altitude;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.Temperature<0)
		AHRS_Data.Temperature=32768-AHRS_Data.Temperature;
	ctemp=AHRS_Data.Temperature>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.Temperature;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	if(AHRS_Data.pressure<0)
		AHRS_Data.pressure=32768-AHRS_Data.pressure;
	ctemp=AHRS_Data.pressure>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.pressure;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	ctemp=AHRS_Data.updatafreq>>8;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;
	ctemp=AHRS_Data.updatafreq;
	IMU_Reprot_Char(ctemp);
	temp+=ctemp;

	IMU_Reprot_Char(temp%256);
	IMU_Reprot_Char(0xaa);
}


