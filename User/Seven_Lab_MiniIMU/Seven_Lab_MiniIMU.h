#ifndef _Seven_Lab_MiniIMU_H_
#define _Seven_Lab_MiniIMU_H_

 
#include "stm32f10x.h"





typedef struct IMU_REPORT
{
	 
		int16_t acc[3];
		int16_t gyro[3];
		int16_t mag[3];
	
 
		int16_t   roll;	//deg			
		int16_t   pitch;
		int16_t 	yaw;
	
	  int16_t   altitude;	 
		int16_t   Temperature;
		int16_t 	pressure;
	
	  uint16_t updatafreq;
	
		
}Imu_Report;


void AHRS_Data_Preper_NRF24l01(void);
void AHRS_Data_Preper_MPU9250(void);

void IMU_Report_AHRSdata(void);
void IMU_Report_AHRS_BasicData(void);//上传原始参数
void UART1_ReportAHRS(void);




#endif
