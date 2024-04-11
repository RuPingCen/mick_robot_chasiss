#ifndef _IMU_H_
#define _IMU_H_
 
#include "stm32f4xx.h"



typedef struct IMU_DAT
{
	 
	int16_t accADC[3];
	int16_t gyroADC[3];
	int16_t magADC[3];

	int16_t gyroOffset[3]; 
	float 	gyroRaw[3];		//rad/s 

	float   q[4];
	float   roll;				//deg
	float   pitch;
	float 	yaw;
	
	uint32_t last_time,curr_time,dt; 
	char update_flag;
		
}imu_Dat;
float invSqrt(float x); 
void IMU_Preper_Data(void);
void IMU_update_6Axis(float gx, float gy, float gz, float ax, float ay, float az);
void IMU_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

void MPU6050_Routing(void);
void MPU9250_Routing(void);


void IMU_Routing(void);
void IMU_Upload_Message(void);
#endif
