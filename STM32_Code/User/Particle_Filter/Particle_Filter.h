#ifndef Particle_Filter_H
#define Particle_Filter_H


#include "stm32f10x.h"
#include "stdio.h"//包含串口发送的FILE属性


#define Random_Data_MAX   2147483647.0

#define  Particle_M   50




void Particle_Process(double out_angle[3],int16_t GYRO[3]);
void Particle_Filter(float GYRO_X,float GYRO_Y,float Acc_angl_x,float Acc_angl_y);
void Kalman_MPU6050data_prepare(void);


#endif //Particle_Filter
