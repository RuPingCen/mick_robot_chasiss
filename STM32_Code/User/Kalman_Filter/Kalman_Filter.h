#ifndef _Kalman_Filter_H_
#define _Kalman_Filter_H_

#include "stm32f10x.h"



 
//个人觉得在使用卡尔曼滤波的时候： 对于取不同的加速度的计算范围计算出的角度
//是需要不同量程的陀螺仪数据来与之相匹配 所以我在获取的陀螺仪数据基础上乘上一个系数
// 在传感器的电压值与角速度转换的过程中是使用的线性的关系的 因此我这里直接按照线性关系处理
//crp


extern void Kalman_Filter(float GYRO_X,float GYRO_Y,float Acc_angl_x,float Acc_angl_y);//Kalman滤波，100MHz的处理时间约****ms；
extern void Kalman_Process(double out_angle[3],int16_t GYRO[3]);
extern void Kalman_mma7361data_prepare(void);
extern void Kalman_MPU6050data_prepare(void);

extern void Kalman_Filter_Verify(float GYRO_Y,float Acc_angl_x);

#endif
/************************************************************************
K60:使用MMA7361 时对ADC的初始化
      LCD_240RGBx320_PutString( 0,150,"MMA_ACCEL_X:",Red,White);
         LCD_240RGBx320_PutString( 0,166,"MMA_ACCEL_Y:",Red,White);
         LCD_240RGBx320_PutString( 0,182,"MMA_ACCEL_Z:",Red,White);
         LCD_240RGBx320_PutString( 0,198,"MMA_GORY_X:",Red,White);
         LCD_240RGBx320_PutString( 0,214,"MMA_GORY_Y:",Red,White);
         LCD_240RGBx320_PutString( 0,230,"MMA_GORY_Z:",Red,White);  
         
          adc_init(ADC0,SE12); //AD初始化，采样 PTB2
          adc_init(ADC0,SE13); //AD初始化，采样 PTB3
          adc_init(ADC0,SE14); //AD初始化，采样 PTC0 
          adc_init(ADC0,SE15); //AD初始化，采样 PTC1
          adc_init(ADC1,SE14); //AD初始化，采样 PTB10
          adc_init(ADC1,SE15); //AD初始化，采样 PTB11
          
          adc_start(ADC0,SE12,ADC_16bit);// 启动adc
          adc_start(ADC0,SE13,ADC_16bit);// 启动adc
          adc_start(ADC0,SE14,ADC_16bit);// 启动adc
          adc_start(ADC0,SE15,ADC_16bit);// 启动adc
          adc_start(ADC1,SE14,ADC_16bit);// 启动adc
          adc_start(ADC1,SE15,ADC_16bit);// 启动adc             
         
*************************************************************************/
