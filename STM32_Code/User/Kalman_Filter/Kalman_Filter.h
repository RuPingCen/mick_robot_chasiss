#ifndef _Kalman_Filter_H_
#define _Kalman_Filter_H_

#include "stm32f10x.h"



 
//���˾�����ʹ�ÿ������˲���ʱ�� ����ȡ��ͬ�ļ��ٶȵļ��㷶Χ������ĽǶ�
//����Ҫ��ͬ���̵���������������֮��ƥ�� �������ڻ�ȡ�����������ݻ����ϳ���һ��ϵ��
// �ڴ������ĵ�ѹֵ����ٶ�ת���Ĺ�������ʹ�õ����ԵĹ�ϵ�� ���������ֱ�Ӱ������Թ�ϵ����
//crp


extern void Kalman_Filter(float GYRO_X,float GYRO_Y,float Acc_angl_x,float Acc_angl_y);//Kalman�˲���100MHz�Ĵ���ʱ��Լ****ms��
extern void Kalman_Process(double out_angle[3],int16_t GYRO[3]);
extern void Kalman_mma7361data_prepare(void);
extern void Kalman_MPU6050data_prepare(void);

extern void Kalman_Filter_Verify(float GYRO_Y,float Acc_angl_x);

#endif
/************************************************************************
K60:ʹ��MMA7361 ʱ��ADC�ĳ�ʼ��
      LCD_240RGBx320_PutString( 0,150,"MMA_ACCEL_X:",Red,White);
         LCD_240RGBx320_PutString( 0,166,"MMA_ACCEL_Y:",Red,White);
         LCD_240RGBx320_PutString( 0,182,"MMA_ACCEL_Z:",Red,White);
         LCD_240RGBx320_PutString( 0,198,"MMA_GORY_X:",Red,White);
         LCD_240RGBx320_PutString( 0,214,"MMA_GORY_Y:",Red,White);
         LCD_240RGBx320_PutString( 0,230,"MMA_GORY_Z:",Red,White);  
         
          adc_init(ADC0,SE12); //AD��ʼ�������� PTB2
          adc_init(ADC0,SE13); //AD��ʼ�������� PTB3
          adc_init(ADC0,SE14); //AD��ʼ�������� PTC0 
          adc_init(ADC0,SE15); //AD��ʼ�������� PTC1
          adc_init(ADC1,SE14); //AD��ʼ�������� PTB10
          adc_init(ADC1,SE15); //AD��ʼ�������� PTB11
          
          adc_start(ADC0,SE12,ADC_16bit);// ����adc
          adc_start(ADC0,SE13,ADC_16bit);// ����adc
          adc_start(ADC0,SE14,ADC_16bit);// ����adc
          adc_start(ADC0,SE15,ADC_16bit);// ����adc
          adc_start(ADC1,SE14,ADC_16bit);// ����adc
          adc_start(ADC1,SE15,ADC_16bit);// ����adc             
         
*************************************************************************/
