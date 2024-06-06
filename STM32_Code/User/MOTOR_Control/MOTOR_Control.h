#ifndef __MOTOR_CONTROL_H
#define	__MOTOR_CONTROL_H

	 
#include "stm32f4xx.h"



typedef struct __command_t //������λ����������Ľṹ������
{
	volatile unsigned char cmd; //������
	
	volatile int16_t tag_rpm[6];	
	volatile int32_t tag_angl[6];
	volatile unsigned char rpm_available; //ָʾת�������Ƿ����
	volatile float tag_theta;
	volatile float tag_speed_w;
	volatile float tag_speed_x;
	volatile float tag_speed_y;
	volatile float tag_speed_z;
	volatile unsigned char speed_available;

	volatile unsigned char chassis_mode;  //����ģʽ
	volatile unsigned char flag;  //��־λ 0x01��ʾ���յ�һ֡���� ������
}command_t;





void DiffX4_Wheel_Speed_Model(float speed_x,float speed_w);
 
void AKM_Wheel_Speed_Model(float speed_x,float speed_w);
void AKM2_Wheel_Speed_Model(float speed_x,float speed_y,float speed_w);

void WSWD_Wheel_Speed_Model(float speed_x,float speed_y,float speed_w);// 4ws4wdȫ������

void Calculate_DiffX4_Odom(void);

void Chasiss_Control_Routing(void);
void ChasissDiffX4_Control_Routing(uint32_t cnt);
void Chasiss4WS4WD_Control_Routing(uint32_t cnt);
	
char Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength);

void Chassis_Motor_Upload_Message(void);
void Chassis_MotorState_Upload_Message(void);
void Chassis_Odom_Upload_Message(void);
#endif
