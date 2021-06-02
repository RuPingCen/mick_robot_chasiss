#ifndef __DJI_MOTOR_H
#define	__DJI_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"




/*CAN���ͻ��ǽ��յ�ID*/
typedef enum
{

	CAN_TxPY12V_ID 	= 0x200,		//��̨12V����ID
	CAN_TxPY24V_ID	= 0x1FF,		//��̨12V����ID
//	CAN_Pitch_ID 	= 0x201,			//��̨Pitch
//	CAN_Yaw_ID   	= 0x203,			//��̨Yaw
	CAN_YAW_FEEDBACK_ID   = 0x205,		//��̨Yaw24v
	CAN_PIT_FEEDBACK_ID  = 0x206,			//��̨Yaw24v
	CAN_POKE_FEEDBACK_ID  = 0x207,
	CAN_ZGYRO_RST_ID 			= 0x404,
	CAN_ZGYRO_FEEDBACK_MSG_ID = 0x401,
	CAN_MotorLF_ID 	= 0x041,    //��ǰ
	CAN_MotorRF_ID 	= 0x042,		//��ǰ
	CAN_MotorLB_ID 	= 0x043,    //���
	CAN_MotorRB_ID 	= 0x044,		//�Һ�

	CAN_EC60_four_ID	= 0x200,	//EC60���ճ���
	CAN_backLeft_EC60_ID = 0x203, //ec60
	CAN_frontLeft_EC60_ID = 0x201, //ec60
	CAN_backRight_EC60_ID = 0x202, //ec60
	CAN_frontRight_EC60_ID = 0x204, //ec60
	
	//add by langgo
	CAN_3510Moto_ALL_ID = 0x200,
	CAN_3510Moto1_ID = 0x201,
	CAN_3510Moto2_ID = 0x202,
	CAN_3510Moto3_ID = 0x203,
	CAN_3510Moto4_ID = 0x204,
	CAN_DriverPower_ID = 0x80,

	CAN_HeartBeat_ID = 0x156,
	
}CAN_Message_ID;

 

#define FILTER_BUF_LEN		5
/*���յ�����̨����Ĳ����ṹ��*/
typedef struct{
		uint16_t 	angle;				//abs angle range:[0,8191] ���ת�Ǿ���ֵ
		uint16_t 	last_angle;	  //abs angle range:[0,8191]
	
		int16_t	 	speed_rpm;       //ת��
		int16_t  	real_current;    //ת��
	
		int16_t  	given_current;   //ʵ�ʵ�ת�ص���
		uint8_t  	Temp;           //�¶�

		uint16_t	offset_angle;   //�������ʱ�����ƫ�Ƕ�
		int32_t		round_cnt;     //���ת��Ȧ��
		int32_t		total_angle;    //���ת�����ܽǶ�
	
		uint8_t		buf_idx;
		uint16_t	angle_buf[FILTER_BUF_LEN];
		uint16_t	fited_angle;
		uint32_t	msg_cnt;
}moto_measure_t;


 
typedef struct __command_t //������λ����������Ľṹ������
{
	unsigned char cmd; //������
	
	int tag_rpm1;
	int tag_rpm2;
	int tag_rpm3;
	int tag_rpm4;
	unsigned char rpm_available; //ָʾת�������Ƿ����
	
	float tag_speed_x;
	float tag_speed_y;
	float tag_speed_z;
	unsigned char speed_available;
	
	unsigned char flag;  //��־λ 0x01��ʾ���յ�һ֡���� ������
}command_t;

 

void DJI_Motor_Init(void);
void DJI_Motor_Control(void);	

void DiffX4_Wheel_Speed_Model(float speed_x,float speed_w);
void DiffX4__Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4);
void Mecanum_Wheel_Speed_Model(int16_t speed_x,int16_t speed_y,int16_t speed_w);
void Mecanum_Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4); 


void DJI_Motor_Show_Message(void);
void DJI_Motor_Upload_Message(void);
char DJI_Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength);
void DJI_Motor_Clear_Odom(void);


void CAN_DJI_C620_DataSend( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_RxCpltCallback(CanRxMsg* RxMessage);
void get_moto_measure(moto_measure_t *ptr,CanRxMsg* RxMessage);
void get_moto_offset(moto_measure_t *ptr,CanRxMsg* RxMessage);
void get_total_angle(moto_measure_t *p);
  

#ifdef __cplusplus
}
#endif

#endif
