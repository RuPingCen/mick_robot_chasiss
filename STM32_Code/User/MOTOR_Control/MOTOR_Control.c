/*

*���ִ�С   0.258m
*���������ּ��   0.74m
* ǰ���ּ��  0.8m 
*/


#include "stm32f4xx.h"
#include "math.h"
#include "bsp_uart.h" 
#include "bsp_usart_dma.h" 
#include "bsp_systick.h"
#include "MOTOR_APSL2DB/MOTOR_APSL2DB.h" 
#include "MOTOR_RMD/MOTOR_RMD.h" 
#include "MOTOR_Control/MOTOR_Control.h"  
#include "DBUS/DBUS.h" 
#include "Mick_IO/Mick_IO.h"
#include "IMU.h"

extern imu_Dat IMU_Data;//IMU���ݽṹ��

#define Chassis_MAX_Speed  2.2  // ��������ٶ�
#define Chassis_Parameter_L  0.4 // ǰ���־�
#define Chassis_Parameter_lw  0.4 // ���Ҳ��־� 
#define PI 3.1415926

volatile uint16_t run_cnt=0;

volatile float odom_vx=0,odom_vy=0,odom_w=0;
volatile uint32_t last_time=0,curr_time=0; //���ڼ���ʱ�� ��ȡ����ʱ��

//������1������ת�浽���� UART1_ReBuff��
volatile uint8_t UART1_ReBuff[100];  
volatile uint16_t UART1_ReCont;  
volatile unsigned char UART1_Reflag; 
command_t recived_cmd;  // ����1�д���λ��������������

extern rc_info_t rc;  //ң����
extern volatile uint32_t Timer2_Counter1,Timer2_Counter2;//ң��������λ��ͨѶ�����ʱ

extern volatile moto_measure_t moto_chassis[4];//�н����״̬
extern volatile moto_measure_t moto_rmd_chassis[4];//ת����״̬

/**
* @funtion	 4�ֲ���
* @Param	 speed_w ��Χ�� [-1 1]  ��λ rad/s
* @Retval	 None
* @Date     2020/8/16
* @maker    crp
*/
void DiffX4_Wheel_Speed_Model_APSL(float speed_x,float speed_w)
{
	recived_cmd.speed_available = 0x01;
 
	float v1=0,v2=0,v3=0,v4=0;
	 
	if(speed_x>Chassis_MAX_Speed)
		speed_x =Chassis_MAX_Speed;
	else if(speed_x < -Chassis_MAX_Speed)
		speed_x = -Chassis_MAX_Speed;
	
	speed_w = -speed_w;
	
	if((speed_x>-0.05) && (speed_x<0.05)) 
		speed_x=0;
	
	if((speed_w>-0.05) && (speed_w<0.05)) 
		speed_w=0;

	v1 = speed_x+speed_w*Chassis_Parameter_L;
	v2 = v1; 
	
	v4 = -(speed_x-speed_w*Chassis_Parameter_L);
	v3 = v4;
//	printf("speed_x: %f \tspeed_w %f \n ",speed_x,speed_w);
//	printf("v1-4: %f \t %f\t%f\t%f\t \n ",v1,v2,v3,v4);
	// ʹ�õ���6.5Ӣ������� ֱ��Ϊ17cm  
	recived_cmd.tag_rpm[0] = -(112.344668*v1)/1; // ���ٶȵ�RPM��ת��ϵ����   60/(PI*D) = 112.344668
	recived_cmd.tag_rpm[1] = -(112.344668*v2)/1;  // �������ĵ�����ת�����ĵľ����� 0.4
	recived_cmd.tag_rpm[2] = -(112.344668*v3)/1;   // speed_w�ķ�Χ������1 �Ƿ���Ҫ����һ��pi ������ת�ٶ���pi/s
	recived_cmd.tag_rpm[3] = -(112.344668*v4)/1;
 
}
 
/**
* @funtion	 ����ԭ��ת����ٶ�
* @Param	 speed_w ��Χ�� [-1 1]  ��λ rad/s
* @Retval	 None
* @Date     2020/8/16
* @maker    crp
*/
void DiffX4_Wheel_Speed_Model(float speed_w)
{
	recived_cmd.chassis_mode = 0x01;
	recived_cmd.tag_angl[0] = -4500;
	recived_cmd.tag_angl[1] = 4500;
	recived_cmd.tag_angl[2] = -4500;
	recived_cmd.tag_angl[3] = 4500;

	recived_cmd.tag_rpm[0] = (37.604*speed_w)/1; // ���ٶȵ�RPM��ת��ϵ����  70.02556
	recived_cmd.tag_rpm[1] = (37.604*speed_w)/1;  // �������ĵ�����ת�����ĵľ����� 0.537
	recived_cmd.tag_rpm[2] = (37.604*speed_w)/1;   // speed_w�ķ�Χ������1 �Ƿ���Ҫ����һ��pi ������ת�ٶ���pi/s
	recived_cmd.tag_rpm[3] = (37.604*speed_w)/1;
}
// ���ְ�����ģ��
//theta �����뷶Χ�� [-90 90]  ��λ m/s
//void AKM_Wheel_Speed_Model(float speed_x,float speed_w)
//{
//	 
//	recived_cmd.chassis_mode = 0x02;
//	int8_t dir_theta =1 ,dir_speed=1; //��ʾת����ٶȵ�������
//	float theta_o = 0,theta_i=0;
//	float v_o = 0, v_i = 0;
//	if((fabs(speed_w) > 1e-3) && fabs(speed_x) > 1e-2)
//	{
//		float R = speed_x/speed_w; // ��V��СW�ܴ��ʱ�򣬴���R-Chassis_Parameter_lw*0.5 С��0�����
//		if(speed_w<0)
//				dir_theta =-1;
//		if(speed_x<0)
//				dir_speed =-1;
//		if(speed_x>0) // ��Ӧǰ���ǰ���������
//		{
//			theta_i = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //�ڲ���̥ת��
//			theta_o = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //�����̥ת��
//			
//			if(R > 0 && R <= Chassis_Parameter_lw*0.5 ) //���ת��뾶�ڳ����ڣ����ڲ����Ӳ�ת
//				v_i = 0;
//			else
//				v_i = dir_speed*fabs(speed_w)*fabs(R-Chassis_Parameter_lw*0.5);
//			
//			if(R < 0 && R >= -Chassis_Parameter_lw*0.5 )
//				v_o = 0;
//			else
//				v_o = dir_speed*fabs(speed_w)*fabs(R+Chassis_Parameter_lw*0.5);
//		}              		
//		else if(speed_x<0) //  ������� R<0(�ٶ�С��0��������ʱ������ת)     //  ������� R>0(�ٶ�С��0������˳ʱ��ת��)
//		{
//			theta_i = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //�ڲ���̥ת��
//			theta_o = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //�����̥ת��
//			
//			if(R < 0 && R >= -Chassis_Parameter_lw*0.5)
//				v_i=0;
//			else
//				v_i = dir_speed*fabs(speed_w)*fabs(R+Chassis_Parameter_lw*0.5);
//			
//			if(R > 0 && R <= Chassis_Parameter_lw*0.5)
//				v_o = 0;
//			else
//				v_o = dir_speed*fabs(speed_w)*fabs(R-Chassis_Parameter_lw*0.5);
//		}
//		else ;

//	}
//	else
//	{
//		theta_o=0;
//		theta_i=0;
//		v_o = speed_x;
//		v_i = speed_x;
//	}
//		
//	theta_o = theta_o*57.3;
//	theta_i = theta_i*57.3;
//	
//	
//	if(theta_o>50) 	theta_o=50; //����ת�Ƿ�Χ
//	else  if(theta_o<-50) theta_o=-50;
//	else;

//	if(theta_i>50) 	theta_i=50;
//	else  if(theta_i<-50) theta_i=-50;
//	else;
// 
//	recived_cmd.tag_angl[0] = (theta_i*100)/1;
//	recived_cmd.tag_angl[1] = 0;
//	recived_cmd.tag_angl[2] = 0;
//	recived_cmd.tag_angl[3] = (theta_o*100)/1;
//	
//	recived_cmd.tag_rpm[0] = -(v_i*70.02556)/1;
//	recived_cmd.tag_rpm[1] = -(v_i*70.02556)/1;
//	recived_cmd.tag_rpm[2] = (v_o*70.02556)/1;
//	recived_cmd.tag_rpm[3] = (v_o*70.02556)/1;
//}
void AKM_Wheel_Speed_Model(float speed_x,float speed_w)
{
	 
	recived_cmd.chassis_mode = 0x02;
	int8_t dir_theta =1 ,dir_speed=1; //��ʾת����ٶȵ�������
	float theta_o = 0,theta_i=0;
	float v_o = 0, v_i = 0;
	if((fabs(speed_w) > 1e-3) && fabs(speed_x) > 1e-2)
	{
		float R = speed_x/speed_w; // ��V��СW�ܴ��ʱ�򣬴���R-Chassis_Parameter_lw*0.5 С��0�����
		if(speed_w<0)
				dir_theta =-1;
		if(speed_x<0)
				dir_speed =-1;
		
	 if( R< 0 && R >= -Chassis_Parameter_lw*0.5)  //������Сת��뾶
		R =-Chassis_Parameter_lw*0.5 - 0.2;
	 if( R >0  && R <= Chassis_Parameter_lw*0.5)
		R = Chassis_Parameter_lw*0.5 + 0.2;
			
			
		if(speed_x>0) // ��Ӧǰ���ǰ���������
		{
			theta_i = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //�ڲ���̥ת��
			theta_o = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //�����̥ת��
			
			if(R > 0 && R <= Chassis_Parameter_lw*0.5 ) //���ת��뾶�ڳ����ڣ����ڲ����Ӳ�ת
				v_i = 0;
			else
				v_i = dir_speed*fabs(speed_w)*fabs(R-Chassis_Parameter_lw*0.5);
			
			if(R < 0 && R >= -Chassis_Parameter_lw*0.5 )
				v_o = 0;
			else
				v_o = dir_speed*fabs(speed_w)*fabs(R+Chassis_Parameter_lw*0.5);
		}              		
		else if(speed_x<0) //  ������� R<0(�ٶ�С��0��������ʱ������ת)     //  ������� R>0(�ٶ�С��0������˳ʱ��ת��)
		{
			theta_i = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //�ڲ���̥ת��
			theta_o = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //�����̥ת��
			
			if(R < 0 && R >= -Chassis_Parameter_lw*0.5)
				v_i=0;
			else
				v_i = dir_speed*fabs(speed_w)*fabs(R+Chassis_Parameter_lw*0.5);
			
			if(R > 0 && R <= Chassis_Parameter_lw*0.5)
				v_o = 0;
			else
				v_o = dir_speed*fabs(speed_w)*fabs(R-Chassis_Parameter_lw*0.5);
		}
		else ;

	}
	else
	{
		theta_o=0;
		theta_i=0;
		v_o = speed_x;
		v_i = speed_x;
	}
		
	theta_o = theta_o*57.3;
	theta_i = theta_i*57.3;
	
	
	if(theta_o>50) 	theta_o=50; //����ת�Ƿ�Χ
	else  if(theta_o<-50) theta_o=-50;
	else;

	if(theta_i>50) 	theta_i=50;
	else  if(theta_i<-50) theta_i=-50;
	else;
 
	recived_cmd.tag_angl[0] = (theta_i*100)/1;
	recived_cmd.tag_angl[1] = 0;
	recived_cmd.tag_angl[2] = 0;
	recived_cmd.tag_angl[3] = (theta_o*100)/1;
	
	recived_cmd.tag_rpm[0] = -(v_i*70.02556)/1;
	recived_cmd.tag_rpm[1] = -(v_i*70.02556)/1;
	recived_cmd.tag_rpm[2] = (v_o*70.02556)/1;
	recived_cmd.tag_rpm[3] = (v_o*70.02556)/1;
}

//ǰ����ת�򰢿���ģʽ
// https://mp.weixin.qq.com/s?__biz=MzI3MTIyMjQwNQ==&mid=2247484711&idx=1&sn=b20922a30f7547f30400a43d05a57caa&chksm=eac456cdddb3dfdb53f08c894c5d772c4065abe5b19cd500f04495964dd6db8aea2d82a95051&scene=178&cur_album_id=2236876593568858115#rd

void AKM2_Wheel_Speed_Model(float speed_x,float speed_y,float speed_w)
{
	float tem = 0;
	float rx = 0.4, ry = 0.4;
	float theta_1 = 0,theta_2 = 0,theta_3 = 0,theta_4 = 0;  
	float v1 = 0,v2 = 0,v3 = 0,v4 = 0; 
	 
	float v1x = speed_x-speed_w*ry;
	float v1y = speed_y+speed_w*rx;
	v1 = sqrt(v1x*v1x + v1y*v1y);
	
	float v2x = speed_x-speed_w*ry;
	float v2y = speed_y-speed_w*rx;
	v2 = sqrt(v2x*v2x + v2y*v2y);

	float v3x = speed_x+speed_w*ry;
	float v3y = speed_y-speed_w*rx;
	v3 = sqrt(v3x*v3x + v3y*v3y);

	float v4x = speed_x+speed_w*ry;
	float v4y = speed_y+speed_w*rx;
	v4 = sqrt(v4x*v4x + v4y*v4y);
	
	recived_cmd.chassis_mode = 0x02;
//	printf("speed_x: %f \tspeed_w %f \n ",speed_x,speed_w);
//	printf("v1-4: %f \t %f\t%f\t%f\t \n ",v1,v2,v3,v4);
	
	if(fabs(speed_x) > 1e-2)
	{
		theta_1 = acos((v1x)/v1); 
		theta_2 = acos((v2x)/v2); 
		theta_3 = acos((v3x)/v3); 
		theta_4 = acos((v4x)/v4); 
		
		
		if(theta_1>0.5*PI ) theta_1 = PI-theta_1;
		if(theta_2>0.5*PI ) theta_2 = PI-theta_2;
		if(theta_3>0.5*PI ) theta_3 = PI-theta_3;
		if(theta_4>0.5*PI ) theta_4 = PI-theta_4;
		
		//printf("theta 1-4: %f \t%f \t%f \t%f  \n",theta_1,theta_2,theta_3,theta_4);
		
		if(speed_x>=0 && speed_w>=0) //��ǰ
		{
			theta_2 = - theta_2;
			theta_3 = - theta_3;
		}
		else if(speed_x>=0 && speed_w<0)  // ��ǰ
		{
			 theta_1=-theta_1;
			 theta_4=-theta_4;
		}
		else if(speed_x<0 && speed_w<=0)//���
		{
			theta_1 = -theta_1;
			theta_4 = -theta_4;
		
			tem = v1; v1 = -v4; v4 = -tem;
			tem = v2; v2 = -v3; v3 = -tem;		
		}
		else if(speed_x<0 && speed_w>0)//�Һ�
		{
			theta_2 =  -theta_2;
			theta_3 = -theta_3; ;
			tem = v1; v1 = -v4; v4 = -tem;
			tem = v2; v2 = -v3; v3 = -tem;		
		}
	}
	else
	{
		if(fabs(speed_x) < 1e-2 && fabs(speed_w) > 1e-2 )
		{
			theta_1 = acos((v1x)/v1); 
			theta_2 = acos((v2x)/v2); 
			theta_3 = acos((v3x)/v3); 
			theta_4 = acos((v4x)/v4); 


			if(theta_1>0.5*PI ) theta_1 = PI-theta_1;
			if(theta_2>0.5*PI ) theta_2 = PI-theta_2;
			if(theta_3>0.5*PI ) theta_3 = PI-theta_3;
			if(theta_4>0.5*PI ) theta_4 = PI-theta_4;
			
			
			if(speed_w >0)
			{
				theta_2 =  -theta_2;
				theta_3 = -theta_3; ;
			}
			else
			{
				theta_1 =  -theta_1;
				theta_4 = -theta_4; ;
			}
		}
		else //��������
		{
			theta_1 = 0;
			theta_2 = 0;
			theta_3 = 0;
			theta_4 = 0;
			
			v1 = 0;
			v2 = 0;
			v3 = 0;
			v4 = 0;
		}
	}
	 
	
//	printf("theta 1-4: %f \t%f \t%f \t%f  \n",theta_1,theta_2,theta_3,theta_4);
//	printf("after th1-4: %f \t%f \t%f \t%f  \n\n\n\n",theta_1,theta_2,theta_3,theta_4);
	theta_1 = theta_1*57.3;
	theta_2 = theta_2*57.3;
	theta_3 = theta_3*57.3;
	theta_4 = theta_4*57.3;	
	
	if(theta_1>60) 	theta_1=60; //����ת�Ƿ�Χ
	else  if(theta_1<-60) theta_1=-60;
	else;

	if(theta_2>60) 	theta_2=60;
	else  if(theta_2<-60) theta_2=-60;
	else;
	
 	if(theta_3>60) 	theta_3=60; 
	else  if(theta_3<-60) theta_3=-60;
	else;

	if(theta_4>60) 	theta_4=60;
	else  if(theta_4<-60) theta_4=-60;
	else;
	
	
	recived_cmd.tag_angl[0] = (theta_1*100)/1;
	recived_cmd.tag_angl[1] = (theta_2*100)/1;
	recived_cmd.tag_angl[2] = (theta_3*100)/1;
	recived_cmd.tag_angl[3] = (theta_4*100)/1;
	
	recived_cmd.tag_rpm[0] = -(v1*70.02556)/1;
	recived_cmd.tag_rpm[1] = -(v2*70.02556)/1;
	recived_cmd.tag_rpm[2] = (v3*70.02556)/1;
	recived_cmd.tag_rpm[3] = (v4*70.02556)/1;
	
//	printf("ANG 1-4: %d \t %d\t%d\t%d  \n  ",recived_cmd.tag_angl[0],recived_cmd.tag_angl[1],recived_cmd.tag_angl[2],recived_cmd.tag_angl[3]);
//	printf("RPM 1-4: %d \t %d\t%d\t%d  \n\n\n\n ",recived_cmd.tag_rpm[0],recived_cmd.tag_rpm[1],recived_cmd.tag_rpm[2],recived_cmd.tag_rpm[3]);
	 
}
/****************************************************************
@param speed_x x�����ٶ�  ��ΧK*[-1 1]
****************************************************************/
void WSWD_Wheel_Speed_Model(float speed_x,float speed_y)
{
	recived_cmd.chassis_mode = 0x03;
	
	int8_t direction = 1;
	double v = sqrt(speed_x*speed_x+speed_y*speed_y);
	float theta =0;
	if(v > 1e-3)
		theta = acosf(speed_x/v)*57.3;
 
	if(theta>90) //������[-pi pi]
		theta=180-theta;
	
	if(speed_y<0) //λ��3/4����
	{
		theta = -theta;
	}
	if(speed_x<0)  
	{
		v = -v;
	}
	if(v<0)
		direction=-direction;
	
	recived_cmd.tag_angl[0] = (direction*theta*100)/1;
	recived_cmd.tag_angl[1] = (direction*theta*100)/1;
	recived_cmd.tag_angl[2] = (direction*theta*100)/1;
	recived_cmd.tag_angl[3] = (direction*theta*100)/1;

	recived_cmd.tag_rpm[0] = -(v*70.02556)/1;// ���ٶȵ�RPMת�� 70.02556=60/(3.1515926*0.258);
	recived_cmd.tag_rpm[1] = -(v*70.02556)/1;
	recived_cmd.tag_rpm[2] = (v*70.02556)/1;
	recived_cmd.tag_rpm[3] = (v*70.02556)/1;
}
void Calculate_DiffX4_Odom(void)
{
	float vr ,vl;
	float w_tem = 0;
	static float last_yaw =0;
	vr = (moto_chassis[0].speed_rpm + moto_chassis[1].speed_rpm)/2000.0f/112.2336;
	vl = (moto_chassis[3].speed_rpm + moto_chassis[2].speed_rpm)/2000.0f/112.2336;

	odom_vx = (vr+vl)/2.0f;
	odom_vy = 0;
	odom_w = (vr-vl)/0.4f;
	if(IMU_Data.update_flag)
	{
		w_tem = (IMU_Data.yaw-last_yaw)/0.05;
		last_yaw = IMU_Data.yaw;
	}
	// �ж�ʲô�����ʹ��IMU�Ľ��ٶ�
	if(fabs(odom_w) > 0.25)
	{
		if(fabs(w_tem)>0.05)
			odom_w = w_tem;
	}
	else
		odom_w = 0;
	
	
}
// ���Ƴ����߳� 10ms����һ��  for mick-v3
void ChasissDiffX4_Control_Routing(uint32_t cnt)
{ 
	switch(cnt)
	{	
		case 1: MOTOR_APSL2DB_Set_RPM(1, recived_cmd.tag_rpm[0]);break;	
		case 2: MOTOR_APSL2DB_Set_RPM(4, recived_cmd.tag_rpm[3]);break;
		case 3: MOTOR_APSL2DB_Set_RPM(3, recived_cmd.tag_rpm[2]);break;
		case 4: MOTOR_APSL2DB_Set_RPM(2, recived_cmd.tag_rpm[1]);break;
		default: ;
	}	
	Calculate_DiffX4_Odom();//���10ms ����һ����̼�
}

void Chasiss4WS4WD_Control_Routing(uint32_t cnt)
{
	 if(recived_cmd.chassis_mode == 0x03) //ȫ��ģʽ��
	 {
		 if(cnt<5)
			cnt=0;
		 else
		 {
			if(-5<(recived_cmd.tag_angl[1]/100 + (moto_rmd_chassis[1].angle))  
				|| (recived_cmd.tag_angl[1]/100 + (moto_rmd_chassis[1].angle)) < 5
				||(recived_cmd.tag_angl[2]/100 + (moto_rmd_chassis[2].angle)) > -5
				|| (recived_cmd.tag_angl[2]/100 + (moto_rmd_chassis[2].angle))< 5
				|| (recived_cmd.tag_angl[3]/100 + (moto_rmd_chassis[3].angle)) > -5
			    || (recived_cmd.tag_angl[3]/100 + (moto_rmd_chassis[3].angle)) <5) // ���С��2��
			{
				;
			}
			else
				cnt=0;
			 
		 }
	 }
	 else if(recived_cmd.chassis_mode == 0x00) // ��ͨ���ٵ���
	 {
		 if(cnt<5)
			cnt=5;
	 }
 	 else if(recived_cmd.chassis_mode == 0x01 && cnt>=5) //����ģʽ��----ԭ��ת��
	 {
		 // ԭ��ת��ģʽ�µȴ�ת�ǵ�����ָ���Ƕ��Ժ��ٿ����ٶ�
//		 if((recived_cmd.tag_angl[0]/100 + moto_rmd_chassis[0].angle) > 2 || (recived_cmd.tag_angl[0]/100 + moto_rmd_chassis[0].angle) < -2)// ������2��
//			cnt=1;
		 if((recived_cmd.tag_angl[1]/100 + moto_rmd_chassis[1].angle) > 2 || (recived_cmd.tag_angl[1]/100 + moto_rmd_chassis[1].angle) < -2)// ������2��
			cnt=2;
		 else if((recived_cmd.tag_angl[2]/100 + moto_rmd_chassis[2].angle) > 2 || (recived_cmd.tag_angl[2]/100 + moto_rmd_chassis[2].angle)<-2) // ������2��
			cnt=3;
		 else if((recived_cmd.tag_angl[3]/100 + (moto_rmd_chassis[3].angle) > 2) || (recived_cmd.tag_angl[3]/100 + (moto_rmd_chassis[3].angle) <-2)) // ������2��
			cnt=4;
	 } 
	 
	 switch(cnt)
		{
			case 0: MOTOR_RMD_Set_Angle_ALLMotor(recived_cmd.tag_angl[0]);break;
			
			case 1: MOTOR_RMD_Set_Angle(1,180,recived_cmd.tag_angl[0]); break;
			case 2: MOTOR_RMD_Set_Angle(2,180,recived_cmd.tag_angl[1]);break;
			case 3: MOTOR_RMD_Set_Angle(3,180,recived_cmd.tag_angl[2]);break;
			case 4: MOTOR_RMD_Set_Angle(4,180,recived_cmd.tag_angl[3]);break;
			
			case 5: MOTOR_APSL2DB_Set_RPM(1, recived_cmd.tag_rpm[0]);break;
			case 6: MOTOR_APSL2DB_Set_RPM(4, recived_cmd.tag_rpm[3]);break;
			case 7: MOTOR_APSL2DB_Set_RPM(3, recived_cmd.tag_rpm[2]);break;
			case 8: MOTOR_APSL2DB_Set_RPM(2, recived_cmd.tag_rpm[1]);break;
			default: ;
		}
}

// ���Ƴ����߳� 50ms����һ��
void Chasiss_Control_Routing(void)
{
	//uint8_t i=0;
	//��ͣ	
	
	
	
	if( Read_Isolated_Input(4) ==1)
	 {
		recived_cmd.rpm_available =0x00;
		recived_cmd.speed_available =0x00;
	 
		recived_cmd.tag_angl[0] = 0;
		recived_cmd.tag_angl[1] = 0;
		recived_cmd.tag_angl[2] = 0;
		recived_cmd.tag_angl[3] = 0;

		recived_cmd.tag_rpm[0] = 0;//  
		recived_cmd.tag_rpm[1] = 0;
		recived_cmd.tag_rpm[2] = 0;
		recived_cmd.tag_rpm[3] = 0;
	 }
	 
	 
	if((Timer2_Counter1>20) || (Timer2_Counter2>1*20 && rc.sw1 ==1))  //��λ��ͨѶ�Ͽ�1s      50msһ���ж�
	{
		recived_cmd.rpm_available =0x00;
		recived_cmd.speed_available =0x00;
		
		recived_cmd.tag_angl[0] = 0;
		recived_cmd.tag_angl[1] = 0;
		recived_cmd.tag_angl[2] = 0;
		recived_cmd.tag_angl[3] = 0;

		recived_cmd.tag_rpm[0] = 0;//  
		recived_cmd.tag_rpm[1] = 0;
		recived_cmd.tag_rpm[2] = 0;
		recived_cmd.tag_rpm[3] = 0;
			
	}
}
//����λ����������д�뵽������
char Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength)
{
	uint16_t i =0;
	uint16_t rpm_offset =10000; //ת��ƫ��10000
	uint16_t speed_offset =10; //�ٶ�ƫ��10m/s
	uint16_t theta_offset =360; //�Ƕ�ƫ�� 360��
	for(i=0;i<datalength;i++)
	{
		UART1_ReBuff[i] = *DataBuff;
		DataBuff++;
	}

	UART1_ReCont = datalength;
	UART1_Reflag =0x01;

	recived_cmd.cmd=UART1_ReBuff[3];
 
    if(recived_cmd.cmd ==0xF2 && rc.sw1 ==1)
	{
		recived_cmd.tag_speed_x = (UART1_ReBuff[4]*256+UART1_ReBuff[5])/100.0-speed_offset; //��λm/s
		recived_cmd.tag_speed_y = (UART1_ReBuff[6]*256+UART1_ReBuff[7])/100.0-speed_offset; //��λm/s
		recived_cmd.tag_speed_w = (UART1_ReBuff[8]*256+UART1_ReBuff[9])/100.0-speed_offset; // rad/s
		recived_cmd.chassis_mode = 0x03; //����ģʽ
		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x01;
		 
		//printf("tag_speed_x: %f  tag_speed_y:%f   tag_speed_w%f",recived_cmd.tag_speed_x,recived_cmd.tag_speed_y,recived_cmd.tag_speed_w);
		//ȫ��ģʽ
		if(recived_cmd.tag_speed_w!=0)
		{
			DiffX4_Wheel_Speed_Model(recived_cmd.tag_speed_w);
		}
		else
		{
			WSWD_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_y);
		}		
	}
	else if(recived_cmd.cmd ==0xF3 && rc.sw1 ==1) //mick-v3 ����
	{
		recived_cmd.tag_speed_x = (UART1_ReBuff[4]*256+UART1_ReBuff[5])/100.0-speed_offset;
		recived_cmd.tag_speed_y = 0;
		recived_cmd.tag_speed_w = (UART1_ReBuff[8]*256+UART1_ReBuff[9])/100.0-speed_offset;
		
		recived_cmd.chassis_mode =0x00; //����ģʽ
		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x01;
		 
		DiffX4_Wheel_Speed_Model_APSL(recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);
	}
	else if(recived_cmd.cmd ==0xF4 && rc.sw1 ==1) //������ģʽ
	{
		recived_cmd.tag_speed_x = (UART1_ReBuff[4]*256+UART1_ReBuff[5])/100.0-speed_offset;
		recived_cmd.tag_speed_y = 0;
		recived_cmd.tag_speed_w = (UART1_ReBuff[8]*256+UART1_ReBuff[9])/100.0-speed_offset;
		recived_cmd.chassis_mode =0x02; //����ģʽ
		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x01;
		//printf("tag_speed_x: %f  tag_theta%f",recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);
		//AKM_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);
		AKM2_Wheel_Speed_Model(recived_cmd.tag_speed_x,0,recived_cmd.tag_speed_w);
	}
	else if(recived_cmd.cmd ==0xFA && rc.sw1 ==1)
	{
		recived_cmd.tag_angl[0] = (UART1_ReBuff[4]*256+UART1_ReBuff[5])-theta_offset*100; //��λ�����͵�ʱ��Ŵ���100��������CAN�����·���ʱ��Ҳ��Ҫ�Ŵ�100�� ������ﲻ��С
		recived_cmd.tag_rpm[0] = UART1_ReBuff[6]*256+UART1_ReBuff[7] - rpm_offset;
		
		recived_cmd.tag_angl[1] = (UART1_ReBuff[8]*256+UART1_ReBuff[9])-theta_offset*100;
		recived_cmd.tag_rpm[1] = UART1_ReBuff[10]*256+UART1_ReBuff[11] - rpm_offset;

		recived_cmd.tag_angl[2] = (UART1_ReBuff[12]*256+UART1_ReBuff[13])-theta_offset*100;
		recived_cmd.tag_rpm[2] = UART1_ReBuff[14]*256+UART1_ReBuff[15] - rpm_offset;

		recived_cmd.tag_angl[3] = (UART1_ReBuff[16]*256+UART1_ReBuff[17])-theta_offset*100;
		recived_cmd.tag_rpm[3] = UART1_ReBuff[18]*256+UART1_ReBuff[19] - rpm_offset;

		recived_cmd.tag_angl[4] = (UART1_ReBuff[20]*256+UART1_ReBuff[21])-theta_offset*100;
		recived_cmd.tag_rpm[4] = UART1_ReBuff[22]*256+UART1_ReBuff[23] - rpm_offset;

		recived_cmd.tag_angl[5] = (UART1_ReBuff[24]*256+UART1_ReBuff[25])-theta_offset*100;
		recived_cmd.tag_rpm[5] = UART1_ReBuff[26]*256+UART1_ReBuff[27] - rpm_offset;
		
//		printf("tag_rpm[0]: %d  tag_rpm[1]:%d   tag_rpm[2]%d  tag_rpm[3]: %d  tag_rpm[4]:%d   tag_rpm[5]%d",
//				recived_cmd.tag_rpm[0],recived_cmd.tag_rpm[1],recived_cmd.tag_rpm[2],
//				recived_cmd.tag_rpm[3],recived_cmd.tag_rpm[4],recived_cmd.tag_rpm[5]);

		recived_cmd.rpm_available=0x01;
		recived_cmd.speed_available=0x00;
	}
	else if(recived_cmd.cmd == 0xE1) //��̼�����
	{
		UART_send_string(USART1,"OK\n"); 
//		 DJI_Motor_Clear_Odom();
	}
	else if(recived_cmd.cmd == 0xE2) //���ø������IO
	{
		;
//		Set_Isolated_Output(1,UART1_ReBuff[4]);
//		Set_Isolated_Output(2,UART1_ReBuff[5]);
//		Set_Isolated_Output(3,UART1_ReBuff[6]);
//		Set_Isolated_Output(4,UART1_ReBuff[7]);
	}
	else if(recived_cmd.cmd == 0xC0) //����
	{
		if((UART1_ReBuff[4] == 0x4D) && (UART1_ReBuff[5] == 0x49) && (UART1_ReBuff[6] == 0x43) && (UART1_ReBuff[7] == 0x4B))
		{
			printf("UART1 CMD  oxC0, Now reset MCU!\n");
			__set_FAULTMASK(1); // �ر������ж�
			NVIC_SystemReset();// ��λ
		}			
	}
	else
	{
		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x00;
		return 0;
	}
	recived_cmd.flag=0x01;
	if(recived_cmd.rpm_available || recived_cmd.speed_available)
	{
		Timer2_Counter2=0; //����
	}	
	
	#if DEBUUG_Motor_RECIVED
		UART_send_string(USART1,"\n  Uart1 recived  data length:  ");
		UART_send_data(USART1,UART1_ReCont);
		UART_send_string(USART1,"  Byte");
	#endif	
	
	return 1;
}

/***************************************************************************
* @brief       �ϴ������Ϣ��PC��
* @retval       ������ 05
* @maker    crp
* @date 2023-8-30
****************************************************************************/
void Chassis_Motor_Upload_Message(void)
{
	unsigned char senddata[60];
	unsigned char i=0,j=0;	
	unsigned int sum=0x00;	

	senddata[i++]=0xAE;
	senddata[i++]=0xEA;
	senddata[i++]=0x01;//���ݳ����ں��渳ֵ
	senddata[i++]=0x07; //������


	for(j=0;j<4;j++) //4*12���ֽ�
	{
		senddata[i++] = moto_chassis[j].speed_rpm>>24; //int32_t
		senddata[i++] = moto_chassis[j].speed_rpm>>16;
		senddata[i++] = moto_chassis[j].speed_rpm>>8; //int32_t
		senddata[i++] = moto_chassis[j].speed_rpm;

		senddata[i++] = moto_chassis[j].round_cnt>>24;
		senddata[i++] = moto_chassis[j].round_cnt>>16;	
		senddata[i++] = moto_chassis[j].round_cnt>>8; //int32_t
		senddata[i++] = moto_chassis[j].round_cnt;
		
		senddata[i++] = moto_chassis[j].angle>>24;
		senddata[i++] = moto_chassis[j].angle>>16;	
		senddata[i++] = moto_chassis[j].angle>>8; //int32
		senddata[i++] = moto_chassis[j].angle;
	}

	senddata[2]=i-1; //���ݳ���
	for(j=2;j<i;j++)
		sum+=senddata[j];
	
	senddata[i++]=sum;

	senddata[i++]=0xEF;
	senddata[i++]=0xFE;

	UART_send_buffer(USART1,senddata,i);
}
void Chassis_MotorState_Upload_Message(void)
{
	unsigned char senddata[25];
	unsigned char i=0,j=0;	
	unsigned int sum=0x00;	

	senddata[i++]=0xAE;
	senddata[i++]=0xEA;
	senddata[i++]=0x01;//���ݳ����ں��渳ֵ
	senddata[i++]=0x08; //������


	for(j=0;j<4;j++) //3*4���ֽ�
	{
		senddata[i++] = moto_chassis[j].driver_status;
		
		senddata[i++] = moto_chassis[j].error_code_HSB; //int16
		senddata[i++] = moto_chassis[j].error_code_LSB;
	}

	senddata[2]=i-1; //���ݳ���
	for(j=2;j<i;j++)
		sum+=senddata[j];
	
	senddata[i++]=sum;

	senddata[i++]=0xEF;
	senddata[i++]=0xFE;

	UART_send_buffer(USART1,senddata,i);
}
void Chassis_Odom_Upload_Message(void)
{
	unsigned char senddata[25];
	unsigned char i=0,j=0;	
	unsigned int sum=0x00;	
	int16_t tem =0;
	
	senddata[i++]=0xAE;
	senddata[i++]=0xEA;
	senddata[i++]=0x01;//���ݳ����ں��渳ֵ
	senddata[i++]=0xA7; //������
	printf("odom %.3f \t %.3f \t %.3f \n",odom_vx,odom_vy,odom_w);
	
	tem = (int16_t)((odom_vx*1000)/1); //odom_vx���ֵ65.535
	senddata[i++] = tem>>8;
	senddata[i++] = tem;
	tem = (int16_t)((odom_vy*1000)/1);
	senddata[i++] = tem>>8;
	senddata[i++] = tem;
	tem = (int16_t)((odom_w*1000)/1);
	senddata[i++] = tem>>8;
	senddata[i++] = tem;
	
	senddata[2]=i-1; //���ݳ���
	for(j=2;j<i;j++)
		sum+=senddata[j];
	
	senddata[i++]=sum;

	senddata[i++]=0xEF;
	senddata[i++]=0xFE;

	UART_send_buffer(USART1,senddata,i);
}