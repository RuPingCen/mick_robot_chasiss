/*

*车轮大小   0.258m
*车辆左右轮间距   0.74m
* 前后轮间距  0.8m 
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

extern imu_Dat IMU_Data;//IMU数据结构体

#define Chassis_MAX_Speed  2.2  // 底盘最大速度
#define Chassis_Parameter_L  0.4 // 前后轮距
#define Chassis_Parameter_lw  0.4 // 左右侧轮距 
#define PI 3.1415926

volatile uint16_t run_cnt=0;

volatile float odom_vx=0,odom_vy=0,odom_w=0;
volatile uint32_t last_time=0,curr_time=0; //用于计算时间 获取积分时间

//将串口1的数据转存到数组 UART1_ReBuff中
volatile uint8_t UART1_ReBuff[100];  
volatile uint16_t UART1_ReCont;  
volatile unsigned char UART1_Reflag; 
command_t recived_cmd;  // 串口1中从上位机解析到的命令

extern rc_info_t rc;  //遥控器
extern volatile uint32_t Timer2_Counter1,Timer2_Counter2;//遥控器和上位机通讯间隔计时

extern volatile moto_measure_t moto_chassis[4];//行进电机状态
extern volatile moto_measure_t moto_rmd_chassis[4];//转向电机状态

/**
* @funtion	 4轮差速
* @Param	 speed_w 范围是 [-1 1]  单位 rad/s
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
	// 使用的是6.5英寸的轮子 直径为17cm  
	recived_cmd.tag_rpm[0] = -(112.344668*v1)/1; // 线速度到RPM的转换系数是   60/(PI*D) = 112.344668
	recived_cmd.tag_rpm[1] = -(112.344668*v2)/1;  // 底盘中心到轮子转动中心的距离是 0.4
	recived_cmd.tag_rpm[2] = -(112.344668*v3)/1;   // speed_w的范围是正负1 是否还需要乘以一个pi 限制旋转速度是pi/s
	recived_cmd.tag_rpm[3] = -(112.344668*v4)/1;
 
}
 
/**
* @funtion	 底盘原点转向角速度
* @Param	 speed_w 范围是 [-1 1]  单位 rad/s
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

	recived_cmd.tag_rpm[0] = (37.604*speed_w)/1; // 线速度到RPM的转换系数是  70.02556
	recived_cmd.tag_rpm[1] = (37.604*speed_w)/1;  // 底盘中心到轮子转动中心的距离是 0.537
	recived_cmd.tag_rpm[2] = (37.604*speed_w)/1;   // speed_w的范围是正负1 是否还需要乘以一个pi 限制旋转速度是pi/s
	recived_cmd.tag_rpm[3] = (37.604*speed_w)/1;
}
// 后轮阿卡曼模型
//theta 的输入范围是 [-90 90]  单位 m/s
//void AKM_Wheel_Speed_Model(float speed_x,float speed_w)
//{
//	 
//	recived_cmd.chassis_mode = 0x02;
//	int8_t dir_theta =1 ,dir_speed=1; //表示转向和速度的正负号
//	float theta_o = 0,theta_i=0;
//	float v_o = 0, v_i = 0;
//	if((fabs(speed_w) > 1e-3) && fabs(speed_x) > 1e-2)
//	{
//		float R = speed_x/speed_w; // 当V很小W很大的时候，存在R-Chassis_Parameter_lw*0.5 小于0的情况
//		if(speed_w<0)
//				dir_theta =-1;
//		if(speed_x<0)
//				dir_speed =-1;
//		if(speed_x>0) // 对应前左和前右两种情况
//		{
//			theta_i = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //内侧轮胎转角
//			theta_o = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //外侧轮胎转角
//			
//			if(R > 0 && R <= Chassis_Parameter_lw*0.5 ) //如果转向半径在车体内，则内侧轮子不转
//				v_i = 0;
//			else
//				v_i = dir_speed*fabs(speed_w)*fabs(R-Chassis_Parameter_lw*0.5);
//			
//			if(R < 0 && R >= -Chassis_Parameter_lw*0.5 )
//				v_o = 0;
//			else
//				v_o = dir_speed*fabs(speed_w)*fabs(R+Chassis_Parameter_lw*0.5);
//		}              		
//		else if(speed_x<0) //  后左情况 R<0(速度小于0，舵轮逆时针向左转)     //  后右情况 R>0(速度小于0，舵轮顺时针转动)
//		{
//			theta_i = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //内侧轮胎转角
//			theta_o = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //外侧轮胎转角
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
//	if(theta_o>50) 	theta_o=50; //限制转角范围
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
	int8_t dir_theta =1 ,dir_speed=1; //表示转向和速度的正负号
	float theta_o = 0,theta_i=0;
	float v_o = 0, v_i = 0;
	if((fabs(speed_w) > 1e-3) && fabs(speed_x) > 1e-2)
	{
		float R = speed_x/speed_w; // 当V很小W很大的时候，存在R-Chassis_Parameter_lw*0.5 小于0的情况
		if(speed_w<0)
				dir_theta =-1;
		if(speed_x<0)
				dir_speed =-1;
		
	 if( R< 0 && R >= -Chassis_Parameter_lw*0.5)  //限制最小转弯半径
		R =-Chassis_Parameter_lw*0.5 - 0.2;
	 if( R >0  && R <= Chassis_Parameter_lw*0.5)
		R = Chassis_Parameter_lw*0.5 + 0.2;
			
			
		if(speed_x>0) // 对应前左和前右两种情况
		{
			theta_i = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //内侧轮胎转角
			theta_o = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //外侧轮胎转角
			
			if(R > 0 && R <= Chassis_Parameter_lw*0.5 ) //如果转向半径在车体内，则内侧轮子不转
				v_i = 0;
			else
				v_i = dir_speed*fabs(speed_w)*fabs(R-Chassis_Parameter_lw*0.5);
			
			if(R < 0 && R >= -Chassis_Parameter_lw*0.5 )
				v_o = 0;
			else
				v_o = dir_speed*fabs(speed_w)*fabs(R+Chassis_Parameter_lw*0.5);
		}              		
		else if(speed_x<0) //  后左情况 R<0(速度小于0，舵轮逆时针向左转)     //  后右情况 R>0(速度小于0，舵轮顺时针转动)
		{
			theta_i = dir_theta*Chassis_Parameter_L/fabs(R+Chassis_Parameter_lw*0.5); //内侧轮胎转角
			theta_o = dir_theta*Chassis_Parameter_L/fabs(R-Chassis_Parameter_lw*0.5); //外侧轮胎转角
			
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
	
	
	if(theta_o>50) 	theta_o=50; //限制转角范围
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

//前后轮转向阿卡曼模式
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
		
		if(speed_x>=0 && speed_w>=0) //左前
		{
			theta_2 = - theta_2;
			theta_3 = - theta_3;
		}
		else if(speed_x>=0 && speed_w<0)  // 右前
		{
			 theta_1=-theta_1;
			 theta_4=-theta_4;
		}
		else if(speed_x<0 && speed_w<=0)//左后
		{
			theta_1 = -theta_1;
			theta_4 = -theta_4;
		
			tem = v1; v1 = -v4; v4 = -tem;
			tem = v2; v2 = -v3; v3 = -tem;		
		}
		else if(speed_x<0 && speed_w>0)//右后
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
		else //设置死区
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
	
	if(theta_1>60) 	theta_1=60; //限制转角范围
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
@param speed_x x方向速度  范围K*[-1 1]
****************************************************************/
void WSWD_Wheel_Speed_Model(float speed_x,float speed_y)
{
	recived_cmd.chassis_mode = 0x03;
	
	int8_t direction = 1;
	double v = sqrt(speed_x*speed_x+speed_y*speed_y);
	float theta =0;
	if(v > 1e-3)
		theta = acosf(speed_x/v)*57.3;
 
	if(theta>90) //量化到[-pi pi]
		theta=180-theta;
	
	if(speed_y<0) //位于3/4象限
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

	recived_cmd.tag_rpm[0] = -(v*70.02556)/1;// 线速度到RPM转换 70.02556=60/(3.1515926*0.258);
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
	// 判断什么情况下使用IMU的角速度
	if(fabs(odom_w) > 0.25)
	{
		if(fabs(w_tem)>0.05)
			odom_w = w_tem;
	}
	else
		odom_w = 0;
	
	
}
// 控制程序线程 10ms调用一次  for mick-v3
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
	Calculate_DiffX4_Odom();//间隔10ms 计算一次里程计
}

void Chasiss4WS4WD_Control_Routing(uint32_t cnt)
{
	 if(recived_cmd.chassis_mode == 0x03) //全向模式下
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
			    || (recived_cmd.tag_angl[3]/100 + (moto_rmd_chassis[3].angle)) <5) // 误差小于2度
			{
				;
			}
			else
				cnt=0;
			 
		 }
	 }
	 else if(recived_cmd.chassis_mode == 0x00) // 普通差速底盘
	 {
		 if(cnt<5)
			cnt=5;
	 }
 	 else if(recived_cmd.chassis_mode == 0x01 && cnt>=5) //差速模式下----原地转向
	 {
		 // 原地转向模式下等待转角到达了指定角度以后再控制速度
//		 if((recived_cmd.tag_angl[0]/100 + moto_rmd_chassis[0].angle) > 2 || (recived_cmd.tag_angl[0]/100 + moto_rmd_chassis[0].angle) < -2)// 误差大于2度
//			cnt=1;
		 if((recived_cmd.tag_angl[1]/100 + moto_rmd_chassis[1].angle) > 2 || (recived_cmd.tag_angl[1]/100 + moto_rmd_chassis[1].angle) < -2)// 误差大于2度
			cnt=2;
		 else if((recived_cmd.tag_angl[2]/100 + moto_rmd_chassis[2].angle) > 2 || (recived_cmd.tag_angl[2]/100 + moto_rmd_chassis[2].angle)<-2) // 误差大于2度
			cnt=3;
		 else if((recived_cmd.tag_angl[3]/100 + (moto_rmd_chassis[3].angle) > 2) || (recived_cmd.tag_angl[3]/100 + (moto_rmd_chassis[3].angle) <-2)) // 误差大于2度
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

// 控制程序线程 50ms调用一次
void Chasiss_Control_Routing(void)
{
	//uint8_t i=0;
	//急停	
	
	
	
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
	 
	 
	if((Timer2_Counter1>20) || (Timer2_Counter2>1*20 && rc.sw1 ==1))  //上位机通讯断开1s      50ms一次中断
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
//将上位机发送数据写入到缓存中
char Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength)
{
	uint16_t i =0;
	uint16_t rpm_offset =10000; //转速偏移10000
	uint16_t speed_offset =10; //速度偏移10m/s
	uint16_t theta_offset =360; //角度偏移 360度
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
		recived_cmd.tag_speed_x = (UART1_ReBuff[4]*256+UART1_ReBuff[5])/100.0-speed_offset; //单位m/s
		recived_cmd.tag_speed_y = (UART1_ReBuff[6]*256+UART1_ReBuff[7])/100.0-speed_offset; //单位m/s
		recived_cmd.tag_speed_w = (UART1_ReBuff[8]*256+UART1_ReBuff[9])/100.0-speed_offset; // rad/s
		recived_cmd.chassis_mode = 0x03; //运行模式
		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x01;
		 
		//printf("tag_speed_x: %f  tag_speed_y:%f   tag_speed_w%f",recived_cmd.tag_speed_x,recived_cmd.tag_speed_y,recived_cmd.tag_speed_w);
		//全向模式
		if(recived_cmd.tag_speed_w!=0)
		{
			DiffX4_Wheel_Speed_Model(recived_cmd.tag_speed_w);
		}
		else
		{
			WSWD_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_y);
		}		
	}
	else if(recived_cmd.cmd ==0xF3 && rc.sw1 ==1) //mick-v3 差速
	{
		recived_cmd.tag_speed_x = (UART1_ReBuff[4]*256+UART1_ReBuff[5])/100.0-speed_offset;
		recived_cmd.tag_speed_y = 0;
		recived_cmd.tag_speed_w = (UART1_ReBuff[8]*256+UART1_ReBuff[9])/100.0-speed_offset;
		
		recived_cmd.chassis_mode =0x00; //运行模式
		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x01;
		 
		DiffX4_Wheel_Speed_Model_APSL(recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);
	}
	else if(recived_cmd.cmd ==0xF4 && rc.sw1 ==1) //阿卡曼模式
	{
		recived_cmd.tag_speed_x = (UART1_ReBuff[4]*256+UART1_ReBuff[5])/100.0-speed_offset;
		recived_cmd.tag_speed_y = 0;
		recived_cmd.tag_speed_w = (UART1_ReBuff[8]*256+UART1_ReBuff[9])/100.0-speed_offset;
		recived_cmd.chassis_mode =0x02; //运行模式
		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x01;
		//printf("tag_speed_x: %f  tag_theta%f",recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);
		//AKM_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_w);
		AKM2_Wheel_Speed_Model(recived_cmd.tag_speed_x,0,recived_cmd.tag_speed_w);
	}
	else if(recived_cmd.cmd ==0xFA && rc.sw1 ==1)
	{
		recived_cmd.tag_angl[0] = (UART1_ReBuff[4]*256+UART1_ReBuff[5])-theta_offset*100; //上位机发送的时候放大了100倍，但是CAN总线下发的时候也需要放大100倍 因此这里不缩小
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
	else if(recived_cmd.cmd == 0xE1) //里程计清零
	{
		UART_send_string(USART1,"OK\n"); 
//		 DJI_Motor_Clear_Odom();
	}
	else if(recived_cmd.cmd == 0xE2) //设置隔离输出IO
	{
		;
//		Set_Isolated_Output(1,UART1_ReBuff[4]);
//		Set_Isolated_Output(2,UART1_ReBuff[5]);
//		Set_Isolated_Output(3,UART1_ReBuff[6]);
//		Set_Isolated_Output(4,UART1_ReBuff[7]);
	}
	else if(recived_cmd.cmd == 0xC0) //重启
	{
		if((UART1_ReBuff[4] == 0x4D) && (UART1_ReBuff[5] == 0x49) && (UART1_ReBuff[6] == 0x43) && (UART1_ReBuff[7] == 0x4B))
		{
			printf("UART1 CMD  oxC0, Now reset MCU!\n");
			__set_FAULTMASK(1); // 关闭所有中断
			NVIC_SystemReset();// 复位
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
		Timer2_Counter2=0; //清零
	}	
	
	#if DEBUUG_Motor_RECIVED
		UART_send_string(USART1,"\n  Uart1 recived  data length:  ");
		UART_send_data(USART1,UART1_ReCont);
		UART_send_string(USART1,"  Byte");
	#endif	
	
	return 1;
}

/***************************************************************************
* @brief       上传电机信息到PC上
* @retval       命令字 05
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
	senddata[i++]=0x01;//数据长度在后面赋值
	senddata[i++]=0x07; //命令字


	for(j=0;j<4;j++) //4*12个字节
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

	senddata[2]=i-1; //数据长度
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
	senddata[i++]=0x01;//数据长度在后面赋值
	senddata[i++]=0x08; //命令字


	for(j=0;j<4;j++) //3*4个字节
	{
		senddata[i++] = moto_chassis[j].driver_status;
		
		senddata[i++] = moto_chassis[j].error_code_HSB; //int16
		senddata[i++] = moto_chassis[j].error_code_LSB;
	}

	senddata[2]=i-1; //数据长度
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
	senddata[i++]=0x01;//数据长度在后面赋值
	senddata[i++]=0xA7; //命令字
	printf("odom %.3f \t %.3f \t %.3f \n",odom_vx,odom_vy,odom_w);
	
	tem = (int16_t)((odom_vx*1000)/1); //odom_vx最大值65.535
	senddata[i++] = tem>>8;
	senddata[i++] = tem;
	tem = (int16_t)((odom_vy*1000)/1);
	senddata[i++] = tem>>8;
	senddata[i++] = tem;
	tem = (int16_t)((odom_w*1000)/1);
	senddata[i++] = tem>>8;
	senddata[i++] = tem;
	
	senddata[2]=i-1; //数据长度
	for(j=2;j<i;j++)
		sum+=senddata[j];
	
	senddata[i++]=sum;

	senddata[i++]=0xEF;
	senddata[i++]=0xFE;

	UART_send_buffer(USART1,senddata,i);
}