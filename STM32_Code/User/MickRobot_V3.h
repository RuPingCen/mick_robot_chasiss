 /* 定义小车底盘 配置变量 -------------------------------------*/
#ifndef __MickRobot_V3_H
#define __MickRobot_V3_H

 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"




#define Chassis_MAX_Speed  2.2  // 底盘最大速度
#define Chassis_Parameter_L  0.4 // 前后轮距
#define Chassis_Parameter_lw  0.4 // 左右侧轮距 (单位 米)

#define PI 3.1415926


// 信息发布

#define Send_IMU_Message  0      // 向上位机发送IMU数据
 

#define Send_RC_Message   0      // 向上位机发送 遥控器数据        默认不发送
#define Debug_RC_Message  0      // 利用printf 函数打印遥控器数据  

#define Send_Motor_Message   0      // 向上位机发送 电机 数据        
#define Send_Odom_Message    1      // 向上位机发送 里程计 数据       

#endif /* __MickRobot_V3_H */
