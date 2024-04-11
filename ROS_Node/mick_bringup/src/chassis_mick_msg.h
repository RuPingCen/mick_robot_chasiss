#ifndef CHASSIS_MICK_MSG_H
#define CHASSIS_MICK_MSG_H

#include <stdint.h>

typedef struct{
		int32_t 	angle;				//abs angle range:[0,8191] 电机转角绝对值
		int32_t 	last_angle;	  //abs angle range:[0,8191]
		int32_t	 	speed_rpm;       //转速
		int16_t  	given_current;   //实际的转矩电流
		uint8_t  	Temp;           //温度
		uint32_t	offset_angle;   //电机启动时候的零偏角度
		int32_t		round_cnt;     //电机转动圈数
		int32_t		total_angle;    //电机转动的总角度
		uint32_t counter;
}moto_measure_t;

typedef struct{
 		uint32_t counter;
		int16_t ax,ay,az;		 
 		int16_t gx,gy,gz;	
 		int16_t mx,my,mz;	
		float pitch,roll,yaw;
		float pitch_rad,roll_rad,yaw_rad;
		float qw,qx,qy,qz;
}imu_measure_t;

typedef struct
{
	uint16_t ch1, ch2, ch3, ch4;
	uint16_t ch5, ch6, ch7, ch8, ch9, ch10, ch11, ch12, ch13, ch14, ch15, ch16, ch17, ch18;
	uint8_t sw1, sw2,sw3, sw4;
	uint16_t ch1_offset, ch2_offset, ch3_offset, ch4_offset;


	 uint8_t type;	 // 1 DJI-DBUS   2 SBUS 遥控器类型

	 uint8_t status; 
	volatile uint8_t update; //表示接收到了一个新数据
	volatile uint8_t available; //是否数据是否有效可用


} rc_info_t;


struct chassis{
		volatile char available;
		float vx,vy,wz;
		float px,py,pz;

		uint8_t chassis_type; // 0:X4  1:M4  2: Ackermann  3:4WS4WD
		uint8_t motor_type; // 0 M3508    1: 安普斯电机
};
#endif 
