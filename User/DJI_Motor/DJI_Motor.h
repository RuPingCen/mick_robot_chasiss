#ifndef __DJI_MOTOR_H
#define	__DJI_MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"




/*CAN发送或是接收的ID*/
typedef enum
{

	CAN_TxPY12V_ID 	= 0x200,		//云台12V发送ID
	CAN_TxPY24V_ID	= 0x1FF,		//云台12V发送ID
//	CAN_Pitch_ID 	= 0x201,			//云台Pitch
//	CAN_Yaw_ID   	= 0x203,			//云台Yaw
	CAN_YAW_FEEDBACK_ID   = 0x205,		//云台Yaw24v
	CAN_PIT_FEEDBACK_ID  = 0x206,			//云台Yaw24v
	CAN_POKE_FEEDBACK_ID  = 0x207,
	CAN_ZGYRO_RST_ID 			= 0x404,
	CAN_ZGYRO_FEEDBACK_MSG_ID = 0x401,
	CAN_MotorLF_ID 	= 0x041,    //左前
	CAN_MotorRF_ID 	= 0x042,		//右前
	CAN_MotorLB_ID 	= 0x043,    //左后
	CAN_MotorRB_ID 	= 0x044,		//右后

	CAN_EC60_four_ID	= 0x200,	//EC60接收程序
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
/*接收到的云台电机的参数结构体*/
typedef struct{
		uint16_t 	angle;				//abs angle range:[0,8191] 电机转角绝对值
		uint16_t 	last_angle;	  //abs angle range:[0,8191]
	
		int16_t	 	speed_rpm;       //转速
		int16_t  	real_current;    //转速
	
		int16_t  	given_current;   //实际的转矩电流
		uint8_t  	Temp;           //温度

		uint16_t	offset_angle;   //电机启动时候的零偏角度
		int32_t		round_cnt;     //电机转动圈数
		int32_t		total_angle;    //电机转动的总角度
	
		uint8_t		buf_idx;
		uint16_t	angle_buf[FILTER_BUF_LEN];
		uint16_t	fited_angle;
		uint32_t	msg_cnt;
}moto_measure_t;

enum{
    LLAST	= 0,
    LAST 	= 1,
    NOW 	= 2,
	
    POSITION_PID,
    DELTA_PID,
};
typedef struct __pid_t
{
    float p;
    float i;
    float d;
    
    float set[3];				//目标值,包含NOW， LAST， LLAST上上次
    float get[3];				//测量值
    float err[3];				//误差
	    
    float pout;							//p输出
    float iout;							//i输出
    float dout;							//d输出
    
    float pos_out;						//本次位置式输出
    float last_pos_out;				//上次输出
    float delta_u;						//本次增量值
    float delta_out;					//本次增量式输出 = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//输出限幅
    uint32_t IntegralLimit;		//积分限幅
    
//    void (*f_param_init)(struct __pid_t *pid,  //PID参数初始化
//                    uint32_t pid_mode,
//                    uint32_t maxOutput,
//                    uint32_t integralLimit,
//                    float p,
//                    float i,
//                    float d);
//    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid三个参数修改

}pid_t;

typedef struct __command_t //接收上位机控制命令的结构体数据
{
	unsigned char cmd; //命令字
	
	int tag_rpm1;
	int tag_rpm2;
	int tag_rpm3;
	int tag_rpm4;
	unsigned char rpm_available; //指示转速命令是否可用
	
	float tag_speed_x;
	float tag_speed_y;
	float tag_speed_z;
	unsigned char speed_available;
	
	unsigned char flag;  //标志位 0x01表示接收到一帧数据 待处理
}command_t;

void DJI_Motor_Init(void);
void DJI_Motor_Control(void);	
void Mecanum_Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4); 
void Mecanum_Wheel_Speed_Model(int16_t speed_x,int16_t speed_y,int16_t speed_w);
void DJI_Motor_Show_Message(void);
void DJI_Motor_Upload_Message(void);
void DJI_Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength);
void DJI_Motor_Clear_Odom(void);


void CAN_DJI_C620_DataSend( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4);
void CAN_RxCpltCallback(CanRxMsg* RxMessage);
void get_moto_measure(moto_measure_t *ptr,CanRxMsg* RxMessage);
void get_moto_offset(moto_measure_t *ptr,CanRxMsg* RxMessage);
void get_total_angle(moto_measure_t *p);

void PID_struct_init(pid_t* pid,uint32_t mode,uint32_t maxout,
    uint32_t intergral_limit, float 	kp, float 	ki, float 	kd);
float pid_calc(pid_t* pid, float get, float set);
float pid_sp_calc(pid_t* pid, float get, float set, float gyro);
		
void abs_limit(float *a, float ABS_MAX); //范围限制
static void pid_reset(pid_t	*pid, float kp, float ki, float kd); //重置PID参数


//extern pid_t pid_rol;
//extern pid_t pid_pit;
//extern pid_t pid_yaw;
//extern pid_t pid_pit_omg;
//extern pid_t pid_yaw_omg;	
extern pid_t pid_spd[4];
//extern pid_t pid_yaw_alfa;
//extern pid_t pid_chassis_angle;
//extern pid_t pid_poke;
//extern pid_t pid_poke_omg;
//extern pid_t pid_imu_tmp;		//imu_temperature
//extern pid_t pid_cali_bby;	//big buff yaw
//extern pid_t pid_cali_bbp;
//extern pid_t pid_omg;
//extern pid_t pid_pos;

#ifdef __cplusplus
}
#endif

#endif
