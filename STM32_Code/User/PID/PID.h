#ifndef __PID_H
#define	__PID_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f10x.h"




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
