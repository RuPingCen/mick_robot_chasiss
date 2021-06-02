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
    
    float set[3];				//Ŀ��ֵ,����NOW�� LAST�� LLAST���ϴ�
    float get[3];				//����ֵ
    float err[3];				//���
	    
    float pout;							//p���
    float iout;							//i���
    float dout;							//d���
    
    float pos_out;						//����λ��ʽ���
    float last_pos_out;				//�ϴ����
    float delta_u;						//��������ֵ
    float delta_out;					//��������ʽ��� = last_delta_out + delta_u
    float last_delta_out;
    
	  float max_err;
	  float deadband;				//err < deadband return
    uint32_t pid_mode;
    uint32_t MaxOutput;				//����޷�
    uint32_t IntegralLimit;		//�����޷�
    
//    void (*f_param_init)(struct __pid_t *pid,  //PID������ʼ��
//                    uint32_t pid_mode,
//                    uint32_t maxOutput,
//                    uint32_t integralLimit,
//                    float p,
//                    float i,
//                    float d);
//    void (*f_pid_reset)(struct __pid_t *pid, float p, float i, float d);		//pid���������޸�

}pid_t;



void PID_struct_init(pid_t* pid,uint32_t mode,uint32_t maxout,
    uint32_t intergral_limit, float 	kp, float 	ki, float 	kd);
float pid_calc(pid_t* pid, float get, float set);
float pid_sp_calc(pid_t* pid, float get, float set, float gyro);
		
void abs_limit(float *a, float ABS_MAX); //��Χ����
static void pid_reset(pid_t	*pid, float kp, float ki, float kd); //����PID����


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
