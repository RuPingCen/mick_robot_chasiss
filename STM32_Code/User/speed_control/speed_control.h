#ifndef __SPEED_CONTROL_H__
#define __SPEED_CONTROL_H__

#include "stm32f10x.h"

#define SPR      800   //多少个脉冲电机转动一圈
#define ALPHA    0.001 //(2*3.14159/SPR)
#define T1_FREQ  1000000



typedef struct
{
  uint8_t dir;     //运行方向
  uint8_t run_state;
  uint32_t step_delay;
  uint32_t min_delay;
  uint32_t acc_val; // 减速度步数
  uint32_t run_val; // 匀速运行步数
  uint32_t dec_val; // 减速运行步数
  uint32_t step_count; 
	uint32_t max_s_lim;
}MotorProperty;


typedef enum
{
  Motor_Dir_Front = 0,
  Motor_Dir_Back,
}Motor_Dir;

typedef enum
{
  Motor_State_STOP = 0,
  Motor_State_ACCEL,
  Motor_State_DECEL,
  Motor_State_RUN,
}Motor_State;


#define GO_FRONT     GPIO_SetBits(GPIOE,GPIO_Pin_0) 
#define GO_BACK      GPIO_ResetBits(GPIOE,GPIO_Pin_0)

//#define SET_PLUSE    GPIO_SetBits(GPIOE,GPIO_Pin_1) 
//#define CLR_PLUSE    GPIO_ResetBits(GPIOE,GPIO_Pin_1)
#define FLIP_PLUSE   GPIO_Flip_level(GPIOE,GPIO_Pin_1)

#define READ_PIN     GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_2)

#define TIM_ENABLE   Timer_2to7_Generalfuncation_start(TIM3)
#define TIM_DISABLE  Timer_2to7_Generalfuncation_stop(TIM3)



void Motor_InitCfg(void);
void Motor_Move(int32_t step,uint32_t accel,uint32_t decel,uint32_t speed);
void MotorControl(void);


extern uint32_t distance;

#endif //__SPEED_CONTROL_H__
