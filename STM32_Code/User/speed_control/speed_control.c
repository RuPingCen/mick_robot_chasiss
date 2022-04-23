#include "stm32f10x.h"
#include "math.h"
#include "speed_control.h"

#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"


MotorProperty StepMoter;
uint32_t cnt_acc;
uint32_t cnt_run;
uint32_t cnt_dec;
uint32_t distance;


void Motor_InitCfg()
{
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
 // GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE,GPIO_PIN_1 | GPIO_PIN_2);//PE1:pluse  PE2:dir
  My_GPIO_Init(GPIOE,GPIO_Pin_0,GPIO_Mode_Out_PP,GPIO_Speed_10MHz);
  My_GPIO_Init(GPIOE,GPIO_Pin_1,GPIO_Mode_Out_PP,GPIO_Speed_10MHz);
	
  Timer_2to7_counter_Generalfuncation(TIM3,2000);
  TIM_ENABLE;
	
}

/*
*@ function:  步进电机初始化函数
*
* step 运行的总步数
* accel 加速的步数
* decel 减速的步数
* speed 匀速运行的速度
*
*/
void Motor_Move(int32_t step,uint32_t accel,uint32_t decel,uint32_t speed)
{
  uint32_t max_s_lim;
  uint32_t accel_lim;//decel_lim;
	uint32_t decel_val,acc_val;
  float ftemp=0.0;
  
  if(step < 0)    //run in back dirction
  {
    StepMoter.dir = Motor_Dir_Back;
    step = -step;
    GO_FRONT;
  }
  else
  {
    StepMoter.dir = Motor_Dir_Front;
    GO_BACK;
  }
  
  if(step==1)  //decel 1 step
  {
    StepMoter.step_delay = 1000; 
    StepMoter.step_count = 1;
    StepMoter.run_state = Motor_State_DECEL;
  }
  else if(step !=0)
  {
    StepMoter.min_delay = (uint32_t)(ALPHA * T1_FREQ * 100 / speed);
    StepMoter.step_delay = (uint32_t)(0.676*T1_FREQ * sqrtf(200 * ALPHA / accel));
    
		//the number of steps needed to accelerate to the desired speed
		max_s_lim = (uint32_t)(((long)speed * speed) / (ALPHA * accel * 200)); //最大加速步数 
		StepMoter.max_s_lim =max_s_lim;
		
		//限制加速和减速的步数
    if((accel+decel)>=step)
    {
      ftemp = (float)decel/(float)(accel + decel);
			//the number of steps before deceleration starts (disregarding desired speed).
      accel_lim = (uint32_t)((float) step* ftemp);
			
			if(max_s_lim > accel_lim ) //最大加速步数
			{
				decel_val = step - accel_lim;
				acc_val = accel_lim;
			}
			else if(max_s_lim <= accel_lim) //最大加速步数
			{
				decel_val = max_s_lim*(float)accel/(float)(decel);
				acc_val = accel_lim;
			}
			else 
			{
				;
			}
    }
		else 
		{
		    acc_val = accel;
  			decel_val = decel;
		}
		
		StepMoter.acc_val = acc_val;
    StepMoter.dec_val = decel_val;
		
    if((StepMoter.acc_val + StepMoter.dec_val)>step)
    {
      StepMoter.run_val = 0;
    }
    else
    {
      StepMoter.run_val = step - StepMoter.acc_val - StepMoter.dec_val;
    }
    StepMoter.run_state = Motor_State_ACCEL;
  }

  TIM_ENABLE;
}


void MotorControl(void)
{
  static uint8_t i = 0;
  static int32_t rest = 0;
  
 // TimerLoadSet(TIMER1_BASE,TIMER_A,StepMoter.step_delay);
		
  if(StepMoter.run_state)
		FLIP_PLUSE;
  
  i++;
  if(i==1)
  {
    i=0;
		TIM_DISABLE;
    switch(StepMoter.run_state)
    {
      case Motor_State_STOP:
        //temp++;
        StepMoter.step_count = 0;
        rest = 0;
        //TIM_DISABLE;
        //if(temp<2)
          //Motor_Move(-3200,3000,6000,6000);
        break;  

        
      case Motor_State_ACCEL:
        cnt_acc++;
        //GPIOD->BRR|=1<<2;
        StepMoter.step_count++;
        StepMoter.step_delay = StepMoter.step_delay - (2 * (long)StepMoter.step_delay + rest) /(4 *StepMoter.step_count + 1);
        rest = (2 * (long)StepMoter.step_delay + rest) % (4 *StepMoter.step_count + 1);
        if(StepMoter.step_count >= StepMoter.acc_val)
        {
          
          if(StepMoter.run_val == 0)
          {
            StepMoter.run_state = Motor_State_DECEL;
            StepMoter.step_count = StepMoter.dec_val;
          }
          else
          {
            StepMoter.run_state = Motor_State_RUN;
            StepMoter.step_delay = StepMoter.min_delay;
            StepMoter.step_count = 0;
          }
          
        }
        break;
        
        
      case Motor_State_RUN:
        cnt_run++;
        //GPIOD->BSRR|=1<<2;
        StepMoter.step_count ++;
        if(StepMoter.step_count >= StepMoter.run_val)
        {
          StepMoter.run_state = Motor_State_DECEL;
          StepMoter.step_count = StepMoter.dec_val;
        }     
        break;
        
      case Motor_State_DECEL:
        cnt_dec++;
        //GPIOD->BRR|=1<<2;
        StepMoter.step_count --;
        StepMoter.step_delay = StepMoter.step_delay + (2 * (long)StepMoter.step_delay + rest) /(4 *StepMoter.step_count + 1);
        rest = (2 * (long)StepMoter.step_delay + rest) % (4 *StepMoter.step_count + 1);
        if(StepMoter.step_count <= 0)
        {
          StepMoter.run_state = Motor_State_STOP;
          StepMoter.step_count = 0;
        }
        break;         
    }
		Timer_2to7_counter_Generalfuncation(TIM3,StepMoter.step_delay);
		TIM_ENABLE;
  }
	

       
}






