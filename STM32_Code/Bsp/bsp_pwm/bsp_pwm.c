 /*************************************************************************

*  函数名称： PWM_config_Init
*  功能说明： STM32 pwm 模块初始化函数
*  参数说明： u8PWMx (PWM1 PWM2 PWM3)  
*             u16channel ( PWM_Channel_x 0、1、2、3 )
*             u32period  PWm周期参数   PWM周期 = u32period * 1us
*             u16duty ( 1 ~ 1000) PWM占空比参数  输出占空比 = u16duty/1000 * 100%
*
*  引脚说明：        PWM1             PWM2               PWM3
*
*  PWM_Channel_0    PA6               PB6                  PA0                     
*  PWM_Channel_1    PA7               PB7                  PA1                   
*  PWM_Channel_2    PB0               PB8                  PA2                       
*  PWM_Channel_3    PB1               PB9                  PA3                               
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP  PWM模块是使用定时器的输出比较功能产生的PWM
*                  PWM1 模块使用了定时器 (TIM3)
*                  PWM2 模块使用了定时器 (TIM4)
*                  PWM3 模块使用了定时器 (TIM2)
*                  因此使用PWm相应的模块  就不要再次使用定时器模块   避免冲突
*
*  作者 :CRP
*  修改时间: 2020-2-3    
*  备    注: 无
*************************************************************************/

#include "bsp_pwm.h"
 
 
 
 
  