 /* 定义公共的数据结构和配置变量 -------------------------------------*/
#ifndef __COMMON_H
#define __COMMON_H

 

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

 //示例
union INT32
{
  uint8_t uint8_value[4];
  int32_t int32_value;
};//定义共用体类型的同时顶定义变量
//示例
union INT16
{
  uint8_t uint8_value[2];
  int16_t int16_value;
};//定义共用体类型的同时顶定义变量


 

#endif /* __COMMON_H */
