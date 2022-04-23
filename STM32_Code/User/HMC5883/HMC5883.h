#ifndef _HMC5883_H_
#define _HMC5883_H_
#include "stm32f10x.h"
/**************************HMC5883内部地址*******************************************/
#define	HMC5883_CAR	        0x00   //配置寄存器A
#define	HMC5883_CBR	        0x01   //配置寄存器B
#define	HMC5883_MOD					0x02   //模式寄存器
#define	COMPASS_XOUT_H			0x03   
#define	COMPASS_XOUT_L			0x04
#define	COMPASS_ZOUT_H			0x05
#define COMPASS_ZOUT_L			0x06
#define	COMPASS_YOUT_H			0x07
#define	COMPASS_YOUT_L			0x08
#define	HMC5883_STR					0x09   //HMC5883状态寄存器
#define HMC58X3_R_IDA 			0x0A
#define HMC58X3_R_IDB 			0x0B
#define HMC58X3_R_IDC 			0x0C


#define	HMC5883_SlaveAddress	0X3C	//IIC写入时的地址字节数据，+1为读取  {0X3C/2}=1E



#define	HMC5883_Averfil_Time1	 0x00	// hmc5883 每次测量输出中选择采样平均数
#define	HMC5883_Averfil_Time2	 0x20	// 
#define	HMC5883_Averfil_Time4	 0x40	// 
#define	HMC5883_Averfil_Time8	 0x60	// 


#define	HMC5883_Data_OR0hz75	 0x00	// hmc5883 数据输出速率
#define	HMC5883_Data_OR1hz5	   0x04	// 
#define	HMC5883_Data_OR3hz	   0x08	// 
#define	HMC5883_Data_OR7hz5	   0x0C	// 
#define	HMC5883_Data_OR15hz	   0x10	// 
#define	HMC5883_Data_OR30hz	   0x14	// 
#define	HMC5883_Data_OR75hz	   0x18	// 



#define	HMC5883_Gain_0Ga88	 0x00	// hmc5883 增益
#define	HMC5883_Gain_1Ga3    0x20	// 
#define	HMC5883_Gain_1Ga9		 0x40
#define	HMC5883_Gain_2Ga5    0x60
#define	HMC5883_Gain_4Ga     0x80
#define	HMC5883_Gain_4Ga7    0xA0
#define	HMC5883_Gain_5Ga6    0xC0
#define	HMC5883_Gain_8Ga1    0xE0
 
//-------------------------函数声明区---------------------------------//
void HMC5883_Init(void);
void HMC5883_GetID(char id[3]);
void HMC5883_newValues(int16_t x,int16_t y,int16_t z);
void Multiple_read_HMC5883(int16_t magADC_data[3]);



#endif


/***************************************HMC5883数据处理**********************************************
******************************************************************************************************
   x=outputData[0] << 8 | outputData[1]; //Combine MSB and LSB of X Data output register
    z=outputData[2] << 8 | outputData[3]; //Combine MSB and LSB of Z Data output register
    y=outputData[4] << 8 | outputData[5]; //Combine MSB and LSB of Y Data output register
    angle= atan2((double)y,(double)x) * (180 / 3.14159265) + 180; // angle in degrees
  ----------------------------------------------------------------------------------------
  atan2(y, x) is the angle in radians between the positive x-axis of a plane and the point
   atan2(y, x)=arctangent(y/x);
  given by the coordinates (x, y) on it. 
  ----------------------------------------------------------------------------------------
 
  This sketch does not utilize the magnetic component Z as tilt compensation can not be done without an Accelerometer
 
   ----------------->y       
  |                               N 
  |                          NW  |  NE
  |                              |  
  |                         W----------E
  |                              |
  |                          SW  |  SE
  |                              S
 \/
  X
    //Print the approximate direction
   下面的方向是指 ‘X’ 轴所指的方向 因为：angle= atan2(y,x);
    Serial.print("You are heading ");  
    if((angle < 22.5) || (angle > 337.5 ))
        Serial.print("South");
    if((angle > 22.5) && (angle < 67.5 ))
        Serial.print("South-West");
    if((angle > 67.5) && (angle < 112.5 ))
        Serial.print("West");
    if((angle > 112.5) && (angle < 157.5 ))
        Serial.print("North-West");
    if((angle > 157.5) && (angle < 202.5 ))
        Serial.print("North");
    if((angle > 202.5) && (angle < 247.5 ))
        Serial.print("NorthEast");
    if((angle > 247.5) && (angle < 292.5 ))
        Serial.print("East");
    if((angle > 292.5) && (angle < 337.5 ))
        Serial.print("SouthEast");
 
    Serial.print(": Angle between X-axis and the South direction ");
    if((0 < angle) && (angle < 180) )
    {
        angle=angle;
    }
    else
    {
        angle=360-angle;
    }
    Serial.print(angle,2);
    Serial.println(" Deg");
    delay(100);
}
******************************************************************************************************/
