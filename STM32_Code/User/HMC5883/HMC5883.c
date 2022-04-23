#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
 
#include <stdio.h>

 
#include "IO_IIC.h"  
#include "HMC5883.h"
 
 
 
 
 
#define  HMC5883_FIFO_Length   10 //滑动窗口滤波长度
 
#define  HMC5883_X_Gain   1.0f // 根据实际情况修改每个轴的增益
#define  HMC5883_Y_Gain   1.0f // 
#define  HMC5883_Z_Gain   1.0f // 


 
int16_t HMC5883_FIFO[3][HMC5883_FIFO_Length+1]={'0'};

 
void HMC5883_Init(void)
{
	  char hmc5883id[3];
	
	  IO_IIC_write_Command(HMC5883_SlaveAddress,HMC5883_MOD,0x00);// 设置为模式0
	  IO_IIC_write_Command(HMC5883_SlaveAddress,HMC5883_CAR,HMC5883_Averfil_Time8 | HMC5883_Data_OR75hz);// 
		IO_IIC_write_Command(HMC5883_SlaveAddress,HMC5883_CBR,HMC5883_Gain_4Ga7);// 
		IO_IIC_write_Command(HMC5883_SlaveAddress,HMC5883_MOD,0x00);// 设置为模式0
		while( (hmc5883id[0]!= 0x48) || (hmc5883id[1]!= 0x34) || (hmc5883id[2]!= 0x33))
		{     HMC5883_GetID(hmc5883id) ;//检测HMC5883 是否已经连接
          //UART_send_string(USART1,"hmc5883等待连接！！！！\n");
    }
			
		
		
     
}
/**************************实现函数********************************************
*函数原型:	  void HMC58X3_getID(char id[3])
*功　　能:	   读取芯片的ID
输入参数：     	ID存放的数组
输出参数：  无
*******************************************************************************/
void HMC5883_GetID(char id[3]) 
{
      id[0]=IO_IIC_read_Data(HMC5883_SlaveAddress,HMC58X3_R_IDA);  
      id[1]=IO_IIC_read_Data(HMC5883_SlaveAddress,HMC58X3_R_IDB);
      id[2]=IO_IIC_read_Data(HMC5883_SlaveAddress,HMC58X3_R_IDC);
}  
/**************************实现函数********************************************
*函数原型:	   void  HMC58X3_newValues(int16_t x,int16_t y,int16_t z)
*功　　能:	   更新一组数据到FIFO数组
输入参数：  磁力计三个轴对应的ADC值
输出参数：  无
*******************************************************************************/
void  HMC5883_newValues(int16_t x,int16_t y,int16_t z)
{
	unsigned char i ;
	int32_t sum=0;

	for(i=1;i<HMC5883_FIFO_Length;i++)
	{
		HMC5883_FIFO[0][i-1]=HMC5883_FIFO[0][i];
		HMC5883_FIFO[1][i-1]=HMC5883_FIFO[1][i];
		HMC5883_FIFO[2][i-1]=HMC5883_FIFO[2][i];
	}

	HMC5883_FIFO[0][HMC5883_FIFO_Length-1]=x;
	HMC5883_FIFO[1][HMC5883_FIFO_Length-1]=y;
	HMC5883_FIFO[2][HMC5883_FIFO_Length-1]=z;

	sum=0;
	for(i=0;i<HMC5883_FIFO_Length;i++)
	{	//取数组内的值进行求和再取平均
   		sum+=HMC5883_FIFO[0][i];
	}
	HMC5883_FIFO[0][HMC5883_FIFO_Length]=sum/HMC5883_FIFO_Length;	//将平均值更新

	sum=0;
	for(i=0;i<HMC5883_FIFO_Length;i++)
	{
   		sum+=HMC5883_FIFO[1][i];
	}
	HMC5883_FIFO[1][HMC5883_FIFO_Length]=sum/HMC5883_FIFO_Length;

	sum=0;
	for(i=0;i<HMC5883_FIFO_Length;i++)
	{
   		sum+=HMC5883_FIFO[2][i];
	}
	HMC5883_FIFO[2][HMC5883_FIFO_Length]=sum/HMC5883_FIFO_Length;
}  
     
//******************************************************
//
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//
//******************************************************
void Multiple_read_HMC5883(int16_t magADC_data[3])
{  
   unsigned char i;
   unsigned char BUF[6];
  // double angle;
   int16_t HMC5883dat_x,HMC5883dat_y,HMC5883dat_z;
   for (i=0; i<6; i++)                      //连续读取6个地址数据，存储中BUF
    {
        BUF[i] =IO_IIC_read_Data(HMC5883_SlaveAddress,i+3);//读寄存器            //BUF[0]存储0x32地址中的数据
        
    }           
   
   HMC5883dat_x=(BUF[0]<<8)|BUF[1];
   HMC5883dat_y=(BUF[4]<<8)|BUF[5]; 
   HMC5883dat_z=(BUF[2]<<8)|BUF[3];
		
   HMC5883_newValues(HMC5883dat_x,HMC5883dat_y,HMC5883dat_z);

		magADC_data[0] = HMC5883_FIFO[0][HMC5883_FIFO_Length]*HMC5883_X_Gain;
		magADC_data[1] = HMC5883_FIFO[1][HMC5883_FIFO_Length]*HMC5883_Y_Gain;
		magADC_data[2]  = HMC5883_FIFO[2][HMC5883_FIFO_Length]*HMC5883_Z_Gain;
 
  //   angle= atan2((double)HMC5883dat_y,(double)HMC5883dat_x) * (180 / 3.14159265) + 180; // angle in degrees
 
   
}

