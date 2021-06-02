/**
*@function MPU9250的磁力计的频率只有8HZ 这个远远小于我们实际的需求值
*
*可以正常读取出MPU9250的数据，但是配置参数没有仔细去研究
*
*
*@data 2020-09-15
*/
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h" 
#include "stm32f10x_Delay.h"
#include  <math.h>    //Keil library

#include "bsp_uart.h"
#include "IO_IIC.h"
#include "MPU9250.h"
 
  
  
#define  MPU9250_FIFO_Length   5 //滑动窗口滤波长度
 
int16_t MPU9250_FIFO[9][MPU9250_FIFO_Length+1]={'0'};
unsigned char BUF[10];      
 
//#define MPU9250_Hadware_IIC 1

 
void MPU9250_Init(void) 
{
	#ifdef  MPU9250_Hadware_IIC 
		I2C_STM32_Config_Init(I2C1,400000);
		Delay_10us(200);// nTime*10us 延时函数

		I2C_STM32_Write_one_Byte(I2C1, MPU9250_GYRO_ADDRESS,MPU9250_PWR_MGMT_1,0x00); 
		I2C_STM32_Write_one_Byte(I2C1, MPU9250_GYRO_ADDRESS,MPU9250_SMPLRT_DIV,0x07); 

		I2C_STM32_Write_one_Byte(I2C1, MPU9250_GYRO_ADDRESS,MPU9250_CONFIG,0x06);
		I2C_STM32_Write_one_Byte(I2C1, MPU9250_GYRO_ADDRESS,MPU9250_GYRO_CONFIG,0x18);
		I2C_STM32_Write_one_Byte(I2C1, MPU9250_GYRO_ADDRESS,MPU9250_ACCEL_CONFIG,0x01);
		while(I2C_STM32_Read_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,MPU9250_WHO_AM_I) != 0x71);//检测MPU9250 是否已经连接

#else
	 
			
		IO_IIC_GPIO_Init();//IIC引脚初始化   
		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_PWR_MGMT1_REG,0x80); 
		Delay_10us(20000);// nTime*10us 延时函数
		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_PWR_MGMT1_REG,0X01);  	 // Clock Source
		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_PWR_MGMT2_REG,0X00);  // Enable Acc & Gyro
		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_CONFIG,0x07);

		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_GYRO_CFG_REG,0x18); 
		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_ACCEL_CFG_REG,0x00);  //0,±2g;1,±4g;2,±8g;3,±16g

		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_INT_EN_REG,0X00);   //关闭所有中断
		//		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_I2CMST_CTRL_REG,0x40); 	
		//		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_USER_CTRL_REG,0x20); 

		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_FIFO_EN_REG,0x00);	//关闭FIFO
		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_INTBP_CFG_REG,0x82);//INT引脚低电平有效，开启bypass模式，
		 
		//		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_I2CSLV0_ADDR_REG,0x0C); 
		//		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_I2CSLV0_REG,0x26); 
		//		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_I2CSLV0_CTRL_REG,0x81); 
		//		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_I2CMST_DELAY_REG,0x01); 

		IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,MPU9250_SMPLRT_DIV,0x07); 

		IO_IIC_write_Command(MPU9250_MAG_ADDRESS,MAG_CNTL1,0x16);		//设置AK8963为单次测量模式

		while(IO_IIC_read_Data(MPU9250_GYRO_ADDRESS,MPU9250_WHO_AM_I)!= 0x71)
		{
			UART_send_string(USART2,".");//检测MPU9250 是否已经连接
			Delay_10us(200);// nTime*10us 延时函数
		}
 
#endif
 

}
 
 
#ifdef  MPU9250_Hadware_IIC 

//**********************************************
void MPU9250_READ_ACCEL(void)
{ 
 
   BUF[0]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_XOUT_L); 
   BUF[1]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_XOUT_H);
   T_X=	(BUF[1]<<8)|BUF[0];						  

   BUF[2]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_YOUT_L);
   BUF[3]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_YOUT_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
  
   BUF[4]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_ZOUT_L);
   BUF[5]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_ZOUT_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
  					   
}

void MPU9250_READ_GYRO(void)
{ 

   BUF[0]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,MPU9250_GYRO_XOUT_L); 
   BUF[1]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,MPU9250_GYRO_XOUT_H);
   T_X=	(BUF[1]<<8)|BUF[0];
						  

   BUF[2]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,MPU9250_GYRO_YOUT_L);
   BUF[3]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,MPU9250_GYRO_YOUT_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
					
	
   BUF[4]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,MPU9250_GYRO_ZOUT_L);
   BUF[5]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,MPU9250_GYRO_ZOUT_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
					  
 
}


void MPU9250_READ_MAG(void)
{ 
   I2C_STM32_Write_one_Byte(I2C1,MPU9250_GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
   Delay_10us(1000);	
   I2C_STM32_Write_one_Byte(I2C1,MPU9250_MAG_ADDRESS,0x0A,0x01);
   Delay_10us(1000);	
   BUF[0]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_MAG_ADDRESS,MPU9250_MAG_XOUT_L);
   BUF[1]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_MAG_ADDRESS,MPU9250_MAG_XOUT_H);
   T_X=(BUF[1]<<8)|BUF[0];

   BUF[2]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_MAG_ADDRESS,MPU9250_MAG_YOUT_L);
   BUF[3]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_MAG_ADDRESS,MPU9250_MAG_YOUT_H);
   T_Y=	(BUF[3]<<8)|BUF[2];
   						   
	 
   BUF[4]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_MAG_ADDRESS,MPU9250_MAG_ZOUT_L);
   BUF[5]=I2C_STM32_Read_one_Byte(I2C1,MPU9250_MAG_ADDRESS,MPU9250_MAG_ZOUT_H);
   T_Z=	(BUF[5]<<8)|BUF[4];
 					      
}
 

#else
 
void MPU9250_READ_ACCEL(int16_t accel[3])
{ 
   BUF[0]=IO_IIC_read_Data(MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_XOUT_L); 
   BUF[1]=IO_IIC_read_Data(MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_XOUT_H);
   accel[0] =	(BUF[1]<<8)|BUF[0];
   					  

   BUF[2]=IO_IIC_read_Data(MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_YOUT_L);
   BUF[3]=IO_IIC_read_Data(MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_YOUT_H);
   accel[1]=	(BUF[3]<<8)|BUF[2];
  
	
   BUF[4]=IO_IIC_read_Data(MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_ZOUT_L);
   BUF[5]=IO_IIC_read_Data(MPU9250_ACCEL_ADDRESS,MPU9250_ACCEL_ZOUT_H);
   accel[2] =	(BUF[5]<<8)|BUF[4];
   					   
}

void MPU9250_READ_GYRO(int16_t gyro[3])
{ 

   BUF[0]=IO_IIC_read_Data(MPU9250_GYRO_ADDRESS,MPU9250_GYRO_XOUT_L); 
   BUF[1]=IO_IIC_read_Data(MPU9250_GYRO_ADDRESS,MPU9250_GYRO_XOUT_H);
   gyro[0]=	(BUF[1]<<8)|BUF[0];
 						  

   BUF[2]=IO_IIC_read_Data(MPU9250_GYRO_ADDRESS,MPU9250_GYRO_YOUT_L);
   BUF[3]=IO_IIC_read_Data(MPU9250_GYRO_ADDRESS,MPU9250_GYRO_YOUT_H);
   gyro[1]=	(BUF[3]<<8)|BUF[2];
   						
	
   BUF[4]=IO_IIC_read_Data(MPU9250_GYRO_ADDRESS,MPU9250_GYRO_ZOUT_L);
   BUF[5]=IO_IIC_read_Data(MPU9250_GYRO_ADDRESS,MPU9250_GYRO_ZOUT_H);
   gyro[2]=	(BUF[5]<<8)|BUF[4];
   					  
 
}
 
void MPU9250_READ_MAG(int16_t mag[3])
{ 
   //IO_IIC_write_Command(MPU9250_GYRO_ADDRESS,0x37,0x02);//turn on Bypass Mode 
   //MPU9250_Delay(5000);
   //IO_IIC_write_Command(MPU9250_MAG_ADDRESS,0x0A,0x01);
   //MPU9250_Delay(55000);	
	
   BUF[0]=IO_IIC_read_Data(MPU9250_MAG_ADDRESS,MPU9250_MAG_XOUT_L);
   BUF[1]=IO_IIC_read_Data(MPU9250_MAG_ADDRESS,MPU9250_MAG_XOUT_H);
   mag[0]=(BUF[1]<<8)|BUF[0];

   BUF[2]=IO_IIC_read_Data(MPU9250_MAG_ADDRESS,MPU9250_MAG_YOUT_L);
   BUF[3]=IO_IIC_read_Data(MPU9250_MAG_ADDRESS,MPU9250_MAG_YOUT_H);
   mag[1]=	(BUF[3]<<8)|BUF[2];
   						   
	 
   BUF[4]=IO_IIC_read_Data(MPU9250_MAG_ADDRESS,MPU9250_MAG_ZOUT_L);
   BUF[5]=IO_IIC_read_Data(MPU9250_MAG_ADDRESS,MPU9250_MAG_ZOUT_H);
   mag[2]=	(BUF[5]<<8)|BUF[4];
	
 	 IO_IIC_write_Command(MPU9250_MAG_ADDRESS,0x0A,0x16);				      
}
 
#endif	
 
 

void MPU9250_GetValue(int16_t mpu9250_value[9])
{
	int16_t tem[3];
	
  MPU9250_READ_ACCEL(tem);
	mpu9250_value[0]=tem[0];  
	mpu9250_value[1]=tem[1];  
	mpu9250_value[2]=tem[2];		 
	 
	MPU9250_READ_GYRO(tem);      
	mpu9250_value[3]=tem[0];  
	mpu9250_value[4]=tem[1];  
	mpu9250_value[5]=tem[2];  	
	 
	MPU9250_READ_MAG(tem);	    
	mpu9250_value[6]=tem[0];   
	mpu9250_value[7]=tem[1];  
	mpu9250_value[8]=tem[2]; 

  MPU9250_newValues(mpu9250_value[0],mpu9250_value[1],mpu9250_value[2],
											mpu9250_value[3],mpu9250_value[4],mpu9250_value[5],
											mpu9250_value[6],mpu9250_value[7],mpu9250_value[8]);
	
	
	mpu9250_value[0]=MPU9250_FIFO[0][MPU9250_FIFO_Length];
	mpu9250_value[1]=MPU9250_FIFO[1][MPU9250_FIFO_Length];
	mpu9250_value[2]=MPU9250_FIFO[2][MPU9250_FIFO_Length];
	mpu9250_value[3]=MPU9250_FIFO[3][MPU9250_FIFO_Length];
	mpu9250_value[4]=MPU9250_FIFO[4][MPU9250_FIFO_Length];
	mpu9250_value[5]=MPU9250_FIFO[5][MPU9250_FIFO_Length];
	mpu9250_value[6]=MPU9250_FIFO[6][MPU9250_FIFO_Length];  
	mpu9250_value[7]=MPU9250_FIFO[7][MPU9250_FIFO_Length];
	mpu9250_value[8]=MPU9250_FIFO[8][MPU9250_FIFO_Length];

}
void  MPU9250_newValues(int16_t ax,int16_t ay,int16_t az,
												int16_t gx,int16_t gy,int16_t gz,
												int16_t mx,int16_t my,int16_t mz)
{
		unsigned char i ;
		int64_t sum=0;
		for(i=1;i<MPU9250_FIFO_Length;i++)
		{	//FIFO 操作
				MPU9250_FIFO[0][i-1]=MPU9250_FIFO[0][i];
				MPU9250_FIFO[1][i-1]=MPU9250_FIFO[1][i];
				MPU9250_FIFO[2][i-1]=MPU9250_FIFO[2][i];
				MPU9250_FIFO[3][i-1]=MPU9250_FIFO[3][i];
				MPU9250_FIFO[4][i-1]=MPU9250_FIFO[4][i];
				MPU9250_FIFO[5][i-1]=MPU9250_FIFO[5][i];
				MPU9250_FIFO[6][i-1]=MPU9250_FIFO[6][i];
				MPU9250_FIFO[7][i-1]=MPU9250_FIFO[7][i];
				MPU9250_FIFO[8][i-1]=MPU9250_FIFO[8][i];
		}
		
		MPU9250_FIFO[0][MPU9250_FIFO_Length-1]=ax;//将新的数据放置到 数据的最后面
		MPU9250_FIFO[1][MPU9250_FIFO_Length-1]=ay;
		MPU9250_FIFO[2][MPU9250_FIFO_Length-1]=az;
		MPU9250_FIFO[3][MPU9250_FIFO_Length-1]=gx;
		MPU9250_FIFO[4][MPU9250_FIFO_Length-1]=gy;
		MPU9250_FIFO[5][MPU9250_FIFO_Length-1]=gz;
		MPU9250_FIFO[6][MPU9250_FIFO_Length-1]=mx;
		MPU9250_FIFO[7][MPU9250_FIFO_Length-1]=my;
		MPU9250_FIFO[8][MPU9250_FIFO_Length-1]=mz;
		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{	//求当前数组的合，再取平均值
			 sum+=MPU9250_FIFO[0][i];
		}
		MPU9250_FIFO[0][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;

		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			 sum+=MPU9250_FIFO[1][i];
		}
		MPU9250_FIFO[1][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;

		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			 sum+=MPU9250_FIFO[2][i];
		}
		MPU9250_FIFO[2][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;

		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			 sum+=MPU9250_FIFO[3][i];
		}
		MPU9250_FIFO[3][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;

		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			 sum+=MPU9250_FIFO[4][i];
		}
		MPU9250_FIFO[4][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;

		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			 sum+=MPU9250_FIFO[5][i];
		}
		MPU9250_FIFO[5][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;
		
		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			sum+=MPU9250_FIFO[6][i];
		}
		MPU9250_FIFO[6][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;

		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			sum+=MPU9250_FIFO[7][i];
		}
		MPU9250_FIFO[7][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;


		sum=0;
		for(i=0;i<MPU9250_FIFO_Length;i++)
		{
			sum+=MPU9250_FIFO[8][i];
		}
		MPU9250_FIFO[8][MPU9250_FIFO_Length]=sum/MPU9250_FIFO_Length;
}

void MPU9250_DebugShowMessage(void)
{
	int16_t ax,ay,az,gx,gy,gz,mx,my,mz;
 	int16_t tem[3];
	
  MPU9250_READ_ACCEL(tem);
	ax=tem[0];  
	ay=tem[1];  
	az=tem[2];		 
	 
	MPU9250_READ_GYRO(tem);      
	gx=tem[0];  
	gy=tem[1];  
	gz=tem[2];
	 
	MPU9250_READ_MAG(tem);	    
	mx=tem[0];  
	my=tem[1];  
	mz=tem[2];

// 	UART_send_intdata(USART1,ax); UART_send_char(USART1,'\t');
// 	UART_send_intdata(USART1,ay); UART_send_char(USART1,'\t');
// 	UART_send_intdata(USART1,az); UART_send_char(USART1,'\t');

// 	UART_send_intdata(USART1,gx); UART_send_char(USART1,'\t');
// 	UART_send_intdata(USART1,gy); UART_send_char(USART1,'\t');
// 	UART_send_intdata(USART1,gz); UART_send_char(USART1,'\t');

// 	UART_send_intdata(USART1,mx); UART_send_char(USART1,'\t');
// 	UART_send_intdata(USART1,my); UART_send_char(USART1,'\t');
// 	UART_send_intdata(USART1,mz); UART_send_char(USART1,'\n');

} 	
void MPU9250_Delay(unsigned int cnt)
{
   while(cnt-- >0);
}
