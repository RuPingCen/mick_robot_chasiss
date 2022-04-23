#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "stm32f10x_rcc.h" 
#include "stm32f10x_Delay.h"
 
#include "IO_IIC.h"
#include "MPU6050.h"
 


	
	
#define  FIFO_Length   5 //滑动窗口滤波长度
 
 
//static int16_t ACC_OFFSET[3]={ 240,-160,14350};      // 加速度计 X、Y、Z轴 偏移值
//static int16_t GYRO_OFFSET[3]={53,-29,-6};   				// 陀螺仪 X、Y、Z轴 偏移值

int16_t MPU6050_FIFO[6][FIFO_Length+1]={'0'};





void mpu6050_delay(unsigned int temp)
{
   while(temp-- >0);
}
/*************************************************************************
*  函数名称：MPU6050_Init
*  功能说明：MPU6050初始化 内部已初始化IIC模块
*  参数说明：I2Cn        模块号（I2C0、I2C1）
*            doub_MPU6050dat    Double型数组
*  函数返回：无
*  修改时间：2014-7-12
*  备    注：CRP
*************************************************************************/
#ifdef  Hadware_IIC 

void MPU6050_Init(I2C_TypeDef* I2Cx)//MPU6050初始化 ± 2000 °/s ± 4g  5hz
{
	 I2C_STM32_Config_Init(I2Cx,400000);
		mpu6050_delay(72000); 
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,MPU6050_clock_PLL_X_gyro);//使用内部集成时钟8Mhz
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_2,MPU6050_wakeupfrequence_10hz);// 设置唤醒频率10 hz
	 //I2C_WriteAddr(i2cn, MPU6050_SlaveAddress,MPU6050_RA_SMPLRT_DIV,0x07);//Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_CONFIG,MPU6050_DLPF_BANDWIDTH_5hz);
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_GYRO_CONFIG,MPU6050_GYRO_Range_2000);//± 2000 °/s 
	 I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_ACCEL_CONFIG,MPU6050_ACCEL_Range_4g|MPU6050_ACCEL_DHPF_5HZ);//± 4g  5hz
}
#else 
void MPU6050_Init(void)//MPU6050初始化 ± 2000 °/s ± 2g  5hz
{
	// unsigned char dev_ID=0;
	 IO_IIC_GPIO_Init();//IIC引脚初始化     
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,0x80);//MPU6050复位
	 Delay_10us(2000);// nTime*100us 延时函数
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,MPU6050_CLOCK_PLL_ZGYRO);//MPU6050初始化 使用GYRO Z
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_SMPLRT_DIV,0x00);
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_CONFIG,MPU6050_DLPF_BW_42);//输出频率1K   带宽42hz
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_GYRO_CONFIG,MPU6050_GYRO_FS_2000);//± 2000 °/s 
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_ACCEL_CONFIG,MPU6050_DHPF_5 | MPU6050_ACCEL_FS_2g);//± 2g  5hz
	/*
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_INT_PIN_CFG,0x30);// 配置6050数据中断引脚   
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_INT_ENABLE,0x01);// （数据准备好以后会在int 引脚上输出 50us 的高电平 并一直持续到读出操作的完成）
	*/
	 while(IO_IIC_read_Data(MPU6050_SlaveAddress, MPU6050_RA_WHO_AM_I) != 0x68);//检测MPU6050 是否已经连接
}
#endif 

 
/*************************************************************************
*  函数名称：MPU6050_Data_Process
*  功能说明：MPU6050数据读取 及处理函数
*  参数说明：I2Cn        模块号（I2C0、I2C1）
*            doub_MPU6050dat    Double型数组
*  函数返回：无
*  修改时间：2014-7-12
*  备    注：CRP
*************************************************************************/
#ifdef  Hadware_IIC 
	
void MPU6050_Data_Process(I2C_TypeDef* I2Cx,short int *u16MPU6050dat)//MPU6050数据读取 及处理函数
{
   /*u16MPU6050dat[0]=(MPU_GetData(i2cn,MPU6050_RA_ACCEL_XOUT_H)-ACC_OFFSET[0])*2/4096.0;//乘 2是因为在初始化的时候选着的量程为 ±2g
     u16MPU6050dat[1]=(MPU_GetData(i2cn,MPU6050_RA_ACCEL_YOUT_H)-ACC_OFFSET[1])*2/4096.0;//若不乘以相应的量程则表示归一化的数据
     u16MPU6050dat[2]=(MPU_GetData(i2cn,MPU6050_RA_ACCEL_ZOUT_H)-ACC_OFFSET[2])*2/4096.0;
     u16MPU6050dat[3]=(MPU_GetData(i2cn,MPU6050_RA_GYRO_XOUT_H)-GYRO_OFFSET[0])/16.4;
     u16MPU6050dat[4]=(MPU_GetData(i2cn,MPU6050_RA_GYRO_YOUT_H)-GYRO_OFFSET[1])/16.4;
     u16MPU6050dat[5]=(MPU_GetData(i2cn,MPU6050_RA_GYRO_ZOUT_H)-GYRO_OFFSET[2])/16.4;      
   */
       
     u16MPU6050dat[0]=(MPU6050_GetData(I2Cx,MPU6050_RA_ACCEL_XOUT_H)-ACC_OFFSET[0]) ;//乘 2是因为在初始化的时候选着的量程为 ±2g
     u16MPU6050dat[1]=(MPU6050_GetData(I2Cx,MPU6050_RA_ACCEL_YOUT_H)+ACC_OFFSET[1]) ;//若不乘以相应的量程则表示归一化的数据
     u16MPU6050dat[2]=(MPU6050_GetData(I2Cx,MPU6050_RA_ACCEL_ZOUT_H)-ACC_OFFSET[2]) ;
     u16MPU6050dat[3]=(MPU6050_GetData(I2Cx,MPU6050_RA_GYRO_XOUT_H)-GYRO_OFFSET[0]) ;
     u16MPU6050dat[4]=(MPU6050_GetData(I2Cx,MPU6050_RA_GYRO_YOUT_H)+GYRO_OFFSET[1]) ;
     u16MPU6050dat[5]=(MPU6050_GetData(I2Cx,MPU6050_RA_GYRO_ZOUT_H)+GYRO_OFFSET[2]) ;
     
}
#else
void MPU6050_Data_Process(int16_t *int16t_MPU6050dat)//MPU6050数据读取 及处理函数
{
	    
	  
//			int16t_MPU6050dat[0]=MPU6050_GetData(MPU6050_RA_ACCEL_XOUT_H)-ACC_OFFSET[0];//使用四元素 加速度数据 不需要减去OFFSET 
//			int16t_MPU6050dat[1]=MPU6050_GetData(MPU6050_RA_ACCEL_YOUT_H)-ACC_OFFSET[1];// 
//			int16t_MPU6050dat[2]=MPU6050_GetData(MPU6050_RA_ACCEL_ZOUT_H)-ACC_OFFSET[2];
//			int16t_MPU6050dat[3]=MPU6050_GetData(MPU6050_RA_GYRO_XOUT_H)-GYRO_OFFSET[0];
//			int16t_MPU6050dat[4]=MPU6050_GetData(MPU6050_RA_GYRO_YOUT_H)-GYRO_OFFSET[1];
//			int16t_MPU6050dat[5]=MPU6050_GetData(MPU6050_RA_GYRO_ZOUT_H)-GYRO_OFFSET[2];  
	
	
	   	int16t_MPU6050dat[0]=MPU6050_GetData(MPU6050_RA_ACCEL_XOUT_H);//使用四元素 加速度数据 不需要减去OFFSET 
			int16t_MPU6050dat[1]=MPU6050_GetData(MPU6050_RA_ACCEL_YOUT_H); 
			int16t_MPU6050dat[2]=MPU6050_GetData(MPU6050_RA_ACCEL_ZOUT_H); 
			int16t_MPU6050dat[3]=MPU6050_GetData(MPU6050_RA_GYRO_XOUT_H);//陀螺仪数据在IMU 里面减去了OFFSET 
			int16t_MPU6050dat[4]=MPU6050_GetData(MPU6050_RA_GYRO_YOUT_H);
			int16t_MPU6050dat[5]=MPU6050_GetData(MPU6050_RA_GYRO_ZOUT_H); 


			MPU6050_newValues(int16t_MPU6050dat[0],int16t_MPU6050dat[1],int16t_MPU6050dat[2],//滑动窗口滤波
											  int16t_MPU6050dat[3],int16t_MPU6050dat[4],int16t_MPU6050dat[5]);


			int16t_MPU6050dat[0]=MPU6050_FIFO[0][FIFO_Length];
			int16t_MPU6050dat[1]=MPU6050_FIFO[1][FIFO_Length];
			int16t_MPU6050dat[2]=MPU6050_FIFO[2][FIFO_Length];
			int16t_MPU6050dat[3]=MPU6050_FIFO[3][FIFO_Length];
			int16t_MPU6050dat[4]=MPU6050_FIFO[4][FIFO_Length];
			int16t_MPU6050dat[5]=MPU6050_FIFO[5][FIFO_Length];

	
//			UART_send_intdata(USART1,int16t_MPU6050dat[0]);UART_send_char(USART1,'\t'); 
//			UART_send_intdata(USART1,int16t_MPU6050dat[1]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[2]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[3]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[4]);UART_send_char(USART1,'\t');
//			UART_send_intdata(USART1,int16t_MPU6050dat[5]);UART_send_char(USART1,'\n');
			 

 
}
#endif

/*************************************************************************
*  函数名称：MPU_GetData
*  功能说明：合成数据
*  参数说明：I2Cn        模块号（I2C0、I2C1）
*  函数返回：short int 型
*  修改时间：2014-7-12
*  备    注：CRP
*************************************************************************/
#ifdef  Hadware_IIC 
short int MPU6050_GetData(I2C_TypeDef* I2Cx,unsigned char REG_Address) 
{
	unsigned char H,L; 
	H=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,REG_Address);            //读取高八位         
	L=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,REG_Address+1);          //读取低八位
	return ((H<<8)+L);                                    //合成数据
}  

#else
short int MPU6050_GetData(unsigned char REG_Address) 
{
	unsigned char H,L;
	H=IO_IIC_read_Data(MPU6050_SlaveAddress,REG_Address);            //读取高八位
	L=IO_IIC_read_Data(MPU6050_SlaveAddress,REG_Address+1);          //读取低八位
      
	return ((H<<8)+L);                                    //合成数据
}   
#endif




/**************************设置6050 工作时钟********************************************
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
*******************************************************************************/
void MPU6050_setClockSource(I2C_TypeDef* I2Cx,unsigned char source)
{
	 
	I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,source);//± 2g  5hz
        
}
/**************************函数功能********************************************
  函数功能：设置6050辅助ＩＩＣ与６０５０主接口ＩＩＣ相连接
 　
*******************************************************************************/
/*
void MPU6050_setI2CBypassEnabled(I2Cn i2cn,unsigned char  enabled) 
{
	　
        I2C_WriteAddr(i2cn, MPU6050_SlaveAddress,PWR_MGMT_1,enabled);//sleep 位等于 0
}
 */

/**************************函数功能********************************************
 函数功能：设置MPU6050 工作模式
 enabled =1   睡觉
 enabled =0   正常工作
*******************************************************************************/
#ifdef  Hadware_IIC 
void MPU6050_setSleepEnabled(I2C_TypeDef* I2Cx,unsigned char  enabled) 
{  
    unsigned char dat;
    dat=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,PWR_MGMT_1);//把寄存器  PWR_MGMT_1中数据读取出来
    if(enabled==1)//睡觉模式
    {
        dat |=0x40; 
        I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,PWR_MGMT_1,dat);//sleep 位等于 1
    } 
    else//正常工作模式
    {
       dat &=0xBF; 
       I2C_STM32_Write_one_Byte(I2Cx, MPU6050_SlaveAddress,PWR_MGMT_1,dat);//sleep 位等于 0
    }
}
#else
void MPU6050_setSleepEnabled(uint8_t enabled) 
{
	  unsigned char tem;
	
   tem= IO_IIC_read_Data(MPU6050_SlaveAddress, MPU6050_RA_PWR_MGMT_1);
	
	 if(enabled ==1)
		  tem |=0x40;
	 else 
		  tem |=0x20;
	 IO_IIC_write_Command(MPU6050_SlaveAddress,MPU6050_RA_PWR_MGMT_1,tem); 
}
#endif



/*************************************************************************
*  函数名称：READ_temp
*  功能说明：读取6050中的温度显示温度
*  参数说明：I2Cn        模块号（I2C0、I2C1）
*  函数返回：short int 型
*  修改时间：2014-7-12
*  备    注：CRP
*************************************************************************/
#ifdef  Hadware_IIC 
short int READ_temp(I2C_TypeDef* I2Cx)//显示温度
 { 
        unsigned char Temp_h,Temp_l;
        short int Temperature;
        Temp_h=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,TEMP_OUT_H);            //读取温度高八位
        mpu6050_delay(30000); 
        Temp_l=I2C_STM32_Read_one_Byte(I2Cx,MPU6050_SlaveAddress,TEMP_OUT_L);             //读取温度低八位
  	Temperature=Temp_h<<8|Temp_l;                               //合成温度
  	Temperature = ((double)Temperature / 340.0) +36.25;   // 计算出温度
  	return  Temperature ;
 }
 #else
 short int READ_temp(void)//显示温度
{ 
		unsigned char Temp_h,Temp_l;
		short int Temperature;
		Temp_h=IO_IIC_read_Data(MPU6050_SlaveAddress,MPU6050_RA_TEMP_OUT_H);            //读取温度高八位

		Temp_l=IO_IIC_read_Data(MPU6050_SlaveAddress,MPU6050_RA_TEMP_OUT_L);             //读取温度低八位
		Temperature=Temp_h<<8|Temp_l;                               //合成温度
		Temperature = ((double) (Temperature + 13200)) / 280;   // 计算出温度
		return  Temperature ;
}
#endif  
 
 /**************************实现函数********************************************
*
*函数原型:		void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
*
*功　　能:	    将新的ADC数据更新到 FIFO数组，进行滤波处理
*
*备注：参考第七实验室
*
*******************************************************************************/
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz)
{
		unsigned char i ;
		int64_t sum=0;
		for(i=1;i<FIFO_Length;i++)
		{	//FIFO 操作
				MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
				MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
				MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
				MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
				MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
				MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
		}
		MPU6050_FIFO[0][FIFO_Length-1]=ax;//将新的数据放置到 数据的最后面
		MPU6050_FIFO[1][FIFO_Length-1]=ay;
		MPU6050_FIFO[2][FIFO_Length-1]=az;
		MPU6050_FIFO[3][FIFO_Length-1]=gx;
		MPU6050_FIFO[4][FIFO_Length-1]=gy;
		MPU6050_FIFO[5][FIFO_Length-1]=gz;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{	//求当前数组的合，再取平均值
			 sum+=MPU6050_FIFO[0][i];
		}
		MPU6050_FIFO[0][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[1][i];
		}
		MPU6050_FIFO[1][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[2][i];
		}
		MPU6050_FIFO[2][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[3][i];
		}
		MPU6050_FIFO[3][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[4][i];
		}
		MPU6050_FIFO[4][FIFO_Length]=sum/FIFO_Length;

		sum=0;
		for(i=0;i<FIFO_Length;i++)
		{
			 sum+=MPU6050_FIFO[5][i];
		}
		MPU6050_FIFO[5][FIFO_Length]=sum/FIFO_Length;
}
