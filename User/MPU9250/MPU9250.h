#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f10x.h"

#define	MPU9250_GYRO_ADDRESS   0xD0	  
#define MPU9250_MAG_ADDRESS    0x18   
#define MPU9250_ACCEL_ADDRESS  0xD0 

#define MPU9250_ADDR            0X68    //MPU6500的器件IIC地址
#define MPU6500_ID				      0X71  	//MPU6500的器件ID

//MPU9250内部封装了一个AK8963磁力计,地址和ID如下:
#define AK8963_ADDR				0X0C	//AK8963的I2C地址
#define AK8963_ID			  	0X48	//AK8963的器件ID


//AK8963的内部寄存器
#define MAG_WIA					0x00	//AK8963的器件ID寄存器地址
#define MAG_CNTL1       0X0A    
#define MAG_CNTL2       0X0B

#define MAG_XOUT_L				0X03	
#define MAG_XOUT_H				0X04
#define MAG_YOUT_L				0X05
#define MAG_YOUT_H				0X06
#define MAG_ZOUT_L				0X07
#define MAG_ZOUT_H				0X08

//MPU92506500的内部寄存器
#define MPU9250_SELF_TESTX_REG		0X0D	//自检寄存器X
#define MPU9250_SELF_TESTY_REG		0X0E	//自检寄存器Y
#define MPU9250_SELF_TESTZ_REG		0X0F	//自检寄存器Z
#define MPU9250_SELF_TESTA_REG		0X10	//自检寄存器A
#define MPU9250_SAMPLE_RATE_REG		0X19	//采样频率分频器
#define MPU9250_CFG_REG				0X1A	//配置寄存器
#define MPU9250_GYRO_CFG_REG		0X1B	//陀螺仪配置寄存器
#define MPU9250_ACCEL_CFG_REG		0X1C	//加速度计配置寄存器
#define MPU9250_MOTION_DET_REG		0X1F	//运动检测阀值设置寄存器
#define MPU9250_FIFO_EN_REG			0X23	//FIFO使能寄存器
#define MPU9250_I2CMST_CTRL_REG		0X24	//IIC主机控制寄存器
#define MPU9250_I2CSLV0_ADDR_REG	0X25	//IIC从机0器件地址寄存器
#define MPU9250_I2CSLV0_REG			0X26	//IIC从机0数据地址寄存器
#define MPU9250_I2CSLV0_CTRL_REG	0X27	//IIC从机0控制寄存器
#define MPU9250_I2CSLV1_ADDR_REG	0X28	//IIC从机1器件地址寄存器
#define MPU9250_I2CSLV1_REG			0X29	//IIC从机1数据地址寄存器
#define MPU9250_I2CSLV1_CTRL_REG	0X2A	//IIC从机1控制寄存器
#define MPU9250_I2CSLV2_ADDR_REG	0X2B	//IIC从机2器件地址寄存器
#define MPU9250_I2CSLV2_REG			0X2C	//IIC从机2数据地址寄存器
#define MPU9250_I2CSLV2_CTRL_REG	0X2D	//IIC从机2控制寄存器
#define MPU9250_I2CSLV3_ADDR_REG	0X2E	//IIC从机3器件地址寄存器
#define MPU9250_I2CSLV3_REG			0X2F	//IIC从机3数据地址寄存器
#define MPU9250_I2CSLV3_CTRL_REG	0X30	//IIC从机3控制寄存器
#define MPU9250_I2CSLV4_ADDR_REG	0X31	//IIC从机4器件地址寄存器
#define MPU9250_I2CSLV4_REG			0X32	//IIC从机4数据地址寄存器
#define MPU9250_I2CSLV4_DO_REG		0X33	//IIC从机4写数据寄存器
#define MPU9250_I2CSLV4_CTRL_REG	0X34	//IIC从机4控制寄存器
#define MPU9250_I2CSLV4_DI_REG		0X35	//IIC从机4读数据寄存器

#define MPU9250_I2CMST_STA_REG		0X36	//IIC主机状态寄存器
#define MPU9250_INTBP_CFG_REG		0X37	//中断/旁路设置寄存器
#define MPU9250_INT_EN_REG			0X38	//中断使能寄存器
#define MPU9250_INT_STA_REG			0X3A	//中断状态寄存器

#define MPU9250_ACCEL_XOUTH_REG		0X3B	//加速度值,X轴高8位寄存器
#define MPU9250_ACCEL_XOUTL_REG		0X3C	//加速度值,X轴低8位寄存器
#define MPU9250_ACCEL_YOUTH_REG		0X3D	//加速度值,Y轴高8位寄存器
#define MPU9250_ACCEL_YOUTL_REG		0X3E	//加速度值,Y轴低8位寄存器
#define MPU9250_ACCEL_ZOUTH_REG		0X3F	//加速度值,Z轴高8位寄存器
#define MPU9250_ACCEL_ZOUTL_REG		0X40	//加速度值,Z轴低8位寄存器

#define MPU9250_TEMP_OUTH_REG		0X41	//温度值高八位寄存器
#define MPU9250_TEMP_OUTL_REG		0X42	//温度值低8位寄存器

#define MPU9250_GYRO_XOUTH_REG		0X43	//陀螺仪值,X轴高8位寄存器
#define MPU9250_GYRO_XOUTL_REG		0X44	//陀螺仪值,X轴低8位寄存器
#define MPU9250_GYRO_YOUTH_REG		0X45	//陀螺仪值,Y轴高8位寄存器
#define MPU9250_GYRO_YOUTL_REG		0X46	//陀螺仪值,Y轴低8位寄存器
#define MPU9250_GYRO_ZOUTH_REG		0X47	//陀螺仪值,Z轴高8位寄存器
#define MPU9250_GYRO_ZOUTL_REG		0X48	//陀螺仪值,Z轴低8位寄存器

#define MPU9250_I2CSLV0_DO_REG		0X63	//IIC从机0数据寄存器
#define MPU9250_I2CSLV1_DO_REG		0X64	//IIC从机1数据寄存器
#define MPU9250_I2CSLV2_DO_REG		0X65	//IIC从机2数据寄存器
#define MPU9250_I2CSLV3_DO_REG		0X66	//IIC从机3数据寄存器

#define MPU9250_I2CMST_DELAY_REG	0X67	//IIC主机延时管理寄存器
#define MPU9250_SIGPATH_RST_REG		0X68	//信号通道复位寄存器
#define MPU9250_MDETECT_CTRL_REG	0X69	//运动检测控制寄存器
#define MPU9250_USER_CTRL_REG		0X6A	//用户控制寄存器
#define MPU9250_PWR_MGMT1_REG		0X6B	//电源管理寄存器1
#define MPU9250_PWR_MGMT2_REG		0X6C	//电源管理寄存器2 
#define MPU9250_FIFO_CNTH_REG		0X72	//FIFO计数寄存器高八位
#define MPU9250_FIFO_CNTL_REG		0X73	//FIFO计数寄存器低八位
#define MPU9250_FIFO_RW_REG			0X74	//FIFO读写寄存器
#define MPU9250_DEVICE_ID_REG		0X75	//器件ID寄存器

//****************************************
#define	MPU9250_SMPLRT_DIV		0x19	
#define	MPU9250_CONFIG			0x1A	
#define	MPU9250_GYRO_CONFIG		0x1B	
#define	MPU9250_ACCEL_CONFIG	0x1C	

#define	MPU9250_ACCEL_XOUT_H	0x3B
#define	MPU9250_ACCEL_XOUT_L	0x3C
#define	MPU9250_ACCEL_YOUT_H	0x3D
#define	MPU9250_ACCEL_YOUT_L	0x3E
#define	MPU9250_ACCEL_ZOUT_H	0x3F
#define	MPU9250_ACCEL_ZOUT_L	0x40

#define	MPU9250_TEMP_OUT_H		0x41
#define	MPU9250_TEMP_OUT_L		0x42

#define	MPU9250_GYRO_XOUT_H		0x43
#define	MPU9250_GYRO_XOUT_L		0x44	
#define	MPU9250_GYRO_YOUT_H		0x45
#define	MPU9250_GYRO_YOUT_L		0x46
#define	MPU9250_GYRO_ZOUT_H		0x47
#define	MPU9250_GYRO_ZOUT_L		0x48

		
#define MPU9250_MAG_XOUT_L		0x03
#define MPU9250_MAG_XOUT_H		0x04
#define MPU9250_MAG_YOUT_L		0x05
#define MPU9250_MAG_YOUT_H		0x06
#define MPU9250_MAG_ZOUT_L		0x07
#define MPU9250_MAG_ZOUT_H		0x08


#define	MPU9250_PWR_MGMT_1		0x6B	
#define	MPU9250_WHO_AM_I		  0x75	


//****************************
 


	 



void MPU9250_Init(void);

void MPU9250_READ_GYRO(int16_t gyro[3]);
void MPU9250_READ_ACCEL(int16_t accel[3]);
void MPU9250_READ_MAG(int16_t mag[3]);

void MPU9250_newValues(int16_t ax,int16_t ay,int16_t az,
												int16_t gx,int16_t gy,int16_t gz,
												int16_t mx,int16_t my,int16_t mz);

void MPU9250_GetValue(int16_t mpu9250_value[9]);
void MPU9250_DebugShowMessage(void);

void MPU9250_Delay(unsigned int cnt);

#endif
