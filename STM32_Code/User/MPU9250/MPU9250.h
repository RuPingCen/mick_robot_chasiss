#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f10x.h"

#define	MPU9250_GYRO_ADDRESS   0xD0	  
#define MPU9250_MAG_ADDRESS    0x18   
#define MPU9250_ACCEL_ADDRESS  0xD0 

#define MPU9250_ADDR            0X68    //MPU6500������IIC��ַ
#define MPU6500_ID				      0X71  	//MPU6500������ID

//MPU9250�ڲ���װ��һ��AK8963������,��ַ��ID����:
#define AK8963_ADDR				0X0C	//AK8963��I2C��ַ
#define AK8963_ID			  	0X48	//AK8963������ID


//AK8963���ڲ��Ĵ���
#define MAG_WIA					0x00	//AK8963������ID�Ĵ�����ַ
#define MAG_CNTL1       0X0A    
#define MAG_CNTL2       0X0B

#define MAG_XOUT_L				0X03	
#define MAG_XOUT_H				0X04
#define MAG_YOUT_L				0X05
#define MAG_YOUT_H				0X06
#define MAG_ZOUT_L				0X07
#define MAG_ZOUT_H				0X08

//MPU92506500���ڲ��Ĵ���
#define MPU9250_SELF_TESTX_REG		0X0D	//�Լ�Ĵ���X
#define MPU9250_SELF_TESTY_REG		0X0E	//�Լ�Ĵ���Y
#define MPU9250_SELF_TESTZ_REG		0X0F	//�Լ�Ĵ���Z
#define MPU9250_SELF_TESTA_REG		0X10	//�Լ�Ĵ���A
#define MPU9250_SAMPLE_RATE_REG		0X19	//����Ƶ�ʷ�Ƶ��
#define MPU9250_CFG_REG				0X1A	//���üĴ���
#define MPU9250_GYRO_CFG_REG		0X1B	//���������üĴ���
#define MPU9250_ACCEL_CFG_REG		0X1C	//���ٶȼ����üĴ���
#define MPU9250_MOTION_DET_REG		0X1F	//�˶���ֵⷧ���üĴ���
#define MPU9250_FIFO_EN_REG			0X23	//FIFOʹ�ܼĴ���
#define MPU9250_I2CMST_CTRL_REG		0X24	//IIC�������ƼĴ���
#define MPU9250_I2CSLV0_ADDR_REG	0X25	//IIC�ӻ�0������ַ�Ĵ���
#define MPU9250_I2CSLV0_REG			0X26	//IIC�ӻ�0���ݵ�ַ�Ĵ���
#define MPU9250_I2CSLV0_CTRL_REG	0X27	//IIC�ӻ�0���ƼĴ���
#define MPU9250_I2CSLV1_ADDR_REG	0X28	//IIC�ӻ�1������ַ�Ĵ���
#define MPU9250_I2CSLV1_REG			0X29	//IIC�ӻ�1���ݵ�ַ�Ĵ���
#define MPU9250_I2CSLV1_CTRL_REG	0X2A	//IIC�ӻ�1���ƼĴ���
#define MPU9250_I2CSLV2_ADDR_REG	0X2B	//IIC�ӻ�2������ַ�Ĵ���
#define MPU9250_I2CSLV2_REG			0X2C	//IIC�ӻ�2���ݵ�ַ�Ĵ���
#define MPU9250_I2CSLV2_CTRL_REG	0X2D	//IIC�ӻ�2���ƼĴ���
#define MPU9250_I2CSLV3_ADDR_REG	0X2E	//IIC�ӻ�3������ַ�Ĵ���
#define MPU9250_I2CSLV3_REG			0X2F	//IIC�ӻ�3���ݵ�ַ�Ĵ���
#define MPU9250_I2CSLV3_CTRL_REG	0X30	//IIC�ӻ�3���ƼĴ���
#define MPU9250_I2CSLV4_ADDR_REG	0X31	//IIC�ӻ�4������ַ�Ĵ���
#define MPU9250_I2CSLV4_REG			0X32	//IIC�ӻ�4���ݵ�ַ�Ĵ���
#define MPU9250_I2CSLV4_DO_REG		0X33	//IIC�ӻ�4д���ݼĴ���
#define MPU9250_I2CSLV4_CTRL_REG	0X34	//IIC�ӻ�4���ƼĴ���
#define MPU9250_I2CSLV4_DI_REG		0X35	//IIC�ӻ�4�����ݼĴ���

#define MPU9250_I2CMST_STA_REG		0X36	//IIC����״̬�Ĵ���
#define MPU9250_INTBP_CFG_REG		0X37	//�ж�/��·���üĴ���
#define MPU9250_INT_EN_REG			0X38	//�ж�ʹ�ܼĴ���
#define MPU9250_INT_STA_REG			0X3A	//�ж�״̬�Ĵ���

#define MPU9250_ACCEL_XOUTH_REG		0X3B	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU9250_ACCEL_XOUTL_REG		0X3C	//���ٶ�ֵ,X���8λ�Ĵ���
#define MPU9250_ACCEL_YOUTH_REG		0X3D	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU9250_ACCEL_YOUTL_REG		0X3E	//���ٶ�ֵ,Y���8λ�Ĵ���
#define MPU9250_ACCEL_ZOUTH_REG		0X3F	//���ٶ�ֵ,Z���8λ�Ĵ���
#define MPU9250_ACCEL_ZOUTL_REG		0X40	//���ٶ�ֵ,Z���8λ�Ĵ���

#define MPU9250_TEMP_OUTH_REG		0X41	//�¶�ֵ�߰�λ�Ĵ���
#define MPU9250_TEMP_OUTL_REG		0X42	//�¶�ֵ��8λ�Ĵ���

#define MPU9250_GYRO_XOUTH_REG		0X43	//������ֵ,X���8λ�Ĵ���
#define MPU9250_GYRO_XOUTL_REG		0X44	//������ֵ,X���8λ�Ĵ���
#define MPU9250_GYRO_YOUTH_REG		0X45	//������ֵ,Y���8λ�Ĵ���
#define MPU9250_GYRO_YOUTL_REG		0X46	//������ֵ,Y���8λ�Ĵ���
#define MPU9250_GYRO_ZOUTH_REG		0X47	//������ֵ,Z���8λ�Ĵ���
#define MPU9250_GYRO_ZOUTL_REG		0X48	//������ֵ,Z���8λ�Ĵ���

#define MPU9250_I2CSLV0_DO_REG		0X63	//IIC�ӻ�0���ݼĴ���
#define MPU9250_I2CSLV1_DO_REG		0X64	//IIC�ӻ�1���ݼĴ���
#define MPU9250_I2CSLV2_DO_REG		0X65	//IIC�ӻ�2���ݼĴ���
#define MPU9250_I2CSLV3_DO_REG		0X66	//IIC�ӻ�3���ݼĴ���

#define MPU9250_I2CMST_DELAY_REG	0X67	//IIC������ʱ����Ĵ���
#define MPU9250_SIGPATH_RST_REG		0X68	//�ź�ͨ����λ�Ĵ���
#define MPU9250_MDETECT_CTRL_REG	0X69	//�˶������ƼĴ���
#define MPU9250_USER_CTRL_REG		0X6A	//�û����ƼĴ���
#define MPU9250_PWR_MGMT1_REG		0X6B	//��Դ����Ĵ���1
#define MPU9250_PWR_MGMT2_REG		0X6C	//��Դ����Ĵ���2 
#define MPU9250_FIFO_CNTH_REG		0X72	//FIFO�����Ĵ����߰�λ
#define MPU9250_FIFO_CNTL_REG		0X73	//FIFO�����Ĵ����Ͱ�λ
#define MPU9250_FIFO_RW_REG			0X74	//FIFO��д�Ĵ���
#define MPU9250_DEVICE_ID_REG		0X75	//����ID�Ĵ���

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
