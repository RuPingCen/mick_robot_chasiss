#ifndef _MPU6050_H_
#define _MPU6050_H_

#include "stm32f10x.h"

 

//#define Hadware_IIC   1   //是否使用硬件IIC 


//----------------- 定义MPU6050内部地址----------------------------------//

#define	MPU6050_SlaveAddress        0XD0	//IIC写入时的地址字节数据，+1为读取  {0XD0/2}=0x68

//----------------- 定义MPU6050内部地址----------------------------------//
#define MPU6050_RA_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU6050_RA_X_FINE_GAIN      0x03 //[7:0] X_FINE_GAIN
#define MPU6050_RA_Y_FINE_GAIN      0x04 //[7:0] Y_FINE_GAIN
#define MPU6050_RA_Z_FINE_GAIN      0x05 //[7:0] Z_FINE_GAIN
#define MPU6050_RA_XA_OFFS_H        0x06 //[15:0] XA_OFFS
#define MPU6050_RA_XA_OFFS_L_TC     0x07
#define MPU6050_RA_YA_OFFS_H        0x08 //[15:0] YA_OFFS
#define MPU6050_RA_YA_OFFS_L_TC     0x09
#define MPU6050_RA_ZA_OFFS_H        0x0A //[15:0] ZA_OFFS
#define MPU6050_RA_ZA_OFFS_L_TC     0x0B
#define MPU6050_RA_XG_OFFS_USRH     0x13 //[15:0] XG_OFFS_USR
#define MPU6050_RA_XG_OFFS_USRL     0x14
#define MPU6050_RA_YG_OFFS_USRH     0x15 //[15:0] YG_OFFS_USR
#define MPU6050_RA_YG_OFFS_USRL     0x16
#define MPU6050_RA_ZG_OFFS_USRH     0x17 //[15:0] ZG_OFFS_USR
#define MPU6050_RA_ZG_OFFS_USRL     0x18
#define MPU6050_RA_SMPLRT_DIV       0x19   //陀螺仪采样率，典型值：0x07(125Hz)
#define MPU6050_RA_CONFIG           0x1A   //低通滤波频率，典型值：0x06(5Hz)
#define MPU6050_RA_GYRO_CONFIG      0x1B   //陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s   0x10:1000deg/s    0x08:500deg/s    0x00:250deg/s)
#define MPU6050_RA_ACCEL_CONFIG     0x1C   //加速计自检、测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz    0x09:4G  0x11:8G   0x19:16G)
#define MPU6050_RA_FF_THR           0x1D
#define MPU6050_RA_FF_DUR           0x1E
#define MPU6050_RA_MOT_THR          0x1F
#define MPU6050_RA_MOT_DUR          0x20
#define MPU6050_RA_ZRMOT_THR        0x21
#define MPU6050_RA_ZRMOT_DUR        0x22
#define MPU6050_RA_FIFO_EN          0x23
#define MPU6050_RA_I2C_MST_CTRL     0x24
#define MPU6050_RA_I2C_SLV0_ADDR    0x25
#define MPU6050_RA_I2C_SLV0_REG     0x26
#define MPU6050_RA_I2C_SLV0_CTRL    0x27
#define MPU6050_RA_I2C_SLV1_ADDR    0x28
#define MPU6050_RA_I2C_SLV1_REG     0x29
#define MPU6050_RA_I2C_SLV1_CTRL    0x2A
#define MPU6050_RA_I2C_SLV2_ADDR    0x2B
#define MPU6050_RA_I2C_SLV2_REG     0x2C
#define MPU6050_RA_I2C_SLV2_CTRL    0x2D
#define MPU6050_RA_I2C_SLV3_ADDR    0x2E
#define MPU6050_RA_I2C_SLV3_REG     0x2F
#define MPU6050_RA_I2C_SLV3_CTRL    0x30
#define MPU6050_RA_I2C_SLV4_ADDR    0x31
#define MPU6050_RA_I2C_SLV4_REG     0x32
#define MPU6050_RA_I2C_SLV4_DO      0x33
#define MPU6050_RA_I2C_SLV4_CTRL    0x34
#define MPU6050_RA_I2C_SLV4_DI      0x35
#define MPU6050_RA_I2C_MST_STATUS   0x36
#define MPU6050_RA_INT_PIN_CFG      0x37
#define MPU6050_RA_INT_ENABLE       0x38
#define MPU6050_RA_DMP_INT_STATUS   0x39
#define MPU6050_RA_INT_STATUS       0x3A

#define MPU6050_RA_ACCEL_XOUT_H     0x3B  //加速度数据输出寄存器
#define MPU6050_RA_ACCEL_XOUT_L     0x3C
#define MPU6050_RA_ACCEL_YOUT_H     0x3D
#define MPU6050_RA_ACCEL_YOUT_L     0x3E
#define MPU6050_RA_ACCEL_ZOUT_H     0x3F
#define MPU6050_RA_ACCEL_ZOUT_L     0x40
#define MPU6050_RA_TEMP_OUT_H       0x41
#define MPU6050_RA_TEMP_OUT_L       0x42
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_GYRO_XOUT_L      0x44
#define MPU6050_RA_GYRO_YOUT_H      0x45
#define MPU6050_RA_GYRO_YOUT_L      0x46
#define MPU6050_RA_GYRO_ZOUT_H      0x47
#define MPU6050_RA_GYRO_ZOUT_L      0x48

#define MPU6050_RA_EXT_SENS_DATA_00 0x49
#define MPU6050_RA_EXT_SENS_DATA_01 0x4A
#define MPU6050_RA_EXT_SENS_DATA_02 0x4B
#define MPU6050_RA_EXT_SENS_DATA_03 0x4C
#define MPU6050_RA_EXT_SENS_DATA_04 0x4D
#define MPU6050_RA_EXT_SENS_DATA_05 0x4E
#define MPU6050_RA_EXT_SENS_DATA_06 0x4F
#define MPU6050_RA_EXT_SENS_DATA_07 0x50
#define MPU6050_RA_EXT_SENS_DATA_08 0x51
#define MPU6050_RA_EXT_SENS_DATA_09 0x52
#define MPU6050_RA_EXT_SENS_DATA_10 0x53
#define MPU6050_RA_EXT_SENS_DATA_11 0x54
#define MPU6050_RA_EXT_SENS_DATA_12 0x55
#define MPU6050_RA_EXT_SENS_DATA_13 0x56
#define MPU6050_RA_EXT_SENS_DATA_14 0x57
#define MPU6050_RA_EXT_SENS_DATA_15 0x58
#define MPU6050_RA_EXT_SENS_DATA_16 0x59
#define MPU6050_RA_EXT_SENS_DATA_17 0x5A
#define MPU6050_RA_EXT_SENS_DATA_18 0x5B
#define MPU6050_RA_EXT_SENS_DATA_19 0x5C
#define MPU6050_RA_EXT_SENS_DATA_20 0x5D
#define MPU6050_RA_EXT_SENS_DATA_21 0x5E
#define MPU6050_RA_EXT_SENS_DATA_22 0x5F
#define MPU6050_RA_EXT_SENS_DATA_23 0x60
#define MPU6050_RA_MOT_DETECT_STATUS    0x61
#define MPU6050_RA_I2C_SLV0_DO      0x63
#define MPU6050_RA_I2C_SLV1_DO      0x64
#define MPU6050_RA_I2C_SLV2_DO      0x65
#define MPU6050_RA_I2C_SLV3_DO      0x66
#define MPU6050_RA_I2C_MST_DELAY_CTRL   0x67
#define MPU6050_RA_SIGNAL_PATH_RESET    0x68 
#define MPU6050_RA_MOT_DETECT_CTRL      0x69
#define MPU6050_RA_USER_CTRL        0x6A
#define MPU6050_RA_PWR_MGMT_1       0x6B    //电源管理，典型值：0x00(正常启用)
#define MPU6050_RA_PWR_MGMT_2       0x6C
#define MPU6050_RA_BANK_SEL         0x6D
#define MPU6050_RA_MEM_START_ADDR   0x6E
#define MPU6050_RA_MEM_R_W          0x6F
#define MPU6050_RA_DMP_CFG_1        0x70
#define MPU6050_RA_DMP_CFG_2        0x71
#define MPU6050_RA_FIFO_COUNTH      0x72
#define MPU6050_RA_FIFO_COUNTL      0x73
#define MPU6050_RA_FIFO_R_W         0x74
#define MPU6050_RA_WHO_AM_I         0x75  //IIC地址寄存器(默认数值0x68，只读)




/********************配置时钟源 for PWR_MGMT_2*************************/
#define  MPU6050_wakeupfrequence_1_25hz    0x00
#define  MPU6050_wakeupfrequence_2_5hz     0x40
#define  MPU6050_wakeupfrequence_5hz       0x80
#define  MPU6050_wakeupfrequence_10hz      0xc0
/*********************************************************************/
  
//---------------------------------------------------------------//
//----------config for MPU6050_RA_PWR_MGMT_1 --------------------// 
#define MPU6050_CLOCK_INTERNAL_8MHZ          0x00
#define MPU6050_CLOCK_PLL_XGYRO              0x01
#define MPU6050_CLOCK_PLL_YGYRO              0x02
#define MPU6050_CLOCK_PLL_ZGYRO              0x03
#define MPU6050_CLOCK_PLL_EXT32768           0x04
#define MPU6050_CLOCK_PLL_EXT19200khz        0x05
//---------------------------------------------------------------//
//----------config for MPU6050_GYRO_FS_2000 --------------------//  
#define MPU6050_GYRO_FS_250         				0x00
#define MPU6050_GYRO_FS_500         				0x08
#define MPU6050_GYRO_FS_1000        				0x10
#define MPU6050_GYRO_FS_2000        				0x18

//---------------------------------------------------------------//
//----------config for MPU6050_RA_ACCEL_CONFIG ------------------//
#define MPU6050_ACCEL_FS_2g          0x00
#define MPU6050_ACCEL_FS_4g          0x08
#define MPU6050_ACCEL_FS_8g          0x10
#define MPU6050_ACCEL_FS_16g         0x18

#define MPU6050_DHPF_RESET          0x00
#define MPU6050_DHPF_5              0x01
#define MPU6050_DHPF_2P5            0x02
#define MPU6050_DHPF_1P25           0x03
#define MPU6050_DHPF_0P63           0x04
#define MPU6050_DHPF_HOLD           0x07
//---------------------------------------------------------------//
//----------config for MPU6050_RA_CONFIG-------------------------//

#define MPU6050_DLPF_BW_256         0x00 //加速度采样频率固定为 1khz  陀螺仪当且仅当DLPF=256hz 时等于 8khz
#define MPU6050_DLPF_BW_188         0x01 //其余时刻等于 1khz
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06
//---------------------------------------------------------------//
//---------------------------------------------------------------//








#ifdef  Hadware_IIC  //使用硬件IIC

extern void MPU6050_Init(I2C_TypeDef* I2Cx);//MPU6050初始化 ± 2000 °/s ± 0g  5hz
extern void MPU6050_Data_Process(I2C_TypeDef* I2Cx,short int *u16MPU6050dat);//MPU6050数据读取 及处理函数
extern void MPU6050_setSleepEnabled(I2C_TypeDef* I2Cx,unsigned char  enabled);
extern short int READ_temp(I2C_TypeDef* I2Cx);//显示温度 
extern short int MPU6050_GetData(I2C_TypeDef* I2Cx,unsigned char REG_Address);//合成数据

#else
 
extern void MPU6050_Init(void);//MPU6050初始化
extern void MPU6050_Data_Process(int16_t *int16t_MPU6050dat);//MPU6050数据读取 及处理函数
extern void MPU6050_setSleepEnabled(uint8_t enabled); 
extern short int READ_temp(void);//显示温度
extern short int MPU6050_GetData(unsigned char REG_Address); 
#endif
void  MPU6050_newValues(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz);

 

#endif
