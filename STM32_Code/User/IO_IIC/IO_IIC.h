#ifndef _IO_IIC_H_
#define _IO_IIC_H_

 #include "stm32f4xx.h"

/*************端口定义***********************/  

#define SCL_OUT_H         GPIO_SetBits(GPIOB,GPIO_Pin_6)        //      PB6
#define SCL_OUT_L         GPIO_ResetBits(GPIOB,GPIO_Pin_6)      // 

#define SDA_OUT_H         GPIO_SetBits(GPIOB,GPIO_Pin_7)        //IIC数据引脚定义    PB7
#define SDA_OUT_L         GPIO_ResetBits(GPIOB,GPIO_Pin_7)      //IIC数据引脚定义 

#define SDA_IN            GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7) //读引脚数据

 
//#define SDA_DIR_IN        GPIOB->MODER &= 0xFFFF3FFF   //STM32使用直接操作地址有点问题 
//#define SDA_DIR_OUT				GPIOB->MODER |= 0x0000C000    //这里使用的函数代替的     

//IO 方向设置
#define SET_SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	//PB9输入模式
#define SET_SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB9输出模式



/********************************************************
 
********************************************************/

unsigned char IO_IIC_read_Data(unsigned char qijian_address,unsigned char reg_address); //address(with bit 0 set) + register
void IO_IIC_write_Command(unsigned char qijian_address,unsigned char reg_address,unsigned char command); //address+register+command

//-----------------------------------------------
void IO_IIC_GPIO_Init(void);
void IO_IIC_Delay(void); //通信频率 226k hz
void IO_IIC_start(void); //I2C start 信号
void IO_IIC_stop(void); //I2C stop 信号
void IO_IIC_ack(void);    //ack   I2C接收应答信号
void IO_IIC_no_ack(void); //not ack 
void IO_IIC_write_byte(unsigned char dat);//write a byte//向I2C总线发送一个字节数据
unsigned char IO_IIC_read_byte(unsigned char ack);//read a byte//从I2C总线接收一个字节数据
void SDA_DIR_IN(void);//设置SDA方向为输入
void SDA_DIR_OUT(void);//设置SDA方向为输出

void I2C_STM32_Write_one_Byte(I2C_TypeDef* I2Cx,uint8_t slave_Address,u8 WriteAddr, u8 pBuffer);

#endif
