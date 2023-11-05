#ifndef _IO_IIC_H_
#define _IO_IIC_H_

 #include "stm32f10x.h"

/*************�˿ڶ���***********************/  

#define SCL_OUT_H         GPIO_SetBits(GPIOB,GPIO_Pin_6)        //      PB6
#define SCL_OUT_L         GPIO_ResetBits(GPIOB,GPIO_Pin_6)      // 

#define SDA_OUT_H         GPIO_SetBits(GPIOB,GPIO_Pin_7)        //IIC�������Ŷ���    PB7
#define SDA_OUT_L         GPIO_ResetBits(GPIOB,GPIO_Pin_7)      //IIC�������Ŷ��� 

#define SDA_IN            GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7) //����������

 
//#define SDA_DIR_IN        GPIOB->MODER &= 0xFFFF3FFF   //STM32ʹ��ֱ�Ӳ�����ַ�е����� 
//#define SDA_DIR_OUT				GPIOB->MODER |= 0x0000C000    //����ʹ�õĺ��������     

//IO ��������
#define SET_SDA_IN() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=8<<28;} //PB7
#define SET_SDA_OUT() {GPIOB->CRL&=0X0FFFFFFF;GPIOB->CRL|=3<<28;}


/********************************************************
 
********************************************************/

unsigned char IO_IIC_read_Data(unsigned char qijian_address,unsigned char reg_address); //address(with bit 0 set) + register
void IO_IIC_write_Command(unsigned char qijian_address,unsigned char reg_address,unsigned char command); //address+register+command

//-----------------------------------------------
void IO_IIC_GPIO_Init(void);
void IO_IIC_Delay(void); //ͨ��Ƶ�� 226k hz
void IO_IIC_start(void); //I2C start �ź�
void IO_IIC_stop(void); //I2C stop �ź�
void IO_IIC_ack(void);    //ack   I2C����Ӧ���ź�
void IO_IIC_no_ack(void); //not ack 
void IO_IIC_write_byte(unsigned char dat);//write a byte//��I2C���߷���һ���ֽ�����
unsigned char IO_IIC_read_byte(unsigned char ack);//read a byte//��I2C���߽���һ���ֽ�����
void SDA_DIR_IN(void);//����SDA����Ϊ����
void SDA_DIR_OUT(void);//����SDA����Ϊ���



#endif
