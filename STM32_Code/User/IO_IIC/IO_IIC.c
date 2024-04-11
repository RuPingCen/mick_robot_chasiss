/*********************************************************************************
 * 文件名       ：IO_IIC.h
 * 描述         ：IIC
 * 作者         ：CRP
 * 备注         ：IO口模拟IIC通信
**********************************************************************************/
 
#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
 
#include "IO_IIC.h"



void IO_IIC_GPIO_Init(void)
{		
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
		IO_IIC_Delay();IO_IIC_Delay();
		IO_IIC_Delay();IO_IIC_Delay();
	
}

//void SDA_DIR_OUT(void)//这里使用函数改变IO方向
//{		
////			GPIO_InitTypeDef GPIO_InitStruct;
////		//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//启动端口时钟
//// 
////			GPIO_InitStruct.GPIO_Mode=GPIO_Mode_OUT; 	
////			GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;                                    
////			GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;     
////			GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;                                      
////			GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;   
////			GPIO_Init(GPIOB, &GPIO_InitStruct);
//    
//				My_GPIO_Init(GPIOB,GPIO_Pin_7,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//自定义初始化函数
//			//	My_GPIO_Init(GPIOB,GPIO_Pin_11,GPIO_Mode_Out_OD, GPIO_Speed_50MHz);//自定义初始化函数
//}
//void SDA_DIR_IN(void)
//{		
////			GPIO_InitTypeDef GPIO_InitStruct;
////		//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);//启动端口时钟
//// 
////			GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN; 	
////			GPIO_InitStruct.GPIO_OType=GPIO_OType_OD;                                    
////			GPIO_InitStruct.GPIO_PuPd=GPIO_PuPd_UP;     
////			GPIO_InitStruct.GPIO_Pin=GPIO_Pin_7;                                      
////			GPIO_InitStruct.GPIO_Speed=GPIO_Speed_100MHz;   
////			GPIO_Init(GPIOB, &GPIO_InitStruct);
//				My_GPIO_Init(GPIOB,GPIO_Pin_7,GPIO_Mode_IPU, GPIO_Speed_50MHz);//自定义初始化函数
//			//My_GPIO_Init(GPIOB,GPIO_Pin_7,GPIO_Mode_IPU, GPIO_Speed_50MHz);//自定义初始化函数
//}

//----------------------------------------------------------------------------------------------------------//
 
void IO_IIC_Delay(void) //通信频率 375 hz
{
	 unsigned char tem_IIC=10;//默认39
	 
	 while(tem_IIC-- >0);
	  
}
 
void IO_IIC_start(void) //I2C start 信号
{
        
	SET_SDA_OUT();				//设置SDA方向为输出
	SDA_OUT_H;	 				//拉高数据线
	SCL_OUT_H;					//拉高时钟线
	IO_IIC_Delay();			//延时
	SDA_OUT_L; 					//产生下降沿
	IO_IIC_Delay();			//延时
	SCL_OUT_L;          //拉低时钟线

}
 
void IO_IIC_stop(void) //I2C stop 信号
{
	SET_SDA_OUT();
	SCL_OUT_L;
	SDA_OUT_L;
	IO_IIC_Delay();
	SCL_OUT_H;
	SDA_OUT_H;
	IO_IIC_Delay();
}


void IO_IIC_ack(void)    //ack   产生应答信号
{
	SCL_OUT_L;
	SET_SDA_OUT();
	SDA_OUT_L;
	IO_IIC_Delay();
	SCL_OUT_H;
	IO_IIC_Delay();
	SCL_OUT_L;
}


void IO_IIC_no_ack(void) //not ack 
{
	SCL_OUT_L;
	SET_SDA_OUT();
	SDA_OUT_H;
	IO_IIC_Delay();
	SCL_OUT_H;
	IO_IIC_Delay();
	SCL_OUT_L;
}
 
void IO_IIC_write_byte(unsigned char dat) //write a byte//向I2C总线发送一个字节数据
{
	unsigned char i ,temp;
	SET_SDA_OUT(); //这两个步骤，设置SDA为输出，和下拉时钟必不可少
	SCL_OUT_L;
	
	for ( i = 0 ; i < 8 ; i ++)
	{
		temp = dat &0x80;
		if ( temp == 0x80)
		{
			SDA_OUT_H;
		}
		else SDA_OUT_L;
		IO_IIC_Delay();
		dat <<= 1;
		SCL_OUT_H;
		IO_IIC_Delay();
		SCL_OUT_L;
		IO_IIC_Delay();
	}
}
 
unsigned char IO_IIC_read_byte(unsigned char ack) //read a byte//从I2C总线接收一个字节数据
{
	unsigned char i, dat = 0;
	SET_SDA_IN();
	IO_IIC_Delay();IO_IIC_Delay();
	for ( i = 0 ; i < 8 ; i ++)
	{
		SCL_OUT_L;
		IO_IIC_Delay();
		SCL_OUT_H;
		dat <<= 1;
		if ( SDA_IN) dat ++;
		IO_IIC_Delay();
	}
	if ( ack == 0) IO_IIC_no_ack();
	else IO_IIC_ack();
	return dat;
}
void IO_IIC_write_Command(unsigned char qijian_address,unsigned char reg_address,unsigned char command) //address+register+command
{
	IO_IIC_start();                      //起始信号
	IO_IIC_write_byte(qijian_address);   //发送设备地址+写信号
	IO_IIC_write_byte(reg_address);      //内部寄存器地址，
	IO_IIC_write_byte(command);          //内部寄存器数据，
	IO_IIC_stop();                       //发送停止信号
}  
unsigned char IO_IIC_read_Data(unsigned char qijian_address,unsigned char reg_address) //address(with bit 0 set) + register
{
	unsigned char REG_data=0;
	IO_IIC_start();                          //起始信号
	IO_IIC_write_byte(qijian_address);       //发送设备地址+写信号
	IO_IIC_write_byte(reg_address);          //内部寄存器地址，
 // IO_IIC_ack();                          //接收应答信号   注：这里加上了应答信号与没加应答信号的结果不同 
	IO_IIC_stop(); 
  IO_IIC_start();                  					//起始信号
	IO_IIC_write_byte(qijian_address+1);      //发送设备地址+读信号
//IO_IIC_ack();                            //接收应答信号
	REG_data = IO_IIC_read_byte(0);           //读出寄存器数据
	
	
/*IO_IIC_Delay();
  SCL = 1;		
	IO_IIC_Delay();
	SCL = 0;
	IO_IIC_Delay();  */
	
	IO_IIC_ack();                         //不接收应答信号
	IO_IIC_stop();                            //停止信号
	return REG_data;
} 

void I2C_STM32_Write_one_Byte(I2C_TypeDef* I2Cx,uint8_t slave_Address,u8 WriteAddr, u8 pBuffer)
{
	
  while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)); //检查IIC总线是否繁忙
    
  /* Send START condition */
  I2C_GenerateSTART(I2Cx, ENABLE);//产生开始条件

  /* Test on EV5 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));  //等待ACK

  /* Send EEPROM address for write */
  I2C_Send7bitAddress(I2Cx, slave_Address, I2C_Direction_Transmitter);
  
  /* Test on EV6 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
      
  /* Send the EEPROM's internal address to write to */
  I2C_SendData(I2Cx, WriteAddr);
  
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

  /* Send the byte to be written */
  I2C_SendData(I2Cx, pBuffer); 
   
	 
//	I2C_AcknowledgeConfig(I2Cx,DISABLE);
  /* Test on EV8 and clear it */
  while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  
  /* Send STOP condition */
  I2C_GenerateSTOP(I2Cx, ENABLE);//结束信号
 
  I2C_ClearFlag(I2C1, I2C_FLAG_AF);/* Clear AF flag */            
  I2C_GenerateSTOP(I2C1, ENABLE);/* STOP condition */
	
} 
