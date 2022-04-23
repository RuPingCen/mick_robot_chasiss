#ifndef __NRF_24l01_H
#define __NRF_24l01_H


#include "stm32f10x.h"

//单片机为主机 24l01为从机    主机MISO接从机MISO
//****************************************NRF24L01端口定义***************************************

#define 	MOSI_Out_H	     GPIO_SetBits(GPIOA,GPIO_Pin_7) //置为引脚   
#define 	MOSI_Out_L	     GPIO_ResetBits(GPIOA,GPIO_Pin_7)//清零

 
#define 	MISO	          GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6)//读引脚数据

#define		SCK_Out_H	      GPIO_SetBits(GPIOA,GPIO_Pin_5) //置为引脚   
#define		SCK_Out_L       GPIO_ResetBits(GPIOA,GPIO_Pin_5)//清零 

#define		CE_Out_H	      GPIO_SetBits(GPIOG,GPIO_Pin_8) //置为引脚   
#define		CE_Out_L	      GPIO_ResetBits(GPIOG,GPIO_Pin_8)//清零

#define		CSN_Out_H	      GPIO_SetBits(GPIOG,GPIO_Pin_15) //置为引脚   
#define		CSN_Out_L       GPIO_ResetBits(GPIOG,GPIO_Pin_15)//清零

 
	//  IRQ ===  GPIOC,  GPIO_Pin_4 
 




//*********************************************NRF24L01*************************************
#define 		TX_ADR_WIDTH    5   	// 5 uints TX address width
#define 		RX_ADR_WIDTH    5   	// 5 uints RX address width
#define 		TX_PLOAD_WIDTH  32  	// 20 uints TX payload （数据长度）
#define 		RX_PLOAD_WIDTH  32  	// 20 uints TX payload


//*************NRF24L01寄存器指令*******************************************************//

#define NRF_24l01_READ_REG        0x00  	// 读寄存器指令
#define NRF_24l01_WRITE_REG       0x20 	// 写寄存器指令
#define NRF_24l01_RD_RX_PLOAD     0x61  	// 读取接收数据指令
#define NRF_24l01_WR_TX_PLOAD     0xA0  	// 写待发数据指令
#define NRF_24l01_FLUSH_TX        0xE1 	 // 冲洗发送 FIFO指令
#define NRF_24l01_FLUSH_RX        0xE2  	// 冲洗接收 FIFO指令
#define NRF_24l01_REUSE_TX_PL     0xE3  	// 定义重复装载数据指令
#define NRF_24l01_NOP             0xFF  	// 保留

//*************SPI(nRF24L01)寄存器地址****************************************************//

#define CONFIG          0x00  // 配置收发状态，CRC校验模式以及收发状态响应方式
#define EN_AA           0x01  // 自动应答功能设置
#define EN_RXADDR       0x02  // 可用信道设置
#define SETUP_AW        0x03  // 收发地址宽度设置
#define SETUP_RETR      0x04  // 自动重发功能设置
#define RF_CH           0x05  // 工作频率设置
#define RF_SETUP        0x06  // 发射速率、功耗功能设置
#define STATUS          0x07  // 状态寄存器
#define OBSERVE_TX      0x08  // 发送监测功能
#define CD              0x09  // 地址检测           
#define RX_ADDR_P0      0x0A  // 频道0接收数据地址
#define RX_ADDR_P1      0x0B  // 频道1接收数据地址
#define RX_ADDR_P2      0x0C  // 频道2接收数据地址
#define RX_ADDR_P3      0x0D  // 频道3接收数据地址
#define RX_ADDR_P4      0x0E  // 频道4接收数据地址
#define RX_ADDR_P5      0x0F  // 频道5接收数据地址
#define TX_ADDR         0x10  // 发送地址寄存器
#define RX_PW_P0        0x11  // 接收频道0接收数据长度
#define RX_PW_P1        0x12  // 接收频道0接收数据长度
#define RX_PW_P2        0x13  // 接收频道0接收数据长度
#define RX_PW_P3        0x14  // 接收频道0接收数据长度
#define RX_PW_P4        0x15  // 接收频道0接收数据长度
#define RX_PW_P5        0x16  // 接收频道0接收数据长度
#define FIFO_STATUS     0x17  // FIFO栈入栈出状态寄存器设置
//*****************************************************************************

void init_NRF24L01(void);//初始化无线通信函数
void SwitchToTxMode(void);//切换到发送模式
void SwitchToRxMode(void);//切换到接收模式
void Transmit_Tx_bufferdata(unsigned char *pBuf);//将txbuff 中的数据发送出去
 
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char uchars);
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char uchars);
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value);
unsigned char SPI_Read(unsigned char reg);
unsigned char SPI_RW(unsigned char dat);

void NRF24L01_Delayus(unsigned int n);

unsigned char NRF24l01_Interrupt_Handle(void);//添加返回值主要是方便中断函数调用;

#endif
 

/****************************中断服务函数************************************************
void IRS_ET0() interrupt 0
{		 
	 EX0=0;  
	 sta=0;
	 sta = SPI_Read(STATUS);	  // 读状态寄存器
	  if(sta&0x40)  //接收中断
		{
			SPI_Read_Buf(RD_RX_PLOAD,RxBuf,TX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer	
			led0=~led0;	   
			flag=1;		  
		}
		else if(sta&0x20)  //发送中断
		{
			led1=~led1;
			flag=2;
      SPI_RW_Reg(FLUSH_TX,0xff); 			
		}
		//else if(sta & 0x10)  //最大重发溢出
		//{
		//	led2=~led2;
		//	flag=3;	
		//}
		else ;
	 SPI_RW_Reg(WRITE_REG + STATUS, 0xff); //清除所有中断标志位 
		EX0=1;
}

*****************************************************************************************/
