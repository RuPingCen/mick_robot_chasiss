#include "stm32f10x_sdio.h"  	
#include "stm32f10x_Delay.h"


#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"


#include "WS2812.h"

#define Length_WS2812_datas 180
unsigned char WS2812_datas[180][3];  // 试用于多个灯的情况，每次都对整个数组进行刷新
unsigned char WS2812_light_flag =0x00;
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//SPI 驱动函数	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/6/13 
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
 

//以下是SPI模块的初始化代码，配置成主机模式，访问SD Card/W25X16/24L01/JF24C							  
//SPI口初始化
//这里针是对SPI1的初始化



void SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//选择了串行时钟的稳态:时钟悬空高
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//数据捕获于第二个时钟沿
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//定义波特率预分频的值:波特率预分频值为256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 10;	//CRC值计算的多项式
	SPI_Init(SPI1, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
 
	SPI_Cmd(SPI1, ENABLE); //使能SPI外设
	
	//SPI1_ReadWriteByte(0xff);//启动传输		 
}
void SPI2_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);//GPIOFB3,4,5
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_SetBits(GPIOB,GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15);
	
	/* SPI2 configuration */  
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//全双工工作方式
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//主机空闲时刻的电平(也是初始化电平)
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//决定采样相位   
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;  
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//SPI分频系数
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 10;//SPI CRC效验
	SPI_Init(SPI2, &SPI_InitStructure);


	SPI_Cmd(SPI2, ENABLE);
	//SPI2_ReadWriteByte(0xff);//启动传输	
} 
//SPI 速度设置函数
//SpeedSet:
//SPI_BaudRatePrescaler_2   2分频   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8分频   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16分频  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256分频 (SPI 281.25K@sys 72M)
  
 

//SPIx 读写一个字节
//TxData:要写入的字节
//返回值:读取到的字节
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	
    
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
            retry++;
            if(retry>200)return 0;
	}			  
    
	SPI_I2S_SendData(SPI1, TxData); //通过外设SPIx发送一个数据		
  SPI2_ReadWriteByte(TxData);
  return 0x00;	
}
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
    
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //检查指定的SPI标志位设置与否:发送缓存空标志位
	{
            retry++;
            if(retry>200)return 0;
	}			  
    
	SPI_I2S_SendData(SPI2, TxData); //通过外设SPIx发送一个数据		
  return 0x00;	
}


void WS2812_RGB_Init(void)
{
	SPI1_Init();		   //初始化SPI	
	SPI2_Init();		   //初始化SPI
  WS2812_RGB_Rest();
}

// 复位
void WS2812_RGB_Rest(void)
{  
	  unsigned int i=0;
		for(i=0;i<250;i++)//50us RES
		{
			SPI1_ReadWriteByte(0x00);//0	   //B
		}
}


 
void WS2812_Send_Byte(unsigned char dat_color)
{
	unsigned char j=0;
	for(j=0;j<8;j++)
	{
		if((dat_color<<j)&0x80) //先发高位
			SPI1_ReadWriteByte(0x3C);//1 0x3C   //R
		else
			SPI1_ReadWriteByte(0x30);//0	0x30
	
	}
		 
}


// 彩色的灯 
void WS2812_RGB_Color(unsigned char R,unsigned char G,unsigned char B,unsigned int j)
{ 
		unsigned int i=0;
	  WS2812_RGB_Rest();
		for(i=0;i<j;i++)
		{
			WS2812_Send_Byte(G); //G
			
			WS2812_Send_Byte(R); //R
			
			WS2812_Send_Byte(B); //B
			;;;;
		}
}
// 让不同的灯显示不同的颜色
// 思路：将所有灯的颜色值存储到一个全局变量中，然后每次改变改变其中的对应灯
// 所在位置的值。 
// 每次写入都多所有的灯进行操作
void WS2812_RGB_All(void)
{ 
	  static unsigned int times_count;
	  unsigned char tem_R,tem_G,tem_B;
	  unsigned char setp_R=0,setp_G=0,setp_B=0;
		unsigned int i=0;
	if(WS2812_light_flag == 0x00) 
	{
		  
			WS2812_RGB_Rest();
			for(i=0;i<Length_WS2812_datas;i++)
			{
	
			WS2812_Send_Byte(WS2812_datas[i][1]); //G
			WS2812_Send_Byte(WS2812_datas[i][0]); //
			WS2812_Send_Byte(WS2812_datas[i][2]); //
			}
 
	}
	else if(WS2812_light_flag == 0x01) //慢慢变亮
	{
		  if(times_count > 8) //呼吸灯效果 由定时器调用
			{
					WS2812_RGB_Rest();
					for(i=0;i<Length_WS2812_datas;i++)
					{
						tem_G = WS2812_datas[i][1];
						tem_R = WS2812_datas[i][0];
						tem_B = WS2812_datas[i][2];
						
						setp_G = times_count/7;
						if(setp_G> tem_G) setp_G=tem_G;
						setp_R = times_count/7;
						if(setp_R> tem_R) setp_R=tem_R;
						setp_B = times_count/7;
						if(setp_B> tem_B) setp_B=tem_B;
						WS2812_Send_Byte(setp_G); //G
						WS2812_Send_Byte(setp_R); //
						WS2812_Send_Byte(setp_B); //
						
			//			WS2812_Send_Byte(WS2812_datas[i][1]); //G
			//			WS2812_Send_Byte(WS2812_datas[i][0]); //
			//			WS2812_Send_Byte(WS2812_datas[i][2]); //
					}
			}
			
			times_count++;
			if(times_count > 70) 
			{
				//WS2812_light_flag=0x01;
			  times_count=0;
			}
	}
	else if(WS2812_light_flag == 0x02) //慢慢变暗
	{
		  if(times_count <100) //呼吸灯效果 由定时器调用
			{
					WS2812_RGB_Rest();
					for(i=0;i<Length_WS2812_datas;i++)
					{
						tem_G = WS2812_datas[i][1];
						tem_R = WS2812_datas[i][0];
						tem_B = WS2812_datas[i][2];
						times_count =100-times_count;
						if(tem_G > 0)
						{
								setp_G = times_count/8;
							  if(setp_G< tem_G) 
									tem_G-=setp_G;
						}
						if(tem_R > 0)
						{
							  setp_R = times_count/8;
								if(setp_R< tem_R) 
									 tem_R-=setp_R;
						}
						if(tem_B > 0)
						{
							 setp_B = times_count/8;
								if(setp_B< tem_B) 
									 tem_B-=setp_B;
						}
						
						WS2812_Send_Byte(tem_G); //G
						WS2812_Send_Byte(tem_R); //
						WS2812_Send_Byte(tem_B); //
						
			//			WS2812_Send_Byte(WS2812_datas[i][1]); //G
			//			WS2812_Send_Byte(WS2812_datas[i][0]); //
			//			WS2812_Send_Byte(WS2812_datas[i][2]); //
					}
			}
			
			
			if(times_count < 1) 
			{
				WS2812_light_flag=0x02;
			  times_count=107;
			}
			times_count--;
	}

}



// 绿色的灯
void WS2812_RGB_Green(unsigned int j)
{  
		unsigned int i=0;
	  WS2812_RGB_Rest();
	  for(i=0;i<j;i++)
		{
			SPI1_ReadWriteByte(0x3C);//1    //R
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 

			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G

			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
		}
}
// 红色的灯
void WS2812_RGB_Red(unsigned int j)
{  
		unsigned int i=0;
	  WS2812_RGB_Rest();
	  for(i=0;i<j;i++)
		{
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
				
				
					SPI1_ReadWriteByte(0x3C);//1    //R
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 

					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
		}
}
// 黄色的灯
void WS2812_RGB_Yellow(unsigned int j)
{  
		unsigned int i=0;
	  WS2812_RGB_Rest();
	  for(i=0;i<j;i++)
		{
					SPI1_ReadWriteByte(0x3C);//0	   //G
					SPI1_ReadWriteByte(0x3C);//0	   //G
					SPI1_ReadWriteByte(0x3C);//0	   //G	
					SPI1_ReadWriteByte(0x3C);//0	   //G
					SPI1_ReadWriteByte(0x3C);//0	   //G
					SPI1_ReadWriteByte(0x3C);//0	   //G
					SPI1_ReadWriteByte(0x3C);//0	   //G	
					SPI1_ReadWriteByte(0x3C);//0	   //G
				
				
					SPI1_ReadWriteByte(0x3C);//1    //R
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 

					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
		}
}
// 蓝色的灯
void WS2812_RGB_Blue(unsigned int j)
{  
		unsigned int i=0;
	  WS2812_RGB_Rest();
	  for(i=0;i<j;i++)
		{
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G
			 
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				
				
				SPI1_ReadWriteByte(0x3C);//1    //G
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
		}
}



// 黑色的灯
void WS2812_RGB_Black(unsigned int j)
{  
		unsigned int i=0;
	  WS2812_RGB_Rest();
	  for(i=0;i<j;i++)
		{
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
				
				
					SPI1_ReadWriteByte(0x30);//0    //R
					SPI1_ReadWriteByte(0x30);//0 
					SPI1_ReadWriteByte(0x30);//0 
					SPI1_ReadWriteByte(0x30);//1 
					SPI1_ReadWriteByte(0x30);//1 
					SPI1_ReadWriteByte(0x30);//1 
					SPI1_ReadWriteByte(0x30);//1 
					SPI1_ReadWriteByte(0x30);//1 

					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
		}
}



void WS2812_RGB_Test2(void)
{
	unsigned int i=0;
	unsigned char G,R,B;
	WS2812_RGB_Rest();

	for(i=0;i<180;i++)
	{
		if(i%2 == 0)
		{
				R =0x01; 	G =0x00; 	B =0x00;
				WS2812_Send_Byte(G); //G
				WS2812_Send_Byte(R); //R
				WS2812_Send_Byte(B); //B
		}
		else
		{
			  R =0x00; 	G =0x01; 	B =0x00;
				WS2812_Send_Byte(G); //G
				WS2812_Send_Byte(R); //R
				WS2812_Send_Byte(B); //B
		}
 
	}
}






void WS2812_RGB_Test(void)
{
	unsigned int i=0;
	
		for(i=0;i<225;i++)//50us RES
		{
			SPI1_ReadWriteByte(0x00);//0	   //B
		}


		for(i=0;i<60;i++)
		{
			SPI1_ReadWriteByte(0x3C);//1    //R
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 
			SPI1_ReadWriteByte(0x3C);//1 

			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G

			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G	
			SPI1_ReadWriteByte(0x30);//0	   //G
			SPI1_ReadWriteByte(0x30);//0	   //G	
		}
		
		Delay_10us(50000);
		
		
			for(i=0;i<225;i++)//50us RES
			{
				SPI1_ReadWriteByte(0x00);//0	   //B
			}
  		for(i=0;i<60;i++)
			{
				
				
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
				
				
					SPI1_ReadWriteByte(0x3C);//1    //G
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 
					SPI1_ReadWriteByte(0x3C);//1 

					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G	
					SPI1_ReadWriteByte(0x30);//0	   //G
					SPI1_ReadWriteByte(0x30);//0	   //G	
				
			}
			Delay_10us(50000);	 
				
				
			for(i=0;i<225;i++)//50us RES
			{
				SPI1_ReadWriteByte(0x00);//0	   //B
			}


			for(i=0;i<60;i++)
			{
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G
			 
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G	
				SPI1_ReadWriteByte(0x30);//0	   //G
				SPI1_ReadWriteByte(0x30);//0	   //G	
				
				
				SPI1_ReadWriteByte(0x3C);//1    //G
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 
				SPI1_ReadWriteByte(0x3C);//1 

			}
   
        Delay_10us(50000);
   
}















