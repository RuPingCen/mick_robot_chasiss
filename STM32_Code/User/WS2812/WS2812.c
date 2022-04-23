#include "stm32f10x_sdio.h"  	
#include "stm32f10x_Delay.h"


#include "stm32f10x_spi.h"
#include "stm32f10x_rcc.h"


#include "WS2812.h"

#define Length_WS2812_datas 180
unsigned char WS2812_datas[180][3];  // �����ڶ���Ƶ������ÿ�ζ��������������ˢ��
unsigned char WS2812_light_flag =0x00;
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//SPI ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2010/6/13 
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  
 

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ������SD Card/W25X16/24L01/JF24C							  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI1�ĳ�ʼ��



void SPI1_Init(void)
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
  
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA|RCC_APB2Periph_SPI1, ENABLE );	
 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
  
 	GPIO_SetBits(GPIOA,GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7);

	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //����SPI�������˫�������ģʽ:SPI����Ϊ˫��˫��ȫ˫��
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//����SPI����ģʽ:����Ϊ��SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//����SPI�����ݴ�С:SPI���ͽ���8λ֡�ṹ
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;		//ѡ���˴���ʱ�ӵ���̬:ʱ�����ո�
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;	//���ݲ����ڵڶ���ʱ����
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS�ź���Ӳ����NSS�ܽţ����������ʹ��SSIλ������:�ڲ�NSS�ź���SSIλ����
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;		//���岨����Ԥ��Ƶ��ֵ:������Ԥ��ƵֵΪ256
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//ָ�����ݴ����MSBλ����LSBλ��ʼ:���ݴ����MSBλ��ʼ
	SPI_InitStructure.SPI_CRCPolynomial = 10;	//CRCֵ����Ķ���ʽ
	SPI_Init(SPI1, &SPI_InitStructure);  //����SPI_InitStruct��ָ���Ĳ�����ʼ������SPIx�Ĵ���
 
	SPI_Cmd(SPI1, ENABLE); //ʹ��SPI����
	
	//SPI1_ReadWriteByte(0xff);//��������		 
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
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;//ȫ˫��������ʽ
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;//��������ʱ�̵ĵ�ƽ(Ҳ�ǳ�ʼ����ƽ)
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;//����������λ   
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;  
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;//SPI��Ƶϵ��
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 10;//SPI CRCЧ��
	SPI_Init(SPI2, &SPI_InitStructure);


	SPI_Cmd(SPI2, ENABLE);
	//SPI2_ReadWriteByte(0xff);//��������	
} 
//SPI �ٶ����ú���
//SpeedSet:
//SPI_BaudRatePrescaler_2   2��Ƶ   (SPI 36M@sys 72M)
//SPI_BaudRatePrescaler_8   8��Ƶ   (SPI 9M@sys 72M)
//SPI_BaudRatePrescaler_16  16��Ƶ  (SPI 4.5M@sys 72M)
//SPI_BaudRatePrescaler_256 256��Ƶ (SPI 281.25K@sys 72M)
  
 

//SPIx ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI1_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
	
    
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
            retry++;
            if(retry>200)return 0;
	}			  
    
	SPI_I2S_SendData(SPI1, TxData); //ͨ������SPIx����һ������		
  SPI2_ReadWriteByte(TxData);
  return 0x00;	
}
u8 SPI2_ReadWriteByte(u8 TxData)
{		
	u8 retry=0;				 	
    
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET) //���ָ����SPI��־λ�������:���ͻ���ձ�־λ
	{
            retry++;
            if(retry>200)return 0;
	}			  
    
	SPI_I2S_SendData(SPI2, TxData); //ͨ������SPIx����һ������		
  return 0x00;	
}


void WS2812_RGB_Init(void)
{
	SPI1_Init();		   //��ʼ��SPI	
	SPI2_Init();		   //��ʼ��SPI
  WS2812_RGB_Rest();
}

// ��λ
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
		if((dat_color<<j)&0x80) //�ȷ���λ
			SPI1_ReadWriteByte(0x3C);//1 0x3C   //R
		else
			SPI1_ReadWriteByte(0x30);//0	0x30
	
	}
		 
}


// ��ɫ�ĵ� 
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
// �ò�ͬ�ĵ���ʾ��ͬ����ɫ
// ˼·�������еƵ���ɫֵ�洢��һ��ȫ�ֱ����У�Ȼ��ÿ�θı�ı����еĶ�Ӧ��
// ����λ�õ�ֵ�� 
// ÿ��д�붼�����еĵƽ��в���
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
	else if(WS2812_light_flag == 0x01) //��������
	{
		  if(times_count > 8) //������Ч�� �ɶ�ʱ������
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
	else if(WS2812_light_flag == 0x02) //�����䰵
	{
		  if(times_count <100) //������Ч�� �ɶ�ʱ������
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



// ��ɫ�ĵ�
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
// ��ɫ�ĵ�
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
// ��ɫ�ĵ�
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
// ��ɫ�ĵ�
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



// ��ɫ�ĵ�
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















