#include "bsp_systick.h"
#include "w25q128.h"
#include "bsp_spi.h"

//4KbytesΪһ��Sector
//16������Ϊ1��Block
//W25Q128
//����Ϊ16M�ֽ�,����128��Block,4096��Sector 


uint16_t W25QXX_TYPE=W25Q128;	//Ĭ����W25Q128
													 
//��ʼ��SPI FLASH��IO��
void W25QXX_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	W25QXX_CS = 1;
	SPI1_Init();
	SPI1_SetSpeed(SPI_BaudRatePrescaler_4);
	W25QXX_TYPE=W25QXX_ReadID();	//��ȡFLASH ID.
}


//��ȡоƬID
//����ֵ����:				   
//0XEF13,��ʾоƬ�ͺ�ΪW25Q80  
//0XEF14,��ʾоƬ�ͺ�ΪW25Q16    
//0XEF15,��ʾоƬ�ͺ�ΪW25Q32  
//0XEF16,��ʾоƬ�ͺ�ΪW25Q64 
//0XEF17,��ʾоƬ�ͺ�ΪW25Q128 	  
uint16_t W25QXX_ReadID(void)
{
	uint16_t Temp = 0;
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_ManufactDeviceID);
	SPI1_ReadWriteByte(0x00);
	SPI1_ReadWriteByte(0x00);
	SPI1_ReadWriteByte(0x00);
	Temp |= SPI1_ReadWriteByte(0xFF) << 8;
	Temp |= SPI1_ReadWriteByte(0xFF);
	W25QXX_CS = 1;
	return Temp;
}


//��ȡW25QXX��״̬�Ĵ���
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:Ĭ��0,״̬�Ĵ�������λ,���WPʹ��
//TB,BP2,BP1,BP0:FLASH����д��������
//WEL:дʹ������
//BUSY:æ���λ(1,æ;0,����)
//Ĭ��:0x00
uint8_t W25QXX_ReadSR(void)
{
	uint8_t status;
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_ReadStatusReg);
	status = SPI1_ReadWriteByte(0xFF);
	W25QXX_CS = 1;
	return status;
}


//дW25QXX״̬�Ĵ���
//ֻ��SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)����д!!!
void W25QXX_Write_SR(uint8_t sr)   
{
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_WriteStatusReg);
	SPI1_ReadWriteByte(sr);
	W25QXX_CS = 1;
}


//W25QXXдʹ��	
//��WEL��λ   
void W25QXX_Write_Enable(void)
{
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_WriteEnable);
	W25QXX_CS = 1;
}


//W25QXXд��ֹ	
//��WEL����  
void W25QXX_Write_Disable(void)
{
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_WriteDisable);
	W25QXX_CS = 1;
}


//�ȴ�����
void W25QXX_Wait_Busy(void)
{
	while ( (W25QXX_ReadSR() & 0x01) == 0x01){}
}


//��ȡSPI FLASH  
//��ָ����ַ��ʼ��ȡָ�����ȵ�����
//pBuffer:���ݴ洢��
//ReadAddr:��ʼ��ȡ�ĵ�ַ(24bit)
//NumByteToRead:Ҫ��ȡ���ֽ���(���65535)
void W25QXX_Read(uint8_t* pBuffer,u32 ReadAddr,uint16_t NumByteToRead)
{
	uint16_t i;
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_ReadData);
	SPI1_ReadWriteByte((uint8_t)(ReadAddr >> 16));
	SPI1_ReadWriteByte((uint8_t)(ReadAddr >> 8));
	SPI1_ReadWriteByte((uint8_t)(ReadAddr));
	for ( i = 0 ; i < NumByteToRead ; i ++)
	{
		pBuffer[i] = SPI1_ReadWriteByte(0xFF);
	}
	W25QXX_CS = 1;
}


//SPI��һҳ(0~65535)��д������256���ֽڵ�����
//��ָ����ַ��ʼд�����256�ֽڵ�����
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���256),������Ӧ�ó�����ҳ��ʣ���ֽ���!!!	 
void W25QXX_Write_Page(uint8_t* pBuffer,u32 WriteAddr,uint16_t NumByteToWrite)
{
	uint16_t i;
	W25QXX_Write_Enable();
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_PageProgram);
	SPI1_ReadWriteByte((uint8_t)(WriteAddr >> 16));
	SPI1_ReadWriteByte((uint8_t)(WriteAddr >> 8));
	SPI1_ReadWriteByte((uint8_t)(WriteAddr));
	
	for ( i = 0 ; i < NumByteToWrite ; i ++)
	{
		SPI1_ReadWriteByte(pBuffer[i]);
	}
	W25QXX_CS = 1;
	W25QXX_Wait_Busy();
}


//�޼���дSPI FLASH 
//����ȷ����д�ĵ�ַ��Χ�ڵ�����ȫ��Ϊ0XFF,�����ڷ�0XFF��д������ݽ�ʧ��!
//�����Զ���ҳ���� 
//��ָ����ַ��ʼд��ָ�����ȵ�����,����Ҫȷ����ַ��Խ��!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)
//NumByteToWrite:Ҫд����ֽ���(���65535)
//CHECK OK
void W25QXX_Write_NoCheck(uint8_t* pBuffer,u32 WriteAddr,uint16_t NumByteToWrite)
{
	uint16_t pageremain;
	pageremain = 256 - WriteAddr%256;
	if ( NumByteToWrite <= pageremain) pageremain = NumByteToWrite;
	while (1)
	{
		W25QXX_Write_Page(pBuffer,WriteAddr,pageremain);
		if ( NumByteToWrite == pageremain) break;
		else
		{
			pBuffer += pageremain;
			WriteAddr += pageremain;
			
			NumByteToWrite -= pageremain;
			
			if ( NumByteToWrite > 256 ) pageremain = 256;
			else
			{
				pageremain = NumByteToWrite;
			}
		}
	}
}


//����һ������
//Dst_Addr:������ַ ����ʵ����������
//����һ��ɽ��������ʱ��:150ms
void W25QXX_Erase_Sector(u32 Dst_Addr)
{
	Dst_Addr *= 4096;
	W25QXX_Write_Enable();
	W25QXX_Wait_Busy();
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_SectorErase);
	SPI1_ReadWriteByte((uint8_t)(Dst_Addr >> 16));
	SPI1_ReadWriteByte((uint8_t)(Dst_Addr >> 8));
	SPI1_ReadWriteByte((uint8_t)(Dst_Addr));
	W25QXX_CS = 1;
	W25QXX_Wait_Busy();
}



//дSPI FLASH  
//��ָ����ַ��ʼд��ָ�����ȵ�����
//�ú�������������!
//pBuffer:���ݴ洢��
//WriteAddr:��ʼд��ĵ�ַ(24bit)						
//NumByteToWrite:Ҫд����ֽ���(���65535)   
uint8_t W25QXX_BUFFER[4096];	
void W25QXX_Write(uint8_t* pBuffer,u32 WriteAddr,uint16_t NumByteToWrite)
{
	u32 secpos;
	uint16_t secoff;
	uint16_t secremain;
	uint16_t i;
	uint8_t* W25QXX_BUF;
	W25QXX_BUF = W25QXX_BUFFER;
	
	secpos = WriteAddr/4096;
	secoff = WriteAddr%4096;
	secremain = 4096 - secoff;
	
	if ( NumByteToWrite <= secremain) secremain = NumByteToWrite;
	
	while (1)
	{
		W25QXX_Read(W25QXX_BUF,secpos * 4096,4096);
		for ( i = 0; i < secremain ; i ++)
		{
			if (W25QXX_BUF[secoff + i] != 0xFF) break;
		}
		if ( i < secremain)
		{
			W25QXX_Erase_Sector(secpos);
			for ( i = 0 ; i < secoff ; i ++)
			{
				W25QXX_BUF[i + secoff] = pBuffer[i];
			}
			W25QXX_Write_NoCheck(W25QXX_BUF,secpos * 4096 , 4096);
		}
		else W25QXX_Write_NoCheck(pBuffer,WriteAddr,secremain);
		
		if ( NumByteToWrite == secremain) break;
		else
		{
			secpos ++;
			secoff = 0;
			
			pBuffer += secremain;
			WriteAddr += secremain;
			NumByteToWrite -= secremain;
			
			if ( NumByteToWrite > 4096) secremain = 4096;
			else secremain = NumByteToWrite;
		}

	}
}


//��������оƬ		  
void W25QXX_Erase_Chip(void)
{
	W25QXX_Write_Enable();
	W25QXX_Wait_Busy();
	
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_ChipErase);
	W25QXX_CS = 1;
	W25QXX_Wait_Busy();
}

//�������ģʽ
void W25QXX_PowerDown(void)   
{ 
  	W25QXX_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_PowerDown);        //���͵�������  
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
   // delay_us(3);                               //�ȴ�TPD  
	W25QXX_Delay(50);
}   
//����
void W25QXX_WAKEUP(void)   
{  
  	W25QXX_CS=0;                            //ʹ������   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
	W25QXX_CS=1;                            //ȡ��Ƭѡ     	      
    //delay_us(3);                               //�ȴ�TRES1
	W25QXX_Delay(350);
}   

void W25QXX_Delay(uint32_t cnt)
{
	while(cnt-->0);
}
