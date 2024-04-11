#include "bsp_systick.h"
#include "w25q128.h"
#include "bsp_spi.h"

//4Kbytes为一个Sector
//16个扇区为1个Block
//W25Q128
//容量为16M字节,共有128个Block,4096个Sector 


uint16_t W25QXX_TYPE=W25Q128;	//默认是W25Q128
													 
//初始化SPI FLASH的IO口
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
	W25QXX_TYPE=W25QXX_ReadID();	//读取FLASH ID.
}


//读取芯片ID
//返回值如下:				   
//0XEF13,表示芯片型号为W25Q80  
//0XEF14,表示芯片型号为W25Q16    
//0XEF15,表示芯片型号为W25Q32  
//0XEF16,表示芯片型号为W25Q64 
//0XEF17,表示芯片型号为W25Q128 	  
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


//读取W25QXX的状态寄存器
//BIT7  6   5   4   3   2   1   0
//SPR   RV  TB BP2 BP1 BP0 WEL BUSY
//SPR:默认0,状态寄存器保护位,配合WP使用
//TB,BP2,BP1,BP0:FLASH区域写保护设置
//WEL:写使能锁定
//BUSY:忙标记位(1,忙;0,空闲)
//默认:0x00
uint8_t W25QXX_ReadSR(void)
{
	uint8_t status;
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_ReadStatusReg);
	status = SPI1_ReadWriteByte(0xFF);
	W25QXX_CS = 1;
	return status;
}


//写W25QXX状态寄存器
//只有SPR,TB,BP2,BP1,BP0(bit 7,5,4,3,2)可以写!!!
void W25QXX_Write_SR(uint8_t sr)   
{
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_WriteStatusReg);
	SPI1_ReadWriteByte(sr);
	W25QXX_CS = 1;
}


//W25QXX写使能	
//将WEL置位   
void W25QXX_Write_Enable(void)
{
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_WriteEnable);
	W25QXX_CS = 1;
}


//W25QXX写禁止	
//将WEL清零  
void W25QXX_Write_Disable(void)
{
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_WriteDisable);
	W25QXX_CS = 1;
}


//等待空闲
void W25QXX_Wait_Busy(void)
{
	while ( (W25QXX_ReadSR() & 0x01) == 0x01){}
}


//读取SPI FLASH  
//在指定地址开始读取指定长度的数据
//pBuffer:数据存储区
//ReadAddr:开始读取的地址(24bit)
//NumByteToRead:要读取的字节数(最大65535)
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


//SPI在一页(0~65535)内写入少于256个字节的数据
//在指定地址开始写入最大256字节的数据
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大256),该数不应该超过该页的剩余字节数!!!	 
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


//无检验写SPI FLASH 
//必须确保所写的地址范围内的数据全部为0XFF,否则在非0XFF处写入的数据将失败!
//具有自动换页功能 
//在指定地址开始写入指定长度的数据,但是要确保地址不越界!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)
//NumByteToWrite:要写入的字节数(最大65535)
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


//擦除一个扇区
//Dst_Addr:扇区地址 根据实际容量设置
//擦除一个山区的最少时间:150ms
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



//写SPI FLASH  
//在指定地址开始写入指定长度的数据
//该函数带擦除操作!
//pBuffer:数据存储区
//WriteAddr:开始写入的地址(24bit)						
//NumByteToWrite:要写入的字节数(最大65535)   
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


//擦除整个芯片		  
void W25QXX_Erase_Chip(void)
{
	W25QXX_Write_Enable();
	W25QXX_Wait_Busy();
	
	W25QXX_CS = 0;
	SPI1_ReadWriteByte(W25X_ChipErase);
	W25QXX_CS = 1;
	W25QXX_Wait_Busy();
}

//进入掉电模式
void W25QXX_PowerDown(void)   
{ 
  	W25QXX_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_PowerDown);        //发送掉电命令  
	W25QXX_CS=1;                            //取消片选     	      
   // delay_us(3);                               //等待TPD  
	W25QXX_Delay(50);
}   
//唤醒
void W25QXX_WAKEUP(void)   
{  
  	W25QXX_CS=0;                            //使能器件   
    SPI1_ReadWriteByte(W25X_ReleasePowerDown);   //  send W25X_PowerDown command 0xAB    
	W25QXX_CS=1;                            //取消片选     	      
    //delay_us(3);                               //等待TRES1
	W25QXX_Delay(350);
}   

void W25QXX_Delay(uint32_t cnt)
{
	while(cnt-->0);
}
