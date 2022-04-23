/******************************************************************************************************
  *
  * 函数名：W25Q16存储器使用相关函数
  *       
  * 备注：      SPIx (SPI1 SPI2 SPI3) spi模块号
  *             pBuffer               数据首地址
  *             WriteAddr 、ReadAddr   24位存放地址
  *             NumByteToWrite         写入数据的个数  
  * 2014-11-2
		
  W25Q16_FLASH_Init(SPI_TypeDef* SPIx);//存储器初始化函数
	W25Q16_FLASH_BufferRead(SPI_TypeDef* SPIx,u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);//数据读函数
  W25Q16_FLASH_BufferWrite(SPI_TypeDef* SPIx,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);//数据写函数
	
	u32 W25Q16_FLASH_ReadID(SPI_TypeDef* SPIx);//读存储器ID函数	  ( W25Q16 ID号== 0xEF4015 )
	W25Q16_FLASH_ChipErase(SPI_TypeDef* SPIx);//存储器整片擦除
	
	W25Q16_Flash_WAKEUP(SPI_TypeDef* SPIx);//从掉电模式下唤醒
  W25Q16_Flash_PowerDown(SPI_TypeDef* SPIx);//掉电模式
	
	
	注： // W25X16: data input on the DIO pin is sampled on the rising edge of the CLK. (W25X16 在时钟上升沿采样数据线上的数据)
       // Data on the DO and DIO pins are clocked out on the falling edge of CLK.(W25X16 下降沿输出数据)
			 
   这与SPI在设置工作模式时候有关   用IO口模拟SPi时序的时候同样要注意这个问题
	
*******************************************************************************************************/
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h" 
#include "stm32f10x_spi.h" 


#include "W25Q16.h"



/*************************************************************************
*  函数名称：W25Q16_FLASH_SendByte
*  功能说明：W25Q16 通过SPI写入一个字节  
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*             
*  函数返回：返回 SPI总线上的回传数据
*  修改时间：2014-11-2
*  备    注：CRP  这样写主要是方便程序移植
*************************************************************************/
u8 W25Q16_FLASH_SendByte(SPI_TypeDef* SPIx,u8 byte)
{
   return My_STM32_SP_FLASH_SendByte(SPIx,byte);
}
/*************************************************************************
*  函数名称：W25Q16_FLASH_Init(SPI_TypeDef* SPIx)
*  功能说明：W25Q16 存储器初始化函数
*              
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*             
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP
*************************************************************************/ 
void W25Q16_FLASH_Init(SPI_TypeDef* SPIx)
{
	   GPIO_InitTypeDef GPIO_InitStructure;
	/***************************************************/
	   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//初始化 W25Q16 片选引脚
		 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
		 GPIO_Init(GPIOA, &GPIO_InitStructure);
		/***************************************************/
     My_STM32_SPI_Init(SPI1);//SPI模块初始化

}
 
 
/*************************************************************************
*  函数名称：W25Q16_FLASH_WaitForWriteEnd
*  功能说明： Polls the status of the Write In Progress (WIP) flag in the
*             FLASH's status  register  and  loop  until write  opertaion
*             has completed.
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*             
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP
*************************************************************************/
void W25Q16_FLASH_WaitForWriteEnd(SPI_TypeDef* SPIx)
{
  u8 FLASH_Status = 0;

  
  W25Q16_FLASH_CS_LOW();/* Select the FLASH: Chip Select low */
  W25Q16_FLASH_SendByte(SPIx, W25X_ReadStatusReg);/* Send "Read Status Register" instruction */
	
  /* Loop as long as the memory is busy with a write cycle */
  do
  {
    /* Send a dummy byte to generate the clock needed by the FLASH
    and put the value of the status register in FLASH_Status variable */
    FLASH_Status = W25Q16_FLASH_SendByte(SPIx, Dummy_Byte);	 
  }
  while ((FLASH_Status & Busy_Flag) == 1);  

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();
}



 /*******************************************************************************
* Function Name  : SPI_FLASH_SectorErase
* Description    : Erases the specified FLASH sector. (4K-bytes)
* Input          : SectorAddr: address of the sector to erase.
* Output         : None
* Return         : None 擦除一个大小为4KB的数据块 
* W25Q16 还支持 32KB 64KB 块的擦写   命令分别是 52h  D8h
*******************************************************************************/
void W25Q16_FLASH_SectorErase(SPI_TypeDef* SPIx,u32 SectorAddr)
{
  /* Send write enable instruction */
  W25Q16_FLASH_WriteEnable(SPIx);
  W25Q16_FLASH_WaitForWriteEnd(SPIx);
  /* Sector Erase */
  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();
  /* Send Sector Erase instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_SectorErase);
  /* Send SectorAddr high nibble address byte */
  W25Q16_FLASH_SendByte(SPIx,(SectorAddr & 0xFF0000) >> 16);
  /* Send SectorAddr medium nibble address byte */
  W25Q16_FLASH_SendByte(SPIx,(SectorAddr & 0xFF00) >> 8);
  /* Send SectorAddr low nibble address byte */
  W25Q16_FLASH_SendByte(SPIx,SectorAddr & 0xFF);
  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();
  /* Wait the end of Flash writing */
  W25Q16_FLASH_WaitForWriteEnd(SPIx);
}

/*************************************************************************
*  函数名称：SPI_FLASH_ChipErase
*  功能说明：W25Q16 所有数据擦除
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*             
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP
*************************************************************************/
void W25Q16_FLASH_ChipErase(SPI_TypeDef* SPIx)
{
  /* Send write enable instruction */
  W25Q16_FLASH_WriteEnable(SPIx);

  /* Chip Erase */
  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();
  /* Send Bulk Erase instruction  */
  W25Q16_FLASH_SendByte(SPIx,W25X_ChipErase);
  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  W25Q16_FLASH_WaitForWriteEnd(SPIx);
}

 
/*************************************************************************
*  函数名称：W25Q16_FLASH_PageWrite
*  功能说明：W25Q16 页编写指令
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*             pBuffer 数据首地址
*             WriteAddr 24位存放地址
*             NumByteToWrite  写入数据的个数  （1 ~ 256） 
*
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP  内部调用
*************************************************************************/
void W25Q16_FLASH_PageWrite(SPI_TypeDef* SPIx, u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
  /* Enable the write access to the FLASH */
 W25Q16_FLASH_WriteEnable(SPIx);
 
  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();
  /* Send "Write to Memory " instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_PageProgram);
  /* Send WriteAddr high nibble address byte to write to */
  W25Q16_FLASH_SendByte(SPIx,(WriteAddr & 0xFF0000) >> 16);
  /* Send WriteAddr medium nibble address byte to write to */
  W25Q16_FLASH_SendByte(SPIx,(WriteAddr & 0xFF00) >> 8);
  /* Send WriteAddr low nibble address byte to write to */
  W25Q16_FLASH_SendByte(SPIx,(WriteAddr & 0xFF));

  if(NumByteToWrite > SPI_FLASH_PerWritePageSize)
  {
     NumByteToWrite = SPI_FLASH_PerWritePageSize;
     //printf("\n\r Err: SPI_FLASH_PageWrite too large!");
  }

  /* while there is data to be written on the FLASH */
  while (NumByteToWrite--)
  {
    /* Send the current byte */
    W25Q16_FLASH_SendByte(SPIx,*pBuffer);
    /* Point on the next byte to be written */
    pBuffer++;
  }

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();

  /* Wait the end of Flash writing */
  W25Q16_FLASH_WaitForWriteEnd(SPIx);
}

/*************************************************************************
*  函数名称：W25Q16_FLASH_BufferWrite(SPI_TypeDef* SPIx,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
*  功能说明：W25Q16 写数据函数
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*             pBuffer 数据首地址
*             WriteAddr 24位存放起始地址 
*             NumByteToWrite  写入数据的个数   
*
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP
*************************************************************************/
void W25Q16_FLASH_BufferWrite(SPI_TypeDef* SPIx,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite)
{
		u8 NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

		Addr = WriteAddr % SPI_FLASH_PageSize;
		count = SPI_FLASH_PageSize - Addr;
		NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
		NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

		if (Addr == 0) /* WriteAddr is SPI_FLASH_PageSize aligned  */
		{
				if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
				{
						W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, NumByteToWrite);
				}
				else /* NumByteToWrite > SPI_FLASH_PageSize */
				{
						while (NumOfPage--)
						{
							W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, SPI_FLASH_PageSize);
							WriteAddr +=  SPI_FLASH_PageSize;
							pBuffer += SPI_FLASH_PageSize;
						}
						if(NumOfSingle != 0)
						 W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, NumOfSingle);
				}
		}
		else /* WriteAddr is not SPI_FLASH_PageSize aligned  */
		{
					if (NumOfPage == 0) /* NumByteToWrite < SPI_FLASH_PageSize */
					{
							if (NumOfSingle > count) /* (NumByteToWrite + WriteAddr) > SPI_FLASH_PageSize */
							{
									temp = NumOfSingle - count;

									W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, count);
									WriteAddr +=  count;
									pBuffer += count;

									W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, temp);
							}
							else
							{
									W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, NumByteToWrite);
							}
				 }
				else /* NumByteToWrite > SPI_FLASH_PageSize */
				{
						NumByteToWrite -= count;
						NumOfPage =  NumByteToWrite / SPI_FLASH_PageSize;
						NumOfSingle = NumByteToWrite % SPI_FLASH_PageSize;

						W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, count);
						WriteAddr +=  count;
						pBuffer += count;

						while (NumOfPage--)
						{
							W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, SPI_FLASH_PageSize);
							WriteAddr +=  SPI_FLASH_PageSize;
							pBuffer += SPI_FLASH_PageSize;
						}

						if (NumOfSingle != 0)
						{
							W25Q16_FLASH_PageWrite(SPIx,pBuffer, WriteAddr, NumOfSingle);
						}
				}
		}
}
/*************************************************************************
*  函数名称： W25Q16_FLASH_BufferRead(SPI_TypeDef* SPIx,u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
*  功能说明： W25Q16 读数据函数
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*             pBuffer 数据首地址
*             ReadAddr 24位存放起始地址 
*             NumByteToRead  需要读出多少个数据
*
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP     W25Q16每次读取一个数据后内部地址会自增一
*************************************************************************/
void W25Q16_FLASH_BufferRead(SPI_TypeDef* SPIx,u8* pBuffer, u32 ReadAddr, u16 NumByteToRead)
{
  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();

  /* Send "Read from Memory " instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_ReadData);
   
  /* Send ReadAddr high nibble address byte to read from */
  W25Q16_FLASH_SendByte(SPIx,(ReadAddr & 0xFF0000) >> 16);
  /* Send ReadAddr medium nibble address byte to read from */
  W25Q16_FLASH_SendByte(SPIx,(ReadAddr& 0xFF00) >> 8);
  /* Send ReadAddr low nibble address byte to read from */
  W25Q16_FLASH_SendByte(SPIx,ReadAddr & 0xFF);

  while (NumByteToRead--) /* while there is data to be read */
  {
			/* Read a byte from the FLASH */
			*pBuffer = W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);
			/* Point to the next location where the byte read will be saved */
			pBuffer++;  
  }

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();
}
/*************************************************************************
*  函数名称： W25Q16_FLASH_ReadID(SPI_TypeDef* SPIx)
*  功能说明： W25Q16 读ID 函数
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*              
*  函数返回：返回一个24位ID号
*  修改时间：2014-11-2
*  备    注：CRP      
*************************************************************************/
u32 W25Q16_FLASH_ReadID(SPI_TypeDef* SPIx)
{
  u32 Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;

  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_JedecDeviceID);

  /* Read a byte from the FLASH */
  Temp0 = W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp1 = W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);

  /* Read a byte from the FLASH */
  Temp2 = W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();

  Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

  return Temp;
}
/*************************************************************************
*  函数名称： W25Q16_FLASH_ReadID(SPI_TypeDef* SPIx)
*  功能说明： W25Q16 读ID 函数
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*              
*  函数返回：返回一个8位ID号 
*  修改时间：2014-11-2
*  备    注：CRP  When used only to obtain the Device ID while not in the power-down state   
*  当W25Q16当前没有处于低功耗、高性能模式下的时候才具有读ID的功能
*************************************************************************/
u32 W25Q16_FLASH_ReadDeviceID(SPI_TypeDef* SPIx)
{
  u32 Temp = 0;

  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();

  /* Send "RDID " instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_DeviceID);
  W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);
  W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);
  W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);
  
  /* Read a byte from the FLASH */
  Temp = W25Q16_FLASH_SendByte(SPIx,Dummy_Byte);

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();

  return Temp;
}
 
/*************************************************************************
*  函数名称： W25Q16_FLASH_WriteEnable(SPI_TypeDef* SPIx)
*  功能说明： W25Q16 写使能函数
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*              
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP      
*************************************************************************/
void W25Q16_FLASH_WriteEnable(SPI_TypeDef* SPIx)
{
  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();

  /* Send "Write Enable" instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_WriteEnable);

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();
}



/*************************************************************************
*  函数名称： W25Q16_Flash_PowerDown(SPI_TypeDef* SPIx)
*  功能说明： W25Q16 进入掉电模式
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*              
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP   
*************************************************************************/
void W25Q16_Flash_PowerDown(SPI_TypeDef* SPIx)   
{ 
  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_PowerDown);

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();
}   
/*************************************************************************
*  函数名称： SPI_Flash_PowerDown(SPI_TypeDef* SPIx)
*  功能说明： W25Q16 从掉电模式下 唤醒
*  参数说明： SPIx (SPI1 SPI2 SPI3)
*              
*  函数返回：无
*  修改时间：2014-11-2
*  备    注：CRP   W25Q16存储器唤醒的前提下 W25Q16必须处于掉电模式       
*************************************************************************/
 
void W25Q16_Flash_WAKEUP(SPI_TypeDef* SPIx)   
{
  /* Select the FLASH: Chip Select low */
  W25Q16_FLASH_CS_LOW();

  /* Send "Power Down" instruction */
  W25Q16_FLASH_SendByte(SPIx,W25X_ReleasePowerDown);

  /* Deselect the FLASH: Chip Select high */
  W25Q16_FLASH_CS_HIGH();                   //等待TRES1
}   
   
/*********************************************END OF FILE**********************/
