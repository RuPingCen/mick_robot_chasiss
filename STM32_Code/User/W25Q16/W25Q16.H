#ifndef __W25Q16_H
#define	__W25Q16_H

#include "stm32f10x.h"

/* Private typedef -----------------------------------------------------------*/
//#define SPI_FLASH_PageSize      4096
#define SPI_FLASH_PageSize              256
#define SPI_FLASH_PerWritePageSize      256

/**************************命令列表***********************************************/
#define W25X_WriteEnable		      0x06 
#define W25X_WriteDisable		      0x04 
#define W25X_ReadStatusReg		    0x05 //W25X_ReadStatusReg
#define W25X_WriteStatusReg		    0x01 
#define W25X_ReadData			        0x03 
#define W25X_FastReadData		      0x0B 
#define W25X_FastReadDual		      0x3B 
#define W25X_PageProgram		      0x02 
#define W25X_BlockErase			      0xD8 
#define W25X_SectorErase		      0x20 
#define W25X_ChipErase			      0xC7  
#define W25X_PowerDown			      0xB9 
#define W25X_ReleasePowerDown	    0xAB 
#define W25X_DeviceID			        0xAB 
#define W25X_ManufactDeviceID   	0x90 
#define W25X_JedecDeviceID		    0x9F 

#define Busy_Flag                 0x01   //存储器繁忙位
#define Dummy_Byte                0xFF 
 
 

//---------------------W25Q16片选信号-----------------------------------//
#define W25Q16_FLASH_CS_LOW()       GPIO_ResetBits(GPIOA, GPIO_Pin_4)
#define W25Q16_FLASH_CS_HIGH()      GPIO_SetBits(GPIOA, GPIO_Pin_4)
 
 
 
 
 
 
 
 
/*********************************************************************/
extern void W25Q16_FLASH_WriteEnable(SPI_TypeDef* SPIx); 
extern void W25Q16_Flash_WAKEUP(SPI_TypeDef* SPIx);  
extern void W25Q16_Flash_PowerDown(SPI_TypeDef* SPIx);
extern u32 W25Q16_FLASH_ReadDeviceID(SPI_TypeDef* SPIx);
extern u32 W25Q16_FLASH_ReadID(SPI_TypeDef* SPIx);
extern void W25Q16_FLASH_BufferRead(SPI_TypeDef* SPIx,u8* pBuffer, u32 ReadAddr, u16 NumByteToRead);
extern void W25Q16_FLASH_BufferWrite(SPI_TypeDef* SPIx,u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
extern void W25Q16_FLASH_PageWrite(SPI_TypeDef* SPIx, u8* pBuffer, u32 WriteAddr, u16 NumByteToWrite);
extern void W25Q16_FLASH_ChipErase(SPI_TypeDef* SPIx);
extern void W25Q16_FLASH_SectorErase(SPI_TypeDef* SPIx,u32 SectorAddr);
extern u8 W25Q16_FLASH_SendByte(SPI_TypeDef* SPIx,u8 byte);
extern void W25Q16_FLASH_WaitForWriteEnd(SPI_TypeDef* SPIx);
extern void W25Q16_FLASH_Init(SPI_TypeDef* SPIx);


#endif   

/***************************W25Q16测试函数***************************************************
int main(void)
{	
  //unsigned char key_value;
	  unsigned int i=0;
 
		 SysTick_Init();  
	
     W25Q16_FLASH_Init(SPI1);
	   My_Config_USART_Init(USART1,115200,1);
	  
	   // I2C_STM32_Config_Init(I2C1,400000);//iic 初始化函数
     // MPU6050_Init(I2C1);//MPU6050初始化 ± 2000 °/s ± 4g  5hz	  
     //UART_send_string(USART1,"IIC测试\n");//字符串函数
	   
      // EnableInterrupt;
  
	  UART_send_string(USART1,"\n 这是一个2M串行flash(W25X16)实验 \n");//字符串函数
		//Get SPI Flash Device ID //
		UART_send_string(USART1,"DeviceID:");//字符串函数
		UART_send_data(USART1,W25Q16_FLASH_ReadDeviceID(SPI1));  // 0~4294967296
		Delay(200);
		
		// Get SPI Flash ID //
		UART_send_string(USART1,"\nDeviceID:");//字符串函数
		UART_send_data(USART1,W25Q16_FLASH_ReadID(SPI1));  // 0~4294967296
		// Check the SPI Flash ID //
		if (W25Q16_FLASH_ReadID(SPI1) == 0xEF4015)  // #define  sFLASH_ID  0xEF3015 //
		{	
				 
				UART_send_string(USART1,"\n 检测到华邦串行flash W25X16 !\n");//字符串函数
				
				W25Q16_FLASH_ChipErase(SPI1);	 // Erase W25Q16 FLASH Sector to write on //	 
			  Delay(2000);
			
			
			// 将发送缓冲区的数据写到flash中 //
			W25Q16_FLASH_BufferWrite(SPI1,Tx_Buffer,0x00, number_test); 	 
		 	UART_send_string(USART1,"\n 写入的数据为");//字符串函数
		 	UART_send_string(USART1,Tx_Buffer);//字符串函数
			
			Delay(200);
			
	    // 将刚刚写入的数据读出来放到接收缓冲区中 //
			W25Q16_FLASH_BufferRead(SPI1,Rx_Buffer,0x00,number_test);		
			UART_send_string(USART1,"\n 读出的数据为：");//字符串函数
			UART_send_string(USART1,Rx_Buffer);//字符串函数
		
			
			// 检查写入的数据与读出的数据是否相等 //
			for(i=0; i < number_test ; i++)
			{
					if( Tx_Buffer[i] != Rx_Buffer[i] )
					{    
						UART_send_string(USART1,"\n 2M串行flash(W25X16)测试失败!\n");//字符串函数
					} 
		  }
		 
		} 
	  else
			{    
				UART_send_string(USART1,"\n 获取不到 W25X16 ID!\n");//字符串函数
			}
	
		W25Q16_Flash_PowerDown(SPI1); 
		UART_send_string(USART1,"\n 2M串行flash(W25X16)测试完毕!\n");//字符串函数
		
	while (1)
	{	       	
	}

}
 

******************************************************************************/
