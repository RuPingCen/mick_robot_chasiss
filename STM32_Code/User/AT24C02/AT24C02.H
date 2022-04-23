#ifndef __AT24C02_H
#define	__AT24C02_H

#include "stm32f10x.h"

 
#define  AT24C02_EEPROM_ADDRESS     0xA0   /* E2 = 0 */
#define  I2C_PageSize               8	  	/* AT24C01/02每页有8个字节 *//* AT24C04/08A/16A每页有16个字节 */






extern void AT24C02_I2C_BufferRead(I2C_TypeDef* I2Cx,u8* pBuffer, u8 ReadAddr, u16 NumByteToRead);
extern void AT24C02_I2C_WaitEepromStandbyState(void); 
extern void AT24C02_I2C_BufferWrite(I2C_TypeDef* I2Cx,u8* pBuffer, u8 WriteAddr, u16 NumByteToWrite);
 
 
 
 
#endif  

/***************************AT24C02测试函数***************************************************

void AT24C02_Test(void)
{
	unsigned int i=0;
  UART_send_string(USART1,"写入的数据\n");//字符串函数  
	for (i=0; i<256; i++ ) //填充缓冲
  {   
      I2c_Buf_Write[i] = 68;
  
   }
   UART_send_string(USART1,I2c_Buf_Write);//字符串函数  
  //将I2c_Buf_Write中顺序递增的数据写入EERPOM中 
 
  AT24C02_I2C_BufferWrite(I2C1,I2c_Buf_Write,0x64,7); 
  UART_send_string(USART1,"\n写成功\n");//字符串函数 
	UART_send_string(USART1,"\n读出的数据\n");//字符串函数 
  //将EEPROM读出数据顺序保持到I2c_Buf_Read中
  I2C_STM32_Read_Bytes(I2C1,AT24C02_EEPROM_ADDRESS,0x64,I2c_Buf_Read,7); 
  //将I2c_Buf_Read中的数据通过串口打印
	for (i=0; i<7; i++)
	{	
		
		if(I2c_Buf_Read[i] != I2c_Buf_Write[i])
		{			
      UART_send_string(USART1,"错误:");//字符串函数  
      UART_send_char(USART1,i+48); 		
			UART_send_string(USART1,"错误:I2C EEPROM写入与读出的数据不一致\n");//字符串函数  
			return;
		}
     
	}
  UART_send_string(USART1,I2c_Buf_Read);//字符串函数   
	UART_send_string(USART1,"\nI2C(AT24C02)读写测试成功\n");//字符串函数  
}

******************************************************************************/
