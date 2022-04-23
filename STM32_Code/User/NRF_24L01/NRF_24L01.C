#include "stm32f10x.h"
 
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_exti.h" 
 
 
 

#include "bsp_uart.h"
#include "NRF_24L01.h"




static unsigned char TX_ADDRESS[TX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//本地地址
static unsigned char RX_ADDRESS[RX_ADR_WIDTH]= {0x34,0x43,0x10,0x10,0x01};	//接收地址

unsigned char NRF24l01_TX_BUFF[TX_PLOAD_WIDTH]={0};	 // 初始化发送与接收缓冲区
unsigned char NRF24l01_RX_BUFF[RX_PLOAD_WIDTH]={0};	

 
void NRF24l01_GPIO_Init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOG,ENABLE);
		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
		GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;   
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_5   | GPIO_Pin_7 ;                                      
		GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_15 |GPIO_Pin_8;                                       
		GPIO_Init(GPIOG, &GPIO_InitStruct);
 

		GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
		GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6 ;                                      
		GPIO_Init(GPIOA, &GPIO_InitStruct);

}





/**************延时函数*******************************/
void NRF24L01_Delayus(unsigned int n)
{
	n=n*10;
	while(n-- >0 );
}
/****************************************************************************************
*NRF24L01初始化
***************************************************************************************/
void init_NRF24L01(void)
{
	 UART_send_string(USART1,"NRF24L01开始初始化\n");//字符串函数
	NRF24l01_GPIO_Init();
	NRF24L01_Delayus(5000);
 	CE_Out_L;    // chip enable  无数据传送
 	CSN_Out_H;  // Spi  disable   CSN为低后SPI接口等待执行命令 
	SCK_Out_H;
  //IRQ=1;
	
	SPI_Write_Buf(NRF_24l01_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // 写本地地址、发送地址	
	SPI_Write_Buf(NRF_24l01_WRITE_REG + RX_ADDR_P0, RX_ADDRESS, RX_ADR_WIDTH); // 写接收端0通道地址	
	SPI_RW_Reg(NRF_24l01_WRITE_REG + EN_AA, 0x00);      //  频道0无ACK应答 	
	SPI_RW_Reg(NRF_24l01_WRITE_REG + EN_RXADDR, 0x01);  //  允许接收地址只有频道0
	SPI_RW_Reg(NRF_24l01_WRITE_REG + SETUP_AW, 0x03);      //  设置接收、发送的地址长度为5字节	
	//SPI_RW_Reg(NRF_24l01_WRITE_REG + SETUP_RETR, 0x00);      // 禁止重发
	SPI_RW_Reg(NRF_24l01_WRITE_REG + RF_CH,40);        //   设置信道工作 频率
	SPI_RW_Reg(NRF_24l01_WRITE_REG + RF_SETUP, 0x0f);   		//设置发射速率为2MHZ，发射功率为最大值0dB	
	SPI_RW_Reg(NRF_24l01_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度	
  SPI_RW_Reg(NRF_24l01_WRITE_REG + CONFIG, 0x0e);              // CRC使能，16位CRC校验，上电，接收模式 允许接收中断、发送中断
	CE_Out_H;
	 UART_send_string(USART1,"NRF24L01初始化完毕\n");//字符串函数
}
/***********************************************************************
*函数：uint SPI_RW(uint uchar)
*功能：NRF24L01的SPI写时序
******************************************************************/
unsigned char SPI_RW(unsigned char dat)
{
		unsigned char i;  
/**********Description:**************************************
  Writes one byte to nRF24L01, and return the byte read from nRF24L01 during write, according to SPI protocol
  假设下面的8位寄存器装的是待发送的数据10101010，上升沿发送、下降沿接收、高位先发送。
  那么第一个上升沿来的时候 数据将会是sdo=1；寄存器中的10101010左移一位，后面补入送来的一位未知数x，成了0101010x。
  下降沿到来的时候，sdi上的电平将锁存到寄存器中去，那么这时寄存器=0101010sdi，
  这样在 8个时钟脉冲以后，两个寄存器的内容互相交换一次。这样就完成里一个spi时序
	************************************************************/
   	for(i=0;i<8;i++) // output 8-bit data
   	{			
        if(dat & 0x80)
				   MOSI_Out_H;
			  else
					  MOSI_Out_L;
			  
				dat = (dat << 1);           // shift next bit into MSB..
				SCK_Out_H;                      // Set SCK high..
			//单片机为主机 24l01为从机    主机MISO接从机MISO	//CLK上升沿为主机发送数据     clk下降沿为从机发送数据
				if(MISO)
				dat ++;       		  // capture current MISO bit 在MISO上捕捉当前位			
				SCK_Out_L;            		  // ..then set SCK low again	
		}
		return(dat);           		  // return read data
}
//**************************************************************************/
//*功能：NRF24L01读写寄存器函数
//****************************************************************/
unsigned char SPI_RW_Reg(unsigned char reg, unsigned char value)
{
	unsigned char status;	
	CSN_Out_L;                   // CSN low, init SPI transaction
	status = SPI_RW(reg);      // select register
	SPI_RW(value);             // ..and write value to it..
	CSN_Out_H;                   // CSN high again	
	return(status);            // return nRF24L01 status uchar
}
//*************************************************************************
//*函数： unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char uchars)
//*功能: 用于写数据：为寄存器地址，pBuf：为待写入数据地址，uchars：写入数据的个数
//************************************************************/
unsigned char SPI_Write_Buf(unsigned char reg, unsigned char *pBuf, unsigned char length)
{	
	unsigned char status,i;
	CSN_Out_L;            //CSN置低 开始传输数据    
	status = SPI_RW(reg);  
 //inerDelay_us(10);	
	for(i=0; i<length; i++) //
		SPI_RW(*pBuf++);
	CSN_Out_H;           //关闭SPI
	return(status);    // 
	
}
//******************************************************************************
//*函数：uchar SPI_Read(uchar reg)
//*功能：NRF24L01的SPI时序
//********************************************************************/
unsigned char SPI_Read(unsigned char reg)
{
	unsigned char reg_val;	
	CSN_Out_L;                // CSN low, initialize SPI communication...
	SPI_RW(reg);            // Select register to read from..
	reg_val= SPI_RW(0);    // ..then read registervalue
	CSN_Out_H;                // CSN high, terminate SPI communication
	return(reg_val);        // return register value
}
//***************************************************************/
//*函数：uint SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
//*功能: 用于读数据，reg：为寄存器地址，pBuf：为待读出数据地址，uchars：读出数据的个数
//********************************************************************/
unsigned char SPI_Read_Buf(unsigned char reg, unsigned char *pBuf, unsigned char uchars)
{
	unsigned char status,i;	
	CSN_Out_L;                    		// Set CSN low, init SPI tranaction
	status = SPI_RW(reg);       		// Select register to write to and read status uchar	
	for(i=0;i<uchars;i++)
			pBuf[i] = SPI_RW(0);    // 	
	CSN_Out_H;                           	
	return(status);                    // return nRF24L01 status uchar
}

/**************************************************
*函数：设置为接收模式
*void SetRX_Mode(void)
*功能：数据接收配置 
**************************************************/
void SwitchToRxMode(void)
{
//	unsigned char value;
	CE_Out_L;	
	SPI_RW_Reg(NRF_24l01_FLUSH_RX,0);//flush Rx
	//value=SPI_RW(STATUS);	 // read register STATUS's value
	SPI_RW_Reg(NRF_24l01_WRITE_REG+STATUS,0xff); // clear RX_DR or TX_DS or MAX_RT interrupt flag	
//	value=SPI_RW(CONFIG);	// read register CONFIG's value
	//value=value|0x01;//set bit 1
	SPI_RW_Reg(NRF_24l01_WRITE_REG + CONFIG, 0x0f); // Set PWR_UP bit, enable CRC(2 length) & Prim:RX. RX_DR enabled..
 
	CE_Out_H;	
	NRF24L01_Delayus(130);
} 

/**************************************************
 Function: SwitchToTxMode();
 Description:
	switch to Tx mode
**************************************************/
 
void SwitchToTxMode(void)
{
	// unsigned char value;
	  CE_Out_L;
	 SPI_RW_Reg(NRF_24l01_FLUSH_TX,0); //flush Tx
	 SPI_RW_Reg(NRF_24l01_WRITE_REG+STATUS,0xff); // clear RX_DR or TX_DS or MAX_RT interrupt flag
	// value=SPI_RW(CONFIG);	// read register CONFIG's value
	 //value=value&0xfe;     //reset bit 0
   SPI_RW_Reg(NRF_24l01_WRITE_REG + CONFIG, 0x0e); //switch to Tx mode	
   CE_Out_H;    //  read-in data
 	 NRF24L01_Delayus(130);
	
}
/************************************************************************
*函数：void Transmit_Tx_bufferdata(void)
*功能：数据发送
**************************************************************************/
void Transmit_Tx_bufferdata(unsigned char *pBuf) 
{
	   CE_Out_L;
	  //SPI_RW_Reg(WRITE_REG + CONFIG, 0x06);      // CRC使能，16位CRC校验，上电 	
  	SPI_Write_Buf(NRF_24l01_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);     // 写入发送地址
  	//SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);  // 为了应答接收设备，接收通道0地址和发送地址相同	
  	SPI_Write_Buf(NRF_24l01_WR_TX_PLOAD, pBuf, TX_PLOAD_WIDTH);                  // 写数据包到TX FIFO 	
	  CE_Out_H;;//激活发送信号
	  NRF24L01_Delayus(8300);
}

/************************************************************************
*函数：unsigned char NRF24l01_Interrupt_Handle(void)
*
*功能：中断处理函数
*
*返回参数  接受的中断返回1     发送中断返回2   否则返回 0
*
*备注：CRP  20150506
*
**************************************************************************/

unsigned char NRF24l01_Interrupt_Handle(void)//添加返回值主要是方便中断函数调用
{


   unsigned char sta=0,nrf_flag=0;
	 sta = SPI_Read(STATUS);	  // 读状态寄存器
	  if(sta&0x40)  //接收中断
		{
			 SPI_Read_Buf(NRF_24l01_RD_RX_PLOAD,NRF24l01_RX_BUFF,RX_PLOAD_WIDTH);// read receive payload from RX_FIFO buffer	
	 	   //for(i=0; i < RX_PLOAD_WIDTH;i++)
				//{

				//	UART_send_char(USART1,NRF24l01_RX_BUFF[i]); //USART1~UART5   u32tempdat：0~4294967296
				//	UART_send_char(USART1,' ');

			// }
				//UART_send_char(USART1,'\n'); 
				//UART_send_char(USART1,'\n'); 
			 nrf_flag=0x01;
		}
		else if(sta&0x20)  //发送中断
		{
			  UART_send_char(USART1,'F'); 
				UART_send_char(USART1,'\n'); 		 
       SPI_RW_Reg(NRF_24l01_FLUSH_TX,0xff); 			
			  nrf_flag=0x02;
		}
		//else if(sta & 0x10)  //最大重发溢出
		//{
		//	led2=~led2;
		//	flag=3;	
		//}
		else ;
	 SPI_RW_Reg(NRF_24l01_WRITE_REG + STATUS, 0xff); //清除所有中断标志位 
		
		return nrf_flag;
		
}
