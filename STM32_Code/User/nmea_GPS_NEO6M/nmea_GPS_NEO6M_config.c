#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h" 
#include "LCD240x320_ILI9341.h"
#include "nmea/nmea.h"     
#include "nmea_GPS_NEO6M_config.h"
 
 //注使用NMEA库的时候需要将STM32的堆栈改大  在 startup_stm32f10x_hd.s文件中 第45行   Heap_Size       EQU     0x00001000 



 


uint8_t gps_rbuff[GPS_NEO6M_RBUFF_SIZE];/* DMA接收缓冲  */
__IO uint8_t GPS_TransferEnd = 0, GPS_HalfTransferEnd = 0;/* DMA传输结束标志 */

 
 
nmeaINFO info;          //GPS解码后得到的信息
nmeaPARSER parser;      //解码时使用的数据结构  
uint8_t new_parse=0;    //是否有新的解码数据标志
nmeaTIME beiJingTime;    //北京时间 
              
/*************************************************************************
*  函数名称： GPS_NEO6M_Init
*  功能说明： GPS NE06M 模块初始化
*  参数说明： 无 
*
*  函数返回： 无
*  修改时间： 2015-5-2
*  备    注： CRP 
*  参考火哥调用STM32 DMA思路
*************************************************************************/
void GPS_NEO6M_Init(void)
{
	
			NVIC_InitTypeDef NVIC_InitStructure;
			DMA_InitTypeDef DMA_InitStructure;	/*开启DMA时钟*/
			GPIO_InitTypeDef GPIO_InitStructure;
			USART_InitTypeDef USART_InitStructure;

			/* config USART2 clock */
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

			/* USART2 GPIO config */
			/* Configure USART2 Tx (PA.02) as alternate function push-pull */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			/* Configure USART2 Rx (PA.03) as input floating */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			/* USART2 mode config */
			USART_InitStructure.USART_BaudRate = 9600;                //GPS模块默认使用波特率：9600
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART2, &USART_InitStructure); 
			USART_Cmd(USART2, ENABLE);

			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		
			DMA_InitStructure.DMA_PeripheralBaseAddr = GPS_NEO6M_DATA_ADDR;	/*设置DMA源：串口数据寄存器地址*/   	
			DMA_InitStructure.DMA_MemoryBaseAddr = (u32)gps_rbuff;/*内存地址(要传输的变量的指针)*/
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	/*方向：从外设到内存*/				
			DMA_InitStructure.DMA_BufferSize = GPS_NEO6M_RBUFF_SIZE;/*传输大小DMA_BufferSize=SENDBUFF_SIZE*/	
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;/*外设地址不增*/	 
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	/*内存地址自增*/
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;/*外设数据单位*/	
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	/*内存数据单位 8bit*/ 
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	/*DMA模式：不断循环*/ 
			DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	/*优先级：中*/	  
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;/*禁止内存到内存的传输	*/
			DMA_Init(DMA1_Channel6, &DMA_InitStructure);/*配置DMA的通道*/	
			
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
			// DMA2 Channel Interrupt ENABLE
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

 
			DMA_ITConfig(DMA1_Channel6,DMA_IT_HT|DMA_IT_TC,ENABLE);  //配置DMA发送完成后产生中断
			DMA_Cmd (DMA1_Channel6,ENABLE);	/*使能DMA*/	
			USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); /* 配置串口 向 DMA发出TX请求 */
  
	
	    
       nmea_zero_INFO(&info);/* 初始化GPS数据结构 */
       nmea_parser_init(&parser);
	
	
}
/*************************************************************************
*  函数名称： GPS_NEO6M_Routing
*  功能说明： GPS NE06M 处理线程
*  参数说明： 无 
*
*  函数返回： 无
*  修改时间： 2015-5-2
*  备    注： CRP 
*  参考火哥的STM32程序修改 
*************************************************************************/
void GPS_NEO6M_Routing(void)
{



    /* 设置用于输出调试信息的函数 */
   // nmea_property()->trace_func = &trace;
   // nmea_property()->error_func = &error;


  

	   
      if(GPS_HalfTransferEnd)     /* 接收到GPS_RBUFF_SIZE一半的数据 */
      {
				 
        /* 进行nmea格式解码 */
        nmea_parse(&parser, (const char*)&gps_rbuff[0], HALF_GPS_NEO6M_RBUFF_SIZE, &info);
        
        GPS_HalfTransferEnd = 0;   //清空标志位
        new_parse = 1;             //设置解码消息标志 
				
      }
      else if(GPS_TransferEnd)    /* 接收到另一半数据 */
      {
        
        nmea_parse(&parser, (const char*)&gps_rbuff[HALF_GPS_NEO6M_RBUFF_SIZE], HALF_GPS_NEO6M_RBUFF_SIZE, &info);
       
        GPS_TransferEnd = 0;
        new_parse =1;
      }
      
      if(new_parse )                //有新的解码消息   
      {    
        /* 对解码后的时间进行转换，转换成北京时间 */
        GMTconvert(&info.utc,&beiJingTime,8,1);
 
				LCD240x320_ILI9341_DispStr(1,1, "Time:", BLUE,WHITE); 
				LCD240x320_ILI9341_write_u16data(30,1,beiJingTime.year+1900,BLUE,WHITE);
				LCD240x320_ILI9341_write_u16data(70,1,beiJingTime.mon+1,BLUE,WHITE);
				LCD240x320_ILI9341_write_u16data(110,1,beiJingTime.day,BLUE,WHITE);
				LCD240x320_ILI9341_write_u16data(150,1,beiJingTime.hour,BLUE,WHITE);
				LCD240x320_ILI9341_write_u16data(190,1,beiJingTime.min,BLUE,WHITE);
		    LCD240x320_ILI9341_write_u16data(190,15,beiJingTime.sec,BLUE,WHITE);

				LCD240x320_ILI9341_DispStr(1,30, "sig:", BLUE,WHITE);
				LCD240x320_ILI9341_write_u16data(30,30,info.sig,BLUE,WHITE);
				LCD240x320_ILI9341_DispStr(1,50, "fix:", BLUE,WHITE);
				LCD240x320_ILI9341_write_u16data(30,50,info.fix,BLUE,WHITE);


				LCD240x320_ILI9341_DispStr(1,70, "PDOP:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,70,info.PDOP,BLUE,WHITE);
				LCD240x320_ILI9341_DispStr(1,90, "HDOP:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,90,info.HDOP,BLUE,WHITE);
				LCD240x320_ILI9341_DispStr(1,110, "VDOP:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,110,info.VDOP,BLUE,WHITE);


				LCD240x320_ILI9341_DispStr(1,130, "lat:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,130,info.lat,BLUE,WHITE);
				LCD240x320_ILI9341_DispStr(1,150, "lon:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,150,info.lon,BLUE,WHITE);

				LCD240x320_ILI9341_DispStr(1,170, "elv:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,170,info.elv,BLUE,WHITE);
				LCD240x320_ILI9341_DispStr(1,190, "speed:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,190,info.speed,BLUE,WHITE);  
				LCD240x320_ILI9341_DispStr(1,210, "Yaw:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,210,info.direction,BLUE,WHITE);
				LCD240x320_ILI9341_DispStr(1,230, "declination:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,230,info.declination,BLUE,WHITE);


				LCD240x320_ILI9341_DispStr(1,250, "inuse:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,250,info.satinfo.inuse,BLUE,WHITE);
				LCD240x320_ILI9341_DispStr(1,270, "inview:", BLUE,WHITE);
				LCD240x320_ILI9341_write_floatdata(30,270,info.satinfo.inview,BLUE,WHITE);			
				
   
        new_parse = 0;
      }
	  
	

 
   //   nmea_parser_destroy(&parser); /* 释放GPS数据结构 */

    
    
} 

/*************************************************************************
*  函数名称： GPS_NEO6M_ProcessDMAIRQ
*  功能说明： GPS NE06M 中断处理函数
*  参数说明： 无 
*
*  函数返回： 无
*  修改时间： 2015-5-2
*  备    注： CRP 
*  参考火哥的STM32程序修改 
*************************************************************************/
void GPS_NEO6M_ProcessDMAIRQ(void) //GPS_ProcessDMAIRQ GPS DMA中断服务函数
{
  
  if(DMA_GetITStatus(DMA1_IT_HT6) )         /* DMA 半传输完成 */
  {
    GPS_HalfTransferEnd = 1;                //设置半传输完成标志位
    DMA_ClearFlag(DMA1_FLAG_HT6);
	 
  }
  else if(DMA_GetITStatus(DMA1_IT_TC6))     /* DMA 传输完成 */
  {
    GPS_TransferEnd = 1;                    //设置传输完成标志位
    DMA_ClearFlag(DMA1_FLAG_TC6);
 
   }
	 else
		 ;
}


















/*************************************************************************
*
*
*  以下程序为将GPS输出的格林尼治时间换算世界各时区时间的相关函数      代码编写者  野火
*   
*  备    注： crp
*  
*************************************************************************/
/******************************************************************************************************** 
**     函数名称:            bit        IsLeapYear(uint8_t    iYear) 
**    功能描述:            判断闰年(仅针对于2000以后的年份) 
**    入口参数：            iYear    两位年数 
**    出口参数:            uint8_t        1:为闰年    0:为平年 
********************************************************************************************************/ 
static uint8_t IsLeapYear(uint8_t iYear) 
{ 
    uint16_t    Year; 
    Year    =    2000+iYear; 
    if((Year&3)==0) 
    { 
        return ((Year%400==0) || (Year%100!=0)); 
    } 
     return 0; 
} 

/******************************************************************************************************** 
**     函数名称:            void    GMTconvert(uint8_t *DT,uint8_t GMT,uint8_t AREA) 
**    功能描述:            格林尼治时间换算世界各时区时间 
**    入口参数：            *DT:    表示日期时间的数组 格式 YY,MM,DD,HH,MM,SS 
**                        GMT:    时区数 
**                        AREA:    1(+)东区 W0(-)西区 
********************************************************************************************************/ 
void GMTconvert(nmeaTIME *SourceTime, nmeaTIME *ConvertTime, uint8_t GMT,uint8_t AREA) 
{ 
    uint32_t    YY,MM,DD,hh,mm,ss;        //年月日时分秒暂存变量 
     
    if(GMT==0)    return;                //如果处于0时区直接返回 
    if(GMT>12)    return;                //时区最大为12 超过则返回         

    YY    =    SourceTime->year;                //获取年 
    MM    =    SourceTime->mon;                 //获取月 
    DD    =    SourceTime->day;                 //获取日 
    hh    =    SourceTime->hour;                //获取时 
    mm    =    SourceTime->min;                 //获取分 
    ss    =    SourceTime->sec;                 //获取秒 

    if(AREA)                        //东(+)时区处理 
    { 
        if(hh+GMT<24)    hh    +=    GMT;//如果与格林尼治时间处于同一天则仅加小时即可 
        else                        //如果已经晚于格林尼治时间1天则进行日期处理 
        { 
            hh    =    hh+GMT-24;        //先得出时间 
            if(MM==1 || MM==3 || MM==5 || MM==7 || MM==8 || MM==10)    //大月份(12月单独处理) 
            { 
                if(DD<31)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==4 || MM==6 || MM==9 || MM==11)                //小月份2月单独处理) 
            { 
                if(DD<30)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==2)    //处理2月份 
            { 
                if((DD==29) || (DD==28 && IsLeapYear(YY)==0))        //本来是闰年且是2月29日 或者不是闰年且是2月28日 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
                else    DD++; 
            } 
            else if(MM==12)    //处理12月份 
            { 
                if(DD<31)    DD++; 
                else        //跨年最后一天 
                {               
                    DD    =    1; 
                    MM    =    1; 
                    YY    ++; 
                } 
            } 
        } 
    } 
    else 
    {     
        if(hh>=GMT)    hh    -=    GMT;    //如果与格林尼治时间处于同一天则仅减小时即可 
        else                        //如果已经早于格林尼治时间1天则进行日期处理 
        { 
            hh    =    hh+24-GMT;        //先得出时间 
            if(MM==2 || MM==4 || MM==6 || MM==8 || MM==9 || MM==11)    //上月是大月份(1月单独处理) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    31; 
                    MM    --; 
                } 
            } 
            else if(MM==5 || MM==7 || MM==10 || MM==12)                //上月是小月份2月单独处理) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    30; 
                    MM    --; 
                } 
            } 
            else if(MM==3)    //处理上个月是2月份 
            { 
                if((DD==1) && IsLeapYear(YY)==0)                    //不是闰年 
                { 
                    DD    =    28; 
                    MM    --; 
                } 
                else    DD--; 
            } 
            else if(MM==1)    //处理1月份 
            { 
                if(DD>1)    DD--; 
                else        //新年第一天 
                {               
                    DD    =    31; 
                    MM    =    12; 
                    YY    --; 
                } 
            } 
        } 
    }         

    ConvertTime->year   =    YY;                //更新年 
    ConvertTime->mon    =    MM;                //更新月 
    ConvertTime->day    =    DD;                //更新日 
    ConvertTime->hour   =    hh;                //更新时 
    ConvertTime->min    =    mm;                //更新分 
    ConvertTime->sec    =    ss;                //更新秒 
}  






/*********************************************************end of file**************************************************/
