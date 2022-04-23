#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h" 
#include "LCD240x320_ILI9341.h"
#include "nmea/nmea.h"     
#include "nmea_GPS_NEO6M_config.h"
 
 //עʹ��NMEA���ʱ����Ҫ��STM32�Ķ�ջ�Ĵ�  �� startup_stm32f10x_hd.s�ļ��� ��45��   Heap_Size       EQU     0x00001000 



 


uint8_t gps_rbuff[GPS_NEO6M_RBUFF_SIZE];/* DMA���ջ���  */
__IO uint8_t GPS_TransferEnd = 0, GPS_HalfTransferEnd = 0;/* DMA���������־ */

 
 
nmeaINFO info;          //GPS�����õ�����Ϣ
nmeaPARSER parser;      //����ʱʹ�õ����ݽṹ  
uint8_t new_parse=0;    //�Ƿ����µĽ������ݱ�־
nmeaTIME beiJingTime;    //����ʱ�� 
              
/*************************************************************************
*  �������ƣ� GPS_NEO6M_Init
*  ����˵���� GPS NE06M ģ���ʼ��
*  ����˵���� �� 
*
*  �������أ� ��
*  �޸�ʱ�䣺 2015-5-2
*  ��    ע�� CRP 
*  �ο�������STM32 DMA˼·
*************************************************************************/
void GPS_NEO6M_Init(void)
{
	
			NVIC_InitTypeDef NVIC_InitStructure;
			DMA_InitTypeDef DMA_InitStructure;	/*����DMAʱ��*/
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
			USART_InitStructure.USART_BaudRate = 9600;                //GPSģ��Ĭ��ʹ�ò����ʣ�9600
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			USART_InitStructure.USART_Parity = USART_Parity_No ;
			USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
			USART_Init(USART2, &USART_InitStructure); 
			USART_Cmd(USART2, ENABLE);

			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);		
			DMA_InitStructure.DMA_PeripheralBaseAddr = GPS_NEO6M_DATA_ADDR;	/*����DMAԴ���������ݼĴ�����ַ*/   	
			DMA_InitStructure.DMA_MemoryBaseAddr = (u32)gps_rbuff;/*�ڴ��ַ(Ҫ����ı�����ָ��)*/
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	/*���򣺴����赽�ڴ�*/				
			DMA_InitStructure.DMA_BufferSize = GPS_NEO6M_RBUFF_SIZE;/*�����СDMA_BufferSize=SENDBUFF_SIZE*/	
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;/*�����ַ����*/	 
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	/*�ڴ��ַ����*/
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;/*�������ݵ�λ*/	
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	/*�ڴ����ݵ�λ 8bit*/ 
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	/*DMAģʽ������ѭ��*/ 
			DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;	/*���ȼ�����*/	  
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;/*��ֹ�ڴ浽�ڴ�Ĵ���	*/
			DMA_Init(DMA1_Channel6, &DMA_InitStructure);/*����DMA��ͨ��*/	
			
			NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
			// DMA2 Channel Interrupt ENABLE
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

 
			DMA_ITConfig(DMA1_Channel6,DMA_IT_HT|DMA_IT_TC,ENABLE);  //����DMA������ɺ�����ж�
			DMA_Cmd (DMA1_Channel6,ENABLE);	/*ʹ��DMA*/	
			USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE); /* ���ô��� �� DMA����TX���� */
  
	
	    
       nmea_zero_INFO(&info);/* ��ʼ��GPS���ݽṹ */
       nmea_parser_init(&parser);
	
	
}
/*************************************************************************
*  �������ƣ� GPS_NEO6M_Routing
*  ����˵���� GPS NE06M �����߳�
*  ����˵���� �� 
*
*  �������أ� ��
*  �޸�ʱ�䣺 2015-5-2
*  ��    ע�� CRP 
*  �ο�����STM32�����޸� 
*************************************************************************/
void GPS_NEO6M_Routing(void)
{



    /* �����������������Ϣ�ĺ��� */
   // nmea_property()->trace_func = &trace;
   // nmea_property()->error_func = &error;


  

	   
      if(GPS_HalfTransferEnd)     /* ���յ�GPS_RBUFF_SIZEһ������� */
      {
				 
        /* ����nmea��ʽ���� */
        nmea_parse(&parser, (const char*)&gps_rbuff[0], HALF_GPS_NEO6M_RBUFF_SIZE, &info);
        
        GPS_HalfTransferEnd = 0;   //��ձ�־λ
        new_parse = 1;             //���ý�����Ϣ��־ 
				
      }
      else if(GPS_TransferEnd)    /* ���յ���һ������ */
      {
        
        nmea_parse(&parser, (const char*)&gps_rbuff[HALF_GPS_NEO6M_RBUFF_SIZE], HALF_GPS_NEO6M_RBUFF_SIZE, &info);
       
        GPS_TransferEnd = 0;
        new_parse =1;
      }
      
      if(new_parse )                //���µĽ�����Ϣ   
      {    
        /* �Խ�����ʱ�����ת����ת���ɱ���ʱ�� */
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
	  
	

 
   //   nmea_parser_destroy(&parser); /* �ͷ�GPS���ݽṹ */

    
    
} 

/*************************************************************************
*  �������ƣ� GPS_NEO6M_ProcessDMAIRQ
*  ����˵���� GPS NE06M �жϴ�����
*  ����˵���� �� 
*
*  �������أ� ��
*  �޸�ʱ�䣺 2015-5-2
*  ��    ע�� CRP 
*  �ο�����STM32�����޸� 
*************************************************************************/
void GPS_NEO6M_ProcessDMAIRQ(void) //GPS_ProcessDMAIRQ GPS DMA�жϷ�����
{
  
  if(DMA_GetITStatus(DMA1_IT_HT6) )         /* DMA �봫����� */
  {
    GPS_HalfTransferEnd = 1;                //���ð봫����ɱ�־λ
    DMA_ClearFlag(DMA1_FLAG_HT6);
	 
  }
  else if(DMA_GetITStatus(DMA1_IT_TC6))     /* DMA ������� */
  {
    GPS_TransferEnd = 1;                    //���ô�����ɱ�־λ
    DMA_ClearFlag(DMA1_FLAG_TC6);
 
   }
	 else
		 ;
}


















/*************************************************************************
*
*
*  ���³���Ϊ��GPS����ĸ�������ʱ�任�������ʱ��ʱ�����غ���      �����д��  Ұ��
*   
*  ��    ע�� crp
*  
*************************************************************************/
/******************************************************************************************************** 
**     ��������:            bit        IsLeapYear(uint8_t    iYear) 
**    ��������:            �ж�����(�������2000�Ժ�����) 
**    ��ڲ�����            iYear    ��λ���� 
**    ���ڲ���:            uint8_t        1:Ϊ����    0:Ϊƽ�� 
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
**     ��������:            void    GMTconvert(uint8_t *DT,uint8_t GMT,uint8_t AREA) 
**    ��������:            ��������ʱ�任�������ʱ��ʱ�� 
**    ��ڲ�����            *DT:    ��ʾ����ʱ������� ��ʽ YY,MM,DD,HH,MM,SS 
**                        GMT:    ʱ���� 
**                        AREA:    1(+)���� W0(-)���� 
********************************************************************************************************/ 
void GMTconvert(nmeaTIME *SourceTime, nmeaTIME *ConvertTime, uint8_t GMT,uint8_t AREA) 
{ 
    uint32_t    YY,MM,DD,hh,mm,ss;        //������ʱ�����ݴ���� 
     
    if(GMT==0)    return;                //�������0ʱ��ֱ�ӷ��� 
    if(GMT>12)    return;                //ʱ�����Ϊ12 �����򷵻�         

    YY    =    SourceTime->year;                //��ȡ�� 
    MM    =    SourceTime->mon;                 //��ȡ�� 
    DD    =    SourceTime->day;                 //��ȡ�� 
    hh    =    SourceTime->hour;                //��ȡʱ 
    mm    =    SourceTime->min;                 //��ȡ�� 
    ss    =    SourceTime->sec;                 //��ȡ�� 

    if(AREA)                        //��(+)ʱ������ 
    { 
        if(hh+GMT<24)    hh    +=    GMT;//������������ʱ�䴦��ͬһ�������Сʱ���� 
        else                        //����Ѿ����ڸ�������ʱ��1����������ڴ��� 
        { 
            hh    =    hh+GMT-24;        //�ȵó�ʱ�� 
            if(MM==1 || MM==3 || MM==5 || MM==7 || MM==8 || MM==10)    //���·�(12�µ�������) 
            { 
                if(DD<31)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==4 || MM==6 || MM==9 || MM==11)                //С�·�2�µ�������) 
            { 
                if(DD<30)    DD++; 
                else 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
            } 
            else if(MM==2)    //����2�·� 
            { 
                if((DD==29) || (DD==28 && IsLeapYear(YY)==0))        //��������������2��29�� ���߲�����������2��28�� 
                { 
                    DD    =    1; 
                    MM    ++; 
                } 
                else    DD++; 
            } 
            else if(MM==12)    //����12�·� 
            { 
                if(DD<31)    DD++; 
                else        //�������һ�� 
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
        if(hh>=GMT)    hh    -=    GMT;    //������������ʱ�䴦��ͬһ�������Сʱ���� 
        else                        //����Ѿ����ڸ�������ʱ��1����������ڴ��� 
        { 
            hh    =    hh+24-GMT;        //�ȵó�ʱ�� 
            if(MM==2 || MM==4 || MM==6 || MM==8 || MM==9 || MM==11)    //�����Ǵ��·�(1�µ�������) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    31; 
                    MM    --; 
                } 
            } 
            else if(MM==5 || MM==7 || MM==10 || MM==12)                //������С�·�2�µ�������) 
            { 
                if(DD>1)    DD--; 
                else 
                { 
                    DD    =    30; 
                    MM    --; 
                } 
            } 
            else if(MM==3)    //�����ϸ�����2�·� 
            { 
                if((DD==1) && IsLeapYear(YY)==0)                    //�������� 
                { 
                    DD    =    28; 
                    MM    --; 
                } 
                else    DD--; 
            } 
            else if(MM==1)    //����1�·� 
            { 
                if(DD>1)    DD--; 
                else        //�����һ�� 
                {               
                    DD    =    31; 
                    MM    =    12; 
                    YY    --; 
                } 
            } 
        } 
    }         

    ConvertTime->year   =    YY;                //������ 
    ConvertTime->mon    =    MM;                //������ 
    ConvertTime->day    =    DD;                //������ 
    ConvertTime->hour   =    hh;                //����ʱ 
    ConvertTime->min    =    mm;                //���·� 
    ConvertTime->sec    =    ss;                //������ 
}  






/*********************************************************end of file**************************************************/
