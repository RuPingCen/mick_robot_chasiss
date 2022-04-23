#ifndef __nmea_GPS_NEO6M_config_H
#define	__nmea_GPS_NEO6M_config_H

#include "stm32f10x.h"
#include "nmea/nmea.h"


 

  

/* GPS�ӿ����� ʹ�ò�ͬ�Ĵ���ʱ��Ҫ�޸Ķ�Ӧ�Ľӿ� */

#define USART2_DataR_Base            (0x40004400+0x04)		  // ����2�����ݼĴ�����ַ


#define GPS_NEO6M_DATA_ADDR             USART2_DataR_Base      //GPSʹ�õĴ��ڵ����ݼĴ�����ַ
#define GPS_NEO6M_RBUFF_SIZE            512                   //���ڽ��ջ�������С
#define HALF_GPS_NEO6M_RBUFF_SIZE       (GPS_NEO6M_RBUFF_SIZE/2)    //���ڽ��ջ�����һ��  







extern uint8_t gps_rbuff[GPS_NEO6M_RBUFF_SIZE];
extern __IO uint8_t GPS_TransferEnd ;
extern __IO uint8_t GPS_HalfTransferEnd;


void GPS_NEO6M_ProcessDMAIRQ(void);
void GPS_NEO6M_Routing(void);
void GPS_NEO6M_Init(void);

void trace(const char *str, int str_size);
void error(const char *str, int str_size);
void GMTconvert(nmeaTIME *SourceTime, nmeaTIME *ConvertTime, uint8_t GMT,uint8_t AREA) ;




#endif 
