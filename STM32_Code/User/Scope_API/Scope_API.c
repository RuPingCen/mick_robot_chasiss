#include "stm32f10x_usart.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_dma.h" 
#include "bsp_uart.h"
  

#include "Scope_API.h" //����ʾ����
 

//ʹ��֮ǰ��Ҫ���������жԴ���ģ���ʼ��  ����ʹ���� UART1 ģ��

 float OutData[4];


/*****************************************************************
 ������ ���ٶȼ� ��λ��  ������9600 
 ��ϴ���ʾ������У��Э��
*****************************************************************
*/
unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
    unsigned short CRC_Temp;
    unsigned char i,j;
    CRC_Temp = 0xffff;

    for (i=0;i<CRC_CNT; i++){      
        CRC_Temp ^= Buf[i];
        for (j=0;j<8;j++) {
            if (CRC_Temp & 0x01)
                CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
            else
                CRC_Temp = CRC_Temp >> 1;
        }
    }
    return(CRC_Temp);
}
/************************************
UART ʾ�������ͺ���   Ĭ��ʹ�ã�UART1
����˵���� 
.    OutData[]  ��Ҫ���͵���ֵ��������� 
*************************************/
void OutPut_Data(void)  
{  
    int temp[4] = {0};  
    unsigned int temp1[4] = {0};  
    unsigned char databuf[10] = {0};  
    unsigned char i;  
    unsigned short CRC16 = 0;  
    
    for(i=0;i<4;i++)  
    {  
        temp[i]  = (int)OutData[i];  
        temp1[i] = (unsigned int)temp[i];  
    }   
  
    for(i=0;i<4;i++)  
    {  
        databuf[i*2]   = (unsigned char)(temp1[i]%256);  
        databuf[i*2+1] = (unsigned char)(temp1[i]/256);  
    }  

    CRC16 = CRC_CHECK(databuf,8);  
    databuf[8] = CRC16%256;  
    databuf[9] = CRC16/256;  

    for(i=0;i<10;i++)  
    {  
        UART_send_char(USART1,(char)databuf[i]);  //�˴��ɸ���UART  
    }  
} 

