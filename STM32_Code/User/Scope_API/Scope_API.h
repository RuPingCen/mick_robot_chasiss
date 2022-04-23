#ifndef _SCOPE_API_H_
#define _SCOPE_API_H_

 
extern float OutData[4];
/*****************************************************************
 陀螺仪 加速度计 上位机  波特率9600 
 配合串口示波器的校验协议
*****************************************************************
*/
extern unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT);
/************************************
UART 示波器发送函数
参数说明： 
.    OutData[]  需要发送的数值赋予该数组 
*************************************/
extern void OutPut_Data(void);




#endif
