#ifndef _MICK_IO_H_
#define _MICK_IO_H_

#include "stm32f4xx.h"


// ���� key1 = PE0
// ���� key2 = PE1

//  IN1 = PE2
//  IN2 = PE3
//  IN3 = PE4
//  IN4 = PE5

//  Out1 = PF5
//  Out2 = PF4
//  Out3 = PF3
//  Out4 = PF2

// sw1 = PE11
// sw2 = PE12
// sw3 = PE13
// sw4 = PE14

// led1 = PF13
// led2 = PF14
// led3 = PF15




#define LED1_FLIP  GPIO_Flip_level(GPIOF,GPIO_Pin_13)   // ��λ������ָ�� ��˸һ��
#define LED2_FLIP  GPIO_Flip_level(GPIOF,GPIO_Pin_14)   // ң��������ָ����˸
#define LED3_FLIP  GPIO_Flip_level(GPIOF,GPIO_Pin_15)    // CAN�����ж� ��˸


 
//mick robot �����˿��ư�

void Init_Mick_GPIO(void);

// ��ȡ�������ı���ֵ
uint8_t Read_Code_Switch(void);

// ��ȡ���� 1 �� 2 �İ���״̬
uint8_t Read_Key(uint8_t channel); 

//��ȡ��������ͨ����״̬
uint8_t Read_Isolated_Input(uint8_t ch);

//���ø��������״̬
void Set_Isolated_Output(uint8_t ch, uint8_t out_value);

//��ȡ4��IO״̬�ϴ���PC
void Isolated_IO_Upload_Message(void);
#endif
