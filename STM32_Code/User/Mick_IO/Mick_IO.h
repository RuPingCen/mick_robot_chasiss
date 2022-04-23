#ifndef _MICK_IO_H_
#define _MICK_IO_H_

#include "stm32f10x.h"

 
#define LED1_FLIP  GPIO_Flip_level(GPIOA,GPIO_Pin_6) 
#define LED2_FLIP  GPIO_Flip_level(GPIOA,GPIO_Pin_5) 
#define LED3_FLIP  GPIO_Flip_level(GPIOC,GPIO_Pin_13) 


 
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
