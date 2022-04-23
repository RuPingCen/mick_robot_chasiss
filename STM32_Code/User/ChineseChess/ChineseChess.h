#ifndef _CHINESECHESS_H_
#define _CHINESECHESS_H_
#include "stm32f10x.h"


/*
������7*2  14 �����
һ�����Ӹ� 16������ �������湲32������



*/
 struct  PIECES
{
	 char team;  //A������ B������
	 unsigned int type; //��������
	 unsigned char x,y; //���ӵ�����
   unsigned char state; // 0��ʾ�����Ѿ�����  1��ʾ���ӻ���
};

enum 
{
	CHESS_TEAM_A =1,
	CHESS_TEAM_B =2
};
enum 
{
	CHESS_STATE_LIVE =1,
	CHESS_STATE_DEAD =2
};


enum 
{
	
		CHESS_A_BING =1,
		CHESS_A_Ju=2,
		CHESS_A_Ma=3,
		CHESS_A_Pao=4,
		CHESS_A_Xiang=5,
		CHESS_A_Si=6,
		CHESS_A_Shuai=7,
	
		CHESS_B_BING =11,
		CHESS_B_Ju=12,
		CHESS_B_Ma=13,
		CHESS_B_Pao=14,
		CHESS_B_Xiang=15,
		CHESS_B_Si=16,
		CHESS_B_Shuai=17
};


 
void Init_ChessBoard(void);
void Chess_Analy_Takeup(unsigned char x, unsigned char y);
void Chess_Analy_FallDown(unsigned char x, unsigned char y);
void Chess_Move_Bing(unsigned char ID);
void Chess_Move_Ma(unsigned char ID);
void Chess_Move_Ju(unsigned char ID);
void Chess_Move_Pao(unsigned char ID);
void Chess_Move_Xiang(unsigned char ID);
void Chess_Move_Si(unsigned char ID);
void Chess_Move_Shuai(unsigned char ID);

unsigned char check_xy( int x,  int y);





//-------------LED ��
void CHESS_TEST_LED_1(void);
void CHESS_TEST_LED_2(void);
void CHESS_Light_xy_WS2812LED(unsigned char x,unsigned char y,unsigned char mode,unsigned char R,unsigned char G,unsigned char B);
void CHESS_Light_yCols_WS2812LED(unsigned char y,unsigned char mode,unsigned char R,unsigned char G,unsigned char B);
void CHESS_Light_xRows_WS2812LED(unsigned char x,unsigned char mode,unsigned char R,unsigned char G,unsigned char B);
void CHESS_Clear_XY_WS2812LED(void);
void CHESS_Clear_xy_WS2812LED(unsigned char x,unsigned char y);
#endif
