#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_Delay.h"
#include <stdio.h>
#include "WS2812.h"
#include "ChineseChess.h"
#include "bsp_uart.h"

extern unsigned char WS2812_datas[180][3];
unsigned char chessboard[10][9]={255}; //�洢ÿ����������ӵ�ID
struct PIECES chess_pieces[32]; //�洢32������

unsigned char chess_laset_pose[1][2]; //��һ���ƶ�������
unsigned char chess_current_pose[1][2];//��ǰ�ƶ�������

unsigned char chess_current_UpID;//��ǰ�����������ӵ�ID
unsigned char chess_current_Upflag; // 0x01 �Ѿ���һ��������������

volatile unsigned char CHESS_LIGHT_R=10;   // ������ʾ��RGB��ɫ
volatile unsigned char CHESS_LIGHT_G=10;
volatile unsigned char CHESS_LIGHT_B=10;
volatile unsigned char CHESS_Breath_flag=0; //�����Ʒ���
volatile unsigned char CHESS_LIGHT_Max=50;

extern unsigned char WS2812_light_flag;  
	
// ����һ��λ�øı�����꣬�������λ���ϵ���������
// ��������ƶ��������д���
void Chess_Analy_Takeup(unsigned char x, unsigned char y)
{
	unsigned char ID =0;
	struct PIECES pieces_t;
//	unsigned char state_tem =0;
	CHESS_Clear_XY_WS2812LED();
	chess_laset_pose[0][0] = 	chess_current_pose[0][0];
	chess_laset_pose[0][1] = 	chess_current_pose[0][1];
	chess_current_pose[0][0] = x;
	chess_current_pose[0][1] = y;
	if((chess_laset_pose[0][0] != 0x00 ) && (chess_laset_pose[0][1] !=0x00))
	{
		CHESS_Clear_xy_WS2812LED(chess_laset_pose[0][0],chess_laset_pose[0][1]);
		CHESS_Light_xy_WS2812LED(chess_laset_pose[0][0] ,chess_laset_pose[0][1] ,2,0,CHESS_LIGHT_G,0); //��һ��λ����ԲȦ
	}

	
	CHESS_Light_xy_WS2812LED(chess_current_pose[0][0] ,chess_current_pose[0][1] ,1,0,CHESS_LIGHT_G,0); //����λ������ɫ��
	
	UART_send_string(USART2,"\n moving :");  UART_send_data(USART2,x); 
	UART_send_string(USART2,"  ");  UART_send_data(USART2,y); 
	 
	ID = chessboard[x-1][y-1];
  pieces_t = chess_pieces[ID];
	
	UART_send_string(USART2,"\t ID :"); UART_send_data(USART2,ID);UART_send_string(USART2,"\t "); 
	if((pieces_t.type == CHESS_A_BING) || (pieces_t.type == CHESS_B_BING))
	{
		 UART_send_string(USART2,"moving bing.\n"); 
		 Chess_Move_Bing(ID);
	}
	else if((pieces_t.type == CHESS_A_Ju) || (pieces_t.type == CHESS_B_Ju))
	{
		  UART_send_string(USART2,"moving Ju.\n"); 
		  Chess_Move_Ju(ID);
	}
	else if((pieces_t.type == CHESS_A_Ma) || (pieces_t.type == CHESS_B_Ma))
	{
		 UART_send_string(USART2,"moving Ma.\n"); 
		 Chess_Move_Ma(ID);
	}
	else if((pieces_t.type == CHESS_A_Pao) || (pieces_t.type == CHESS_B_Pao))
	{
		 UART_send_string(USART2,"moving Pao.\n"); 
		 Chess_Move_Pao(ID);
	}
	else if((pieces_t.type == CHESS_A_Xiang) || (pieces_t.type == CHESS_B_Xiang))
	{
		 UART_send_string(USART2,"moving Xiang.\n"); 
		 Chess_Move_Xiang(ID);
	}
	else if((pieces_t.type == CHESS_B_Si) || (pieces_t.type == CHESS_A_Si))
	{
		 UART_send_string(USART2,"moving Si.\n"); 
		 Chess_Move_Si(ID);
	}
	else if((pieces_t.type == CHESS_B_Shuai) || (pieces_t.type == CHESS_A_Shuai))
	{
		 UART_send_string(USART2,"moving Shuai.\n"); 
		 Chess_Move_Shuai(ID);
	}
	else ;
	
  chess_current_UpID = ID; //�������ӵ�״̬
  chess_current_Upflag ++; 
	WS2812_light_flag =0x01; //��⵽������������������WS2812 ���к�����
}
// ��������
void Chess_Analy_FallDown(unsigned char x, unsigned char y)
{ 
	 unsigned char tem_i,tem_j;
	//1���޸� chessboard ÿ��λ�ô洢������ID �������̵�״̬
	// �������� �ߵ���λ��
		unsigned char new_ID =200;
		struct PIECES Old_pieces_t;// New_pieces_t;
		new_ID = chessboard[x-1][y-1];
		chess_current_pose[0][0] = x;
		chess_current_pose[0][1] = y;
	  //New_pieces_t = chess_pieces[new_ID];             //�·���ȥ��λ�����ӵ�ID
	   
		//if (new_ID == 0xff ) //�����ӵ�λ��Ϊ��
		{
			 chessboard[x-1][y-1] = chess_current_UpID; // �������������ӵķ������ӵ�λ��
			 Old_pieces_t = chess_pieces[chess_current_UpID]; // ��������ID
			 chessboard[Old_pieces_t.x-1][Old_pieces_t.y-1] = 0xff; //ɾ��ԭ��λ���ϴ��������ӵ�ID
			 chess_pieces[chess_current_UpID].x =x;    //����������ӵ�λ��
			 chess_pieces[chess_current_UpID].y =y; 
		   // �رճ���һ��λ��������е�Ч��
			 CHESS_Clear_XY_WS2812LED();
			 //CHESS_Light_xy_WS2812LED(chess_current_pose[0][0] ,chess_current_pose[0][1] ,2,0,CHESS_LIGHT_G,0); //��һ��λ����ԲȦ
				CHESS_Light_xy_WS2812LED(x ,y,2,0,CHESS_LIGHT_G,0);  
				// 
				UART_send_string(USART2,"chessboard state: \n"); 
				for(tem_i=0;tem_i<10;tem_i++)
				{
					for(tem_j=0;tem_j<9;tem_j++)
					{
							UART_send_string(USART2,"  ");  UART_send_data(USART2,chessboard[tem_i][tem_j]);			
					}
					UART_send_string(USART2,"\n"); 
				}
		}
	//2�����ǳ��� �����Ե������Ӵ�chessboard��ɾ��
	//����ʱ���������һ�����ӣ�Ȼ������ڶ����Ե������ӣ���󽫵�һ�������������ӷ���Ŀ��λ���ϡ�
	;
	
	WS2812_light_flag =0x02; //����WS2812 ���к�����
}

// �����ƶ�  ���뷢���ƶ�λ�õ�����ID
void Chess_Move_Bing(unsigned char ID)
{ 
	unsigned char tem_x,tem_y,tem_value;
	unsigned char led_x,led_y,condition;
	struct PIECES pieces_tem;
	pieces_tem = chess_pieces[ID];
	tem_x = pieces_tem.x-1;
	tem_y = pieces_tem.y-1;
	
	
//û�й��ӵı�ֻ����ǰ�ƶ�
//�����Ժ�����������ƶ�
	if(pieces_tem.team == CHESS_TEAM_A) //A��������
	{
	  if(pieces_tem.x > 5)  //������Ѿ�������
		{
			//�ж���ǰ�����Ƿ������������棬 û�еĻ�����λ��Ϊ0

			condition = check_xy(tem_x+1,tem_y); 
			if(condition)
			{
			  UART_send_string(USART2," condition bing A.2 ");
				tem_value = chessboard[tem_x+1][tem_y];
				led_x = tem_x+1+1;
				led_y = tem_y+1;
				 if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;
			}
			 
			condition = check_xy(tem_x,tem_y+1); 
			if(condition)
			{
				UART_send_string(USART2," condition bing A.3 ");
				tem_value = chessboard[tem_x][tem_y+1];
				led_x = tem_x+1;
				led_y = tem_y+1+1;
				 if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;
			}
			condition = check_xy(tem_x,tem_y-1); 
			if(condition)
			{
				UART_send_string(USART2," condition bing A.4 ");
				tem_value = chessboard[tem_x][tem_y-1];
				led_x = tem_x+1;
				led_y = tem_y-1+1;
				 if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;
      }
		}
		else
		{
			 condition = check_xy(tem_x+1,tem_y); 
			if(condition)
			{
				UART_send_string(USART2," condition bing A.1 ");
				tem_value = chessboard[tem_x+1][tem_y];
				led_x = tem_x+1+1;
				led_y = tem_y+1;
				 if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;
      }
			 
		}
	}
	else if(pieces_tem.team == CHESS_TEAM_B)
	{
				//�ж���ǰ�����Ƿ������������棬 û�еĻ�����λ��Ϊ0
		if(pieces_tem.x < 6)  //������Ѿ�������
		{
				condition = check_xy(tem_x-1,tem_y); 
				if(condition)
				{
					UART_send_string(USART2," condition bing B.2 ");
					tem_value = chessboard[tem_x-1][tem_y];
					led_x = tem_x-1+1;
					led_y = tem_y+1;
					 if(tem_value == 0xff)
								CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
						else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
						else;
				}
				 
				condition = check_xy(tem_x,tem_y+1); 
				if(condition)
				{
					UART_send_string(USART2," condition bing A.3 ");
					tem_value = chessboard[tem_x][tem_y+1];
					led_x = tem_x+1;
					led_y = tem_y+1+1;
					 if(tem_value == 0xff)
								CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
						else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
						else;
				}
				condition = check_xy(tem_x,tem_y-1); 
				if(condition)
				{
					UART_send_string(USART2," condition bing B.4 ");
					tem_value = chessboard[tem_x][tem_y-1];
					led_x = tem_x+1;
					led_y = tem_y-1+1;
					 if(tem_value == 0xff)
								CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
						else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
						else;
				}
			}
			else
			{
				 condition = check_xy(tem_x-1,tem_y); 
				if(condition)
				{
					UART_send_string(USART2," condition bing B.1 ");
					tem_value = chessboard[tem_x-1][tem_y];
					led_x = tem_x-1+1;
					led_y = tem_y+1;
					 if(tem_value == 0xff)
								CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
						else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
						else;
				}
				 
			}
		
		}
	else
		;
	

}

// ����ƶ�  ���뷢���ƶ�λ�õ�����ID
// Ŀ��λ�ÿɴ�д��ڶԷ�����ʱ��
void Chess_Move_Ma(unsigned char ID)
{
	unsigned char tem_x,tem_y,tem_value;
	unsigned char led_x,led_y;
	unsigned char condition1=0,condition2=0,condition3=0,condition4=0;
	struct PIECES pieces_tem;
	pieces_tem = chess_pieces[ID];
 
	
	
	// ���ɿ����ߵ�·��
 
	tem_x = pieces_tem.x-1 ;
	tem_y = pieces_tem.y-1 ;
	
	condition1 = check_xy(tem_x-1,tem_y);   
	if(condition1 )
	{
		if(chessboard[tem_x-1][tem_y] ==0xff)
		{
			UART_send_string(USART2," condition1 ");
			condition1 = check_xy(tem_x-2,tem_y-1);//
			if(condition1)
			{
				tem_value = chessboard[tem_x-2][tem_y-1];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
				led_x = tem_x-2+1;
				led_y = tem_y-1+1;
				if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
				else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
				else;
			}
			
			condition1 = check_xy(tem_x-2,tem_y+1);//
			if(condition1)
			{
				tem_value = chessboard[tem_x-2][tem_y+1];
				led_x = tem_x-2+1;
				led_y = tem_y+1+1;
				if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
				else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
				else;
			}
		}
	}
	
 	condition2 = check_xy(tem_x+1,tem_y);  
 if(condition2 )
	{
		if(chessboard[tem_x+1][tem_y] == 0xff)
		{
			condition2 = check_xy(tem_x+2,tem_y-1);//
			if(condition2)
			{
				UART_send_string(USART2," condition2.1 ");
				tem_value = chessboard[tem_x+2][tem_y-1];
				led_x = tem_x+2+1;
				led_y = tem_y-1+1;
				if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
				else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
				else;
			}
			
			condition2 = check_xy(tem_x+2,tem_y+1);//
			if(condition2)
			{
				UART_send_string(USART2," condition2.2 ");
				tem_value = chessboard[tem_x+2][tem_y+1];
				led_x = tem_x+2+1;
				led_y = tem_y+1+1;
				if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
				else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
				else;
			}
		}
	}	
 
	tem_x = pieces_tem.x-1 ;
	tem_y = pieces_tem.y-1 ;
	condition3 = check_xy(tem_x,tem_y-1);  
	if(condition3 )
	{
		if(chessboard[tem_x][tem_y-1] == 0xff)
		{
				condition3 = check_xy(tem_x-1,tem_y-2);//
				if(condition3 )
				{
					UART_send_string(USART2," condition3.1 ");
					tem_value = chessboard[tem_x-1][tem_y-2];
					led_x = tem_x-1+1;
					led_y = tem_y-2+1;
					if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;
				}
				
				condition3 = check_xy(tem_x+1,tem_y-2);//
				if(condition3)
				{
					UART_send_string(USART2," condition3.2 ");
					tem_value = chessboard[tem_x+1][tem_y-2];
					led_x = tem_x+1+1;
					led_y = tem_y-2+1;
					if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;
				}
		}
	}	
	
	condition4 = check_xy(tem_x,tem_y+1); 		
	if(condition4 )
	{
		if(chessboard[tem_x][tem_y+1] ==0xff)
		{
			condition4 = check_xy(tem_x-1,tem_y+2);//
			if(condition4)
			{
				UART_send_string(USART2," condition4.1 ");
				tem_value = chessboard[tem_x-1][tem_y+2];
				led_x = tem_x-1+1;
				led_y = tem_y+2+1;
				if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
				else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
				else;
			}
 
			condition4 = check_xy(tem_x+1,tem_y+2);//
			if(condition4)
			{
				UART_send_string(USART2," condition4.2 ");
				tem_value = chessboard[tem_x+1][tem_y+2];
				led_x = tem_x+1+1;
				led_y = tem_y+2+1;
				if(tem_value == 0xff)
						CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
				else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
						CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
				else;
			}
		}
	}
 
}
// ��������
void Chess_Move_Xiang(unsigned char ID)
{
	unsigned char tem_x,tem_y,tem_value;
	unsigned char led_x,led_y;
	unsigned char condition=0,condition2=0;
	struct PIECES pieces_tem;
	pieces_tem = chess_pieces[ID];
  
	// ���ɿ����ߵ�·��
	tem_x = pieces_tem.x-1 ;
	tem_y = pieces_tem.y-1 ;
	
	condition = check_xy(tem_x-1,tem_y-1);  //���� 
	if(condition )
	{
		if(chessboard[tem_x-1][tem_y-1] ==0xff)
		{
			UART_send_string(USART2," condition1 ");
			condition = check_xy(tem_x-2,tem_y-2);//
			if(condition)
			{
				tem_value = chessboard[tem_x-2][tem_y-2];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
				led_x = tem_x-2+1;
				led_y = tem_y-2+1;
				 if(pieces_tem.team == CHESS_TEAM_A && (led_x < 6))
						 condition2 =1;
				 else if(pieces_tem.team == CHESS_TEAM_B && (led_x > 5))
						condition2 =1;
        else 
					  condition2=0;
				 if(condition2 )
				 {
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;
				}
			}
		}
	}
	condition = check_xy(tem_x-1,tem_y+1);  //���� 
	if(condition )
	{
		if(chessboard[tem_x-1][tem_y+1] ==0xff)
		{
			UART_send_string(USART2," condition2 ");
			condition = check_xy(tem_x-2,tem_y+2);//
			if(condition)
			{
				tem_value = chessboard[tem_x-2][tem_y+2];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
				led_x = tem_x-2+1;
				led_y = tem_y+2+1;
				if(pieces_tem.team == CHESS_TEAM_A && (led_x < 6))
						 condition2 =1;
				else if(pieces_tem.team == CHESS_TEAM_B && (led_x > 5))
						condition2 =1;
        else 
					  condition2=0;
				if(condition2 )
				 {
						if(tem_value == 0xff)
								CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
						else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
						else;
				}

			}
		}
	}
 	condition = check_xy(tem_x+1,tem_y-1);  //���� 
	if(condition )
	{
		if(chessboard[tem_x+1][tem_y-1] ==0xff)
		{
			UART_send_string(USART2," condition3 ");
			condition = check_xy(tem_x+2,tem_y-2);//
			if(condition)
			{
				tem_value = chessboard[tem_x+2][tem_y-2];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
				led_x = tem_x+2+1;
				led_y = tem_y-2+1;
				if(pieces_tem.team == CHESS_TEAM_A && (led_x < 6))
						 condition2 =1;
				else if(pieces_tem.team == CHESS_TEAM_B && (led_x > 5))
						condition2 =1;
        else 
					  condition2=0;
				if(condition2 )
				 {
						if(tem_value == 0xff)
								CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
						else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
						else;
				}
			}
		}
	}
	condition = check_xy(tem_x+1,tem_y+1);  //���� 
	if(condition )
	{
		if(chessboard[tem_x+1][tem_y+1] ==0xff)
		{
			UART_send_string(USART2," condition4 ");
			condition = check_xy(tem_x+2,tem_y+2);//
			if(condition)
			{
				tem_value = chessboard[tem_x+2][tem_y+2];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
				led_x = tem_x+2+1;
				led_y = tem_y+2+1;
				if(pieces_tem.team == CHESS_TEAM_A && (led_x < 6))
						 condition2 =1;
				else if(pieces_tem.team == CHESS_TEAM_B && (led_x > 5))
						condition2 =1;
				else 
					  condition2=0;
				if(condition2 )
				 {
						if(tem_value == 0xff)
								CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
						else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
						else;
				}
			}
		}
	}
		 
}
// ʿ�߶Խ�
void Chess_Move_Si(unsigned char ID)
{
	unsigned char tem_x,tem_y,tem_value;
	unsigned char led_x,led_y;
	unsigned char condition=0;
	unsigned char tem_i=0;
	struct PIECES pieces_tem;
	pieces_tem = chess_pieces[ID];
	
  
	// ���ɿ����ߵ�·��
	tem_x = pieces_tem.x-1 ;
	tem_y = pieces_tem.y-1 ;
	
	 
	if(pieces_tem.team == CHESS_TEAM_A) //A��������
	{
		for(tem_i =0;tem_i<4;tem_i++)
		{
			 if(tem_i == 0)
			 {
			  led_x = tem_x+1;
				led_y = tem_y+1;
			 }
			 else if(tem_i == 1)
			 {
			  led_x = tem_x+1;
				led_y = tem_y-1;
			 }
			  else if(tem_i == 2)
			 {
			  led_x = tem_x-1;
				led_y = tem_y+1;
			 }
			  else if(tem_i == 3)
			 {
			  led_x = tem_x-1;
				led_y = tem_y-1;
			 }
			 else 
				 ;
			 // 1 2 3 �� ��  4 5 6 ��
			 if((led_x > 2) || (led_x < 0) || (led_y > 5) || (led_y < 3))
				 condition = 0x00;
			 else 
				 condition = 0x01;
			 if(condition)
				{	 
					UART_send_string(USART2," condition1 ");
					tem_value = chessboard[led_x][led_y];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					led_x = led_x+1;
					led_y = led_y+1;
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;  
				}
		}
	}
	else if(pieces_tem.team == CHESS_TEAM_B) //B��������
	{
		for(tem_i =0;tem_i<4;tem_i++)
		{
			 if(tem_i == 0)
			 {
			  led_x = tem_x+1;
				led_y = tem_y+1;
			 }
			 else if(tem_i == 1)
			 {
			  led_x = tem_x+1;
				led_y = tem_y-1;
			 }
			  else if(tem_i == 2)
			 {
			  led_x = tem_x-1;
				led_y = tem_y+1;
			 }
			  else if(tem_i == 3)
			 {
			  led_x = tem_x-1;
				led_y = tem_y-1;
			 }
			 else 
				 ;
			// 8 9 10 �� ��  4 5 6 ��
		 if((led_x > 9) || (led_x < 7) || (led_y > 5) || (led_y < 3))
				 condition = 0x00;
			 else 
				 condition = 0x01;
			 if(condition)
				{	 
					UART_send_string(USART2," condition1 ");
					tem_value = chessboard[led_x][led_y];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					led_x = led_x+1;
					led_y = led_y+1;
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;  
				}
		}
	}
	else ;
		
	  
}

// ˧
void Chess_Move_Shuai(unsigned char ID)
{
	unsigned char tem_x,tem_y,tem_value;
	unsigned char led_x,led_y,tem_i;
	unsigned char condition=0;
	struct PIECES pieces_tem;
	pieces_tem = chess_pieces[ID];
  
	// ���ɿ����ߵ�·��
	tem_x = pieces_tem.x-1 ;
	tem_y = pieces_tem.y-1 ;
	 
	if(pieces_tem.team == CHESS_TEAM_A) //A��������
	{
		for(tem_i =0;tem_i<4;tem_i++)
		{
			 if(tem_i == 0)
			 {
			  led_x = tem_x+1;
				led_y = tem_y;
			 }
			 else if(tem_i == 1)
			 {
			  led_x = tem_x-1;
				led_y = tem_y;
			 }
			  else if(tem_i == 2)
			 {
			  led_x = tem_x;
				led_y = tem_y+1;
			 }
			  else if(tem_i == 3)
			 {
			  led_x = tem_x;
				led_y = tem_y-1;
			 }
			 else 
				 ;
			 // 1 2 3 �� ��  4 5 6 ��
			 if((led_x > 2) || (led_x < 0) || (led_y > 5) || (led_y < 3))
				 condition = 0x00;
			 else 
				 condition = 0x01;
			 if(condition)
				{	 
					UART_send_string(USART2," condition1 ");
					tem_value = chessboard[led_x][led_y];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					led_x = led_x+1;
					led_y = led_y+1;
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;  
				}
		}
	}
	else if(pieces_tem.team == CHESS_TEAM_B) //B��������
	{
		for(tem_i =0;tem_i<4;tem_i++)
		{
			  if(tem_i == 0)
			 {
			  led_x = tem_x+1;
				led_y = tem_y;
			 }
			 else if(tem_i == 1)
			 {
			  led_x = tem_x-1;
				led_y = tem_y;
			 }
			  else if(tem_i == 2)
			 {
			  led_x = tem_x;
				led_y = tem_y+1;
			 }
			  else if(tem_i == 3)
			 {
			  led_x = tem_x;
				led_y = tem_y-1;
			 }
			 else 
				 ;
			// 8 9 10 �� ��  4 5 6 ��
		 if((led_x > 9) || (led_x < 7) || (led_y > 5) || (led_y < 3))
				 condition = 0x00;
			 else 
				 condition = 0x01;
			 if(condition)
				{	 
					UART_send_string(USART2," condition1 ");
					tem_value = chessboard[led_x][led_y];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					led_x = led_x+1;
					led_y = led_y+1;
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(led_x,led_y,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
							CHESS_Light_xy_WS2812LED(led_x,led_y,2,0,0,CHESS_LIGHT_B); //����Ч��
					else;  
				}
		}
	}
	else ;
}

void Chess_Move_Ju(unsigned char ID)
{
	unsigned char tem_x,tem_y,tem_value,condition;
	unsigned char tem_max_rows =10,tem_max_clos=9;
	unsigned char tem_i,tem_j;
 
 
	struct PIECES pieces_tem;
	pieces_tem = chess_pieces[ID];
  
	// ���ɿ����ߵ�·��
	tem_x = pieces_tem.x-1 ;
	tem_y = pieces_tem.y-1 ;
	
	tem_i = tem_x;
	tem_j = tem_y;
	
	condition = check_xy(tem_x+1,tem_y);
	if(condition)
	{
		UART_send_string(USART2,"Ju condition1 \n");
		for(tem_i = tem_x+1;tem_i<tem_max_rows;tem_i++)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
					{
						CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
						break;
					}
					else
						break;
			}
			else
					break;
		}
	}
	
	tem_i = tem_x;
	tem_j = tem_y;
	condition = check_xy(tem_x-1,tem_y);
	if(condition)
	{
		UART_send_string(USART2,"Ju condition2 \n");
		for(tem_i = tem_x-1;tem_i>=0;tem_i--)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
					{
						CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
						break;
					}
					else
						break;
			}
			else
					break;
		}
	}
	tem_i = tem_x;
	tem_j = tem_y;
	condition = check_xy(tem_x,tem_y+1);
	if(condition)
	{
		UART_send_string(USART2,"Ju condition3 \n");
		for(tem_j = tem_y+1;tem_i<tem_max_clos;tem_j++)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
					{
						CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
						break;
					}
					else
						break;
			}
			else
					break;
		}
	}
	tem_i = tem_x;
	tem_j = tem_y;
	condition = check_xy(tem_x,tem_y-1);
	if(condition)
	{
		UART_send_string(USART2,"Ju condition4 \n");
		for(tem_j = tem_y-1;tem_i>=0;tem_j--)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if(tem_value == 0xff)
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
					{
						CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
						break;
					}
					else
						break;
			}
			else
					break;
		}
	}
 
	
}
void Chess_Move_Pao(unsigned char ID)
{
	unsigned char tem_x,tem_y,tem_value,condition;
	unsigned char tem_max_rows =10,tem_max_clos=9;
	unsigned char tem_i,tem_j;
//	unsigned char led_x,led_y;
  unsigned char tem_state =0; //������ҵ� ֧��
	struct PIECES pieces_tem;
	pieces_tem = chess_pieces[ID];
  
	// ���ɿ����ߵ�·��
	tem_x = pieces_tem.x-1 ;
	tem_y = pieces_tem.y-1 ;
	
	tem_i = tem_x;
	tem_j = tem_y;
	
	condition = check_xy(tem_x+1,tem_y);
	if(condition)
	{
		UART_send_string(USART2,"Pao condition 1 \n");
		for(tem_i = tem_x+1;tem_i<tem_max_rows;tem_i++)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if((tem_value == 0xff )&& (tem_state == 0x00))
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if( tem_value != 0xff)
					{
						 if(tem_state == 0x00) //��һ��ɨ�赽���������и�����
						 {
						   tem_state=0x01;
						 }
						 else if(tem_state == 0x01) //��2��ɨ�赽���������и�����
						 {
							 if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								{
									CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
									break;
								}
						 }
					}
					else
						;
			}
			else
					break;
		}
	}
	
	tem_i = tem_x;
	tem_j = tem_y;
	tem_state =0;
	condition = check_xy(tem_x-1,tem_y);
	if(condition)
	{
		UART_send_string(USART2,"Pao condition 2 \n");
		for(tem_i = tem_x-1;tem_i>=0;tem_i--)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if((tem_value == 0xff )&& (tem_state == 0x00))
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if( tem_value != 0xff)
					{
						 if(tem_state == 0x00) //��һ��ɨ�赽���������и�����
						 {
						   tem_state=0x01;
						 }
						 else if(tem_state == 0x01) //��2��ɨ�赽���������и�����
						 {
							 if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								{
									CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
									break;
								}
						 }
					}
					else
						;
			}
			else
					break;
		}
	}
	tem_i = tem_x;
	tem_j = tem_y;
	tem_state =0;
	condition = check_xy(tem_x,tem_y+1);
	if(condition)
	{
		UART_send_string(USART2,"Pao condition 3 \n");
		for(tem_j = tem_y+1;tem_j<tem_max_clos;tem_j++)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if((tem_value == 0xff )&& (tem_state == 0x00))
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if( tem_value != 0xff)
					{
						 if(tem_state == 0x00) //��һ��ɨ�赽���������и�����
						 {
						   tem_state=0x01;
						 }
						 else if(tem_state == 0x01) //��2��ɨ�赽���������и�����
						 {
							 if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								{
									CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
									break;
								}
						 }
					}
					else
						;
			}
			else
					break;
		}
	}
	
	tem_i = tem_x;
	tem_j = tem_y;
	tem_state =0;
	condition = check_xy(tem_x,tem_y-1);
	if(condition)
	{
		UART_send_string(USART2,"Pao condition 4 \n");
		for(tem_j = tem_y-1;tem_j>=0;tem_j--)
		{
			condition = check_xy(tem_i,tem_j);   
			if(condition)
			{      
					tem_value = chessboard[tem_i][tem_j];//����Ŀ��λ���ϵ�ID  ��û�������򷵻�0xff
					if((tem_value == 0xff )&& (tem_state == 0x00))
							CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,1,0,0,CHESS_LIGHT_B); //��Ч��
					else if( tem_value != 0xff)
					{
						 if(tem_state == 0x00) //��һ��ɨ�赽���������и�����
						 {
						   tem_state=0x01;
						 }
						 else if(tem_state == 0x01) //��2��ɨ�赽���������и�����
						 {
							 if(chess_pieces[tem_value].team != pieces_tem.team) //��ǰ�ƶ����ӵĶ��鲻����Ŀ�����ӵĶ��� ��ǿɳ������
								{
									CHESS_Light_xy_WS2812LED(tem_i+1,tem_j+1,2,0,0,CHESS_LIGHT_B); //����Ч��
									break;
								}
						 }
					}
					else
						;
			}
			else
					break;
		}
	}
 
}

unsigned char check_xy( int x,  int y)
{
	 unsigned char result;
	 if((x>=0) && (x<10) && (y>=0) && (y<9) )
 		  result =0x01;
	 else 
		 result =0x00;
	 
	 return result;
}

// ����ÿ�������ϵ�����
void Init_ChessBoard(void)
{
//��ÿһ�����ӽ��г�ʼ��
//�������Ͻǿ�ʼ   A����  ������ʿ˧ʿ��
// ����chessboard �洢��Ӧ��״̬
	
	unsigned char tem_i =0,tem_j;
	chess_laset_pose[0][0] =0;
	chess_laset_pose[0][1] = 	0;		
	chess_current_pose[0][0] =0;
	chess_current_pose[0][1] = 	0;
 
	chess_current_UpID = 0;  
	chess_current_Upflag = 0; 
	
	for(tem_i=0;tem_i<32;tem_i++)
	{
		  if(tem_i<16)
				chess_pieces[tem_i].team = CHESS_TEAM_A;
			else
				chess_pieces[tem_i].team = CHESS_TEAM_B;
			
			chess_pieces[tem_i].state = CHESS_STATE_LIVE;
	}
	for(tem_i=0;tem_i<10;tem_i++)
	{
		for(tem_j=0;tem_j<9;tem_j++)
		{
		 chessboard[tem_i][tem_j]=255;
		}

	}
	
  tem_i=0;
	chess_pieces[tem_i].type =CHESS_A_Ju;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =1; 
	//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Ma;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =2;
	//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Xiang;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =3;
  //chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Si;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =4; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Shuai;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =5; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Si;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =6; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Xiang;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =7; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Ma;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =8; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Ju;
	chess_pieces[tem_i].x =1; chess_pieces[tem_i].y =9; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Pao;
	chess_pieces[tem_i].x =3; chess_pieces[tem_i].y =2; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_Pao;
	chess_pieces[tem_i].x =3; chess_pieces[tem_i].y =8; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_BING;
	chess_pieces[tem_i].x =4; chess_pieces[tem_i].y =1; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_BING;
	chess_pieces[tem_i].x =4; chess_pieces[tem_i].y =3; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_BING;
	chess_pieces[tem_i].x =4; chess_pieces[tem_i].y =5; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_BING;
	chess_pieces[tem_i].x =4; chess_pieces[tem_i].y =7; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_A_BING;
	chess_pieces[tem_i].x =4; chess_pieces[tem_i].y =9;
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_BING;
	chess_pieces[tem_i].x =7; chess_pieces[tem_i].y =1; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_BING;
	chess_pieces[tem_i].x =7; chess_pieces[tem_i].y =3; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_BING;
	chess_pieces[tem_i].x =7; chess_pieces[tem_i].y =5; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_BING;
	chess_pieces[tem_i].x =7; chess_pieces[tem_i].y =7; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_BING;
	chess_pieces[tem_i].x =7; chess_pieces[tem_i].y =9;
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Pao;
	chess_pieces[tem_i].x =8; chess_pieces[tem_i].y =2; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Pao;
	chess_pieces[tem_i].x =8; chess_pieces[tem_i].y =8; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type =CHESS_B_Ju;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =1; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Ma;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =2; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Xiang;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =3; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Si;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =4;
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;	
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Shuai;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =5; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Si;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =6; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Xiang;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =7; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Ma;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =8; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	tem_i++;
	chess_pieces[tem_i].type = CHESS_B_Ju;
	chess_pieces[tem_i].x =10; chess_pieces[tem_i].y =9; 
//chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=chess_pieces[tem_i].type;
	chessboard[chess_pieces[tem_i].x-1][chess_pieces[tem_i].y-1]=tem_i;
	
	UART_send_string(USART2,"chessboard tem_i: \n"); UART_send_data(USART2,tem_i);			
	if(tem_i == 31)
	{
		UART_send_string(USART2,"chessboard state: \n"); 
		for(tem_i=0;tem_i<10;tem_i++)
		{
			for(tem_j=0;tem_j<9;tem_j++)
			{
             
					UART_send_string(USART2,"  ");  UART_send_data(USART2,chessboard[tem_i][tem_j]);			
			}
		UART_send_string(USART2,"\n"); 
		}
 
	}
	
	
}







// ����LED���Ƴ���
void CHESS_TEST_LED_1(void)
{
	uint32_t delay_time = 20000;
	unsigned char tem_mode =1;
	unsigned char tem_R=0,tem_G=0,tem_B=0x05;
	CHESS_Clear_XY_WS2812LED();
	CHESS_Light_yCols_WS2812LED(1,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(2,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(3,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(4,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(5,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(6,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(7,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(8,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_yCols_WS2812LED(9,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
 
}
void CHESS_TEST_LED_2(void)
{
	uint32_t delay_time = 20000;
	unsigned char tem_mode =2;
	unsigned char tem_R=0,tem_G=0x05,tem_B=0;
	CHESS_Clear_XY_WS2812LED();
	CHESS_Light_xRows_WS2812LED(1,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(2,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(3,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(4,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(5,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(6,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(7,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(8,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(9,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
	CHESS_Light_xRows_WS2812LED(10,tem_mode,tem_R,tem_G,tem_B);
	WS2812_RGB_All();
	Delay_10us(delay_time); 
}


// ����ָ������λ�õ�LED�� (x,y)����
// mode = 1 �������ĵ�ĵ�
// mode =2  �������ε�
// mode =3   �������ĺͻ��ε�
void CHESS_Light_xy_WS2812LED(unsigned char x,unsigned char y,unsigned char mode,
	                      unsigned char R,unsigned char G,unsigned char B)
{
	int ID =0;
	int tem_inde =0;
	//unsigned char step =0,continiutime=0;
  if(x == 6) x=10;
	else if(x == 7) x=9;
	else if(x == 9) x=7;
	else if(x == 10) x=6;
	
	if(x%2 == 1) //����
	{
	  ID = (x-1)*9+y-1;
	}
	else
	{
		ID = x*9-y;
	}
	
	if(mode == 1)
	{
	  tem_inde=ID*2+1;
		if(tem_inde > 180) tem_inde=180;
		else if(tem_inde < 0) tem_inde=0;
		else ;
		WS2812_datas[tem_inde][0] = R;
		WS2812_datas[tem_inde][1] = G;
		WS2812_datas[tem_inde][2] = B;
		
	}
	else if (mode == 2)
	{
		tem_inde=ID*2;
		if(tem_inde > 180) tem_inde=180;
		else if(tem_inde < 0) tem_inde=0;
		else ;
		WS2812_datas[tem_inde][0] = R;
		WS2812_datas[tem_inde][1] = G;
		WS2812_datas[tem_inde][2] = B;
	}
	else if (mode == 3)
	{
		tem_inde=ID*2;
		if(tem_inde > 180) tem_inde=180;
		else if(tem_inde < 0) tem_inde=0;
		else ;
		WS2812_datas[tem_inde][0] = R;
		WS2812_datas[tem_inde][1] = G;
		WS2812_datas[tem_inde][2] = B;
		WS2812_datas[tem_inde+1][0] = R;
		WS2812_datas[tem_inde+1][1] = G;
		WS2812_datas[tem_inde+1][2] = B;
	}
  //�������滻��
}
// ����һ��
void CHESS_Light_xRows_WS2812LED(unsigned char x,unsigned char mode,
	                         unsigned char R,unsigned char G,unsigned char B)
{
	
	CHESS_Light_xy_WS2812LED(x,1,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,2,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,3,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,4,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,5,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,6,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,7,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,8,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(x,9,mode,R,G,B);
}
// ����һ��
void CHESS_Light_yCols_WS2812LED(unsigned char y,unsigned char mode,
	                         unsigned char R,unsigned char G,unsigned char B)
{
	
	CHESS_Light_xy_WS2812LED(1,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(2,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(3,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(4,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(5,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(6,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(7,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(8,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(9,y,mode,R,G,B);
	CHESS_Light_xy_WS2812LED(10,y,mode,R,G,B);
}

void CHESS_Clear_xy_WS2812LED(unsigned char x,unsigned char y)
{
	   CHESS_Light_xy_WS2812LED(x,y,3,0,0,0);
}
void CHESS_Clear_XY_WS2812LED(void)
{
	 unsigned char col_inde,row_index;
		for(row_index=0;row_index<10;row_index++)
		{
			for(col_inde=0;col_inde<9;col_inde++)
			{
				 CHESS_Light_xy_WS2812LED(row_index+1,col_inde+1,3,0,0,0);
			}
		}
}




