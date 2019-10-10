/*
*            
*
* update 2019-10-02
* maker: crp
*/
#include "stm32f10x.h"
#include "stdio.h"//包含串口发送的FILE属性
#include "stdlib.h"

#include "stm32f10x_Delay.h"
 
// #include "speed_cntr.h"
#include "speed_control.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "DBUS.h"
#include "DJI_Motor.h"
/*********************************************************/
/**********************宏定义****************************/
#define EnableInterrupt 	__set_PRIMASK(0) //总中断 写‘0‘ 开总中断   ’1‘关总中断
#define DisableInterrupt 	__set_PRIMASK(1) //总中断 写‘0‘ 开总中断   ’1‘关总中断
uint32_t TimingDelay; //用于delay控制的精确延时
uint16_t ADC_ConvertedValue; 
 
#define DEBUUG 1
#define DEBUUG_MATRIX_KEYSACN 0

#define LED1_FLIP  GPIO_Flip_level(GPIOE,GPIO_Pin_5) 
#define LED2_FLIP  GPIO_Flip_level(GPIOE,GPIO_Pin_6) 
#define LED3_FLIP  GPIO_Flip_level(GPIOF,GPIO_Pin_8) 
 
/*********************************************************/
/******************全局变量声明***************************/

volatile uint8_t UART1_DMA_Flag=0x00;
volatile uint8_t UART2_Flag=0x00;
volatile uint8_t CAN1_Flag=0x00;

volatile uint32_t Timer2_Counter1=0; //分别用来标记接收命令是否超过了限制范围
volatile uint32_t Timer2_Counter2=0;
volatile uint32_t Timer2_Counter3=0; 


extern int set_v,set_spd[4];  //底盘四个电机目标速度
extern command_t recived_cmd; //底盘接收上位机命令结构体

extern CanRxMsg RxMessage;				 //接收缓冲区
extern uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节
extern rc_info_t dbus_rc;               //DBUS 变量
extern moto_measure_t moto_chassis[4] ; //CAN读取电机状态数据变量 
extern moto_measure_t moto_info;
/*********************************************************/
/**********************函数声明***************************/
	
 
 
void control(void);
void Main_Delay(unsigned int delayvalue);
void Main_Delay_us(unsigned int delayvalue);
/*********************************************************/
/*****************主函数**********************************/
int main(void)
{	
	   unsigned int i=0;
     
	   
		 SysTick_Init();//初始化滴答定时器
	
	   //由于使用了PB3引脚，因此将JTAG的引脚重定义了
	   //RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_AFIO,ENABLE);
	   //GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); 
	
	   My_GPIO_Init(GPIOE,GPIO_Pin_5,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);//初始化LED端口
	   My_GPIO_Init(GPIOE,GPIO_Pin_6,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);//初始化LED端口
	   //My_GPIO_Init(GPIOF,GPIO_Pin_8,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);//初始化LED端口
	
	   My_Config_USART_Init(USART2,115200,1);	    
	   UART_send_string(USART2,"this is a test project！ \n");//字符串函数
	
		 USART_DMA_Rec_Config(USART1,100000); //开启串口1 DMA接收方式

     CAN_Config();
		 DJI_Motor_Init();
     //CAN_DJI_C620_DataSend(10,10,10,10);
		 Mecanum_Wheel_Speed_Model(0,0,0);
		 	   
		 Timer_2to7_counter_Generalfuncation(TIM2,1000);//1ms
		 Timer_2to7_Generalfuncation_start(TIM2);
		 
		 //Timer_2to7_counter_Generalfuncation(TIM3,50000);//500ms
		 //Timer_2to7_Generalfuncation_start(TIM3);
     UART_send_string(USART2,"Init Successful ....\n");//字符串函数
		 while(1)
			{
					
 
						if(UART1_DMA_Flag) //遥控器介入控制命令逻辑
						{									
							UART1_DMA_Flag=0x00;
							Timer2_Counter1=0; //清空定时计数器
							control();
								LED1_FLIP;
						  //rc_show_message();
						}
						if(UART2_Flag) //上位机指令下发接口
						{
							   
							  Timer2_Counter2=0; //清空定时计数器
							  UART2_Flag=0x00;
						}
					
						if(CAN1_Flag) //打印CAN中断接收的数据
						{
							CAN1_Flag=0x00;  
							if(Timer2_Counter3 == 20) //1ms*20  50HZ 打印频率
							{
								LED2_FLIP;
								 Timer2_Counter3=0;
								DJI_Motor_Upload_Message();
								//DJI_Motor_Show_Message();
							}
							else if(Timer2_Counter3 > 20)  
							{
								 Timer2_Counter3=0;
							}
//							for(i=0;i<4;i++)
//							{
//								// 打印四个电机的转速、转角、温度等信息
//								UART_send_string(USART2,"M"); UART_send_data(USART2,i);UART_send_string(USART2,"： ");
//								UART_send_string(USART2,"v:");UART_send_floatdat(USART2,moto_chassis[i].speed_rpm);UART_send_char(USART2,'\t');	
//								UART_send_string(USART2,"t_a:");UART_send_floatdat(USART2,moto_chassis[i].total_angle);UART_send_char(USART2,'\t');	
//								UART_send_string(USART2,"n:");UART_send_floatdat(USART2,moto_chassis[i].round_cnt);UART_send_char(USART2,'\t');	
//								UART_send_string(USART2,"a:");UART_send_floatdat(USART2,moto_chassis[i].angle);UART_send_char(USART2,'\n');	
//							}
//						  UART_send_char(USART2,'\n');
						}
						
						if((Timer2_Counter2>100*10) ) // 如果定时计数器操作4s还没有被清零，说明通讯出现了中断
						{			     
									recived_cmd.flag =0; //标记接收数据不可用
									Timer2_Counter2=0;
							    //Mecanum_Wheel_Speed_Model(0,0,0);
						}
						if((Timer2_Counter1>100*10) ) // 如果定时计数器操作4s还没有被清零，说明通讯出现了中断
						{			     
									dbus_rc.sw1  =5; //标记接收数据不可用
							    Timer2_Counter1=0;
									Mecanum_Wheel_Speed_Model(0,0,0);
						}
				
      }
      Mecanum_Wheel_Speed_Model(0,0,0);
 // exit from main() function
}
// 主控制逻辑
void control(void)
{
	if(dbus_rc.sw1 !=1 )
	{
		if(dbus_rc.sw1 ==3)
		{
			//Mecanum_Wheel_Speed_Model((dbus_rc.ch4-1024)*5,(1027-dbus_rc.ch1)*5,(1024-dbus_rc.ch3)*2);
			Mecanum_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*3,(dbus_rc.ch1_offset-dbus_rc.ch1)*3,(dbus_rc.ch3_offset-dbus_rc.ch3)*1);
		}
		else if(dbus_rc.sw1 ==2)
		{
			//Mecanum_Wheel_Speed_Model((dbus_rc.ch4-1024)*5,(1027-dbus_rc.ch1)*5,(1024-dbus_rc.ch3)*2);
			Mecanum_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*6,(dbus_rc.ch1_offset-dbus_rc.ch1)*5,(dbus_rc.ch3_offset-dbus_rc.ch3)*2);
		}
		else
			Mecanum_Wheel_Speed_Model(0,0,0);
	}
	else if(dbus_rc.sw1 ==1 )
	{
		if(recived_cmd.flag) //串口接收有数据过来
		{
			if(recived_cmd.cmd == 0xF1)
			{	
					Mecanum_Wheel_Rpm_Model(recived_cmd.tag_rpm1,recived_cmd.tag_rpm2,recived_cmd.tag_rpm3,recived_cmd.tag_rpm4);
			}
			else if(recived_cmd.cmd == 0xF2)
			{
					Mecanum_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_y,recived_cmd.tag_speed_z);
			}
			else if(recived_cmd.cmd == 0xE1) //里程计清零
			{
					 DJI_Motor_Clear_Odom();
			}
			else;
			//Delay_10us(50000);
			//DJI_Motor_Show_Message();
			//recived_cmd.flag =0;//使用一次以后丢弃该数据
		}
		else
			Mecanum_Wheel_Speed_Model(0,0,0);
			
	}
	else;
//	{
//			Mecanum_Wheel_Speed_Model(0,0,0);
//	}

}
 
 //延时函数 6.3ms
void Main_Delay(unsigned int delayvalue)
{
	unsigned int i;
	while(delayvalue-->0)
	{	
		i=5000;
		while(i-->0);
	}
}
void Main_Delay_us(unsigned int delayvalue)
{
//	unsigned int i;
	while(delayvalue-->0)
	{	
		;
	}
}
// --------------------------------------------------------------//
/***************************END OF FILE**********************/
