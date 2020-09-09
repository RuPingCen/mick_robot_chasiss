/******************************************************************************************************
#2019-10-07
	1、添加ROS节点下发命令清零里程计数据功能
	
#2020-9-8
	1、更新了DBUS中的函数名称
	2、更新遥控器信号丢失造成的数据乱码引起“疯转”的问题
	3、统一4轮和2轮差速小车模型电机控制函数的单位为 m/s  和 rad/s      
	
	
* update 2019-10-02
* maker: crp
******************************************************************************************************/
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
volatile uint32_t Timer2_Counter4=0;
volatile uint32_t Timer2_Counter5=0; 

extern int set_v,set_spd[4];  //底盘四个电机目标速度
extern command_t recived_cmd; //底盘接收上位机命令结构体

extern CanRxMsg RxMessage;				               //接收缓冲区
extern uint8_t USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节
extern rc_info_t dbus_rc;               //DBUS 变量
extern moto_measure_t moto_chassis[4] ; //CAN读取电机状态数据变量 
extern moto_measure_t moto_info;

/*********************************************************/
/**********************函数声明***************************/
void IMU_Test_Upload_Message(void);
void S_Test_Upload_Message(void);
 
 
void Main_Delay(unsigned int delayvalue);
void Main_Delay_us(unsigned int delayvalue);
/*********************************************************/
/*****************主函数**********************************/
int main(void)
{	
 
		 SysTick_Init();//初始化滴答定时器
 
	   My_GPIO_Init(GPIOE,GPIO_Pin_5,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);//初始化LED端口
	   My_GPIO_Init(GPIOE,GPIO_Pin_6,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);//初始化LED端口
	   //My_GPIO_Init(GPIOF,GPIO_Pin_8,GPIO_Mode_Out_OD, GPIO_Speed_10MHz);//初始化LED端口
	
	   My_Config_USART_Init(USART2,115200,1);	    
	   UART_send_string(USART2,"MickX4 ROS Car \n");//字符串函数
	
		 USART_DMA_Rec_Config(USART1,100000); //开启串口1 DMA接收方式

     CAN_Config();//初始化can模块
	
		 DJI_Motor_Init();//初始化PID模块
	
		 Mecanum_Wheel_Speed_Model(0,0,0); //开机以后发送设置电机目标值为0
		 	   
		 Timer_2to7_counter_Generalfuncation(TIM2,1000);//1ms
		 Timer_2to7_Generalfuncation_start(TIM2);
		 
		 //Timer_2to7_counter_Generalfuncation(TIM3,50000);//500ms
		 //Timer_2to7_Generalfuncation_start(TIM3);
     UART_send_string(USART2,"Init Successful ....\n");//字符串函数
		   
		 while(1)
			{
					//if(Timer2_Counter4 == 10) //1ms*10  100HZ 打印频率
					//{			 
						//UART_send_string(USART2,"IMU");
						//IMU_Test_Upload_Message();
						//Timer2_Counter4=0;
						//Main_Delay_us(800);	
					//}
//					if(Timer2_Counter4 >= 50) //1ms*50  20HZ 打印频率
//					{
//						//UART_send_string(USART2,"Supers");
//						S_Test_Upload_Message();
//						Timer2_Counter4=0;
//						//Main_Delay_us(800);	
//					}
							
						if(UART1_DMA_Flag) //遥控器介入控制命令逻辑  7ms 发送一次
						{									
							UART1_DMA_Flag=0x00;	
							if((dbus_rc.sw1 !=1) && (dbus_rc.available)) //使能遥控器模式
							{
								Timer2_Counter1=0; //清空定时计数器
								if(dbus_rc.sw2 ==1) //1 档模式 最大1m/s
								{
									DiffX4_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.00152,(dbus_rc.ch1_offset-dbus_rc.ch1)*0.00152);
								}
								else if(dbus_rc.sw2 ==3) //2 档模式 最大2m/s
								{
									DiffX4_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.00304,(dbus_rc.ch1_offset-dbus_rc.ch1)*0.00304);
								}
								else if(dbus_rc.sw2 ==2) //3 档模式 最大3.5m/s
								{
									DiffX4_Wheel_Speed_Model((dbus_rc.ch2-dbus_rc.ch2_offset)*0.0053,(dbus_rc.ch1_offset-dbus_rc.ch1)*0.0053);
								}
								else
									DiffX4_Wheel_Speed_Model(0,0);
							}
							LED1_FLIP;
						}
						
						if(UART2_Flag && recived_cmd.flag) //上位机指令下发接口
						{
								if((dbus_rc.sw1 ==1) && (dbus_rc.available))//串口接收有数据过来
								{
									Timer2_Counter1=0; //清空定时计数器
									if(recived_cmd.cmd == 0xF1)
									{	
										Mecanum_Wheel_Rpm_Model(recived_cmd.tag_rpm1,recived_cmd.tag_rpm2,recived_cmd.tag_rpm3,recived_cmd.tag_rpm4);
									}
									else if(recived_cmd.cmd == 0xF2)
									{
										Mecanum_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_y,recived_cmd.tag_speed_z);
									}
									else if(recived_cmd.cmd == 0xF3)
									{
										DiffX4_Wheel_Speed_Model(recived_cmd.tag_speed_x,recived_cmd.tag_speed_z);
									}
									else if(recived_cmd.cmd == 0xE1) //里程计清零
									{
										 DJI_Motor_Clear_Odom();
									}
									else;
									//Delay_10us(50000);
									//DJI_Motor_Show_Message();
								}
								else
									Mecanum_Wheel_Rpm_Model(0,0,0,0);
					 
							recived_cmd.flag =0;//使用一次以后丢弃该数据
							UART2_Flag=0x00;
						}
						  
						if((Timer2_Counter1>100*10)) // 如果定时计数器操作4s还没有被清零，说明通讯出现了中断
						{			
									if(dbus_rc.available == 0x00)
									{
										dbus_rc.sw1 = 5; //标记接收数据不可用
										Mecanum_Wheel_Rpm_Model(0,0,0,0);
									}
									else if(recived_cmd.flag ==0x00)
									{
										Mecanum_Wheel_Rpm_Model(0,0,0,0);
									}
									Timer2_Counter1=0;
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
						}
      }
     
 // exit from main() function
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

// 测试 ros接收IMU数据
void IMU_Test_Upload_Message(void)
{
		short IMU_test=1;
		static  uint32_t IMU_upload_counter=0;
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	

		short ax=IMU_test+1;
		short ay=IMU_test+2;
		short az=IMU_test+3;
		short gx=IMU_test+4;
		short gy=IMU_test+5;
		short gz=IMU_test+6;
		short mx=IMU_test+7;
		short my=IMU_test+8;
		short mz=IMU_test+9;
		short pitch = IMU_test + 10;
		short roll=IMU_test+11;
		short yaw=IMU_test+12;

		pitch = pitch*100;
		roll=roll*100;
		yaw=yaw*100;
			
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0x10;
	
	  //上传数据帧计数
		senddat[i++]=(IMU_upload_counter>>24);
		senddat[i++]=(IMU_upload_counter>>16);
		senddat[i++]=(IMU_upload_counter>>8);
		senddat[i++]=(IMU_upload_counter);
			
		senddat[i++] = ax>>8; //int16
		senddat[i++] = ax;
		senddat[i++] = ax>>8; 
		senddat[i++] = ax;
		senddat[i++] = ay>>8; 
		senddat[i++] = ay;
		senddat[i++] = az>>8; 
		senddat[i++] = az;
		senddat[i++] = gx>>8; 
		senddat[i++] = gx;
		senddat[i++] = gy>>8; 
		senddat[i++] = gy;
		senddat[i++] = gz>>8; 
		senddat[i++] = gz;
		senddat[i++] = mx>>8; 
		senddat[i++] = mx;
		senddat[i++] = my>>8; 
		senddat[i++] = my;
		senddat[i++] = mz>>8; 
		senddat[i++] = mz;
		senddat[i++] = pitch>>8; 
		senddat[i++] = pitch;
		senddat[i++] = roll>>8; 
		senddat[i++] = roll;
		senddat[i++] = yaw>>8; 
		senddat[i++] = yaw;

		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
    senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		IMU_upload_counter++;
}

// 测试 ros接收超声波数据
void S_Test_Upload_Message(void)
{
	  uint16_t S_test=1;
	  static  uint32_t S_upload_counter=0;
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	
	  unsigned char z;
	
	   u16 S_data[12]={0};
		for(z=0;z<12;z++){
			S_data[z]=S_test+2;
		}
			
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//数据长度在后面赋值
		senddat[i++]=0x11;
	
	  //上传数据帧计数
		senddat[i++]=(S_upload_counter>>24);
		senddat[i++]=(S_upload_counter>>16);
		senddat[i++]=(S_upload_counter>>8);
		senddat[i++]=(S_upload_counter);
	
		for(j=0;j<12;j++){
			senddat[i++] = S_data[j]>>8; //int16
			senddat[i++] = S_data[j];
		}
		
		
		senddat[2]=i-1; //数据长度
		for(j=2;j<i;j++)
			sum+=senddat[j];
      senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		S_upload_counter++;
}
// --------------------------------------------------------------//
/***************************END OF FILE**********************/
