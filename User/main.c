/******************************************************************************************************
#2021-4-18
	1、将DBUS、上位机发送的命令移到中断函数中进行处理
	2、将PID相关计算函数移到PID.c中
  3、添加MPU9250 读取和姿态计算函数 (由于中断资源不能长时间被占用，因此暂时放在了while中)
	4、在mick robot controller V1.0.0板子上测试通过
	5、LED1 指示程序是否正常运行（20HZ闪烁） 
		 LED2指示遥控器是否有数据
		 LED3指示CAN总线是否有数据

#2020-9-21 
	1、添加了MPU9250的数据读取 姿态解算函数
	2、目前MPU9250使用的端口为PB10  PB11
  3、在小车1.0的板子上测试通过了，可正确读取数据。2.0的通讯板串口有问题
	
#2020-9-8
	1、更新了DBUS中的函数名称
	2、更新遥控器信号丢失造成的数据乱码引起“疯转”的问题
	3、统一4轮和2轮差速小车模型电机控制函数的单位为 m/s  和 rad/s   
	
#2019-10-07
	1、创建麦克纳姆轮PID控制函数
	2、增加与ROS节点通讯接口
	3、增加DMA方式接收DBUS遥控器
	
* update 2020-9-21
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
#include "MPU9250.h"
#include "IMU.h"

#include "Mick_IO.h"
#include "Seven_Lab_MiniIMU.h"

/*********************************************************/
/**********************宏定义****************************/
#define EnableInterrupt 	__set_PRIMASK(0) //总中断 写‘0‘ 开总中断   ’1‘关总中断
#define DisableInterrupt 	__set_PRIMASK(1) //总中断 写‘0‘ 开总中断   ’1‘关总中断
uint32_t TimingDelay; //用于delay控制的精确延时
uint16_t ADC_ConvertedValue; 
 
#define DEBUUG 1
#define DEBUUG_MATRIX_KEYSACN 0


 
/*********************************************************/
/******************全局变量声明***************************/

volatile uint8_t UART1_DMA_Flag=0x00;
volatile uint8_t UART1_DMA_Flag2=0x00;
volatile uint8_t UART2_Flag=0x00;
volatile uint8_t CAN1_Flag=0x00;
volatile uint8_t TIM3_Flag=0x00;

volatile uint32_t Timer2_Counter1=0; //分别用来标记接收命令是否超过了时间限制范围
volatile uint32_t Timer2_Counter2=0;
volatile uint32_t Timer2_Counter3=0; 
volatile uint32_t Timer2_Counter4=0;
volatile uint32_t Timer2_Counter5=0; 

 

extern command_t recived_cmd; //底盘接收上位机命令结构体
extern uint8_t Code_Switch_Value;

/*********************************************************/
/**********************函数声明***************************/
void Main_Delay(unsigned int delayvalue);
void Main_Delay_us(unsigned int delayvalue);
void Test_Mick_GPIO(void);


/*********************************************************/
/*****************主函数**********************************/
int main(void)
{	
	SysTick_Init();//初始化滴答定时器
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	Init_Mick_GPIO();

	USART_DMA_Rec_Config(USART1,100000); //开启串口1 DMA接收方式
	My_Config_USART_Init(USART2,115200,1);	
 	My_Config_USART_Init(USART3,115200,1);	
	UART_send_string(USART2,"MickX4 ROS Car \n");//字符串函数
	UART_send_string(USART3,"MickX4 ROS Car \n");//字符串函数	
	UART_send_string(USART2,"Start Init MPU9250 ... \n");//字符串函数
	MPU9250_Init();//MPU6050初始化 ± 2000 °/s ± 4g  5hz
	UART_send_string(USART2,"Successed Init MPU9250 !\n"); 
	
	CAN_Config();//初始化can模块
	DJI_Motor_Init();//初始化3508模块
	Mecanum_Wheel_Speed_Model(0,0,0); //开机以后发送设置电机目标值为0

	Code_Switch_Value = Read_Code_Switch(); //设置小车模型
	if(Code_Switch_Value == 0x01)
	{
		UART_send_string(USART2,"The differential model is used. \n"); 
	}
	else if(Code_Switch_Value == 0x02)
	{
		UART_send_string(USART2,"The mecanum model is used. \n"); 
	}
	else
	{
		UART_send_string(USART2,"The car model is error. \n"); 
	}
	
	Set_Isolated_Output(1,0);// 设置所有输出为高阻输出
	Set_Isolated_Output(2,0);			
	Set_Isolated_Output(3,0);
	Set_Isolated_Output(4,0);
		
	
	Timer_2to7_counter_Generalfuncation(TIM2,1000);//1ms
	Timer_2to7_Generalfuncation_start(TIM2);
	
	//Timer_2to7_counter_Generalfuncation(TIM3,5000);//5ms
	//Timer_2to7_Generalfuncation_start(TIM3);
	UART_send_string(USART2,"Init Successful ....\n");//字符串函数
		   
	while(1)
	{
		if(Timer2_Counter2 > 50) //状态指示，显示程序正常运行
		{
			LED1_FLIP;
			Timer2_Counter2=0;
		}

		if(CAN1_Flag)  //CAN总线中断
		{
			CAN1_Flag=0x00;  
		}
		
		if(UART1_DMA_Flag) //遥控器介入控制命令逻辑  7ms 发送一次
		{	
			UART1_DMA_Flag2++;
			if(UART1_DMA_Flag2>2)
			{
				RC_Upload_Message();//上传遥控器状态
				UART1_DMA_Flag2=0;
			}			
			UART1_DMA_Flag=0x00;	
		}
		
		if(UART2_Flag && recived_cmd.flag) //上位机指令下发接口
		{
			//------------------------- 非实时线程 --------------------//
			
			//---------------------------------------------------------//
			recived_cmd.flag =0;//使用一次以后丢弃该数据
			UART2_Flag=0x00;
		}
	
		if(Timer2_Counter3 > 8) //1ms*8  125 HZ 上传电机数据
		{
			Timer2_Counter3=0;
			DJI_Motor_Upload_Message();
			//DJI_Motor_Show_Message();
		}
		if(Timer2_Counter4 > 10) //1ms*10  100HZ 打印频率
		{			 
			IMU_Routing();
			IMU_Upload_Message();
			//IMU_Report_AHRSdata();
			Timer2_Counter4=0;
		}
		if(Timer2_Counter5 > 100) //1ms*100  10HZ 打印频率
		{			 
			Isolated_IO_Upload_Message(); 
			Timer2_Counter5=0;
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

void Test_Mick_GPIO(void)
{
	uint8_t code_switch_value=0;
	while(1)
	{
		if(Read_Key(1))
		{
			UART_send_string(USART2,"key1 pressed ...\n"); 
			Set_Isolated_Output(1,1);
			Set_Isolated_Output(2,1);
		}
		else
		{
			Set_Isolated_Output(1,0);
			Set_Isolated_Output(2,0);
		}
		if(Read_Key(2))
		{
			UART_send_string(USART2,"key2 pressed ...\n"); 
			Set_Isolated_Output(3,1);
			Set_Isolated_Output(4,1);
		}
		else
		{
			Set_Isolated_Output(3,0);
			Set_Isolated_Output(4,0);
		}
		
		code_switch_value=0;
		code_switch_value = Read_Code_Switch();
		if(code_switch_value)
		{
			UART_send_string(USART2,"Code Switch value: "); 
			UART_send_data(USART2,code_switch_value);
			UART_send_string(USART2," \n"); 
		}
		
		if(Read_Isolated_Input(1))
		{
			UART_send_string(USART2,"Isolated channel 1 input VCC ...\n"); 
		}
		
		if(Read_Isolated_Input(2))
		{
			UART_send_string(USART2,"Isolated channel 2 input VCC ...\n"); 
		}
		
		if(Read_Isolated_Input(3))
		{
			UART_send_string(USART2,"Isolated channel 3 input VCC ...\n"); 
		}

		if(Read_Isolated_Input(4))
		{
			UART_send_string(USART2,"Isolated channel 4 input VCC ...\n"); 
		}
 
		
		LED1_FLIP;
		LED2_FLIP;
		LED3_FLIP;
		
		Main_Delay(200);
	}
}
// --------------------------------------------------------------//
/***************************END OF FILE**********************/
