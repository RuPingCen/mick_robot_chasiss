/******************************************************************************************************
#2023-11-04
	1、增加mpu6050 DMP解算，在while循环中读取IMU数据
		// 在mpu6050.h中增加函数
			extern uint8_t MPU6050_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);//写多字节
			extern uint8_t MPU6050_IIC_Wait_Ack(void);//等待ACK信号
			extern uint8_t MPU6050_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);//读多字节
		// IO_IIC.h中增加函数
			unsigned char IO_IIC_read_byte(unsigned char ack);//read a byte//从I2C总线接收一个字节数据
			void SDA_DIR_IN(void);//设置SDA方向为输入
			void SDA_DIR_OUT(void);//设置SDA方向为输出
#2022-4-22
	1、增加 SBUS 函数遥控器解码功能 
	2、增加 SBUS 部分代码注释

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
#include "Mick_IO.h"

#include "inv_mpu.h"
#include "IMU.h"
//#include "Seven_Lab_MiniIMU.h"
//#include "MPU9250.h"
//#include "mpu6050.h"
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
volatile uint8_t IMU_Init_Flag=0x00;

volatile uint32_t Timer2_Counter1=0; //分别用来标记接收命令是否超过了时间限制范围
volatile uint32_t Timer2_Counter2=0;
volatile uint32_t Timer2_Counter3=0; 
volatile uint32_t Timer2_Counter4=0;
volatile uint32_t Timer2_Counter5=0; 

 

extern command_t recived_cmd; //底盘接收上位机命令结构体
extern uint8_t Code_Switch_Value;
extern rc_info_t rc;  // remote command
/*********************************************************/
/**********************函数声明***************************/
void Main_Delay(unsigned int delayvalue);
void Main_Delay_us(unsigned int delayvalue);
void Test_Mick_GPIO(void);
void Test_MPU6050(void);

/*********************************************************/
/*****************主函数**********************************/
int main(void)
{	

	uint8_t main_counter = 0;
	unsigned char dma_lendat = 18;
	SysTick_Init();//初始化滴答定时器
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	//step1 初始化IO ---------------------------------------------
	Init_Mick_GPIO();
	Set_Isolated_Output(1,0);// 设置所有输出为高阻输出
	Set_Isolated_Output(2,0);			
	Set_Isolated_Output(3,0);
	Set_Isolated_Output(4,0);	
	
	//step2 初始化串口2/3 -----------------------------------------
	My_Config_USART_Init(USART2,115200,1);// 串口初始化函数 波特率115200	 与工控机通讯
 	My_Config_USART_Init(USART3,115200,1);	
	Delay_10us(200);
	UART_send_string(USART2,"\n\n MickX4 ROS Car \n");//字符串函数
	UART_send_string(USART3,"\n\n MickX4 ROS Car \n");//字符串函数	

	//step3 初始化IMU ---------------------------------------------
	UART_send_string(USART2,"Start Init MPU6050 ... \n");
 
	while (mpu_dmp_init() && main_counter<10)
	{
		UART_send_string(USART2,"MPU6050 ReInit... \r\n");
		Delay_10us(9000);
		main_counter++;
	}
	if(main_counter<10)
	{
		IMU_Init_Flag = 0x01;
		UART_send_string(USART2,"MPU6050 Start Success   \r\n");
	}
	else
	{
		IMU_Init_Flag = 0x00;
		UART_send_string(USART2,"MPU6050 Init Failed   \r\n");
	}

	 
	//step4 初始化CAN ---------------------------------------------
	CAN_Config();//初始化can模块
	DJI_Motor_Init();//初始化3508模块
	Mecanum_Wheel_Speed_Model(0,0,0); //开机以后发送设置电机目标值为0
	
	//step5 设置小车运动学模型 -------------------------------------
	Code_Switch_Value = Read_Code_Switch(); //拨码开关低两位设置底盘的运动学模型
	if((Code_Switch_Value & 0x03) == 0x00)
	{
		UART_send_string(USART2,"The differential model is used. \n"); 
	}
	else if((Code_Switch_Value & 0x03) == 0x01)
	{
		UART_send_string(USART2,"The mecanum model is used. \n"); 
	}
	else if((Code_Switch_Value & 0x03) == 0x02)
	{
		UART_send_string(USART2,"The 4WS4WD model is used. \n"); 
	}
	else if((Code_Switch_Value & 0x03) == 0x03)
	{
		UART_send_string(USART2,"The Ackermann model is used. \n"); 
	}
	
	//step6 设置遥控器类型 -------------------------------------
	Code_Switch_Value = 0x00;
	Code_Switch_Value = Read_Code_Switch(); 
	if((Code_Switch_Value & 0x04) == 0x00) // 获取第3位开关状态
	{
		UART_send_string(USART2,"SBUS (for T8FB)model is used. \n"); 
		rc.type =2; // SBUS模型     乐迪协议
		dma_lendat = 25;
	}
	else
	{
		UART_send_string(USART2,"DBUS (for DJI DT7) model is used. \n"); 
		rc.type =1; // 默认为DBUS模型  大疆协议
		dma_lendat = 18;
	}
	USART_DMA_Rec_Config(USART1,100000,dma_lendat); //开启串口1 DMA接收方式  用于接收遥控器信号
	
	//step7 初始化定时器 -------------------------------------	
	Timer_2to7_counter_Generalfuncation(TIM2,1000);//1ms
	Timer_2to7_Generalfuncation_start(TIM2);
	
	//Timer_2to7_counter_Generalfuncation(TIM3,5000);//5ms
	//Timer_2to7_Generalfuncation_start(TIM3);
	UART_send_string(USART2,"Init Successful !!! \n\n");//字符串函数
		   
	while(1)
	{
		if(Timer2_Counter2 > 100) //状态指示，显示程序正常运行
		{
			LED1_FLIP;
			Timer2_Counter2=0;
		}

		if(CAN1_Flag)  //CAN总线中断
		{
			CAN1_Flag=0x00;  
		}
		
		if(UART1_DMA_Flag) //遥控器介入控制命令逻辑  3*7ms 发送一次
		{	
			UART1_DMA_Flag2++;
			if(UART1_DMA_Flag2>5)
			{
				LED2_FLIP;
				//RC_Debug_Message();
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
		if(IMU_Init_Flag && Timer2_Counter4 > 10) //1ms*10  100HZ 打印频率
		{			 
			IMU_Routing();
			IMU_Upload_Message();
			//IMU_Report_AHRSdata();
			Timer2_Counter4=0;
		}
		if(Timer2_Counter5 > 100) //1ms*100  10HZ 打印频率
		{			 
		  //Isolated_IO_Upload_Message(); 
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
void Test_MPU6050(void)
{
	float pitch,roll,yaw; 		//欧拉角
	while (1)
	{
		mpu_dmp_get_data(&pitch,&roll,&yaw);
		UART_send_string(USART2,"\npitch:");
		UART_send_floatdat(USART2,pitch);
	 
		
		UART_send_string(USART2,"\t roll:");
		UART_send_floatdat(USART2,roll);
	 		
		UART_send_string(USART2,"\t yaw:");
		UART_send_floatdat(USART2,yaw);
	 
		LED1_FLIP;
		Delay_10us(20000);
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
