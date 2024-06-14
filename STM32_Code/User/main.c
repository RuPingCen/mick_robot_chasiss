/*
* 串口1 与上位机通讯
* 串口2 DBUS DMA接收遥控器数据
* 串口3 485总线
* 串口4 串口5 备用
* 串口6 用于打印调试信息 printf() 函数
*/
#include "stm32f4xx.h"
#include "bsp_systick.h" 
#include "bsp_gpio.h"
#include "bsp_uart.h" 
#include "bsp_timer.h"
#include "bsp_usart_dma.h" 
#include "bsp_can.h" 
#include "bsp_adc.h" 
#include "key.h" 
#include "led.h" 

#include "DBUS/DBUS.h" 
#include "MOTOR_APSL2DB/MOTOR_APSL2DB.h" 
#include "MOTOR_RMD/MOTOR_RMD.h" 
#include "Battery/Battery.h"
#include "Mick_IO/Mick_IO.h"
#include "MOTOR_Control/MOTOR_Control.h"  
#include "MOTOR_EULER.h" 

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "IO_IIC.h"
#include "IMU.h"
#include "MickRobot_V3.h"

#include "w25q128.h"

#include "lwip/tcp.h"
#include "netconf.h"
#include "tcp_echoclient.h"
#include "stm32f4x7_phy.h"

extern  __IO uint16_t ADC_ConvertedValue[1]; // AD转换DMA数据存储  用于测量输入电压 若低于24V则报警

extern volatile Battery battery; //电池结构体数据


volatile uint32_t Timer2_Counter1=0;  //检测遥控器 通讯是否超时
volatile uint32_t Timer2_Counter2=0; //检测上位机 通讯是否超时
volatile uint32_t Timer2_Counter3=0; //电池读取
volatile uint32_t Timer2_Counter4=0; //IO上传任务
volatile uint32_t Timer2_Counter5=0; // 
volatile uint32_t Timer6_Counter2=0;  // IMU上传
  

volatile uint8_t UART1_Flag;
volatile uint8_t UART2_DMA_Flag=0x00,UART2_DMA_Flag2=0; //串口2中断标志位

volatile uint8_t UART3_Flag; //串口6中断标志位  接收电池
volatile uint8_t USART3_RX_BUF[RS485_RX_Len];  // 串口6 接收缓存
extern volatile uint8_t RS485_Recv_Data[RS485_RX_Len];//电池接收数据
 
volatile uint8_t flag_can1 = 0 ,flag_can1_2=0;		 //用于标志是否接收到数据，在中断函数中赋值
volatile uint8_t flag_can2 = 0;		  
//volatile CanTxMsg CAN2_TxMessage;		 //发送缓冲区
volatile CanRxMsg CAN1_RxMessage;		  //接收缓冲区
volatile CanRxMsg CAN2_RxMessage;		  //接收缓冲区
//extern uint8_t data_can[8];

extern volatile moto_measure_t moto_chassis[4];

volatile uint8_t motor_errors_cnt=0; //电机报错次数
volatile uint8_t motor_detect_cnt=0; //电机状态监测次数

// 以太网测试变量
extern __IO uint8_t EthLinkStatus;
__IO uint32_t LocalTime = 0;


static void TIM3_Config(uint16_t period,uint16_t prescaler);
void Main_Delay(unsigned int delayvalue);
void Test_Mick_IO(void);
int main(void)
{
	uint8_t motor_i=0;
	uint8_t main_counter = 0,IMU_Init_Flag=0;
	uint8_t W25QXX_ID;
	uint8_t W25QXX_SendData[] = "W25Q128读写测试";
	uint8_t W25QXX_Recv[256];
	
	
	SysTick_Init(); //滴答定时器初始化
	Initial_micros(); //TIM2 TIM3级联
	
	Init_Mick_GPIO();// 初始化LED 隔离型输出端口

 
	//上位机通讯 串口1
	My_Config_USART_Init(USART1,115200,1);
	UART_send_string(USART1,"USART1 Chassiss for MickRobot-V3 .....\n");

	//printf 占用串口6
	My_Config_USART_Init(USART6,256000,1);
	UART_send_string(USART6,"USART6 Chassiss for MickRobot-V3 .....\n");
	
	// FLASH芯片初始化
	W25QXX_Init();
	W25QXX_ID = W25QXX_ReadID();
	printf("ID:%X\r\n",W25QXX_ID);
	
	// MPU 6050初始化
	printf("Start Init MPU6050 ... \n");
    LED1_FLIP;
	while (mpu_dmp_init() && main_counter<10)
	{
		LED1_FLIP;
		printf("MPU6050 ReInit... \r\n");
		Delay_10us(9000);
		main_counter++;
	}
	if(main_counter<10)
	{
		IMU_Init_Flag = 1;
		printf("MPU6050 Start Success   \r\n");
	}
	else
	{
		printf("MPU6050 Init Failed   \r\n");
	}
	
	// 是否进入测试模式
	Test_Mick_IO();
	
    // DBUS 遥控器   占用串口2
	printf("RC_Remote_Init ...\n");	 
	RC_Remote_Init(); 
	printf("RC_Remote_Init Successful !\n");
	
	// CAN初始化
	printf("CAN1 Init ...\n");	
	CAN_Config(CAN1); //行进电机
	printf("CAN1 Init Successful !\n");	
	
	printf("MOTOR_APSL2DB Init ... \n");
	MOTOR_APSL2DB_Init();
	printf("MOTOR_APSL2DB Init Successful !\n");	
 
    Timer_2to7_Init(TIM6,10*1000);// 10ms
	Timer_start(TIM6);
	
    Timer_2to7_Init(TIM7,50*1000);// 50 ms
	Timer_start(TIM7);
 
	printf("mickrobot-V3 Init seccessful...\n");	
	while(1)
	{
		if(UART2_DMA_Flag) //遥控器介入控制命令逻辑  DBUS 7ms 发送一次数据
		{	
			UART2_DMA_Flag2++;
			if(UART2_DMA_Flag2>10)
			{
				LED2_FLIP;
				Flip_Isolated_Output(3);//绿色
				
				if(Send_RC_Message)
				{
					RC_Upload_Message();//上传遥控器状态
				}
				if(Debug_RC_Message)
				{
					RC_Debug_Message();
				}
				UART2_DMA_Flag2=0;
			}			
			UART2_DMA_Flag=0x00;	
		}
		
		if(flag_can1)
		{
			flag_can1_2++;
			if(flag_can1_2>3)
			{
				flag_can1_2 = 0;
				LED3_FLIP;
			}
				
			//printf("\nStdId: 0x%x   FMI: 0x%x \n",RxMessage.StdId,RxMessage.FMI); 
			//MOTOR_APSL2DB_PDO_Debug();
			
			if((moto_chassis[0].driver_status != 0x00) //表示有报警
				|| (moto_chassis[1].driver_status != 0x00) 
				|| (moto_chassis[2].driver_status != 0x00) 
				|| (moto_chassis[3].driver_status != 0x00))
			{
				motor_errors_cnt++; //每隔1秒清除一次计数	
				motor_i=0;
				for(motor_i=0;motor_i<4;motor_i++)
				{
					if((moto_chassis[0].driver_status != 0x00)) //表示有报警
					{
						 printf("Motor ID: %d  with warning !!!\n",motor_i);	
						if(((moto_chassis[motor_i].error_code_LSB & 0x80) || (moto_chassis[motor_i].error_code_HSB & 0x08)) 
							&& motor_errors_cnt<10) //驱动器输出短路 
						{
							    printf("Motor ID: %d  is overhead, now clear the error flag with reset APSL2DB\n",motor_i);	
								MOTOR_APSL2DB_Init();
						}
						else
						{
							Set_Isolated_Output(4,1);//红
						}
					}
				}				
			}
			else
			{
				Set_Isolated_Output(4,0);//红
			}
			if(Send_Motor_Message)
				Chassis_Motor_Upload_Message();
			if(Send_Odom_Message)
				Chassis_Odom_Upload_Message();
			flag_can1=0;
		}
		// IMU 数据读取 上传
		if(IMU_Init_Flag && Timer6_Counter2 > 1) //10ms  100HZ 读取DMP
		{
			IMU_Routing();	
			if(Send_IMU_Message)
			{
				IMU_Upload_Message();	
			}
			Timer6_Counter2=0;
		}
		
		// -----IO 状态上传
		if(Timer2_Counter4>= 20*1) //1秒读取一次 IO状态
		{
			Isolated_IO_Upload_Message();
			Timer2_Counter4=0;
		}
		
		//------电机状态监测
		if(Timer2_Counter5>= 20*1) //1秒读取一次电机状态
		{
			Timer2_Counter5=0;
			
			motor_errors_cnt=0;  //电机报警次数清零
			
			motor_detect_cnt++;
			if(motor_detect_cnt<5)
				APS_L2DB_Read_Temp(motor_detect_cnt);
			else
			{
				motor_detect_cnt =0;
				if((moto_chassis[0].Temp >= 60) //表示有报警
					|| (moto_chassis[1].Temp >= 60) 
					|| (moto_chassis[2].Temp >= 60) 
					|| (moto_chassis[3].Temp >= 60))
				{
					printf("Motor ID 1-4 tempture: %d, %d, %d, %d \n",moto_chassis[0].Temp,moto_chassis[1].Temp,moto_chassis[2].Temp,moto_chassis[3].Temp);	
					Set_Isolated_Output(4,0);//红
				}
			}
		}
	}
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

void Test_Mick_IO(void)
{
	uint8_t code =0x00;
	uint8_t cnt =0;
//	while(1)
//	{
//		// ch4 红 靠近VCC这一侧
//		// ch3 绿色  
//		// ch2 黄 
//		 Set_Isolated_Output(2,1);
//		Main_Delay(500); Main_Delay(500);  
//		Set_Isolated_Output(2,0);
//		Main_Delay(500); Main_Delay(500);  
//	}
	
	//  蜂鸣器是 PA0
		
	//	ADC 是 PA1
	MY_ADC_DMA_Init(ADC1,ADC_Channel_1);
	float tem =0;
 
	while(1)
	{
		  
		cnt++;
		{
			tem = ADC_ConvertedValue[0]*3.3/4096.0*21.0;
			UART_send_string(USART1,"\n ADC: ");
			UART_send_floatdat(USART1,tem);
			if(tem < 23)
				BUZZER_ON;
			else
				BUZZER_OFF;
		}
		// 测试IO 输出
		{
			 if(cnt>=0 &  cnt <30)
			 {
				 Set_Isolated_Output(1,0);
				 Set_Isolated_Output(2,0);
				 Set_Isolated_Output(3,0);
				 Set_Isolated_Output(4,1);
			 }
			 else if(cnt>=30 &  cnt <60)
			 {
				 Set_Isolated_Output(1,0);
				 Set_Isolated_Output(2,0);
				 Set_Isolated_Output(3,1);
				 Set_Isolated_Output(4,0);
			 }
			 else if(cnt>=60 &  cnt <90)
			 {
				 Set_Isolated_Output(1,0);
				 Set_Isolated_Output(2,1);
				 Set_Isolated_Output(3,0);
				 Set_Isolated_Output(4,0);
			 }
			 else if(cnt>=90 &  cnt <120)
			 {
				 Set_Isolated_Output(1,0);
				 Set_Isolated_Output(2,0);
				 Set_Isolated_Output(3,0);
				 Set_Isolated_Output(4,0);
			 }
			else
				cnt =0;
 
		
			Main_Delay(500); Main_Delay(500);  
		}
		{
				LED1_FLIP;
				LED2_FLIP;
				LED3_FLIP;
		}
		{
			if(Read_Key(1) == 1)
			{
				 printf("key1 pressed \n");
				 UART_send_string(USART1,"key1 pressed \n");
				 Set_Isolated_Output(1,1);   
				 BUZZER_ON;
				 //Set_Isolated_Output(2,1);   
			}
			else
			{
				 Set_Isolated_Output(1,0);   
				 BUZZER_OFF;
				 //Set_Isolated_Output(2,0);   
			}

			if(Read_Key(2) == 1)
			{
				printf("key2 pressed \n");
				UART_send_string(USART1,"key2 pressed \n");
				 Set_Isolated_Output(2,1);   
				// Set_Isolated_Output(4,1); 
			}
			else
			{
				 Set_Isolated_Output(2,0);   
				// Set_Isolated_Output(4,0);   
			}

		}
		{
			code = Read_Code_Switch();
			printf("Read_Code_Switch   0x%x\n",code);
			
		}
		// 测试IO 输入
		{
			if( Read_Isolated_Input(1) ==1)
			 {
				printf("Read_Isolated_Input(1)\n");	
				UART_send_string(USART1,"Read_Isolated_Input(1)\n");
			 }
				if( Read_Isolated_Input(2) ==1)
			 {
				printf("Read_Isolated_Input(2)\n");	
				UART_send_string(USART1,"Read_Isolated_Input(2)\n");
			 }
				if( Read_Isolated_Input(3) ==1)
			 {
				printf("Read_Isolated_Input(3)\n");	
				UART_send_string(USART1,"Read_Isolated_Input(3)\n");
			 }
				if( Read_Isolated_Input(4) ==1)
			 {
				printf("Read_Isolated_Input(4)\n");	
				UART_send_string(USART1,"Read_Isolated_Input(4)\n"); 
			 }
		}
	}
}