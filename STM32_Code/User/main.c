/******************************************************************************************************
#2023-11-04
	1������mpu6050 DMP���㣬��whileѭ���ж�ȡIMU����
		// ��mpu6050.h�����Ӻ���
			extern uint8_t MPU6050_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);//д���ֽ�
			extern uint8_t MPU6050_IIC_Wait_Ack(void);//�ȴ�ACK�ź�
			extern uint8_t MPU6050_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf);//�����ֽ�
		// IO_IIC.h�����Ӻ���
			unsigned char IO_IIC_read_byte(unsigned char ack);//read a byte//��I2C���߽���һ���ֽ�����
			void SDA_DIR_IN(void);//����SDA����Ϊ����
			void SDA_DIR_OUT(void);//����SDA����Ϊ���
#2022-4-22
	1������ SBUS ����ң�������빦�� 
	2������ SBUS ���ִ���ע��

#2021-4-18
	1����DBUS����λ�����͵������Ƶ��жϺ����н��д���
	2����PID��ؼ��㺯���Ƶ�PID.c��
	3�����MPU9250 ��ȡ����̬���㺯�� (�����ж���Դ���ܳ�ʱ�䱻ռ�ã������ʱ������while��)
	4����mick robot controller V1.0.0�����ϲ���ͨ��
	5��LED1 ָʾ�����Ƿ��������У�20HZ��˸�� 
		 LED2ָʾң�����Ƿ�������
		 LED3ָʾCAN�����Ƿ�������

#2020-9-21 
	1�������MPU9250�����ݶ�ȡ ��̬���㺯��
	2��ĿǰMPU9250ʹ�õĶ˿�ΪPB10  PB11
	3����С��1.0�İ����ϲ���ͨ���ˣ�����ȷ��ȡ���ݡ�2.0��ͨѶ�崮��������
	
#2020-9-8
	1��������DBUS�еĺ�������
	2������ң�����źŶ�ʧ��ɵ������������𡰷�ת��������
	3��ͳһ4�ֺ�2�ֲ���С��ģ�͵�����ƺ����ĵ�λΪ m/s  �� rad/s   
	
#2019-10-07
	1�����������ķ��PID���ƺ���
	2��������ROS�ڵ�ͨѶ�ӿ�
	3������DMA��ʽ����DBUSң����
	
* update 2020-9-21
* maker: crp
******************************************************************************************************/
#include "stm32f10x.h"
#include "stdio.h"//�������ڷ��͵�FILE����
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
/**********************�궨��****************************/
#define EnableInterrupt 	__set_PRIMASK(0) //���ж� д��0�� �����ж�   ��1�������ж�
#define DisableInterrupt 	__set_PRIMASK(1) //���ж� д��0�� �����ж�   ��1�������ж�
uint32_t TimingDelay; //����delay���Ƶľ�ȷ��ʱ
uint16_t ADC_ConvertedValue; 
 
#define DEBUUG 1
#define DEBUUG_MATRIX_KEYSACN 0


 
/*********************************************************/
/******************ȫ�ֱ�������***************************/

volatile uint8_t UART1_DMA_Flag=0x00;
volatile uint8_t UART1_DMA_Flag2=0x00;
volatile uint8_t UART2_Flag=0x00;
volatile uint8_t CAN1_Flag=0x00;
volatile uint8_t TIM3_Flag=0x00;
volatile uint8_t IMU_Init_Flag=0x00;

volatile uint32_t Timer2_Counter1=0; //�ֱ�������ǽ��������Ƿ񳬹���ʱ�����Ʒ�Χ
volatile uint32_t Timer2_Counter2=0;
volatile uint32_t Timer2_Counter3=0; 
volatile uint32_t Timer2_Counter4=0;
volatile uint32_t Timer2_Counter5=0; 

 

extern command_t recived_cmd; //���̽�����λ������ṹ��
extern uint8_t Code_Switch_Value;
extern rc_info_t rc;  // remote command
/*********************************************************/
/**********************��������***************************/
void Main_Delay(unsigned int delayvalue);
void Main_Delay_us(unsigned int delayvalue);
void Test_Mick_GPIO(void);
void Test_MPU6050(void);

/*********************************************************/
/*****************������**********************************/
int main(void)
{	

	uint8_t main_counter = 0;
	unsigned char dma_lendat = 18;
	SysTick_Init();//��ʼ���δ�ʱ��
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable,ENABLE);
	
	//step1 ��ʼ��IO ---------------------------------------------
	Init_Mick_GPIO();
	Set_Isolated_Output(1,0);// �����������Ϊ�������
	Set_Isolated_Output(2,0);			
	Set_Isolated_Output(3,0);
	Set_Isolated_Output(4,0);	
	
	//step2 ��ʼ������2/3 -----------------------------------------
	My_Config_USART_Init(USART2,115200,1);// ���ڳ�ʼ������ ������115200	 �빤�ػ�ͨѶ
 	My_Config_USART_Init(USART3,115200,1);	
	Delay_10us(200);
	UART_send_string(USART2,"\n\n MickX4 ROS Car \n");//�ַ�������
	UART_send_string(USART3,"\n\n MickX4 ROS Car \n");//�ַ�������	

	//step3 ��ʼ��IMU ---------------------------------------------
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

	 
	//step4 ��ʼ��CAN ---------------------------------------------
	CAN_Config();//��ʼ��canģ��
	DJI_Motor_Init();//��ʼ��3508ģ��
	Mecanum_Wheel_Speed_Model(0,0,0); //�����Ժ������õ��Ŀ��ֵΪ0
	
	//step5 ����С���˶�ѧģ�� -------------------------------------
	Code_Switch_Value = Read_Code_Switch(); //���뿪�ص���λ���õ��̵��˶�ѧģ��
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
	
	//step6 ����ң�������� -------------------------------------
	Code_Switch_Value = 0x00;
	Code_Switch_Value = Read_Code_Switch(); 
	if((Code_Switch_Value & 0x04) == 0x00) // ��ȡ��3λ����״̬
	{
		UART_send_string(USART2,"SBUS (for T8FB)model is used. \n"); 
		rc.type =2; // SBUSģ��     �ֵ�Э��
		dma_lendat = 25;
	}
	else
	{
		UART_send_string(USART2,"DBUS (for DJI DT7) model is used. \n"); 
		rc.type =1; // Ĭ��ΪDBUSģ��  ��Э��
		dma_lendat = 18;
	}
	USART_DMA_Rec_Config(USART1,100000,dma_lendat); //��������1 DMA���շ�ʽ  ���ڽ���ң�����ź�
	
	//step7 ��ʼ����ʱ�� -------------------------------------	
	Timer_2to7_counter_Generalfuncation(TIM2,1000);//1ms
	Timer_2to7_Generalfuncation_start(TIM2);
	
	//Timer_2to7_counter_Generalfuncation(TIM3,5000);//5ms
	//Timer_2to7_Generalfuncation_start(TIM3);
	UART_send_string(USART2,"Init Successful !!! \n\n");//�ַ�������
		   
	while(1)
	{
		if(Timer2_Counter2 > 100) //״ָ̬ʾ����ʾ������������
		{
			LED1_FLIP;
			Timer2_Counter2=0;
		}

		if(CAN1_Flag)  //CAN�����ж�
		{
			CAN1_Flag=0x00;  
		}
		
		if(UART1_DMA_Flag) //ң����������������߼�  3*7ms ����һ��
		{	
			UART1_DMA_Flag2++;
			if(UART1_DMA_Flag2>5)
			{
				LED2_FLIP;
				//RC_Debug_Message();
				RC_Upload_Message();//�ϴ�ң����״̬
				UART1_DMA_Flag2=0;
			}			
			UART1_DMA_Flag=0x00;	
		}
		
		if(UART2_Flag && recived_cmd.flag) //��λ��ָ���·��ӿ�
		{
			//------------------------- ��ʵʱ�߳� --------------------//
			
			//---------------------------------------------------------//
			recived_cmd.flag =0;//ʹ��һ���Ժ���������
			UART2_Flag=0x00;
		}
	
		if(Timer2_Counter3 > 8) //1ms*8  125 HZ �ϴ��������
		{
			Timer2_Counter3=0;
			DJI_Motor_Upload_Message();
			//DJI_Motor_Show_Message();
		}
		if(IMU_Init_Flag && Timer2_Counter4 > 10) //1ms*10  100HZ ��ӡƵ��
		{			 
			IMU_Routing();
			IMU_Upload_Message();
			//IMU_Report_AHRSdata();
			Timer2_Counter4=0;
		}
		if(Timer2_Counter5 > 100) //1ms*100  10HZ ��ӡƵ��
		{			 
		  //Isolated_IO_Upload_Message(); 
			Timer2_Counter5=0;
		}
		
	}
 // exit from main() function
}
 
 //��ʱ���� 6.3ms
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
	float pitch,roll,yaw; 		//ŷ����
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
