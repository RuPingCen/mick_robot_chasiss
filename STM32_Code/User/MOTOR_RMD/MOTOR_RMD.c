/*
* 利用上位机设置电机ID   在上位机-》电机升级界面下的位置多圈模式写入1
* 驱动器型号：MC-X-300-O
* CAN总线速率 1M
*
* 问题：
* 数据下发自动返回报文是否可以关闭、CAN 总线接线方式、编码器记忆零点问题、
*



*/
#include "stm32f4xx.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_can.h" 
#include "common.h" 
#include "key.h" 
#include "MOTOR_RMD/MOTOR_RMD.h" 
#include "MOTOR_APSL2DB/MOTOR_APSL2DB.h" 

#define MOTOR_RMD_CAN CAN2

union INT16 rmd_motor_tem_int16;

extern CanRxMsg CAN2_RxMessage;				 //接收缓冲区
extern volatile uint8_t flag_can2;	
volatile moto_measure_t moto_rmd_chassis[4] = {0};//4 chassis moto
/**
* @funtion 安普斯 L2DB 驱动器   CAN open数据协议
* @Brief        
* @Param		
* @Retval	None
* @Date     2023/4/28
* @maker    crp
*/

void MOTOR_RMD_Init(void)
{
	uint8_t index_i=0;
	uint16_t delay_MAX = 50; //延时时间
	uint16_t init_times = 0; //初始化次数
	uint16_t init_times_MAX = 5;
	
	printf("Reset motor RMD...\n");
 
	MOTOR_RMD_Read_Cmd_ALLMotor(0x76);// 电机复位
	MOTOR_RMD_Delay(1200);
	 
	printf("Read motor information...\n");
	for(index_i=1;index_i<5;index_i++)
	{
		flag_can2=0;
		memset(&CAN2_RxMessage,0,sizeof(CAN2_RxMessage));
		while(flag_can2==0)
		{
			MOTOR_RMD_Read_Cmd(index_i,0xB2);  //读取系统版本
			MOTOR_RMD_Delay(400);
		};
		if(MOTOR_RMD_State_Analysis()!=0x01 || CAN2_RxMessage.Data[0] != 0xB2)
		{
			printf("step1 ERROR: failed to read RMD motor version!\n");
		}
		//MOTOR_RMD_Delay(50);
	}
	
	for(index_i=1;index_i<5;index_i++)
	{
		flag_can2=0;
		memset(&CAN2_RxMessage,0,sizeof(CAN2_RxMessage));
		MOTOR_RMD_Read_Cmd(index_i,0xB5);  //读取电机型号
		while(flag_can2==0);
		if(MOTOR_RMD_State_Analysis()!=0x01 || CAN2_RxMessage.Data[0] != 0xB5)
		{
			printf("step2 ERROR: failed to read RMD type!\n");
		}
	}
	for(index_i=1;index_i<5;index_i++)
	{ 
		MOTOR_RMD_Read_Cmd(index_i,0x77);        //系统抱闸释放指令
		MOTOR_RMD_Delay(40);
	}
	 
	printf("Set motor Angle to Zero...\n");
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_RMD_Set_Angle(index_i,100,0);   //设置电机角度为0    绝对位置模式
		MOTOR_RMD_Delay(4);
	}
	MOTOR_RMD_Delay(3000);
 
	
	for(index_i=1;index_i<5;index_i++)
	{ 
		flag_can2=0;
	    memset(&CAN2_RxMessage,0,sizeof(CAN2_RxMessage));
		MOTOR_RMD_Read_Cmd(index_i,0x70);  //读取电机模式  确保处于位置模式
		while(flag_can2==0);
		if(MOTOR_RMD_State_Analysis()!=0x01 || CAN2_RxMessage.Data[7] != 0x03)
		{
			printf("step3 ERROR: failed to set position mode for motor id %d! \n",index_i);
		}
	}
	
	//读取电机状态  （电机温度 抱闸控制指令： 电压 voltage  错误标志 errorState）  检测当前电机是否有错误状态
	for(index_i=1;index_i<5;index_i++)
	{ 
		flag_can2=0;  
	    memset(&CAN2_RxMessage,0,sizeof(CAN2_RxMessage));
		MOTOR_RMD_Read_Cmd(index_i,0x9A);  //读取电机状态1  
		while(flag_can2==0)
		if(MOTOR_RMD_State_Analysis()!=0x01 || CAN2_RxMessage.Data[0] != 0x9A)
		{
			printf("step4 ERROR: motor id is unnormal ...%d! \n",index_i);
		}
	}
	
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_RMD_Set_Angle(index_i,100,0);  
		MOTOR_RMD_Delay(4);
	}
	
	
//	MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//	MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//	MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//    while(1) 
//	{		
//		if(	Key_Scan(KEY1_GPIO_PORT,KEY1_PIN) == KEY_ON)
//		{
//			MOTOR_RMD_Set_Angle_ALLMotor(0);		
//			MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//			MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);			
//			//CAN_DEBUG_ARRAY(TxMessage.Data,8); 
//		}
//		else
//		{
//			//		for(index_i=1;index_i<5;index_i++)
//			//		{
//			//			MOTOR_RMD_Set_Angle(index_i,6000);  MOTOR_RMD_Delay(3);
//			//		}
//					printf("MOTOR_RMD_Set_Angle 60° \n");
//					MOTOR_RMD_Set_Angle_ALLMotor(-6000);
//					MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//					MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//					
//			//		for(index_i=1;index_i<5;index_i++)
//			//		{
//			//			MOTOR_RMD_Set_Angle(index_i,6000);  MOTOR_RMD_Delay(3);
//			//		}
//					MOTOR_RMD_Set_Angle_ALLMotor(6000);
//					MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//					MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);MOTOR_RMD_Delay(3000);
//					printf("MOTOR_RMD_Set_Angle -60° \n");
//		}
//	}

}
uint8_t MOTOR_RMD_Calibrate_Offset(void)
{
	uint8_t flag = 0x00;
	int angle_left=0,angle_right=0;
	uint8_t index_i=0;
	
	uint8_t try_times =0;
	
	printf("MOTOR_RMD_Set_Speed  \n");
	try_times =0;
	memset(&CAN2_RxMessage,0,sizeof(CAN2_RxMessage));
	while(1)
	{
		MOTOR_RMD_Set_Speed(2,5000); // 80度/s
		MOTOR_RMD_Delay(9);
		if(CAN2_RxMessage.StdId  == 0x242 && CAN2_RxMessage.Data[0] == 0xA2)
		{
			try_times++;
			if(try_times > 2)
			{
				if((CAN2_RxMessage.Data[3]<<8|CAN2_RxMessage.Data[2]) >180) // 1.8A
				{
					MOTOR_RMD_Set_Speed(2,0);
					angle_left = (int16_t)(CAN2_RxMessage.Data[7]<<8|CAN2_RxMessage.Data[6]);
					//MOTOR_RMD_Read_Cmd(2,0x76);// 电机复位
					//MOTOR_RMD_Delay(2);
					printf("left limit position! %d\n",angle_left);
					 flag = 0x0f;
					break;
					
				}
			}	
		}
	}
	MOTOR_RMD_Delay(30000);
	printf("MOTOR_RMD_Set_Speed right \n");
	try_times =0;
	memset(&CAN2_RxMessage,0,sizeof(CAN2_RxMessage));
	while(1)
	{
		MOTOR_RMD_Set_Speed(2,-3000);
		MOTOR_RMD_Delay(2);
		if(CAN2_RxMessage.StdId  == 0x242 && CAN2_RxMessage.Data[0] == 0xA2)
		{
			if((CAN2_RxMessage.Data[3]<<8|CAN2_RxMessage.Data[2]) >180) // 1.8A
			{
				try_times++;
				if(try_times > 2)
				{
					MOTOR_RMD_Set_Speed(2,0);
					angle_right = (int16_t)(CAN2_RxMessage.Data[7]<<8|CAN2_RxMessage.Data[6]);
					printf("right limit position! %d\n",angle_right);
					MOTOR_RMD_Read_Cmd(2,0x76);// 电机复位
					flag = 0x0f;
					break;
				}
			}
		}
	}
		
		 
 
	return 1;
	
}

// 调用急停以后需要，使用急停解除函数，电机才能控制
void MOTOR_RMD_EmStop(uint16_t ID)
{
	MOTOR_RMD_Read_Cmd_ALLMotor(0x81);
}
// 输入单位 0.01度/LSB
void MOTOR_RMD_Set_Angle_ALLMotor(int32_t Angle)
{
	uint16_t max_speed = 180;
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x280;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0xA4;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = max_speed;
	TxMessage.Data[3] = max_speed>>8;
	TxMessage.Data[4] = Angle;
	TxMessage.Data[5] = Angle>>8;
	TxMessage.Data[6] = Angle>>16;
	TxMessage.Data[7] = Angle>>24;
	CAN_Transmit(MOTOR_RMD_CAN, &TxMessage); // 发送消息  
}
// 输入单位 0.01度/LSB
void MOTOR_RMD_Set_Angle(uint16_t ID,uint16_t max_speed,int32_t Angle)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x140+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0xA4;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = max_speed;
	TxMessage.Data[3] = max_speed>>8;
	TxMessage.Data[4] = Angle;
	TxMessage.Data[5] = Angle>>8;
	TxMessage.Data[6] = Angle>>16;
	TxMessage.Data[7] = Angle>>24;
	CAN_Transmit(MOTOR_RMD_CAN, &TxMessage); // 发送消息  
}
void MOTOR_RMD_Set_Speed_ALLMotor(int32_t rpm)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x280;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0xA2;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = 0x00;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = rpm;
	TxMessage.Data[5] = rpm>>8;
	TxMessage.Data[6] = rpm>>16;
	TxMessage.Data[7] = rpm>>24;
	CAN_Transmit(MOTOR_RMD_CAN, &TxMessage); // 发送消息  
}
void MOTOR_RMD_Set_Speed(uint16_t ID,int32_t rpm)
{
	uint16_t max_speed = 100;
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x140+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0xA2;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = 0x00;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = rpm;
	TxMessage.Data[5] = rpm>>8;
	TxMessage.Data[6] = rpm>>16;
	TxMessage.Data[7] = rpm>>24;
	CAN_Transmit(MOTOR_RMD_CAN, &TxMessage); // 发送消息  
}
void MOTOR_RMD_Read_Cmd_ALLMotor(uint8_t cmd_id)
{
	uint16_t max_speed = 100;
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x280;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = cmd_id;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = 0x00;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_RMD_CAN, &TxMessage); // 发送消息  
}
// 转向电机状态读取命令  麦塔科技电机
void MOTOR_RMD_Read_Cmd(uint16_t ID,uint8_t cmd_id)
{
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x140+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = cmd_id;
	TxMessage.Data[1] = 0x00;
	TxMessage.Data[2] = 0x00;
	TxMessage.Data[3] = 0x00;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_RMD_CAN, &TxMessage); // 发送消息  
}
void MOTOR_RMD_Delay(uint32_t delayvalue)
{
	unsigned int i;
	while(delayvalue-->0)
	{	
		i=5000;
		while(i-->0);
	}
}
// CAN2中断中调用
uint8_t MOTOR_RMD_State_Analysis(void)
{
	if((CAN2_RxMessage.StdId &0xffff )>>4 == 0x24)
	{
		if(CAN2_RxMessage.Data[0] == 0xB2)
		{
//			printf("RMD MOTOR id %d version: %d\n",
//				CAN2_RxMessage.StdId-0x240,(CAN2_RxMessage.Data[7]<<24 | CAN2_RxMessage.Data[6]<<16 |CAN2_RxMessage.Data[5]<<8 |CAN2_RxMessage.Data[4]));
			return 0x01;
		}
		else if(CAN2_RxMessage.Data[0] == 0xB5)
		{
//			printf("RMD MOTOR id %d type: %c%c%c%c%c%c%c\n",
//				CAN2_RxMessage.StdId-0x240,CAN2_RxMessage.Data[1],CAN2_RxMessage.Data[2],CAN2_RxMessage.Data[3],
//				CAN2_RxMessage.Data[4],CAN2_RxMessage.Data[5],CAN2_RxMessage.Data[6],CAN2_RxMessage.Data[7]);
			return 0x01;
		}
		else if(CAN2_RxMessage.Data[0] == 0x70) //电机模式 Byte[7]: 01 电流， 02 速度模式，03位置模式
		{
			//printf("RMD MOTOR id %d mode: %d\n",CAN2_RxMessage.StdId-0x240,CAN2_RxMessage.Data[7]);
			return 0x01;
		}
		else if(CAN2_RxMessage.Data[0] == 0x9A) //电机温度  抱闸控制指令： 电压 voltage  错误标志 errorState
		{
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].Temp = CAN2_RxMessage.Data[1];
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].driver_status = CAN2_RxMessage.Data[3];			
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].Voltage = (CAN2_RxMessage.Data[5]<<8|CAN2_RxMessage.Data[4]);
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].error_code_LSB = CAN2_RxMessage.Data[6];
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].error_code_HSB = CAN2_RxMessage.Data[7];
			//printf("RMD MOTOR id %d tempture: %d ",CAN2_RxMessage.StdId-0x240,CAN2_RxMessage.Data[1]);
			//printf("brake state: %d, voltage: %1fV, error code: 0x%4x\n",moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].driver_status,
			//																moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].Voltage/10.0,
			//							(moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].error_code_HSB<<8|moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].error_code_LSB));
			return 0x01;
		}
		else if(CAN2_RxMessage.Data[0] == 0xA4) //设置速度
		{

			rmd_motor_tem_int16.int16_value =0; //度
			rmd_motor_tem_int16.uint8_value[0] = CAN2_RxMessage.Data[6];
			rmd_motor_tem_int16.uint8_value[1] = CAN2_RxMessage.Data[7];
		  
			
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].Temp = CAN2_RxMessage.Data[1];
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].real_current = (CAN2_RxMessage.Data[3]<<8|CAN2_RxMessage.Data[2]);
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].speed_rpm = (CAN2_RxMessage.Data[5]<<8|CAN2_RxMessage.Data[4]); // dps单位是度每秒吗？度每秒/360*60 = 度每秒*6
			moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].angle = rmd_motor_tem_int16.int16_value;
			//printf("set speed .");
			//// 速度是否有减速度比存在  角度的分辨率是多少   手册上标注的 angle范围是±32767
			//printf("RMD MOTOR id %d tempture(°C): %d   current(0.01A): %d",CAN2_RxMessage.StdId-0x240,CAN2_RxMessage.Data[1],(CAN2_RxMessage.Data[3]<<8|CAN2_RxMessage.Data[2]));
			//printf("speed(dps): %d, angle(degree): %d\n",(CAN2_RxMessage.Data[5]<<8|CAN2_RxMessage.Data[4]),moto_rmd_chassis[CAN2_RxMessage.StdId-0x240].angle);

			return 0x01;
		}
		else
			return CAN2_RxMessage.Data[0];
	}	
	return 0x00;
}
 
void MOTOR_RMD_Debug(void)
{
	printf("EMD ID 1-4 speed_rpm:%d \t %d \t%d \t%d \t\n",moto_rmd_chassis[0].speed_rpm
			,moto_rmd_chassis[1].speed_rpm,moto_rmd_chassis[2].speed_rpm,moto_rmd_chassis[3].speed_rpm);
	printf("EMD ID 1-4 angle:%d \t %d \t%d \t%d \t\n",moto_rmd_chassis[0].angle
			,moto_rmd_chassis[1].angle,moto_rmd_chassis[2].angle,moto_rmd_chassis[3].angle);
}		