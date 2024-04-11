#include "stm32f4xx.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_can.h" 

#include "common.h" 
#include "MOTOR_APSL2DB/MOTOR_APSL2DB.h" 
 
 
#define MOTOR_CAN CAN1

volatile moto_measure_t moto_chassis[4] = {0};//4 chassis moto
union INT32 tem_int32;
union INT16 tem_int16;

extern CanRxMsg CAN1_RxMessage;				 //接收缓冲区
extern volatile uint8_t flag_can1;	
/**
* @funtion 安普斯 L2DB 驱动器   CAN open数据协议
* @Brief        
* @Param		
* @Retval	None
* @Date     2023/4/28
* @maker    crp
*/

void MOTOR_APSL2DB_Init(void)
{
	uint8_t index_i=0;
	uint16_t delay_MAX = 50; //延时时间
	uint16_t init_times = 0; //初始化次数
	uint16_t init_times_MAX = 5;
	
	// step0  0x86 清除驱动器报警
   for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Enable(index_i,0x06);MOTOR_Delay(delay_MAX);
	}
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Enable(index_i,0x0F);MOTOR_Delay(delay_MAX);
	}
	printf("MOTOR_APSL2DB_Infor_Get...\n");
	MOTOR_APSL2DB_Infor_Get();
	
	//printf("MOTOR_APSL2DB: step1 check can line is OK?\n");
	// step1  检测CAN总线是连接否正常
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_Model(index_i,3);// 设置1-4号电机为速度模式
		MOTOR_Delay(delay_MAX);
		init_times = 0;
		while(flag_can1==0 && (init_times++ < init_times_MAX))
		{
			flag_can1 = 0;
			if(CAN1_RxMessage.StdId == 0x580+index_i)
			{
				CAN1_RxMessage.StdId =0x00;
				;//清除接收缓存
			}
			else
			{
				printf("Motro APSL2DB: %d is offline, check CAN bus connection\n",index_i);
			}
		}
		if(init_times == init_times_MAX)
		{
			index_i--;
			printf("Motro APSL2DB: %d not responding, motor init failed\n",index_i);
		}
	}
	
	// step2  0x86 清除驱动器报警
    for(index_i=1;index_i<5;index_i++)
		MOTOR_APSL2DB_Enable(index_i,0x86);MOTOR_Delay(delay_MAX);
	
	 // step3  电机上电(使能)
    for(index_i=1;index_i<5;index_i++)
	{  
		MOTOR_APSL2DB_Enable(index_i,0x0F);MOTOR_Delay(delay_MAX);
		init_times = 0;
		while(flag_can1==0 && (init_times++ < init_times_MAX))
		{
			flag_can1 = 0;
			if((CAN1_RxMessage.StdId == 0x580+index_i) && (CAN1_RxMessage.Data[0] == 0x60))
			{
				break;//清除接收缓存
			}
		}
		if(init_times >= init_times_MAX)
		{
			printf("Motro APSL2DB: %d enable failed\n",index_i);
		}
	}
	printf("Motro APSL2DB: Set Motor IS 1-4 Target speed 0");
	// step4    设置电机目标速度为0 
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_RPM(index_i,0);MOTOR_Delay(delay_MAX);
	}
	
	// step5    设置电机限制电流 25A
	printf("Motro APSL2DB: Set Motor IS 1-4 Current Limt 25");
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_CurrentLimt(index_i,28);MOTOR_Delay(delay_MAX);
	}
	
	
	// step6  梯形加速、减速
	printf("Motro APSL2DB: Set Motor IS 1-4 Trapezoidal Acceleration and Deceleration ");
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_Trap_Accel(index_i,5);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);
		MOTOR_APSL2DB_Set_Trap_Decel(index_i,13);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);
	}
	 
 
	
	// step7    设置电机位置自动上报  PDO 
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Enable_PDO(index_i,1);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);
	}

	//APS_L2DB_Read_Position(1);  
}

// 读取电机固件信息 以保证电机是正确连接在CAN总线上
uint8_t MOTOR_APSL2DB_Infor_Get(void)
{
	uint8_t index_i=0,j=0;
	uint16_t delay_MAX = 50; //延时时间
	uint16_t init_times;
	uint16_t init_times_MAX = 50;
	//0x3A 本机型号-系列  默认:L2DB
	//0x30003B 32U,RO ASCII 本机型号-电压与电流等级
	//0x103C 0x30003C   
	//本机型号-反馈与总线类型  默认(485版本):CAFR   默认(CAN版本):CAFC
	//0x103D 0x30003D  本机型号-配方 
						//默认(7Nm电机):ASL3 = 0x41534C33
						//默认(10Nm电机):ASM4 = 0x41536D34
						//默认(15Nm电机):ASM5 = 0x41536D35
						//默认(30Nm电机):ASH8= 0x41534838
	//0x1007 0x2500F1  固件日期 //格式:20190114 代表2019年1月14日
	//0x1009 0x2500F3 32U,RO ASCII 硬件版本
	uint16_t cmd_tem1[] = {0x3000,0x3000,0x3000,0x3000,0x2500,0x2500};
	uint8_t  cmd_tem2[] = {0x3A,0x3B,0x3C,0x3D,0xF1,0xF3};
	for(index_i=1;index_i<5;index_i++)
	{
		for(j=0;j<6;j++)
		{
			MOTOR_APSL2DB_Read_Cmd(index_i,cmd_tem1[j],cmd_tem2[j]); 
			MOTOR_Delay(delay_MAX);
			init_times = 0;
			while(flag_can1==0 && (init_times++ < init_times_MAX))
			{
				;
			}
			if(init_times >= init_times_MAX)
				printf("Motro APSL2DB: %d read motor ID faild\n",index_i);
			else
				MOTOR_APSL2DB_Measurements_Debug();
		}
	}
	return 0x01;
}

void MOTOR_Delay(uint32_t delayvalue)
{
	unsigned int i;
	while(delayvalue-->0)
	{	
		i=5000;
		while(i-->0);
	}
}
 

// 设置电机工作模式   3 带加减速控制的速度模式 -3 立即速度模式 4 力矩模式 1 位置模式
void MOTOR_APSL2DB_Set_Model(uint16_t ID, int8_t param_value)
{
	uint16_t index = 0x6060;
	uint8_t subindex = 0x00;
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2F; //一个字节
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_value;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
// 设置电机工作模式   0x06 电机断电（松轴） 0x0F 电机上电(使能)  0x86 清除驱动器报警
void MOTOR_APSL2DB_Enable(uint16_t ID, int16_t param_value)
{
	uint16_t index = 0x6040;
	uint8_t subindex = 0x00;
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2B;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_value;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
// 调用急停以后需要，使用急停解除函数，电机才能控制
// 急停 1:急停生效 0:急停解除
void MOTOR_APSL2DB_EmStop(uint16_t ID, uint8_t param_value)
{
	uint16_t index = 0x605A;
	uint8_t subindex = 0x11;
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2F;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_value;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}

// 设定电机速度 单位RPM
void MOTOR_APSL2DB_Set_RPM(uint16_t ID, double speed_rpm)
{
	uint16_t index = 0x60FF;
	uint8_t subindex = 0x00;
	// 计算速度  ([rpm]*512*[4096])/1875  = rpm*1118.4810666
	int32_t v = (speed_rpm*1118.4810667)/1;
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x23;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = v;
	TxMessage.Data[5] = v>> 8;
	TxMessage.Data[6] = v>> 16;
	TxMessage.Data[7] = v>> 24;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}



// 通讯掉线停机延时 param_value=0 不使能  1 使能
void MOTOR_APSL2DB_Enable_Disconnect(uint16_t ID, uint8_t param_value)
{
	uint16_t index = 0x4100;
	uint8_t subindex = 0x10;
	
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2F;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_value;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
// 通讯掉线停机延时参数 单位ms 当总线断线超过设置时长,电机将松轴
void MOTOR_APSL2DB_Enable_Disconnect_Time( uint16_t ID,uint32_t param_T)
{
	uint16_t index = 0x4100;
	uint8_t subindex = 0x11;
	
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x23;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_T;
	TxMessage.Data[5] = param_T>> 8;
	TxMessage.Data[6] = param_T>> 16;
	TxMessage.Data[7] = param_T>> 24;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
// 自动上报实际位置和实际速度 param_value=0 不使能  1 使能
void MOTOR_APSL2DB_Enable_PDO(uint16_t ID, uint8_t param_value)
{
	uint16_t index = 0x4700;
	uint8_t subindex = 0x01;
	
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2F;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_value;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
// 设置PDO周期 单位ms  默认10ms
void MOTOR_APSL2DB_Set_PDO_Period( uint16_t ID,uint16_t param_T)
{
	uint16_t index = 0x1800;
	uint8_t subindex = 0x03;
	
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2B;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_T;
	TxMessage.Data[5] = param_T>> 8;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
// 读取电流
void MOTOR_APSL2DB_Read_CurrentLimt(uint16_t ID)
{
	MOTOR_APSL2DB_Read_Cmd(ID,0x6073,0x00); // 电流环限制电流
}
// 设置电机电流环工作电流
void MOTOR_APSL2DB_Set_CurrentLimt(uint16_t ID, uint16_t curr_value)
{
	uint16_t index = 0x6073;
	uint8_t subindex = 0x00;
	uint16_t valu = (curr_value*96.5)/1;// curr_value*1.414*2048/33 驱动器最大电流
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2B; //发送1个字节是0x2F 2个字节是0x2B  4个字节是 0x23
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = valu; 
	TxMessage.Data[5] = valu>>8;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
//梯形加速 
void MOTOR_APSL2DB_Set_Trap_Accel(uint16_t ID, uint16_t curr_value)
{
	uint16_t index = 0x6083;
	uint8_t subindex = 0x00;
	uint32_t valu = (curr_value*67.1)/1;// [DEC]=[rps/s]*256*[反馈精度]/15625      反馈精度=4096
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x23; //发送1个字节是0x2F 2个字节是0x2B  4个字节是 0x23
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = valu; 
	TxMessage.Data[5] = valu>>8;
	TxMessage.Data[6] = valu>>16;
	TxMessage.Data[7] = valu>>24;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
void MOTOR_APSL2DB_Set_Trap_Decel(uint16_t ID, uint16_t curr_value)
{
	uint16_t index = 0x6084;
	uint8_t subindex = 0x00;
	uint32_t valu = (curr_value*67.1)/1;// [DEC]=[rps/s]*256*[反馈精度]/15625      反馈精度=4096
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x23; //发送1个字节是0x2F 2个字节是0x2B  4个字节是 0x23
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = valu; 
	TxMessage.Data[5] = valu>>8;
	TxMessage.Data[6] = valu>>16;
	TxMessage.Data[7] = valu>>24;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}


/*
*读取编号为ID的电机编码器速度和位置
// 前4位为速度  后4位是位置
//返回数据正数是低位在前，高位在后
//负数是返回的补码
*/
void APS_L2DB_Read_Position(uint16_t ID)
{
	uint8_t cmd_id = 0x40;
	uint16_t index = 0x6063;
	uint8_t subindex = 0x00;
	
    CanTxMsg TxMessage;
//	TxMessage.ExtId = 0x600+ID;
//	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = cmd_id;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}
// 读取电流
void APS_L2DB_Read_Current(uint16_t ID)
{
	uint8_t cmd_id = 0x40;
	uint16_t index = 0x6078;
	uint8_t subindex = 0x00;
	
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = cmd_id;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}

// 读取温度
void APS_L2DB_Read_Temp(uint16_t ID)
{
	 MOTOR_APSL2DB_Read_Cmd(ID,0x60F7,0x0B); //驱动器实际温度，单位℃
}


// 电机状态读取命令
// 如果接收数据为 1 个字节,则接收命令字为 0x4F；
// 如果接收数据为 2 个字节,则接收命令字为 0x4B；
// 如果接收数据为 4 个字节,则接收命令字为 0x43；
// 如果接收数据存在错误,则接收命令字为 0x80
void MOTOR_APSL2DB_Read_Cmd(uint16_t ID,uint16_t index,uint8_t subindex)
{
	uint8_t cmd_id = 0x40; //状态读取的命令控制字固定为 0x40
	CanTxMsg TxMessage;
//	TxMessage.ExtId = 0x600+ID;
//	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = cmd_id;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}

// 电机参数设置
// 如果接收数据为 1 个字节,则接收命令字为 0x4F；
// 如果接收数据为 2 个字节,则接收命令字为 0x4B；
// 如果接收数据为 4 个字节,则接收命令字为 0x43；
// 如果接收数据存在错误,则接收命令字为 0x80
void MOTOR_APSL2DB_Set_Parameter(uint16_t ID,uint16_t index,uint8_t subindex)
{
	uint8_t cmd_id = 0x2F; //如果待发数据为 1 个字节,则发送命令字为 0x2F；
                           //  2 个字节,则发送命令字为 0x2B；  4 个字节, 发送命令字为 0x23.
	CanTxMsg TxMessage; 
//	TxMessage.ExtId = 0x600+ID;
//	TxMessage.IDE = CAN_ID_EXT;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = cmd_id;
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = 0x00;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // 发送消息  
}


void MOTOR_APSL2DB_Measurements_Analy(void)
{
	if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x18) //行进电机 安普斯 CAN数据解析 PDO数据上传
	{
		tem_int32.int32_value =0; //读取电机转动速度 单位 rpm
		tem_int32.uint8_value[0] = CAN1_RxMessage.Data[0]; //电机 PDO上传的RPM放大了1000倍
		tem_int32.uint8_value[1] = CAN1_RxMessage.Data[1];
		tem_int32.uint8_value[2] = CAN1_RxMessage.Data[2];
		tem_int32.uint8_value[3] = CAN1_RxMessage.Data[3];

		if((CAN1_RxMessage.StdId-0x181)<2)
			moto_chassis[CAN1_RxMessage.StdId-0x181].speed_rpm = -tem_int32.int32_value;
		else
			moto_chassis[CAN1_RxMessage.StdId-0x181].speed_rpm = tem_int32.int32_value; 
		
		tem_int32.int32_value =0; //读取电机转动位置  L2DB4830-CAFX_XXXX 是12bit的分辨率  即4096
		tem_int32.uint8_value[0] = CAN1_RxMessage.Data[4];
		tem_int32.uint8_value[1] = CAN1_RxMessage.Data[5];
		tem_int32.uint8_value[2] = CAN1_RxMessage.Data[6];
		tem_int32.uint8_value[3] = CAN1_RxMessage.Data[7];
		if((CAN1_RxMessage.StdId-0x181)<2)
			moto_chassis[CAN1_RxMessage.StdId-0x181].angle = -tem_int32.int32_value; 
		else
			moto_chassis[CAN1_RxMessage.StdId-0x181].angle = tem_int32.int32_value; 
	 
			
	}
	else if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x58) //行进电机 安普斯 CAN数据解析
	{
		if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x63 && CAN1_RxMessage.Data[3] == 0x00)
		{
			tem_int32.int32_value =0; //读取电机转动位置
			tem_int32.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int32.uint8_value[1] = CAN1_RxMessage.Data[5];
			tem_int32.uint8_value[2] = CAN1_RxMessage.Data[6];
			tem_int32.uint8_value[3] = CAN1_RxMessage.Data[7];
			moto_chassis[CAN1_RxMessage.StdId-0x581].angle = tem_int32.int32_value;
			//printf("\n\n moto_chassis id:%d   angle:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].angle);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x18)
		{
			tem_int16.int16_value =0; //读取rpm   整数单位
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm = tem_int16.int16_value;
			//printf("\n\n moto_chassis id:%d   speed_rpm:%d  \n",CAN1_RxMessage.StdId-0x581,tem_int16.int16_value);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x19)
		{
			tem_int32.int32_value =0; //读取rpm   0.001 rpm
			tem_int32.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int32.uint8_value[1] = CAN1_RxMessage.Data[5];
			tem_int32.uint8_value[2] = CAN1_RxMessage.Data[6];
			tem_int32.uint8_value[3] = CAN1_RxMessage.Data[7];
			moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm = tem_int32.int32_value;
			//printf("\n\n moto_chassis id:%d   speed_rpm(0.001):%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x78 && CAN1_RxMessage.Data[3] == 0x00)
		{
			tem_int16.int16_value =0; //实际电流
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].real_current = tem_int16.int16_value*87.75/1; // 87.75 = 1.414*2048/33
			//printf("\n\n moto_chassis id:%d   real_current:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].real_current);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x12)
		{
			tem_int16.int16_value =0; //实际直流母线电压
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].Voltage = tem_int16.int16_value;
			//printf("\n\n moto_chassis id:%d   Voltage:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Voltage);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x0B)
		{
			tem_int16.int16_value =0; //温度
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].Temp = tem_int16.int16_value;
			//printf("\n\n moto_chassis id:%d   Temp:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Temp);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x41 && CAN1_RxMessage.Data[3] == 0x00)//--------驱动器状态
		{
			moto_chassis[CAN1_RxMessage.StdId-0x581].driver_status = CAN1_RxMessage.Data[4];
		}
		else if(CAN1_RxMessage.Data[2] == 0x26 && CAN1_RxMessage.Data[1] == 0x01 && CAN1_RxMessage.Data[3] == 0x00)
		{
			moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB = CAN1_RxMessage.Data[4];
			moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB = CAN1_RxMessage.Data[5];
		}		
	}
} 
//调试PDO自动上报电机状态数据函数
void MOTOR_APSL2DB_PDO_Debug(void)
{
	printf("PDO ID 1-4 speed_rpm:%d \t %d \t%d \t%d \t\n",moto_chassis[0].speed_rpm
			,moto_chassis[1].speed_rpm,moto_chassis[2].speed_rpm,moto_chassis[3].speed_rpm);
//	printf("PDO ID 1-4 angle:%d \t %d \t%d \t%d \t\n",moto_chassis[0].angle
//			,moto_chassis[1].angle,moto_chassis[2].angle,moto_chassis[3].angle);
}
// 用字符串打印行进电机驱动器信息
void MOTOR_APSL2DB_Measurements_Debug(void)
{
	if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x18) //行进电机 安普斯 CAN数据解析 PDO数据上传
	{
		// PDO 上传模式下 RPM单位为(rmp扩大了1000)
		printf("Motor APSL2DB id:%d  PDO speed_rpm:%d  angle:%d\n",CAN1_RxMessage.StdId-0x181,moto_chassis[CAN1_RxMessage.StdId-0x181].speed_rpm,moto_chassis[CAN1_RxMessage.StdId-0x181].angle);
	}
	else if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x58) //行进电机 安普斯 CAN数据解析
	{
		if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x63 && CAN1_RxMessage.Data[3] == 0x00)//读取电机转动位置
			printf("Motor APSL2DB id:%d   angle:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].angle);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x18)//读取rpm   整数单位
			printf("Motor APSL2DB id:%d   speed_rpm:%d  \n",CAN1_RxMessage.StdId-0x581,tem_int16.int16_value);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x19)//读取rpm   0.001 rpm
			printf("Motor APSL2DB id:%d   speed_rpm(0.001):%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x78 && CAN1_RxMessage.Data[3] == 0x00)//实际电流
			printf("Motor APSL2DB id:%d   real_current:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].real_current);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x12)//实际直流母线电压
			printf("Motor APSL2DB id:%d   Voltage:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Voltage);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x0B)//温度
			printf("Motor APSL2DB id:%d   Temp:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Temp);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x41 && CAN1_RxMessage.Data[3] == 0x00)//--------驱动器状态
		{
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].driver_status == 0x08)
			{
				printf("Motor APSL2DB driver  id:%d  error !!! \n",moto_chassis[CAN1_RxMessage.StdId-0x581].driver_status);
			}
			else if(moto_chassis[CAN1_RxMessage.StdId-0x581].driver_status == 0x00)
			{
				printf("Motor APSL2DB driver  id:%d  is OK !!! \n",moto_chassis[CAN1_RxMessage.StdId-0x581].driver_status);
			} 
		}
		else if(CAN1_RxMessage.Data[2] == 0x26 && CAN1_RxMessage.Data[1] == 0x01 && CAN1_RxMessage.Data[3] == 0x00)
		{
			printf("Motor APSL2DB driver error code: \t");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x01) printf("内部错误 \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x02) printf("编码器 ABZ 信号错误\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x04) printf("编码器 UVW 信号错误\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x08) printf("编码器计数错误\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x10) printf("驱动器温度过高 \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x20) printf("驱动器母线电压过高\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x40) printf("驱动器母线电压过低\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x80) printf("驱动器输出短路\n");
			
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x01) printf("驱动器制动电阻过温报警 \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x02) printf("实际跟随误差超过允许值\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x04) printf("保留备用\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x08) printf("I2*T 故障(驱动器或电机过载)\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x10) printf("速度跟随误差超过允许值 \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x20) printf("电机过温报警\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x40) printf("寻找电机错误(通讯式编码器)\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x80) printf("通信掉线报警\n");			
		}
		//------- 电机属性
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3A)
		{
			printf("本机型号-系列:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3B)
		{
			printf("本机型号-电压与电流等级:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3C)
		{
			printf("本机型号-反馈与总线类型:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3D)
		{
			printf("本机型号:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x25 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0xF1)
		{
			printf("固件日期:%d  \n",(CAN1_RxMessage.Data[7]<<24|CAN1_RxMessage.Data[6]<<16|CAN1_RxMessage.Data[5]<<8|CAN1_RxMessage.Data[4]));
		}
		else if(CAN1_RxMessage.Data[2] == 0x25 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0xF3)
		{
			printf("硬件版本:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
	}
}
