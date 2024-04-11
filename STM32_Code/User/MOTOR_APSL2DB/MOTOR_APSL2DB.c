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

extern CanRxMsg CAN1_RxMessage;				 //���ջ�����
extern volatile uint8_t flag_can1;	
/**
* @funtion ����˹ L2DB ������   CAN open����Э��
* @Brief        
* @Param		
* @Retval	None
* @Date     2023/4/28
* @maker    crp
*/

void MOTOR_APSL2DB_Init(void)
{
	uint8_t index_i=0;
	uint16_t delay_MAX = 50; //��ʱʱ��
	uint16_t init_times = 0; //��ʼ������
	uint16_t init_times_MAX = 5;
	
	// step0  0x86 �������������
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
	// step1  ���CAN���������ӷ�����
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_Model(index_i,3);// ����1-4�ŵ��Ϊ�ٶ�ģʽ
		MOTOR_Delay(delay_MAX);
		init_times = 0;
		while(flag_can1==0 && (init_times++ < init_times_MAX))
		{
			flag_can1 = 0;
			if(CAN1_RxMessage.StdId == 0x580+index_i)
			{
				CAN1_RxMessage.StdId =0x00;
				;//������ջ���
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
	
	// step2  0x86 �������������
    for(index_i=1;index_i<5;index_i++)
		MOTOR_APSL2DB_Enable(index_i,0x86);MOTOR_Delay(delay_MAX);
	
	 // step3  ����ϵ�(ʹ��)
    for(index_i=1;index_i<5;index_i++)
	{  
		MOTOR_APSL2DB_Enable(index_i,0x0F);MOTOR_Delay(delay_MAX);
		init_times = 0;
		while(flag_can1==0 && (init_times++ < init_times_MAX))
		{
			flag_can1 = 0;
			if((CAN1_RxMessage.StdId == 0x580+index_i) && (CAN1_RxMessage.Data[0] == 0x60))
			{
				break;//������ջ���
			}
		}
		if(init_times >= init_times_MAX)
		{
			printf("Motro APSL2DB: %d enable failed\n",index_i);
		}
	}
	printf("Motro APSL2DB: Set Motor IS 1-4 Target speed 0");
	// step4    ���õ��Ŀ���ٶ�Ϊ0 
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_RPM(index_i,0);MOTOR_Delay(delay_MAX);
	}
	
	// step5    ���õ�����Ƶ��� 25A
	printf("Motro APSL2DB: Set Motor IS 1-4 Current Limt 25");
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_CurrentLimt(index_i,28);MOTOR_Delay(delay_MAX);
	}
	
	
	// step6  ���μ��١�����
	printf("Motro APSL2DB: Set Motor IS 1-4 Trapezoidal Acceleration and Deceleration ");
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Set_Trap_Accel(index_i,5);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);
		MOTOR_APSL2DB_Set_Trap_Decel(index_i,13);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);
	}
	 
 
	
	// step7    ���õ��λ���Զ��ϱ�  PDO 
	for(index_i=1;index_i<5;index_i++)
	{
		MOTOR_APSL2DB_Enable_PDO(index_i,1);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);MOTOR_Delay(delay_MAX);
	}

	//APS_L2DB_Read_Position(1);  
}

// ��ȡ����̼���Ϣ �Ա�֤�������ȷ������CAN������
uint8_t MOTOR_APSL2DB_Infor_Get(void)
{
	uint8_t index_i=0,j=0;
	uint16_t delay_MAX = 50; //��ʱʱ��
	uint16_t init_times;
	uint16_t init_times_MAX = 50;
	//0x3A �����ͺ�-ϵ��  Ĭ��:L2DB
	//0x30003B 32U,RO ASCII �����ͺ�-��ѹ������ȼ�
	//0x103C 0x30003C   
	//�����ͺ�-��������������  Ĭ��(485�汾):CAFR   Ĭ��(CAN�汾):CAFC
	//0x103D 0x30003D  �����ͺ�-�䷽ 
						//Ĭ��(7Nm���):ASL3 = 0x41534C33
						//Ĭ��(10Nm���):ASM4 = 0x41536D34
						//Ĭ��(15Nm���):ASM5 = 0x41536D35
						//Ĭ��(30Nm���):ASH8= 0x41534838
	//0x1007 0x2500F1  �̼����� //��ʽ:20190114 ����2019��1��14��
	//0x1009 0x2500F3 32U,RO ASCII Ӳ���汾
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
 

// ���õ������ģʽ   3 ���Ӽ��ٿ��Ƶ��ٶ�ģʽ -3 �����ٶ�ģʽ 4 ����ģʽ 1 λ��ģʽ
void MOTOR_APSL2DB_Set_Model(uint16_t ID, int8_t param_value)
{
	uint16_t index = 0x6060;
	uint8_t subindex = 0x00;
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2F; //һ���ֽ�
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = param_value;
	TxMessage.Data[5] = 0x00;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
// ���õ������ģʽ   0x06 ����ϵ磨���ᣩ 0x0F ����ϵ�(ʹ��)  0x86 �������������
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
// ���ü�ͣ�Ժ���Ҫ��ʹ�ü�ͣ���������������ܿ���
// ��ͣ 1:��ͣ��Ч 0:��ͣ���
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}

// �趨����ٶ� ��λRPM
void MOTOR_APSL2DB_Set_RPM(uint16_t ID, double speed_rpm)
{
	uint16_t index = 0x60FF;
	uint8_t subindex = 0x00;
	// �����ٶ�  ([rpm]*512*[4096])/1875  = rpm*1118.4810666
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}



// ͨѶ����ͣ����ʱ param_value=0 ��ʹ��  1 ʹ��
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
// ͨѶ����ͣ����ʱ���� ��λms �����߶��߳�������ʱ��,���������
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
// �Զ��ϱ�ʵ��λ�ú�ʵ���ٶ� param_value=0 ��ʹ��  1 ʹ��
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
// ����PDO���� ��λms  Ĭ��10ms
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
// ��ȡ����
void MOTOR_APSL2DB_Read_CurrentLimt(uint16_t ID)
{
	MOTOR_APSL2DB_Read_Cmd(ID,0x6073,0x00); // ���������Ƶ���
}
// ���õ����������������
void MOTOR_APSL2DB_Set_CurrentLimt(uint16_t ID, uint16_t curr_value)
{
	uint16_t index = 0x6073;
	uint8_t subindex = 0x00;
	uint16_t valu = (curr_value*96.5)/1;// curr_value*1.414*2048/33 ������������
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x2B; //����1���ֽ���0x2F 2���ֽ���0x2B  4���ֽ��� 0x23
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = valu; 
	TxMessage.Data[5] = valu>>8;
	TxMessage.Data[6] = 0x00;
	TxMessage.Data[7] = 0x00;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
//���μ��� 
void MOTOR_APSL2DB_Set_Trap_Accel(uint16_t ID, uint16_t curr_value)
{
	uint16_t index = 0x6083;
	uint8_t subindex = 0x00;
	uint32_t valu = (curr_value*67.1)/1;// [DEC]=[rps/s]*256*[��������]/15625      ��������=4096
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x23; //����1���ֽ���0x2F 2���ֽ���0x2B  4���ֽ��� 0x23
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = valu; 
	TxMessage.Data[5] = valu>>8;
	TxMessage.Data[6] = valu>>16;
	TxMessage.Data[7] = valu>>24;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
void MOTOR_APSL2DB_Set_Trap_Decel(uint16_t ID, uint16_t curr_value)
{
	uint16_t index = 0x6084;
	uint8_t subindex = 0x00;
	uint32_t valu = (curr_value*67.1)/1;// [DEC]=[rps/s]*256*[��������]/15625      ��������=4096
    CanTxMsg TxMessage;
	TxMessage.StdId = 0x600+ID;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = 0x23; //����1���ֽ���0x2F 2���ֽ���0x2B  4���ֽ��� 0x23
	TxMessage.Data[1] = index;
	TxMessage.Data[2] = index >> 8;
	TxMessage.Data[3] = subindex;
	TxMessage.Data[4] = valu; 
	TxMessage.Data[5] = valu>>8;
	TxMessage.Data[6] = valu>>16;
	TxMessage.Data[7] = valu>>24;
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}


/*
*��ȡ���ΪID�ĵ���������ٶȺ�λ��
// ǰ4λΪ�ٶ�  ��4λ��λ��
//�������������ǵ�λ��ǰ����λ�ں�
//�����Ƿ��صĲ���
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}
// ��ȡ����
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}

// ��ȡ�¶�
void APS_L2DB_Read_Temp(uint16_t ID)
{
	 MOTOR_APSL2DB_Read_Cmd(ID,0x60F7,0x0B); //������ʵ���¶ȣ���λ��
}


// ���״̬��ȡ����
// �����������Ϊ 1 ���ֽ�,�����������Ϊ 0x4F��
// �����������Ϊ 2 ���ֽ�,�����������Ϊ 0x4B��
// �����������Ϊ 4 ���ֽ�,�����������Ϊ 0x43��
// ����������ݴ��ڴ���,�����������Ϊ 0x80
void MOTOR_APSL2DB_Read_Cmd(uint16_t ID,uint16_t index,uint8_t subindex)
{
	uint8_t cmd_id = 0x40; //״̬��ȡ����������̶ֹ�Ϊ 0x40
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}

// �����������
// �����������Ϊ 1 ���ֽ�,�����������Ϊ 0x4F��
// �����������Ϊ 2 ���ֽ�,�����������Ϊ 0x4B��
// �����������Ϊ 4 ���ֽ�,�����������Ϊ 0x43��
// ����������ݴ��ڴ���,�����������Ϊ 0x80
void MOTOR_APSL2DB_Set_Parameter(uint16_t ID,uint16_t index,uint8_t subindex)
{
	uint8_t cmd_id = 0x2F; //�����������Ϊ 1 ���ֽ�,����������Ϊ 0x2F��
                           //  2 ���ֽ�,����������Ϊ 0x2B��  4 ���ֽ�, ����������Ϊ 0x23.
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
	CAN_Transmit(MOTOR_CAN, &TxMessage); // ������Ϣ  
}


void MOTOR_APSL2DB_Measurements_Analy(void)
{
	if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x18) //�н���� ����˹ CAN���ݽ��� PDO�����ϴ�
	{
		tem_int32.int32_value =0; //��ȡ���ת���ٶ� ��λ rpm
		tem_int32.uint8_value[0] = CAN1_RxMessage.Data[0]; //��� PDO�ϴ���RPM�Ŵ���1000��
		tem_int32.uint8_value[1] = CAN1_RxMessage.Data[1];
		tem_int32.uint8_value[2] = CAN1_RxMessage.Data[2];
		tem_int32.uint8_value[3] = CAN1_RxMessage.Data[3];

		if((CAN1_RxMessage.StdId-0x181)<2)
			moto_chassis[CAN1_RxMessage.StdId-0x181].speed_rpm = -tem_int32.int32_value;
		else
			moto_chassis[CAN1_RxMessage.StdId-0x181].speed_rpm = tem_int32.int32_value; 
		
		tem_int32.int32_value =0; //��ȡ���ת��λ��  L2DB4830-CAFX_XXXX ��12bit�ķֱ���  ��4096
		tem_int32.uint8_value[0] = CAN1_RxMessage.Data[4];
		tem_int32.uint8_value[1] = CAN1_RxMessage.Data[5];
		tem_int32.uint8_value[2] = CAN1_RxMessage.Data[6];
		tem_int32.uint8_value[3] = CAN1_RxMessage.Data[7];
		if((CAN1_RxMessage.StdId-0x181)<2)
			moto_chassis[CAN1_RxMessage.StdId-0x181].angle = -tem_int32.int32_value; 
		else
			moto_chassis[CAN1_RxMessage.StdId-0x181].angle = tem_int32.int32_value; 
	 
			
	}
	else if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x58) //�н���� ����˹ CAN���ݽ���
	{
		if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x63 && CAN1_RxMessage.Data[3] == 0x00)
		{
			tem_int32.int32_value =0; //��ȡ���ת��λ��
			tem_int32.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int32.uint8_value[1] = CAN1_RxMessage.Data[5];
			tem_int32.uint8_value[2] = CAN1_RxMessage.Data[6];
			tem_int32.uint8_value[3] = CAN1_RxMessage.Data[7];
			moto_chassis[CAN1_RxMessage.StdId-0x581].angle = tem_int32.int32_value;
			//printf("\n\n moto_chassis id:%d   angle:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].angle);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x18)
		{
			tem_int16.int16_value =0; //��ȡrpm   ������λ
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm = tem_int16.int16_value;
			//printf("\n\n moto_chassis id:%d   speed_rpm:%d  \n",CAN1_RxMessage.StdId-0x581,tem_int16.int16_value);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x19)
		{
			tem_int32.int32_value =0; //��ȡrpm   0.001 rpm
			tem_int32.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int32.uint8_value[1] = CAN1_RxMessage.Data[5];
			tem_int32.uint8_value[2] = CAN1_RxMessage.Data[6];
			tem_int32.uint8_value[3] = CAN1_RxMessage.Data[7];
			moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm = tem_int32.int32_value;
			//printf("\n\n moto_chassis id:%d   speed_rpm(0.001):%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x78 && CAN1_RxMessage.Data[3] == 0x00)
		{
			tem_int16.int16_value =0; //ʵ�ʵ���
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].real_current = tem_int16.int16_value*87.75/1; // 87.75 = 1.414*2048/33
			//printf("\n\n moto_chassis id:%d   real_current:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].real_current);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x12)
		{
			tem_int16.int16_value =0; //ʵ��ֱ��ĸ�ߵ�ѹ
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].Voltage = tem_int16.int16_value;
			//printf("\n\n moto_chassis id:%d   Voltage:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Voltage);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x0B)
		{
			tem_int16.int16_value =0; //�¶�
			tem_int16.uint8_value[0] = CAN1_RxMessage.Data[4];
			tem_int16.uint8_value[1] = CAN1_RxMessage.Data[5];
			moto_chassis[CAN1_RxMessage.StdId-0x581].Temp = tem_int16.int16_value;
			//printf("\n\n moto_chassis id:%d   Temp:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Temp);
		}
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x41 && CAN1_RxMessage.Data[3] == 0x00)//--------������״̬
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
//����PDO�Զ��ϱ����״̬���ݺ���
void MOTOR_APSL2DB_PDO_Debug(void)
{
	printf("PDO ID 1-4 speed_rpm:%d \t %d \t%d \t%d \t\n",moto_chassis[0].speed_rpm
			,moto_chassis[1].speed_rpm,moto_chassis[2].speed_rpm,moto_chassis[3].speed_rpm);
//	printf("PDO ID 1-4 angle:%d \t %d \t%d \t%d \t\n",moto_chassis[0].angle
//			,moto_chassis[1].angle,moto_chassis[2].angle,moto_chassis[3].angle);
}
// ���ַ�����ӡ�н������������Ϣ
void MOTOR_APSL2DB_Measurements_Debug(void)
{
	if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x18) //�н���� ����˹ CAN���ݽ��� PDO�����ϴ�
	{
		// PDO �ϴ�ģʽ�� RPM��λΪ(rmp������1000)
		printf("Motor APSL2DB id:%d  PDO speed_rpm:%d  angle:%d\n",CAN1_RxMessage.StdId-0x181,moto_chassis[CAN1_RxMessage.StdId-0x181].speed_rpm,moto_chassis[CAN1_RxMessage.StdId-0x181].angle);
	}
	else if((CAN1_RxMessage.StdId &0xffff )>>4 == 0x58) //�н���� ����˹ CAN���ݽ���
	{
		if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x63 && CAN1_RxMessage.Data[3] == 0x00)//��ȡ���ת��λ��
			printf("Motor APSL2DB id:%d   angle:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].angle);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x18)//��ȡrpm   ������λ
			printf("Motor APSL2DB id:%d   speed_rpm:%d  \n",CAN1_RxMessage.StdId-0x581,tem_int16.int16_value);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF9 && CAN1_RxMessage.Data[3] == 0x19)//��ȡrpm   0.001 rpm
			printf("Motor APSL2DB id:%d   speed_rpm(0.001):%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].speed_rpm);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x78 && CAN1_RxMessage.Data[3] == 0x00)//ʵ�ʵ���
			printf("Motor APSL2DB id:%d   real_current:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].real_current);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x12)//ʵ��ֱ��ĸ�ߵ�ѹ
			printf("Motor APSL2DB id:%d   Voltage:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Voltage);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0xF7 && CAN1_RxMessage.Data[3] == 0x0B)//�¶�
			printf("Motor APSL2DB id:%d   Temp:%d  \n",CAN1_RxMessage.StdId-0x581,moto_chassis[CAN1_RxMessage.StdId-0x581].Temp);
		else if(CAN1_RxMessage.Data[2] == 0x60 && CAN1_RxMessage.Data[1] == 0x41 && CAN1_RxMessage.Data[3] == 0x00)//--------������״̬
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
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x01) printf("�ڲ����� \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x02) printf("������ ABZ �źŴ���\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x04) printf("������ UVW �źŴ���\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x08) printf("��������������\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x10) printf("�������¶ȹ��� \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x20) printf("������ĸ�ߵ�ѹ����\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x40) printf("������ĸ�ߵ�ѹ����\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_LSB & 0x80) printf("�����������·\n");
			
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x01) printf("�������ƶ�������±��� \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x02) printf("ʵ�ʸ�����������ֵ\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x04) printf("��������\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x08) printf("I2*T ����(��������������)\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x10) printf("�ٶȸ�����������ֵ \n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x20) printf("������±���\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x40) printf("Ѱ�ҵ������(ͨѶʽ������)\n");
			if(moto_chassis[CAN1_RxMessage.StdId-0x581].error_code_HSB & 0x80) printf("ͨ�ŵ��߱���\n");			
		}
		//------- �������
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3A)
		{
			printf("�����ͺ�-ϵ��:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3B)
		{
			printf("�����ͺ�-��ѹ������ȼ�:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3C)
		{
			printf("�����ͺ�-��������������:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x30 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0x3D)
		{
			printf("�����ͺ�:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
		else if(CAN1_RxMessage.Data[2] == 0x25 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0xF1)
		{
			printf("�̼�����:%d  \n",(CAN1_RxMessage.Data[7]<<24|CAN1_RxMessage.Data[6]<<16|CAN1_RxMessage.Data[5]<<8|CAN1_RxMessage.Data[4]));
		}
		else if(CAN1_RxMessage.Data[2] == 0x25 && CAN1_RxMessage.Data[1] == 0x00 && CAN1_RxMessage.Data[3] == 0xF3)
		{
			printf("Ӳ���汾:%c%c%c%c  \n",CAN1_RxMessage.Data[7],CAN1_RxMessage.Data[6],CAN1_RxMessage.Data[5],CAN1_RxMessage.Data[4]);
		}
	}
}
