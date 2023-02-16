#include "stm32f10x.h"
#include "math.h"
#include "bsp_uart.h"
#include "bsp_can.h"

#include "DBUS.h" 
#include "PID.h" 
#include "DJI_Motor.h"  
#include "Mick_IO.h"

// C620������Ʒ�Χ-16384~0~16384 ��Ӧ������ -20A-0-20A ����Ƶ��1KHz
// C620������� ��еת�ӽǶ� 0-360��  ��Ӧ0-8191  ת���ٶ�ΪRPM��λ  �¶ȵ�λΪ��
// 3508���������ת��482  ���ת��469 �����10A �������ת��3 N*M
// ����M3508����䱸��1:19�ļ����� ���ת�ӵ����ת��Ϊ 482*19(RPM)

//�����ֳ�ʹ�õ���10Ӣ�������   25.4cm
// �����ķ�ֵ�ֱ��Ϊ 152.5mm

/***********************************************************

// �����ĸ�������Ӧ�˶�ѧģ�ͣ���С�������ٶ�ת��Ϊÿһ�����ӵ�ת��
void DiffX4_Wheel_Speed_Model(float speed_x,float speed_w)
void DiffX4__Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4)
void Mecanum_Wheel_Speed_Model(int16_t speed_x,int16_t speed_y,int16_t speed_w)
void Mecanum_Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4)




***********************************************************/

#define ABS(x)		((x>0)? (x): (-x)) 
#define MAX_WHEEL_SPEED 9158  //������ת�ٵ�λ RPM   482*19=9158


int set_v,set_spd[4]; 
uint32_t motor_upload_counter; //�����ϴ�������
volatile moto_measure_t moto_chassis[4] = {0};//4 chassis moto
volatile moto_measure_t moto_info;
pid_t pid_spd[4]; //�ĸ������Ӧ��PID�ṹ��
command_t recived_cmd;  // ����2�д���λ��������������
extern rc_info_t rc;  //DBUS ����





// ��ʼ��PID����
void DJI_Motor_Init(void)
{
	unsigned char i=0;
	for(i=0; i<4; i++)
	{
		PID_struct_init(&pid_spd[i], POSITION_PID, 20000, 5000,15.8f,	0.019f,	0.0f);  //4 motos angular rate closeloop.
	}
	set_spd[0] = set_spd[1] = set_spd[2] = set_spd[3] = 0;//�����ĸ������Ŀ���ٶ�
	
	motor_upload_counter=0;
}

// C620������Ʒ�Χ-16384~0~16384 ��Ӧ������ -20A-0-20A ����Ƶ��1KHz
// C620������� ��еת�ӽǶ� 0-360��  ��Ӧ0-8191  ת���ٶ�ΪRPM��λ  �¶ȵ�λΪ��
//3508���������ת��482  ���ת��469 �����10A �������ת��3 N*M
/**
* @funtion			�����ֲ���ģ��
* @Brief        �����趨��X������ٶȺͺ�����ٶ�W 
*  							������ĸ����ӵ��ٶ�
* @Param		speed_x С��X��������ٶ�(m/s)  speed_wС����ת���ٶ�(rad/s)
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void DiffX4_Wheel_Speed_Model(float speed_x,float speed_w)
{
	float v1=0,v2=0,v3=0,v4=0;
	float L=0.68;//�������ӵļ��
	
  if((speed_x<-5) || (speed_x>5)) 
		speed_x=0;
  if((speed_w<-5) || (speed_w>5)) 
		speed_w=0;
 
	v1 = speed_x+speed_w*L;
	v4 = -(speed_x-speed_w*L);
	
	// ����ʹ�õ���10Ӣ������� ֱ��Ϊ25.4cm �뾶Ϊ12.7cm
  //60/(2*PI*R) = 75.191313 
	//����19�����ڵ����1:19�ļ��ٱ�
	
	v1 =75.191313*v1*19;// from (m/s) to (RPM)
	v4 =75.191313*v4*19;
	
	v2 = v1; 
	v3 = v4;
	
	Mecanum_Wheel_Rpm_Model(v1,v2,v3,v4);
}
/**
* @funtion			�����ֲ���ģ��
* @Brief        �趨4�����ӵ�ת��
* @Param		speed_x С��X��������ٶ�(m/s)  speed_wС����ת���ٶ�(rad/s)
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void DiffX4__Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{
	// ���ڶ����趨���ӵ�ת�٣���������ģ���޹أ�
	// Ϊ��ͳһ��ʾ ���԰���δ�������������
	Mecanum_Wheel_Rpm_Model(v1,v2,v3,v4);
}
/**
* @funtion			�����ķ���˶�ѧģ��(�����ķ�ֳʡ��ס���������)
* @Brief        ��С�����ٶȷֽ��ÿ�����ӵ��ٶ�
* @Param		speed_x С��X��������ٶ�(m/s)  speed_wС����ת���ٶ�(rad/s)
* @Retval		None (�ú����޸ĺ�δ����)
* @Date     20190916
* @maker    crp
*/
void Mecanum_Wheel_Speed_Model(float speed_x,float speed_y,float speed_w)
{
	float v1=0,v2=0,v3=0,v4=0;
	
	float K =0.76; // K=abs(Xn)+abs(Yn) Xn Yn��ʾ���ֵİ�װ����
								//�����־� 40cm ǰ���־�36cm
//	float r_1 = 6.5574;//   6.5574= 1/0.1525 
	 
	if((speed_x<-5) || (speed_x>5)) 
		speed_x=0;
	if((speed_y<-5) || (speed_y>5)) 
		speed_x=0;
	if((speed_w<-5) || (speed_w>5)) 
		speed_w=0;	
	
	v1 =speed_x-speed_y-K*speed_w;
	v2 =speed_x+speed_y-K*speed_w;
	v3 =-(speed_x-speed_y+K*speed_w);
	v4 =-(speed_x+speed_y+K*speed_w);
	
	// �����ķ�ֵ�ֱ��Ϊ 152.5mm     60/(2*PI*R) = 125.23668 
	//����19�����ڵ����1:19�ļ��ٱ�
	
	v1 =125.23668*v1*19;// from (m/s) to (RPM)
	v2 =125.23668*v2*19;
	v3 =125.23668*v3*19;
	v4 =125.23668*v4*19;
	
	Mecanum_Wheel_Rpm_Model(v1,v2,v3,v4);
}



/**
* @funtion	 ����ÿ������PID��������Ŀ��ֵ
* @Brief     ���������λ��RPM��   
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void Mecanum_Wheel_Rpm_Model(int16_t v1,int16_t v2,int16_t v3,int16_t v4)
{	 
	if(v1>MAX_WHEEL_SPEED) 		v1=MAX_WHEEL_SPEED;
	else if(v1< -MAX_WHEEL_SPEED)		v1=-MAX_WHEEL_SPEED;
	else ;
	
	if((v1>-5) && (v1<5))  v1=0;
	
	if(v2>MAX_WHEEL_SPEED) 		v2=MAX_WHEEL_SPEED;
	else if(v2< -MAX_WHEEL_SPEED)		v2=-MAX_WHEEL_SPEED;
	else ;
	
	if((v2>-5) && (v2<5))  v2=0;
	
	if(v3>MAX_WHEEL_SPEED) 		v3=MAX_WHEEL_SPEED;
	else if(v3< -MAX_WHEEL_SPEED)		v3=-MAX_WHEEL_SPEED;
	else ;
	
	if((v3>-5) && (v3<5))  v3=0;
	
	if(v4>MAX_WHEEL_SPEED) 		v4=MAX_WHEEL_SPEED;
	else if(v4< -MAX_WHEEL_SPEED)		v4=-MAX_WHEEL_SPEED;
	else ;	
	
	if((v4>-5) && (v4<5))  v4=0;
   
	set_spd[0] = v1;
	set_spd[1] = v2;
	set_spd[2] = v3;
	set_spd[3] = v4;
}

/**
* @funtion			����PID�������ͨ��CAN���߷��ͳ�ȥ
* @Brief        ����ʱ�жϺ�������,ÿ����һ�κ����������һ��PID���
*								����������ͨ������ CAN_DJI_C620_DataSend(iq1,iq2,iq3,iq4) �·������
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void DJI_Motor_Control(void)
{
	unsigned char i=0;// ����ٶȿ���
	for(i=0; i<4; i++)
	{
		// set_spd Ϊ���õ�Ŀ���ٶ�
		// pid_spd Ϊ�ĸ�Ŀ��ṹ��
		// moto_chassis Ϊ������ת��
		pid_calc(&pid_spd[i], moto_chassis[i].speed_rpm, set_spd[i]);
	}
	// ���͵��ĸ������ת��CAN������
	CAN_DJI_C620_DataSend(pid_spd[0].pos_out,pid_spd[1].pos_out,pid_spd[2].pos_out,pid_spd[3].pos_out);				 
}

/**
* @funtion			��C620 ���CAN�������ݷ��ͺ���
* @Brief        
* @Param		
* @Retval		None
* @Date     2020/8/16
* @maker    crp
*/
void CAN_DJI_C620_DataSend( int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4)
{
  CanTxMsg TxMessage;
	TxMessage.StdId = 0x200;
	TxMessage.IDE = CAN_ID_STD;
	TxMessage.RTR = CAN_RTR_DATA;
	TxMessage.DLC = 0x08;
	TxMessage.Data[0] = iq1 >> 8;
	TxMessage.Data[1] = iq1;
	TxMessage.Data[2] = iq2 >> 8;
	TxMessage.Data[3] = iq2;
	TxMessage.Data[4] = iq3 >> 8;
	TxMessage.Data[5] = iq3;
	TxMessage.Data[6] = iq4 >> 8;
	TxMessage.Data[7] = iq4;
	CAN_Transmit(CAN1, &TxMessage); // ������Ϣ  
}


//����ϱ�����Ƶ��Ϊ1KHZ
void CAN_RxCpltCallback(CanRxMsg* RxMessage)
{
	static uint8_t i;
	//ignore can1 or can2.
	switch(RxMessage->StdId)
		{
		case CAN_3510Moto1_ID:
		case CAN_3510Moto2_ID:
		case CAN_3510Moto3_ID:
		case CAN_3510Moto4_ID:
			{
				i = RxMessage->StdId - CAN_3510Moto1_ID;
				moto_chassis[i].msg_cnt++ <= 50	?	get_moto_offset(&moto_chassis[i], RxMessage) : get_moto_measure(&moto_chassis[i], RxMessage);
				get_moto_measure(&moto_info,RxMessage);
			}
			break;
	   }
}

/*******************************************************************************************
  * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
  * @Brief    ����,3510���ͨ��CAN����������Ϣ
  * @Param		
  * @Retval		None
  * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t *ptr,CanRxMsg* RxMessage)
{
//	u32  sum=0;
//	u8	 i = FILTER_BUF_LEN;	
	/*BUG!!! dont use this para code*/
//	ptr->angle_buf[ptr->buf_idx] = (uint16_t)(hcan->pRxMsg->Data[0]<<8 | hcan->pRxMsg->Data[1]) ;
//	ptr->buf_idx = ptr->buf_idx++ > FILTER_BUF_LEN ? 0 : ptr->buf_idx;
//	while(i){
//		sum += ptr->angle_buf[--i];
//	}
//	ptr->fited_angle = sum / FILTER_BUF_LEN;
	
	
	int delta=0;

	ptr->last_angle = ptr->angle;
	ptr->angle = (uint16_t)(RxMessage->Data[0]<<8 | RxMessage->Data[1]) ; //���ת��
	
	ptr->real_current  = (int16_t)(RxMessage->Data[2]<<8 | RxMessage->Data[3]);
	ptr->speed_rpm = ptr->real_current;	//��������Ϊ���ֵ����Ӧλ��һ������Ϣ
	
	ptr->given_current = (int16_t)(RxMessage->Data[4]<<8 | RxMessage->Data[5])/-5;
	ptr->Temp = RxMessage->Data[6];
	
	if(ptr->speed_rpm > 10 ) //�����ת
	{
		if((ptr->angle - ptr->last_angle) >= 50)
		{
		  delta = ptr->angle - ptr->last_angle;
		}
		else if ((ptr->angle - ptr->last_angle) <= -50)
		{
		  delta = ptr->angle + 8192 - ptr->last_angle;
			ptr->round_cnt++;
		}
		else;
	}
  else if(ptr->speed_rpm < -10) //�����ת
	{
		if ((ptr->angle - ptr->last_angle) <= -50)
		{
			delta =ptr->angle - ptr->last_angle;
		}
		else if((ptr->angle - ptr->last_angle) >= 50)
		{
			delta = ptr->angle - 8192 - ptr->last_angle;
			ptr->round_cnt--;
		}
		else;
	}
	else
		delta=0;
	//ptr->total_angle += delta;
 
	ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

 
// ��ȡ�����ֹʱ���ֵ
void get_moto_offset(moto_measure_t *ptr,CanRxMsg* RxMessage)
{
	ptr->angle = (uint16_t)(RxMessage->Data[0]<<8 | RxMessage->Data[1]) ;
	ptr->offset_angle = ptr->angle;
}





//------------------------------���ڽ���ָ��Э��-------------------------------//
//������2������ת�浽���� UART2_ReBuff��
volatile uint8_t UART2_ReBuff[100];  
volatile uint16_t UART2_ReCont;  
volatile unsigned char UART2_Reflag; 

char DJI_Motor_WriteData_In_Buff(uint8_t *DataBuff,uint16_t datalength)
{
	uint16_t i =0;
	uint16_t rpm_offset =10000; //ת��ƫ��һ��ת
	uint16_t speed_offset =10; //�ٶ�ƫ��10m/s

	for(i=0;i<datalength;i++)
	{
		UART2_ReBuff[i] = *DataBuff;
		DataBuff++;
	}

	UART2_ReCont = datalength;
	UART2_Reflag =0x01;

	recived_cmd.cmd=UART2_ReBuff[3];

	if(recived_cmd.cmd ==0xF1)
	{
		recived_cmd.tag_rpm1 = UART2_ReBuff[4]*256+UART2_ReBuff[5] - rpm_offset;
		recived_cmd.tag_rpm2 = UART2_ReBuff[6]*256+UART2_ReBuff[7] - rpm_offset;
		recived_cmd.tag_rpm3 = UART2_ReBuff[8]*256+UART2_ReBuff[9] - rpm_offset;
		recived_cmd.tag_rpm4 = UART2_ReBuff[10]*256+UART2_ReBuff[11] - rpm_offset;
		recived_cmd.rpm_available=0x01;
		recived_cmd.speed_available=0x00;
	}
	else if(recived_cmd.cmd ==0xF2)
	{
		recived_cmd.tag_speed_x = (UART2_ReBuff[4]*256+UART2_ReBuff[5])/100.0-speed_offset;
		recived_cmd.tag_speed_y = (UART2_ReBuff[6]*256+UART2_ReBuff[7])/100.0-speed_offset;
		recived_cmd.tag_speed_z = (UART2_ReBuff[8]*256+UART2_ReBuff[9])/100.0-speed_offset;

		recived_cmd.rpm_available=0x00;
		recived_cmd.speed_available=0x01;
	}
	else if(recived_cmd.cmd == 0xE1) //��̼�����
	{
		 DJI_Motor_Clear_Odom();
	}
	else if(recived_cmd.cmd == 0xE2) //���ø������IO
	{
		Set_Isolated_Output(1,UART2_ReBuff[4]);
		Set_Isolated_Output(2,UART2_ReBuff[5]);
		Set_Isolated_Output(3,UART2_ReBuff[6]);
		Set_Isolated_Output(4,UART2_ReBuff[7]);
	}
	else
	{
		//recived_cmd.rpm_available=0x00;
		//recived_cmd.speed_available=0x00;
		return 0;
	}
	recived_cmd.flag=0x01;
	
	// ----ִ������
	if((rc.sw1 ==1) && (rc.available))//���ڽ��������ݹ���
	{
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
		else;
		//Delay_10us(50000);
		//DJI_Motor_Show_Message();
	}
	else
	{
		Mecanum_Wheel_Rpm_Model(0,0,0,0);
		return 0;
	}
		

	#if DEBUUG_Motor_RECIVED
		UART_send_string(USART2,"\n  Uart2 recived  data length:  ");
		UART_send_data(USART2,UART2_ReCont);
		UART_send_string(USART2,"  Byte");
	#endif	
	
	return 1;
}


 



/**
*@funtion   ���ڴ�ӡ������Ϣ
*@Brief ���ı��ķ�ʽ��ӡ������ٶ���Ϣ ����ʹ��
*@data 20200816  
*/
void DJI_Motor_Show_Message(void)
{
	unsigned char i=0;
	if(recived_cmd.flag)
	{
		if(recived_cmd.cmd == 0xF1)
		{
			UART_send_string(USART2,"DJI_Motor:"); 
			UART_send_string(USART2,"tag_rpm1:");UART_send_data(USART2,recived_cmd.tag_rpm1);UART_send_char(USART2,'\t');	
			UART_send_string(USART2,"tag_rpm2:");UART_send_data(USART2,recived_cmd.tag_rpm2);UART_send_char(USART2,'\t');	
			UART_send_string(USART2,"tag_rpm3:");UART_send_data(USART2,recived_cmd.tag_rpm3);UART_send_char(USART2,'\t');	
			UART_send_string(USART2,"tag_rpm4:");UART_send_data(USART2,recived_cmd.tag_rpm4);UART_send_char(USART2,'\n');	
		}
		else if(recived_cmd.cmd == 0xF2)
		{
			UART_send_string(USART2,"DJI_Motor:"); 
			UART_send_string(USART2,"tag_speed_x:");UART_send_floatdat(USART2,recived_cmd.tag_speed_x);UART_send_char(USART2,'\t');	
			UART_send_string(USART2,"tag_speed_y:");UART_send_floatdat(USART2,recived_cmd.tag_speed_y);UART_send_char(USART2,'\t');	
			UART_send_string(USART2,"tag_speed_z:");UART_send_floatdat(USART2,recived_cmd.tag_speed_z);UART_send_char(USART2,'\n');	
		}
		else;
	}
	for(i=0;i<4;i++)
	{
		// ��ӡ�ĸ������ת�١�ת�ǡ��¶ȵ���Ϣ
		UART_send_string(USART2,"M"); UART_send_data(USART2,i);UART_send_string(USART2,"�� ");
		UART_send_string(USART2,"v:");UART_send_floatdat(USART2,moto_chassis[i].speed_rpm);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"t_a:");UART_send_floatdat(USART2,moto_chassis[i].total_angle);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"n:");UART_send_floatdat(USART2,moto_chassis[i].round_cnt);UART_send_char(USART2,'\t');	
		UART_send_string(USART2,"a:");UART_send_floatdat(USART2,moto_chassis[i].angle);UART_send_char(USART2,'\n');	
	}
	UART_send_char(USART2,'\n');	
	UART_send_char(USART2,'\n');	
}

/**
*@funtion   ����ϱ���Ϣ��PC��
*@Brief ����Ԥ����Э���ʽ�ϴ��ĸ������ת�١�λ�á��¶ȵ����ػ� 
*@data 20200816  
*/
void DJI_Motor_Upload_Message(void)
{
		unsigned char senddat[70];
		unsigned char i=0,j=0;	
		unsigned int sum=0x00;	
	
		senddat[i++]=0xAE;
		senddat[i++]=0xEA;
		senddat[i++]=0x01;//���ݳ����ں��渳ֵ
		senddat[i++]=0x01;
	
	  //�ϴ�����֡����
		senddat[i++]=(motor_upload_counter>>24);
		senddat[i++]=(motor_upload_counter>>16);
		senddat[i++]=(motor_upload_counter>>8);
		senddat[i++]=(motor_upload_counter);
	
		for(j=0;j<4;j++) //4*11���ֽ�
		{
			senddat[i++] = moto_chassis[j].speed_rpm>>8; //int16
			senddat[i++] = moto_chassis[j].speed_rpm;

			senddat[i++] = moto_chassis[j].total_angle>>24;
			senddat[i++] = moto_chassis[j].total_angle>>16;	
			senddat[i++] = moto_chassis[j].total_angle>>8;		
			senddat[i++] = moto_chassis[j].total_angle;	

			senddat[i++] = moto_chassis[j].round_cnt>>24;
			senddat[i++] = moto_chassis[j].round_cnt>>16;	
			senddat[i++] = moto_chassis[j].round_cnt>>8; //int16
			senddat[i++] = moto_chassis[j].round_cnt;
			
			senddat[i++] = moto_chassis[j].angle>>8; //int16
			senddat[i++] = moto_chassis[j].angle;

			senddat[i++] = moto_chassis[j].Temp;
		}
		
		senddat[2]=i-1; //���ݳ���
		for(j=2;j<i;j++)
			sum+=senddat[j];
    senddat[i++]=sum;
		
		senddat[i++]=0xEF;
		senddat[i++]=0xFE;
		 
		//UART_send_string(USART2,senddat);
		UART_send_buffer(USART2,senddat,i);
		motor_upload_counter++;
}


/**
*@funtion   ��ROS�ڵ�����ʱ�������̼ƣ�ʹ֮���㿪ʼ����
*@Brief ����λ�����չ̶�ָ���·�ʱ�������ú��� �����̼Ƶ��ۼ�ֵ
*@data 20200816 
*/
void DJI_Motor_Clear_Odom(void)
{
	unsigned char j=0;
	for(j=0;j<4;j++)  
	{
		moto_chassis[j].msg_cnt=0;  

		moto_chassis[j].angle=0;
		moto_chassis[j].last_angle=0;
		moto_chassis[j].speed_rpm=0;  
		moto_chassis[j].real_current=0;  
		moto_chassis[j].given_current=0;  
					
		moto_chassis[j].Temp=0;
		moto_chassis[j].offset_angle=0;
		moto_chassis[j].round_cnt=0;
		moto_chassis[j].total_angle=0;	
	}

	motor_upload_counter=0;
	UART_send_string(USART2,"OK");
}

