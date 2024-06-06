#include "stm32f4xx.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_uart.h" 
#include "bsp_usart_dma.h" 
#include "bsp_systick.h"


#include "DBUS/DBUS.h" 
#include "MOTOR_Control/MOTOR_Control.h"  
#include "Mick_IO/Mick_IO.h"

unsigned char DBUS_flag=0;
volatile int32_t ch1_offset_sum=0,ch2_offset_sum=0,ch3_offset_sum=0,ch4_offset_sum=0;
volatile uint8_t rc_counter=0;

rc_info_t rc; // ���ｫDBUS��SBUS����һ���ṹ��
volatile uint8_t Code_Switch_Value;
 
uint8_t faild_times = 0; // ͳ�Ƴ�ʼ��ʧ�ܴ�������������ֱ�Ӹ�λ
/***************************************************************************
* @brief       ң������ʼ������  ��Ҫռ�ô���1��DMA ����ģʽ
* @param[out]  :    
* @param[in]   ��:  
* @retval 
* @maker    crp
* @date 2023-4-26
***************************************************************************/
char RC_Remote_Init(void)
{
	uint32_t i=0;
	My_USART_DMA_config(USART2,100000);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE); 
	
	rc.type =2; // ��DBUS 1      �ֵ�SBUS 2
	Code_Switch_Value |= 0x00; //  ����ģ��  0x00   �����ķ�� 0x01    4ws4wdģ��  0x02    ������0x03
	
	while(rc.available == 0x00)
	{
		i=6553600;
		while(i-->0);
		printf("wait for RC init...\n");
	}
	
	return 0x01;
}

/***************************************************************************
* @brief       DBUS���ڽ��ջص����� 
* @param[out]  rc:   ת��Ϊÿ��ͨ��������
* @param[in]   pData: ���볤��Ϊ18�ֽڵ�����
* @retval 
* @maker    crp
* @date 2019-9-8
***************************************************************************/
char RC_Callback_Handler(uint8_t *pData)
{
	unsigned char i=0;
	if(pData == NULL)
	{
		return 0;
	}
	
	if(rc.type == 1) // ��DBUSЭ��
	{
		rc.ch1 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
		rc.ch2 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5))	& 0x07FF;
		rc.ch3 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
		rc.ch4 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) &0x07FF;
		rc.sw1 = ((pData[5] >> 4) & 0x000C) >> 2;
		rc.sw2 = ((pData[5] >> 4) & 0x0003);
		rc.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
		rc.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
		rc.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
		rc.press_l = pData[12];
		rc.press_r = pData[13];
		rc.v = ((int16_t)pData[14]);// | ((int16_t)pData[15] << 8);
		rc.update =0x01;
	}
	else if(rc.type == 2) // ��ģSBUSЭ��
	{
		// SBUS����
		if((pData[0] == 0x0f && pData[USART2_MAX_RX_LEN-1]==0 && pData[USART2_MAX_RX_LEN-2]==0) )
		{
			i=0;
		}
		else //DMA�ǹ̶����Ƚ��գ������һ�����ݲ���0x0f��ô���������е����ݶ����λ�����������ط���λ
		{	
			if(rc.rc_state == DBUS_INIT) //�����û�г�ʼ���ɹ����յ��˴���֡ͷ ��������Ƭ��
			{
				printf("SBUS data: ");
				for(i=0;i<USART2_MAX_RX_LEN;i++)
				{
					printf("  0x%2x",pData[i]);
				}
				printf(" \r\n");
				printf("SBUS fram head != 0x0f, reset MCU.\n");			
				__set_FAULTMASK(1); // �ر������ж�
				NVIC_SystemReset();// ��λ				
			}
			else // ��������й������յ�����֡ͷ�ͷ�����һ֡ ������
			{
				printf("RUN state:  SBUS fram head != 0x0f, throw away this fram\n");	
				return 0;
			}
		}
		rc.ch1 = ((int16_t)pData[(i+1)%USART2_MAX_RX_LEN] | ((int16_t)pData[(i+2)%USART2_MAX_RX_LEN] << 8)) & 0x07FF;
		rc.ch2 = (((int16_t)pData[(i+2)%USART2_MAX_RX_LEN] >> 3) | ((int16_t)pData[(i+3)%USART2_MAX_RX_LEN] << 5))	& 0x07FF;
		rc.ch3 = (((int16_t)pData[(i+3)%USART2_MAX_RX_LEN] >> 6) | ((int16_t)pData[(i+4)%USART2_MAX_RX_LEN] << 2) |((int16_t)pData[(i+5)%USART2_MAX_RX_LEN] << 10)) & 0x07FF;
		rc.ch4 = (((int16_t)pData[(i+5)%USART2_MAX_RX_LEN] >> 1) | ((int16_t)pData[(i+6)%USART2_MAX_RX_LEN]<<7)) &0x07FF;
		rc.ch5 = ((int16_t)pData[(i+6)%USART2_MAX_RX_LEN] >> 4 | ((int16_t)pData[(i+7)%USART2_MAX_RX_LEN] << 4)) & 0x07FF;
		rc.ch6 = (((int16_t)pData[(i+7)%USART2_MAX_RX_LEN] >> 7) | ((int16_t)pData[(i+8)%USART2_MAX_RX_LEN] << 1) |((int16_t)pData[(i+9)%USART2_MAX_RX_LEN] << 9))	& 0x07FF;
		rc.ch7 = (((int16_t)pData[(i+9)%USART2_MAX_RX_LEN] >> 2) | ((int16_t)pData[(i+10)%USART2_MAX_RX_LEN] << 6)) & 0x07FF;
		rc.ch8 = (((int16_t)pData[(i+10)%USART2_MAX_RX_LEN] >> 5) | ((int16_t)pData[(i+11)%USART2_MAX_RX_LEN]<<3)) &0x07FF;

		rc.ch9 = ((int16_t)pData[(i+12)%USART2_MAX_RX_LEN] | ((int16_t)pData[(i+13)%USART2_MAX_RX_LEN] << 8)) & 0x07FF;
		rc.ch10 = (((int16_t)pData[(i+13)%USART2_MAX_RX_LEN] >> 3) | ((int16_t)pData[(i+14)%USART2_MAX_RX_LEN] << 5))	& 0x07FF;
		rc.ch11 = (((int16_t)pData[(i+14)%USART2_MAX_RX_LEN] >> 6) | ((int16_t)pData[(i+15)%USART2_MAX_RX_LEN] << 2) |((int16_t)pData[(i+16)%USART2_MAX_RX_LEN] << 10)) & 0x07FF;
		rc.ch12 = (((int16_t)pData[(i+16)%USART2_MAX_RX_LEN] >> 1) | ((int16_t)pData[(i+17)%USART2_MAX_RX_LEN]<<7)) &0x07FF;
		rc.ch13 = ((int16_t)pData[(i+17)%USART2_MAX_RX_LEN] >> 4 | ((int16_t)pData[(i+18)%USART2_MAX_RX_LEN] << 4)) & 0x07FF;
		rc.ch14 = (((int16_t)pData[(i+18)%USART2_MAX_RX_LEN] >> 7) | ((int16_t)pData[(i+19)%USART2_MAX_RX_LEN] << 1) |((int16_t)pData[(i+20)%USART2_MAX_RX_LEN] << 9))	& 0x07FF;
		rc.ch15 = (((int16_t)pData[(i+20)%USART2_MAX_RX_LEN] >> 2) | ((int16_t)pData[(i+21)%USART2_MAX_RX_LEN] << 6)) & 0x07FF;
		rc.ch16 = (((int16_t)pData[(i+21)%USART2_MAX_RX_LEN] >> 5) | ((int16_t)pData[(i+22)%USART2_MAX_RX_LEN]<<3)) &0x07FF;

		if(rc.ch7>1500) 		rc.sw1 = 3; //��ͨ��5-6���������� 1-3
		else if(rc.ch7<500)  	rc.sw1 = 1;
		else            		rc.sw1 = 2;
		
		if(rc.ch5>1500) 		rc.sw2 = 2;  // ��DJIң��������һ��
		else if(rc.ch5<500)		rc.sw2 = 1;
		else 					rc.sw2 = 3;
		
		rc.update =0x01;
	}
	else //δ֪��ң��������
	{
		rc.update =0x01;
		rc.available =0x00;
		return 0;
	}	
	 
	
	if(rc.rc_state == DBUS_INIT) 
	{
		printf("RC Init ");
		rc.available =0x00;
		if(RC_Offset_Init())
		{
			rc.available =0x02; //��ʼ���ɹ�
			rc.rc_state = DBUS_RUN;
			printf("successful!\n");
		}
		else
		{
			rc.available =0x01; //����offset��
			rc.rc_state = DBUS_INIT; //��ʼ��ʧ��
			printf("failed..\n");
			faild_times++;
			if(faild_times>30)
			{
				printf("faild_times>30 reset MCU..\n");
				__set_FAULTMASK(1); // �ر������ж�
				NVIC_SystemReset();// ��λ
			}
		}
		return 0; //��ʾ���ڳ�ʼ���� ���ݻ���������
	}
	else
	{
		if((rc.ch1 > 2500) || (rc.ch1<100) 
			|| (rc.ch2 > 2500) || (rc.ch2<100)
			|| (rc.ch3 > 2500) || (rc.ch3<100)
			|| (rc.sw1 > 3) || (rc.sw1<1)
			|| (rc.sw2 > 3) || (rc.sw2<1))
		{
			printf("RC Remote ERROR\n");
			RC_Debug_Message();
			//rc.rc_state = DBUS_INIT;
			rc.available =0x03; // ��ʼ���Ժ�������쳣����
			return 0;
		}
		else
		{
			rc.available =0x0f;
			RC_Routing();
		}
		rc.cnt =rc.cnt +1;	
	}
	return 1;
}
/***************************************************************************
* @brief       ң������ʼ������ �ɼ�10��������ȡƽ��ֵ
* @param[out]  У׼��ɷ���1 ���򷵻�0
* @param[in]    
* @retval 
* @maker    crp
* @date 2019-9-8
****************************************************************************/
char RC_Offset_Init(void)
{
	if((rc.ch1 < 900) || (rc.ch1 > 1200))
	{
		printf("rc.ch1 < 900) || (rc.ch1 > 1200)!   data:%d\n",rc.ch1);
		return 0;
	}
		
	if((rc.ch2 < 900) || (rc.ch2 > 1200))
	{
		printf("rc.ch2 < 900) || (rc.ch2 > 1200)!\n");
		return 0;
	}
	if((rc.ch3 < 900) || (rc.ch3 > 1200))
	{
		printf("rc.ch3 < 900) || (rc.ch3 > 1200)!\n");
		return 0;
	}
	if((rc.ch4 < 900) || (rc.ch4 > 1200))
	{
		printf("rc.ch4 < 900) || (rc.ch4 > 1200)!\n");
		return 0;
	}
	
	if(rc_counter == 0)
	{
		ch1_offset_sum = 0;
		ch2_offset_sum = 0;
		ch3_offset_sum = 0;
		ch4_offset_sum = 0;
	}
	
	ch1_offset_sum+=rc.ch1;
	ch2_offset_sum+=rc.ch2;
	ch3_offset_sum+=rc.ch3;
	ch4_offset_sum+=rc.ch4;
	rc_counter++;
 
	// ��10�����ݵ�ƽ��ֵ
	if(rc_counter>=10)
	{
		rc.ch1_offset =(ch1_offset_sum/rc_counter);
		rc.ch2_offset =(ch2_offset_sum/rc_counter);
		rc.ch3_offset =(ch3_offset_sum/rc_counter);
		rc.ch4_offset =(ch4_offset_sum/rc_counter);

		//calibration failed ͨ����������λֵ������0
		if((rc.ch1_offset ==0) || (rc.ch2_offset ==0) || (rc.ch3_offset ==0) || (rc.ch4_offset ==0) || 
			abs(rc.ch1_offset)>2500 || abs(rc.ch2_offset)>2500 || abs(rc.ch3_offset)>2500 || abs(rc.ch4_offset)>2500)
		{
			RC_Debug_Message();
			rc.available =0x00; 
			rc_counter=0;
			ch1_offset_sum = 0;
			ch2_offset_sum = 0;
			ch3_offset_sum = 0;
			ch4_offset_sum = 0;
			printf("rc.ch1_offset ==0  reset MCU!\n");
			__set_FAULTMASK(1); // �ر������ж�
			NVIC_SystemReset();// ��λ
			return 0;
		}
		else
		{
			rc.available =0x01; 
			rc.cnt =rc_counter;
			return 1;
		}
	}
	else
	{
		rc.available =0x00;
		rc.cnt =rc_counter;
		return 0;
	}
	//return 1;
}
/***************************************************************************
* @brief       �ɹ����յ�һ֡DBUS�����Ժ���øú����趨PID���Ƶ�Ŀ��ֵ
* @retval 
* @maker    crp
* @date 2019-9-8
****************************************************************************/
void RC_Routing(void)
{
	float RC_K = 0;
	int err_ch1 = 0;
	int err_ch2 = 0;
	int err_ch3 = 0;
	if((rc.sw1 !=1) && (rc.available) && Read_Isolated_Input(4)==0)//ʹ��ң����ģʽ
	{
		if(rc.sw2 ==1) //1 ��ģʽ ���1m/s
		{
			RC_K=1;
		}
		else if(rc.sw2 ==3) //2 ��ģʽ ���2m/s
		{
			RC_K = 2;
		}
		else if(rc.sw2 ==2) //3 ��ģʽ ���3.5m/s
		{
			RC_K = 4;
		}
		else
			RC_K = 0.0;
		
		// ��������
		err_ch1 = rc.ch1-rc.ch1_offset;
		err_ch2 = rc.ch2-rc.ch2_offset;
		err_ch3 = rc.ch3-rc.ch3_offset;
		
		if(abs(err_ch1) < 5)
				err_ch1 =0;
		if(abs(err_ch2) < 10)
				err_ch2 = 0;
		if(abs(err_ch3) < 50)
				err_ch3 = 0;
		
		if((Code_Switch_Value & 0x03) == 0x00) // ����ģ��
		{  // printf("Diff_Wheel_Speed_Model\n");
			//DiffX4_Wheel_Speed_Model_APSL(0,-0.5);
			DiffX4_Wheel_Speed_Model(err_ch2/660.0*RC_K,-err_ch1/660.0*RC_K); 
		}
		else if((Code_Switch_Value & 0x03) == 0x01) // �����ķ��ģ��
		{
			;//Mecanum_Wheel_Speed_Model(err_ch2*RC_K,-err_ch1*RC_K,-err_ch3*RC_K);
		}
		else if((Code_Switch_Value & 0x03) == 0x02)// 4WS4WDģ��
		{
			if(err_ch3!=0) //ԭ��ת��ģʽ
			{
				//printf("DiffX4_Wheel_Speed_Model\n");
				WSWD_Wheel_Speed_Model(0,0,-err_ch3/660.0);
			}
			else 
			{
				if(rc.sw1 ==3) //������ת��ģʽ
				{
					//AKM_Wheel_Speed_Model(err_ch2/660.0*RC_K,-err_ch1/660.0*RC_K); 
					AKM2_Wheel_Speed_Model(err_ch2/660.0*RC_K,0,-err_ch1/660.0*RC_K); 
				}				
				else if(rc.sw1 ==2) //ȫ��ģʽ
					WSWD_Wheel_Speed_Model(err_ch2/660.0*RC_K,-err_ch1/660.0*RC_K,0);
				else ;
			}
		}
		else if((Code_Switch_Value & 0x03) == 0x03)// ������ģ��
		{
			; 
		}
		else ;
		
	}
}



/***************************************************************************
* @brief       ����ң��������ӡ����
* @retval 
* @maker    crp
* @date 2019-9-8
****************************************************************************/
void RC_Debug_Message(void)
{
//	unsigned char i=0;
	if(rc.type == 1) // ��DBUSЭ��
	{
		UART_send_string(USART6,"DBUS:  ch1:");UART_send_data(USART6,rc.ch1);UART_send_char(USART6,'\t');		
		UART_send_string(USART6,"ch2:");UART_send_data(USART6,rc.ch2);UART_send_char(USART6,'\t');	
		UART_send_string(USART6,"ch3:");UART_send_data(USART6,rc.ch3);UART_send_char(USART6,'\t');	
		UART_send_string(USART6,"ch4:");UART_send_data(USART6,rc.ch4);UART_send_char(USART6,'\t');	
		UART_send_string(USART6,"sw1:");UART_send_data(USART6,rc.sw1);UART_send_char(USART6,'\t');	
		UART_send_string(USART6,"sw2:");UART_send_data(USART6,rc.sw2);UART_send_char(USART6,'\n');	
	}
	else if(rc.type == 2) // ��ģSBUSЭ��
	{
		UART_send_string(USART6,"SBUS:  ch1:");UART_send_data(USART6,rc.ch1);UART_send_char(USART6,'\t');		
		UART_send_data(USART6,rc.ch2);UART_send_char(USART6,'\t');	
		UART_send_data(USART6,rc.ch3);UART_send_char(USART6,'\t');	
		UART_send_data(USART6,rc.ch4);UART_send_char(USART6,'\t');	
		UART_send_data(USART6,rc.ch5);UART_send_char(USART6,'\t');	
		UART_send_data(USART6,rc.ch6);UART_send_char(USART6,'\t');	
		UART_send_data(USART6,rc.ch7);UART_send_char(USART6,'\t');	
		UART_send_data(USART6,rc.ch8);UART_send_char(USART6,'\t');	
		UART_send_string(USART6,"sw1:");UART_send_data(USART6,rc.sw1);UART_send_char(USART6,'\t');	
		UART_send_string(USART6,"sw2:");UART_send_data(USART6,rc.sw2);UART_send_char(USART6,'\n');	
//		UART_send_data(USART6,rc.ch9);UART_send_char(USART6,'\t');	
//		UART_send_data(USART6,rc.ch10);UART_send_char(USART6,'\t');	
//		UART_send_data(USART6,rc.ch11);UART_send_char(USART6,'\t');	
//		UART_send_data(USART6,rc.ch12);UART_send_char(USART6,'\t');	
//		UART_send_data(USART6,rc.ch13);UART_send_char(USART6,'\t');	
//		UART_send_data(USART6,rc.ch14);UART_send_char(USART6,'\t');	
//		UART_send_data(USART6,rc.ch15);UART_send_char(USART6,'\t');	
//		UART_send_data(USART6,rc.ch16);
		UART_send_char(USART6,'\n');	
		
	}
//	
//	UART_send_string(USART6,"ch1_offset_sum:");UART_send_data(USART6,ch1_offset_sum);UART_send_char(USART6,'\t');		
//	UART_send_string(USART6,"ch2_offset_sum:");UART_send_data(USART6,ch2_offset_sum);UART_send_char(USART6,'\t');	
//	UART_send_string(USART6,"ch3_offset_sum:");UART_send_data(USART6,ch3_offset_sum);UART_send_char(USART6,'\t');	
//	UART_send_string(USART6,"ch4_offset_sum:");UART_send_data(USART6,ch4_offset_sum);UART_send_char(USART6,'\n');
//	
//	UART_send_string(USART6,"cnt:");UART_send_data(USART6,rc.cnt);UART_send_char(USART6,'\t');
//	UART_send_string(USART6,"rc_counter:");UART_send_data(USART6,rc_counter);UART_send_char(USART6,'\t');
	UART_send_string(USART6,"ch1_offset:");UART_send_data(USART6,rc.ch1_offset);UART_send_char(USART6,'\t');		
	UART_send_string(USART6,"ch2_offset:");UART_send_data(USART6,rc.ch2_offset);UART_send_char(USART6,'\t');	
	UART_send_string(USART6,"ch3_offset:");UART_send_data(USART6,rc.ch3_offset);UART_send_char(USART6,'\t');	
	UART_send_string(USART6,"ch4_offset:");UART_send_data(USART6,rc.ch4_offset);UART_send_char(USART6,'\n');	
}

/***************************************************************************
* @brief       DBUS�ϴ���Ϣ��PC��
* @retval 
* @maker    crp
* @date 2019-9-8
****************************************************************************/
void RC_Upload_Message(void)
{
	unsigned char senddata[50];
	unsigned char i=0,j=0;	
	unsigned char cmd=0xA3;	
	unsigned int sum=0x00;	
	senddata[i++]=0xAE;
	senddata[i++]=0xEA;
	senddata[i++]=0x00;
	senddata[i++]=cmd;

	senddata[i++]=rc.ch1>>8;
	senddata[i++]=rc.ch1;
	senddata[i++]=rc.ch2>>8;
	senddata[i++]=rc.ch2;
	senddata[i++]=rc.ch3>>8;
	senddata[i++]=rc.ch3;
	senddata[i++]=rc.ch4>>8;
	senddata[i++]=rc.ch4;
	senddata[i++]=rc.sw1;
	senddata[i++]=rc.sw2;
	senddata[i++]=0x00;
	senddata[i++]=0x00;
	senddata[i++]=rc.type;
	senddata[i++]=rc.rc_state;
	senddata[i++]=0x00; //������
	for(j=2;j<i;j++)
		sum+=senddata[j];
	senddata[i++]=sum;
	senddata[2]=i-2; //���ݳ���
	senddata[i++]=0xEF;
	senddata[i++]=0xFE;
	senddata[i++]='\0';
	UART_send_buffer(USART1,senddata,i);
}
