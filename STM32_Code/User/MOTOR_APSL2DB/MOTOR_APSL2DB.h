#ifndef __MOTOR_APSL2DB_H
#define	__MOTOR_APSL2DB_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

 



 

#define FILTER_BUF_LEN		5
/*���յ��ĵ���Ĳ����ṹ��*/
typedef struct{
		int32_t 	angle;		//abs angle range:[0,8191] ���ת�Ǿ���ֵ
		int32_t 	last_angle;	  //abs angle range:[0,8191]
		int32_t	 	speed_rpm;       //ת�� 0.001 rpm
	
		uint8_t 	driver_status;
		uint8_t 	error_code_LSB;
		uint8_t 	error_code_HSB;
	
		int16_t  	given_current;   // 
		int16_t  	real_current;    //ʵ�ʵ�ת�ص���
	
	    int16_t  	Voltage;        //ĸ�ߵ�ѹ ��λV
		int16_t  	Temp;           //�¶�  ��λ��

		uint16_t	offset_angle;   //�������ʱ�����ƫ�Ƕ�
		int32_t		round_cnt;      //���ת��Ȧ��
		int32_t		total_angle;    //���ת�����ܽǶ�
	
		uint8_t		buf_idx;
		uint16_t	angle_buf[FILTER_BUF_LEN];
		uint16_t	fited_angle;
		uint32_t	msg_cnt;
}moto_measure_t;


/** 
* @brief  remote control information
*/

void MOTOR_APSL2DB_Init(void);
uint8_t MOTOR_APSL2DB_Infor_Get(void);

void MOTOR_APSL2DB_Set_Model(uint16_t ID, int8_t param_value);
void MOTOR_APSL2DB_Enable(uint16_t ID, int16_t param_value);
void MOTOR_APSL2DB_EmStop(uint16_t ID, uint8_t param_value);
void MOTOR_APSL2DB_Set_RPM(uint16_t ID, double speed_rpm);

void MOTOR_APSL2DB_Enable_Disconnect(uint16_t ID, uint8_t param_value);
void MOTOR_APSL2DB_Enable_Disconnect_Time( uint16_t ID,uint32_t param_T);
void MOTOR_APSL2DB_Enable_PDO(uint16_t ID, uint8_t param_value);
void MOTOR_APSL2DB_Set_PDO_Period( uint16_t ID,uint16_t param_T);

void MOTOR_APSL2DB_Set_CurrentLimt(uint16_t ID, uint16_t curr_value);
void MOTOR_APSL2DB_Set_Trap_Accel(uint16_t ID, uint16_t curr_value);
void MOTOR_APSL2DB_Set_Trap_Decel(uint16_t ID, uint16_t curr_value);
void APS_L2DB_Read_Temp(uint16_t ID);

void MOTOR_APSL2DB_Read_Cmd(uint16_t ID,uint16_t index,uint8_t subindex);
void MOTOR_APSL2DB_Set_Parameter(uint16_t ID,uint16_t index,uint8_t subindex);

void MOTOR_Delay(uint32_t delayvalue);


//���ݽ���
void MOTOR_APSL2DB_Measurements_Analy(void);
void MOTOR_APSL2DB_Measurements_Debug(void);
	

#ifdef __cplusplus
}
#endif

#endif
