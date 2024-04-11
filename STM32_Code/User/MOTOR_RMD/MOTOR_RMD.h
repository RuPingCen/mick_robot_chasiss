#ifndef __MOTOR_RMD_H
#define	__MOTOR_RMD_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx.h"
#include "stm32f4xx_usart.h"

 

 
/** 
* @brief  remote control information
*/

void MOTOR_RMD_Init(void);
uint8_t MOTOR_RMD_Calibrate_Offset(void);


//void MOTOR_APSL2DB_Set_Model(uint16_t ID, int8_t param_value);
//void MOTOR_APSL2DB_Enable(uint16_t ID, int16_t param_value);
//void MOTOR_APSL2DB_EmStop(uint16_t ID, uint8_t param_value);
//void MOTOR_APSL2DB_Set_RPM(uint16_t ID, double speed_rpm);

//void MOTOR_APSL2DB_Enable_Disconnect(uint16_t ID, uint8_t param_value);
//void MOTOR_APSL2DB_Enable_Disconnect_Time( uint16_t ID,uint32_t param_T);
//void MOTOR_APSL2DB_Enable_PDO(uint16_t ID, uint8_t param_value);
//void MOTOR_APSL2DB_Set_PDO_Period( uint16_t ID,uint16_t param_T);
 

void MOTOR_RMD_Set_Angle_ALLMotor(int32_t Angle);
void MOTOR_RMD_Set_Angle(uint16_t ID,uint16_t max_speed,int32_t Angle);
void MOTOR_RMD_Set_Speed_ALLMotor(int32_t rpm);
void MOTOR_RMD_Set_Speed(uint16_t ID,int32_t rpm);
	
void MOTOR_RMD_Read_Cmd_ALLMotor(uint8_t cmd_id);
void MOTOR_RMD_Read_Cmd(uint16_t ID,uint8_t cmd_id);
void MOTOR_RMD_Delay(uint32_t delayvalue);

uint8_t MOTOR_RMD_State_Analysis(void);

#ifdef __cplusplus
}
#endif

#endif
