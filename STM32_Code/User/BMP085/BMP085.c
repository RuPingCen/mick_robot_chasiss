/* BMP085.c file
��д�ߣ�lisn3188
��ַ��www.chiplab7.com
����E-mail��lisn3188@163.com
���뻷����MDK-Lite  Version: 4.23
����ʱ��: 2012-04-25
���ԣ� ���������ڵ���ʵ���ҵ�mini IMU����ɲ���
���ܣ�
�ṩBMP085 ��ʼ�� ���� ��ȡ�¶� ��ѹ API
------------------------------------
 */
#include "BMP085.h"
#include <math.h>
#include "UARTs.h"

// ��ѹ��״̬��
#define SCTemperature  0x01
#define CTemperatureing  0x02
#define SCPressure  0x03
#define SCPressureing  0x04

volatile int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;     // �궨������  
volatile uint16_t ac4,ac5,ac6;                   // �궨������
volatile int32_t b5;                    //�¶�
  
uint8_t _buff[BUFFER_SIZE];    // ���ݻ�����
int16_t _oss;                 // ����������
  
int32_t _cm_Offset, _Pa_Offset;
int32_t _param_datum, _param_centimeters;
volatile unsigned char BPM085_ST;
int32_t last_Temperature,last_Pressure,last_Alt;
int32_t  BMP085_FIFO[2][11]; //�Ƚ��ȳ�����������
int32_t  BMP085_FIFOH[21];	 //�Ƚ��ȳ�����������

unsigned char BMP085_IS_Finish(void);
void BMP085_writemem(uint8_t _addr, uint8_t _val);
void BMP085_calcTrueTemperature(u8 rw);
void BMP085_getTemperature(int32_t *_Temperature,u8 rw);
void BMP085_calcTruePressure(int32_t *_TruePressure,u8 writeread);
void BMP085_getAltitude(int32_t *_centimeters,u8 rw);

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_newTemperature(int32_t T)
*��������:		���һ���µ�ֵ���¶ȹ�����
*******************************************************************************/
void BMP085_newTemperature(int32_t T)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){
	BMP085_FIFO[0][i-1]=BMP085_FIFO[0][i];
}
BMP085_FIFO[0][9]=T; //����ֵ���õ������ĩβ
sum=0;
for(i=0;i<10;i++){
   sum+=BMP085_FIFO[0][i]; //�ۼ�
}
last_Temperature=BMP085_FIFO[0][10]=sum/10;	//ȡƽ��ֵ
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_newPressure(int32_t P)
*��������:		���һ���µ�ֵ����ѹ������
*******************************************************************************/
void BMP085_newPressure(int32_t P)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<10;i++){
BMP085_FIFO[1][i-1]=BMP085_FIFO[1][i];
}
BMP085_FIFO[1][9]=P;
sum=0;
for(i=0;i<10;i++){
   sum+=BMP085_FIFO[1][i];
}
last_Pressure=BMP085_FIFO[1][10]=sum/10;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_newALT(int32_t A)
*��������:		���һ���µ�ֵ���߶ȹ�����
*******************************************************************************/
void BMP085_newALT(int32_t A)
{
unsigned char i ;
int32_t sum=0;
for(i=1;i<20;i++){
BMP085_FIFOH[i-1]=BMP085_FIFOH[i];
}
BMP085_FIFOH[19]=A;
sum=0;
for(i=0;i<20;i++){
   sum+=BMP085_FIFOH[i];
}
last_Alt=BMP085_FIFOH[20]=sum/20;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_getAlt(int32_t *_centimeters)
*��������:		��ȡ���µĸ߶�ֵ����λΪ cm 
*******************************************************************************/
void BMP085_getAlt(int32_t *_centimeters)
{
	*_centimeters = last_Alt;	
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_getPress(int32_t *_TruePressure)
*��������:		��ȡ���µ���ѹֵ����λΪ pa 
*******************************************************************************/
void BMP085_getPress(int32_t *_TruePressure)
{
	*_TruePressure = last_Pressure;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_getTemperat(int32_t *_Temperature)
*��������:		��ȡ���µ��¶�ֵ����λΪ 0.1C 
*******************************************************************************/
void BMP085_getTemperat(int32_t *_Temperature)
{
	*_Temperature =	last_Temperature;
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_Routing(void)
*��������:		BMP085 ����ʱ���õĳ���
				�ó���᲻ͣ�ض�ȡ��ѹֵ���¶�ֵ��
				�û���Ҫ���ڵ�����������Ը��¸߶Ⱥ��¶���Ϣ
*******************************************************************************/
void BMP085_Routing(void)
{
  switch(BPM085_ST){
  case SCTemperature: 
  				BMP085_writemem(CONTROL, READ_TEMPERATURE); 
				BPM085_ST=CTemperatureing;
				break;
  case CTemperatureing: 
  			 	if(BMP085_IS_Finish()){
				BMP085_calcTrueTemperature(0);
				BMP085_getTemperature(&last_Temperature,0);
				BMP085_newTemperature(last_Temperature);
				BPM085_ST=SCPressure;
				}
  				break;
  case SCPressure:  
  				BMP085_writemem(CONTROL, READ_PRESSURE+(_oss << 6));
				BPM085_ST=SCPressureing;
  				break;
  case SCPressureing:  
  				if(BMP085_IS_Finish()){
				BMP085_getAltitude(&last_Alt,0);
				BMP085_newALT(last_Alt);
				BPM085_ST=SCTemperature;
				}
  				break;
  default :BPM085_ST=SCTemperature; break;
  }

}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		unsigned char BMP085_IS_Finish(void)
*��������:		��� BMP085  �Ƿ������ת��
*******************************************************************************/
unsigned char BMP085_IS_Finish(void)
{
 	if(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4)==Bit_SET){
	  return 1;	 // finish 
	 }
	 else return 0;	  //running
}

void BMP085_writemem(uint8_t _addr, uint8_t _val) {
  IICwriteByte(BMP085_ADDR,_addr,_val);
}

void BMP085_readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) {
  IICreadBytes(BMP085_ADDR,_addr,_nbytes,__buff);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_getCalData(void)
*��������:		���� BMP085 �ڲ��ı궨��Ϣ
*******************************************************************************/
void BMP085_getCalData(void) {
  BMP085_readmem(CAL_AC1, 2, _buff);
  ac1 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  BMP085_readmem(CAL_AC2, 2, _buff);
  ac2 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  BMP085_readmem(CAL_AC3, 2, _buff);
  ac3 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  BMP085_readmem(CAL_AC4, 2, _buff);
  ac4 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1]));
  BMP085_readmem(CAL_AC5, 2, _buff);
  ac5 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1]));
  BMP085_readmem(CAL_AC6, 2, _buff);
  ac6 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1])); 
  BMP085_readmem(CAL_B1, 2, _buff);
  b1 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
  BMP085_readmem(CAL_B2, 2, _buff);
  b2 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
  BMP085_readmem(CAL_MB, 2, _buff);
  mb = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  BMP085_readmem(CAL_MC, 2, _buff);
  mc = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
  BMP085_readmem(CAL_MD, 2, _buff);
  md = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
}

void BMP085_calcTrueTemperature(u8 rw){
  int32_t ut,x1,x2,mctemp,mdtemp;
  if(rw){
  BMP085_writemem(CONTROL, READ_TEMPERATURE);
  delay_ms(10);                          // min. 4.5ms read Temp delay
  }
  BMP085_readmem(CONTROL_OUTPUT, 2, _buff); 
  ut = ((int32_t)_buff[0] << 8 | ((int32_t)_buff[1]));    // uncompensated temperature value
 
  // calculate temperature
  x1 = (((int32_t)ut - (int32_t)ac6) * (int32_t)ac5) >> 15;
  mctemp= mc;
  mdtemp= md;
  x2 = (mctemp <<11) / (x1 + mdtemp);
  b5 = x1 + x2;
}

void BMP085_setMode(u8 _BMPMode){
  _oss = _BMPMode;
}

void BMP085_calcTruePressure(int32_t *_TruePressure,u8 writeread) {
  volatile int32_t up,x1,x2,x3,b3,b6,p;
  volatile int32_t b4,b7;
  volatile int32_t tmp; 

 //read Raw Pressure
 if(writeread){

  #if AUTO_UPDATE_TEMPERATURE
  BMP085_calcTrueTemperature(writeread);        // b5 update 
  #endif 
  BMP085_writemem(CONTROL, READ_PRESSURE+(_oss << 6));
  delay_ms(30); 
  }
     
  BMP085_readmem(CONTROL_OUTPUT, 3, _buff);  
  up = ((((int32_t)_buff[0] <<16) | ((int32_t)_buff[1] <<8) | ((int32_t)_buff[2])) >> (8-_oss)); // uncompensated pressure value
  
  // calculate true pressure
  b6 = b5 - 4000;             // b5 is updated by calcTrueTemperature().
  x1 = (b2* ((b6 * b6) >> 12)) >> 11;
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  tmp = ac1;
  tmp = (tmp * 4 + x3) << _oss;
  b3 = (tmp + 2) >> 2;
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
  b7 = ((uint32_t)up - b3) * (50000 >> _oss);
  if(b7 < 0x80000000){
   p = ((uint32_t)b7 << 1) / b4;
  }	else{
  p = (b7 / b4) << 1;
  }
  //p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  *_TruePressure = p + ((x1 + x2 + 3791) >> 4);

}

void BMP085_getPressure(int32_t *_Pa,u8 wr){   
  int32_t TruePressure;
  BMP085_calcTruePressure(&TruePressure,wr); 
  //TruePressure;
  *_Pa = TruePressure / pow((1 - (float)_param_centimeters / 4433000), 5.255) + _Pa_Offset;
  // converting from float to int32_t truncates toward zero, 1010.999985 becomes 1010 resulting in 1 Pa error (max).  
  // Note that BMP085 abs accuracy from 700...1100hPa and 0..+65�C is +-100Pa (typ.)
}

void BMP085_setLocalAbsAlt(int32_t _centimeters){  
  int32_t tmp_Pa;
 
  _param_centimeters = _centimeters;   
  BMP085_getPressure(&tmp_Pa,1);    // calc pressure based on current altitude
  _param_datum = tmp_Pa;
}

void BMP085_ResetAlt(int32_t _centimeters){  
  int32_t tmp_Pa;
 
  _param_centimeters = _centimeters;   
  BMP085_getPress(&tmp_Pa);    // calc pressure based on current altitude
  _param_datum = tmp_Pa;
}

void BMP085_getAltitude(int32_t *_centimeters,u8 rw){
  int32_t TruePressure;
  BMP085_calcTruePressure(&TruePressure,rw); 
  BMP085_newPressure(TruePressure);
   TruePressure = last_Pressure;
  *_centimeters =  4433000 * (1 - pow((TruePressure / (float)_param_datum), 0.1903)) + _cm_Offset;  
  // converting from float to int32_t truncates toward zero, 100.999985 becomes 100 resulting in 1 cm error (max).
}

void BMP085_setLocalPressure(int32_t _Pa){   
  int32_t tmp_alt;
 
  _param_datum = _Pa;   
  BMP085_getAltitude(&tmp_alt,1);    // calc altitude based on current pressure   
  _param_centimeters = tmp_alt;
}

void BMP_init(u8 _BMPMode, int32_t _initVal, u8 _Unitmeters){     
  BMP085_getCalData();               // initialize cal data
  BMP085_calcTrueTemperature(1);      // initialize b5
  BMP085_setMode(_BMPMode);
  _Unitmeters>0 ? BMP085_setLocalAbsAlt(_initVal) : BMP085_setLocalPressure(_initVal); 
}
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void BMP085_init(void)
*��������:		���ⲿ���õĳ�ʼ������
*******************************************************************************/
void BMP085_init(void) {  
  _cm_Offset = 0;
  _Pa_Offset = 0;               // 1hPa = 100Pa = 1mbar	
  BMP_init(MODE_ULTRA_HIGHRES, 0, 1);
}

void BMP085_getTemperature(int32_t *_Temperature,u8 rw) {
  BMP085_calcTrueTemperature(rw);                            // force b5 update
  *_Temperature = ((b5 + (int32_t)8) >> 4);
}

//------------------End of File----------------------------
