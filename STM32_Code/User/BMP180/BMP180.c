#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_Delay.h"
#include <math.h>
 
 
#include "IO_IIC.h"
#include "BMP180.h"




#define    BMP180_FIFO_length         10

 
//------------------------------------------------------------//
volatile unsigned char BPM085_ST;//BMP180��ѹ��״̬��
volatile int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;     // �궨������  
volatile uint16_t ac4,ac5,ac6;                   // �궨������
volatile int32_t b5;                         //�¶� �궨������
int16_t _oss;                 // ���������ã�ģʽ���ã�
uint8_t _buff[BUFFER_SIZE];    // ����IIC���ݶ�ȡ������
int32_t _param_datum=1, _param_centimeters=0;
//------------------------------------------------------------//

 
int32_t _Temper_Offset=-35,_cm_Offset=-4500, _Pa_Offset=-676;//�¶� �߶� ��ѹƫ����
int32_t _Temperature=0,_centimeters=0,_Pa=0;//�¶� �߶� ��ѹ��ȡ��ԭʼ�����ݴ���� 
 



#define    BMP180_FIFO_length         10     //FIFO����
  
int32_t _Temperature_FIFO[BMP180_FIFO_length+1]={'0'};//�¶� ������
int32_t _centimeters_FIFO[BMP180_FIFO_length+1]={'0'};//�߶�
int32_t _Pa_FIFO[BMP180_FIFO_length+1]={'0'};//��ѹ  //�������һ�����ݴ��ƽ��ֵ

//------------------------------------------------------------//

 
/****************************************************************
  *
  * ��������   BMP180_writemem
	* �������ܣ� ��ѹ��IIC�������ݷ��ͺ���
  *
  * ����˵����  _addr:  �Ĵ�����ַ
  *    					_val�� ����
  *   
  * ��ע�� CRP
  * �ο�����ʵ���Ҵ���
****************************************************************/
void BMP180_writemem(uint8_t _addr, uint8_t _val) 
{
	 IO_IIC_write_Command(BMP180_ADDR,_addr,_val); //address+register+command
  
}

/****************************************************************
  *
  * ��������   BMP180_readmem
	* �������ܣ� ��ѹ��ͨ��IIC���߶��Ĵ�������
  *
  * ����˵����  _addr:  �Ĵ�����ַ
  *            _nbytes: ��ȡ�ֽ���
  *    					__buff�� ���ݴ�ŵ�ַ
  *   
  * ��ע�� CRP
  * �ο�����ʵ���Ҵ���
****************************************************************/
void BMP180_readmem(uint8_t _addr, uint8_t _nbytes, uint8_t __buff[]) 
{
	uint16_t num_i=0;
	uint8_t  num__addr=_addr;
	for(num_i=0;num_i<_nbytes;num_i++)
	{

     __buff[num_i]=IO_IIC_read_Data(BMP180_ADDR,num__addr);//address(with bit 0 set) + register
		 num__addr++;
	//	Delay_100us(1);
  }
	
   
}


/****************************************************************
  *
  * ��������   Nem_Temperature_Data_FIFO
	* �������ܣ� �¶�����FIFO��
  *
  * ����˵����  temp_da:  �»�ȡ�Ĳ���
  *   
  * ��ע�� CRP     �������һ�����ݴ��ƽ��ֵ
  * �ο�����ʵ���Ҵ���
****************************************************************/
void Nem_Temperature_Data_FIFO(int32_t temp_da)
{
   unsigned char i=0;
	 int64_t  sum=0;
	 for(i=0;i<(BMP180_FIFO_length-1);i++)
	{

       _Temperature_FIFO[i]=_Temperature_FIFO[i+1];

  }
	 _Temperature_FIFO[BMP180_FIFO_length-1]= temp_da;
	
	
	 for(i=0;i<BMP180_FIFO_length;i++)
	{

       sum+=_Temperature_FIFO[i];

  }
	 _Temperature_FIFO[BMP180_FIFO_length]=(sum+BMP180_FIFO_length/2)/BMP180_FIFO_length;
 
}
 /****************************************************************
  *
  * ��������   Nem_centimeters_Data_FIFO
	* �������ܣ� ��������FIFO��
  *
  * ����˵����  temp_da:  �»�ȡ�Ĳ���
  *   
  * ��ע�� CRP     �������һ�����ݴ��ƽ��ֵ
  * �ο�����ʵ���Ҵ���
****************************************************************/
void Nem_centimeters_Data_FIFO(int32_t temp_da)
{
   unsigned char i=0;
	 int64_t  sum=0;
	 for(i=0;i<(BMP180_FIFO_length-1);i++)
	{

       _centimeters_FIFO[i]=_centimeters_FIFO[i+1];

  }
	 _centimeters_FIFO[BMP180_FIFO_length-1]= temp_da;
	
	
	 for(i=0;i<BMP180_FIFO_length;i++)
	{

       sum+=_centimeters_FIFO[i];

  }
	 _centimeters_FIFO[BMP180_FIFO_length]=(sum+BMP180_FIFO_length/2)/BMP180_FIFO_length;
 
}
/****************************************************************
  *
  * ��������   Nem_Pa_Data_FIFO
	* �������ܣ� ��ѹ����FIFO��
  *
  * ����˵����  temp_da:  �»�ȡ�Ĳ���
  *   
  * ��ע�� CRP     �������һ�����ݴ��ƽ��ֵ
  * �ο�����ʵ���Ҵ���
****************************************************************/
void Nem_Pa_Data_FIFO(int32_t temp_da)
{
   unsigned char i=0;
	 int64_t  sum=0;
	 for(i=0;i<(BMP180_FIFO_length-1);i++)
	{

       _Pa_FIFO[i]=_Pa_FIFO[i+1];

  }
	 _Pa_FIFO[BMP180_FIFO_length-1]= temp_da;
	
	
	 for(i=0;i<BMP180_FIFO_length;i++)
	{

       sum+=_Pa_FIFO[i];

  }
	 _Pa_FIFO[BMP180_FIFO_length]=(sum+BMP180_FIFO_length/2)/BMP180_FIFO_length;
 
}


/****************************************************************
  *
  * ��������   BMP180_calcTrueTemperature
	* �������ܣ� BMP180�¶����ݼ��㺯�� 
  *
  * ����˵����  rw:  ѡ���Ƿ�˴�����ת��  һ��Ϊ0
  *            
  * ��ע�� CRP          �¶ȼ�����Ҫ���ڸ��²���B5
  * �ο�����ʵ���Ҵ���
****************************************************************/
void BMP180_calcTrueTemperature(u8 rw)
{
		int32_t ut,x1,x2,mctemp,mdtemp;
		if(rw)
		{
				BMP180_writemem(CONTROL, READ_TEMPERATURE);
				Delay_10us(3000);  // min. 4.5ms read Temp delay
		}
		
		BMP180_readmem(CONTROL_OUTPUT, 2, _buff); 
		ut = ((int32_t)_buff[0] << 8 | ((int32_t)_buff[1]));    // uncompensated temperature value
		x1 = (((int32_t)ut - (int32_t)ac6) * (int32_t)ac5) >> 15;  // calculate temperature
		mctemp= mc;
		mdtemp= md;
		x2 = (mctemp <<11) / (x1 + mdtemp);
		b5 = x1 + x2;
		_Temperature = ((b5 + (int32_t)8) >> 4)+_Temper_Offset;

     Nem_Temperature_Data_FIFO(_Temperature);
}

/****************************************************************
  *
  * ��������   BMP180_calcTruePressure
	* �������ܣ� BMP180��ѹ���������ݶ�ȡת�� 
  *
  * ����˵����  rw:  ѡ���Ƿ�˴�����ת��  һ��Ϊ0
  *            
  * ��ע�� CRP          
  * �ο�����ʵ���Ҵ���
****************************************************************/
void BMP180_calcTruePressure(u8 writeread) 
{
	
			volatile int32_t up,x1,x2,x3,b3,b6,p;
			volatile int32_t b4,b7;
			volatile int32_t tmp; 
      int32_t TruePressure;
		 
		 if(writeread)//read Raw Pressure
			 {
					BMP180_writemem(CONTROL, READ_PRESSURE+(_oss << 6));
					Delay_10us(3000); 
			}
				 
			BMP180_readmem(CONTROL_OUTPUT, 3, _buff);  
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
			if(b7 < 0x80000000)
			{
					p = ((uint32_t)b7 << 1) / b4;
			}	
			else
			{
					p = (b7 / b4) << 1;
			}
			//p = b7 < 0x80000000 ? (b7 << 1) / b4 : (b7 / b4) << 1;
			x1 = (p >> 8) * (p >> 8);
			x1 = (x1 * 3038) >> 16;
			x2 = (-7357 * p) >> 16;
			TruePressure = p + ((x1 + x2 + 3791) >> 4);
			


		  	//_Pa = TruePressure + _Pa_Offset;// �ο��������� 	
      _Pa = TruePressure/pow((1 - (float)_param_centimeters / 4433000), 5.255) + _Pa_Offset;//����Ϊ����ʵ����Դ���� 
  
			_centimeters =  4433000 * (1 - pow((TruePressure / 101325.0f), 0.1903)) + _cm_Offset;  //����߶�  
			
			// �ڵ���ʵ���Ҵ�����101325.0f ��Ϊ��һ������  _param_datum  ���Ǽ���������ݺ��񲻶� ����ֱ�Ӹ�Ϊ�� 101325.0f ---�ο���������

			
			Nem_Pa_Data_FIFO(_Pa);//����FIFO����
			Nem_centimeters_Data_FIFO(_centimeters);
}



/****************************************************************
  *
  * ��������   BMP180_getCalData
  *
	* �������ܣ� ���� BMP180 �ڲ��ı궨��Ϣ
  *       
  * ��ע�� CRP          
  * �ο�����ʵ���Ҵ���
****************************************************************/
void BMP180_getCalData(void) 
{
  BMP180_readmem(CAL_AC1, 2, _buff);
  ac1 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
	
  BMP180_readmem(CAL_AC2, 2, _buff);
  ac2 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
	
  BMP180_readmem(CAL_AC3, 2, _buff);
  ac3 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
	
  BMP180_readmem(CAL_AC4, 2, _buff);
  ac4 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1]));
	
  BMP180_readmem(CAL_AC5, 2, _buff);
  ac5 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1]));
	
  BMP180_readmem(CAL_AC6, 2, _buff);
  ac6 = ((uint16_t)_buff[0] <<8 | ((uint16_t)_buff[1])); 
	
  BMP180_readmem(CAL_B1, 2, _buff);
  b1 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
	
  BMP180_readmem(CAL_B2, 2, _buff);
  b2 = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1])); 
	
  BMP180_readmem(CAL_MB, 2, _buff);
  mb = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
	
  BMP180_readmem(CAL_MC, 2, _buff);
  mc = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
	
  BMP180_readmem(CAL_MD, 2, _buff);
  md = ((int16_t)_buff[0] <<8 | ((int16_t)_buff[1]));
	
}
 
/****************************************************************
  *
  * ��������   BMP180_init
  *
	* �������ܣ� BMP180 ��ʼ������
  *       
  * ��ע�� CRP          
  * �ο�����ʵ���Ҵ���
****************************************************************/ 
void BMP180_init(void) 
{    
	// 1hPa = 100Pa = 1mbar	
	_oss = MODE_HIGHRES;//���ù�����  MODE_ULTRA_HIGHRES  �߾��Ȳ���ģʽ

	BMP180_getCalData();               // initialize cal data
	BMP180_calcTrueTemperature(1);      // initialize b5
//  _param_centimeters=0;
//	BMP180_calcTruePressure(1); //TruePressure; 
 // _param_datum=	_Pa;

}

/****************************************************************
  *
  * ��������   BMP180_Routing
  *
	* �������ܣ� BMP180 ����ʱ���õĳ��� �ó���᲻ͣ�ض�ȡ��ѹֵ���¶�ֵ��
  *                                     �û���Ҫ���ڵ�����������Ը��¸߶Ⱥ��¶���Ϣ
  *               ����Ǽ��5ms~20ms����                    
  * ��ע�� CRP          
  * �ο�����ʵ���Ҵ���
****************************************************************/ 
void BMP180_Routing(void)
{
			switch(BPM085_ST)
			{
					case SCTemperature: //��ʼ�¶�ת��
											BMP180_writemem(CONTROL, READ_TEMPERATURE); 
											BPM085_ST = CTemperatureing;//״̬�л� 
								break;
					
					case CTemperatureing: //���ڽ����¶�ת��
									 
											BMP180_calcTrueTemperature(0);
											BPM085_ST = SCPressure;
											UART_send_string(USART1,"\n�¶ȣ�");
											UART_send_data(USART1,_Temperature_FIFO[BMP180_FIFO_length]);  //�¶�
									break;
					
					case SCPressure:  //��ʼ��ѹ��ת��
											BMP180_writemem(CONTROL, READ_PRESSURE+(_oss << 6)); 
											BPM085_ST = SCPressureing; 
									break;
					
					case SCPressureing: //���ڽ�����ѹת�� 
											BMP180_calcTruePressure(0);
											UART_send_string(USART1,"\t��ѹ��");
											UART_send_data(USART1,_Pa_FIFO[BMP180_FIFO_length]); //��ѹ 1hPa = 100Pa = 1mbar	
											UART_send_string(USART1," Pa\t�߶ȣ�");
											UART_send_data(USART1,_centimeters_FIFO[BMP180_FIFO_length]); //�߶�
					            UART_send_string(USART1," cm");
											BPM085_ST = SCTemperature;		 
									break;
					
					default :
											BPM085_ST=SCTemperature; 
									break;
		 
			}

}
 

//------------------End of File----------------------------
