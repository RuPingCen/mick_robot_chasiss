#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_Delay.h"
#include <math.h>
 
 
#include "IO_IIC.h"
#include "BMP180.h"




#define    BMP180_FIFO_length         10

 
//------------------------------------------------------------//
volatile unsigned char BPM085_ST;//BMP180气压计状态机
volatile int16_t ac1,ac2,ac3,b1,b2,mb,mc,md;     // 标定的数据  
volatile uint16_t ac4,ac5,ac6;                   // 标定的数据
volatile int32_t b5;                         //温度 标定的数据
int16_t _oss;                 // 过采样设置（模式设置）
uint8_t _buff[BUFFER_SIZE];    // 用于IIC数据读取缓冲区
int32_t _param_datum=1, _param_centimeters=0;
//------------------------------------------------------------//

 
int32_t _Temper_Offset=-35,_cm_Offset=-4500, _Pa_Offset=-676;//温度 高度 气压偏移量
int32_t _Temperature=0,_centimeters=0,_Pa=0;//温度 高度 气压读取的原始数据暂存变量 
 



#define    BMP180_FIFO_length         10     //FIFO长度
  
int32_t _Temperature_FIFO[BMP180_FIFO_length+1]={'0'};//温度 缓冲区
int32_t _centimeters_FIFO[BMP180_FIFO_length+1]={'0'};//高度
int32_t _Pa_FIFO[BMP180_FIFO_length+1]={'0'};//气压  //数组最后一个数据存放平均值

//------------------------------------------------------------//

 
/****************************************************************
  *
  * 函数名：   BMP180_writemem
	* 函数功能： 气压计IIC总线数据发送函数
  *
  * 参数说明：  _addr:  寄存器地址
  *    					_val： 数据
  *   
  * 备注： CRP
  * 参考第七实验室代码
****************************************************************/
void BMP180_writemem(uint8_t _addr, uint8_t _val) 
{
	 IO_IIC_write_Command(BMP180_ADDR,_addr,_val); //address+register+command
  
}

/****************************************************************
  *
  * 函数名：   BMP180_readmem
	* 函数功能： 气压计通过IIC总线读寄存器函数
  *
  * 参数说明：  _addr:  寄存器地址
  *            _nbytes: 读取字节数
  *    					__buff： 数据存放地址
  *   
  * 备注： CRP
  * 参考第七实验室代码
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
  * 函数名：   Nem_Temperature_Data_FIFO
	* 函数功能： 温度数据FIFO区
  *
  * 参数说明：  temp_da:  新获取的参数
  *   
  * 备注： CRP     数组最后一个数据存放平均值
  * 参考第七实验室代码
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
  * 函数名：   Nem_centimeters_Data_FIFO
	* 函数功能： 海拔数据FIFO区
  *
  * 参数说明：  temp_da:  新获取的参数
  *   
  * 备注： CRP     数组最后一个数据存放平均值
  * 参考第七实验室代码
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
  * 函数名：   Nem_Pa_Data_FIFO
	* 函数功能： 气压数据FIFO区
  *
  * 参数说明：  temp_da:  新获取的参数
  *   
  * 备注： CRP     数组最后一个数据存放平均值
  * 参考第七实验室代码
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
  * 函数名：   BMP180_calcTrueTemperature
	* 函数功能： BMP180温度数据计算函数 
  *
  * 参数说明：  rw:  选择是否此次启动转化  一般为0
  *            
  * 备注： CRP          温度计算主要用于更新参数B5
  * 参考第七实验室代码
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
  * 函数名：   BMP180_calcTruePressure
	* 函数功能： BMP180气压、海拔数据读取转化 
  *
  * 参数说明：  rw:  选择是否此次启动转化  一般为0
  *            
  * 备注： CRP          
  * 参考第七实验室代码
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
			


		  	//_Pa = TruePressure + _Pa_Offset;// 参考网上资料 	
      _Pa = TruePressure/pow((1 - (float)_param_centimeters / 4433000), 5.255) + _Pa_Offset;//改行为第七实验室源代码 
  
			_centimeters =  4433000 * (1 - pow((TruePressure / 101325.0f), 0.1903)) + _cm_Offset;  //计算高度  
			
			// 在第七实验室代码中101325.0f 改为了一个参数  _param_datum  但是计算出的数据好像不对 这里直接改为了 101325.0f ---参考网上资料

			
			Nem_Pa_Data_FIFO(_Pa);//更新FIFO数据
			Nem_centimeters_Data_FIFO(_centimeters);
}



/****************************************************************
  *
  * 函数名：   BMP180_getCalData
  *
	* 函数功能： 读到 BMP180 内部的标定信息
  *       
  * 备注： CRP          
  * 参考第七实验室代码
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
  * 函数名：   BMP180_init
  *
	* 函数功能： BMP180 初始化函数
  *       
  * 备注： CRP          
  * 参考第七实验室代码
****************************************************************/ 
void BMP180_init(void) 
{    
	// 1hPa = 100Pa = 1mbar	
	_oss = MODE_HIGHRES;//设置过采样  MODE_ULTRA_HIGHRES  高精度测量模式

	BMP180_getCalData();               // initialize cal data
	BMP180_calcTrueTemperature(1);      // initialize b5
//  _param_centimeters=0;
//	BMP180_calcTruePressure(1); //TruePressure; 
 // _param_datum=	_Pa;

}

/****************************************************************
  *
  * 函数名：   BMP180_Routing
  *
	* 函数功能： BMP180 运行时调用的程序。 该程序会不停地读取气压值和温度值，
  *                                     用户需要定期调用这个程序，以更新高度和温度信息
  *               最好是间隔5ms~20ms调用                    
  * 备注： CRP          
  * 参考第七实验室代码
****************************************************************/ 
void BMP180_Routing(void)
{
			switch(BPM085_ST)
			{
					case SCTemperature: //开始温度转换
											BMP180_writemem(CONTROL, READ_TEMPERATURE); 
											BPM085_ST = CTemperatureing;//状态切换 
								break;
					
					case CTemperatureing: //正在进行温度转换
									 
											BMP180_calcTrueTemperature(0);
											BPM085_ST = SCPressure;
											UART_send_string(USART1,"\n温度：");
											UART_send_data(USART1,_Temperature_FIFO[BMP180_FIFO_length]);  //温度
									break;
					
					case SCPressure:  //开始气压计转换
											BMP180_writemem(CONTROL, READ_PRESSURE+(_oss << 6)); 
											BPM085_ST = SCPressureing; 
									break;
					
					case SCPressureing: //正在进行气压转换 
											BMP180_calcTruePressure(0);
											UART_send_string(USART1,"\t气压：");
											UART_send_data(USART1,_Pa_FIFO[BMP180_FIFO_length]); //气压 1hPa = 100Pa = 1mbar	
											UART_send_string(USART1," Pa\t高度：");
											UART_send_data(USART1,_centimeters_FIFO[BMP180_FIFO_length]); //高度
					            UART_send_string(USART1," cm");
											BPM085_ST = SCTemperature;		 
									break;
					
					default :
											BPM085_ST=SCTemperature; 
									break;
		 
			}

}
 

//------------------End of File----------------------------
