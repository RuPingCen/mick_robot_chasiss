/*********************************************************************************
 *********************************************************************
  *
  * 函数功能： ADC功能模块相关使用函数
  *
  * 参数说明： ADCx: 串口模块选着（ADC1 、ADC2 、ADC3）  
  *    					 
  *            ADC_Channel： ADC转换通道     ADC1：ADC_Channel_x(0~7)
  *                                           ADC2：ADC_Channel_x(8~15)
  *                                           ADC3：ADC_Channel_x(4~8)
  * 备注： CRP
  * 注： 若通道之间出现串扰的现象  可以加大函数 MY_ADC_Value_Get中的采样周期
  
  unsigned int MY_ADC_Value_Get(ADC_TypeDef* ADCx, uint8_t ADC_Channel);//完成单次ADC模拟量采集到数字量的装换
  
  void MY_ADC_NoDMA_Init(ADC_TypeDef* ADCx, uint8_t ADC_Channel);//ADC初始化函数---不采用DMA模式
  

//------------------------------STM32F4 ADC模块引脚分配表----------------------------------------//
							ADC1                 ADC2              ADC3

		ADC_Channel_0       PA0                  PA0               PA0         
		ADC_Channel_1       PA1                  PA1               PA1           
		ADC_Channel_2       PA2                  PA2               PA2              
		ADC_Channel_3       PA3                  PA3               PA3             
		ADC_Channel_4       PA4                  PA4               PF6                 
		ADC_Channel_5       PA5                  PA5               PF7              
		ADC_Channel_6       PA6                  PA6               PF8                  
		ADC_Channel_7       PA7                  PA7               PF9                     
		ADC_Channel_8       PB0                  PB0               PF10            
		ADC_Channel_9       PB1                  PB1               PF3                
		ADC_Channel_10      PC0                  PC0               PC0               
		ADC_Channel_11      PC1                  PC1               PC1                  
		ADC_Channel_12      PC2                  PC2               PC2               
		ADC_Channel_13      PC3                  PC3               PC3                      
		ADC_Channel_14      PC4                  PC4               PF4                             
		ADC_Channel_15      PC5                  PC5               PF5     
		ADC_Channel_16    TempSensor               
		ADC_Channel_17    Vrefint                  
**********************************************************************************/

#include "stm32f4xx.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"

#include "bsp_adc.h" 
 
 __IO uint16_t ADC_ConvertedValue[1]={0};
 
/****************************************************************
  *
  * 函数名：   MY_ADC_NoDMA_Init
	* 函数功能： ADC初始化函数
  *
  * 参数说明：  ADCx: 串口模块选着（ADC1 、ADC2 、ADC3）  
  *    					 
	*             ADC_Channel： ADC转换通道   ADC1：ADC_Channel_x(0~7)
  *                                          ADC2：ADC_Channel_x(8~15)
  *                                          ADC3：ADC_Channel_x(4~8)
  * 备注： CRP
  * 2014-10-28
  ****************************************************************/
void MY_ADC_DMA_Init(ADC_TypeDef* ADCx, uint8_t ADC_Channel)
{
	ADC_InitTypeDef ADC_InitStructure;	
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	if(ADCx == ADC1)
	{
		// 使能ADC和GPIOA的时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		
		 /* Enable ADC1 and GPIOC clock */
		if(ADC_Channel == ADC_Channel_0)
		{	    
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // 配置PA0为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_1)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // 配置PA1为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_2)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // 配置PA2为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_3)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // 配置PA3为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_4)
		{	    
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // 配置PA4为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_5)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // 配置PA5为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_6)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // 配置PA6为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_7)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // 配置PA7为模拟输入模式
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_TempSensor)
		{
			ADC_TempSensorVrefintCmd(ENABLE);
		}
		else if(ADC_Channel == ADC_Channel_Vrefint )
		{
			ADC_TempSensorVrefintCmd(ENABLE);
		}
		else 
		{
			while(1);
		}
	 
		ADC_CommonInitTypeDef ADC_CommonInitStructure;// 配置ADC公用设置	
		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; //独立ADC模式
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
		ADC_CommonInit(&ADC_CommonInitStructure);

		// 配置ADC1设置
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//禁止扫描模式，扫描模式用于多通道采集
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//开启连续转换模式，即不停地进行ADC转换
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//采集数据右对齐
		ADC_InitStructure.ADC_NbrOfConversion = 1;//要转换的通道数目1
		ADC_Init(ADC1, &ADC_InitStructure);

		 
		
		// ------------------DMA Init 结构体参数 初始化--------------------------
		// ADC1使用DMA2，数据流0，通道0，这个是手册固定的
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // 开启DMA时钟
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1+0x4c;	// 外设基址为：ADC 数据寄存器地址
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC_ConvertedValue;  // 存储器地址，实际上就是一个内部SRAM的变量	  
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	  // 数据传输方向为外设到存储器	
		DMA_InitStructure.DMA_BufferSize = 1;	 //这里只用了一个通道// 缓冲区大小为，指一次传输的数据量
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// 外设寄存器只有一个，地址不用递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; // 存储器地址固定
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // // 外设数据大小为半字，即两个字节 
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	 //	存储器数据大小也为半字，跟外设数据大小相同
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	// 循环传输模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;  // DMA 传输通道优先级为高，当使用一个DMA通道时，优先级设置不影响
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // 禁止DMA FIFO	，使用直连模式
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; // FIFO 大小，FIFO模式禁止时，这个不用配置	
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
		
		DMA_InitStructure.DMA_Channel = DMA_Channel_0; // 选择 DMA 通道，通道存在于流中
		DMA_Init(DMA2_Stream0, &DMA_InitStructure);//初始化DMA流，流相当于一个大的管道，管道里面有很多通道
		DMA_Cmd(DMA2_Stream0, ENABLE);// 使能DMA流
	 
		
		ADC_RegularChannelConfig(ADCx, ADC_Channel, 1, ADC_SampleTime_56Cycles);// 配置 ADC 通道转换顺序和采样时间周期

		ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);// 使能DMA请求
		ADC_DMACmd(ADCx, ENABLE); // 使能ADC DMA
		ADC_Cmd(ADCx, ENABLE);  // 使能ADC
		ADC_SoftwareStartConv(ADCx); //开始adc转换，软件触发
  
	}
	else 
	{
		while(1);
	}
}

 

 /****************************************************************
  *
  * 函数名：   MY_ADC_Value_Get
	* 函数功能： 完成单次ADC模拟量采集到数字量的装换
  *
  * 参数说明：  ADCx: 串口模块选着（ADC1 、ADC2 、ADC3）  
  *    					 
	*             ADC_Channel： ADC转换通道   ADC1：ADC_Channel_x(0~7)
  *                                         ADC2：ADC_Channel_x(8~15)
  *                                         ADC3：ADC_Channel_x(4~8)
  * 备注： CRP
  *  2014-10-28
  ****************************************************************/
unsigned int MY_ADC_Value_Get(ADC_TypeDef* ADCx, uint8_t ADC_Channel)
{
      while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC ));// 
      return ADC_GetConversionValue(ADCx);    // 
}
 
 

 
/**************************END OF FILE************************************/

