/*********************************************************************************
 *********************************************************************
  *
  * �������ܣ� ADC����ģ�����ʹ�ú���
  *
  * ����˵���� ADCx: ����ģ��ѡ�ţ�ADC1 ��ADC2 ��ADC3��  
  *    					 
  *            ADC_Channel�� ADCת��ͨ��     ADC1��ADC_Channel_x(0~7)
  *                                           ADC2��ADC_Channel_x(8~15)
  *                                           ADC3��ADC_Channel_x(4~8)
  * ��ע�� CRP
  * ע�� ��ͨ��֮����ִ��ŵ�����  ���ԼӴ��� MY_ADC_Value_Get�еĲ�������
  
  unsigned int MY_ADC_Value_Get(ADC_TypeDef* ADCx, uint8_t ADC_Channel);//��ɵ���ADCģ�����ɼ�����������װ��
  
  void MY_ADC_NoDMA_Init(ADC_TypeDef* ADCx, uint8_t ADC_Channel);//ADC��ʼ������---������DMAģʽ
  

//------------------------------STM32F4 ADCģ�����ŷ����----------------------------------------//
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
  * ��������   MY_ADC_NoDMA_Init
	* �������ܣ� ADC��ʼ������
  *
  * ����˵����  ADCx: ����ģ��ѡ�ţ�ADC1 ��ADC2 ��ADC3��  
  *    					 
	*             ADC_Channel�� ADCת��ͨ��   ADC1��ADC_Channel_x(0~7)
  *                                          ADC2��ADC_Channel_x(8~15)
  *                                          ADC3��ADC_Channel_x(4~8)
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void MY_ADC_DMA_Init(ADC_TypeDef* ADCx, uint8_t ADC_Channel)
{
	ADC_InitTypeDef ADC_InitStructure;	
	GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	if(ADCx == ADC1)
	{
		// ʹ��ADC��GPIOA��ʱ��
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
		
		 /* Enable ADC1 and GPIOC clock */
		if(ADC_Channel == ADC_Channel_0)
		{	    
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; // ����PA0Ϊģ������ģʽ
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_1)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; // ����PA1Ϊģ������ģʽ
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_2)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; // ����PA2Ϊģ������ģʽ
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_3)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; // ����PA3Ϊģ������ģʽ
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_4)
		{	    
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; // ����PA4Ϊģ������ģʽ
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_5)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // ����PA5Ϊģ������ģʽ
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_6)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // ����PA6Ϊģ������ģʽ
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);
		}
		else if(ADC_Channel == ADC_Channel_7)
		{
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; // ����PA7Ϊģ������ģʽ
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
	 
		ADC_CommonInitTypeDef ADC_CommonInitStructure;// ����ADC��������	
		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent; //����ADCģʽ
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
		ADC_CommonInit(&ADC_CommonInitStructure);

		// ����ADC1����
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//��ֹɨ��ģʽ��ɨ��ģʽ���ڶ�ͨ���ɼ�
		ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//��������ת��ģʽ������ͣ�ؽ���ADCת��
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
		ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//�ɼ������Ҷ���
		ADC_InitStructure.ADC_NbrOfConversion = 1;//Ҫת����ͨ����Ŀ1
		ADC_Init(ADC1, &ADC_InitStructure);

		 
		
		// ------------------DMA Init �ṹ����� ��ʼ��--------------------------
		// ADC1ʹ��DMA2��������0��ͨ��0��������ֲ�̶���
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE); // ����DMAʱ��
		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)ADC1+0x4c;	// �����ַΪ��ADC ���ݼĴ�����ַ
		DMA_InitStructure.DMA_Memory0BaseAddr = (u32)ADC_ConvertedValue;  // �洢����ַ��ʵ���Ͼ���һ���ڲ�SRAM�ı���	  
		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	  // ���ݴ��䷽��Ϊ���赽�洢��	
		DMA_InitStructure.DMA_BufferSize = 1;	 //����ֻ����һ��ͨ��// ��������СΪ��ָһ�δ����������
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;// ����Ĵ���ֻ��һ������ַ���õ���
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable; // �洢����ַ�̶�
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;  // // �������ݴ�СΪ���֣��������ֽ� 
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;	 //	�洢�����ݴ�СҲΪ���֣����������ݴ�С��ͬ
		DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;	// ѭ������ģʽ
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;  // DMA ����ͨ�����ȼ�Ϊ�ߣ���ʹ��һ��DMAͨ��ʱ�����ȼ����ò�Ӱ��
		DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  // ��ֹDMA FIFO	��ʹ��ֱ��ģʽ
		DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull; // FIFO ��С��FIFOģʽ��ֹʱ�������������	
		DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  
		
		DMA_InitStructure.DMA_Channel = DMA_Channel_0; // ѡ�� DMA ͨ����ͨ������������
		DMA_Init(DMA2_Stream0, &DMA_InitStructure);//��ʼ��DMA�������൱��һ����Ĺܵ����ܵ������кܶ�ͨ��
		DMA_Cmd(DMA2_Stream0, ENABLE);// ʹ��DMA��
	 
		
		ADC_RegularChannelConfig(ADCx, ADC_Channel, 1, ADC_SampleTime_56Cycles);// ���� ADC ͨ��ת��˳��Ͳ���ʱ������

		ADC_DMARequestAfterLastTransferCmd(ADCx, ENABLE);// ʹ��DMA����
		ADC_DMACmd(ADCx, ENABLE); // ʹ��ADC DMA
		ADC_Cmd(ADCx, ENABLE);  // ʹ��ADC
		ADC_SoftwareStartConv(ADCx); //��ʼadcת�����������
  
	}
	else 
	{
		while(1);
	}
}

 

 /****************************************************************
  *
  * ��������   MY_ADC_Value_Get
	* �������ܣ� ��ɵ���ADCģ�����ɼ�����������װ��
  *
  * ����˵����  ADCx: ����ģ��ѡ�ţ�ADC1 ��ADC2 ��ADC3��  
  *    					 
	*             ADC_Channel�� ADCת��ͨ��   ADC1��ADC_Channel_x(0~7)
  *                                         ADC2��ADC_Channel_x(8~15)
  *                                         ADC3��ADC_Channel_x(4~8)
  * ��ע�� CRP
  *  2014-10-28
  ****************************************************************/
unsigned int MY_ADC_Value_Get(ADC_TypeDef* ADCx, uint8_t ADC_Channel)
{
      while(!ADC_GetFlagStatus(ADCx, ADC_FLAG_EOC ));// 
      return ADC_GetConversionValue(ADCx);    // 
}
 
 

 
/**************************END OF FILE************************************/

