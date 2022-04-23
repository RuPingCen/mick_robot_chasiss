 /****************************************************************************
*
*  �������ƣ� 
*  ����˵����STM32  DAC ͨ�� �����ʼ������
*  ����˵����u16TIM_Period    �����������==  1/72M  *( u16TIM_Period * u16data_number)
*            send_datatemp[]  �������ݴ�ŵĵ�ַ
*            u16data_number    ����Ԫ�صĸ���
*
*  DAC_1   PA4
*  DAC_2   PA5
*  
*  STM32_DAC_channel1_DMAmode_Init(u16TIM_Period,send_datatemp[],u16data_number);//DACͨ��1 ��ʼ������ 
*  STM32_DAC_channel2_DMAmode_Init(u16TIM_Period,send_datatemp[],u16data_number);//DACͨ��2 ��ʼ������ 
*
* 
* ��    ע��CRP   (DAC ͨ��2ʹ�ö�ʱ��6  ��DMA2 ͨ�� 4) ע�ⲻҪ����ģ��������ͻ   
*                 (DAC ͨ��1ʹ�ö�ʱ��5  ��DMA2 ͨ�� 3) ע�ⲻҪ����ģ��������ͻ
* �޸�ʱ�䣺2014-11-1


		 uint16_t Sine12bit[32] = {
			2448,2832,3186,3496,3751,3940,4057,4095,4057,3940,
			3751,3496,3186,2832,2448,2048,1648,1264,910,600,345,
			156,39,0,39,156,345,600,910,1264,1648,2048
		};//���Ҳ���� ��������


 *****************************************************************************/
 
 
#include "stm32f10x_dac.h"
#include "stm32f10x_rcc.h"





#define DAC_DHR12RD_Address      0x40007420 //0x40007420 ˫ͨ��ģʽ�µ����ݴ�ŵ�ַ

#define DAC_DHR12R1_Address      0x40007408 
#define DAC_DHR12R2_Address      0x40007414 



 
#define CR_CLEAR_MASK              ((uint32_t)0x00000FFE)

/* DAC Dual Channels SWTRIG masks */
#define DUAL_SWTRIG_SET            ((uint32_t)0x00000003)
#define DUAL_SWTRIG_RESET          ((uint32_t)0xFFFFFFFC)

/* DHR registers offsets */
#define DHR12R1_OFFSET             ((uint32_t)0x00000008)
#define DHR12R2_OFFSET             ((uint32_t)0x00000014)
#define DHR12RD_OFFSET             ((uint32_t)0x00000020)

/* DOR register offset */
#define DOR_OFFSET                 ((uint32_t)0x0000002C)
 /*************************************************************************
*  �������ƣ�STM32_DAC_channel2_DMAmode_Init
*  ����˵����STM32  DAC ͨ�� 2�����ʼ������
*  ����˵����u16TIM_Period    �����������== 1/72M  * u16TIM_Period * u16data_number
*            send_datatemp[]  �������ݴ�ŵĵ�ַ
*            u16data_number    ����Ԫ�صĸ���
*
*  �������أ���
*  �޸�ʱ�䣺2014-11-1
*  ��    ע��CRP  (DAC ͨ��2ʹ�ö�ʱ��6  ��DMA2 ͨ�� 4) ע�ⲻҪ����ģ��������ͻ
*************************************************************************/
void STM32_DAC_channel2_DMAmode_Init(uint16_t u16TIM_Period , uint16_t send_datatemp[],uint16_t u16data_number )
{
		 GPIO_InitTypeDef          GPIO_InitStructure;
			DAC_InitTypeDef            DAC_InitStructure;
			DMA_InitTypeDef            DMA_InitStructure;
			TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;

		 
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	/* ʹ��GPIOAʱ�� */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);/* ʹ��DACʱ�� */	
			
			/* DAC��GPIO���ã�ģ������ */
			GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//DAC���ܱ�������Ϊģ������ģʽ
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			

			/* ����DAC ͨ��2 */
			DAC_InitStructure.DAC_Trigger = DAC_Trigger_T6_TRGO;						//ʹ��TIM5��Ϊ����Դ
			DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;	//��ʹ�ò��η�����
			DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;	//��ʹ��DAC������� ���û�ʹ���������Ӵ�
			DAC_Init(DAC_Channel_2, &DAC_InitStructure);
			DAC_Cmd(DAC_Channel_2, ENABLE);  /* ʹ��ͨ��2 ��PA5��� */
			DAC_DMACmd(DAC_Channel_2, ENABLE); /* ʹ��DAC��DMA���� */
        
				
			 /* TIM5������ʱ������ */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);/* ʹ��TIM5ʱ�ӣ�TIM5CLK Ϊ72M */
			TIM_TimeBaseStructure.TIM_Period = u16TIM_Period-1;       									//��ʱ���� 20  
			TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       							//Ԥ��Ƶ������Ƶ 72M / (0+1) = 72M
			TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    						//ʱ�ӷ�Ƶϵ��
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ
			TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);
			TIM_SelectOutputTrigger(TIM6, TIM_TRGOSource_Update); /* ����TIM6����Դ */
			TIM_Cmd(TIM6, ENABLE);/* ʹ��TIM6 */

					 /* ����DMA2 */
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);/* ʹ��DMA2ʱ�� */
			DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12R2_Address;					//�������ݵ�ַ
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)send_datatemp ;				//�ڴ����ݵ�ַ DualSine12bit
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;											//���ݴ��䷽���ڴ�������
			DMA_InitStructure.DMA_BufferSize = u16data_number;																	//�����СΪ32�ֽ�	
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//�������ݵ�ַ�̶�	
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//�ڴ����ݵ�ַ����
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//���������԰���Ϊ��λ
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;					//�ڴ������԰���Ϊ��λ	
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;													//ѭ��ģʽ
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;											//��DMAͨ�����ȼ�
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;														//���ڴ����ڴ�ģʽ	
			DMA_Init(DMA2_Channel4, &DMA_InitStructure);
			DMA_Cmd(DMA2_Channel4, ENABLE); /* ʹ��DMA2-4ͨ�� */

  }
/*************************************************************************
*  �������ƣ�STM32_DAC_channel1_DMAmode_Init
*  ����˵����STM32  DAC ͨ�� 1�����ʼ������
*  ����˵����u16TIM_Period    �����������== 1/72M  * u16TIM_Period * u16data_number
*            send_datatemp[]  �������ݴ�ŵĵ�ַ
*            u16data_number    ����Ԫ�صĸ���
*
*  �������أ���
*  �޸�ʱ�䣺2014-11-1
*  ��    ע��CRP  (DAC ͨ��1ʹ�ö�ʱ��5  ��DMA2 ͨ�� 3) ע�ⲻҪ����ģ��������ͻ
*************************************************************************/
void STM32_DAC_channel1_DMAmode_Init(uint16_t u16TIM_Period , uint16_t send_datatemp[],uint16_t u16data_number)
{
			GPIO_InitTypeDef          GPIO_InitStructure;
			DAC_InitTypeDef            DAC_InitStructure;
			DMA_InitTypeDef            DMA_InitStructure;
			TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;

		 
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	/* ʹ��GPIOAʱ�� */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);/* ʹ��DACʱ�� */	
			
			/* DAC��GPIO���ã�ģ������ */
			GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 ;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;//DAC���ܱ�������Ϊģ������ģʽ
			GPIO_Init(GPIOA, &GPIO_InitStructure);
			

			/* ����DAC ͨ��1 */
			DAC_InitStructure.DAC_Trigger = DAC_Trigger_T5_TRGO;						//ʹ��TIM5��Ϊ����Դ
			DAC_InitStructure.DAC_WaveGeneration = DAC_WaveGeneration_None;	//��ʹ�ò��η�����
			DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;	//��ʹ��DAC������� ���û�ʹ���������Ӵ�
			DAC_Init(DAC_Channel_1, &DAC_InitStructure);
			DAC_Cmd(DAC_Channel_1, ENABLE);  /* ʹ��ͨ��1 ��PA4��� */
			DAC_DMACmd(DAC_Channel_1, ENABLE); /* ʹ��DAC��DMA���� */
        
				
			 /* TIM5������ʱ������ */
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);/* ʹ��TIM5ʱ�ӣ�TIM5CLK Ϊ72M */
			TIM_TimeBaseStructure.TIM_Period = u16TIM_Period-1;       									//��ʱ���� 20  
			TIM_TimeBaseStructure.TIM_Prescaler = 0x0;       							//Ԥ��Ƶ������Ƶ 72M / (0+1) = 72M
			TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;    						//ʱ�ӷ�Ƶϵ��
			TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  	//���ϼ���ģʽ
			TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
			TIM_SelectOutputTrigger(TIM5, TIM_TRGOSource_Update); /* ����TIM5����Դ */
			TIM_Cmd(TIM5, ENABLE);/* ʹ��TIM5 */

					 /* ����DMA2 */
			RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);/* ʹ��DMA2ʱ�� */
			DMA_InitStructure.DMA_PeripheralBaseAddr = DAC_DHR12R1_Address;					//�������ݵ�ַ
			DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)send_datatemp ;				//�ڴ����ݵ�ַ DualSine12bit
			DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;											//���ݴ��䷽���ڴ�������
			DMA_InitStructure.DMA_BufferSize = u16data_number;																	//�����СΪ32�ֽ�	
			DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//�������ݵ�ַ�̶�	
			DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									//�ڴ����ݵ�ַ����
			DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;	//���������԰���Ϊ��λ
			DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;					//�ڴ������԰���Ϊ��λ	
			DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;													//ѭ��ģʽ
			DMA_InitStructure.DMA_Priority = DMA_Priority_High;											//��DMAͨ�����ȼ�
			DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;														//���ڴ����ڴ�ģʽ	
			DMA_Init(DMA2_Channel3, &DMA_InitStructure);
			DMA_Cmd(DMA2_Channel3, ENABLE); /* ʹ��DMA2-3ͨ�� */
	
}
void DAC_DeInit(void)
{
  /* Enable DAC reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, ENABLE);
  /* Release DAC from reset state */
  RCC_APB1PeriphResetCmd(RCC_APB1Periph_DAC, DISABLE);
}

/**
  * @brief  Initializes the DAC peripheral according to the specified 
  *         parameters in the DAC_InitStruct.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_InitStruct: pointer to a DAC_InitTypeDef structure that
  *        contains the configuration information for the specified DAC channel.
  * @retval None
  */
void DAC_Init(uint32_t DAC_Channel, DAC_InitTypeDef* DAC_InitStruct)
{
  uint32_t tmpreg1 = 0, tmpreg2 = 0;
  /* Check the DAC parameters */
  assert_param(IS_DAC_TRIGGER(DAC_InitStruct->DAC_Trigger));
  assert_param(IS_DAC_GENERATE_WAVE(DAC_InitStruct->DAC_WaveGeneration));
  assert_param(IS_DAC_LFSR_UNMASK_TRIANGLE_AMPLITUDE(DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude));
  assert_param(IS_DAC_OUTPUT_BUFFER_STATE(DAC_InitStruct->DAC_OutputBuffer));
/*---------------------------- DAC CR Configuration --------------------------*/
  /* Get the DAC CR value */
  tmpreg1 = DAC->CR;
  /* Clear BOFFx, TENx, TSELx, WAVEx and MAMPx bits */
  tmpreg1 &= ~(CR_CLEAR_MASK << DAC_Channel);
  /* Configure for the selected DAC channel: buffer output, trigger, wave generation,
     mask/amplitude for wave generation */
  /* Set TSELx and TENx bits according to DAC_Trigger value */
  /* Set WAVEx bits according to DAC_WaveGeneration value */
  /* Set MAMPx bits according to DAC_LFSRUnmask_TriangleAmplitude value */ 
  /* Set BOFFx bit according to DAC_OutputBuffer value */   
  tmpreg2 = (DAC_InitStruct->DAC_Trigger | DAC_InitStruct->DAC_WaveGeneration |
             DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude | DAC_InitStruct->DAC_OutputBuffer);
  /* Calculate CR register value depending on DAC_Channel */
  tmpreg1 |= tmpreg2 << DAC_Channel;
  /* Write to DAC CR */
  DAC->CR = tmpreg1;
}

/**
  * @brief  Fills each DAC_InitStruct member with its default value.
  * @param  DAC_InitStruct : pointer to a DAC_InitTypeDef structure which will
  *         be initialized.
  * @retval None
  */
void DAC_StructInit(DAC_InitTypeDef* DAC_InitStruct)
{
/*--------------- Reset DAC init structure parameters values -----------------*/
  /* Initialize the DAC_Trigger member */
  DAC_InitStruct->DAC_Trigger = DAC_Trigger_None;
  /* Initialize the DAC_WaveGeneration member */
  DAC_InitStruct->DAC_WaveGeneration = DAC_WaveGeneration_None;
  /* Initialize the DAC_LFSRUnmask_TriangleAmplitude member */
  DAC_InitStruct->DAC_LFSRUnmask_TriangleAmplitude = DAC_LFSRUnmask_Bit0;
  /* Initialize the DAC_OutputBuffer member */
  DAC_InitStruct->DAC_OutputBuffer = DAC_OutputBuffer_Enable;
}

/**
  * @brief  Enables or disables the specified DAC channel.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the DAC channel. 
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_Cmd(uint32_t DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected DAC channel */
    DAC->CR |= (DAC_CR_EN1 << DAC_Channel);
  }
  else
  {
    /* Disable the selected DAC channel */
    DAC->CR &= ~(DAC_CR_EN1 << DAC_Channel);
  }
}
#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
/**
  * @brief  Enables or disables the specified DAC interrupts.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt sources to be enabled or disabled. 
  *   This parameter can be the following values:
  *     @arg DAC_IT_DMAUDR: DMA underrun interrupt mask                      
  * @param  NewState: new state of the specified DAC interrupts.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */ 
void DAC_ITConfig(uint32_t DAC_Channel, uint32_t DAC_IT, FunctionalState NewState)  
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  assert_param(IS_DAC_IT(DAC_IT)); 

  if (NewState != DISABLE)
  {
    /* Enable the selected DAC interrupts */
    DAC->CR |=  (DAC_IT << DAC_Channel);
  }
  else
  {
    /* Disable the selected DAC interrupts */
    DAC->CR &= (~(uint32_t)(DAC_IT << DAC_Channel));
  }
}
#endif

/**
  * @brief  Enables or disables the specified DAC channel DMA request.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel DMA request.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_DMACmd(uint32_t DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected DAC channel DMA request */
    DAC->CR |= (DAC_CR_DMAEN1 << DAC_Channel);
  }
  else
  {
    /* Disable the selected DAC channel DMA request */
    DAC->CR &= ~(DAC_CR_DMAEN1 << DAC_Channel);
  }
}

/**
  * @brief  Enables or disables the selected DAC channel software trigger.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  NewState: new state of the selected DAC channel software trigger.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_SoftwareTriggerCmd(uint32_t DAC_Channel, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable software trigger for the selected DAC channel */
    DAC->SWTRIGR |= (uint32_t)DAC_SWTRIGR_SWTRIG1 << (DAC_Channel >> 4);
  }
  else
  {
    /* Disable software trigger for the selected DAC channel */
    DAC->SWTRIGR &= ~((uint32_t)DAC_SWTRIGR_SWTRIG1 << (DAC_Channel >> 4));
  }
}

/**
  * @brief  Enables or disables simultaneously the two DAC channels software
  *   triggers.
  * @param  NewState: new state of the DAC channels software triggers.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_DualSoftwareTriggerCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable software trigger for both DAC channels */
    DAC->SWTRIGR |= DUAL_SWTRIG_SET ;
  }
  else
  {
    /* Disable software trigger for both DAC channels */
    DAC->SWTRIGR &= DUAL_SWTRIG_RESET;
  }
}

/**
  * @brief  Enables or disables the selected DAC channel wave generation.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_Wave: Specifies the wave type to enable or disable.
  *   This parameter can be one of the following values:
  *     @arg DAC_Wave_Noise: noise wave generation
  *     @arg DAC_Wave_Triangle: triangle wave generation
  * @param  NewState: new state of the selected DAC channel wave generation.
  *   This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DAC_WaveGenerationCmd(uint32_t DAC_Channel, uint32_t DAC_Wave, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_WAVE(DAC_Wave)); 
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the selected wave generation for the selected DAC channel */
    DAC->CR |= DAC_Wave << DAC_Channel;
  }
  else
  {
    /* Disable the selected wave generation for the selected DAC channel */
    DAC->CR &= ~(DAC_Wave << DAC_Channel);
  }
}

/**
  * @brief  Set the specified data holding register value for DAC channel1.
  * @param  DAC_Align: Specifies the data alignment for DAC channel1.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignment selected
  *     @arg DAC_Align_12b_L: 12bit left data alignment selected
  *     @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data : Data to be loaded in the selected data holding register.
  * @retval None
  */
void DAC_SetChannel1Data(uint32_t DAC_Align, uint16_t Data)
{  
  __IO uint32_t tmp = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data));
  
  tmp = (uint32_t)DAC_BASE; 
  tmp += DHR12R1_OFFSET + DAC_Align;

  /* Set the DAC channel1 selected data holding register */
  *(__IO uint32_t *) tmp = Data;
}

/**
  * @brief  Set the specified data holding register value for DAC channel2.
  * @param  DAC_Align: Specifies the data alignment for DAC channel2.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignment selected
  *     @arg DAC_Align_12b_L: 12bit left data alignment selected
  *     @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data : Data to be loaded in the selected data holding register.
  * @retval None
  */
void DAC_SetChannel2Data(uint32_t DAC_Align, uint16_t Data)
{
  __IO uint32_t tmp = 0;

  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data));
  
  tmp = (uint32_t)DAC_BASE;
  tmp += DHR12R2_OFFSET + DAC_Align;

  /* Set the DAC channel2 selected data holding register */
  *(__IO uint32_t *)tmp = Data;
}

/**
  * @brief  Set the specified data holding register value for dual channel
  *   DAC.
  * @param  DAC_Align: Specifies the data alignment for dual channel DAC.
  *   This parameter can be one of the following values:
  *     @arg DAC_Align_8b_R: 8bit right data alignment selected
  *     @arg DAC_Align_12b_L: 12bit left data alignment selected
  *     @arg DAC_Align_12b_R: 12bit right data alignment selected
  * @param  Data2: Data for DAC Channel2 to be loaded in the selected data 
  *   holding register.
  * @param  Data1: Data for DAC Channel1 to be loaded in the selected data 
  *   holding register.
  * @retval None
  */
void DAC_SetDualChannelData(uint32_t DAC_Align, uint16_t Data2, uint16_t Data1)
{
  uint32_t data = 0, tmp = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_ALIGN(DAC_Align));
  assert_param(IS_DAC_DATA(Data1));
  assert_param(IS_DAC_DATA(Data2));
  
  /* Calculate and set dual DAC data holding register value */
  if (DAC_Align == DAC_Align_8b_R)
  {
    data = ((uint32_t)Data2 << 8) | Data1; 
  }
  else
  {
    data = ((uint32_t)Data2 << 16) | Data1;
  }
  
  tmp = (uint32_t)DAC_BASE;
  tmp += DHR12RD_OFFSET + DAC_Align;

  /* Set the dual DAC selected data holding register */
  *(__IO uint32_t *)tmp = data;
}

/**
  * @brief  Returns the last data output value of the selected DAC channel.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @retval The selected DAC channel data output value.
  */
uint16_t DAC_GetDataOutputValue(uint32_t DAC_Channel)
{
  __IO uint32_t tmp = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  
  tmp = (uint32_t) DAC_BASE ;
  tmp += DOR_OFFSET + ((uint32_t)DAC_Channel >> 2);
  
  /* Returns the DAC channel data output register value */
  return (uint16_t) (*(__IO uint32_t*) tmp);
}

#if defined (STM32F10X_LD_VL) || defined (STM32F10X_MD_VL) || defined (STM32F10X_HD_VL)
/**
  * @brief  Checks whether the specified DAC flag is set or not.
  * @param  DAC_Channel: thee selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_FLAG: specifies the flag to check. 
  *   This parameter can be only of the following value:
  *     @arg DAC_FLAG_DMAUDR: DMA underrun flag                                                 
  * @retval The new state of DAC_FLAG (SET or RESET).
  */
FlagStatus DAC_GetFlagStatus(uint32_t DAC_Channel, uint32_t DAC_FLAG)
{
  FlagStatus bitstatus = RESET;
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_FLAG(DAC_FLAG));

  /* Check the status of the specified DAC flag */
  if ((DAC->SR & (DAC_FLAG << DAC_Channel)) != (uint8_t)RESET)
  {
    /* DAC_FLAG is set */
    bitstatus = SET;
  }
  else
  {
    /* DAC_FLAG is reset */
    bitstatus = RESET;
  }
  /* Return the DAC_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Clears the DAC channelx's pending flags.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_FLAG: specifies the flag to clear. 
  *   This parameter can be of the following value:
  *     @arg DAC_FLAG_DMAUDR: DMA underrun flag                           
  * @retval None
  */
void DAC_ClearFlag(uint32_t DAC_Channel, uint32_t DAC_FLAG)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_FLAG(DAC_FLAG));

  /* Clear the selected DAC flags */
  DAC->SR = (DAC_FLAG << DAC_Channel);
}

/**
  * @brief  Checks whether the specified DAC interrupt has occurred or not.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt source to check. 
  *   This parameter can be the following values:
  *     @arg DAC_IT_DMAUDR: DMA underrun interrupt mask                       
  * @retval The new state of DAC_IT (SET or RESET).
  */
ITStatus DAC_GetITStatus(uint32_t DAC_Channel, uint32_t DAC_IT)
{
  ITStatus bitstatus = RESET;
  uint32_t enablestatus = 0;
  
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_IT(DAC_IT));

  /* Get the DAC_IT enable bit status */
  enablestatus = (DAC->CR & (DAC_IT << DAC_Channel)) ;
  
  /* Check the status of the specified DAC interrupt */
  if (((DAC->SR & (DAC_IT << DAC_Channel)) != (uint32_t)RESET) && enablestatus)
  {
    /* DAC_IT is set */
    bitstatus = SET;
  }
  else
  {
    /* DAC_IT is reset */
    bitstatus = RESET;
  }
  /* Return the DAC_IT status */
  return  bitstatus;
}

/**
  * @brief  Clears the DAC channelx's interrupt pending bits.
  * @param  DAC_Channel: the selected DAC channel. 
  *   This parameter can be one of the following values:
  *     @arg DAC_Channel_1: DAC Channel1 selected
  *     @arg DAC_Channel_2: DAC Channel2 selected
  * @param  DAC_IT: specifies the DAC interrupt pending bit to clear.
  *   This parameter can be the following values:
  *     @arg DAC_IT_DMAUDR: DMA underrun interrupt mask                         
  * @retval None
  */
void DAC_ClearITPendingBit(uint32_t DAC_Channel, uint32_t DAC_IT)
{
  /* Check the parameters */
  assert_param(IS_DAC_CHANNEL(DAC_Channel));
  assert_param(IS_DAC_IT(DAC_IT)); 

  /* Clear the selected DAC interrupt pending bits */
  DAC->SR = (DAC_IT << DAC_Channel);
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
