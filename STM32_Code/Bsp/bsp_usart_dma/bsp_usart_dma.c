/*
* ÿһ������������ܴ��� 256K ���ֽ�
*
* USART2  Tx ����6    Rx����5    DMAͨ��4
*
*
*   My_USART_DMA_config(USARTx);  // ��ʼ������ΪDMAģʽ
*
*   USART_DMA_Tx_Data(USART1,data_buff, 1024); //������д�뻺���ַ ������DMA����
* 	
*	USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);  // DMA��������
*
*/
#include "bsp_usart_dma.h"


uint8_t SendBuff[USART_MAX_TX_LEN];
uint8_t rx1_buff[USART_MAX_RX_LEN];

uint8_t USART2_RX_DMA_BUF[USART2_MAX_RX_LEN];  // ����2 ���ڽ��մ�ң��������

uint8_t wichbuf = 0;

/*
 * ��������USART_DMA_Tx_Data
 * ����  ������DMA��Ϣ����
 * ����  �����ںš����������ݵĴ洢��ַ�����������ݵ��ֽ���
 * ���  : ��
 * ����  ���ⲿ����
 */	 
void USART_DMA_Tx_Data(USART_TypeDef* USARTx,uint8_t *send_buffer, uint32_t send_count)
{   
    //My_USART_DMA_config(USARTx);
    
    if(send_count <= USART_MAX_TX_LEN)
    {
        if(USARTx == USART1)
        {
            memcpy(SendBuff,send_buffer,send_count);
            //DMA_Cmd(DMA2_Stream7,DISABLE);
            //while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
            DMA_SetCurrDataCounter(DMA2_Stream7,send_count);
            DMA_Cmd(DMA2_Stream7,ENABLE);
            USART_DMACmd(USART1,USART_DMAReq_Tx,ENABLE);
        }
        else if(USARTx == USART2)
        {
            memcpy(SendBuff,send_buffer,send_count);
            //DMA_Cmd(DMA1_Stream6,DISABLE);
            //while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
            DMA_SetCurrDataCounter(DMA1_Stream6,send_count);
            DMA_Cmd(DMA1_Stream6,ENABLE);
            USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);
        }
        else if(USARTx == USART3)
        {
            memcpy(SendBuff,send_buffer,send_count);
            //DMA_Cmd(DMA1_Stream3,DISABLE);
            //while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
            DMA_SetCurrDataCounter(DMA1_Stream3,send_count);
            DMA_Cmd(DMA1_Stream3,ENABLE);
            USART_DMACmd(USART3,USART_DMAReq_Tx,ENABLE);
        }
        else
        {
            memcpy(SendBuff,send_buffer,send_count);
            //DMA_Cmd(DMA2_Stream6,DISABLE);
            //while(DMA_GetCmdStatus(DMA2_Stream6) != DISABLE);
            DMA_SetCurrDataCounter(DMA2_Stream6,send_count);
            DMA_Cmd(DMA2_Stream6,ENABLE);
            USART_DMACmd(USART6,USART_DMAReq_Tx,ENABLE);
        }
    }
}

// ����1���� �ж�
void DMA2_Stream7_IRQHandler(void)
{
	GPIO_ToggleBits(GPIOF,GPIO_Pin_6);
	if(DMA_GetITStatus(DMA2_Stream7 , DMA_IT_TCIF7) != RESET)
	{
		DMA_Cmd(DMA2_Stream7, DISABLE);
	    DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);
	}
}
// ����1 DMA���� �ж�
void DMA2_Stream2_IRQHandler(void)
{
//    uint8_t *p;
    if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2) != RESET)                       //DMA������ɱ�־
    {
		DMA_Cmd(DMA2_Stream2,DISABLE);
        DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);                         //���DMA�����жϱ�־
        USART_ClearFlag(USART1,USART_FLAG_TC);                                      //�ر�USART1��־λ
        
//        if(wichbuf)
//        {
//            p = rx1_buff;
//            DMA2_Stream2->M1AR = (u32)rx1_buff;
//            wichbuf = 0;            
//        }
//        else
//        {
//            p = rx1_buff;
//            DMA2_Stream2->M1AR = (u32)rx1_buff;
//            wichbuf = 1; 
//        }
        DMA2_Stream2->NDTR = USART_MAX_RX_LEN;
        DMA_Cmd(DMA2_Stream2,ENABLE);
        //*****************�������������������ݴ������������*****************//
       GPIO_ToggleBits(GPIOF,GPIO_Pin_6); 
        //*****************�������������������ݴ������������*****************//
    }
	
}



/*
 * ��������My_USART_DMA_config
 * ����  ������DMA��ʼ��
 * ����  �����ں�
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART_DMA_config(USART_TypeDef* USARTx,unsigned int u32_Baud)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    DMA_Stream_TypeDef * DMAx_Steamy_TX;
    DMA_Stream_TypeDef * DMAx_Steamy_RX;
    
    My_Config_USART_Init(USARTx,u32_Baud,0);//��ʼ����Ӧ���ڣ����򿪴����ж�
	
    if(USARTx == USART1)
    {
        DMAx_Steamy_TX = DMA2_Stream7;        
        DMAx_Steamy_RX = DMA2_Stream2;  //Ĭ��ʹ��DMA2_Stream2,Channel4��ΪUSART1_RX,�ɸ���ΪDMA2_Stream5,Channel4
        //DMAx_Steamy_RX = DMA2_Stream5;
        
        /* DMA���ա����ͳ�ʼ�� */
        My_USART1_DMA_TX_Init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//ѡ���ж���
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //���������ȼ���1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //����DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
		
        My_USART1_DMA_RX_Init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//ѡ���ж���
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //���������ȼ���0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		                        //����DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
    }
    else if(USARTx == USART2)
    {
        DMAx_Steamy_TX = DMA1_Stream6;        
        DMAx_Steamy_RX = DMA1_Stream5;
        
        /* DMA���ա����ͳ�ʼ�� */
        My_USART2_DMA_TX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//ѡ���ж���
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //���������ȼ���1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //����DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
		
		My_USART2_DMA_RX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//ѡ���ж���
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //���������ȼ���0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	                            //����DMA���� NVIC
        NVIC_Init(&NVIC_InitStructure);	
    }
    else if(USARTx == USART3)
    {
        DMAx_Steamy_TX = DMA1_Stream3;                                              //Ĭ��ʹ��DMA1_Stream3,Channel4��ΪUSART3_TX,�ɸ���ΪDMA1_Stream4,Channel7;���������뽫TX��RX��DMA_Channel�ֿ�����
        DMAx_Steamy_RX = DMA1_Stream1;
        
        /* DMA���ա����ͳ�ʼ�� */
        My_USART3_DMA_TX_Init();
        My_USART3_DMA_RX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//ѡ���ж���
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //���������ȼ���1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //����DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
        
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//ѡ���ж���
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //���������ȼ���0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	                            //����DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
    }
    else if(USARTx == USART6)
    {
        DMAx_Steamy_TX = DMA2_Stream6;                                              //Ĭ��ʹ��DMA2_Stream6,Channel5��ΪUSART6_TX,�ɸ���ΪDMA2_Stream7,Channel5
        //DMAx_Steamy_TX = DMA2_Stream7;
        DMAx_Steamy_RX = DMA2_Stream1;                                              //Ĭ��ʹ��DMA2_Stream1,Channel5��ΪUSART6_RX,�ɸ���ΪDMA2_Stream2,Channel5
        //DMAx_Steamy_TX = DMA2_Stream2;
        
        /* DMA���ա����ͳ�ʼ�� */
        My_USART6_DMA_TX_Init();
        My_USART6_DMA_RX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//ѡ���ж���
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //���������ȼ���1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //����DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
        
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;                     //����NVICͨ��
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //������ռ���ȼ���3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //���������ȼ���0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	                            //����DMA NVIC	
        NVIC_Init(&NVIC_InitStructure);	
    }
    else
	{
		while(1);
	}
 
	DMA_Cmd(DMAx_Steamy_TX,ENABLE);                                                   //ʹ��DMA
    while(DMA_GetCmdStatus(DMAx_Steamy_TX) != ENABLE); 								//�ȴ�DMA��������Ч
	
	DMA_Cmd(DMAx_Steamy_RX,ENABLE);                                                   //ʹ��DMA
    while(DMA_GetCmdStatus(DMAx_Steamy_RX) != ENABLE); 								//�ȴ�DMA��������Ч
	
    USART_DMACmd(USARTx,USART_DMAReq_Tx,ENABLE);                                   
    USART_DMACmd(USARTx,USART_DMAReq_Rx,ENABLE);                                  
	
    DMA_ITConfig(DMAx_Steamy_TX,DMA_IT_TC,ENABLE);
    DMA_ITConfig(DMAx_Steamy_RX,DMA_IT_TC,ENABLE);
	
    USART_DMACmd(USARTx,USART_DMAReq_Tx,DISABLE);                                   //Ĭ�Ϲر�DMA����ͨ��
    USART_DMACmd(USARTx,USART_DMAReq_Rx,DISABLE);                                   //Ĭ�Ϲر�DMA����ͨ��
 
}

/*
 * ��������My_USART1_DMA_TX_Init
 * ����  ������1�ķ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART1_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            // ʹ��DMA2ʱ��
    
    DMA_DeInit(DMA2_Stream7);                                                       // ��DMA2_Stream7�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //ѡ��Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;               //���������ַ��USART1�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //���÷��ͻ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //���䷽�򣺴��ڴ浽����
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                            //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                           // FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);
    
}
/*
 * ��������My_USART1_DMA_RX_Init
 * ����  ������1�Ľ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART1_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            //ʹ��DMA2ʱ��
    
    DMA_DeInit(DMA2_Stream2);                                                       // ��DMA2_Stream2�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //ѡ��Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;               //���������ַ��USART1�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rx1_buff;                          //���ý��ջ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //���䷽�򣺴����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = USART_MAX_RX_LEN;                            //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //����FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������ 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������ 
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream2,ENABLE);                                                   //ʹ��DMA
}
/*
 * ��������My_USART2_DMA_TX_Init
 * ����  ������2�ķ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART2_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            // ʹ��DMA1ʱ��
    
    DMA_DeInit(DMA1_Stream6);                                                       // ��DMA1_Stream6�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //ѡ��Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;               //���������ַ��USART2�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //���÷��ͻ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //���䷽�򣺴��ڴ浽����
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                           //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //����FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream6,ENABLE);                                                   //ʹ��DMA
}
/*
 * ��������My_USART2_DMA_RX_Init
 * ����  ������2�Ľ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART2_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            //ʹ��DMA1ʱ��
    
    DMA_DeInit(DMA1_Stream5);                                                       // ��DMA1_Stream5�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //ѡ��Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;               //���������ַ��USART2�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)USART2_RX_DMA_BUF;                 //���ý��ջ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //���䷽�򣺴����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = USART2_MAX_RX_LEN;                            //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //����FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������ 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������ 
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream5,ENABLE);                                                   //ʹ��DMA
}
/*
 * ��������My_USART3_DMA_TX_Init
 * ����  ������3�ķ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART3_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            // ʹ��DMA1ʱ��
    
    DMA_DeInit(DMA1_Stream3);                                                       // ��DMA1_Stream3�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //ѡ��Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;               //���������ַ��USART3�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //���÷��ͻ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //���䷽�򣺴��ڴ浽����
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                            //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //����FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream3,ENABLE);                                                   //ʹ��DMA
}
/*
 * ��������My_USART3_DMA_RX_Init
 * ����  ������3�Ľ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART3_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            //ʹ��DMA1ʱ��
    
    DMA_DeInit(DMA1_Stream1);                                                       // ��DMA1_Stream1�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //ѡ��Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;               //���������ַ��USART3�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rx1_buff;                          //���ý��ջ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //���䷽�򣺴����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = USART_MAX_RX_LEN;                            //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //����FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������ 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������ 
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1,ENABLE);                                                   //ʹ��DMA
}
/*
 * ��������My_USART6_DMA_TX_Init
 * ����  ������6�ķ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART6_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            // ʹ��DMA2ʱ��
    
    DMA_DeInit(DMA2_Stream6);                                                       // ��DMA2_Stream6�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA2_Stream6) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;                                  //ѡ��Channel_5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;               //���������ַ��USART6�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //���÷��ͻ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //���䷽�򣺴��ڴ浽����
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                            //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //����FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������
    DMA_Init(DMA2_Stream6, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream6,ENABLE);                                                   //ʹ��DMA
}




 

/*
 * ��������My_USART6_DMA_RX_Init
 * ����  ������6�Ľ���DMA��ͨ����ʼ��
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	
void My_USART6_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            //ʹ��DMA2ʱ��
    
    DMA_DeInit(DMA2_Stream1);                                                       // ��DMA2_Stream1�ļĴ�������Ϊȱʡֵ
    while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;                                  //ѡ��Channel_5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;               //���������ַ��USART6�����ݼĴ���
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rx1_buff;                          //���ý��ջ�������ַ
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //���䷽�򣺴����赽�ڴ�
    DMA_InitStructure.DMA_BufferSize = USART_MAX_RX_LEN;                            //����DMA�����С
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //�����ַ����
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //�ڴ��ַ����
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //�����������ݵ�λ
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //�����ڴ����ݵ�λ
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //����DMA����ģʽ������ģʽ
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //����DMA���ȼ�����
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //����FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //�洢��ͻ������16������ 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //����ͻ������1������ 
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream1,ENABLE);                                                   //ʹ��DMA
}

