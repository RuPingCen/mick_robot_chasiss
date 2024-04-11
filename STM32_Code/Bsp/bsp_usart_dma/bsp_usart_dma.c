/*
* 每一条控制流最大能传输 256K 个字节
*
* USART2  Tx 流：6    Rx流：5    DMA通道4
*
*
*   My_USART_DMA_config(USARTx);  // 初始化串口为DMA模式
*
*   USART_DMA_Tx_Data(USART1,data_buff, 1024); //将数据写入缓存地址 并启动DMA传输
* 	
*	USART_DMACmd(USART1,USART_DMAReq_Rx,DISABLE);  // DMA接收配置
*
*/
#include "bsp_usart_dma.h"


uint8_t SendBuff[USART_MAX_TX_LEN];
uint8_t rx1_buff[USART_MAX_RX_LEN];

uint8_t USART2_RX_DMA_BUF[USART2_MAX_RX_LEN];  // 串口2 用于接收大疆遥控器数据

uint8_t wichbuf = 0;

/*
 * 函数名：USART_DMA_Tx_Data
 * 描述  ：串口DMA消息发送
 * 输入  ：串口号、待发送数据的存储地址、待发送数据的字节数
 * 输出  : 无
 * 调用  ：外部调用
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

// 串口1发送 中断
void DMA2_Stream7_IRQHandler(void)
{
	GPIO_ToggleBits(GPIOF,GPIO_Pin_6);
	if(DMA_GetITStatus(DMA2_Stream7 , DMA_IT_TCIF7) != RESET)
	{
		DMA_Cmd(DMA2_Stream7, DISABLE);
	    DMA_ClearFlag(DMA2_Stream7, DMA_IT_TCIF7);
	}
}
// 串口1 DMA接收 中断
void DMA2_Stream2_IRQHandler(void)
{
//    uint8_t *p;
    if(DMA_GetITStatus(DMA2_Stream2,DMA_IT_TCIF2) != RESET)                       //DMA接收完成标志
    {
		DMA_Cmd(DMA2_Stream2,DISABLE);
        DMA_ClearITPendingBit(DMA2_Stream2,DMA_IT_TCIF2);                         //清除DMA接收中断标志
        USART_ClearFlag(USART1,USART_FLAG_TC);                                      //关闭USART1标志位
        
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
        //*****************↓↓↓↓↓这里做数据处理↓↓↓↓↓*****************//
       GPIO_ToggleBits(GPIOF,GPIO_Pin_6); 
        //*****************↑↑↑↑↑这里做数据处理↑↑↑↑↑*****************//
    }
	
}



/*
 * 函数名：My_USART_DMA_config
 * 描述  ：串口DMA初始化
 * 输入  ：串口号
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART_DMA_config(USART_TypeDef* USARTx,unsigned int u32_Baud)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    
    DMA_Stream_TypeDef * DMAx_Steamy_TX;
    DMA_Stream_TypeDef * DMAx_Steamy_RX;
    
    My_Config_USART_Init(USARTx,u32_Baud,0);//初始化相应串口，并打开串口中断
	
    if(USARTx == USART1)
    {
        DMAx_Steamy_TX = DMA2_Stream7;        
        DMAx_Steamy_RX = DMA2_Stream2;  //默认使用DMA2_Stream2,Channel4作为USART1_RX,可更换为DMA2_Stream5,Channel4
        //DMAx_Steamy_RX = DMA2_Stream5;
        
        /* DMA接收、发送初始化 */
        My_USART1_DMA_TX_Init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//选着中断组
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //设置子优先级：1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //禁用DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
		
        My_USART1_DMA_RX_Init();
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//选着中断组
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //设置子优先级：0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;		                        //禁用DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
    }
    else if(USARTx == USART2)
    {
        DMAx_Steamy_TX = DMA1_Stream6;        
        DMAx_Steamy_RX = DMA1_Stream5;
        
        /* DMA接收、发送初始化 */
        My_USART2_DMA_TX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//选着中断组
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //设置子优先级：1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //禁用DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
		
		My_USART2_DMA_RX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//选着中断组
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //设置子优先级：0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	                            //开启DMA接收 NVIC
        NVIC_Init(&NVIC_InitStructure);	
    }
    else if(USARTx == USART3)
    {
        DMAx_Steamy_TX = DMA1_Stream3;                                              //默认使用DMA1_Stream3,Channel4作为USART3_TX,可更换为DMA1_Stream4,Channel7;若更换，请将TX和RX的DMA_Channel分开定义
        DMAx_Steamy_RX = DMA1_Stream1;
        
        /* DMA接收、发送初始化 */
        My_USART3_DMA_TX_Init();
        My_USART3_DMA_RX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//选着中断组
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //设置子优先级：1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //禁用DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
        
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);//选着中断组
        NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream1_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //设置子优先级：0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	                            //禁用DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
    }
    else if(USARTx == USART6)
    {
        DMAx_Steamy_TX = DMA2_Stream6;                                              //默认使用DMA2_Stream6,Channel5作为USART6_TX,可更换为DMA2_Stream7,Channel5
        //DMAx_Steamy_TX = DMA2_Stream7;
        DMAx_Steamy_RX = DMA2_Stream1;                                              //默认使用DMA2_Stream1,Channel5作为USART6_RX,可更换为DMA2_Stream2,Channel5
        //DMAx_Steamy_TX = DMA2_Stream2;
        
        /* DMA接收、发送初始化 */
        My_USART6_DMA_TX_Init();
        My_USART6_DMA_RX_Init();
        NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//选着中断组
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;                          //设置子优先级：1
        NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;	                        //禁用DMA NVIC
        NVIC_Init(&NVIC_InitStructure);	
        
        NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;                     //设置NVIC通道
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;                   //设置抢占优先级：3
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                          //设置子优先级：0
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	                            //禁用DMA NVIC	
        NVIC_Init(&NVIC_InitStructure);	
    }
    else
	{
		while(1);
	}
 
	DMA_Cmd(DMAx_Steamy_TX,ENABLE);                                                   //使能DMA
    while(DMA_GetCmdStatus(DMAx_Steamy_TX) != ENABLE); 								//等待DMA数据流有效
	
	DMA_Cmd(DMAx_Steamy_RX,ENABLE);                                                   //使能DMA
    while(DMA_GetCmdStatus(DMAx_Steamy_RX) != ENABLE); 								//等待DMA数据流有效
	
    USART_DMACmd(USARTx,USART_DMAReq_Tx,ENABLE);                                   
    USART_DMACmd(USARTx,USART_DMAReq_Rx,ENABLE);                                  
	
    DMA_ITConfig(DMAx_Steamy_TX,DMA_IT_TC,ENABLE);
    DMA_ITConfig(DMAx_Steamy_RX,DMA_IT_TC,ENABLE);
	
    USART_DMACmd(USARTx,USART_DMAReq_Tx,DISABLE);                                   //默认关闭DMA发送通道
    USART_DMACmd(USARTx,USART_DMAReq_Rx,DISABLE);                                   //默认关闭DMA接收通道
 
}

/*
 * 函数名：My_USART1_DMA_TX_Init
 * 描述  ：串口1的发送DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART1_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            // 使能DMA2时钟
    
    DMA_DeInit(DMA2_Stream7);                                                       // 将DMA2_Stream7的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //选择Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;               //设置外设地址：USART1的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //设置发送缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //传输方向：从内存到外设
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                            //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                           // FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍
    DMA_Init(DMA2_Stream7, &DMA_InitStructure);
    
}
/*
 * 函数名：My_USART1_DMA_RX_Init
 * 描述  ：串口1的接收DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART1_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            //使能DMA2时钟
    
    DMA_DeInit(DMA2_Stream2);                                                       // 将DMA2_Stream2的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //选择Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART1->DR;               //设置外设地址：USART1的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rx1_buff;                          //设置接收缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //传输方向：从外设到内存
    DMA_InitStructure.DMA_BufferSize = USART_MAX_RX_LEN;                            //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍 
    DMA_Init(DMA2_Stream2, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream2,ENABLE);                                                   //使能DMA
}
/*
 * 函数名：My_USART2_DMA_TX_Init
 * 描述  ：串口2的发送DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART2_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            // 使能DMA1时钟
    
    DMA_DeInit(DMA1_Stream6);                                                       // 将DMA1_Stream6的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //选择Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;               //设置外设地址：USART2的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //设置发送缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //传输方向：从内存到外设
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                           //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍
    DMA_Init(DMA1_Stream6, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream6,ENABLE);                                                   //使能DMA
}
/*
 * 函数名：My_USART2_DMA_RX_Init
 * 描述  ：串口2的接收DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART2_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            //使能DMA1时钟
    
    DMA_DeInit(DMA1_Stream5);                                                       // 将DMA1_Stream5的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA1_Stream5) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //选择Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART2->DR;               //设置外设地址：USART2的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)USART2_RX_DMA_BUF;                 //设置接收缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //传输方向：从外设到内存
    DMA_InitStructure.DMA_BufferSize = USART2_MAX_RX_LEN;                            //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍 
    DMA_Init(DMA1_Stream5, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream5,ENABLE);                                                   //使能DMA
}
/*
 * 函数名：My_USART3_DMA_TX_Init
 * 描述  ：串口3的发送DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART3_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            // 使能DMA1时钟
    
    DMA_DeInit(DMA1_Stream3);                                                       // 将DMA1_Stream3的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //选择Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;               //设置外设地址：USART3的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //设置发送缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //传输方向：从内存到外设
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                            //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍
    DMA_Init(DMA1_Stream3, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream3,ENABLE);                                                   //使能DMA
}
/*
 * 函数名：My_USART3_DMA_RX_Init
 * 描述  ：串口3的接收DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART3_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);                            //使能DMA1时钟
    
    DMA_DeInit(DMA1_Stream1);                                                       // 将DMA1_Stream1的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA1_Stream1) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_4;                                  //选择Channel_4
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART3->DR;               //设置外设地址：USART3的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rx1_buff;                          //设置接收缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //传输方向：从外设到内存
    DMA_InitStructure.DMA_BufferSize = USART_MAX_RX_LEN;                            //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍 
    DMA_Init(DMA1_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA1_Stream1,ENABLE);                                                   //使能DMA
}
/*
 * 函数名：My_USART6_DMA_TX_Init
 * 描述  ：串口6的发送DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART6_DMA_TX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            // 使能DMA2时钟
    
    DMA_DeInit(DMA2_Stream6);                                                       // 将DMA2_Stream6的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA2_Stream6) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;                                  //选择Channel_5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;               //设置外设地址：USART6的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)SendBuff;                          //设置发送缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;                         //传输方向：从内存到外设
    DMA_InitStructure.DMA_BufferSize = USART_MAX_TX_LEN;                            //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍
    DMA_Init(DMA2_Stream6, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream6,ENABLE);                                                   //使能DMA
}




 

/*
 * 函数名：My_USART6_DMA_RX_Init
 * 描述  ：串口6的接收DMA流通道初始化
 * 输入  ：无
 * 输出  : 无
 * 调用  ：外部调用
 */	
void My_USART6_DMA_RX_Init(void)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);                            //使能DMA2时钟
    
    DMA_DeInit(DMA2_Stream1);                                                       // 将DMA2_Stream1的寄存器重设为缺省值
    while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE);
    DMA_InitStructure.DMA_Channel = DMA_Channel_5;                                  //选择Channel_5
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&USART6->DR;               //设置外设地址：USART6的数据寄存器
    DMA_InitStructure.DMA_Memory0BaseAddr = (u32)rx1_buff;                          //设置接收缓冲区地址
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;                         //传输方向：从外设到内存
    DMA_InitStructure.DMA_BufferSize = USART_MAX_RX_LEN;                            //设置DMA缓存大小
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;                //外设地址不变
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;                         //内存地址自增
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;         //设置外设数据单位
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;                 //设置内存数据单位
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;                                   //设置DMA工作模式：正常模式
    DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;                           //设置DMA优先级：中
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;                          //禁用FIFO
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;                     //存储器突发传输16个节拍 
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;             //外设突发传输1个节拍 
    DMA_Init(DMA2_Stream1, &DMA_InitStructure);
    DMA_Cmd(DMA2_Stream1,ENABLE);                                                   //使能DMA
}

