/************************************************************************
* 
*  CAN 差分信号的表示
*  1:隐性电平   H2.5v - L2.5v = 0v
*  0:显性电平   H3.5v - L1.5v = 2v
*
*  参数说明： CANx:  CAN1
*             Dvice_ID : 目的ID
*             tempdat  数据存放首地址
*             dat_number 数据发送个数( 0 ~ 8 ) 
*
*  函数返回： 无
*  修改时间： 2014-11-2 
*  备    注： CRP   
*
*  STM32_CAN_Commun_Config(u32 Our_Dvice_ID);//初始化函数
*  STM32_CAN_Transmit(CAN_TypeDef* CANx,u32 Dvice_ID,u8 tempdat[],u8 dat_number);//数据发送函数
*

void USB_LP_CAN1_RX0_IRQHandler(void)//中断函数书写范例
{
 
 
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //从邮箱中读出报文//

  // 比较ID和数据是否为0x1314及DCBA // 
   if((RxMessage.ExtId==0x1314) && (RxMessage.IDE==CAN_ID_EXT)
     && (RxMessage.DLC==2))
		{
			  UART_send_string(USART1,"STM32 can 接收成功\n");//字符串函数
		}
		else
		{
			  ;
		}
}
**************************************************************************/
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_can.h"
#include "stm32f10x_rcc.h"

 

 

#define MCR_DBF      ((uint32_t)0x00010000) /* software master reset */

/* CAN Mailbox Transmit Request */
#define TMIDxR_TXRQ  ((uint32_t)0x00000001) /* Transmit mailbox request */

/* CAN Filter Master Register bits */
#define FMR_FINIT    ((uint32_t)0x00000001) /* Filter init mode */

/* Time out for INAK bit */
#define INAK_TIMEOUT        ((uint32_t)0x0000FFFF)
/* Time out for SLAK bit */
#define SLAK_TIMEOUT        ((uint32_t)0x0000FFFF)



/* Flags in TSR register */
#define CAN_FLAGS_TSR              ((uint32_t)0x08000000) 
/* Flags in RF1R register */
#define CAN_FLAGS_RF1R             ((uint32_t)0x04000000) 
/* Flags in RF0R register */
#define CAN_FLAGS_RF0R             ((uint32_t)0x02000000) 
/* Flags in MSR register */
#define CAN_FLAGS_MSR              ((uint32_t)0x01000000) 
/* Flags in ESR register */
#define CAN_FLAGS_ESR              ((uint32_t)0x00F00000) 

/* Mailboxes definition */
#define CAN_TXMAILBOX_0                   ((uint8_t)0x00)
#define CAN_TXMAILBOX_1                   ((uint8_t)0x01)
#define CAN_TXMAILBOX_2                   ((uint8_t)0x02) 



#define CAN_MODE_MASK              ((uint32_t) 0x00000003)
 

static ITStatus CheckITStatus(uint32_t CAN_Reg, uint32_t It_Bit);




/*************************************************************************
*  函数名称： STM32_CAN_Commun_Config
*  功能说明： STM32 CAN 通信协议的配置
*  参数说明： Our_Dvice_ID 本机ID号
*
*  函数返回： 无
*  修改时间： 2014-11-2 
*  备    注： CRP     只能使用CAN1 没有发现模块二的接收中断向量
*                     模式是正常模式   需要两个开发板才可以测试   
*                     一个开发板测试时需要将通讯模式设置为 环回模式
*************************************************************************/
void STM32_CAN_Commun_Config(u32 Our_Dvice_ID)
{
   	GPIO_InitTypeDef GPIO_InitStructure;
   	NVIC_InitTypeDef NVIC_InitStructure;
	  CAN_InitTypeDef  CAN_InitStructure;
	  CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	
	 
     /*RCC_APB2Periph_AFIO  启用引脚重映射功能 */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//开启can1 通道
	  GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);//引脚重映射
  	 
	/* Configure CAN pin: RX */									               // PB8
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // 上拉输入
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);	
    /* Configure CAN pin: TX */									               // PB9
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // 复用推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
     
		/*中断设置*/
	  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);/* Configure one bit for preemption priority */	 	
	  NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;			   //子优先级为7
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
		
		
	 	/************************CAN通信参数设置**********************************/
	  /*CAN寄存器初始化*/
	  CAN_DeInit(CAN1);
	  CAN_StructInit(&CAN_InitStructure);
	
	   /*CAN单元初始化*/
	  CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  关闭时间触发通信模式使能
    CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  自动离线管理 
    CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  使用自动唤醒模式  当检测到CAN总线活动的时候，会自动进入正常模式
    
	  CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  禁止报文自动重传	  DISABLE-自动重传
    
	  CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  接收FIFO 锁定模式  DISABLE-溢出时新报文会覆盖原有报文  
    CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  发送FIFO优先级 DISABLE-优先级取决于报文标示符 
    
	  CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //正常工作模式 CAN_Mode_Normal  CAN_Mode_LoopBack
	
    CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		   //BTR-SJW 重新同步跳跃宽度 2个时间单元
    CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;		   //BTR-TS1 时间段1 占用了6个时间单元
    CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 时间段2 占用了3个时间单元
    CAN_InitStructure.CAN_Prescaler =4;		   ////BTR-BRP 波特率分频器  定义了时间单元的时间长度 36/(1+6+3)/4=0.9Mbps
	  CAN_Init(CAN1, &CAN_InitStructure);
		


	  /*CAN过滤器初始化*/
	  CAN_FilterInitStructure.CAN_FilterNumber=0;						//过滤器组0
    CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//工作在标识符屏蔽位模式
	  CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//过滤器位宽为单个32位。
	  /* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */

    CAN_FilterInitStructure.CAN_FilterIdHigh= (((u32)Our_Dvice_ID<<3)&0xFFFF0000)>>16;				//要过滤的ID高位 
    CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)Our_Dvice_ID<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //要过滤的ID低位 
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0xFFFF;			//过滤器高16位每位必须匹配
    CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0xFFFF;			//过滤器低16位每位必须匹配
	  CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//过滤器被关联到FIFO0
	  CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//使能过滤器
	  CAN_FilterInit(&CAN_FilterInitStructure);
	   /*CAN通信中断使能*/
	  CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
 
}
/*************************************************************************
*  函数名称： STM32_CAN_Transmit
*  功能说明： STM32 CAN 数据传送
*  参数说明： CANx:  CAN1
*             Dvice_ID : 目的ID
*             tempdat  数据存放首地址
*             dat_number 数据发送个数( 0 ~ 8 ) 
*
*  函数返回： 无
*  修改时间： 2014-11-2 
*  备    注： CRP      
*************************************************************************/
void STM32_CAN_Transmit(CAN_TypeDef* CANx,u32 Dvice_ID,u8 tempdat[],u8 dat_number)
{
     CanTxMsg TxMessage;
	   unsigned char i=0;
	
	TxMessage.ExtId=Dvice_ID;					 //使用的扩展ID
  TxMessage.IDE=CAN_ID_EXT;					 //扩展模式
  TxMessage.RTR=CAN_RTR_DATA;				 //发送的是数据
  TxMessage.DLC=dat_number;				   //数据长度为字节
	
	for(i=0;i<dat_number;i++)
	{
     TxMessage.Data[i]=*tempdat;
		 tempdat++;

  }
  CAN_Transmit(CANx, &TxMessage);
	
	 
}
 

/**
  * @}
  */

/** @defgroup CAN_Private_Functions
  * @{
  */

/**
  * @brief  Deinitializes the CAN peripheral registers to their default reset values.
  * @param  CANx: where x can be 1 or 2 to select the CAN peripheral.
  * @retval None.
  */
void CAN_DeInit(CAN_TypeDef* CANx)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
 
  if (CANx == CAN1)
  {
    /* Enable CAN1 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, ENABLE);
    /* Release CAN1 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN1, DISABLE);
  }
  else
  {  
    /* Enable CAN2 reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, ENABLE);
    /* Release CAN2 from reset state */
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_CAN2, DISABLE);
  }
}

/**
  * @brief  Initializes the CAN peripheral according to the specified
  *         parameters in the CAN_InitStruct.
  * @param  CANx:           where x can be 1 or 2 to to select the CAN 
  *                         peripheral.
  * @param  CAN_InitStruct: pointer to a CAN_InitTypeDef structure that
  *                         contains the configuration information for the 
  *                         CAN peripheral.
  * @retval Constant indicates initialization succeed which will be 
  *         CAN_InitStatus_Failed or CAN_InitStatus_Success.
  */
uint8_t CAN_Init(CAN_TypeDef* CANx, CAN_InitTypeDef* CAN_InitStruct)
{
  uint8_t InitStatus = CAN_InitStatus_Failed;
  uint32_t wait_ack = 0x00000000;
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_TTCM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_ABOM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_AWUM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_NART));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_RFLM));
  assert_param(IS_FUNCTIONAL_STATE(CAN_InitStruct->CAN_TXFP));
  assert_param(IS_CAN_MODE(CAN_InitStruct->CAN_Mode));
  assert_param(IS_CAN_SJW(CAN_InitStruct->CAN_SJW));
  assert_param(IS_CAN_BS1(CAN_InitStruct->CAN_BS1));
  assert_param(IS_CAN_BS2(CAN_InitStruct->CAN_BS2));
  assert_param(IS_CAN_PRESCALER(CAN_InitStruct->CAN_Prescaler));

  /* Exit from sleep mode */
  CANx->MCR &= (~(uint32_t)CAN_MCR_SLEEP);

  /* Request initialisation */
  CANx->MCR |= CAN_MCR_INRQ ;

  /* Wait the acknowledge */
  while (((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
  {
    wait_ack++;
  }

  /* Check acknowledge */
  if ((CANx->MSR & CAN_MSR_INAK) != CAN_MSR_INAK)
  {
    InitStatus = CAN_InitStatus_Failed;
  }
  else 
  {
    /* Set the time triggered communication mode */
    if (CAN_InitStruct->CAN_TTCM == ENABLE)
    {
      CANx->MCR |= CAN_MCR_TTCM;
    }
    else
    {
      CANx->MCR &= ~(uint32_t)CAN_MCR_TTCM;
    }

    /* Set the automatic bus-off management */
    if (CAN_InitStruct->CAN_ABOM == ENABLE)
    {
      CANx->MCR |= CAN_MCR_ABOM;
    }
    else
    {
      CANx->MCR &= ~(uint32_t)CAN_MCR_ABOM;
    }

    /* Set the automatic wake-up mode */
    if (CAN_InitStruct->CAN_AWUM == ENABLE)
    {
      CANx->MCR |= CAN_MCR_AWUM;
    }
    else
    {
      CANx->MCR &= ~(uint32_t)CAN_MCR_AWUM;
    }

    /* Set the no automatic retransmission */
    if (CAN_InitStruct->CAN_NART == ENABLE)
    {
      CANx->MCR |= CAN_MCR_NART;
    }
    else
    {
      CANx->MCR &= ~(uint32_t)CAN_MCR_NART;
    }

    /* Set the receive FIFO locked mode */
    if (CAN_InitStruct->CAN_RFLM == ENABLE)
    {
      CANx->MCR |= CAN_MCR_RFLM;
    }
    else
    {
      CANx->MCR &= ~(uint32_t)CAN_MCR_RFLM;
    }

    /* Set the transmit FIFO priority */
    if (CAN_InitStruct->CAN_TXFP == ENABLE)
    {
      CANx->MCR |= CAN_MCR_TXFP;
    }
    else
    {
      CANx->MCR &= ~(uint32_t)CAN_MCR_TXFP;
    }

    /* Set the bit timing register */
    CANx->BTR = (uint32_t)((uint32_t)CAN_InitStruct->CAN_Mode << 30) | \
                ((uint32_t)CAN_InitStruct->CAN_SJW << 24) | \
                ((uint32_t)CAN_InitStruct->CAN_BS1 << 16) | \
                ((uint32_t)CAN_InitStruct->CAN_BS2 << 20) | \
               ((uint32_t)CAN_InitStruct->CAN_Prescaler - 1);

    /* Request leave initialisation */
    CANx->MCR &= ~(uint32_t)CAN_MCR_INRQ;

   /* Wait the acknowledge */
   wait_ack = 0;

   while (((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) && (wait_ack != INAK_TIMEOUT))
   {
     wait_ack++;
   }

    /* ...and check acknowledged */
    if ((CANx->MSR & CAN_MSR_INAK) == CAN_MSR_INAK)
    {
      InitStatus = CAN_InitStatus_Failed;
    }
    else
    {
      InitStatus = CAN_InitStatus_Success ;
    }
  }

  /* At this step, return the status of initialization */
  return InitStatus;
}

/**
  * @brief  Initializes the CAN peripheral according to the specified
  *         parameters in the CAN_FilterInitStruct.
  * @param  CAN_FilterInitStruct: pointer to a CAN_FilterInitTypeDef
  *                               structure that contains the configuration 
  *                               information.
  * @retval None.
  */
void CAN_FilterInit(CAN_FilterInitTypeDef* CAN_FilterInitStruct)
{
  uint32_t filter_number_bit_pos = 0;
  /* Check the parameters */
  assert_param(IS_CAN_FILTER_NUMBER(CAN_FilterInitStruct->CAN_FilterNumber));
  assert_param(IS_CAN_FILTER_MODE(CAN_FilterInitStruct->CAN_FilterMode));
  assert_param(IS_CAN_FILTER_SCALE(CAN_FilterInitStruct->CAN_FilterScale));
  assert_param(IS_CAN_FILTER_FIFO(CAN_FilterInitStruct->CAN_FilterFIFOAssignment));
  assert_param(IS_FUNCTIONAL_STATE(CAN_FilterInitStruct->CAN_FilterActivation));

  filter_number_bit_pos = ((uint32_t)1) << CAN_FilterInitStruct->CAN_FilterNumber;

  /* Initialisation mode for the filter */
  CAN1->FMR |= FMR_FINIT;

  /* Filter Deactivation */
  CAN1->FA1R &= ~(uint32_t)filter_number_bit_pos;

  /* Filter Scale */
  if (CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_16bit)
  {
    /* 16-bit scale for the filter */
    CAN1->FS1R &= ~(uint32_t)filter_number_bit_pos;

    /* First 16-bit identifier and First 16-bit mask */
    /* Or First 16-bit identifier and Second 16-bit identifier */
    CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 = 
    ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow) << 16) |
        (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdLow);

    /* Second 16-bit identifier and Second 16-bit mask */
    /* Or Third 16-bit identifier and Fourth 16-bit identifier */
    CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 = 
    ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
        (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdHigh);
  }

  if (CAN_FilterInitStruct->CAN_FilterScale == CAN_FilterScale_32bit)
  {
    /* 32-bit scale for the filter */
    CAN1->FS1R |= filter_number_bit_pos;
    /* 32-bit identifier or First 32-bit identifier */
    CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR1 = 
    ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdHigh) << 16) |
        (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterIdLow);
    /* 32-bit mask or Second 32-bit identifier */
    CAN1->sFilterRegister[CAN_FilterInitStruct->CAN_FilterNumber].FR2 = 
    ((0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdHigh) << 16) |
        (0x0000FFFF & (uint32_t)CAN_FilterInitStruct->CAN_FilterMaskIdLow);
  }

  /* Filter Mode */
  if (CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdMask)
  {
    /*Id/Mask mode for the filter*/
    CAN1->FM1R &= ~(uint32_t)filter_number_bit_pos;
  }
  else /* CAN_FilterInitStruct->CAN_FilterMode == CAN_FilterMode_IdList */
  {
    /*Identifier list mode for the filter*/
    CAN1->FM1R |= (uint32_t)filter_number_bit_pos;
  }

  /* Filter FIFO assignment */
  if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_Filter_FIFO0)
  {
    /* FIFO 0 assignation for the filter */
    CAN1->FFA1R &= ~(uint32_t)filter_number_bit_pos;
  }

  if (CAN_FilterInitStruct->CAN_FilterFIFOAssignment == CAN_Filter_FIFO1)
  {
    /* FIFO 1 assignation for the filter */
    CAN1->FFA1R |= (uint32_t)filter_number_bit_pos;
  }
  
  /* Filter activation */
  if (CAN_FilterInitStruct->CAN_FilterActivation == ENABLE)
  {
    CAN1->FA1R |= filter_number_bit_pos;
  }

  /* Leave the initialisation mode for the filter */
  CAN1->FMR &= ~FMR_FINIT;
}

/**
  * @brief  Fills each CAN_InitStruct member with its default value.
  * @param  CAN_InitStruct: pointer to a CAN_InitTypeDef structure which
  *                         will be initialized.
  * @retval None.
  */
void CAN_StructInit(CAN_InitTypeDef* CAN_InitStruct)
{
  /* Reset CAN init structure parameters values */
  
  /* Initialize the time triggered communication mode */
  CAN_InitStruct->CAN_TTCM = DISABLE;
  
  /* Initialize the automatic bus-off management */
  CAN_InitStruct->CAN_ABOM = DISABLE;
  
  /* Initialize the automatic wake-up mode */
  CAN_InitStruct->CAN_AWUM = DISABLE;
  
  /* Initialize the no automatic retransmission */
  CAN_InitStruct->CAN_NART = DISABLE;
  
  /* Initialize the receive FIFO locked mode */
  CAN_InitStruct->CAN_RFLM = DISABLE;
  
  /* Initialize the transmit FIFO priority */
  CAN_InitStruct->CAN_TXFP = DISABLE;
  
  /* Initialize the CAN_Mode member */
  CAN_InitStruct->CAN_Mode = CAN_Mode_Normal;
  
  /* Initialize the CAN_SJW member */
  CAN_InitStruct->CAN_SJW = CAN_SJW_1tq;
  
  /* Initialize the CAN_BS1 member */
  CAN_InitStruct->CAN_BS1 = CAN_BS1_4tq;
  
  /* Initialize the CAN_BS2 member */
  CAN_InitStruct->CAN_BS2 = CAN_BS2_3tq;
  
  /* Initialize the CAN_Prescaler member */
  CAN_InitStruct->CAN_Prescaler = 1;
}

/**
  * @brief  Select the start bank filter for slave CAN.
  * @note   This function applies only to STM32 Connectivity line devices.
  * @param  CAN_BankNumber: Select the start slave bank filter from 1..27.
  * @retval None.
  */
void CAN_SlaveStartBank(uint8_t CAN_BankNumber) 
{
  /* Check the parameters */
  assert_param(IS_CAN_BANKNUMBER(CAN_BankNumber));
  
  /* Enter Initialisation mode for the filter */
  CAN1->FMR |= FMR_FINIT;
  
  /* Select the start slave bank */
  CAN1->FMR &= (uint32_t)0xFFFFC0F1 ;
  CAN1->FMR |= (uint32_t)(CAN_BankNumber)<<8;
  
  /* Leave Initialisation mode for the filter */
  CAN1->FMR &= ~FMR_FINIT;
}

/**
  * @brief  Enables or disables the DBG Freeze for CAN.
  * @param  CANx:     where x can be 1 or 2 to to select the CAN peripheral.
  * @param  NewState: new state of the CAN peripheral. This parameter can 
  *                   be: ENABLE or DISABLE.
  * @retval None.
  */
void CAN_DBGFreeze(CAN_TypeDef* CANx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable Debug Freeze  */
    CANx->MCR |= MCR_DBF;
  }
  else
  {
    /* Disable Debug Freeze */
    CANx->MCR &= ~MCR_DBF;
  }
}


/**
  * @brief  Enables or disabes the CAN Time TriggerOperation communication mode.
  * @param  CANx:      where x can be 1 or 2 to to select the CAN peripheral.
  * @param  NewState : Mode new state , can be one of @ref FunctionalState.
  * @note   when enabled, Time stamp (TIME[15:0]) value is sent in the last 
  *         two data bytes of the 8-byte message: TIME[7:0] in data byte 6 
  *         and TIME[15:8] in data byte 7 
  * @note   DLC must be programmed as 8 in order Time Stamp (2 bytes) to be 
  *         sent over the CAN bus.  
  * @retval None
  */
void CAN_TTComModeCmd(CAN_TypeDef* CANx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  if (NewState != DISABLE)
  {
    /* Enable the TTCM mode */
    CANx->MCR |= CAN_MCR_TTCM;

    /* Set TGT bits */
    CANx->sTxMailBox[0].TDTR |= ((uint32_t)CAN_TDT0R_TGT);
    CANx->sTxMailBox[1].TDTR |= ((uint32_t)CAN_TDT1R_TGT);
    CANx->sTxMailBox[2].TDTR |= ((uint32_t)CAN_TDT2R_TGT);
  }
  else
  {
    /* Disable the TTCM mode */
    CANx->MCR &= (uint32_t)(~(uint32_t)CAN_MCR_TTCM);

    /* Reset TGT bits */
    CANx->sTxMailBox[0].TDTR &= ((uint32_t)~CAN_TDT0R_TGT);
    CANx->sTxMailBox[1].TDTR &= ((uint32_t)~CAN_TDT1R_TGT);
    CANx->sTxMailBox[2].TDTR &= ((uint32_t)~CAN_TDT2R_TGT);
  }
}
/**
  * @brief  Initiates the transmission of a message.
  * @param  CANx:      where x can be 1 or 2 to to select the CAN peripheral.
  * @param  TxMessage: pointer to a structure which contains CAN Id, CAN
  *                    DLC and CAN data.
  * @retval The number of the mailbox that is used for transmission
  *                    or CAN_TxStatus_NoMailBox if there is no empty mailbox.
  */
uint8_t CAN_Transmit(CAN_TypeDef* CANx, CanTxMsg* TxMessage)
{
  uint8_t transmit_mailbox = 0;
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_IDTYPE(TxMessage->IDE));
  assert_param(IS_CAN_RTR(TxMessage->RTR));
  assert_param(IS_CAN_DLC(TxMessage->DLC));

  /* Select one empty transmit mailbox */
  if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
  {
    transmit_mailbox = 0;
  }
  else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
  {
    transmit_mailbox = 1;
  }
  else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
  {
    transmit_mailbox = 2;
  }
  else
  {
    transmit_mailbox = CAN_TxStatus_NoMailBox;
  }

  if (transmit_mailbox != CAN_TxStatus_NoMailBox)
  {
    /* Set up the Id */
    CANx->sTxMailBox[transmit_mailbox].TIR &= TMIDxR_TXRQ;
    if (TxMessage->IDE == CAN_Id_Standard)
    {
      assert_param(IS_CAN_STDID(TxMessage->StdId));  
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->StdId << 21) | \
                                                  TxMessage->RTR);
    }
    else
    {
      assert_param(IS_CAN_EXTID(TxMessage->ExtId));
      CANx->sTxMailBox[transmit_mailbox].TIR |= ((TxMessage->ExtId << 3) | \
                                                  TxMessage->IDE | \
                                                  TxMessage->RTR);
    }
    
    /* Set up the DLC */
    TxMessage->DLC &= (uint8_t)0x0000000F;
    CANx->sTxMailBox[transmit_mailbox].TDTR &= (uint32_t)0xFFFFFFF0;
    CANx->sTxMailBox[transmit_mailbox].TDTR |= TxMessage->DLC;

    /* Set up the data field */
    CANx->sTxMailBox[transmit_mailbox].TDLR = (((uint32_t)TxMessage->Data[3] << 24) | 
                                             ((uint32_t)TxMessage->Data[2] << 16) |
                                             ((uint32_t)TxMessage->Data[1] << 8) | 
                                             ((uint32_t)TxMessage->Data[0]));
    CANx->sTxMailBox[transmit_mailbox].TDHR = (((uint32_t)TxMessage->Data[7] << 24) | 
                                             ((uint32_t)TxMessage->Data[6] << 16) |
                                             ((uint32_t)TxMessage->Data[5] << 8) |
                                             ((uint32_t)TxMessage->Data[4]));
    /* Request transmission */
    CANx->sTxMailBox[transmit_mailbox].TIR |= TMIDxR_TXRQ;
  }
  return transmit_mailbox;
}

/**
  * @brief  Checks the transmission of a message.
  * @param  CANx:            where x can be 1 or 2 to to select the 
  *                          CAN peripheral.
  * @param  TransmitMailbox: the number of the mailbox that is used for 
  *                          transmission.
  * @retval CAN_TxStatus_Ok if the CAN driver transmits the message, CAN_TxStatus_Failed 
  *         in an other case.
  */
uint8_t CAN_TransmitStatus(CAN_TypeDef* CANx, uint8_t TransmitMailbox)
{
  uint32_t state = 0;

  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_TRANSMITMAILBOX(TransmitMailbox));
 
  switch (TransmitMailbox)
  {
    case (CAN_TXMAILBOX_0): 
      state =   CANx->TSR &  (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0);
      break;
    case (CAN_TXMAILBOX_1): 
      state =   CANx->TSR &  (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1);
      break;
    case (CAN_TXMAILBOX_2): 
      state =   CANx->TSR &  (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2);
      break;
    default:
      state = CAN_TxStatus_Failed;
      break;
  }
  switch (state)
  {
      /* transmit pending  */
    case (0x0): state = CAN_TxStatus_Pending;
      break;
      /* transmit failed  */
     case (CAN_TSR_RQCP0 | CAN_TSR_TME0): state = CAN_TxStatus_Failed;
      break;
     case (CAN_TSR_RQCP1 | CAN_TSR_TME1): state = CAN_TxStatus_Failed;
      break;
     case (CAN_TSR_RQCP2 | CAN_TSR_TME2): state = CAN_TxStatus_Failed;
      break;
      /* transmit succeeded  */
    case (CAN_TSR_RQCP0 | CAN_TSR_TXOK0 | CAN_TSR_TME0):state = CAN_TxStatus_Ok;
      break;
    case (CAN_TSR_RQCP1 | CAN_TSR_TXOK1 | CAN_TSR_TME1):state = CAN_TxStatus_Ok;
      break;
    case (CAN_TSR_RQCP2 | CAN_TSR_TXOK2 | CAN_TSR_TME2):state = CAN_TxStatus_Ok;
      break;
    default: state = CAN_TxStatus_Failed;
      break;
  }
  return (uint8_t) state;
}

/**
  * @brief  Cancels a transmit request.
  * @param  CANx:     where x can be 1 or 2 to to select the CAN peripheral. 
  * @param  Mailbox:  Mailbox number.
  * @retval None.
  */
void CAN_CancelTransmit(CAN_TypeDef* CANx, uint8_t Mailbox)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_TRANSMITMAILBOX(Mailbox));
  /* abort transmission */
  switch (Mailbox)
  {
    case (CAN_TXMAILBOX_0): CANx->TSR |= CAN_TSR_ABRQ0;
      break;
    case (CAN_TXMAILBOX_1): CANx->TSR |= CAN_TSR_ABRQ1;
      break;
    case (CAN_TXMAILBOX_2): CANx->TSR |= CAN_TSR_ABRQ2;
      break;
    default:
      break;
  }
}


/**
  * @brief  Receives a message.
  * @param  CANx:       where x can be 1 or 2 to to select the CAN peripheral.
  * @param  FIFONumber: Receive FIFO number, CAN_FIFO0 or CAN_FIFO1.
  * @param  RxMessage:  pointer to a structure receive message which contains 
  *                     CAN Id, CAN DLC, CAN datas and FMI number.
  * @retval None.
  */
void CAN_Receive(CAN_TypeDef* CANx, uint8_t FIFONumber, CanRxMsg* RxMessage)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_FIFO(FIFONumber));
  /* Get the Id */
  RxMessage->IDE = (uint8_t)0x04 & CANx->sFIFOMailBox[FIFONumber].RIR;
  if (RxMessage->IDE == CAN_Id_Standard)
  {
    RxMessage->StdId = (uint32_t)0x000007FF & (CANx->sFIFOMailBox[FIFONumber].RIR >> 21);
  }
  else
  {
    RxMessage->ExtId = (uint32_t)0x1FFFFFFF & (CANx->sFIFOMailBox[FIFONumber].RIR >> 3);
  }
  
  RxMessage->RTR = (uint8_t)0x02 & CANx->sFIFOMailBox[FIFONumber].RIR;
  /* Get the DLC */
  RxMessage->DLC = (uint8_t)0x0F & CANx->sFIFOMailBox[FIFONumber].RDTR;
  /* Get the FMI */
  RxMessage->FMI = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDTR >> 8);
  /* Get the data field */
  RxMessage->Data[0] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONumber].RDLR;
  RxMessage->Data[1] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 8);
  RxMessage->Data[2] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 16);
  RxMessage->Data[3] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDLR >> 24);
  RxMessage->Data[4] = (uint8_t)0xFF & CANx->sFIFOMailBox[FIFONumber].RDHR;
  RxMessage->Data[5] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 8);
  RxMessage->Data[6] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 16);
  RxMessage->Data[7] = (uint8_t)0xFF & (CANx->sFIFOMailBox[FIFONumber].RDHR >> 24);
  /* Release the FIFO */
  /* Release FIFO0 */
  if (FIFONumber == CAN_FIFO0)
  {
    CANx->RF0R |= CAN_RF0R_RFOM0;
  }
  /* Release FIFO1 */
  else /* FIFONumber == CAN_FIFO1 */
  {
    CANx->RF1R |= CAN_RF1R_RFOM1;
  }
}

/**
  * @brief  Releases the specified FIFO.
  * @param  CANx:       where x can be 1 or 2 to to select the CAN peripheral. 
  * @param  FIFONumber: FIFO to release, CAN_FIFO0 or CAN_FIFO1.
  * @retval None.
  */
void CAN_FIFORelease(CAN_TypeDef* CANx, uint8_t FIFONumber)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_FIFO(FIFONumber));
  /* Release FIFO0 */
  if (FIFONumber == CAN_FIFO0)
  {
    CANx->RF0R |= CAN_RF0R_RFOM0;
  }
  /* Release FIFO1 */
  else /* FIFONumber == CAN_FIFO1 */
  {
    CANx->RF1R |= CAN_RF1R_RFOM1;
  }
}

/**
  * @brief  Returns the number of pending messages.
  * @param  CANx:       where x can be 1 or 2 to to select the CAN peripheral.
  * @param  FIFONumber: Receive FIFO number, CAN_FIFO0 or CAN_FIFO1.
  * @retval NbMessage : which is the number of pending message.
  */
uint8_t CAN_MessagePending(CAN_TypeDef* CANx, uint8_t FIFONumber)
{
  uint8_t message_pending=0;
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_FIFO(FIFONumber));
  if (FIFONumber == CAN_FIFO0)
  {
    message_pending = (uint8_t)(CANx->RF0R&(uint32_t)0x03);
  }
  else if (FIFONumber == CAN_FIFO1)
  {
    message_pending = (uint8_t)(CANx->RF1R&(uint32_t)0x03);
  }
  else
  {
    message_pending = 0;
  }
  return message_pending;
}


/**
  * @brief   Select the CAN Operation mode.
  * @param CAN_OperatingMode : CAN Operating Mode. This parameter can be one 
  *                            of @ref CAN_OperatingMode_TypeDef enumeration.
  * @retval status of the requested mode which can be 
  *         - CAN_ModeStatus_Failed    CAN failed entering the specific mode 
  *         - CAN_ModeStatus_Success   CAN Succeed entering the specific mode 

  */
uint8_t CAN_OperatingModeRequest(CAN_TypeDef* CANx, uint8_t CAN_OperatingMode)
{
  uint8_t status = CAN_ModeStatus_Failed;
  
  /* Timeout for INAK or also for SLAK bits*/
  uint32_t timeout = INAK_TIMEOUT; 

  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_OPERATING_MODE(CAN_OperatingMode));

  if (CAN_OperatingMode == CAN_OperatingMode_Initialization)
  {
    /* Request initialisation */
    CANx->MCR = (uint32_t)((CANx->MCR & (uint32_t)(~(uint32_t)CAN_MCR_SLEEP)) | CAN_MCR_INRQ);

    /* Wait the acknowledge */
    while (((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_INAK) && (timeout != 0))
    {
      timeout--;
    }
    if ((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_INAK)
    {
      status = CAN_ModeStatus_Failed;
    }
    else
    {
      status = CAN_ModeStatus_Success;
    }
  }
  else  if (CAN_OperatingMode == CAN_OperatingMode_Normal)
  {
    /* Request leave initialisation and sleep mode  and enter Normal mode */
    CANx->MCR &= (uint32_t)(~(CAN_MCR_SLEEP|CAN_MCR_INRQ));

    /* Wait the acknowledge */
    while (((CANx->MSR & CAN_MODE_MASK) != 0) && (timeout!=0))
    {
      timeout--;
    }
    if ((CANx->MSR & CAN_MODE_MASK) != 0)
    {
      status = CAN_ModeStatus_Failed;
    }
    else
    {
      status = CAN_ModeStatus_Success;
    }
  }
  else  if (CAN_OperatingMode == CAN_OperatingMode_Sleep)
  {
    /* Request Sleep mode */
    CANx->MCR = (uint32_t)((CANx->MCR & (uint32_t)(~(uint32_t)CAN_MCR_INRQ)) | CAN_MCR_SLEEP);

    /* Wait the acknowledge */
    while (((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_SLAK) && (timeout!=0))
    {
      timeout--;
    }
    if ((CANx->MSR & CAN_MODE_MASK) != CAN_MSR_SLAK)
    {
      status = CAN_ModeStatus_Failed;
    }
    else
    {
      status = CAN_ModeStatus_Success;
    }
  }
  else
  {
    status = CAN_ModeStatus_Failed;
  }

  return  (uint8_t) status;
}

/**
  * @brief  Enters the low power mode.
  * @param  CANx:   where x can be 1 or 2 to to select the CAN peripheral.
  * @retval status: CAN_Sleep_Ok if sleep entered, CAN_Sleep_Failed in an 
  *                 other case.
  */
uint8_t CAN_Sleep(CAN_TypeDef* CANx)
{
  uint8_t sleepstatus = CAN_Sleep_Failed;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
    
  /* Request Sleep mode */
   CANx->MCR = (((CANx->MCR) & (uint32_t)(~(uint32_t)CAN_MCR_INRQ)) | CAN_MCR_SLEEP);
   
  /* Sleep mode status */
  if ((CANx->MSR & (CAN_MSR_SLAK|CAN_MSR_INAK)) == CAN_MSR_SLAK)
  {
    /* Sleep mode not entered */
    sleepstatus =  CAN_Sleep_Ok;
  }
  /* return sleep mode status */
   return (uint8_t)sleepstatus;
}

/**
  * @brief  Wakes the CAN up.
  * @param  CANx:    where x can be 1 or 2 to to select the CAN peripheral.
  * @retval status:  CAN_WakeUp_Ok if sleep mode left, CAN_WakeUp_Failed in an 
  *                  other case.
  */
uint8_t CAN_WakeUp(CAN_TypeDef* CANx)
{
  uint32_t wait_slak = SLAK_TIMEOUT;
  uint8_t wakeupstatus = CAN_WakeUp_Failed;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
    
  /* Wake up request */
  CANx->MCR &= ~(uint32_t)CAN_MCR_SLEEP;
    
  /* Sleep mode status */
  while(((CANx->MSR & CAN_MSR_SLAK) == CAN_MSR_SLAK)&&(wait_slak!=0x00))
  {
   wait_slak--;
  }
  if((CANx->MSR & CAN_MSR_SLAK) != CAN_MSR_SLAK)
  {
   /* wake up done : Sleep mode exited */
    wakeupstatus = CAN_WakeUp_Ok;
  }
  /* return wakeup status */
  return (uint8_t)wakeupstatus;
}


/**
  * @brief  Returns the CANx's last error code (LEC).
  * @param  CANx:          where x can be 1 or 2 to to select the CAN peripheral.  
  * @retval CAN_ErrorCode: specifies the Error code : 
  *                        - CAN_ERRORCODE_NoErr            No Error  
  *                        - CAN_ERRORCODE_StuffErr         Stuff Error
  *                        - CAN_ERRORCODE_FormErr          Form Error
  *                        - CAN_ERRORCODE_ACKErr           Acknowledgment Error
  *                        - CAN_ERRORCODE_BitRecessiveErr  Bit Recessive Error
  *                        - CAN_ERRORCODE_BitDominantErr   Bit Dominant Error
  *                        - CAN_ERRORCODE_CRCErr           CRC Error
  *                        - CAN_ERRORCODE_SoftwareSetErr   Software Set Error  
  */
 
uint8_t CAN_GetLastErrorCode(CAN_TypeDef* CANx)
{
  uint8_t errorcode=0;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  
  /* Get the error code*/
  errorcode = (((uint8_t)CANx->ESR) & (uint8_t)CAN_ESR_LEC);
  
  /* Return the error code*/
  return errorcode;
}
/**
  * @brief  Returns the CANx Receive Error Counter (REC).
  * @note   In case of an error during reception, this counter is incremented 
  *         by 1 or by 8 depending on the error condition as defined by the CAN 
  *         standard. After every successful reception, the counter is 
  *         decremented by 1 or reset to 120 if its value was higher than 128. 
  *         When the counter value exceeds 127, the CAN controller enters the 
  *         error passive state.  
  * @param  CANx: where x can be 1 or 2 to to select the CAN peripheral.  
  * @retval CAN Receive Error Counter. 
  */
uint8_t CAN_GetReceiveErrorCounter(CAN_TypeDef* CANx)
{
  uint8_t counter=0;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  
  /* Get the Receive Error Counter*/
  counter = (uint8_t)((CANx->ESR & CAN_ESR_REC)>> 24);
  
  /* Return the Receive Error Counter*/
  return counter;
}


/**
  * @brief  Returns the LSB of the 9-bit CANx Transmit Error Counter(TEC).
  * @param  CANx:   where x can be 1 or 2 to to select the CAN peripheral.  
  * @retval LSB of the 9-bit CAN Transmit Error Counter. 
  */
uint8_t CAN_GetLSBTransmitErrorCounter(CAN_TypeDef* CANx)
{
  uint8_t counter=0;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  
  /* Get the LSB of the 9-bit CANx Transmit Error Counter(TEC) */
  counter = (uint8_t)((CANx->ESR & CAN_ESR_TEC)>> 16);
  
  /* Return the LSB of the 9-bit CANx Transmit Error Counter(TEC) */
  return counter;
}


/**
  * @brief  Enables or disables the specified CANx interrupts.
  * @param  CANx:   where x can be 1 or 2 to to select the CAN peripheral.
  * @param  CAN_IT: specifies the CAN interrupt sources to be enabled or disabled.
  *                 This parameter can be: 
  *                 - CAN_IT_TME, 
  *                 - CAN_IT_FMP0, 
  *                 - CAN_IT_FF0,
  *                 - CAN_IT_FOV0, 
  *                 - CAN_IT_FMP1, 
  *                 - CAN_IT_FF1,
  *                 - CAN_IT_FOV1, 
  *                 - CAN_IT_EWG, 
  *                 - CAN_IT_EPV,
  *                 - CAN_IT_LEC, 
  *                 - CAN_IT_ERR, 
  *                 - CAN_IT_WKU or 
  *                 - CAN_IT_SLK.
  * @param  NewState: new state of the CAN interrupts.
  *                   This parameter can be: ENABLE or DISABLE.
  * @retval None.
  */
void CAN_ITConfig(CAN_TypeDef* CANx, uint32_t CAN_IT, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_IT(CAN_IT));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected CANx interrupt */
    CANx->IER |= CAN_IT;
  }
  else
  {
    /* Disable the selected CANx interrupt */
    CANx->IER &= ~CAN_IT;
  }
}
/**
  * @brief  Checks whether the specified CAN flag is set or not.
  * @param  CANx:     where x can be 1 or 2 to to select the CAN peripheral.
  * @param  CAN_FLAG: specifies the flag to check.
  *                   This parameter can be one of the following flags: 
  *                  - CAN_FLAG_EWG
  *                  - CAN_FLAG_EPV 
  *                  - CAN_FLAG_BOF
  *                  - CAN_FLAG_RQCP0
  *                  - CAN_FLAG_RQCP1
  *                  - CAN_FLAG_RQCP2
  *                  - CAN_FLAG_FMP1   
  *                  - CAN_FLAG_FF1       
  *                  - CAN_FLAG_FOV1   
  *                  - CAN_FLAG_FMP0   
  *                  - CAN_FLAG_FF0       
  *                  - CAN_FLAG_FOV0   
  *                  - CAN_FLAG_WKU 
  *                  - CAN_FLAG_SLAK  
  *                  - CAN_FLAG_LEC       
  * @retval The new state of CAN_FLAG (SET or RESET).
  */
FlagStatus CAN_GetFlagStatus(CAN_TypeDef* CANx, uint32_t CAN_FLAG)
{
  FlagStatus bitstatus = RESET;
  
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_GET_FLAG(CAN_FLAG));
  

  if((CAN_FLAG & CAN_FLAGS_ESR) != (uint32_t)RESET)
  { 
    /* Check the status of the specified CAN flag */
    if ((CANx->ESR & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
    { 
      /* CAN_FLAG is set */
      bitstatus = SET;
    }
    else
    { 
      /* CAN_FLAG is reset */
      bitstatus = RESET;
    }
  }
  else if((CAN_FLAG & CAN_FLAGS_MSR) != (uint32_t)RESET)
  { 
    /* Check the status of the specified CAN flag */
    if ((CANx->MSR & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
    { 
      /* CAN_FLAG is set */
      bitstatus = SET;
    }
    else
    { 
      /* CAN_FLAG is reset */
      bitstatus = RESET;
    }
  }
  else if((CAN_FLAG & CAN_FLAGS_TSR) != (uint32_t)RESET)
  { 
    /* Check the status of the specified CAN flag */
    if ((CANx->TSR & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
    { 
      /* CAN_FLAG is set */
      bitstatus = SET;
    }
    else
    { 
      /* CAN_FLAG is reset */
      bitstatus = RESET;
    }
  }
  else if((CAN_FLAG & CAN_FLAGS_RF0R) != (uint32_t)RESET)
  { 
    /* Check the status of the specified CAN flag */
    if ((CANx->RF0R & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
    { 
      /* CAN_FLAG is set */
      bitstatus = SET;
    }
    else
    { 
      /* CAN_FLAG is reset */
      bitstatus = RESET;
    }
  }
  else /* If(CAN_FLAG & CAN_FLAGS_RF1R != (uint32_t)RESET) */
  { 
    /* Check the status of the specified CAN flag */
    if ((uint32_t)(CANx->RF1R & (CAN_FLAG & 0x000FFFFF)) != (uint32_t)RESET)
    { 
      /* CAN_FLAG is set */
      bitstatus = SET;
    }
    else
    { 
      /* CAN_FLAG is reset */
      bitstatus = RESET;
    }
  }
  /* Return the CAN_FLAG status */
  return  bitstatus;
}

/**
  * @brief  Clears the CAN's pending flags.
  * @param  CANx:     where x can be 1 or 2 to to select the CAN peripheral.
  * @param  CAN_FLAG: specifies the flag to clear.
  *                   This parameter can be one of the following flags: 
  *                    - CAN_FLAG_RQCP0
  *                    - CAN_FLAG_RQCP1
  *                    - CAN_FLAG_RQCP2
  *                    - CAN_FLAG_FF1       
  *                    - CAN_FLAG_FOV1   
  *                    - CAN_FLAG_FF0       
  *                    - CAN_FLAG_FOV0   
  *                    - CAN_FLAG_WKU   
  *                    - CAN_FLAG_SLAK    
  *                    - CAN_FLAG_LEC       
  * @retval None.
  */
void CAN_ClearFlag(CAN_TypeDef* CANx, uint32_t CAN_FLAG)
{
  uint32_t flagtmp=0;
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_CLEAR_FLAG(CAN_FLAG));
  
  if (CAN_FLAG == CAN_FLAG_LEC) /* ESR register */
  {
    /* Clear the selected CAN flags */
    CANx->ESR = (uint32_t)RESET;
  }
  else /* MSR or TSR or RF0R or RF1R */
  {
    flagtmp = CAN_FLAG & 0x000FFFFF;

    if ((CAN_FLAG & CAN_FLAGS_RF0R)!=(uint32_t)RESET)
    {
      /* Receive Flags */
      CANx->RF0R = (uint32_t)(flagtmp);
    }
    else if ((CAN_FLAG & CAN_FLAGS_RF1R)!=(uint32_t)RESET)
    {
      /* Receive Flags */
      CANx->RF1R = (uint32_t)(flagtmp);
    }
    else if ((CAN_FLAG & CAN_FLAGS_TSR)!=(uint32_t)RESET)
    {
      /* Transmit Flags */
      CANx->TSR = (uint32_t)(flagtmp);
    }
    else /* If((CAN_FLAG & CAN_FLAGS_MSR)!=(uint32_t)RESET) */
    {
      /* Operating mode Flags */
      CANx->MSR = (uint32_t)(flagtmp);
    }
  }
}

/**
  * @brief  Checks whether the specified CANx interrupt has occurred or not.
  * @param  CANx:    where x can be 1 or 2 to to select the CAN peripheral.
  * @param  CAN_IT:  specifies the CAN interrupt source to check.
  *                  This parameter can be one of the following flags: 
  *                 -  CAN_IT_TME               
  *                 -  CAN_IT_FMP0              
  *                 -  CAN_IT_FF0               
  *                 -  CAN_IT_FOV0              
  *                 -  CAN_IT_FMP1              
  *                 -  CAN_IT_FF1               
  *                 -  CAN_IT_FOV1              
  *                 -  CAN_IT_WKU  
  *                 -  CAN_IT_SLK  
  *                 -  CAN_IT_EWG    
  *                 -  CAN_IT_EPV    
  *                 -  CAN_IT_BOF    
  *                 -  CAN_IT_LEC    
  *                 -  CAN_IT_ERR 
  * @retval The current state of CAN_IT (SET or RESET).
  */
ITStatus CAN_GetITStatus(CAN_TypeDef* CANx, uint32_t CAN_IT)
{
  ITStatus itstatus = RESET;
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_IT(CAN_IT));
  
  /* check the enable interrupt bit */
 if((CANx->IER & CAN_IT) != RESET)
 {
   /* in case the Interrupt is enabled, .... */
    switch (CAN_IT)
    {
      case CAN_IT_TME:
               /* Check CAN_TSR_RQCPx bits */
	             itstatus = CheckITStatus(CANx->TSR, CAN_TSR_RQCP0|CAN_TSR_RQCP1|CAN_TSR_RQCP2);  
	      break;
      case CAN_IT_FMP0:
               /* Check CAN_RF0R_FMP0 bit */
	             itstatus = CheckITStatus(CANx->RF0R, CAN_RF0R_FMP0);  
	      break;
      case CAN_IT_FF0:
               /* Check CAN_RF0R_FULL0 bit */
               itstatus = CheckITStatus(CANx->RF0R, CAN_RF0R_FULL0);  
	      break;
      case CAN_IT_FOV0:
               /* Check CAN_RF0R_FOVR0 bit */
               itstatus = CheckITStatus(CANx->RF0R, CAN_RF0R_FOVR0);  
	      break;
      case CAN_IT_FMP1:
               /* Check CAN_RF1R_FMP1 bit */
               itstatus = CheckITStatus(CANx->RF1R, CAN_RF1R_FMP1);  
	      break;
      case CAN_IT_FF1:
               /* Check CAN_RF1R_FULL1 bit */
	             itstatus = CheckITStatus(CANx->RF1R, CAN_RF1R_FULL1);  
	      break;
      case CAN_IT_FOV1:
               /* Check CAN_RF1R_FOVR1 bit */
	             itstatus = CheckITStatus(CANx->RF1R, CAN_RF1R_FOVR1);  
	      break;
      case CAN_IT_WKU:
               /* Check CAN_MSR_WKUI bit */
               itstatus = CheckITStatus(CANx->MSR, CAN_MSR_WKUI);  
	      break;
      case CAN_IT_SLK:
               /* Check CAN_MSR_SLAKI bit */
	             itstatus = CheckITStatus(CANx->MSR, CAN_MSR_SLAKI);  
	      break;
      case CAN_IT_EWG:
               /* Check CAN_ESR_EWGF bit */
	             itstatus = CheckITStatus(CANx->ESR, CAN_ESR_EWGF);  
	      break;
      case CAN_IT_EPV:
               /* Check CAN_ESR_EPVF bit */
	             itstatus = CheckITStatus(CANx->ESR, CAN_ESR_EPVF);  
	      break;
      case CAN_IT_BOF:
               /* Check CAN_ESR_BOFF bit */
	             itstatus = CheckITStatus(CANx->ESR, CAN_ESR_BOFF);  
	      break;
      case CAN_IT_LEC:
               /* Check CAN_ESR_LEC bit */
	             itstatus = CheckITStatus(CANx->ESR, CAN_ESR_LEC);  
	      break;
      case CAN_IT_ERR:
               /* Check CAN_MSR_ERRI bit */ 
               itstatus = CheckITStatus(CANx->MSR, CAN_MSR_ERRI); 
	      break;
      default :
               /* in case of error, return RESET */
              itstatus = RESET;
              break;
    }
  }
  else
  {
   /* in case the Interrupt is not enabled, return RESET */
    itstatus  = RESET;
  }
  
  /* Return the CAN_IT status */
  return  itstatus;
}

/**
  * @brief  Clears the CANx's interrupt pending bits.
  * @param  CANx:    where x can be 1 or 2 to to select the CAN peripheral.
  * @param  CAN_IT: specifies the interrupt pending bit to clear.
  *                  -  CAN_IT_TME                     
  *                  -  CAN_IT_FF0               
  *                  -  CAN_IT_FOV0                     
  *                  -  CAN_IT_FF1               
  *                  -  CAN_IT_FOV1              
  *                  -  CAN_IT_WKU  
  *                  -  CAN_IT_SLK  
  *                  -  CAN_IT_EWG    
  *                  -  CAN_IT_EPV    
  *                  -  CAN_IT_BOF    
  *                  -  CAN_IT_LEC    
  *                  -  CAN_IT_ERR 
  * @retval None.
  */
void CAN_ClearITPendingBit(CAN_TypeDef* CANx, uint32_t CAN_IT)
{
  /* Check the parameters */
  assert_param(IS_CAN_ALL_PERIPH(CANx));
  assert_param(IS_CAN_CLEAR_IT(CAN_IT));

  switch (CAN_IT)
  {
      case CAN_IT_TME:
              /* Clear CAN_TSR_RQCPx (rc_w1)*/
	      CANx->TSR = CAN_TSR_RQCP0|CAN_TSR_RQCP1|CAN_TSR_RQCP2;  
	      break;
      case CAN_IT_FF0:
              /* Clear CAN_RF0R_FULL0 (rc_w1)*/
	      CANx->RF0R = CAN_RF0R_FULL0; 
	      break;
      case CAN_IT_FOV0:
              /* Clear CAN_RF0R_FOVR0 (rc_w1)*/
	      CANx->RF0R = CAN_RF0R_FOVR0; 
	      break;
      case CAN_IT_FF1:
              /* Clear CAN_RF1R_FULL1 (rc_w1)*/
	      CANx->RF1R = CAN_RF1R_FULL1;  
	      break;
      case CAN_IT_FOV1:
              /* Clear CAN_RF1R_FOVR1 (rc_w1)*/
	      CANx->RF1R = CAN_RF1R_FOVR1; 
	      break;
      case CAN_IT_WKU:
              /* Clear CAN_MSR_WKUI (rc_w1)*/
	      CANx->MSR = CAN_MSR_WKUI;  
	      break;
      case CAN_IT_SLK:
              /* Clear CAN_MSR_SLAKI (rc_w1)*/ 
	      CANx->MSR = CAN_MSR_SLAKI;   
	      break;
      case CAN_IT_EWG:
              /* Clear CAN_MSR_ERRI (rc_w1) */
	      CANx->MSR = CAN_MSR_ERRI;
              /* Note : the corresponding Flag is cleared by hardware depending 
                        of the CAN Bus status*/ 
	      break;
      case CAN_IT_EPV:
              /* Clear CAN_MSR_ERRI (rc_w1) */
	      CANx->MSR = CAN_MSR_ERRI; 
              /* Note : the corresponding Flag is cleared by hardware depending 
                        of the CAN Bus status*/
	      break;
      case CAN_IT_BOF:
              /* Clear CAN_MSR_ERRI (rc_w1) */ 
	      CANx->MSR = CAN_MSR_ERRI; 
              /* Note : the corresponding Flag is cleared by hardware depending 
                        of the CAN Bus status*/
	      break;
      case CAN_IT_LEC:
              /*  Clear LEC bits */
	      CANx->ESR = RESET; 
              /* Clear CAN_MSR_ERRI (rc_w1) */
	      CANx->MSR = CAN_MSR_ERRI; 
	      break;
      case CAN_IT_ERR:
              /*Clear LEC bits */
	      CANx->ESR = RESET; 
              /* Clear CAN_MSR_ERRI (rc_w1) */
	      CANx->MSR = CAN_MSR_ERRI; 
	      /* Note : BOFF, EPVF and EWGF Flags are cleared by hardware depending 
                  of the CAN Bus status*/
	      break;
      default :
	      break;
   }
}

/**
  * @brief  Checks whether the CAN interrupt has occurred or not.
  * @param  CAN_Reg: specifies the CAN interrupt register to check.
  * @param  It_Bit:  specifies the interrupt source bit to check.
  * @retval The new state of the CAN Interrupt (SET or RESET).
  */
static ITStatus CheckITStatus(uint32_t CAN_Reg, uint32_t It_Bit)
{
  ITStatus pendingbitstatus = RESET;
  
  if ((CAN_Reg & It_Bit) != (uint32_t)RESET)
  {
    /* CAN_IT is set */
    pendingbitstatus = SET;
  }
  else
  {
    /* CAN_IT is reset */
    pendingbitstatus = RESET;
  }
  return pendingbitstatus;
}


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
