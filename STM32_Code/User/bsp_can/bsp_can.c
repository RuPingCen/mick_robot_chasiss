/*********************************************************************************
�޸���Ұ��STM32�̵̳�����
CRP
2019-09-14

ʹ�÷�����
	 CAN_Config();   //����CANģ�� 
	 CAN_SetMsg();// ����Ҫͨ��CAN���͵���Ϣ
	 CAN_Transmit(CAN1, &TxMessage); // ������Ϣ ��ABCD��
	 
�жϷ�������
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  CAN_Receive(CAN1, CAN_FIFO0, &RxMessage); //�������ж�������
  //  �Ƚ�ID�������Ƿ�Ϊ0x1314��DCBA 
  if((RxMessage.ExtId==0x1314) && (RxMessage.IDE==CAN_ID_EXT)
     && (RxMessage.DLC==2) && ((RxMessage.Data[1]|RxMessage.Data[0]<<8)==0xDCBA))
  {
    flag = 0; 					       //���ճɹ�  
  }
  else
  {
    flag = 0xff; 					   //����ʧ��
  }
}

**********************************************************************************/
#include "stm32f10x.h"
#include "bsp_can.h" 


//CanRxMsg RxMessage;				 //���ջ�����

 

/*
 *  CAN ����źŵı�ʾ
 *  1:���Ե�ƽ   H2.5v - L2.5v = 0v
 *  0:���Ե�ƽ   H3.5v - L1.5v = 2v
 */

/*
 * ��������CAN_GPIO_Config
 * ����  ��CAN��GPIO ����,PB8�������룬PB9�������
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_GPIO_Config(void)
{
 	GPIO_InitTypeDef GPIO_InitStructure;
   	
  	/*����ʱ������*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

//  	/*IO����*/
//	GPIO_PinRemapConfig(GPIO_Remap1_CAN1, ENABLE);
//	/* Configure CAN pin: RX */									               // PB8
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	             // ��������
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//	
//    /* Configure CAN pin: TX */									               // PB9
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;		         // �����������
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;	//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);			//��ʼ��IO
	
}

/*
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_NVIC_Config(void)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
		/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	 	/*�ж�����*/
	NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;	   //CAN1 RX0�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		   //��ռ���ȼ�0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			   //�����ȼ�Ϊ0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Mode_Config(void)
{
   	CAN_InitTypeDef        CAN_InitStructure;
	 	/************************CANͨ�Ų�������**********************************/
	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CAN1);
	CAN_StructInit(&CAN_InitStructure);
	
	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			//��ʱ�䴥��ͨ��ģʽ  
	CAN_InitStructure.CAN_ABOM=DISABLE;			//����Զ����߹���	 
	CAN_InitStructure.CAN_AWUM=DISABLE;			//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;			//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;		 	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;			//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;		   //BTR-SJW ����ͬ����Ծ��� 1��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS1=CAN_BS1_3tq;		   //BTR-TS1 ʱ���1 ռ����3��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_5tq;		   //BTR-TS1 ʱ���2 ռ����5��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_Prescaler =4;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+5+3)/4=1Mbps
	
	 //   CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		   //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
   // CAN_InitStructure.CAN_BS1=CAN_BS1_6tq;		   //BTR-TS1 ʱ���1 ռ����6��ʱ�䵥Ԫ
   // CAN_InitStructure.CAN_BS2=CAN_BS2_3tq;		   //BTR-TS1 ʱ���2 ռ����3��ʱ�䵥Ԫ
   // CAN_InitStructure.CAN_Prescaler =4;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 36/(1+6+3)/4=0.9Mbps
		
	CAN_Init(CAN1, &CAN_InitStructure);
}

/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ����
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Filter_Config(void)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CAN��������ʼ��*/
	CAN_FilterInitStructure.CAN_FilterNumber=0;						//��������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//�����ڱ�ʶ������λģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//������λ��Ϊ����32λ��
	/* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */

	//CAN_FilterInitStructure.CAN_FilterIdHigh= (((u32)0x1314<<3)&0xFFFF0000)>>16;				//Ҫ���˵�ID��λ 
	//CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x1314<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫ���˵�ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;	//32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000;			//��������16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;			//��������16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;				//��������������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ�ܹ�����
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}


/*
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_Config(void)
{
  CAN_GPIO_Config();
  CAN_Mode_Config();
  CAN_Filter_Config();   
	CAN_NVIC_Config();
}


/*
 * ��������CAN_SetMsg
 * ����  ��CANͨ�ű�����������
 * ����  ����
 * ���  : ��
 * ����  ���ⲿ����
 */	 
void CAN_SetMsg(void)
{	  
	CanTxMsg TxMessage;			   //���ͻ�����
  //TxMessage.StdId=0x00;						 
  TxMessage.ExtId=0x1314;					 //ʹ�õ���չID
  TxMessage.IDE=CAN_ID_EXT;					 //��չģʽ
  TxMessage.RTR=CAN_RTR_DATA;				 //���͵�������
  TxMessage.DLC=2;							 //���ݳ���Ϊ2�ֽ�
  TxMessage.Data[0]=0xAB;
  TxMessage.Data[1]=0xCD;
	
	//CAN_Transmit(CAN1, &TxMessage); // ������Ϣ ��ABCD��
}




/**************************END OF FILE************************************/

