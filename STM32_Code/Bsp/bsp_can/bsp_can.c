/*********************************************************************************
�޸���Ұ��STM32�̵̳�����
CRP
2023-08-29

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
#include "stm32f4xx.h"
#include "bsp_can.h" 
#include "stm32f4xx_rcc.h"
#include "bsp_gpio.h"
#include "bsp_uart.h"

uint8_t data_num = 8;
uint8_t data_can[8];
extern __IO uint32_t flag;	


//CanRxMsg RxMessage;				 //���ջ�����

 

/*
 *  CAN ����źŵı�ʾ
 *  1:���Ե�ƽ   H2.5v - L2.5v = 0v
 *  0:���Ե�ƽ   H3.5v - L1.5v = 2v
 */

/*
 * ��������CAN_GPIO_Config
 * ����  ��CAN��GPIO ����,PB8�������룬PB9�������
 * ����  ��CAN1��CAN2
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_GPIO_Config(CAN_TypeDef* CANx)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	

    if(CANx == CAN1)
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1,ENABLE);//ʹ��CAN1ʱ��
        /* ʹ��GPIOʱ�� */
        RCC_AHB1PeriphClockCmd(CAN1_TX_GPIO_CLK|CAN1_RX_GPIO_CLK, ENABLE);
        
        /* �˿ڸ��� */
        GPIO_PinAFConfig(CAN1_TX_GPIO_PORT, CAN1_RX_SOURCE, CAN1_AF_PORT);
        GPIO_PinAFConfig(CAN1_RX_GPIO_PORT, CAN1_TX_SOURCE, CAN1_AF_PORT);
        
        /* ����CAN1 TX���� */
        GPIO_InitStructure.GPIO_Pin = CAN1_TX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(CAN1_TX_GPIO_PORT, &GPIO_InitStructure);

        /* ����CAN1 RX���� */
        GPIO_InitStructure.GPIO_Pin = CAN1_RX_PIN ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Init(CAN1_RX_GPIO_PORT, &GPIO_InitStructure);
    }
    else
    {
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2,ENABLE);//ʹ��CAN2ʱ��		
        /* ʹ��GPIOʱ�� */
        RCC_AHB1PeriphClockCmd(CAN2_TX_GPIO_CLK|CAN2_RX_GPIO_CLK, ENABLE);
        
        /* �˿ڸ��� */
        GPIO_PinAFConfig(CAN2_TX_GPIO_PORT, CAN2_RX_SOURCE, CAN2_AF_PORT);
        GPIO_PinAFConfig(CAN2_RX_GPIO_PORT, CAN2_TX_SOURCE, CAN2_AF_PORT);
        
        /* ����CAN2 TX���� */
        GPIO_InitStructure.GPIO_Pin = CAN2_TX_PIN;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
        GPIO_Init(CAN2_TX_GPIO_PORT, &GPIO_InitStructure);

        /* ����CAN2 RX���� */
        GPIO_InitStructure.GPIO_Pin = CAN2_RX_PIN ;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_Init(CAN2_RX_GPIO_PORT, &GPIO_InitStructure);
    }
}

/*
 * ��������CAN_NVIC_Config
 * ����  ��CAN��NVIC ����,��1���ȼ��飬0��0���ȼ�
 * ����  ��CAN1��CAN2
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_NVIC_Config(CAN_TypeDef* CANx)
{
   	NVIC_InitTypeDef NVIC_InitStructure;
	/* Configure one bit for preemption priority */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	/*�ж�����*/
    if(CANx == CAN1) 
	{
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		        //��ռ���ȼ�0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;			            //�����ȼ�Ϊ0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}
	else 
	{
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;		        //��ռ���ȼ�0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;			            //�����ȼ�Ϊ1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
	}	           
    
}

/*
 * ��������CAN_Mode_Config
 * ����  ��CAN��ģʽ ����
 * ����  ��CAN1��CAN2
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Mode_Config(CAN_TypeDef* CANx)
{
	CAN_InitTypeDef        CAN_InitStructure;
	/************************CANͨ�Ų�������**********************************/
	/* Enable CAN clock */
    if(CANx == CAN1)
    {
        RCC_APB1PeriphClockCmd(CAN1_CLK, ENABLE);
    }
    else
    {
        RCC_APB1PeriphClockCmd(CAN2_CLK, ENABLE);
    }
	/*CAN�Ĵ�����ʼ��*/
	CAN_DeInit(CANx);
	CAN_StructInit(&CAN_InitStructure);

	/*CAN��Ԫ��ʼ��*/
	CAN_InitStructure.CAN_TTCM=DISABLE;			   //MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ��
	CAN_InitStructure.CAN_ABOM=ENABLE;			   //MCR-ABOM  �Զ����߹��� 
	CAN_InitStructure.CAN_AWUM=ENABLE;			   //MCR-AWUM  ʹ���Զ�����ģʽ
	CAN_InitStructure.CAN_NART=DISABLE;			   //MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش�
	CAN_InitStructure.CAN_RFLM=DISABLE;			   //MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б���  
	CAN_InitStructure.CAN_TXFP=DISABLE;			   //MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� 
	CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;  //��������ģʽ
	CAN_InitStructure.CAN_SJW=CAN_SJW_2tq;		   //BTR-SJW ����ͬ����Ծ��� 2��ʱ�䵥Ԫ
	 
	/* ss=1 bs1=4 bs2=2 λʱ����Ϊ(1+4+2) �����ʼ�Ϊʱ������tq*(1+4+2)  */
	CAN_InitStructure.CAN_BS1=CAN_BS1_4tq;		   //BTR-TS1 ʱ���1 ռ����4��ʱ�䵥Ԫ
	CAN_InitStructure.CAN_BS2=CAN_BS2_2tq;		   //BTR-TS1 ʱ���2 ռ����2��ʱ�䵥Ԫ	
	
	/* CAN Baudrate = 1 MBps (1MBps��Ϊstm32��CAN�������) (CAN ʱ��Ƶ��Ϊ APB 1 = 42 MHz) */
	CAN_InitStructure.CAN_Prescaler =6;		   ////BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 42/(1+4+2)/6=1 Mbps
	CAN_Init(CANx, &CAN_InitStructure);
}


/*
 * ��������CAN_Filter_Config
 * ����  ��CAN�Ĺ����� ����
 * ����  ��CAN1��CAN2
 * ���  : ��
 * ����  ���ڲ�����
 */
static void CAN_Filter_Config(CAN_TypeDef* CANx)
{
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	/*CANɸѡ����ʼ��*/
    if(CANx == CAN1) {CAN_FilterInitStructure.CAN_FilterNumber=0;}
	else {CAN_FilterInitStructure.CAN_FilterNumber=14;}				//ɸѡ����14
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;	//����������ģʽ
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;	//ɸѡ��λ��Ϊ����32λ��
	/* ʹ��ɸѡ�������ձ�־�����ݽ��бȶ�ɸѡ����չID�������µľ����������ǵĻ��������FIFO0�� */

	CAN_FilterInitStructure.CAN_FilterIdHigh= ((((u32)0x580<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF0000)>>16;		//Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterIdLow= (((u32)0x580<<3)|CAN_ID_EXT|CAN_RTR_DATA)&0xFFFF; //Ҫɸѡ��ID��λ 
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh= 0x0000; //ɸѡ����16λÿλ����ƥ�� (1��ʾƥ�� 0��ʾ���Ը�λ)
	CAN_FilterInitStructure.CAN_FilterMaskIdLow= 0x0000;//ɸѡ����16λÿλ����ƥ��
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0 ;	 //ɸѡ����������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;			//ʹ��ɸѡ��
	CAN_FilterInit(&CAN_FilterInitStructure);
	/*CANͨ���ж�ʹ��*/
	CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);
}

/*0x580
 * ��������CAN_Config
 * ����  ����������CAN�Ĺ���
 * ����  ��CAN1��CAN2
 * ���  : ��
 * ����  ���ⲿ����
 */
void CAN_Config(CAN_TypeDef* CANx)
{
    CAN_GPIO_Config(CANx);          //GPIO����
    CAN_Mode_Config(CANx);          //CAN������
    CAN_Filter_Config(CANx);        //CAN����������
    CAN_NVIC_Config(CANx);          //CAN�ж�����
	 
}

 
 
/**************************END OF FILE************************************/

