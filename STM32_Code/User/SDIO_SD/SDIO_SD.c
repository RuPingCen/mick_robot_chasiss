/**********************************************************************************************           
  *          +-----------------------------------------------------------+
  *          |                     Pin assignment                        |
  *          +-----------------------------+---------------+-------------+
  *          |  STM32 SDIO Pins            |     SD        |    Pin      |
  *          +-----------------------------+---------------+-------------+
  *          |      SDIO D2                |   D2          |    1        |
  *          |      SDIO D3                |   D3          |    2        |
  *          |      SDIO CMD               |   CMD         |    3        |
  *          |                             |   VCC         |    4 (3.3 V)|
  *          |      SDIO CLK               |   CLK         |    5        |
  *          |                             |   GND         |    6 (0 V)  |
  *          |      SDIO D0                |   D0          |    7        |
  *          |      SDIO D1                |   D1          |    8        |  
  *          +-----------------------------+---------------+-------------+  
	*
	*         1�� //ע����������0xff��0x00    startaddr������512�������� 
	*         2��//��Ҫ������������ı�����    SD_Error Status = SD_OK;//���ڴ洢�õײ㺯�����ص�״̬
	*																					 extern SD_CardInfo SDCardInfo;//����Ҫ���ýṹ��SD_CardInfo ��ŵ�SD������Ϣʱ����
	*         3��//��Ҫ���жϺ�����д����δ��� 
	*							void SDIO_IRQHandler(void) //��SDIO_ITConfig(���������������sdio�ж�	�� ���ݴ������ʱ�����ж�
	*							{
	*			
	*								SD_ProcessIRQSrc();// Process All SDIO Interrupt Sources 
	*							}
	*         4������Ҫ�ı�DMA�жϵ����ȼ�   ������  SD_Init() �������޸�
	*
	*         ����������ײ�  ��û����ȫ��� Ŀǰֻ����˳�ʼ������  ����ʹ�ãģͣ����������ǿ顡������û�кܺõ���⣩
	*
	*  SD_Error SD_Init(void)//��ʼ��SD����ʹ�����ھ���״̬(׼����������) 
	*  SD_Error SD_Erase(uint32_t startaddr, uint32_t endaddr);// SD_Error: �ɹ���᷵��   SD_OK          
  *
//---------------------------------------------------------------------//
//---------------------����ռ�Ķ�����------------------------------------//	
 
	    SD_Error SD_ReadBlock(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize)//startaddr������512��������  
			SD_WaitReadOperation();  //�ȴ�DMA�������
			while(SD_GetStatus() != SD_TRANSFER_OK);// ��ѯ�����Ƿ����    ��ɺ�᷵��  SD_TRANSFER_OK
 
//---------------------------------------------------------------------//
//---------------------����ռ��д����------------------------------------//
	    SD_WriteBlock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize);  //startaddr������512��������  
			SD_WaitWriteOperation(); //�ȴ�DMA�������
			while(SD_GetStatus() != SD_TRANSFER_OK); // ��ѯ�����Ƿ����    ��ɺ�᷵��  SD_TRANSFER_OK	
//------------------------------------------------------------------------//
//---------------------���ռ�Ķ�����------------------------------------// 			
		 SD_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
		 SD_WaitReadOperation();
     while(SD_GetStatus() != SD_TRANSFER_OK);	
//------------------------------------------------------------------------//
//---------------------���ռ��д����------------------------------------// 
		
	   SD_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)	
     SD_WaitWriteOperation();//�ȴ�DMA�������
     while(SD_GetStatus() != SD_TRANSFER_OK);// ��ѯ�����Ƿ����    ��ɺ�᷵��  SD_TRANSFER_OK	

***********************************************************************************************/ 


#include "SDIO_SD.h"
#include "stm32f10x_sdio.h"  	
#include "bsp_uart.h"

SD_CardInfo SDCardInfo;	  //���ڴ洢������Ϣ��DSR��һ����  �ڳ�ʼ���������ú�SD������Ϣ���Զ��洢������ṹ����









/* Private macro -------------------------------------------------------------*/
/** 
  * @brief  SDIO Static flags, TimeOut, FIFO Address  
  */
#define NULL 0
#define SDIO_STATIC_FLAGS               ((uint32_t)0x000005FF)
#define SDIO_CMD0TIMEOUT                ((uint32_t)0x00010000)

/** 
  * @brief  Mask for errors Card Status R1 (OCR Register) 
  */
#define SD_OCR_ADDR_OUT_OF_RANGE        ((uint32_t)0x80000000)
#define SD_OCR_ADDR_MISALIGNED          ((uint32_t)0x40000000)
#define SD_OCR_BLOCK_LEN_ERR            ((uint32_t)0x20000000)
#define SD_OCR_ERASE_SEQ_ERR            ((uint32_t)0x10000000)
#define SD_OCR_BAD_ERASE_PARAM          ((uint32_t)0x08000000)
#define SD_OCR_WRITE_PROT_VIOLATION     ((uint32_t)0x04000000)
#define SD_OCR_LOCK_UNLOCK_FAILED       ((uint32_t)0x01000000)
#define SD_OCR_COM_CRC_FAILED           ((uint32_t)0x00800000)
#define SD_OCR_ILLEGAL_CMD              ((uint32_t)0x00400000)
#define SD_OCR_CARD_ECC_FAILED          ((uint32_t)0x00200000)
#define SD_OCR_CC_ERROR                 ((uint32_t)0x00100000)
#define SD_OCR_GENERAL_UNKNOWN_ERROR    ((uint32_t)0x00080000)
#define SD_OCR_STREAM_READ_UNDERRUN     ((uint32_t)0x00040000)
#define SD_OCR_STREAM_WRITE_OVERRUN     ((uint32_t)0x00020000)
#define SD_OCR_CID_CSD_OVERWRIETE       ((uint32_t)0x00010000)
#define SD_OCR_WP_ERASE_SKIP            ((uint32_t)0x00008000)
#define SD_OCR_CARD_ECC_DISABLED        ((uint32_t)0x00004000)
#define SD_OCR_ERASE_RESET              ((uint32_t)0x00002000)
#define SD_OCR_AKE_SEQ_ERROR            ((uint32_t)0x00000008)
#define SD_OCR_ERRORBITS                ((uint32_t)0xFDFFE008)

/** 
  * @brief  Masks for R6 Response 
  */
#define SD_R6_GENERAL_UNKNOWN_ERROR     ((uint32_t)0x00002000)
#define SD_R6_ILLEGAL_CMD               ((uint32_t)0x00004000)
#define SD_R6_COM_CRC_FAILED            ((uint32_t)0x00008000)

#define SD_VOLTAGE_WINDOW_SD            ((uint32_t)0x80100000)
#define SD_HIGH_CAPACITY                ((uint32_t)0x40000000)
#define SD_STD_CAPACITY                 ((uint32_t)0x00000000)
#define SD_CHECK_PATTERN                ((uint32_t)0x000001AA)//

#define SD_MAX_VOLT_TRIAL               ((uint32_t)0x0000FFFF)
#define SD_ALLZERO                      ((uint32_t)0x00000000)

#define SD_WIDE_BUS_SUPPORT             ((uint32_t)0x00040000)
#define SD_SINGLE_BUS_SUPPORT           ((uint32_t)0x00010000)
#define SD_CARD_LOCKED                  ((uint32_t)0x02000000)

#define SD_DATATIMEOUT                  ((uint32_t)0xFFFFFFFF)
#define SD_0TO7BITS                     ((uint32_t)0x000000FF)
#define SD_8TO15BITS                    ((uint32_t)0x0000FF00)
#define SD_16TO23BITS                   ((uint32_t)0x00FF0000)
#define SD_24TO31BITS                   ((uint32_t)0xFF000000)
#define SD_MAX_DATA_LENGTH              ((uint32_t)0x01FFFFFF)

#define SD_HALFFIFO                     ((uint32_t)0x00000008)
#define SD_HALFFIFOBYTES                ((uint32_t)0x00000020)

/** 
  * @brief  Command Class Supported 
  */
#define SD_CCCC_LOCK_UNLOCK             ((uint32_t)0x00000080)
#define SD_CCCC_WRITE_PROT              ((uint32_t)0x00000040)
#define SD_CCCC_ERASE                   ((uint32_t)0x00000020)

/** 
  * @brief  Following commands are SD Card Specific commands.
  *         SDIO_APP_CMD should be sent before sending these commands. 
  */
#define SDIO_SEND_IF_COND               ((uint32_t)0x00000008)
																



static uint32_t CardType =  SDIO_STD_CAPACITY_SD_CARD_V1_1;	//�洢�������ͣ��Ȱ�����ʼ��Ϊ1.1Э��Ŀ�
static uint32_t CSD_Tab[4], CID_Tab[4], RCA = 0;//�洢CSD��CID���Ĵ����Ϳ���Ե�ַ

static uint8_t SDSTATUS_Tab[16]; //�洢��״̬����CSR��һ����

__IO uint32_t StopCondition = 0; //����ֹͣ�������ı�־
__IO SD_Error TransferError = SD_OK; //���ڴ洢�����󣬳�ʼ��Ϊ����״̬
__IO uint32_t TransferEnd = 0;	   //���ڱ�־�����Ƿ���������жϷ������е���





/*����sdio��ʼ���Ľṹ��*/
SDIO_InitTypeDef SDIO_InitStructure;
SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
SDIO_DataInitTypeDef SDIO_DataInitStructure;   


/* Private function prototypes -----------------------------------------------*/
static SD_Error CmdError(void);
static SD_Error CmdResp1Error(uint8_t cmd);
static SD_Error CmdResp7Error(void);
static SD_Error CmdResp3Error(void);
static SD_Error CmdResp2Error(void);
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca);

static SD_Error SDEnWideBus(FunctionalState NewState);
static SD_Error IsCardProgramming(uint8_t *pstatus);
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr);

static void GPIO_Configuration(void);
static uint32_t SD_DMAEndOfTransferStatus(void);
static void SD_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize);
static void SD_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize);

uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes);
  
/* Private functions ---------------------------------------------------------*/

/*
 * ��������SD_DeInit
 * ����  ����λSDIO�˿�
 * ����  ����
 * ���  ����
 */
void SD_DeInit(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;  
  
  SDIO_ClockCmd(DISABLE);/*!< Disable SDIO Clock */
  SDIO_SetPowerState(SDIO_PowerState_OFF);/*!< Set Power State to OFF */
  SDIO_DeInit();//SDIOģ�麯��/*!< DeInitializes the SDIO peripheral */
    
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, DISABLE);/*!< Disable the SDIO AHB Clock */

  /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; /*!< Configure PD.02 CMD line */
  GPIO_Init(GPIOD, &GPIO_InitStructure);
}

/*
 * ��������NVIC_Configuration
 * ����  ��SDIO ���ȼ�����Ϊ������ȼ���
 * ����  ����
 * ���  ����
 */
void NVIC_Configuration(void)//��ռ���ȼ�Ϊ0   ��Ӧ���ȼ�Ϊ4
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 4;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}

/*
 * ��������GPIO_Configuration
 * ����  ����ʼ��SDIO�õ������ţ�����ʱ�ӡ�
 * ����  ����
 * ���  ����
 * ����  ���ڲ�����
 */
static void GPIO_Configuration(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  /*!< GPIOC and GPIOD Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD , ENABLE);

  /*!< Configure PC.08, PC.09, PC.10, PC.11, PC.12 pin: D0, D1, D2, D3, CLK pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  /*!< Configure PD.02 CMD line */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_Init(GPIOD, &GPIO_InitStructure); 

  /*!< Enable the SDIO AHB Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_SDIO, ENABLE);

  /*!< Enable the DMA2 Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
}


















/**
  * @brief  Returns the DMA End Of Transfer Status.
  * @param  None
  * @retval DMA SDIO Channel Status.
  */
uint32_t SD_DMAEndOfTransferStatus(void)
{
  return (uint32_t)DMA_GetFlagStatus(DMA2_FLAG_TC4);   //Channel4 transfer complete flag. 

}


 /*
  * ��������SD_DMA_RxConfig
  * ����  ��ΪSDIO������������DMA2��ͨ��4������
  * ����  ��BufferDST������װ�����ݵı���ָ��
 	*	      : BufferSize��	��������С
  * ���  ����
  */
void SD_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize)
{
  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);//���DMA��־λ

  /*!< DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);	//SDIOΪ����ͨ��

  /*!< DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;  //�����ַ��fifo
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferDST; //Ŀ���ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;	//����Ϊԭ��ַ
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;  //1/4�����С
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//ʹ�������ַ������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	  //ʹ�ܴ洢Ŀ���ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;  //�������ݴ�СΪ�֣�32λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;	//�������ݴ�СΪ�֣�32λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;			   //��ѭ����ѭ��ģʽ��Ҫ����adc��
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	 //ͨ�����ȼ���
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;			 //�� �洢�����洢��ģʽ
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /*!< DMA2 Channel4 enable */			   //������dma�жϣ�
  DMA_Cmd(DMA2_Channel4, ENABLE); 
}

 /*
 * ��������SD_DMA_RxConfig
 * ����  ��ΪSDIO������������DMA2��ͨ��4������
 * ����  ��BufferDST��װ�������ݵı���ָ��
 		   BufferSize��	��������С
 * ���  ����
 */
void SD_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize)
{

  DMA_InitTypeDef DMA_InitStructure;

  DMA_ClearFlag(DMA2_FLAG_TC4 | DMA2_FLAG_TE4 | DMA2_FLAG_HT4 | DMA2_FLAG_GL4);

  /*!< DMA2 Channel4 disable */
  DMA_Cmd(DMA2_Channel4, DISABLE);

  /*!< DMA2 Channel4 Config */
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)BufferSRC;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;//����Ϊд��Ŀ��
  DMA_InitStructure.DMA_BufferSize = BufferSize / 4;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;	//�����ַ������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA2_Channel4, &DMA_InitStructure);

  /*!< DMA2 Channel4 enable */
  DMA_Cmd(DMA2_Channel4, ENABLE);  
}






/**
  * @brief  Gets the cuurent sd card data transfer status.
  * @param  None
  * @retval SDTransferState: Data Transfer state.
  *   This value can be: 
  *        - SD_TRANSFER_OK: No data transfer is acting
  *        - SD_TRANSFER_BUSY: Data transfer is acting
  */
SDTransferState SD_GetStatus(void)
{
  SDCardState cardstate =  SD_CARD_TRANSFER;

  cardstate = SD_GetState();
  
  if (cardstate == SD_CARD_TRANSFER)
  {
    return(SD_TRANSFER_OK);
  }
  else if(cardstate == SD_CARD_ERROR)
  {
    return (SD_TRANSFER_ERROR);
  }
  else
  {
    return(SD_TRANSFER_BUSY);
  }
}

/**
  * @brief  Returns the current card's state.
  * @param  None
  * @retval SDCardState: SD Card Error or SD Card Current State.
  */
SDCardState SD_GetState(void)
{
  uint32_t resp1 = 0;   

    if (SD_SendStatus(&resp1) != SD_OK)
    {
      return SD_CARD_ERROR;
    }
    else
    {
      return (SDCardState)((resp1 >> 9) & 0x0F);
    } 
}







/**
  * @brief  Enables wide bus opeartion for the requeseted card if supported by 
  *         card.
  * @param  WideMode: Specifies the SD card wide bus mode. 
  *   This parameter can be one of the following values:
  *     @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
  *     @arg SDIO_BusWide_4b: 4-bit data transfer
  *     @arg SDIO_BusWide_1b: 1-bit data transfer
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_GetCardStatus(SD_CardStatus *cardstatus)
{
  SD_Error errorstatus = SD_OK;
  uint8_t tmp = 0;

  errorstatus = SD_SendSDStatus((uint32_t *)SDSTATUS_Tab);

  if (errorstatus  != SD_OK)
  {
    return(errorstatus);
  }

  /*!< Byte 0 */
  tmp = (uint8_t)((SDSTATUS_Tab[0] & 0xC0) >> 6);
  cardstatus->DAT_BUS_WIDTH = tmp;

  /*!< Byte 0 */
  tmp = (uint8_t)((SDSTATUS_Tab[0] & 0x20) >> 5);
  cardstatus->SECURED_MODE = tmp;

  /*!< Byte 2 */
  tmp = (uint8_t)((SDSTATUS_Tab[2] & 0xFF));
  cardstatus->SD_CARD_TYPE = tmp << 8;

  /*!< Byte 3 */
  tmp = (uint8_t)((SDSTATUS_Tab[3] & 0xFF));
  cardstatus->SD_CARD_TYPE |= tmp;

  /*!< Byte 4 */
  tmp = (uint8_t)(SDSTATUS_Tab[4] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA = tmp << 24;

  /*!< Byte 5 */
  tmp = (uint8_t)(SDSTATUS_Tab[5] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 16;

  /*!< Byte 6 */
  tmp = (uint8_t)(SDSTATUS_Tab[6] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 8;

  /*!< Byte 7 */
  tmp = (uint8_t)(SDSTATUS_Tab[7] & 0xFF);
  cardstatus->SIZE_OF_PROTECTED_AREA |= tmp;

  /*!< Byte 8 */
  tmp = (uint8_t)((SDSTATUS_Tab[8] & 0xFF));
  cardstatus->SPEED_CLASS = tmp;

  /*!< Byte 9 */
  tmp = (uint8_t)((SDSTATUS_Tab[9] & 0xFF));
  cardstatus->PERFORMANCE_MOVE = tmp;

  /*!< Byte 10 */
  tmp = (uint8_t)((SDSTATUS_Tab[10] & 0xF0) >> 4);
  cardstatus->AU_SIZE = tmp;

  /*!< Byte 11 */
  tmp = (uint8_t)(SDSTATUS_Tab[11] & 0xFF);
  cardstatus->ERASE_SIZE = tmp << 8;

  /*!< Byte 12 */
  tmp = (uint8_t)(SDSTATUS_Tab[12] & 0xFF);
  cardstatus->ERASE_SIZE |= tmp;

  /*!< Byte 13 */
  tmp = (uint8_t)((SDSTATUS_Tab[13] & 0xFC) >> 2);
  cardstatus->ERASE_TIMEOUT = tmp;

  /*!< Byte 13 */
  tmp = (uint8_t)((SDSTATUS_Tab[13] & 0x3));
  cardstatus->ERASE_OFFSET = tmp;
 
  return(errorstatus);
}





/**
  * @brief  Allows to read one block from a specified address in a card. The Data
  *         transfer can be managed by DMA mode or Polling mode. 
  * @note   This operation should be followed by two functions to check if the 
  *         DMA Controller and SD Card status.
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controller has finished all data transfer.
  *          - SD_GetStatus(): to check that the SD Card has finished the 
  *            data transfer and it is ready for data.            
  * @param  readbuff: pointer to the buffer that will contain the received data
  * @param  ReadAddr: Address from where data are to be read.  
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_ReadBlock(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize)
{
  SD_Error errorstatus = SD_OK;
#if defined (SD_POLLING_MODE) 
  uint32_t count = 0, *tempbuff = (uint32_t *)readbuff;
#endif

  TransferError = SD_OK;
  TransferEnd = 0;	 //�����������λ�����жϷ�����1
  StopCondition = 0;  //��ô�õģ�
  
  SDIO->DCTRL = 0x0;

  
  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    ReadAddr /= 512;
  }
  /*******************add��û����һ�����׿�����DMA�����*************************************/
  /* Set Block Size for Card��cmd16,
	 * ����sdsc���������������ÿ��С��
	 * ����sdhc�������СΪ512�ֽڣ�����cmd16Ӱ�� 
	 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (SD_OK != errorstatus)
  {
    return(errorstatus);
  }
 /*********************************************************************************/
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
  SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);

  /*!< Send CMD17 READ_SINGLE_BLOCK */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)ReadAddr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

#if defined (SD_POLLING_MODE)  
  /*!< In case of single block transfer, no need of stop transfer at all.*/
  /*!< Polling mode */
  while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
  {
    if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
    {
      for (count = 0; count < 8; count++)
      {
        *(tempbuff + count) = SDIO_ReadData();
      }
      tempbuff += 8;
    }
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }
  while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
  {
    *tempbuff = SDIO_ReadData();
    tempbuff++;
  }
  
  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

#elif defined (SD_DMA_MODE)
    SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
    SDIO_DMACmd(ENABLE);
    SD_DMA_RxConfig((uint32_t *)readbuff, BlockSize);
#endif

  return(errorstatus);
}

/**
  * @brief  Allows to read blocks from a specified address  in a card.  The Data
  *         transfer can be managed by DMA mode or Polling mode. //������ģʽ
  * @note   This operation should be followed by two functions to check if the 
  *         DMA Controller and SD Card status.	   //dmaģʽʱҪ����������������
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controller has finished all data transfer. 
  *          - SD_GetStatus(): to check that the SD Card has finished the 
  *            data transfer and it is ready for data.   
  * @param  readbuff: pointer to the buffer that will contain the received data.
  * @param  ReadAddr: Address from where data are to be read.
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @param  NumberOfBlocks: number of blocks to be read.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error errorstatus = SD_OK;
  TransferError = SD_OK;
  TransferEnd = 0;
  StopCondition = 1;
	
  SDIO->DCTRL = 0x0;	 //��λ���ݿ��ƼĴ���

  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)//sdhc���ĵ�ַ�Կ�Ϊ��λ��ÿ��512�ֽ�
  {
    BlockSize = 512;
    ReadAddr /= 512;
  }

  /*!< Set Block Size for Card��cmd16,����sdsc���������������ÿ��С������sdhc�������СΪ512�ֽڣ�����cmd16Ӱ�� */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (SD_OK != errorstatus)
  {
    return(errorstatus);
  }
    
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;	 //�ȴ���ʱ����
  SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;	 //���ڿ����ݴ��䣬���ݳ��ȼĴ����е���ֵ���������ݿ鳤��(��SDIO_DCTRL)�ı���
  SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4; //ֱ���ò�����á�����SDIO_DataBlockSize_512b
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;//���䷽��
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block; //����ģʽ
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;	//��������״̬��
  SDIO_DataConfig(&SDIO_DataInitStructure);

  /*!< Send CMD18 READ_MULT_BLOCK with argument data address */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)ReadAddr;	//��ʼ��ַ
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short; //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_READ_MULT_BLOCK);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);  //�������ݴ�������ж� ��Data end (data counter, SDIDCOUNT, is zero) interrupt 
  SDIO_DMACmd(ENABLE); //ʹ��dma��ʽ
  SD_DMA_RxConfig((uint32_t *)readbuff, (NumberOfBlocks * BlockSize));//����DMA����

  return(errorstatus);
}

/**
  * @brief  This function waits until the SDIO DMA data transfer is finished. 
  *         This function should be called after SDIO_ReadMultiBlocks() function
  *         to insure that all data sent by the card are already transferred by 
  *         the DMA controller.        
  * @param  None.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_WaitReadOperation(void)
{
  SD_Error errorstatus = SD_OK;
		  //�ȴ�dma�������
  while ((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
  {}

  if (TransferError != SD_OK)
  {
    return(TransferError);
  }

  return(errorstatus);
}

/**
  * @brief  Allows to write one block starting from a specified address in a card.
  *         The Data transfer can be managed by DMA mode or Polling mode.
  * @note   This operation should be followed by two functions to check if the 
  *         DMA Controller and SD Card status.
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controller has finished all data transfer.
  *          - SD_GetStatus(): to check that the SD Card has finished the 
  *            data transfer and it is ready for data.      
  * @param  writebuff: pointer to the buffer that contain the data to be transferred.
  * @param  WriteAddr: Address from where data are to be read.   
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_WriteBlock(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize)
{
  SD_Error errorstatus = SD_OK;

#if defined (SD_POLLING_MODE)
  uint32_t bytestransferred = 0, count = 0, restwords = 0;
  uint32_t *tempbuff = (uint32_t *)writebuff;
#endif

  TransferError = SD_OK;
  TransferEnd = 0;
  StopCondition = 0;
  
  SDIO->DCTRL = 0x0;


  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    WriteAddr /= 512;
  }

	/*-------------- add , û����һ�����׿�����DMA����� -------------------*/
	/* Set Block Size for Card��cmd16,
	 * ����sdsc���������������ÿ��С��
	 * ����sdhc�������СΪ512�ֽڣ�����cmd16Ӱ�� 
	 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (SD_OK != errorstatus)
  {
    return(errorstatus);
  }
 /*********************************************************************************/
  
  /*!< Send CMD24 WRITE_SINGLE_BLOCK */
  SDIO_CmdInitStructure.SDIO_Argument = WriteAddr;	  //д���ַ
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;	 //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
	
	//����sdio��д���ݼĴ���
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
  SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4;  //���ô˲�������SDIO_DataBlockSize_512b
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;//д���ݣ�
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;	 //��������ͨ��״̬��
  SDIO_DataConfig(&SDIO_DataInitStructure);

  /*!< In case of single data block transfer no need of stop command at all */
#if defined (SD_POLLING_MODE) //��ͨģʽ
  while (!(SDIO->STA & (SDIO_FLAG_DBCKEND | SDIO_FLAG_TXUNDERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR)))
  {
    if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET)
    {
      if ((512 - bytestransferred) < 32)
      {
        restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4) : (( 512 -  bytestransferred) / 4 + 1);
        for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4)
        {
          SDIO_WriteData(*tempbuff);
        }
      }
      else
      {
        for (count = 0; count < 8; count++)
        {
          SDIO_WriteData(*(tempbuff + count));
        }
        tempbuff += 8;
        bytestransferred += 32;
      }
    }
  }
  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
    errorstatus = SD_TX_UNDERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }
#elif defined (SD_DMA_MODE)	//dmaģʽ
  SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);  //���ݴ�������ж�
  SD_DMA_TxConfig((uint32_t *)writebuff, BlockSize); //����dma����rx����
  SDIO_DMACmd(ENABLE);	 //	ʹ��sdio��dma����
#endif

  return(errorstatus);
}

/**
  * @brief  Allows to write blocks starting from a specified address in a card.
  *         The Data transfer can be managed by DMA mode only. 
  * @note   This operation should be followed by two functions to check if the 
  *         DMA Controller and SD Card status.
  *          - SD_ReadWaitOperation(): this function insure that the DMA
  *            controller has finished all data transfer.
  *          - SD_GetStatus(): to check that the SD Card has finished the 
  *            data transfer and it is ready for data.     
  * @param  WriteAddr: Address from where data are to be read.
  * @param  writebuff: pointer to the buffer that contain the data to be transferred.
  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
  * @param  NumberOfBlocks: number of blocks to be written.
  * @retval SD_Error: SD Card Error code.
  */
  
  /*
 * ��������SD_WriteMultiBlocks
 * ����  �����������ʼ��ַ��ʼ����д�������ݿ飬
 		  ֻ����DMAģʽ��ʹ���������
	ע�⣺�������������һ��Ҫ����
			SD_WaitWriteOperation�������ȴ�DMA�������
			��	SD_GetStatus() ��⿨��SDIO��FIFO���Ƿ��Ѿ���ɴ���
 * ����  �� 
		  * @param  WriteAddr: Address from where data are to be read.
		  * @param  writebuff: pointer to the buffer that contain the data to be transferred.
		  * @param  BlockSize: the SD card Data block size. The Block size should be 512.
		  * @param  NumberOfBlocks: number of blocks to be written.
 * ���  ��SD��������
 */
SD_Error SD_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteAddr, uint16_t BlockSize, uint32_t NumberOfBlocks)
{
  SD_Error errorstatus = SD_OK;
  __IO uint32_t count = 0;

  TransferError = SD_OK;
  TransferEnd = 0;
  StopCondition = 1;
  
  SDIO->DCTRL = 0x0;

  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    BlockSize = 512;
    WriteAddr /= 512;
  }

    /*******************add��û����һ�����׿�����DMA�����*************************************/
    /*!< Set Block Size for Card��cmd16,����sdsc���������������ÿ��С������sdhc�������СΪ512�ֽڣ�����cmd16Ӱ�� */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;   //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (SD_OK != errorstatus)
  {
    return(errorstatus);
  }
 /*********************************************************************************/

  /*!< To improve performance  */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (RCA << 16);
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;	// cmd55
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);


  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  /*!< To improve performance *///  pre-erased���ڶ��д��ʱ�ɷ��ʹ��������Ԥ����
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)NumberOfBlocks;  //����Ϊ��Ҫд��Ŀ���Ŀ
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;	 //cmd23
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }


  /*!< Send CMD25 WRITE_MULT_BLOCK with argument data address */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)WriteAddr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_WRITE_MULT_BLOCK);

  if (SD_OK != errorstatus)
  {
    return(errorstatus);
  }

  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
  SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);

  SDIO_ITConfig(SDIO_IT_DATAEND, ENABLE);
  SDIO_DMACmd(ENABLE);    
  SD_DMA_TxConfig((uint32_t *)writebuff, (NumberOfBlocks * BlockSize));

  return(errorstatus);
}

/**
  * @brief  This function waits until the SDIO DMA data transfer is finished. 
  *         This function should be called after SDIO_WriteBlock() and
  *         SDIO_WriteMultiBlocks() function to insure that all data sent by the 
  *         card are already transferred by the DMA controller.        
  * @param  None.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_WaitWriteOperation(void)
{
  SD_Error errorstatus = SD_OK;
		  //�ȴ�dma�Ƿ������
  while ((SD_DMAEndOfTransferStatus() == RESET) && (TransferEnd == 0) && (TransferError == SD_OK))
  {}

  if (TransferError != SD_OK)
  {
    return(TransferError);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/**
  * @brief  Gets the cuurent data transfer state.
  * @param  None
  * @retval SDTransferState: Data Transfer state.
  *   This value can be: 
  *        - SD_TRANSFER_OK: No data transfer is acting
  *        - SD_TRANSFER_BUSY: Data transfer is acting
  */
SDTransferState SD_GetTransferState(void)
{
  if (SDIO->STA & (SDIO_FLAG_TXACT | SDIO_FLAG_RXACT))
  {
    return(SD_TRANSFER_BUSY);
  }
  else
  {
    return(SD_TRANSFER_OK);
  }
}

/**
  * @brief  Aborts an ongoing data transfer.
  * @param  None
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_StopTransfer(void)
{
  SD_Error errorstatus = SD_OK;

  /*!< Send CMD12 STOP_TRANSMISSION  */
  SDIO->ARG = 0x0;
  SDIO->CMD = 0x44C;
  errorstatus = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);

  return(errorstatus);
}

/**
  * @brief  Allows to erase memory area specified for the given card.
  * @param  startaddr: the start address.
  * @param  endaddr: the end address.
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_Erase(uint32_t startaddr, uint32_t endaddr)
{
  SD_Error errorstatus = SD_OK;
  uint32_t delay = 0;
  __IO uint32_t maxdelay = 0;
  uint8_t cardstate = 0;

  /*!< Check if the card coomnd class supports erase command */
  if (((CSD_Tab[1] >> 20) & SD_CCCC_ERASE) == 0)
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }

  maxdelay = 120000 / ((SDIO->CLKCR & 0xFF) + 2);//��ʱ������ʱ�ӷ�Ƶ����������

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)	  //��������
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)//sdhc����ΪʲôҪ /512�����2.0Э��page52
  {	//��sdhc������ַ����Ϊ���ַ��ÿ��512�ֽڣ�sdsc����ַΪ�ֽڵ�ַ
    startaddr /= 512;	  
    endaddr /= 512;
  }
  
  /*!< According to sd-card spec 1.0 ERASE_GROUP_START (CMD32) and erase_group_end(CMD33) */
  if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {
    /*!< Send CMD32 SD_ERASE_GRP_START with argument as addr  */
    SDIO_CmdInitStructure.SDIO_Argument = startaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_ERASE_GRP_START;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;	//R1
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SD_ERASE_GRP_START);
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }

    /*!< Send CMD33 SD_ERASE_GRP_END with argument as addr  */
    SDIO_CmdInitStructure.SDIO_Argument = endaddr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_ERASE_GRP_END;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SD_ERASE_GRP_END);
    if (errorstatus != SD_OK)
    {
      return(errorstatus);
    }
  }

  /*!< Send CMD38 ERASE */
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ERASE;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_ERASE);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  for (delay = 0; delay < maxdelay; delay++)
  {}

  /*!< Wait till the card is in programming state */
  errorstatus = IsCardProgramming(&cardstate);

  while ((errorstatus == SD_OK) && ((SD_CARD_PROGRAMMING == cardstate) || (SD_CARD_RECEIVING == cardstate)))
  {
    errorstatus = IsCardProgramming(&cardstate);
  }

  return(errorstatus);
}

/**
  * @brief  Returns the current card's status.
  * @param  pcardstatus: pointer to the buffer that will contain the SD card 
  *         status (Card Status register).
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_SendStatus(uint32_t *pcardstatus)
{
  SD_Error errorstatus = SD_OK;

  SDIO->ARG = (uint32_t) RCA << 16;
  SDIO->CMD = 0x44D;
  
  errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  *pcardstatus = SDIO->RESP1;
  return(errorstatus);
}

/**
  * @brief  Returns the current SD card's status.
  * @param  psdstatus: pointer to the buffer that will contain the SD card status 
  *         (SD Status register).
  * @retval SD_Error: SD Card Error code.
  */
SD_Error SD_SendSDStatus(uint32_t *psdstatus)
{
  SD_Error errorstatus = SD_OK;
  uint32_t count = 0;

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED)
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /*!< Set block size for card if it is not equal to current block size for card. */
  SDIO_CmdInitStructure.SDIO_Argument = 64;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /*!< CMD55 */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 64;
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_64b;
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);

  /*!< Send ACMD13 SD_APP_STAUS  with argument as card's RCA.*/
  SDIO_CmdInitStructure.SDIO_Argument = 0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_STAUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);
  errorstatus = CmdResp1Error(SD_CMD_SD_APP_STAUS);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR)))
  {
    if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET)
    {
      for (count = 0; count < 8; count++)
      {
        *(psdstatus + count) = SDIO_ReadData();
      }
      psdstatus += 8;
    }
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)
  {
    *psdstatus = SDIO_ReadData();
    psdstatus++;
  }

  /*!< Clear all the static status flags*/
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

/*
 * ��������SD_ProcessIRQSrc
 * ����  �����ݴ�������ж�
 * ����  ����		 
 * ���  ��SD��������
 */
SD_Error SD_ProcessIRQSrc(void)
{
  if (StopCondition == 1)  //ʲôʱ����1�ˣ�
  {
    SDIO->ARG = 0x0;   //��������Ĵ���
    SDIO->CMD = 0x44C;	  // ����Ĵ����� 0100 	01 	 	001100
						//						[7:6]  	[5:0]
						//				CPSMEN  WAITRESP CMDINDEX
						//		��������״̬��	����Ӧ   cmd12 STOP_ TRANSMISSION						
    TransferError = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);
  }
  else
  {
    TransferError = SD_OK;
  }
  SDIO_ClearITPendingBit(SDIO_IT_DATAEND); //���ж�
  SDIO_ITConfig(SDIO_IT_DATAEND, DISABLE); //�ر�sdio�ж�ʹ��
  TransferEnd = 1;
  return(TransferError);
}


  

  /*
 * ��������SDEnWideBus
 * ����  ��ʹ�ܻ�ر�SDIO��4bitģʽ
 * ����  ����״̬	ENABLE ��DISABLE
 * ���  ��SD��������
 */
static SD_Error SDEnWideBus(FunctionalState NewState)
{
  SD_Error errorstatus = SD_OK;

  uint32_t scr[2] = {0, 0};

  if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED) //��⿨�Ƿ�������
  {
    errorstatus = SD_LOCK_UNLOCK_FAILED;
    return(errorstatus);
  }

  /*!< Get SCR Register */
  errorstatus = FindSCR(RCA, scr);//��ȡscr�Ĵ������ݵ�scr������

  if (errorstatus != SD_OK)		  //degug,crc����scr��ȡ������ֵ
  {
    return(errorstatus);
  }

  /*!< If wide bus operation to be enabled */
  if (NewState == ENABLE)
  {
    /*!< If requested card supports wide bus operation */
    if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO)  //�жϿ��Ƿ�֧��4λ��ʽ
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 2 for wide bus mode */
	  /*����4bitģʽ������acmd6*/
      SDIO_CmdInitStructure.SDIO_Argument = 0x2;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }   /*!< If wide bus operation to be disabled */
  else
  {
    /*!< If requested card supports 1 bit mode operation */
    if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO)
    {
      /*!< Send CMD55 APP_CMD with argument as card's RCA.*/
      SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);


      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      /*!< Send ACMD6 APP_CMD with argument as 0 for single bus mode */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }

      return(errorstatus);
    }
    else
    {
      errorstatus = SD_REQUEST_NOT_APPLICABLE;
      return(errorstatus);
    }
  }
}


  /*
 * ��������IsCardProgramming
 * ����  �����SD���ǲ������ڽ����ڲ���д����
 * ����  ������װ��SD state״̬��ָ��
 * ���  ��SD��������
 */
static SD_Error IsCardProgramming(uint8_t *pstatus)
{
  SD_Error errorstatus = SD_OK;
  __IO uint32_t respR1 = 0, status = 0;

	 /*cmd13�ÿ����Ϳ�״̬�Ĵ������洢��m3��λ��Ϊsdio_sta�Ĵ���*/
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16; //����Ե�ַ����
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  status = SDIO->STA;
  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }
	  /*һϵ�е�״̬�ж�*/
  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  status = (uint32_t)SDIO_GetCommandResponse();

  /*!< Check response received is of desired command */
  if (status != SD_CMD_SEND_STATUS)
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);


  /*!< We have received response, retrieve it for analysis  */
  respR1 = SDIO_GetResponse(SDIO_RESP1);

  /*!< Find out card status */
  *pstatus = (uint8_t) ((respR1 >> 9) & 0x0000000F);   //status[12:9] :cardstate 

  if ((respR1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
  {
    return(errorstatus);
  }

  if (respR1 & SD_OCR_ADDR_OUT_OF_RANGE)
  {
    return(SD_ADDR_OUT_OF_RANGE);
  }

  if (respR1 & SD_OCR_ADDR_MISALIGNED)
  {
    return(SD_ADDR_MISALIGNED);
  }

  if (respR1 & SD_OCR_BLOCK_LEN_ERR)
  {
    return(SD_BLOCK_LEN_ERR);
  }

  if (respR1 & SD_OCR_ERASE_SEQ_ERR)
  {
    return(SD_ERASE_SEQ_ERR);
  }

  if (respR1 & SD_OCR_BAD_ERASE_PARAM)
  {
    return(SD_BAD_ERASE_PARAM);
  }

  if (respR1 & SD_OCR_WRITE_PROT_VIOLATION)
  {
    return(SD_WRITE_PROT_VIOLATION);
  }

  if (respR1 & SD_OCR_LOCK_UNLOCK_FAILED)
  {
    return(SD_LOCK_UNLOCK_FAILED);
  }

  if (respR1 & SD_OCR_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  if (respR1 & SD_OCR_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (respR1 & SD_OCR_CARD_ECC_FAILED)
  {
    return(SD_CARD_ECC_FAILED);
  }

  if (respR1 & SD_OCR_CC_ERROR)
  {
    return(SD_CC_ERROR);
  }

  if (respR1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (respR1 & SD_OCR_STREAM_READ_UNDERRUN)
  {
    return(SD_STREAM_READ_UNDERRUN);
  }

  if (respR1 & SD_OCR_STREAM_WRITE_OVERRUN)
  {
    return(SD_STREAM_WRITE_OVERRUN);
  }

  if (respR1 & SD_OCR_CID_CSD_OVERWRIETE)
  {
    return(SD_CID_CSD_OVERWRITE);
  }

  if (respR1 & SD_OCR_WP_ERASE_SKIP)
  {
    return(SD_WP_ERASE_SKIP);
  }

  if (respR1 & SD_OCR_CARD_ECC_DISABLED)
  {
    return(SD_CARD_ECC_DISABLED);
  }

  if (respR1 & SD_OCR_ERASE_RESET)
  {
    return(SD_ERASE_RESET);
  }

  if (respR1 & SD_OCR_AKE_SEQ_ERROR)
  {
    return(SD_AKE_SEQ_ERROR);
  }

  return(errorstatus);
}


  /*
 * ��������FindSCR
 * ����  ����ȡSD����SCR�Ĵ���������
 * ����  ��RCA����Ե�ַ
 		   pscr	����װ��SCR���ݵ�ָ��
 * ���  ��SD��������
 */
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr)
{ 

  uint32_t index = 0;
  SD_Error errorstatus = SD_OK;
  uint32_t tempscr[2] = {0, 0};	

  /*!< Set Block Size To 8 Bytes */ 
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;	 //���С�����sdhc�����޷��ı���С��	//ԭ����8
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN; //	 cmd16
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }

  /*!< Send CMD55 APP_CMD with argument as card's RCA */
  SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) RCA << 16; 
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }
  /*�������ݽ��ռĴ���*/
  SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
  SDIO_DataInitStructure.SDIO_DataLength = 8;  //8byte,64λ
  SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b  ;  //���С8byte 
  SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
  SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
  SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
  SDIO_DataConfig(&SDIO_DataInitStructure);			 

  /*!< Send ACMD51 SD_APP_SEND_SCR with argument as 0 */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_SEND_SCR;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  //r1
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);

  if (errorstatus != SD_OK)
  {
    return(errorstatus);
  }			   
   
  /*�ȴ��������� */
  /*������Щ�����ѭ��*/																						  
 /*�������	  //����crcʧ��		//���ݳ�ʱ	  //�ѽ������ݿ飬crc���ɹ�	//û���������������ϼ�⵽��ʼ�ź�*/
 while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND| SDIO_FLAG_STBITERR)))
   {	   		
   if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET)	//���յ��������Ƿ����
	{  	
     	 	*(tempscr + index) = SDIO_ReadData();  
			   index++;	

		/*   //add������ڹٷ�Դ��û�м��ж�            */		     
		  	if(index > 1 ) 
				break;
    }
	
  }

  if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
    errorstatus = SD_DATA_TIMEOUT;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
    errorstatus = SD_DATA_CRC_FAIL;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
    errorstatus = SD_RX_OVERRUN;
    return(errorstatus);
  }
  else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET)
  {
    SDIO_ClearFlag(SDIO_FLAG_STBITERR);
    errorstatus = SD_START_BIT_ERR;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

  *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

  return(errorstatus);
}


/**
  * @brief  Converts the number of bytes in power of two and returns the power.
  * @param  NumberOfBytes: number of bytes.
  * @retval None
  */
uint8_t convert_from_bytes_to_power_of_two(uint16_t NumberOfBytes)
{
  uint8_t count = 0;

  while (NumberOfBytes != 1)
  {
    NumberOfBytes >>= 1;
    count++;
  }
  return(count);
}




















/*************************************************************************
*  �������ƣ�Display_SD_Information
*  ����˵����ͨ�����ڴ�ӡSD������Ϣ   ����ʼ�����ɹ��򲻴�ӡ
*  ����˵������
*             
*  �������أ���
*  �޸�ʱ�䣺2014-12-25
*  ��    ע��CRP   
*************************************************************************/
 void Display_SD_Information(void)
 {
	  UART_send_string(USART1," \r\n\n\tSD����Ϣ\n");//�ַ�������
    UART_send_string(USART1," \r\n CardType is �� ");//�ַ�������
	 
	  if(SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1)
		{
        UART_send_string(USART1,"SDIO_STD_CAPACITY_SD_CARD_V1_1");//�ַ�������
    }
		else if(SDCardInfo.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0)
		{ 
        UART_send_string(USART1,"SDIO_STD_CAPACITY_SD_CARD_V2_0");//�ַ�������
    }
		else if(SDCardInfo.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
		{ 
        UART_send_string(USART1,"SDIO_HIGH_CAPACITY_SD_CARD");//�ַ�������
    }
		else if(SDCardInfo.CardType == SDIO_MULTIMEDIA_CARD)
		{ 
        UART_send_string(USART1,"SDIO_MULTIMEDIA_CARD");//�ַ�������
    }
		else if(SDCardInfo.CardType == SDIO_SECURE_DIGITAL_IO_CARD)
		{ 
        UART_send_string(USART1,"SDIO_SECURE_DIGITAL_IO_CARD");//�ַ�������
    }	
		else if(SDCardInfo.CardType == SDIO_HIGH_SPEED_MULTIMEDIA_CARD)
		{ 
        UART_send_string(USART1,"SDIO_HIGH_SPEED_MULTIMEDIA_CARD");//�ַ�������
    }	
		else if(SDCardInfo.CardType == SDIO_SECURE_DIGITAL_IO_COMBO_CARD)
		{ 
        UART_send_string(USART1,"SDIO_SECURE_DIGITAL_IO_COMBO_CARD");//�ַ�������
    }
		else if(SDCardInfo.CardType == SDIO_HIGH_CAPACITY_MMC_CARD)
		{ 
        UART_send_string(USART1,"SDIO_HIGH_CAPACITY_MMC_CARD");//�ַ�������
    }
		else ;
		
		
	  UART_send_string(USART1," \r\n CardBlockSize is :  ");//�ַ�������
	  UART_send_data(USART1,SDCardInfo.CardBlockSize);  // 0~4294967296	 
		
		UART_send_string(USART1," \r\n CardCapacity is �� ");//�ַ�������
	  UART_send_data(USART1,SDCardInfo.CardCapacity );  // 0~4294967296
 
    UART_send_string(USART1," \r\n RCA is :  "); 
		UART_send_data(USART1,SDCardInfo.RCA);  // 0~4294967296	 	  
	  UART_send_string(USART1," \r\n ManufacturerID is : "); 
	  UART_send_data(USART1,SDCardInfo.SD_cid.ManufacturerID);  // 0~4294967296	 
		UART_send_string(USART1," \r\n\n"); 

}




//-------------------------------SD����ʼ����غ���---------------------------------------------------//
//----------------------------------------------------------------------------------------------------//


/*
 * ��������SD_Init
 * ����  ����ʼ��SD����ʹ�����ھ���״̬(׼����������)
 * ����  ����
 * ���  ��-SD_Error SD���������
 *         �ɹ�ʱ��Ϊ SD_OK
 * ����  ���ⲿ����
 */
SD_Error SD_Init(void) 
{
	static char SD_timers=0;
  SD_Error errorstatus = SD_OK;/*����SD_Error״̬*/  
	if(SD_timers==0)
	{
      	SD_DeInit();//SDIOģ��˿ڸ�λ  ---���Բ�����
	
				NVIC_Configuration();/* �жϳ�ʼ�� */
				GPIO_Configuration();/* SDIO ����ײ����ų�ʼ�� */
				SDIO_DeInit(); /*��SDIO�����мĴ������и�λ*/ 
  }

 
	
  errorstatus = SD_PowerON(); /*�ϵ粢���п�ʶ�����̣�ȷ�Ͽ��Ĳ�����ѹ  */ 
  if (errorstatus != SD_OK)/*����ϵ磬ʶ�𲻳ɹ������ء���Ӧ��ʱ������ */
  {  
    return(errorstatus);	 /*!< CMD Response TimeOut (wait for CMDSENT flag) */
  }
	
	
  errorstatus = SD_InitializeCards();  /*��ʶ��ɹ������п���ʼ��    */
  if (errorstatus != SD_OK)	  //ʧ�ܷ���
  {   
    return(errorstatus); /*!< CMD Response TimeOut (wait for CMDSENT flag) */
  }

  /* ����SDIO����
   * �ϵ�ʶ�𣬿���ʼ������ɺ󣬽������ݴ���ģʽ����߶�д�ٶ�
   * �ٶ�������24MҪ����bypassģʽ 
   */
  
  /* SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_TRANSFER_CLK_DIV) */  
  SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV;
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;/*�����زɼ����� */	
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;/* ʱ��Ƶ��������24M,Ҫ������ģʽ */ 		
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;/* �������˹��ܣ������߿���ʱ�ر�sd_clkʱ�� */ 	
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;/* ��ʱ���ó�1bitģʽ */	
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;/* Ӳ����������������FIFO���ܽ��з��ͺͽ�������ʱ�����ݴ�����ͣ */ 	
  SDIO_Init(&SDIO_InitStructure);
  
  if (errorstatus == SD_OK)
  {   
     errorstatus = SD_GetCardInfo(&SDCardInfo); /* ��ȡSD���ľ�����Ϣ  ����ڸõײ�ĺ����--SDCardInfo�ṹ����*/	
  }
  if (errorstatus == SD_OK)
  {    
    errorstatus = SD_SelectDeselect((uint32_t) (SDCardInfo.RCA << 16));	/* ͨ��cmd7  ,rcaѡ��Ҫ�����Ŀ� */  
  }
	
  if (errorstatus == SD_OK)
  {			
    errorstatus = SD_EnableWideBusOperation(SDIO_BusWide_4b);/* ���Ϊ����߶�д������4bitsģʽ */
  } 
	if(SD_timers==0)
	{
      	Display_SD_Information();
		    SD_timers=1;
  }	
  
  return(errorstatus);
}

/*
 * ��������SD_PowerON
 * ����  ��ȷ��SD���Ĺ�����ѹ�����ÿ���ʱ��
 * ����  ����
 * ���  ��-SD_Error SD���������
 *         �ɹ�ʱ��Ϊ SD_OK
 * ����  ���� SD_Init() ����
 */
SD_Error SD_PowerON(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t response = 0, count = 0, validvoltage = 0;
  uint32_t SDType = SD_STD_CAPACITY;
	
/********************************************************************************************************/
  /* �ϵ��ʼ�� 
   * ����SDIO������
   * SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_INIT_CLK_DIV)   
   * ��ʼ��ʱ��ʱ�Ӳ��ܴ���400KHz
   */
/* HCLK = 72MHz, SDIOCLK = 72MHz, SDIO_CK = HCLK/(178 + 2) = 400 KHz */
  SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;	
  SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;		
  SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;/* ��ʹ��bypassģʽ��ֱ����HCLK���з�Ƶ�õ�SDIO_CK ����24Mʱ��ʹ��bypassģʽ*/	
  SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;/* ����ʱ���ر�ʱ�ӵ�Դ */  	
  SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;/* ��ʼ����ʱ����ʱ�Ȱ����������ó�1�� */	
  SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;/* ʹ��Ӳ�������� */
  SDIO_Init(&SDIO_InitStructure);
  SDIO_SetPowerState(SDIO_PowerState_ON); /* ����SDIO����ĵ�Դ */  
  SDIO_ClockCmd(ENABLE);  /* ʹ�� SDIO ʱ�� */
	
/********************************************************************************************************/   
  /* ���淢��һϵ������,��ʼ��ʶ������
   * CMD0: GO_IDLE_STATE(��λ����SD���������״̬) 
   * û����Ӧ 
	 */
  SDIO_CmdInitStructure.SDIO_Argument = 0x0;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;/* û����Ӧ */	
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;/* �رյȴ��ж� */	
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable; /* ��CPSM�ڿ�ʼ��������֮ǰ�ȴ����ݴ������ */
  SDIO_SendCommand(&SDIO_CmdInitStructure);	  		
	
	
  errorstatus = CmdError();/* ����Ƿ�ɹ�����cmd0 */
  if (errorstatus != SD_OK)//�����ʱ�᷵��	SDIO_CMD0TIMEOUT  ��֮���� SD_OK
  {     
    return(errorstatus);
  }
	
/********************************************************************************************************/
  /* Send CMD8 to verify SD card interface operating condition
	 *          
   * Argument: - [31:12]: Reserved (shall be set to '0')
   *           - [11:8] : Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
   *           - [7:0]  : Check Pattern (recommended 0xAA) 
	 */ 
  SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;/* ���յ�����sd�᷵��������� */
  SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;//CMD8   CMD Response: R7 
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;	 
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;			 				
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp7Error();  /*����Ƿ���յ�����*/
	
	
  if (errorstatus == SD_OK)	  	/* ����Ӧ��card��ѭsdЭ��2.0�汾 */
  {	
   // printf( " \r\n SD Card 2.0  \r\n " );				
    CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0;/* SD Card 2.0 ���Ȱ��������sdsc���͵Ŀ� */				
    SDType = SD_HIGH_CAPACITY;	/* �����������ACMD41�Ĳ���������ѯ����sdsc������sdhc�� */
  }
  else	/* ����Ӧ��˵����1.x�Ļ�mmc�Ŀ� */
  {
   	// printf( " \r\n SD Card 1.0  \r\n " );	  
    SDIO_CmdInitStructure.SDIO_Argument = 0x00; /* ������ CMD55 */
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
  }
	

  SDIO_CmdInitStructure.SDIO_Argument = 0x00;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;//����cmd55�����ڼ����sd������mmc�������ǲ�֧�ֵĿ�
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);		
  errorstatus = CmdResp1Error(SD_CMD_APP_CMD);	/* �Ƿ���Ӧ��û��Ӧ����mmc��֧�ֵĿ� */
	
	
/********************************************************************************************************/
  /* If errorstatus is Command TimeOut, it is a MMC card 
   * If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
   * or SD card 1.x 
	 */
	
  if (errorstatus == SD_OK)	//��Ӧ��cmd55����sd��������Ϊ1.x,����Ϊ2.0
  {
  	/*���濪ʼѭ���ط���sdio֧�ֵĵ�ѹ��Χ��ѭ��һ������*/

    /* SD CARD
     * Send ACMD41 SD_APP_OP_COND with Argument 0x80100000 
		 */
    while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL))
    {	 
			/* �ڷ���ACMD����ǰ��Ҫ���򿨷���CMD55 
       * SEND CMD55 APP_CMD with RCA as 0 
			 */
      SDIO_CmdInitStructure.SDIO_Argument = 0x00;
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;	  
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
			
      if (errorstatus != SD_OK)
      {
        return(errorstatus);
      }
			
			/* ACMD41
			 * ���������֧�ֵĵ�ѹ��Χ��HCSλ��ɣ�HCSλ��һ�����ֿ���SDSC����SDHC
			 * 0:SDSC
			 * 1:SDHC
       * ��Ӧ��R3,��Ӧ����OCR�Ĵ���			
			 */			
      SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType; // 0x80100000 | SDType	  
      SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;//  ACMD41 command
      SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;  
      SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
      SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
      SDIO_SendCommand(&SDIO_CmdInitStructure);

      errorstatus = CmdResp3Error();
			
      if (errorstatus != SD_OK)
      {
        return(errorstatus); 
      }
      response = SDIO_GetResponse(SDIO_RESP1);//��ȡ���Ĵ�������״̬
      validvoltage = (((response >> 31) == 1) ? 1 : 0);	/* ��ȡ����ocr�Ĵ�����pwr_upλ�����Ƿ��ѹ�����������ѹ */
      count++;			  /* ����ѭ������ */
    }
		
    if (count >= SD_MAX_VOLT_TRIAL)					 /* ѭ����ⳬ��һ��������û�ϵ� */
    {
      errorstatus = SD_INVALID_VOLTRANGE;	   /* SDIO��֧��card�Ĺ����ѹ */
      return(errorstatus);
    }
		
		/*��鿨������Ϣ�е�HCSλ*/
		/* �ж�ocr�е�ccsλ �������sdsc����ִ���������� */
    if (response &= SD_HIGH_CAPACITY)       
    {
      CardType = SDIO_HIGH_CAPACITY_SD_CARD; /* �ѿ����ʹӳ�ʼ����sdsc�͸�Ϊsdhc�� */
    }

  }/* else MMC Card */

  return(errorstatus);		
}

/*
 * ��������SD_PowerOFF
 * ����  ���ص�SDIO������ź�
 * ����  ����
 * ���  ��-SD_Error SD���������
 *         �ɹ�ʱ��Ϊ SD_OK
 * ����  ���ⲿ����
 */
SD_Error SD_PowerOFF(void)
{
  SD_Error errorstatus = SD_OK;
  SDIO_SetPowerState(SDIO_PowerState_OFF);/*!< Set Power State to OFF */
  return(errorstatus);
}

  /*
 * ��������SD_InitializeCards
 * ����  ����ʼ�����еĿ����ߵ������������״̬
 * ����  ����
 * ���  ��-SD_Error SD���������
 *         �ɹ�ʱ��Ϊ SD_OK
 * ����  ���� SD_Init() ���ã��ڵ���power_on�����ϵ翨ʶ����Ϻ󣬵��ô˺������п���ʼ��
 */
SD_Error SD_InitializeCards(void)
{
  SD_Error errorstatus = SD_OK;
  uint16_t rca = 0x01;

  if (SDIO_GetPowerState() == SDIO_PowerState_OFF)
  {
    errorstatus = SD_REQUEST_NOT_APPLICABLE;
    return(errorstatus);
  }
	
	/* �жϿ������� */
  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
    /* Send CMD2 ALL_SEND_CID 
		 * ��Ӧ��R2����ӦCID�Ĵ���
		 */
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
		
		
    CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);	/* �����ص�CID��Ϣ�洢���� */
    CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
		
  }
/********************************************************************************************************/
  if (   (SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) 
		   ||(SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) 
	     ||(SDIO_SECURE_DIGITAL_IO_COMBO_CARD == CardType)
       ||(SDIO_HIGH_CAPACITY_SD_CARD == CardType) )	 /* ʹ�õ���2.0�Ŀ� */
  {
    /* Send CMD3 SET_REL_ADDR with argument 0 
     * SD Card publishes its RCA.
     * ��Ӧ��R6����ӦRCA�Ĵ���		
		 */
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;		
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;		
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
		
		/* �ѽ��յ��Ŀ���Ե�ַ������ */
    errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);	

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }
  }
/********************************************************************************************************/
  if (SDIO_SECURE_DIGITAL_IO_CARD != CardType)
  {
     RCA = rca;

    /* Send CMD9 SEND_CSD with argument as card's RCA 
		 * ��Ӧ:R2  ��Ӧ�Ĵ���CSD(Card-Specific Data)
		 */
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(rca << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_CSD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp2Error();

    if (SD_OK != errorstatus)
    {
      return(errorstatus);
    }

    CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
    CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
    CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
    CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
  }
/********************************************************************************************************/	
	/*ȫ������ʼ���ɹ� */
  errorstatus = SD_OK; 

  return(errorstatus);
}

/*
 * ��������SD_GetCardInfo
 * ����  ����ȡSD���ľ�����Ϣ
 * ����  ��-cardinfo ָ��SD_CardInfo�ṹ���ָ��
 *         ����ṹ���������SD���ľ�����Ϣ
 * ���  ��-SD_Error SD���������
 *         �ɹ�ʱ��Ϊ SD_OK
 * ����  ���ⲿ����
 */
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo)
{
  SD_Error errorstatus = SD_OK;
  uint8_t tmp = 0;

  cardinfo->CardType = (uint8_t)CardType;
  cardinfo->RCA = (uint16_t)RCA;

  /*!< Byte 0 */
  tmp = (uint8_t)((CSD_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
  cardinfo->SD_csd.Reserved1 = tmp & 0x03;

  /*!< Byte 1 */
  tmp = (uint8_t)((CSD_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.TAAC = tmp;

  /*!< Byte 2 */
  tmp = (uint8_t)((CSD_Tab[0] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.NSAC = tmp;

  /*!< Byte 3 */
  tmp = (uint8_t)(CSD_Tab[0] & 0x000000FF);
  cardinfo->SD_csd.MaxBusClkFrec = tmp;

  /*!< Byte 4 */
  tmp = (uint8_t)((CSD_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_csd.CardComdClasses = tmp << 4;

  /*!< Byte 5 */
  tmp = (uint8_t)((CSD_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
  cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

  /*!< Byte 6 */
  tmp = (uint8_t)((CSD_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.Reserved2 = 0; /*!< Reserved */

  if ((CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0))
  {
    cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

    /*!< Byte 7 */
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

    /*!< Byte 8 */
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

    cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
    cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

    /*!< Byte 9 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
    cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;
    /*!< Byte 10 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;
    
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
    cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
    cardinfo->CardBlockSize = 512;//Ĭ���������� 512 Bytes
    cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.RdBlockLen));
  }
  else if (CardType == SDIO_HIGH_CAPACITY_SD_CARD)
  {
    /*!< Byte 7 */
    tmp = (uint8_t)(CSD_Tab[1] & 0x000000FF);
    cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

    /*!< Byte 8 */
    tmp = (uint8_t)((CSD_Tab[2] & 0xFF000000) >> 24);

    cardinfo->SD_csd.DeviceSize |= (tmp << 8);

    /*!< Byte 9 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x00FF0000) >> 16);

    cardinfo->SD_csd.DeviceSize |= (tmp);

    /*!< Byte 10 */
    tmp = (uint8_t)((CSD_Tab[2] & 0x0000FF00) >> 8);
    
    cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 ;
    cardinfo->CardBlockSize = 512;    
  }


  cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

  /*!< Byte 11 */
  tmp = (uint8_t)(CSD_Tab[2] & 0x000000FF);
  cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
  cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

  /*!< Byte 12 */
  tmp = (uint8_t)((CSD_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
  cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
  cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

  /*!< Byte 13 */
  tmp = (uint8_t)((CSD_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
  cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.Reserved3 = 0;
  cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

  /*!< Byte 14 */
  tmp = (uint8_t)((CSD_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
  cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
  cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
  cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
  cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
  cardinfo->SD_csd.ECC = (tmp & 0x03);

  /*!< Byte 15 */
  tmp = (uint8_t)(CSD_Tab[3] & 0x000000FF);
  cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_csd.Reserved4 = 1;


  /*!< Byte 0 */
  tmp = (uint8_t)((CID_Tab[0] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ManufacturerID = tmp;

  /*!< Byte 1 */
  tmp = (uint8_t)((CID_Tab[0] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.OEM_AppliID = tmp << 8;

  /*!< Byte 2 */
  tmp = (uint8_t)((CID_Tab[0] & 0x000000FF00) >> 8);
  cardinfo->SD_cid.OEM_AppliID |= tmp;

  /*!< Byte 3 */
  tmp = (uint8_t)(CID_Tab[0] & 0x000000FF);
  cardinfo->SD_cid.ProdName1 = tmp << 24;

  /*!< Byte 4 */
  tmp = (uint8_t)((CID_Tab[1] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdName1 |= tmp << 16;

  /*!< Byte 5 */
  tmp = (uint8_t)((CID_Tab[1] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdName1 |= tmp << 8;

  /*!< Byte 6 */
  tmp = (uint8_t)((CID_Tab[1] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdName1 |= tmp;

  /*!< Byte 7 */
  tmp = (uint8_t)(CID_Tab[1] & 0x000000FF);
  cardinfo->SD_cid.ProdName2 = tmp;

  /*!< Byte 8 */
  tmp = (uint8_t)((CID_Tab[2] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdRev = tmp;

  /*!< Byte 9 */
  tmp = (uint8_t)((CID_Tab[2] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.ProdSN = tmp << 24;

  /*!< Byte 10 */
  tmp = (uint8_t)((CID_Tab[2] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ProdSN |= tmp << 16;

  /*!< Byte 11 */
  tmp = (uint8_t)(CID_Tab[2] & 0x000000FF);
  cardinfo->SD_cid.ProdSN |= tmp << 8;

  /*!< Byte 12 */
  tmp = (uint8_t)((CID_Tab[3] & 0xFF000000) >> 24);
  cardinfo->SD_cid.ProdSN |= tmp;

  /*!< Byte 13 */
  tmp = (uint8_t)((CID_Tab[3] & 0x00FF0000) >> 16);
  cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
  cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

  /*!< Byte 14 */
  tmp = (uint8_t)((CID_Tab[3] & 0x0000FF00) >> 8);
  cardinfo->SD_cid.ManufactDate |= tmp;

  /*!< Byte 15 */
  tmp = (uint8_t)(CID_Tab[3] & 0x000000FF);
  cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
  cardinfo->SD_cid.Reserved2 = 1;
  
  return(errorstatus);
}
/*
 * ��������SD_SelectDeselect
 * ����  ������cmd7��ѡ����Ե�ַΪaddr�Ŀ���ȡ��ѡ��������
 *   		���addr = 0,��ȡ��ѡ�����еĿ�
 * ����  ��-addr ѡ�񿨵ĵ�ַ
 * ���  ��-SD_Error SD���������
 *         �ɹ�ʱ��Ϊ SD_OK
 * ����  ���ⲿ����
 */	
SD_Error SD_SelectDeselect(uint32_t addr)
{
  SD_Error errorstatus = SD_OK;

  /*!< Send CMD7 SDIO_SEL_DESEL_CARD */
  SDIO_CmdInitStructure.SDIO_Argument =  addr;
  SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
  SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
  SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
  SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
  SDIO_SendCommand(&SDIO_CmdInitStructure);

  errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);

  return(errorstatus);
}
/*
 * ��������SD_EnableWideBusOperation
 * ����  �����ÿ������ݿ��(���ÿ����Ƿ�֧��)
 * ����  ��-WideMode ָ��SD���������߿�
 *         �������������
 *         @arg SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
 *         @arg SDIO_BusWide_4b: 4-bit data transfer
 *         @arg SDIO_BusWide_1b: 1-bit data transfer (Ĭ��)
 * ���  ��-SD_Error SD���������
 *         �ɹ�ʱ��Ϊ SD_OK
 * ����  ���ⲿ����
 */
SD_Error SD_EnableWideBusOperation(uint32_t WideMode)
{
  SD_Error errorstatus = SD_OK;

  /*!< MMC Card doesn't support this feature */
  if (SDIO_MULTIMEDIA_CARD == CardType)
  {
    errorstatus = SD_UNSUPPORTED_FEATURE;
    return(errorstatus);
  }
  else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == CardType))
  {														 
    if (SDIO_BusWide_8b == WideMode)   //2.0 sd��֧��8bits
    {
      errorstatus = SD_UNSUPPORTED_FEATURE;
      return(errorstatus);
    }
    else if (SDIO_BusWide_4b == WideMode)//4������ģʽ
    {
      errorstatus = SDEnWideBus(ENABLE);//ʹ��acmd6�������߿�ȣ����ÿ��Ĵ��䷽ʽ

      if (SD_OK == errorstatus)
      {
        /*!< Configure the SDIO peripheral */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;	//���������stm32��sdio�Ĵ��䷽ʽ ���л�ģʽ����ӿ���sdio����Ӧ��
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
    else//��������ģʽ
    {
      errorstatus = SDEnWideBus(DISABLE);

      if (SD_OK == errorstatus)
      {
        /*!< Configure the SDIO peripheral */
        SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV; 
        SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
        SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
        SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
        SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
        SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
        SDIO_Init(&SDIO_InitStructure);
      }
    }
  }

  return(errorstatus);
}




//-------------------------------�����������麯��---------------------------------------------------//
//----------------------------------------------------------------------------------------------------//



 /*******************************************
 * ��������CmdError
 * ����  ����CMD0����ļ�顣
 * ����  ����
 * ���  ��SD��������
 *******************************************/
static SD_Error CmdError(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t timeout;

  timeout = SDIO_CMD0TIMEOUT;  // SDIO_CMD0TIMEOUTΪö�ٱ���
	
  while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET))	/*��������Ƿ��ѷ���*/
  {
    timeout--;
  }
  if (timeout == 0)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);//�����̬��־λ

  return(errorstatus);
}


 /*
 * ��������CmdResp7Error
 * ����  ������Ӧ����ΪR7��������м��
 * ����  ����
 * ���  ��SD��������
 */
static SD_Error CmdResp7Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t timeout = SDIO_CMD0TIMEOUT;

  status = SDIO->STA;	//��ȡSDIO״̬�Ĵ��� ����״̬�Ĵ�����stm32�ļĴ��� 
  /* Command response received (CRC check failed) ��Command response received (CRC check passed)��Command response timeout */
  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0))
  {
    timeout--;
    status = SDIO->STA;		  
  }
	 //������Ӧcmd8
  if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT))
  {
    /*!< Card is not V2.0 complient or card does not support the set voltage range */
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  if (status & SDIO_FLAG_CMDREND)
  {
    /*!< Card is SD V2.0 compliant */
    errorstatus = SD_OK;
    SDIO_ClearFlag(SDIO_FLAG_CMDREND);
    return(errorstatus);
  }
  return(errorstatus);
}


 /*
 * ��������CmdResp1Error
 * ����  ������Ӧ����ΪR1��������м��
 * ����  ����
 * ���  ��SD��������
 */
static SD_Error CmdResp1Error(uint8_t cmd) //����Ĳ�����ʲô�ã�
{		   
	/*������Щ״̬�͵ȴ�	*/
  while (!(SDIO->STA & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
		;
  }

  SDIO->ICR = SDIO_STATIC_FLAGS;	//���жϱ�־
  return (SD_Error)(SDIO->RESP1 &  SD_OCR_ERRORBITS);		//�ж��Ƿ��ڹ��緶Χ
}

 /*
 * ��������CmdResp3Error
 * ����  ������Ӧ����ΪR3��������м��
 * ����  ����
 * ���  ��SD��������
 */
static SD_Error CmdResp3Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);
  return(errorstatus);
}

 /*
 * ��������CmdResp2Error
 * ����  ������Ӧ����ΪR2��������м��
 * ����  ����
 * ���  ��SD��������
 */
static SD_Error CmdResp2Error(void)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  return(errorstatus);
}

 /*
 * ��������CmdResp6Error
 * ����  ������Ӧ����ΪR6��������м��
 * ����  ��cmd ���������ţ�
 			prca �����洢���յ��Ŀ���Ե�ַ
 * ���  ��SD��������
 */
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca)
{
  SD_Error errorstatus = SD_OK;
  uint32_t status;
  uint32_t response_r1;

  status = SDIO->STA;

  while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND)))
  {
    status = SDIO->STA;
  }

  if (status & SDIO_FLAG_CTIMEOUT)
  {
    errorstatus = SD_CMD_RSP_TIMEOUT;
    SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
    return(errorstatus);
  }
  else if (status & SDIO_FLAG_CCRCFAIL)
  {
    errorstatus = SD_CMD_CRC_FAIL;
    SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
    return(errorstatus);
  }

  /*!< Check response received is of desired command */
  if (SDIO_GetCommandResponse() != cmd)		 //����Ƿ���յ���������
  {
    errorstatus = SD_ILLEGAL_CMD;
    return(errorstatus);
  }

  /*!< Clear all the static flags */
  SDIO_ClearFlag(SDIO_STATIC_FLAGS);

  /*!< We have received response, retrieve it.  */
  response_r1 = SDIO_GetResponse(SDIO_RESP1);

	/*����״̬ȫΪ0�����ɹ����յ�card���ص�rca */
  if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED)))
  {
    *prca = (uint16_t) (response_r1 >> 16);//����16λ�����ǽ��յ��ķ���rca
    return(errorstatus);
  }

  if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR)
  {
    return(SD_GENERAL_UNKNOWN_ERROR);
  }

  if (response_r1 & SD_R6_ILLEGAL_CMD)
  {
    return(SD_ILLEGAL_CMD);
  }

  if (response_r1 & SD_R6_COM_CRC_FAILED)
  {
    return(SD_COM_CRC_FAILED);
  }

  return(errorstatus);
}	 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
