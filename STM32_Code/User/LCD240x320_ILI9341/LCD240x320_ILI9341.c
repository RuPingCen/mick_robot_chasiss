/******************************************************************************************************
  *
  * ��������Һ����ʹ����غ���    
  *       
  * ��ע��ʹ��FSMCģ������Һ��������  
	*
	*       1�����ײ�Ĭ��ʹ��12*6��Ӣ���ַ�   ����������Ҫʱ��������ȡģ ���޸ĺ��� LCD240x320_ILI9341_DispChar �п��贰�Ĵ�С
	*
	*
  * 2014-10-28

  LCD240x320_ILI9341_Init(u8 MODE);//Һ����ʼ������   MODE =  (VERTICAL: ����ģʽ       CROSS:  ����ģʽ ) 
  LCD240x320_ILI9341_DispChar(x,y, uint8_t ascii, fColor,bColor);//�ַ���ʾ����
  LCD240x320_ILI9341_DispStr(x,y, uint8_t *pstr, fColor,bColor);;//�ַ�����ʾ����
  LCD240x320_ILI9341_write_intdata(x,y,double tep_data,fColor,bColor);//�з�������ʾ������-65535 ~  +65535��
  LCD240x320_ILI9341_write_u16data(x,y,unsigned int tep_data,fColor,bColor);//�޷�����ʾ������0 ~ 65535��
  LCD240x320_ILI9341_write_floatdata(x,y,int tep_data,fColor,bColor);//��������ʾ���� ��ȷ��С�������λ
  LCD240x320_ILI9341_show_GB3232_string(x,y,char *s,fColor,bColor);	//16*16������ʾ����
  LCD240x320_ILI9341_show_GB1616_string(x,y,char *s,fColor,bColor); //32*32������ʾ����
  LCD240x320_ILI9341_show_Image(x,y,x1,y1,unsigned char temp[]);//ͼƬ��ʾ����  x1 x2 ��ʾͼƬ�ĳߴ�
	
	
  LCD240x320_ILI9341_SetPoint(uint16_t x , uint16_t y , uint16_t color);//	���㺯��
  LCD240x320_ILI9341_Clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);//��������

	
	
	
	
	
//  _ _ _ _ _ __________X
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |- - - - - -
// |
// |
// Y
*******************************************************************************************************/





#include "stm32f10x_fsmc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"



#include "LCD240x320_ILI9341.h"
#include "ascii.h"	//ASCII �ַ���
#include "GB1616.h"	//GB1616 ���ֱ�
#include "GB3232.h"	//GB3232 ���ֱ�
//#include "IMAGE.h"	//ͼƬ���ͷ�ļ�   �������ж����˸�ͷ�ļ��� ������Ҫȡ���� ���߻ᱨ�� 


#define DEBUG_DELAY()
 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_Delay(__IO uint32_t nCount)
	* �������ܣ� Һ������ʱ����   
	*              
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_Delay(__IO uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_GPIO_Config(void)
	* �������ܣ� Һ����ʹ��ʹ��ģ��FMSC IO�ڳ�ʼ������         
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    /* ʹ��FSMCʱ��*/
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);
    
    /* ʹ��FSMC��Ӧ��Ӧ�ܽ�ʱ��*/
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE 
	                          | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOG 
	                          | RCC_APB2Periph_GPIOF , ENABLE);
    
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    
    /* ����LCD������ƹܽ�*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;		
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    /* ����LCD��λ���ƹܽ�*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 ; 	 
    GPIO_Init(GPIOG, &GPIO_InitStructure);  		   
    
    /* ����FSMC���Ӧ��������,FSMC-D0~D15: PD 14 15 0 1,PE 7 8 9 10 11 12 13 14 15,PD 8 9 10*/	
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode =  GPIO_Mode_AF_PP;
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_8 | GPIO_Pin_9 | 
                                  GPIO_Pin_10 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | 
                                  GPIO_Pin_11 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | 
                                  GPIO_Pin_15;
    GPIO_Init(GPIOE, &GPIO_InitStructure); 
    
		/* ����FSMC���Ӧ�Ŀ�����
		 * PD4-FSMC_NOE   :LCD-RD
		 * PD5-FSMC_NWE   :LCD-WR
		 * PG12-FSMC_NE4  :LCD-CS
		 * PE2-FSMC_A23   :LCD-DC
		*/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; 
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12; 
    GPIO_Init(GPIOG, &GPIO_InitStructure);  
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; 
    GPIO_Init(GPIOE, &GPIO_InitStructure);  
    
    /* tft control gpio init */	 
    /* ������ */
		GPIO_ResetBits(GPIOB, GPIO_Pin_1);  
}

 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_FSMC_Config(void)
	* �������ܣ� Һ����ʹ��ʹ��ģ��FMSC ��ʼ������         
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_FSMC_Config(void)
{
    FSMC_NORSRAMInitTypeDef  FSMC_NORSRAMInitStructure;
    FSMC_NORSRAMTimingInitTypeDef  p; 
    
    
    p.FSMC_AddressSetupTime = 0x02;	 //��ַ����ʱ��
    p.FSMC_AddressHoldTime = 0x00;	 //��ַ����ʱ��
    p.FSMC_DataSetupTime = 0x05;		 //���ݽ���ʱ��
    p.FSMC_BusTurnAroundDuration = 0x00;
    p.FSMC_CLKDivision = 0x00;
    p.FSMC_DataLatency = 0x00;
    p.FSMC_AccessMode = FSMC_AccessMode_B;	 // һ��ʹ��ģʽB������LCD
    
    FSMC_NORSRAMInitStructure.FSMC_Bank = FSMC_Bank1_NORSRAM4;
    FSMC_NORSRAMInitStructure.FSMC_DataAddressMux = FSMC_DataAddressMux_Disable;
    //FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_SRAM;
		FSMC_NORSRAMInitStructure.FSMC_MemoryType = FSMC_MemoryType_NOR;
    FSMC_NORSRAMInitStructure.FSMC_MemoryDataWidth = FSMC_MemoryDataWidth_16b;
    FSMC_NORSRAMInitStructure.FSMC_BurstAccessMode = FSMC_BurstAccessMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalPolarity = FSMC_WaitSignalPolarity_Low;
    FSMC_NORSRAMInitStructure.FSMC_WrapMode = FSMC_WrapMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignalActive = FSMC_WaitSignalActive_BeforeWaitState;
    FSMC_NORSRAMInitStructure.FSMC_WriteOperation = FSMC_WriteOperation_Enable;
    FSMC_NORSRAMInitStructure.FSMC_WaitSignal = FSMC_WaitSignal_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ExtendedMode = FSMC_ExtendedMode_Disable;
    FSMC_NORSRAMInitStructure.FSMC_WriteBurst = FSMC_WriteBurst_Disable;
    FSMC_NORSRAMInitStructure.FSMC_ReadWriteTimingStruct = &p;
    FSMC_NORSRAMInitStructure.FSMC_WriteTimingStruct = &p;  
    
    FSMC_NORSRAMInit(&FSMC_NORSRAMInitStructure); 
    
    /* ʹ�� FSMC Bank1_SRAM Bank */
    FSMC_NORSRAMCmd(FSMC_Bank1_NORSRAM4, ENABLE);  
}

 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_Rst(void)
	* �������ܣ� Һ������λ����  
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_Rst(void)
{			
		GPIO_ResetBits(GPIOG, GPIO_Pin_11);	 //�͵�ƽ��λ
    LCD240x320_ILI9341_Delay(0xAFFf<<2); 					   
    GPIO_SetBits(GPIOG, GPIO_Pin_11);		 	 
    LCD240x320_ILI9341_Delay(0xAFFf<<2); 	
}

 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_REG_Config(u8  MODE)
	* �������ܣ� Һ�������ò�������  
  * ��ע�� CRP
  * 2014-10-28   MODE =  (VERTICAL: ����ģʽ       CROSS:  ����ģʽ ) 
  ****************************************************************/
void LCD240x320_ILI9341_REG_Config(u8  MODE)
{
	/*  Power control B (CFh)  */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xCF);
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x81);
	LCD_ILI9341_Parameter(0x30);
	
	/*  Power on sequence control (EDh) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xED);
	LCD_ILI9341_Parameter(0x64);
	LCD_ILI9341_Parameter(0x03);
	LCD_ILI9341_Parameter(0x12);
	LCD_ILI9341_Parameter(0x81);
	
	/*  Driver timing control A (E8h) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xE8);
	LCD_ILI9341_Parameter(0x85);
	LCD_ILI9341_Parameter(0x10);
	LCD_ILI9341_Parameter(0x78);
	
	/*  Power control A (CBh) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xCB);
	LCD_ILI9341_Parameter(0x39);
	LCD_ILI9341_Parameter(0x2C);
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x34);
	LCD_ILI9341_Parameter(0x02);
	
	/* Pump ratio control (F7h) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xF7);
	LCD_ILI9341_Parameter(0x20);
	
	/* Driver timing control B */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xEA);
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x00);
	
	/* Frame Rate Control (In Normal Mode/Full Colors) (B1h) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xB1);
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x1B);
	
	/*  Display Function Control (B6h) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xB6);
	LCD_ILI9341_Parameter(0x0A);
	LCD_ILI9341_Parameter(0xA2);
	
	/* Power Control 1 (C0h) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xC0);
	LCD_ILI9341_Parameter(0x35);
	
	/* Power Control 2 (C1h) */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0xC1);
	LCD_ILI9341_Parameter(0x11);
	
	/* VCOM Control 1(C5h) */
	LCD_ILI9341_CMD(0xC5);
	LCD_ILI9341_Parameter(0x45);
	LCD_ILI9341_Parameter(0x45);
	
	/*  VCOM Control 2(C7h)  */
	LCD_ILI9341_CMD(0xC7);
	LCD_ILI9341_Parameter(0xA2);
	
	/* Enable 3G (F2h) */
	LCD_ILI9341_CMD(0xF2);
	LCD_ILI9341_Parameter(0x00);
	
	/* Gamma Set (26h) */
	LCD_ILI9341_CMD(0x26);
	LCD_ILI9341_Parameter(0x01);
	DEBUG_DELAY();
	
	/* Positive Gamma Correction */
	LCD_ILI9341_CMD(0xE0); //Set Gamma
	LCD_ILI9341_Parameter(0x0F);
	LCD_ILI9341_Parameter(0x26);
	LCD_ILI9341_Parameter(0x24);
	LCD_ILI9341_Parameter(0x0B);
	LCD_ILI9341_Parameter(0x0E);
	LCD_ILI9341_Parameter(0x09);
	LCD_ILI9341_Parameter(0x54);
	LCD_ILI9341_Parameter(0xA8);
	LCD_ILI9341_Parameter(0x46);
	LCD_ILI9341_Parameter(0x0C);
	LCD_ILI9341_Parameter(0x17);
	LCD_ILI9341_Parameter(0x09);
	LCD_ILI9341_Parameter(0x0F);
	LCD_ILI9341_Parameter(0x07);
	LCD_ILI9341_Parameter(0x00);
	
	/* Negative Gamma Correction (E1h) */
	LCD_ILI9341_CMD(0XE1); //Set Gamma
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x19);
	LCD_ILI9341_Parameter(0x1B);
	LCD_ILI9341_Parameter(0x04);
	LCD_ILI9341_Parameter(0x10);
	LCD_ILI9341_Parameter(0x07);
	LCD_ILI9341_Parameter(0x2A);
	LCD_ILI9341_Parameter(0x47);
	LCD_ILI9341_Parameter(0x39);
	LCD_ILI9341_Parameter(0x03);
	LCD_ILI9341_Parameter(0x06);
	LCD_ILI9341_Parameter(0x06);
	LCD_ILI9341_Parameter(0x30);
	LCD_ILI9341_Parameter(0x38);
	LCD_ILI9341_Parameter(0x0F);
	
	/* memory access control set */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0x36); 
  if(MODE== CROSS)
	{
      LCD_ILI9341_Parameter(0x68);      //����  0xC8    ���Ͻǵ�(���)�����½�(�յ�)ɨ�跽ʽ     
  }	                                    //����  0x68    ����ģʽ�µ����½�  �����Ͻ�
	else if(MODE==VERTICAL)	
	{
      LCD_ILI9341_Parameter(0xC8);    
	}
	
	DEBUG_DELAY();                 
	
	/* column address control set */
	LCD_ILI9341_CMD(0X2A); 
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0xEF);
	
	/* page address control set */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0X2B); 
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x00);
	LCD_ILI9341_Parameter(0x01);
	LCD_ILI9341_Parameter(0x3F);
	
	/*  Pixel Format Set (3Ah)  */
	DEBUG_DELAY();
	LCD_ILI9341_CMD(0x3a); 
	LCD_ILI9341_Parameter(0x55);
	
	/* Sleep Out (11h)  */
	LCD_ILI9341_CMD(0x11);	
	LCD240x320_ILI9341_Delay(0xAFFf<<2);
	DEBUG_DELAY();
	
	/* Display ON (29h) */
	LCD_ILI9341_CMD(0x29); 
}

 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_Init(void)
	* �������ܣ� Һ������ʼ������
  * ��ע�� CRP
  * 2014-10-28   MODE =  (VERTICAL: ����ģʽ       CROSS:  ����ģʽ ) 
  ****************************************************************/
void LCD240x320_ILI9341_Init(u8  MODE )
{
	LCD240x320_ILI9341_GPIO_Config();
	LCD240x320_ILI9341_FSMC_Config();
	
	LCD240x320_ILI9341_Rst();
	LCD240x320_ILI9341_REG_Config(MODE);
}
 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_Clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
	* �������ܣ� Һ������������
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_Clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
	uint32_t i = 0;
	
	/* column address control set */
	LCD_ILI9341_CMD(0X2A);
	LCD_ILI9341_Parameter( x >> 8 );	 /* �ȸ�8λ��Ȼ���8λ */
	LCD_ILI9341_Parameter( x & 0xff );	         /* column start   */ 
	LCD_ILI9341_Parameter( (x+width-1) >> 8 );   /* column end   */
	LCD_ILI9341_Parameter( (x+width-1) & 0xff );
	
	/* page address control set */	
  LCD_ILI9341_CMD(0X2B); 			     
	LCD_ILI9341_Parameter( y >> 8 );			/* page start   */
	LCD_ILI9341_Parameter( y & 0xff );
	LCD_ILI9341_Parameter( (y+height-1) >> 8);  /* page end     */
	LCD_ILI9341_Parameter( (y+height-1) & 0xff);
	
	/* memory write */
	LCD_ILI9341_CMD(0x2c);	
		
	for( i=0; i < width*height; i++ )
	{
		LCD_WR_Data( color );
		//Delay(0x0FFf);
	}	
}
 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_SetCursor(uint16_t x, uint16_t y)	
	* �������ܣ� Һ������ʾ�������ú���
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_SetCursor(uint16_t x, uint16_t y)	
{	
	LCD_ILI9341_CMD(0X2A); 				 /* ����X���� */
	LCD_ILI9341_Parameter(x>>8);	 /* �ȸ�8λ��Ȼ���8λ */
	LCD_ILI9341_Parameter(x&0xff);	 /* ������ʼ��ͽ�����*/
	LCD_ILI9341_Parameter(x>>8);
	LCD_ILI9341_Parameter(x&0xff);

    LCD_ILI9341_CMD(0X2B); 			     /* ����Y����*/
	LCD_ILI9341_Parameter(y>>8);
	LCD_ILI9341_Parameter(y&0xff);
	LCD_ILI9341_Parameter(y>>8);
	LCD_ILI9341_Parameter(y&0xff);		     
}
 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_OpenWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
	* �������ܣ� Һ������������
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
//  _ _ _ _ _ _
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
// |           |
//  - - - - - -
void LCD240x320_ILI9341_OpenWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height)
{	
	LCD_ILI9341_CMD(0X2A); 				 /* ����X���� */
	LCD_ILI9341_Parameter( x >> 8 );	 /* �ȸ�8λ��Ȼ���8λ */
	LCD_ILI9341_Parameter( x & 0xff );	 /* ������ʼ��ͽ�����*/
	LCD_ILI9341_Parameter( (x+width-1) >> 8 );
	LCD_ILI9341_Parameter( (x+width-1) & 0xff );

	LCD_ILI9341_CMD(0X2B); 			     /* ����Y����*/
	LCD_ILI9341_Parameter( y >> 8 );
	LCD_ILI9341_Parameter( y & 0xff );
	LCD_ILI9341_Parameter( (y+height-1) >> 8);
	LCD_ILI9341_Parameter( (y+height-1) & 0xff);
}
 /****************************************************************
  *
  * ��������  LCD240x320_ILI9341_SetPoint(uint16_t x , uint16_t y , uint16_t color)
	* �������ܣ� Һ�������㺯��
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_SetPoint(uint16_t x , uint16_t y , uint16_t color)	
{	
	LCD240x320_ILI9341_SetCursor(x, y);
	LCD_ILI9341_CMD(0x2c);	         /* д���� */
	LCD_WR_Data(color);
}
 /****************************************************************
  *
  * ��������  uint16_t LCD240x320_ILI9341_RD_data(void)	
	* �������ܣ� ��RGBֵ  ����
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
uint16_t LCD240x320_ILI9341_RD_data(void)	
{	
	uint16_t R=0, G=0, B=0 ;

	R = *(__IO uint16_t *)Bank1_LCD_D; 	  /*FIRST READ OUT DUMMY DATA*/
	R = *(__IO uint16_t *)Bank1_LCD_D;  	/*READ OUT RED DATA  */
	B = *(__IO uint16_t *)Bank1_LCD_D;  	/*READ OUT BLACK DATA*/
	G = *(__IO uint16_t *)Bank1_LCD_D;  	/*READ OUT GREEN DATA*/
	
    return (((R>>11)<<11) | ((G>>10)<<5) | (B>>11));
}
 /****************************************************************
  *
  * ��������  uint16_t LCD240x320_ILI9341_GetPoint(uint16_t x , uint16_t y)
	* �������ܣ� ��RGBֵĳһ����ĺ���
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
uint16_t LCD240x320_ILI9341_GetPoint(uint16_t x , uint16_t y)
{ 
	uint16_t temp;

	LCD240x320_ILI9341_SetCursor(x, y);
	LCD_ILI9341_CMD(0x2e);         /* ������ */
	temp=LCD240x320_ILI9341_RD_data();
	return (temp);
}
 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_DispChar(uint16_t x, uint16_t y, uint8_t ascii, unsigned short int fColor, unsigned short int bColor)
	* �������ܣ� �ַ���ʾ����
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_DispChar(uint16_t x, uint16_t y, uint8_t ascii, unsigned short int fColor, unsigned short int bColor)
{
	uint16_t page, column, temp, i;
	i = ascii - ' ';
	
	LCD240x320_ILI9341_OpenWindow(x, y, STR_WIDTH, STR_HEIGHT);//����ÿһ���ַ�����ʾ�ռ�
	LCD_ILI9341_CMD(0X2C);	
	
	for( page=0; page < STR_HEIGHT; page++ )
	{
		temp = asc2_1206[i][page];
		for( column=0; column < STR_WIDTH; column++ )
		{
			if( temp & 0x01 )
			{
				LCD_WR_Data( fColor );
			}
			else
			{
				LCD_WR_Data( bColor);								
			}
			temp >>= 1;		
		}/* һ��д�� */
	}/* ȫ��д�� */
}
 /****************************************************************
  *
  * ��������LCD240x320_ILI9341_DispStr(uint16_t x, uint16_t y, uint8_t *pstr, unsigned short int fColor, unsigned short int bColor)
	* �������ܣ� �ִ���ʾ����
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_DispStr(uint16_t x, uint16_t y, uint8_t *pstr, unsigned short int fColor, unsigned short int bColor)
{
	while( *pstr != '\0' )
	{
		if( x > (COLUMN-STR_WIDTH) )
		{
			x = 0;
			y += STR_HEIGHT;
		}
		if( y > (PAGE-STR_HEIGHT) )
		{
			x = 0;
			y = 0;
		}
		LCD240x320_ILI9341_DispChar(x, y, *pstr, fColor,bColor);
		x += STR_WIDTH;
		pstr++;
	}
}

 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_write_intdata(uint16_t x, uint16_t y, uint32_t num, uint16_t color)
	* �������ܣ� ��ֵ��ʾ����
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_write_intdata(unsigned short int x, unsigned short int y,int tep_data,unsigned short int fColor, unsigned short int bColor)
{
  
    unsigned char translateData[7];
    
    if(tep_data < 0)  
      {
        translateData[0] = '-';
        tep_data=tep_data*(-1);
      }  
    else
      translateData[0] = ' ';  
    
    translateData[1] = tep_data /10000 + '0';
    translateData[2] = tep_data % 10000/1000 + '0';
    translateData[3] = tep_data % 1000 / 100 + '0';
    translateData[4] = tep_data % 100 / 10 + '0';
    translateData[5] = tep_data % 10 + '0';
    translateData[6] = '\0';

		LCD240x320_ILI9341_DispStr( x,y,translateData,fColor,bColor);
}
  /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_write_u16data(uint16_t x, uint16_t y, uint32_t num, uint16_t color)
	* �������ܣ� �޷�������ֵ��ʾ����
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_write_u16data(unsigned short int x, unsigned short int y,unsigned int tep_data,unsigned short int fColor, unsigned short int bColor)
{
  
    unsigned char translateData[6];    
    translateData[0] = tep_data /10000 + '0';
    translateData[1] = tep_data % 10000/1000 + '0';
    translateData[2] = tep_data % 1000 / 100 + '0';
    translateData[3] = tep_data % 100 / 10 + '0';
    translateData[4] = tep_data % 10 + '0';
    translateData[5] = '\0';
	  LCD240x320_ILI9341_DispStr( x,y,translateData,fColor,bColor);
}
   /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_write_floatdata(uint16_t x, uint16_t y, uint32_t num, uint16_t color)
	* �������ܣ� ��������ʾ����
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_write_floatdata(unsigned short int x, unsigned short int y,double tep_data,unsigned short int fColor, unsigned short int bColor)
{
    unsigned char translateData[11]; 
    int tep1,tep2;
    if(tep_data < 0)
    {  
         tep_data=tep_data*(-1);
         translateData[0] = '-';
    }
    else
         translateData[0] = ' ';

    tep1=tep_data/1;//?????
    tep2=((tep_data-tep1)*1000)/1;//???????1000?
    translateData[1] = tep1 /10000  + '0';
    translateData[2] = tep1 % 10000/1000 + '0';
    translateData[3] = tep1 % 1000/100 + '0';
    translateData[4] = tep1 % 100/10 + '0';
    translateData[5] = tep1 % 10 + '0';
    translateData[6] =  '.';
    translateData[7] = tep2/100 + '0';
    translateData[8] = tep2/10%10 + '0';
    translateData[9] = tep2%10  + '0';
    translateData[10] = '\0';
	
		 LCD240x320_ILI9341_DispStr( x,y,translateData,fColor,bColor);
}

/****************************************************************
  *
  * ��������   LCD240x320_ILI9341_show_GB1616 
	* �������ܣ� ������ʾ����  16*16
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_show_GB1616(unsigned int x,unsigned int y,char c[2],unsigned short int fColor, unsigned short int bColor)	
{  
	      unsigned int i,j,k;
        unsigned short m;
	 
       LCD240x320_ILI9341_OpenWindow(x, y, 16, 16);//����ÿһ���ַ�����ʾ�ռ�
	     LCD_ILI9341_CMD(0X2C);	 
        for (k=0;k<64;k++)  
        {
          if ((codeGB_16[k].Index[0]==c[0])&&(codeGB_16[k].Index[1]==c[1]))
           { 
               for(i=0;i<32;i++) 
               {
                   m=codeGB_16[k].Msk[i];
                   for(j=0;j<8;j++) 
                   {
											if((m&0x80)==0x80) 
											{
												 LCD_WR_Data(fColor); 
											}
											else 
											{
												 LCD_WR_Data(bColor); 
											}
											m<<=1;
                   } 
               }
           }  
     }
	 
}
/****************************************************************
  *
  * ��������   LCD240x320_ILI9341_show_GB1616_string 
	* �������ܣ� һ��������ʾ����  16*16
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_show_GB1616_string(unsigned int x,unsigned int y,char *s,unsigned short int fColor, unsigned short int bColor)	
{  
		while(*s)
			{
				 LCD240x320_ILI9341_show_GB1616(x,y,s,fColor,bColor);
				 s+=2;
				 x+=16;
				 
			} 
}
/****************************************************************
  *
  * ��������   LCD240x320_ILI9341_show_GB1616 
	* �������ܣ� ������ʾ���� 32*32
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_show_GB3232(unsigned int x,unsigned int y,char c[2],unsigned short int fColor, unsigned short int bColor)	
{  
	      unsigned int i,j,k;
        unsigned short m;
	 
       LCD240x320_ILI9341_OpenWindow(x, y, 32, 32);//����ÿһ���ַ�����ʾ�ռ�
	     LCD_ILI9341_CMD(0X2C);	 
        for (k=0;k<64;k++)  
        {
          if ((codeGB_32[k].Index[0]==c[0])&&(codeGB_32[k].Index[1]==c[1]))
           { 
               for(i=0;i<128;i++) 
               {
                   m=codeGB_32[k].Msk[i];
                   for(j=0;j<8;j++) 
                   {
											if((m&0x80)==0x80) 
											{
												 LCD_WR_Data(fColor); 
											}
											else 
											{
												 LCD_WR_Data(bColor); 
											}
											m<<=1;
                   } 
               }
           }  
     }
	 
}
/****************************************************************
  *
  * ��������   LCD240x320_ILI9341_show_GB3232_string 
	* �������ܣ� һ��������ʾ����  32*32
  * ��ע�� CRP
  * 2014-10-28
  ****************************************************************/
void LCD240x320_ILI9341_show_GB3232_string(unsigned int x,unsigned int y,char *s,unsigned short int fColor, unsigned short int bColor)	
{  

		while(*s)
			{
				 LCD240x320_ILI9341_show_GB3232(x,y,s,fColor,bColor);
				 s+=2;
				 x+=32;
				 
			} 
}
 /****************************************************************
  *
  * ��������   LCD240x320_ILI9341_show_GB3232_string 
	* �������ܣ� ͼƬ��ʾ����  x1 x2 ��ʾͼƬ�ĳߴ�
  * ��ע�� CRP
  * 2014-10-28  
  ****************************************************************/
void LCD240x320_ILI9341_show_Image(unsigned int x,unsigned int y,unsigned short int x1,unsigned short int y1,unsigned char temp[])	
{  
		unsigned int i,k;
    k=x1*y1;
    LCD240x320_ILI9341_OpenWindow(x, y, x1, y1);//����ÿһ���ַ�����ʾ�ռ�
	  LCD_ILI9341_CMD(0X2C);	       
    
    for(i=0;i<k;i++)
    { 	 				   
       LCD_WR_Data((temp[i*2+1] << 8)+temp[i*2]);//image   
      
    }	 
}


