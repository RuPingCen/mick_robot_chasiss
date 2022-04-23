/************************************************************************************************
*  ����˵���� ������ʹ�ú��� ʹ��оƬXPT2046  ʹ��Io��ģ��SPIʱ��
*  ����˵���� ��
*
*  �������أ� Coordinate * displayPtr   �ṹ�����  ����X Y ����16λ����   ���ڴ�Ŷ�ȡ������Һ������ֵ
* 				    0	---	У��ʧ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� У׼����ʹ���ⲿfalsh �洢У׼����   ʹ�õ�ַ 0~48   ��49���ֽ�   У׼�������λ  0x33 (��ַ0)
*
*
*		u8 LCD240x320_Touch_Calibrate();//����У׼���� �������أ� 1	---	У���ɹ� 0	---	У��ʧ��
*		LCD240x320_Touch_Init();//����ģ��SPI IO �� �ж� IO ��ʼ��
*   Get_touch_point(Coordinate * displayPtr);//ͨ�� K A B C D E F ��ͨ��X Y��ֵת��ΪҺ��������
*                                             //�����ȡ�Ĵ�����Ϣ���󣬽�����DISABLE
*
//----------------------------������Ժ���---------------------------------------------------//
*
*		Palette_Init();//�����ʼ��������ȡɫ�� 
*		Palette_draw_point(uint16_t x, uint16_t y);//��LCDָ��λ�û�һ�����(�����ĸ�С��)
*
************************************************************************************************/

#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_Delay.h"
#include "stm32f10x_usart.h" 



#include "LCD240x320_ILI9341.h"
#include "LCD240x320_Touch.h" 
#include "W25Q16.h"
 


volatile unsigned char LCD240x320_touch_flag; //�жϱ�־λ
Coordinate DisplaySample[4] = //��ʾʮ�ֲ�����  X,Y
{
    { 10,  35 },
    { 10,  200},
    { 200, 200},
    { 200, 35}
};
Coordinate ScreenSample[4];/* ��������ADֵ����ṹ�� */
Parameter  touch_para ;/* ���ڱ���У��ϵ�� */
long double aa1=0,bb1=0,cc1=0,aa2=0,bb2=0,cc2=0;/* ������У��ϵ�� */


/* 
long double aa1=0.088370,\
            bb1=-0.000468,\
            cc1=-24.042172,\
            aa2=0.0001891,\
            bb2=0.062395,\
            cc2=-10.223455;
 
*/
/* 
//����ģʽ�µĲ���
long double aa1=0.000176,\
            bb1=0.066735,\
            cc1=-23.160664,\
            aa2=-0.087162,\
            bb2=0.00459,\
            cc2=328.250406;
*/

/*************************************************************************
*  �������ƣ� LCD240x320_Touch_DelayUS
*  ����˵���� us�� �Ǿ�ȷ��ʱ
*  ����˵���� cnt ��ʱ���� 
*
*  �������أ� ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP   
*************************************************************************/  
void LCD240x320_Touch_DelayUS(vu32 cnt)
{
    uint16_t i;
    for(i = 0;i<cnt;i++)
    {
        uint8_t us = 12; /* ����ֵΪ12����Լ��1΢�� */    
        while (us--)     /* ��1΢��	*/
        {
            ;   
        }
    }
}

/*************************************************************************
*  �������ƣ� LCD240x320_Touch_IO_Init
*  ����˵���� ����ģ��SPI IO �� �ж� IO ��ʼ��
*  ����˵���� �� 
*
*  �������أ� ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP   
*************************************************************************/
void LCD240x320_Touch_Init(void)
{
				GPIO_InitTypeDef  GPIO_InitStructure;
				EXTI_InitTypeDef EXTI_InitStructure;
				NVIC_InitTypeDef NVIC_InitStructure;
//------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------//	
				/* ģ��SPI GPIO��ʼ�� */
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);/* ����GPIOʱ�� */				          
				GPIO_InitStructure.GPIO_Pin=SPI_CLK_PIN;
				GPIO_InitStructure.GPIO_Speed=GPIO_Speed_10MHz ;	  
				GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
				GPIO_Init(SPI_CLK_PORT, &GPIO_InitStructure);

				GPIO_InitStructure.GPIO_Pin = SPI_MOSI_PIN;
				GPIO_Init(SPI_MOSI_PORT, &GPIO_InitStructure);

				GPIO_InitStructure.GPIO_Pin = SPI_MISO_PIN; 
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;      
				GPIO_Init(SPI_MISO_PORT, &GPIO_InitStructure);

				GPIO_InitStructure.GPIO_Pin = SPI_CS_PIN; 
				GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz ;
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      
				GPIO_Init(SPI_CS_PORT, &GPIO_InitStructure); 
				 
				/* ����Ƭѡ��ѡ��XPT2046 */
				GPIO_ResetBits(SPI_CS_PORT,SPI_CS_PIN);
				//GPIO_SetBits(SPI_CS_PORT,SPI_CS_PIN);
				
//------------------------------------------------------------------------------------------//			
//------------------------------------------------------------------------------------------//
				/* XPT2046 �ж�IO���� */
				/* config the extiline clock and AFIO clock */
			  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_AFIO,ENABLE);
					NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); /* config the NVIC */						      
					NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn  ; /*ʹ��EXTI9_5 �ж� */	   
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);  
//------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------//
				/* EXTI line gpio config */	
				GPIO_InitStructure.GPIO_Pin = TP_INT_PIN;       
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	 // ��������
				GPIO_Init(TP_INT_PORT, &GPIO_InitStructure);
				/* EXTI line mode config */
				GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource9);  // PF9 
				EXTI_InitStructure.EXTI_Line = EXTI_Line9;
				EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
				EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½����ж�
				EXTI_InitStructure.EXTI_LineCmd = ENABLE;
				EXTI_Init(&EXTI_InitStructure);
//------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------//
	
}

/*************************************************************************
*  �������ƣ� XPT2046_WriteCMD
*  ����˵���� ������д�����
*  ����˵���� cmd:  CHX 	0x90 	//ͨ��Y+��ѡ������� CHY 	0xd0	//ͨ��X+��ѡ�������
*
*  �������أ� ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP   
*************************************************************************/
void XPT2046_WriteCMD(unsigned char cmd) 
{
      unsigned char buf;
      unsigned char i;
//    Touch_CS_Out(1);
      Touch_MOSI_Out(0);
      Touch_CLK_Out(0);
//    Touch_CS_Out(0);
			for(i=0;i<8;i++) 
			{
					buf=(cmd>>(7-i))&0x1;
					Touch_MOSI_Out(buf);
					 
					LCD240x320_Touch_DelayUS(5);
					Touch_CLK_Out(1);
					 
					LCD240x320_Touch_DelayUS(5);
					Touch_CLK_Out(0);
			}
}
/*************************************************************************
*  �������ƣ� XPT2046_ReadCMD
*  ����˵���� �����������������
*  ����˵���� ѡ��һ��ģ��ͨ��������ADC��������ADC�������
*
*  �������أ� ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP   
*************************************************************************/
unsigned short XPT2046_ReadCMD(void) 
{
    unsigned short buf=0,temp;
    unsigned char i;
    Touch_MOSI_Out(0);
    Touch_CLK_Out(1);
    for(i=0;i<12;i++) 
    {
        Touch_CLK_Out(0);          
        temp= (Touch_MISO_In) ? 1:0;
        buf|=(temp<<(11-i));
        Touch_CLK_Out(1);
    }
    buf&=0x0fff;

    return(buf);
}
/*************************************************************************
*  �������ƣ� Touch_GetAdXY
*  ����˵���� ��ȡTP x y ��ADֵ(12bit�������4096)
*  ����˵���� X  Y    ��ʾ�����ŵĵ�ַ
*
*  �������أ� ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP   
*************************************************************************/
void Touch_GetAdXY(int *x,int *y)  //��ȡTP x y ��ADֵ(12bit�������4096)
{ 
    int adx,ady; 
    XPT2046_WriteCMD(0x90);//ͨ��X+��ѡ�������
    adx =XPT2046_ReadCMD();//ѡ��һ��ģ��ͨ��������ADC��������ADC�������
	
    LCD240x320_Touch_DelayUS(3);  
	
    XPT2046_WriteCMD(0xd0);//ͨ��X+��ѡ�������
    ady =XPT2046_ReadCMD();
    
    *x=adx; 
    *y=ady; 
}
/*************************************************************************
*  �������ƣ� Read_LCD2046_XY
*  ����˵���� �õ����˲�֮���X Y  ����10��  ��ֵ�˲�
*  ����˵���� ��
*
*  �������أ� Coordinate �ṹ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP  ������Ӧ��ʵ��"ר��,���Ǻܾ�׼�������ٶȱȽϿ� 
*************************************************************************/ 
Coordinate *Read_LCD2046_XY(void)
{
    static Coordinate  screen2;
    int TP_X[1],TP_Y[1];
    uint8_t count=0;
    int buffer[2][10]={{0},{0}};  /*����X��Y���ж�β���*/
    int min_x,max_x;
    int min_y,max_y;
    int	i=0;
    
    do					       				
    {		/* ѭ������10�� */   
        Touch_GetAdXY(TP_X,TP_Y);  
        buffer[0][count]=TP_X[0];  
        buffer[1][count]=TP_Y[0];
        count++;  
    }	/*�û����������ʱ��TP_INT_IN�ź�Ϊ�� ���� count<10*/
    while(!INT_IN_2046&& count<10);
    
		
    if(INT_IN_2046)/*������ʵ���*/						
    {
				
        LCD240x320_touch_flag = 0;/*�жϱ�־��λ*/					 
    }
		
		/*����ɹ�����10������*/
    if(count ==10)		 					
    {
        max_x=min_x=buffer[0][0];
        max_y=min_y=buffer[1][0];       
        for(i=1; i<10; i++)
        {
            if(buffer[0][i]<min_x)
            {
                min_x=buffer[0][i];
            }
            else
            if(buffer[0][i]>max_x)
            {
                max_x = buffer[0][i];
            }
        }
        
        for(i=1; i<10; i++)
        {
            if(buffer[1][i]<min_y)
            {
                min_y=buffer[1][i];
            }
            else
            if(buffer[1][i]>max_y)
            {
                max_y = buffer[1][i];
            }
        }
				/*ȥ����Сֵ�����ֵ֮����ƽ��ֵ*/
        screen2.x=(buffer[0][0]+buffer[0][1]+buffer[0][2]+buffer[0][3]+buffer[0][4]+buffer[0][5]+buffer[0][6]+buffer[0][7]+buffer[0][8]+buffer[0][9]-min_x-max_x)>>3;
        screen2.y=(buffer[1][0]+buffer[1][1]+buffer[1][2]+buffer[1][3]+buffer[1][4]+buffer[1][5]+buffer[1][6]+buffer[1][7]+buffer[1][8]+buffer[1][9]-min_y-max_y)>>3; 
        
        return &screen2;
    }    
    return 0;    
}
/******************************************************
* ��������Calcu_touch_para
* ����  ���������������Һ��������任��ת�������� K A B C D E Fϵ��
* ����  : ��
* ���  ������1��ʾ�ɹ� 0ʧ��
* ����  ����
* ע��  ��ֻ����LCD�ʹ�����������Ƕȷǳ�Сʱ,�����������湫ʽ
*********************************************************/  
FunctionalState Calcu_touch_para( Coordinate * displayPtr,Coordinate * screenPtr,Parameter * para)
{    
    FunctionalState retTHRESHOLD = ENABLE ;

    /* K��(X0��X2) (Y1��Y2)��(X1��X2) (Y0��Y2) */
    para->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
    
		if( para->Divider == 0 )
    {
        retTHRESHOLD = DISABLE;
    }
    else
    {
        /* A��((XD0��XD2) (Y1��Y2)��(XD1��XD2) (Y0��Y2))��K	*/
        para->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                   ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y));
        
        /* B��((X0��X2) (XD1��XD2)��(XD0��XD2) (X1��X2))��K	*/
        para->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                   ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x));
        
        /* C��(Y0(X2XD1��X1XD2)+Y1(X0XD2��X2XD0)+Y2(X1XD0��X0XD1))��K */
        para->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                   (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                   (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;
        
        /* D��((YD0��YD2) (Y1��Y2)��(YD1��YD2) (Y0��Y2))��K	*/
        para->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                   ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
        
        /* E��((X0��X2) (YD1��YD2)��(YD0��YD2) (X1��X2))��K	*/
        para->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                   ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;
        
        
        /* F��(Y0(X2YD1��X1YD2)+Y1(X0YD2��X2YD0)+Y2(X1YD0��X0YD1))��K */
        para->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                   (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                   (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y;
        
    }
    return( retTHRESHOLD ) ;
}
/*************************************************************************
*  �������ƣ� Touchl_Calibrate
*  ����˵���� ������У������
*  ����˵���� ��
*
*  �������أ� 1	---	У���ɹ�
* 				    0	---	У��ʧ��
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP  
*************************************************************************/ 
u8 LCD240x320_Touch_Calibrate(void)
{
     
    uint8_t i;
    u16 test_x=0, test_y=0;
    u16 gap_x=0, gap_y=0;
    Coordinate * Ptr;   
    long double cal_p[6]={0};/* ������У��ϵ���ṹ�� */
    W25Q16_FLASH_BufferRead(SPI1,&i,0,1);
		if(i==0x33)
		{		
        W25Q16_FLASH_BufferRead(SPI1,(void*)cal_p,1,sizeof(cal_p));//��ȡ����  
			
				aa1 =cal_p[0];/* У��ϵ��Ϊȫ�ֱ��� */
				bb1 =cal_p[1];
				cc1 =cal_p[2];
				aa2 =cal_p[3];
				bb2 =cal_p[4];
				cc2 =cal_p[5];
			
				return 1; 
		}
		else//���SPI��û�б�����У������
		{
        	   
				for(i=0; i<4; i++)
				{        
						LCD240x320_ILI9341_Clear(0, 0,240,320, BLACK);        
						LCD240x320_ILI9341_DispStr(80,110,"Touch Calibrate......", RED,BLACK);;//�ַ�����ʾ����			
						LCD240x320_ILI9341_write_u16data(100,90,i+1,RED,BLACK);//�޷�����ʾ������0 ~ 65535��
										
						LCD240x320_TOUCH_Delay_ms(500);/* �ʵ�����ʱ���б�Ҫ */ 
		 
					 LCD240x320_ILI9341_Clear(DisplaySample[i].x,DisplaySample[i].y,20,1,RED); //��ʾУ���õġ�ʮ����	
					 LCD240x320_ILI9341_Clear(DisplaySample[i].x+10,DisplaySample[i].y-10,1,20,RED);//��������					
						
						do
						{
								Ptr=Read_LCD2046_XY();        //��ȡXPT2046���ݵ�����ptr            
						}
						while( Ptr == (void*)0 );     //��ptrΪ��ʱ��ʾû�д��㱻����
						ScreenSample[i].x= Ptr->x; 	  //�Ѷ�ȡ��ԭʼ���ݴ�ŵ�ȫ�ֱ���ScreenSample�ṹ��
						ScreenSample[i].y= Ptr->y;

				}
				
				Calcu_touch_para( &DisplaySample[0],&ScreenSample[0],&touch_para ) ;/* ��ԭʼ�������ȡ��������� ԭʼ�����������ת��ϵ���� */  	   
				
				/*ȡһ�������Xֵ*/
				test_x = ( (touch_para.An * ScreenSample[3].x) + 
									 (touch_para.Bn * ScreenSample[3].y) + 
										touch_para.Cn 
								 ) / touch_para.Divider ;			 
				
				/*ȡһ�������Yֵ*/
				test_y = ( (touch_para.Dn * ScreenSample[3].x) + 
									 (touch_para.En * ScreenSample[3].y) + 
									 touch_para.Fn 
								 ) / touch_para.Divider ;
				
				/* ʵ���������������Ĳ� */
				gap_x = (test_x > DisplaySample[3].x)?(test_x - DisplaySample[3].x):(DisplaySample[3].x - test_x);
				gap_y = (test_y > DisplaySample[3].y)?(test_y - DisplaySample[3].y):(DisplaySample[3].y - test_y);
		 
				LCD240x320_ILI9341_Clear(0, 0, 240, 320, BLACK);
				
			
				if((gap_x>10)||(gap_y>10))/* ����ͨ���޸�������ֵ�Ĵ�С���������� */
				{
			 
					 LCD240x320_ILI9341_DispStr(60,100,"Calibrate fail", RED,BLACK);;//�ַ�����ʾ����	
					 LCD240x320_ILI9341_DispStr(60,120,"try again", RED,BLACK);;//�ַ�����ʾ����				
					 LCD240x320_TOUCH_Delay_ms(2000);
					 return 0;
				}    

				/* У��ϵ��Ϊȫ�ֱ��� */
				aa1 = (touch_para.An*1.0)/touch_para.Divider;
				bb1 = (touch_para.Bn*1.0)/touch_para.Divider;
				cc1 = (touch_para.Cn*1.0)/touch_para.Divider;
				
				aa2 = (touch_para.Dn*1.0)/touch_para.Divider;
				bb2 = (touch_para.En*1.0)/touch_para.Divider;
				cc2 = (touch_para.Fn*1.0)/touch_para.Divider;
				
				{
					 cal_p[0]=aa1;//�����ݲ���д�뵽�ⲿflash�б���
					 cal_p[1]=bb1;
					 cal_p[2]=cc1;
					 cal_p[3]=aa2;
					 cal_p[4]=bb2;
					 cal_p[5]=cc2;
					 i=0x33;					  
					 W25Q16_FLASH_BufferWrite(SPI1,&i, 0, 1);//����д����
					 W25Q16_FLASH_BufferWrite(SPI1,(void*)cal_p, 1, sizeof(cal_p));//����д����
				}
				
				/*
				{
				//���ص���λ����ʾ
				UART_send_string(USART1,"\n aa1: ");//�ַ�������
				UART_send_floatdat(USART1,(aa1*1000.0));//���������ͺ��� ��ȷ��С�������λ
				UART_send_string(USART1,"\n bb1: ");//�ַ�������
				UART_send_floatdat(USART1,(bb1*1000.0));//���������ͺ��� ��ȷ��С�������λ
				UART_send_string(USART1,"\n cc1: ");//�ַ�������
				UART_send_floatdat(USART1,(cc1*1000.0));//���������ͺ��� ��ȷ��С�������λ
				
				UART_send_string(USART1,"\n aa2: ");//�ַ�������
				UART_send_floatdat(USART1,(aa2*1000.0));//���������ͺ��� ��ȷ��С�������λ
				UART_send_string(USART1,"\n bb2: ");//�ַ�������
				UART_send_floatdat(USART1,(bb2*1000.0));//���������ͺ��� ��ȷ��С�������λ
				UART_send_string(USART1,"\n cc2: ");//�ַ�������
				UART_send_floatdat(USART1,(cc2*10.0));//���������ͺ��� ��ȷ��С�������λ
				UART_send_string(USART1,"\n");//�ַ�������
			  }*/
				
				 LCD240x320_ILI9341_DispStr(60,100,"Calibrate Succed", RED,BLACK);;//�ַ�����ʾ����	
				 LCD240x320_TOUCH_Delay_ms(1000);
			
				return 1;   
		}		
}

/*************************************************************************
*  �������ƣ� Touchl_Calibrate
*  ����˵���� ͨ�� K A B C D E F ��ͨ��X Y��ֵת��ΪҺ��������
*  ����˵����  Coordinate * displayPtr   �ṹ�����  ����X Y ����16λ����   ���ڴ�Ŷ�ȡ������Һ������ֵ
*              Parameter * para   У��ϵ����Žṹ��
*
*  �������أ� ����1��ʾ�ɹ� 0ʧ��			     
*  �޸�ʱ�䣺 2014-11-2
*  ��    ע�� CRP   �����ȡ�Ĵ�����Ϣ���󣬽�����DISABLE
*************************************************************************/ 
FunctionalState Get_touch_point(Coordinate * displayPtr)
{
  FunctionalState retTHRESHOLD =ENABLE ; //FunctionalState  ��M3�ڲ���״̬��
  Coordinate * screenPtr=Read_LCD2046_XY();
	Parameter * para=&touch_para;
	
  if(screenPtr==0)/*�����ȡ�Ĵ�����Ϣ�����򷵻�DISABLE*/
  {
    
    retTHRESHOLD = DISABLE;			
  }
  else
  {    
    if(para->Divider != 1 )
    {        
      displayPtr->x = ( (aa1 * screenPtr->x) + (bb1 * screenPtr->y) + cc1);        
      displayPtr->y = ((aa2 * screenPtr->x) + (bb2 * screenPtr->y) + cc2 );
    }
    else
    {
      retTHRESHOLD = DISABLE;
    }
  }
  return(retTHRESHOLD);
} 







//-----------------------------------------------------------------------------------------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------//

/*
 * �����ʼ��������ȡɫ��  
 */
 
void Palette_Init(void)
{
  /* ������Ϊ��ɫ */
  LCD240x320_ILI9341_Clear(0, 0, 240, 320, WHITE);
  
  /* ������ֱ�� */
  LCD240x320_ILI9341_Clear(29, 0, 1, 40, BLACK);
  LCD240x320_ILI9341_Clear(0, 39, 30, 1, BLACK);
   
  LCD240x320_ILI9341_DispStr(7,12,"CLR", RED,WHITE);;//�ַ�����ʾ����	
	
  LCD240x320_ILI9341_Clear(30, 0, 30, 40, RED);
  LCD240x320_ILI9341_Clear(60, 0, 30, 40, MAGENTA);
  LCD240x320_ILI9341_Clear(90, 0, 30, 40, GREEN);
  LCD240x320_ILI9341_Clear(120, 0, 30, 40, YELLOW);
  LCD240x320_ILI9341_Clear(150, 0, 30, 40, CYAN);
	LCD240x320_ILI9341_Clear(180, 0, 30, 40, BLUE);
  LCD240x320_ILI9341_Clear(210, 0, 30, 40, BLUE2);  
 
  LCD240x320_TOUCH_Delay_ms(500);
}

/******************************************************
* ��������Palette_draw_point
* ����  ����LCDָ��λ�û�һ�����(�����ĸ�С��)
* ����  : Xpos		--X����λ��
*         Ypos		--Y����λ��
* ���  ����
* ����  ��Palette_draw_point(100,100);
* ע��  ���ú����� "��������Ӧ��ʵ��" ר�ú���
*********************************************************/    
void Palette_draw_point(uint16_t x, uint16_t y)
{
  /* ����Ĭ��Ϊ��ɫ */
  static u16 Pen_color=WHITE; 

  /* �ڻ�����ȡɫ */
  if( y<40 )
  {
    if( x < 30 )
		{
       LCD240x320_ILI9341_Clear(0, 40, 240, 280, WHITE);  
        return;
    }
		else if(x < 60)
		{
      Pen_color=RED;
    }
    else if(x < 90)
		{
      Pen_color=MAGENTA;
    }
    else if(x < 120)
		{
      Pen_color=GREEN;
    }
		else if(x < 150)
				{
					Pen_color=YELLOW;
				}
		else if(x < 180)
				{
					Pen_color=CYAN;
				}
		else if(x < 210)
				{
					Pen_color=BLUE;
				}		
    else 
			 Pen_color=BLUE2;
  }
      
 
    LCD240x320_ILI9341_SetPoint(x , y , Pen_color);
    LCD240x320_ILI9341_SetPoint(x-1 , y , Pen_color);
    LCD240x320_ILI9341_SetPoint(x , y-1 , Pen_color);
    LCD240x320_ILI9341_SetPoint(x+1 , y , Pen_color);
    LCD240x320_ILI9341_SetPoint(x , y+1 , Pen_color);
 
   	
}
