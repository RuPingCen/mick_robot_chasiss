/************************************************************************************************
*  功能说明： 触摸屏使用函数 使用芯片XPT2046  使用Io口模拟SPI时序
*  参数说明： 无
*
*  函数返回： Coordinate * displayPtr   结构体变量  包含X Y 两个16位变量   用于存放读取出来的液晶坐标值
* 				    0	---	校正失败
*  修改时间： 2014-11-2
*  备    注： 校准函数使用外部falsh 存储校准数据   使用地址 0~48   共49个字节   校准参数检测位  0x33 (地址0)
*
*
*		u8 LCD240x320_Touch_Calibrate();//画板校准函数 函数返回： 1	---	校正成功 0	---	校正失败
*		LCD240x320_Touch_Init();//触摸模拟SPI IO 和 中断 IO 初始化
*   Get_touch_point(Coordinate * displayPtr);//通过 K A B C D E F 把通道X Y的值转换为液晶屏坐标
*                                             //如果获取的触点信息有误，将返回DISABLE
*
//----------------------------画板测试函数---------------------------------------------------//
*
*		Palette_Init();//画板初始化，用于取色用 
*		Palette_draw_point(uint16_t x, uint16_t y);//在LCD指定位置画一个大点(包含四个小点)
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
 


volatile unsigned char LCD240x320_touch_flag; //中断标志位
Coordinate DisplaySample[4] = //显示十字叉坐标  X,Y
{
    { 10,  35 },
    { 10,  200},
    { 200, 200},
    { 200, 35}
};
Coordinate ScreenSample[4];/* 触摸采样AD值保存结构体 */
Parameter  touch_para ;/* 用于保存校正系数 */
long double aa1=0,bb1=0,cc1=0,aa2=0,bb2=0,cc2=0;/* 触摸屏校正系数 */


/* 
long double aa1=0.088370,\
            bb1=-0.000468,\
            cc1=-24.042172,\
            aa2=0.0001891,\
            bb2=0.062395,\
            cc2=-10.223455;
 
*/
/* 
//竖屏模式下的参数
long double aa1=0.000176,\
            bb1=0.066735,\
            cc1=-23.160664,\
            aa2=-0.087162,\
            bb2=0.00459,\
            cc2=328.250406;
*/

/*************************************************************************
*  函数名称： LCD240x320_Touch_DelayUS
*  功能说明： us级 非精确延时
*  参数说明： cnt 延时参数 
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP   
*************************************************************************/  
void LCD240x320_Touch_DelayUS(vu32 cnt)
{
    uint16_t i;
    for(i = 0;i<cnt;i++)
    {
        uint8_t us = 12; /* 设置值为12，大约延1微秒 */    
        while (us--)     /* 延1微秒	*/
        {
            ;   
        }
    }
}

/*************************************************************************
*  函数名称： LCD240x320_Touch_IO_Init
*  功能说明： 触摸模拟SPI IO 和 中断 IO 初始化
*  参数说明： 无 
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP   
*************************************************************************/
void LCD240x320_Touch_Init(void)
{
				GPIO_InitTypeDef  GPIO_InitStructure;
				EXTI_InitTypeDef EXTI_InitStructure;
				NVIC_InitTypeDef NVIC_InitStructure;
//------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------//	
				/* 模拟SPI GPIO初始化 */
				RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_GPIOG, ENABLE);/* 开启GPIO时钟 */				          
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
				 
				/* 拉低片选，选择XPT2046 */
				GPIO_ResetBits(SPI_CS_PORT,SPI_CS_PIN);
				//GPIO_SetBits(SPI_CS_PORT,SPI_CS_PIN);
				
//------------------------------------------------------------------------------------------//			
//------------------------------------------------------------------------------------------//
				/* XPT2046 中断IO配置 */
				/* config the extiline clock and AFIO clock */
			  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF | RCC_APB2Periph_AFIO,ENABLE);
					NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); /* config the NVIC */						      
					NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn  ; /*使能EXTI9_5 中断 */	   
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);  
//------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------//
				/* EXTI line gpio config */	
				GPIO_InitStructure.GPIO_Pin = TP_INT_PIN;       
				GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;	 // 上拉输入
				GPIO_Init(TP_INT_PORT, &GPIO_InitStructure);
				/* EXTI line mode config */
				GPIO_EXTILineConfig(GPIO_PortSourceGPIOF, GPIO_PinSource9);  // PF9 
				EXTI_InitStructure.EXTI_Line = EXTI_Line9;
				EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
				EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿中断
				EXTI_InitStructure.EXTI_LineCmd = ENABLE;
				EXTI_Init(&EXTI_InitStructure);
//------------------------------------------------------------------------------------------//
//------------------------------------------------------------------------------------------//
	
}

/*************************************************************************
*  函数名称： XPT2046_WriteCMD
*  功能说明： 触摸屏写命令函数
*  参数说明： cmd:  CHX 	0x90 	//通道Y+的选择控制字 CHY 	0xd0	//通道X+的选择控制字
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP   
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
*  函数名称： XPT2046_ReadCMD
*  功能说明： 触摸屏读数据命令函数
*  参数说明： 选择一个模拟通道，启动ADC，并返回ADC采样结果
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP   
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
*  函数名称： Touch_GetAdXY
*  功能说明： 读取TP x y 的AD值(12bit，最大是4096)
*  参数说明： X  Y    表示结果存放的地址
*
*  函数返回： 无
*  修改时间： 2014-11-2
*  备    注： CRP   
*************************************************************************/
void Touch_GetAdXY(int *x,int *y)  //读取TP x y 的AD值(12bit，最大是4096)
{ 
    int adx,ady; 
    XPT2046_WriteCMD(0x90);//通道X+的选择控制字
    adx =XPT2046_ReadCMD();//选择一个模拟通道，启动ADC，并返回ADC采样结果
	
    LCD240x320_Touch_DelayUS(3);  
	
    XPT2046_WriteCMD(0xd0);//通道X+的选择控制字
    ady =XPT2046_ReadCMD();
    
    *x=adx; 
    *y=ady; 
}
/*************************************************************************
*  函数名称： Read_LCD2046_XY
*  功能说明： 得到简单滤波之后的X Y  采样10次  均值滤波
*  参数说明： 无
*
*  函数返回： Coordinate 结构体
*  修改时间： 2014-11-2
*  备    注： CRP  ”画板应用实例"专用,不是很精准，但是速度比较快 
*************************************************************************/ 
Coordinate *Read_LCD2046_XY(void)
{
    static Coordinate  screen2;
    int TP_X[1],TP_Y[1];
    uint8_t count=0;
    int buffer[2][10]={{0},{0}};  /*坐标X和Y进行多次采样*/
    int min_x,max_x;
    int min_y,max_y;
    int	i=0;
    
    do					       				
    {		/* 循环采样10次 */   
        Touch_GetAdXY(TP_X,TP_Y);  
        buffer[0][count]=TP_X[0];  
        buffer[1][count]=TP_Y[0];
        count++;  
    }	/*用户点击触摸屏时即TP_INT_IN信号为低 并且 count<10*/
    while(!INT_IN_2046&& count<10);
    
		
    if(INT_IN_2046)/*如果触笔弹起*/						
    {
				
        LCD240x320_touch_flag = 0;/*中断标志复位*/					 
    }
		
		/*如果成功采样10个样本*/
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
				/*去除最小值和最大值之后求平均值*/
        screen2.x=(buffer[0][0]+buffer[0][1]+buffer[0][2]+buffer[0][3]+buffer[0][4]+buffer[0][5]+buffer[0][6]+buffer[0][7]+buffer[0][8]+buffer[0][9]-min_x-max_x)>>3;
        screen2.y=(buffer[1][0]+buffer[1][1]+buffer[1][2]+buffer[1][3]+buffer[1][4]+buffer[1][5]+buffer[1][6]+buffer[1][7]+buffer[1][8]+buffer[1][9]-min_y-max_y)>>3; 
        
        return &screen2;
    }    
    return 0;    
}
/******************************************************
* 函数名：Calcu_touch_para
* 描述  ：计算出触摸屏到液晶屏坐标变换的转换函数的 K A B C D E F系数
* 输入  : 无
* 输出  ：返回1表示成功 0失败
* 举例  ：无
* 注意  ：只有在LCD和触摸屏间的误差角度非常小时,才能运用下面公式
*********************************************************/  
FunctionalState Calcu_touch_para( Coordinate * displayPtr,Coordinate * screenPtr,Parameter * para)
{    
    FunctionalState retTHRESHOLD = ENABLE ;

    /* K＝(X0－X2) (Y1－Y2)－(X1－X2) (Y0－Y2) */
    para->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                    ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;
    
		if( para->Divider == 0 )
    {
        retTHRESHOLD = DISABLE;
    }
    else
    {
        /* A＝((XD0－XD2) (Y1－Y2)－(XD1－XD2) (Y0－Y2))／K	*/
        para->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                   ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y));
        
        /* B＝((X0－X2) (XD1－XD2)－(XD0－XD2) (X1－X2))／K	*/
        para->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                   ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x));
        
        /* C＝(Y0(X2XD1－X1XD2)+Y1(X0XD2－X2XD0)+Y2(X1XD0－X0XD1))／K */
        para->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                   (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                   (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;
        
        /* D＝((YD0－YD2) (Y1－Y2)－(YD1－YD2) (Y0－Y2))／K	*/
        para->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                   ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
        
        /* E＝((X0－X2) (YD1－YD2)－(YD0－YD2) (X1－X2))／K	*/
        para->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                   ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;
        
        
        /* F＝(Y0(X2YD1－X1YD2)+Y1(X0YD2－X2YD0)+Y2(X1YD0－X0YD1))／K */
        para->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                   (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                   (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y;
        
    }
    return( retTHRESHOLD ) ;
}
/*************************************************************************
*  函数名称： Touchl_Calibrate
*  功能说明： 触摸屏校正函数
*  参数说明： 无
*
*  函数返回： 1	---	校正成功
* 				    0	---	校正失败
*  修改时间： 2014-11-2
*  备    注： CRP  
*************************************************************************/ 
u8 LCD240x320_Touch_Calibrate(void)
{
     
    uint8_t i;
    u16 test_x=0, test_y=0;
    u16 gap_x=0, gap_y=0;
    Coordinate * Ptr;   
    long double cal_p[6]={0};/* 触摸屏校正系数结构体 */
    W25Q16_FLASH_BufferRead(SPI1,&i,0,1);
		if(i==0x33)
		{		
        W25Q16_FLASH_BufferRead(SPI1,(void*)cal_p,1,sizeof(cal_p));//读取数据  
			
				aa1 =cal_p[0];/* 校正系数为全局变量 */
				bb1 =cal_p[1];
				cc1 =cal_p[2];
				aa2 =cal_p[3];
				bb2 =cal_p[4];
				cc2 =cal_p[5];
			
				return 1; 
		}
		else//如果SPI中没有保存有校正参数
		{
        	   
				for(i=0; i<4; i++)
				{        
						LCD240x320_ILI9341_Clear(0, 0,240,320, BLACK);        
						LCD240x320_ILI9341_DispStr(80,110,"Touch Calibrate......", RED,BLACK);;//字符串显示函数			
						LCD240x320_ILI9341_write_u16data(100,90,i+1,RED,BLACK);//无符号显示函数（0 ~ 65535）
										
						LCD240x320_TOUCH_Delay_ms(500);/* 适当的延时很有必要 */ 
		 
					 LCD240x320_ILI9341_Clear(DisplaySample[i].x,DisplaySample[i].y,20,1,RED); //显示校正用的“十”字	
					 LCD240x320_ILI9341_Clear(DisplaySample[i].x+10,DisplaySample[i].y-10,1,20,RED);//开窗函数					
						
						do
						{
								Ptr=Read_LCD2046_XY();        //读取XPT2046数据到变量ptr            
						}
						while( Ptr == (void*)0 );     //当ptr为空时表示没有触点被按下
						ScreenSample[i].x= Ptr->x; 	  //把读取的原始数据存放到全局变量ScreenSample结构体
						ScreenSample[i].y= Ptr->y;

				}
				
				Calcu_touch_para( &DisplaySample[0],&ScreenSample[0],&touch_para ) ;/* 用原始参数与读取参数计算出 原始参数与坐标的转换系数。 */  	   
				
				/*取一个点计算X值*/
				test_x = ( (touch_para.An * ScreenSample[3].x) + 
									 (touch_para.Bn * ScreenSample[3].y) + 
										touch_para.Cn 
								 ) / touch_para.Divider ;			 
				
				/*取一个点计算Y值*/
				test_y = ( (touch_para.Dn * ScreenSample[3].x) + 
									 (touch_para.En * ScreenSample[3].y) + 
									 touch_para.Fn 
								 ) / touch_para.Divider ;
				
				/* 实际坐标与计算坐标的差 */
				gap_x = (test_x > DisplaySample[3].x)?(test_x - DisplaySample[3].x):(DisplaySample[3].x - test_x);
				gap_y = (test_y > DisplaySample[3].y)?(test_y - DisplaySample[3].y):(DisplaySample[3].y - test_y);
		 
				LCD240x320_ILI9341_Clear(0, 0, 240, 320, BLACK);
				
			
				if((gap_x>10)||(gap_y>10))/* 可以通过修改这两个值的大小来调整精度 */
				{
			 
					 LCD240x320_ILI9341_DispStr(60,100,"Calibrate fail", RED,BLACK);;//字符串显示函数	
					 LCD240x320_ILI9341_DispStr(60,120,"try again", RED,BLACK);;//字符串显示函数				
					 LCD240x320_TOUCH_Delay_ms(2000);
					 return 0;
				}    

				/* 校正系数为全局变量 */
				aa1 = (touch_para.An*1.0)/touch_para.Divider;
				bb1 = (touch_para.Bn*1.0)/touch_para.Divider;
				cc1 = (touch_para.Cn*1.0)/touch_para.Divider;
				
				aa2 = (touch_para.Dn*1.0)/touch_para.Divider;
				bb2 = (touch_para.En*1.0)/touch_para.Divider;
				cc2 = (touch_para.Fn*1.0)/touch_para.Divider;
				
				{
					 cal_p[0]=aa1;//将数据参数写入到外部flash中保存
					 cal_p[1]=bb1;
					 cal_p[2]=cc1;
					 cal_p[3]=aa2;
					 cal_p[4]=bb2;
					 cal_p[5]=cc2;
					 i=0x33;					  
					 W25Q16_FLASH_BufferWrite(SPI1,&i, 0, 1);//数据写函数
					 W25Q16_FLASH_BufferWrite(SPI1,(void*)cal_p, 1, sizeof(cal_p));//数据写函数
				}
				
				/*
				{
				//发回到上位机显示
				UART_send_string(USART1,"\n aa1: ");//字符串函数
				UART_send_floatdat(USART1,(aa1*1000.0));//浮点数发送函数 精确到小数点后三位
				UART_send_string(USART1,"\n bb1: ");//字符串函数
				UART_send_floatdat(USART1,(bb1*1000.0));//浮点数发送函数 精确到小数点后三位
				UART_send_string(USART1,"\n cc1: ");//字符串函数
				UART_send_floatdat(USART1,(cc1*1000.0));//浮点数发送函数 精确到小数点后三位
				
				UART_send_string(USART1,"\n aa2: ");//字符串函数
				UART_send_floatdat(USART1,(aa2*1000.0));//浮点数发送函数 精确到小数点后三位
				UART_send_string(USART1,"\n bb2: ");//字符串函数
				UART_send_floatdat(USART1,(bb2*1000.0));//浮点数发送函数 精确到小数点后三位
				UART_send_string(USART1,"\n cc2: ");//字符串函数
				UART_send_floatdat(USART1,(cc2*10.0));//浮点数发送函数 精确到小数点后三位
				UART_send_string(USART1,"\n");//字符串函数
			  }*/
				
				 LCD240x320_ILI9341_DispStr(60,100,"Calibrate Succed", RED,BLACK);;//字符串显示函数	
				 LCD240x320_TOUCH_Delay_ms(1000);
			
				return 1;   
		}		
}

/*************************************************************************
*  函数名称： Touchl_Calibrate
*  功能说明： 通过 K A B C D E F 把通道X Y的值转换为液晶屏坐标
*  参数说明：  Coordinate * displayPtr   结构体变量  包含X Y 两个16位变量   用于存放读取出来的液晶坐标值
*              Parameter * para   校正系数存放结构体
*
*  函数返回： 返回1表示成功 0失败			     
*  修改时间： 2014-11-2
*  备    注： CRP   如果获取的触点信息有误，将返回DISABLE
*************************************************************************/ 
FunctionalState Get_touch_point(Coordinate * displayPtr)
{
  FunctionalState retTHRESHOLD =ENABLE ; //FunctionalState  是M3内部的状态量
  Coordinate * screenPtr=Read_LCD2046_XY();
	Parameter * para=&touch_para;
	
  if(screenPtr==0)/*如果获取的触点信息有误，则返回DISABLE*/
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
 * 画板初始化，用于取色用  
 */
 
void Palette_Init(void)
{
  /* 整屏清为白色 */
  LCD240x320_ILI9341_Clear(0, 0, 240, 320, WHITE);
  
  /* 画两条直线 */
  LCD240x320_ILI9341_Clear(29, 0, 1, 40, BLACK);
  LCD240x320_ILI9341_Clear(0, 39, 30, 1, BLACK);
   
  LCD240x320_ILI9341_DispStr(7,12,"CLR", RED,WHITE);;//字符串显示函数	
	
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
* 函数名：Palette_draw_point
* 描述  ：在LCD指定位置画一个大点(包含四个小点)
* 输入  : Xpos		--X方向位置
*         Ypos		--Y方向位置
* 输出  ：无
* 举例  ：Palette_draw_point(100,100);
* 注意  ：该函数是 "触摸画板应用实例" 专用函数
*********************************************************/    
void Palette_draw_point(uint16_t x, uint16_t y)
{
  /* 画笔默认为白色 */
  static u16 Pen_color=WHITE; 

  /* 在画板内取色 */
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
