#ifndef __LCD240x320_ILI9341_H
#define	__LCD240x320_ILI9341_H

#include "stm32f10x.h"

/***************************************************************************************

2^26 =0X0400 0000 = 64MB,每个 BANK 有4*64MB = 256MB
64MB:FSMC_Bank1_NORSRAM1:0X6000 0000 ~ 0X63FF FFFF
64MB:FSMC_Bank1_NORSRAM2:0X6400 0000 ~ 0X67FF FFFF
64MB:FSMC_Bank1_NORSRAM3:0X6800 0000 ~ 0X6BFF FFFF
64MB:FSMC_Bank1_NORSRAM4:0X6C00 0000 ~ 0X6FFF FFFF

选择BANK1-BORSRAM4 连接 TFT，地址范围为0X6C00 0000 ~ 0X6FFF FFFF
FSMC_A23 接LCD的DC(寄存器/数据选择)脚
寄存器基地址 = 0X6C00 0000
RAM基地址 = 0X6D00 0000 = 0X6C00 0000+2^23*2 = 0X6C00 0000 + 0X100 0000 = 0X6D00 0000
当选择不同的地址线时，地址要重新计算  
****************************************************************************************/


#define Bank1_LCD_C    ((u32)0x6C000000)	    //Disp Reg ADDR
#define Bank1_LCD_D    ((u32)0x6D000000)       //Disp Data ADDR       // A23 PE2

/*选定LCD指定寄存器*/
#define LCD_WR_REG(index)    ((*(__IO u16 *) (Bank1_LCD_C)) = ((u16)index))
/*往LCD GRAM写入数据*/
#define LCD_WR_Data(val)       ((*(__IO u16 *) (Bank1_LCD_D)) = ((u16)(val)))

#define LCD_ILI9341_CMD(index)       LCD_WR_REG(index)
#define LCD_ILI9341_Parameter(val)	 LCD_WR_Data(val)

#define COLUMN		240
#define PAGE		320	

// SRT 是string的缩写
#define STR_WIDTH		  6		/* 字符宽度 */
#define STR_HEIGHT		12		/* 字符高度 */

 

#define WHITE		 		 0xFFFF	/* 白色 */
#define BLACK        0x0000	/* 黑色 */
#define GREY         0xF7DE	/* 灰色 */
#define BLUE         0x001F	/* 蓝色 */
#define BLUE2        0x051F	/* 浅蓝色 */
#define RED          0xF800	/* 红色 */
#define MAGENTA      0xF81F	/* 红紫色，洋红色 */
#define GREEN        0x07E0	/* 绿色 */
#define CYAN         0x7FFF	/* 蓝绿色，青色 */
#define YELLOW       0xFFE0	/* 黄色 */

 

#define VERTICAL       0x02	/* 竖屏模式 */
#define CROSS          0x03	/* 横屏模式 */
 
extern void LCD240x320_ILI9341_Init(u8  MODE);
extern void LCD240x320_ILI9341_GramScan( uint16_t option );
extern void LCD240x320_ILI9341_Clear(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
extern void LCD240x320_ILI9341_SetCursor(uint16_t x, uint16_t y);
extern void LCD240x320_ILI9341_OpenWindow(uint16_t x, uint16_t y, uint16_t width, uint16_t height);
extern void LCD240x320_ILI9341_SetPoint(uint16_t x , uint16_t y , uint16_t color);
uint16_t LCD240x320_ILI9341_GetPoint(uint16_t x , uint16_t y);
extern void LCD240x320_ILI9341_DispChar(uint16_t x, uint16_t y, uint8_t ascii, unsigned short int fColor, unsigned short int bColor);
extern void LCD240x320_ILI9341_DispStr(uint16_t x, uint16_t y, uint8_t *pstr, unsigned short int fColor, unsigned short int bColor);
extern void LCD240x320_ILI9341_Delay(__IO uint32_t nCount);

extern void LCD240x320_ILI9341_write_floatdata(unsigned short int x, unsigned short int y,double tep_data,unsigned short int fColor, unsigned short int bColor);
extern void LCD240x320_ILI9341_write_u16data(unsigned short int x, unsigned short int y,unsigned int tep_data,unsigned short int fColor, unsigned short int bColor);
extern void LCD240x320_ILI9341_write_intdata(unsigned short int x, unsigned short int y,int tep_data,unsigned short int fColor, unsigned short int bColor);
extern void LCD240x320_ILI9341_show_GB1616(unsigned int x,unsigned int y,char c[2],unsigned short int fColor, unsigned short int bColor);
extern void LCD240x320_ILI9341_show_GB3232(unsigned int x,unsigned int y,char c[2],unsigned short int fColor, unsigned short int bColor);

extern void LCD240x320_ILI9341_show_GB3232_string(unsigned int x,unsigned int y,char *s,unsigned short int fColor, unsigned short int bColor);	
extern void LCD240x320_ILI9341_show_GB1616_string(unsigned int x,unsigned int y,char *s,unsigned short int fColor, unsigned short int bColor);
extern void LCD240x320_ILI9341_show_Image(unsigned int x,unsigned int y,unsigned short int x1,unsigned short int y1,unsigned char temp[]);

#endif /* __LCD240x320_ILI9341_H */
