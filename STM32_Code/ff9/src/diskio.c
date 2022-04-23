/*-----------------------------------------------------------------------*/
/* Low level disk I/O module skeleton for FatFs     (C)ChaN, 2007        */
/*-----------------------------------------------------------------------*/
/* This is a stub disk I/O module that acts as front end of the existing */
/* disk I/O modules and attach it to FatFs module with common interface. */
/*-----------------------------------------------------------------------*/
/**************************************************************************

 注： 移植的时候需要自己修改 diskio.c 文件下的底层驱动函数 （SD卡初始化、单块的读写、多块的读写）
 
 各函数功能：
 
		 disk_status - Get device status
		 disk_initialize - Initialize device
		 disk_read - Read sector(s)
		 disk_write - Write sector(s)
		 disk_ioctl - Control device dependent features
		 get_fattime - Get current time


 注： 工程文件下面有文件系统的帮助文档  会指明文件系统下的函数如何调用 或查网站

 fatfs 文件系统：http://elm-chan.org/fsw/ff/00index_e.html

 crp
**************************************************************************/


#include "diskio.h"
#include "stm32f10x.h"
#include "SDIO_SD.h"

#define BLOCK_SIZE            512 /* Block Size in Bytes */



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
	BYTE drv				/* Physical drive nmuber (0..) */
)
{
	SD_Error  Status;
	
	/* Supports only single drive */
	if (drv)
	{
		return STA_NOINIT;
	}
/*-------------------------- SD Init ----------------------------- */
  Status = SD_Init();
	if (Status!=SD_OK )
	{
		return STA_NOINIT;
	}
	else
	{
		return RES_OK;
	}

}



/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
	BYTE drv		/* Physical drive nmuber (0..) */
)
{
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE *buff,		/* Data buffer to store read data */
	DWORD sector,	/* Sector address (LBA) */
	BYTE count		/* Number of sectors to read (1..255) */
)
{

	if (count > 1)
	{
		SD_ReadMultiBlocks(buff, sector*BLOCK_SIZE, BLOCK_SIZE, count);
	
			  /* Check if the Transfer is finished */
	     SD_WaitReadOperation();  //循环查询dma传输是否结束
	
	    /* Wait until end of DMA transfer */
	    while(SD_GetStatus() != SD_TRANSFER_OK);

	}
	else
	{
		
		SD_ReadBlock(buff, sector*BLOCK_SIZE, BLOCK_SIZE);

			  /* Check if the Transfer is finished */
	     SD_WaitReadOperation();  //循环查询dma传输是否结束
	
	    /* Wait until end of DMA transfer */
	    while(SD_GetStatus() != SD_TRANSFER_OK);

	}
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if _READONLY == 0
DRESULT disk_write (
	BYTE drv,			/* Physical drive nmuber (0..) */
	const BYTE *buff,	/* Data to be written */
	DWORD sector,		/* Sector address (LBA) */
	BYTE count			/* Number of sectors to write (1..255) */
)
{

	if (count > 1)
	{
		SD_WriteMultiBlocks((uint8_t *)buff, sector*BLOCK_SIZE, BLOCK_SIZE, count);
		
		  /* Check if the Transfer is finished */
	  	 SD_WaitWriteOperation();	   //等待dma传输结束
	    while(SD_GetStatus() != SD_TRANSFER_OK); //等待sdio到sd卡传输结束
	}
	else
	{
		SD_WriteBlock((uint8_t *)buff,sector*BLOCK_SIZE, BLOCK_SIZE);
		
		  /* Check if the Transfer is finished */
	   		SD_WaitWriteOperation();	   //等待dma传输结束
	    while(SD_GetStatus() != SD_TRANSFER_OK); //等待sdio到sd卡传输结束
	}
	return RES_OK;
}
#endif /* _READONLY */




/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
	BYTE drv,		/* Physical drive nmuber (0..) */
	BYTE ctrl,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	return RES_OK;
}
							 
/*-----------------------------------------------------------------------*/
/* Get current time                                                      */
/*-----------------------------------------------------------------------*/ 
DWORD get_fattime(void)
{

 	return 0;

} 
