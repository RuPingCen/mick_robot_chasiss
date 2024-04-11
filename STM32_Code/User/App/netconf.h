/**
  ******************************************************************************
  * @file    netconf.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013 
  * @brief   This file contains all the functions prototypes for the netconf.c 
  *          file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __NETCONF_H
#define __NETCONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
	 
/* Exported types ------------------------------------------------------------*/
/* DHCP状态 */
#define DHCP_START                 1
#define DHCP_WAIT_ADDRESS          2
#define DHCP_ADDRESS_ASSIGNED      3
#define DHCP_TIMEOUT               4
#define DHCP_LINK_DOWN             5

//#define USE_DHCP       /* enable DHCP, if disabled static address is used */

/* 调试信息输出 */  
#define SERIAL_DEBUG 
/* 远端IP地址和端口 */
#define DEST_IP_ADDR0               192
#define DEST_IP_ADDR1               168
#define DEST_IP_ADDR2                 1
#define DEST_IP_ADDR3               105
#define DEST_PORT                  6000

/* MAC地址：网卡地址 */
#define MAC_ADDR0                     2
#define MAC_ADDR1                     0
#define MAC_ADDR2                     0
#define MAC_ADDR3                     0
#define MAC_ADDR4                     0
#define MAC_ADDR5                     0

/*静态IP地址 */
#define IP_ADDR0                    192
#define IP_ADDR1                    168
#define IP_ADDR2                      1
#define IP_ADDR3                    122

/* 子网掩码 */
#define NETMASK_ADDR0               255
#define NETMASK_ADDR1               255
#define NETMASK_ADDR2               255
#define NETMASK_ADDR3                 0

/* 网关 */
#define GW_ADDR0                    192
#define GW_ADDR1                    168
#define GW_ADDR2                      1
#define GW_ADDR3                      1

/* 检测PHY链路状态的实际间隔(单位：ms) */
#ifndef LINK_TIMER_INTERVAL
#define LINK_TIMER_INTERVAL        1000
#endif

/* MII and RMII mode selection ***********/
#define RMII_MODE  
//#define MII_MODE

/* 在MII模式时，使能MCO引脚输出25MHz脉冲 */
#ifdef 	MII_MODE
 #define PHY_CLOCK_MCO
#endif
	 
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void LwIP_Init(void);
void LwIP_Pkt_Handle(void);
void LwIP_Periodic_Handle(__IO uint32_t localtime);

#ifdef __cplusplus
}
#endif

#endif /* __NETCONF_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

