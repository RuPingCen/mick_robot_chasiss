/**
  ******************************************************************************
  * @file    stm32f4x7_eth_bsp.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    31-July-2013 
  * @brief   Header for stm32f4x7_eth_bsp.c file.
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
#ifndef __STM32F4X7_ETH_BSP_H
#define __STM32F4X7_ETH_BSP_H

#ifdef __cplusplus
 extern "C" {
#endif
	 
#include "stm32f4x7_eth.h"
#include "netif.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* վ����ӿڣ�SMI
 * վ����ӿ�(SMI)����Ӧ�ó���ͨ��2���ߣ�ʱ��(MDC)��������(MDIO)��������PHY�Ĵ�����
 * �ýӿ�֧�ַ��ʶ��32��PHY��Ӧ�ó�����Դ�32��PHY��ѡ��һ��PHY��Ȼ�������PHY��
 * ����32���Ĵ�����ѡ��һ���Ĵ��������Ϳ������ݻ����״̬��Ϣ���������ʱ����ֻ��
 * ��һ��PHY�е�һ���Ĵ�������Ѱַ����MAC��PHY���ж�д������ʱ��Ӧ�ó������޸�
 * MII�ĵ�ַ�Ĵ�����MII�����ݼĴ������ڴ��ڼ��MII��ַ�Ĵ�����MII���ݼĴ���ִ�е�
 * д�������ᱻ���ԡ�
 * ��������ʹ�õ�LAN8742AоƬ��Ӧ����һ��PHY��ռ��������һ����ַ
 * ���������ַȷ���������Բο�LAN8742A�ֲᡰ3.7.1 PHYAD[0]: PHY Address 
 * Configuration��˵����
 */
#define ETHERNET_PHY_ADDRESS       0x00 /* ����Ӳ�������� */
	 
/*
	ETH_MDIO -------------------------> PA2
	ETH_MDC --------------------------> PC1
	ETH_MII_RX_CLK/ETH_RMII_REF_CLK---> PA1
	ETH_MII_RX_DV/ETH_RMII_CRS_DV ----> PA7
	ETH_MII_RXD0/ETH_RMII_RXD0 -------> PC4
	ETH_MII_RXD1/ETH_RMII_RXD1 -------> PC5
	ETH_MII_TX_EN/ETH_RMII_TX_EN -----> PG11
	ETH_MII_TXD0/ETH_RMII_TXD0 -------> PG13
	ETH_MII_TXD1/ETH_RMII_TXD1 -------> PG14
																						*/
/* ETH_MDIO */
#define ETH_MDIO_GPIO_CLK               RCC_AHB1Periph_GPIOA
#define ETH_MDIO_PORT                   GPIOA
#define ETH_MDIO_PIN                    GPIO_Pin_2
#define ETH_MDIO_AF                     GPIO_AF_ETH
#define ETH_MDIO_SOURCE                 GPIO_PinSource2

/* ETH_MDC */
#define ETH_MDC_GPIO_CLK                RCC_AHB1Periph_GPIOC
#define ETH_MDC_PORT                    GPIOC
#define ETH_MDC_PIN                     GPIO_Pin_1
#define ETH_MDC_AF                      GPIO_AF_ETH
#define ETH_MDC_SOURCE                  GPIO_PinSource1

/* ETH_RMII_REF_CLK */
#define ETH_RMII_REF_CLK_GPIO_CLK       RCC_AHB1Periph_GPIOA
#define ETH_RMII_REF_CLK_PORT           GPIOA
#define ETH_RMII_REF_CLK_PIN            GPIO_Pin_1
#define ETH_RMII_REF_CLK_AF             GPIO_AF_ETH
#define ETH_RMII_REF_CLK_SOURCE         GPIO_PinSource1

/* ETH_RMII_CRS_DV */
#define ETH_RMII_CRS_DV_GPIO_CLK        RCC_AHB1Periph_GPIOA
#define ETH_RMII_CRS_DV_PORT            GPIOA
#define ETH_RMII_CRS_DV_PIN             GPIO_Pin_7
#define ETH_RMII_CRS_DV_AF              GPIO_AF_ETH
#define ETH_RMII_CRS_DV_SOURCE          GPIO_PinSource7

/* ETH_RMII_RXD0 */
#define ETH_RMII_RXD0_GPIO_CLK          RCC_AHB1Periph_GPIOC
#define ETH_RMII_RXD0_PORT              GPIOC
#define ETH_RMII_RXD0_PIN               GPIO_Pin_4
#define ETH_RMII_RXD0_AF                GPIO_AF_ETH
#define ETH_RMII_RXD0_SOURCE            GPIO_PinSource4

/* ETH_RMII_RXD1 */
#define ETH_RMII_RXD1_GPIO_CLK          RCC_AHB1Periph_GPIOC
#define ETH_RMII_RXD1_PORT              GPIOC
#define ETH_RMII_RXD1_PIN               GPIO_Pin_5
#define ETH_RMII_RXD1_AF                GPIO_AF_ETH
#define ETH_RMII_RXD1_SOURCE            GPIO_PinSource5

/* ETH_RMII_TX_EN */
#define ETH_RMII_TX_EN_GPIO_CLK         RCC_AHB1Periph_GPIOG
#define ETH_RMII_TX_EN_PORT             GPIOG
#define ETH_RMII_TX_EN_PIN              GPIO_Pin_11
#define ETH_RMII_TX_EN_AF               GPIO_AF_ETH
#define ETH_RMII_TX_EN_SOURCE           GPIO_PinSource11

/* ETH_RMII_TXD0 */
#define ETH_RMII_TXD0_GPIO_CLK          RCC_AHB1Periph_GPIOG
#define ETH_RMII_TXD0_PORT              GPIOG
#define ETH_RMII_TXD0_PIN               GPIO_Pin_13
#define ETH_RMII_TXD0_AF                GPIO_AF_ETH
#define ETH_RMII_TXD0_SOURCE            GPIO_PinSource13

/* ETH_RMII_TXD1 */
#define ETH_RMII_TXD1_GPIO_CLK          RCC_AHB1Periph_GPIOG
#define ETH_RMII_TXD1_PORT              GPIOG
#define ETH_RMII_TXD1_PIN               GPIO_Pin_14
#define ETH_RMII_TXD1_AF                GPIO_AF_ETH
#define ETH_RMII_TXD1_SOURCE            GPIO_PinSource14

/* Ethernet Flags for EthStatus variable */   
#define ETH_INIT_FLAG           0x01 /* Ethernet Init Flag */
#define ETH_LINK_FLAG           0x10 /* Ethernet Link Flag */

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void  ETH_BSP_Config(void);
void ETH_link_callback(struct netif *netif);
void ETH_CheckLinkStatus(uint16_t PHYAddress);

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4X7_ETH_BSP_H */


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
