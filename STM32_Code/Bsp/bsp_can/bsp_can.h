#ifndef __BSP_CAN_H
#define	__BSP_CAN_H

#include "stm32f4xx.h"
#include "stm32f4xx_can.h"
#include <stdio.h>
#include "string.h"


/*CAN1端口定义*/
#define CAN1_CLK                    RCC_APB1Periph_CAN1
#define CAN1_RX_IRQ				    CAN1_RX0_IRQn
#define CAN1_RX_IRQHandler		    CAN1_RX0_IRQHandler

#define CAN1_RX_PIN                 GPIO_Pin_0
#define CAN1_TX_PIN                 GPIO_Pin_1
#define CAN1_TX_GPIO_PORT           GPIOD
#define CAN1_RX_GPIO_PORT           GPIOD
#define CAN1_TX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CAN1_RX_GPIO_CLK            RCC_AHB1Periph_GPIOD
#define CAN1_AF_PORT                GPIO_AF_CAN1
#define CAN1_RX_SOURCE              GPIO_PinSource0
#define CAN1_TX_SOURCE              GPIO_PinSource1 

/*CAN2端口定义*/
#define CAN2_CLK                    RCC_APB1Periph_CAN1 |RCC_APB1Periph_CAN2
#define CAN2_RX_IRQ				    CAN2_RX0_IRQn
#define CAN2_RX_IRQHandler		    CAN2_RX0_IRQHandler

#define CAN2_RX_PIN                 GPIO_Pin_12
#define CAN2_TX_PIN                 GPIO_Pin_13
#define CAN2_TX_GPIO_PORT           GPIOB
#define CAN2_RX_GPIO_PORT           GPIOB
#define CAN2_TX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN2_RX_GPIO_CLK            RCC_AHB1Periph_GPIOB
#define CAN2_AF_PORT                GPIO_AF_CAN2
#define CAN2_RX_SOURCE              GPIO_PinSource12
#define CAN2_TX_SOURCE              GPIO_PinSource13 

/*debug*/
#define CAN_DEBUG_ON          1
#define CAN_DEBUG_ARRAY_ON   1
#define CAN_DEBUG_FUNC_ON    1

// Log define
#define CAN_INFO(fmt,arg...)           printf("<<-CAN-INFO->> "fmt"\n",##arg)
#define CAN_ERROR(fmt,arg...)          printf("<<-CAN-ERROR->> "fmt"\n",##arg)
#define CAN_DEBUG(fmt,arg...)          do{\
                                         if(CAN_DEBUG_ON)\
                                         printf("<<-CAN-DEBUG->> [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)

#define CAN_DEBUG_ARRAY(array, num)    do{\
                                         int32_t i;\
                                         uint8_t* a = array;\
                                         if(CAN_DEBUG_ARRAY_ON)\
                                         {\
                                            printf("<<-CAN-DEBUG-ARRAY->>\n");\
                                            for (i = 0; i < (num); i++)\
                                            {\
                                                printf("%02x   ", (a)[i]);\
                                                if ((i + 1 ) %10 == 0)\
                                                {\
                                                    printf("\n");\
                                                }\
                                            }\
                                            printf("\n");\
                                        }\
                                       }while(0)

#define CAN_DEBUG_FUNC()               do{\
                                         if(CAN_DEBUG_FUNC_ON)\
                                         printf("<<-CAN-FUNC->> Func:%s@Line:%d\n",__func__,__LINE__);\
                                       }while(0)


static void CAN_GPIO_Config(CAN_TypeDef* CANx);
static void CAN_NVIC_Config(CAN_TypeDef* CANx);
static void CAN_Mode_Config(CAN_TypeDef* CANx);
static void CAN_Filter_Config(CAN_TypeDef* CANx);
void CAN_Config(CAN_TypeDef* CANx);
void Init_RxMes(CanRxMsg* RxMessage);
void CAN_SetMsg(CanTxMsg* TxMessage,uint8_t *data_can,uint8_t data_num,uint32_t MSG_ID,uint8_t Ext,uint8_t RMT);                                 
#endif
