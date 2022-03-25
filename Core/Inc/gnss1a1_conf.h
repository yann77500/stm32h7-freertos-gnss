/**
 ******************************************************************************
 * @file    gnss1a1_conf.h
 * @author  SRA
 * @brief   This file contains definitions for the GNSS components bus interfaces
 ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef GNSS1A1_CONF_H
#define GNSS1A1_CONF_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_bus.h"



#define USE_I2C 1U

#define USE_GNSS1A1_GNSS_TESEO_LIV3F	1U


#define GNSS1A1_GNSS_UART_ClearOREF   BSP_USART1_ClearOREF


//#define GPIOA    								1
//#define GPIO_PIN_8 								1
//#define GPIO_PIN_5 								1




#define GNSS1A1_RST_PORT                        GPIOD
#define GNSS1A1_RST_PIN                         GPIO_PIN_15

#define GNSS1A1_WAKEUP_PORT                     GPIOA
#define GNSS1A1_WAKEUP_PIN                      GPIO_PIN_5


#define GNSS1A1_GNSS_I2C_Init        			BSP_I2C1_Init
#define GNSS1A1_GNSS_I2C_DeInit      			BSP_I2C1_DeInit
#define GNSS1A1_GNSS_I2C_Transmit_IT 			BSP_I2C1_Send_IT
#define GNSS1A1_GNSS_I2C_Receive_IT  			BSP_I2C1_Recv_IT
#define GNSS1A1_GNSS_GetTick         			BSP_GetTick

//#define GNSS1A1_RegisterDefaultMspCallbacks     BSP_I2C1_RegisterDefaultMspCallbacks
#define GNSS1A1_RegisterRxCb                    BSP_I2C1_RegisterRxCallback
#define GNSS1A1_RegisterErrorCb                 BSP_I2C1_RegisterErrorCallback
#define GNSS1A1_RegisterAbortCb                 BSP_I2C1_RegisterAbortCallback

/* To be checked */
#define GNSS1A1_I2C_EV_IRQHanlder      BSP_I2C1_EV_IRQHanlder
#define GNSS1A1_I2C_ER_IRQHanlder      BSP_I2C1_ER_IRQHanlder




/* To be checked */
#define GNSS1A1_UART_IRQHanlder                 BSP_USART1_IRQHanlder

#ifdef __cplusplus
}
#endif

#endif /* GNSS1A1_CONF_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
