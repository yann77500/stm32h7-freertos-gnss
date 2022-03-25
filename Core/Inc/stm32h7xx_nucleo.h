/*
 * stm32h7xx_nucleo.h
 *
 *  Created on: Mar 25, 2022
 *      Author: yann
 */

#ifndef INC_STM32H7XX_NUCLEO_H_
#define INC_STM32H7XX_NUCLEO_H_


#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_errno.h"
#include "main.h"


 /* COM Feature define */
 #define USE_BSP_COM_FEATURE                 1U

 /* COM define */
 #define USE_COM_LOG                         1U

 /* IRQ priorities */
 #define BSP_BUTTON_USER_IT_PRIORITY         15U

 /* I2C1 Frequeny in Hz  */
 #define BUS_I2C1_FREQUENCY                  100000U /* Frequency of I2C1 = 100 KHz*/

 /* SPI1 Baud rate in bps  */
 #define BUS_SPI1_BAUDRATE                   16000000U /* baud rate of SPIn = 16 Mbps */

 /* UART1 Baud rate in bps  */
 #define BUS_UART1_BAUDRATE                  9600U /* baud rate of UARTn = 9600 baud */


#if (USE_BSP_COM_FEATURE > 0)
  #if (USE_COM_LOG > 0)
    #if defined(__ICCARM__) || defined(__CC_ARM) /* For IAR and MDK-ARM */
      #include <stdio.h>
    #endif
  #endif
#endif
/** @addtogroup BSP
 * @{
 */

/** @defgroup STM32L4XX_NUCLEO
 * @{
 */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL
 * @{
 */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_Exported_Constants LOW LEVEL Exported Constants
  * @{
  */
/**
 * @brief STM32L4XX NUCLEO BSP Driver version number V1.0.0
 */
#define __STM32L4XX_NUCLEO_BSP_VERSION_MAIN   (uint32_t)(0x01) /*!< [31:24] main version */
#define __STM32L4XX_NUCLEO_BSP_VERSION_SUB1   (uint32_t)(0x00) /*!< [23:16] sub1 version */
#define __STM32L4XX_NUCLEO_BSP_VERSION_SUB2   (uint32_t)(0x00) /*!< [15:8]  sub2 version */
#define __STM32L4XX_NUCLEO_BSP_VERSION_RC     (uint32_t)(0x00) /*!< [7:0]  release candidate */
#define __STM32L4XX_NUCLEO_BSP_VERSION        ((__STM32L4XX_NUCLEO_BSP_VERSION_MAIN << 24)\
                                                    |(__STM32L4XX_NUCLEO_BSP_VERSION_SUB1 << 16)\
                                                    |(__STM32L4XX_NUCLEO_BSP_VERSION_SUB2 << 8 )\
                                                    |(__STM32L4XX_NUCLEO_BSP_VERSION_RC))

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_Exported_Types STM32L4XX_NUCLEO LOW LEVEL Exported Types
 * @{
 */

 /**
  * @brief Define for STM32L4XX_NUCLEO board
  */
#if !defined (USE_STM32L4XX_NUCLEO)
 #define USE_STM32L4XX_NUCLEO
#endif
#ifndef USE_BSP_COM_FEATURE
   #define USE_BSP_COM_FEATURE                  0U
#endif

/**
 * @}
 */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_COM STM32L4XX_NUCLEO LOW LEVEL COM
 * @{
 */
/**
 * @brief Definition for COM portx, connected to USART2
 */

#define BUS_USART2_INSTANCE USART2
#define BUS_USART2_TX_GPIO_AF GPIO_AF7_USART2
#define BUS_USART2_TX_GPIO_PIN GPIO_PIN_2
#define BUS_USART2_TX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_USART2_TX_GPIO_PORT GPIOA
#define BUS_USART2_TX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_USART2_RX_GPIO_PORT GPIOA
#define BUS_USART2_RX_GPIO_CLK_ENABLE() __HAL_RCC_GPIOA_CLK_ENABLE()
#define BUS_USART2_RX_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE()
#define BUS_USART2_RX_GPIO_PIN GPIO_PIN_3
#define BUS_USART2_RX_GPIO_AF GPIO_AF7_USART2

/**
 * @}
 */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_Exported_Types LOW LEVEL Exported Types
  * @{
  */
#ifndef USE_BSP_COM
  #define USE_BSP_COM                           0U
#endif

#ifndef USE_COM_LOG
  #define USE_COM_LOG                           1U
#endif

#if (USE_BSP_COM_FEATURE > 0)
typedef enum
{
  COM1 = 0U,
  COMn
}COM_TypeDef;

typedef enum
{
 COM_WORDLENGTH_8B     =   UART_WORDLENGTH_8B,
 COM_WORDLENGTH_9B     =   UART_WORDLENGTH_9B,
}COM_WordLengthTypeDef;

typedef enum
{
 COM_STOPBITS_1     =   UART_STOPBITS_1,
 COM_STOPBITS_2     =   UART_STOPBITS_2,
}COM_StopBitsTypeDef;

typedef enum
{
 COM_PARITY_NONE     =  UART_PARITY_NONE,
 COM_PARITY_EVEN     =  UART_PARITY_EVEN,
 COM_PARITY_ODD      =  UART_PARITY_ODD,
}COM_ParityTypeDef;

typedef enum
{
 COM_HWCONTROL_NONE    =  UART_HWCONTROL_NONE,
 COM_HWCONTROL_RTS     =  UART_HWCONTROL_RTS,
 COM_HWCONTROL_CTS     =  UART_HWCONTROL_CTS,
 COM_HWCONTROL_RTS_CTS =  UART_HWCONTROL_RTS_CTS,
}COM_HwFlowCtlTypeDef;

typedef struct
{
  uint32_t             BaudRate;
  COM_WordLengthTypeDef  WordLength;
  COM_StopBitsTypeDef  StopBits;
  COM_ParityTypeDef    Parity;
  COM_HwFlowCtlTypeDef HwFlowCtl;
}COM_InitTypeDef;
#endif

#define MX_UART_InitTypeDef          COM_InitTypeDef
#define MX_UART_StopBitsTypeDef      COM_StopBitsTypeDef
#define MX_UART_ParityTypeDef        COM_ParityTypeDef
#define MX_UART_HwFlowCtlTypeDef     COM_HwFlowCtlTypeDef
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
typedef struct
{
  void (* pMspInitCb)(UART_HandleTypeDef *);
  void (* pMspDeInitCb)(UART_HandleTypeDef *);
} BSP_COM_Cb_t;
#endif /* (USE_HAL_UART_REGISTER_CALLBACKS == 1U) */

/**
 * @}
 */

#define COMn                             1U
#define COM1_UART                        USART3

#define COM_POLL_TIMEOUT                 1000
extern UART_HandleTypeDef hcom_uart[COMn];
#define  huart2 hcom_uart[COM1]

/**
 * @}
 */

/**
  * @}
  */

/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_Exported_Variables LOW LEVEL Exported Constants
  * @{
  */
/**
  * @}
  */

/** @defgroup STM32L4XX_NUCLEO_LOW_LEVEL_Exported_Functions STM32L4XX_NUCLEO LOW LEVEL Exported Functions
 * @{
 */

int32_t  BSP_GetVersion(void);
#if (USE_BSP_COM_FEATURE > 0)
int32_t  BSP_COM_Init(COM_TypeDef COM);
int32_t  BSP_COM_DeInit(COM_TypeDef COM);
#endif

#if (USE_COM_LOG > 0)
int32_t  BSP_COM_SelectLogPort(COM_TypeDef COM);
#endif

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1U)
int32_t BSP_COM_RegisterDefaultMspCallbacks(COM_TypeDef COM);
int32_t BSP_COM_RegisterMspCallbacks(COM_TypeDef COM , BSP_COM_Cb_t *Callback);
#endif /* USE_HAL_UART_REGISTER_CALLBACKS */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */
#ifdef __cplusplus
}
#endif


#endif /* INC_STM32H7XX_NUCLEO_H_ */
