/*
 * stm32_bus_ex.h
 *
 *  Created on: Mar 25, 2022
 *      Author: yann
 */

#ifndef INC_STM32_BUS_EX_H_
#define INC_STM32_BUS_EX_H_




#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "stm32h7xx_nucleo_errno.h"
#include "stm32h7xx_nucleo.h"

/* Defines -------------------------------------------------------------------*/

#define BUS_I2C1_EV_IRQn        I2C1_EV_IRQn
#define BUS_I2C1_ER_IRQn        I2C1_ER_IRQn

int32_t BSP_I2C1_Send_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
int32_t BSP_I2C1_Recv_IT(uint16_t DevAddr, uint8_t *pData, uint16_t Length);
#if (USE_HAL_I2C_REGISTER_CALLBACKS == 1)
int32_t BSP_I2C1_RegisterRxCallback(pI2C_CallbackTypeDef pCallback);
int32_t BSP_I2C1_RegisterErrorCallback(pI2C_CallbackTypeDef pCallback);
int32_t BSP_I2C1_RegisterAbortCallback(pI2C_CallbackTypeDef pCallback);
#endif

void BSP_EV_I2C1_IRQHanlder(void);
void BSP_ER_I2C1_IRQHanlder(void);

#endif /* INC_STM32_BUS_EX_H_ */
