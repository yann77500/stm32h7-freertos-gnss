/**
  ******************************************************************************
  * @file  : app_gnss.h
  * @brief : Header file
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
#ifndef APP_GNSS_H
#define APP_GNSS_H
#define CLOUD_UPLOAD_INTERVAL_MS       9000

#include <string.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef double float64_t;
#define INTERFACE_UART   1
#define INTERFACE_I2C   2


/* Includes ------------------------------------------------------------------*/

/* Exported Functions --------------------------------------------------------*/
void MX_GNSS_Init(void);
char* tostring(char str[], int num);
uint64_t toUint64(uint8_t val);
int UploadSensorData(float64_t lat,float64_t lng , float64_t alt);
#ifdef __cplusplus
}
#endif
#endif /* APP_GNSS_H */
