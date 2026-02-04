/**
  ******************************************************************************
  * @file           : spi.h
  * @brief          : This file contains the headers of the SPI initialization.
  ******************************************************************************
  */

#ifndef __SPI_H
#define __SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2;

/* Exported functions prototypes ---------------------------------------------*/
void MX_SPI2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H */
