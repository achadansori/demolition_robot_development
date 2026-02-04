/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : switch.h
  * @brief          : Header for switch.c file - Switch/Button reading
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca status semua switch dan button digital
  * Mengemas 13 input dalam struktur 2-byte yang efisien
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __SWITCH_H
#define __SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "gpio.h"
#include "var.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Switch_Init(void);
void Switch_Read(Switch_Data_t* data);
bool Switch_ReadPin(GPIO_TypeDef* port, uint16_t pin);

#ifdef __cplusplus
}
#endif

#endif /* __SWITCH_H */
