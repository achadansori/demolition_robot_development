/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : joystick.h
  * @brief          : Header for joystick.c file - Joystick reading
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca data ADC dari joystick dan potentiometer
  * Mengkonversi nilai 12-bit ADC (0-4095) ke 8-bit (0-255)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "adc.h"
#include "var.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Joystick_Init(void);
void Joystick_Read(Joystick_Data_t* data);
uint16_t Joystick_ReadChannel(uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
