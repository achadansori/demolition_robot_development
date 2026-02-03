/**
  ******************************************************************************
  * @file           : switch.h
  * @brief          : Header for switch.c file - Switch/Button reading
  *                   STM32F407 Discovery Transmitter
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca status semua switch dan button digital
  * Mengemas 13 input dalam struktur 2-byte yang efisien
  *
  * Pin Configuration:
  * - JOY_LEFT_BTN1:  PA3
  * - JOY_LEFT_BTN2:  PA1
  * - JOY_RIGHT_BTN1: PB1
  * - JOY_RIGHT_BTN2: PC5
  * - S0:             PC0
  * - S1_1:           PE5
  * - S1_2:           PE3
  * - S2_1:           PE1
  * - S2_2:           PB9
  * - S4_1:           PB3
  * - S4_2:           PD6
  * - S5_1:           PB7
  * - S5_2:           PB5
  *
  ******************************************************************************
  */

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
