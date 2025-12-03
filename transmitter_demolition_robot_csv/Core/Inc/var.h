/**
  ******************************************************************************
  * @file           : var.h
  * @brief          : Header for var.c file (Variable Resistor/Potentiometer)
  ******************************************************************************
  */

#ifndef __VAR_H
#define __VAR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint16_t r1;      // Raw ADC value R1 (PA1 - ADC_CHANNEL_1)
    uint16_t r8;      // Raw ADC value R8 (PA0 - ADC_CHANNEL_0)
} Var_Data_t;

/* Exported constants --------------------------------------------------------*/
#define VAR_R1_CHANNEL    ADC_CHANNEL_1    // PA1
#define VAR_R8_CHANNEL    ADC_CHANNEL_0    // PA0

/* Exported functions prototypes ---------------------------------------------*/
void Var_Init(ADC_HandleTypeDef *hadc);
void Var_Read(Var_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __VAR_H */
