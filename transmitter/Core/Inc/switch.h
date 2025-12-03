/**
  ******************************************************************************
  * @file           : switch.h
  * @brief          : Header for switch.c file.
  ******************************************************************************
  */

#ifndef __SWITCH_H
#define __SWITCH_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t s0;       // S0 (PB12)
    uint8_t s1_1;     // S1_1 (PB5)
    uint8_t s1_2;     // S1_2 (PB4)
    uint8_t s2_1;     // S2_1 (PB3)
    uint8_t s2_2;     // S2_2 (PA15)
    uint8_t s4_1;     // S4_1 (PB14)
    uint8_t s4_2;     // S4_2 (PB13)
    uint8_t s5_1;     // S5_1 (PA8)
    uint8_t s5_2;     // S5_2 (PB15)
} Switch_Data_t;

/* Exported functions prototypes ---------------------------------------------*/
void Switch_Init(void);
void Switch_Read(Switch_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __SWITCH_H */
