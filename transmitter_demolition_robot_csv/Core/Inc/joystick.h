/**
  ******************************************************************************
  * @file           : joystick.h
  * @brief          : Header for joystick.c file.
  ******************************************************************************
  */

#ifndef __JOYSTICK_H
#define __JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint16_t left_x;      // Raw ADC value Left X (PA3 - ADC_CHANNEL_3)
    uint16_t left_y;      // Raw ADC value Left Y (PA2 - ADC_CHANNEL_2)
    uint8_t left_btn1;    // Left Button 1 (PA5) - 1=pressed, 0=released
    uint8_t left_btn2;    // Left Button 2 (PA4) - 1=pressed, 0=released
    uint16_t right_x;     // Raw ADC value Right X (PA7 - ADC_CHANNEL_7)
    uint16_t right_y;     // Raw ADC value Right Y (PA6 - ADC_CHANNEL_6)
    uint8_t right_btn1;   // Right Button 1 (PB1) - 1=pressed, 0=released
    uint8_t right_btn2;   // Right Button 2 (PB0) - 1=pressed, 0=released
} Joystick_Data_t;

/* Exported constants --------------------------------------------------------*/
#define JOY_LEFT_X_CHANNEL     ADC_CHANNEL_3    // PA3
#define JOY_LEFT_Y_CHANNEL     ADC_CHANNEL_2    // PA2
#define JOY_RIGHT_X_CHANNEL    ADC_CHANNEL_7    // PA7
#define JOY_RIGHT_Y_CHANNEL    ADC_CHANNEL_6    // PA6

/* Exported functions prototypes ---------------------------------------------*/
void Joystick_Init(ADC_HandleTypeDef *hadc);
void Joystick_Read(Joystick_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
