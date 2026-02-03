/**
  ******************************************************************************
  * @file           : joystick.h
  * @brief          : Header for joystick.c file - ADC DMA reading
  *                   STM32F407 Discovery Transmitter
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca data ADC via DMA dari joystick dan potentiometer
  * Mengkonversi nilai 12-bit ADC (0-4095) ke 8-bit (0-255)
  *
  * DMA Buffer Order (sesuai Rank di CubeMX):
  * [0] = joy_left_y  (PC1 - IN11)
  * [1] = joy_left_x  (PC3 - IN13)
  * [2] = joy_right_y (PA5 - IN5)
  * [3] = joy_right_x (PA7 - IN7)
  * [4] = r1          (PA0 - IN0)
  * [5] = r8          (PA2 - IN2)
  *
  ******************************************************************************
  */

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
#define ADC_CHANNELS    6   // Number of ADC channels

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern uint16_t adc_buffer[ADC_CHANNELS];

/* Exported functions prototypes ---------------------------------------------*/
void Joystick_Init(void);
void Joystick_Read(Joystick_Data_t* data);
void Joystick_StartDMA(void);

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
