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
  * [4] = battery     (PA0 - IN0) via voltage divider
  *
  * Note: PA2 (was R8 pot) removed from ADC scan to eliminate floating-pin
  * cross-talk noise that caused joystick spikes to 100%.
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
#define ADC_CHANNELS    5   // Number of ADC channels (PA2 removed)

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern uint16_t adc_buffer[ADC_CHANNELS];
extern int8_t joy_cal_offset[4];  // Calibration offset per axis (LX, LY, RY, RX)

/* Exported functions prototypes ---------------------------------------------*/
void Joystick_Init(void);
void Joystick_Read(Joystick_Data_t* data);
void Joystick_StartDMA(void);
void Joystick_Calibrate(void);

#ifdef __cplusplus
}
#endif

#endif /* __JOYSTICK_H */
