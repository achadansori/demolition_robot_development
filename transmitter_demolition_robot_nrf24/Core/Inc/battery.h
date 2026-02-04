/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : battery.h
  * @brief          : Header for battery.c file - Battery voltage monitoring
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca tegangan baterai 7.2V melalui voltage divider
  * Voltage divider: 10kΩ dan 20kΩ (rasio 1:3)
  * Vout = Vin / 3
  *
  * Battery specs:
  * - Fully charged: 8.4V (1.4V per cell × 6 cells)
  * - Nominal: 7.2V (1.2V per cell × 6 cells)
  * - Discharged: 6.0V (1.0V per cell × 6 cells)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __BATTERY_H
#define __BATTERY_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "adc.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
// Battery voltage thresholds (in millivolts)
#define BATTERY_FULL_MV         8400    // 8.4V - Fully charged
#define BATTERY_NOMINAL_MV      7200    // 7.2V - Nominal voltage
#define BATTERY_LOW_MV          6600    // 6.6V - Low battery warning
#define BATTERY_CRITICAL_MV     6000    // 6.0V - Critical battery (shut down recommended)

// Voltage divider ratio (Vout = Vin / 3)
#define VOLTAGE_DIVIDER_RATIO   3

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Battery_Init(void);
uint16_t Battery_ReadADC(void);
uint16_t Battery_GetVoltage_mV(void);
uint8_t Battery_GetPercentage(void);
uint8_t Battery_IsLow(void);
uint8_t Battery_IsCritical(void);

#ifdef __cplusplus
}
#endif

#endif /* __BATTERY_H */
