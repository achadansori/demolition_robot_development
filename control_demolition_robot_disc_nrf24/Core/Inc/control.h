/**
  ******************************************************************************
  * @file           : control.h
  * @brief          : Demolition Robot Control Logic
  *                   Maps LoRa joystick/switch inputs to PWM outputs
  ******************************************************************************
  */

#ifndef __CONTROL_H
#define __CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "nrf24.h"
#include <stdbool.h>

/* Public function prototypes ------------------------------------------------*/
void Control_Init(void);
void Control_Update(NRF24_ReceivedData_t *nrf24_data);
void Control_RequireMotorRestart(void);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_H */
