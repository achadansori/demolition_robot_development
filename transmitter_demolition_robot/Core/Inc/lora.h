/**
  ******************************************************************************
  * @file           : lora.h
  * @brief          : Header for lora.c file (E220-900T22D LoRa Module)
  *                   Transmitter with Configuration
  ******************************************************************************
  */

#ifndef __LORA_H
#define __LORA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef enum {
    LORA_MODE_NORMAL = 0,      // M0=0, M1=0 - Normal mode
    LORA_MODE_SLEEP            // M0=1, M1=1 - Configuration mode
} LoRa_Mode_t;

/* Exported functions prototypes ---------------------------------------------*/
void LoRa_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *m0_port, uint16_t m0_pin,
               GPIO_TypeDef *m1_port, uint16_t m1_pin);
bool LoRa_Configure(void);
bool LoRa_SendCSVString(const char* csv_string);  // Send CSV format string
bool LoRa_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_H */
