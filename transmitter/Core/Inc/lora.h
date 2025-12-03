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
#include "joystick.h"
#include "switch.h"
#include "var.h"

/* Exported types ------------------------------------------------------------*/
typedef struct {
    Joystick_Data_t joystick;
    Switch_Data_t switch_data;
    Var_Data_t var;
} LoRa_Packet_t;

typedef enum {
    LORA_MODE_NORMAL = 0,      // M0=0, M1=0 - Normal mode
    LORA_MODE_SLEEP            // M0=1, M1=1 - Configuration mode
} LoRa_Mode_t;

/* Exported constants --------------------------------------------------------*/
#define LORA_PACKET_SIZE    36    // Size of data packet in bytes

/* Exported functions prototypes ---------------------------------------------*/
void LoRa_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *m0_port, uint16_t m0_pin,
               GPIO_TypeDef *m1_port, uint16_t m1_pin);
bool LoRa_Configure(void);
bool LoRa_SendData(LoRa_Packet_t *packet);
bool LoRa_SendCSV(Joystick_Data_t *joy, Switch_Data_t *sw, Var_Data_t *var);
bool LoRa_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_H */
