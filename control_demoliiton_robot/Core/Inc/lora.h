/**
  ******************************************************************************
  * @file           : lora_receiver.h
  * @brief          : Header for LoRa receiver (E220-900T22D)
  *                   Receiver with Configuration
  ******************************************************************************
  */

#ifndef __LORA_RECEIVER_H
#define __LORA_RECEIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint16_t joy_left_x;
    uint16_t joy_left_y;
    uint8_t joy_left_btn1;
    uint8_t joy_left_btn2;
    uint16_t joy_right_x;
    uint16_t joy_right_y;
    uint8_t joy_right_btn1;
    uint8_t joy_right_btn2;
    uint8_t s0;
    uint8_t s1_1;
    uint8_t s1_2;
    uint8_t s2_1;
    uint8_t s2_2;
    uint8_t s4_1;
    uint8_t s4_2;
    uint8_t s5_1;
    uint8_t s5_2;
    uint16_t r1;
    uint16_t r8;
    uint8_t motor_active;  // Motor starter trigger (from S1_1 hold logic)
} LoRa_ReceivedData_t;

typedef enum {
    LORA_MODE_NORMAL = 0,      // M0=0, M1=0 - Normal mode
    LORA_MODE_SLEEP            // M0=1, M1=1 - Configuration mode
} LoRa_Mode_t;

/* Exported constants --------------------------------------------------------*/
#define LORA_RX_BUFFER_SIZE    256

/* Exported functions prototypes ---------------------------------------------*/
void LoRa_Receiver_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *m0_port, uint16_t m0_pin,
                        GPIO_TypeDef *m1_port, uint16_t m1_pin);
bool LoRa_Receiver_Configure(void);
void LoRa_Receiver_StartListening(void);
bool LoRa_Receiver_IsDataAvailable(void);
bool LoRa_Receiver_GetData(LoRa_ReceivedData_t *data);
void LoRa_Receiver_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_RECEIVER_H */
