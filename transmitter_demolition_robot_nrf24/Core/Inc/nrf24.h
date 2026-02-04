/**
  ******************************************************************************
  * @file           : nrf24.h
  * @brief          : Header for NRF24L01+ wireless module (Hardware SPI2)
  *                   Compatible with transmitter_demolition_robot_nrf24
  ******************************************************************************
  */

#ifndef __NRF24_H
#define __NRF24_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>
#include <stdint.h>

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
    uint8_t motor_active;  // Motor starter trigger
} NRF24_Data_t;

/* Exported constants --------------------------------------------------------*/
#define NRF24_PAYLOAD_SIZE    32  // Maximum payload size

/* Exported functions prototypes ---------------------------------------------*/
void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin,
                GPIO_TypeDef *csn_port, uint16_t csn_pin);
bool NRF24_Configure(void);
bool NRF24_TransmitData(NRF24_Data_t *data);
bool NRF24_IsConnected(void);
uint8_t NRF24_GetStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_H */
