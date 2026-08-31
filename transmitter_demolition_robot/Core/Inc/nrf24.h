/**
  ******************************************************************************
  * @file           : nrf24.h
  * @brief          : Header for NRF24L01+ wireless module (Hardware SPI2)
  *                   STM32F407 Discovery Transmitter
  *                   CE: PC7, CSN: PC6, IRQ: PC8
  ******************************************************************************
  */

#ifndef __NRF24_H
#define __NRF24_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include <stdbool.h>
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
#define NRF24_PAYLOAD_SIZE    8  // Payload size (same as LoRa binary format)

/* Exported functions prototypes ---------------------------------------------*/
void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin,
                GPIO_TypeDef *csn_port, uint16_t csn_pin);
bool NRF24_Configure(void);
bool NRF24_SendBinary(const uint8_t* data, uint16_t size);
bool NRF24_IsConnected(void);
bool NRF24_IsReady(void);
uint8_t NRF24_GetStatus(void);
uint8_t NRF24_GetLinkQuality(void);  // Rolling link quality 0-100 from ACK results
void NRF24_PowerDown(void);          // CE low + PWR_UP=0 (CAN mode)

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_H */
