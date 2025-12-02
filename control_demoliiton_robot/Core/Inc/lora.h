/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lora.h
  * @brief          : Header for lora.c file - E220-900T22D LoRa Module (Receiver)
  ******************************************************************************
  * @attention
  *
  * LoRa E220-900T22D Driver - Receiver Side
  * Optimized for low latency reception (<20ms)
  *
  * Hardware (STM32F407 Discovery):
  * - UART: PB6 (TX), PB7 (RX) - USART1
  * - M0: PE4 (Mode control 0)
  * - M1: PE5 (Mode control 1)
  *
  * Modes:
  * - M0=0, M1=0: Normal mode (transmit/receive)
  * - M0=1, M1=0: WOR mode (wake on radio)
  * - M0=0, M1=1: Configuration mode
  * - M0=1, M1=1: Deep sleep
  *
  * For lowest latency: Use Normal mode with DMA reception
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LORA_H
#define __LORA_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include "main.h"
#include "usart.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief LoRa operating modes
 */
typedef enum {
    LORA_MODE_NORMAL = 0,      // M0=0, M1=0 - Normal TX/RX
    LORA_MODE_WOR    = 1,      // M0=1, M1=0 - Wake on Radio
    LORA_MODE_CONFIG = 2,      // M0=0, M1=1 - Configuration
    LORA_MODE_SLEEP  = 3       // M0=1, M1=1 - Deep sleep
} LoRa_Mode_t;

/**
 * @brief LoRa status
 */
typedef enum {
    LORA_OK = 0,
    LORA_ERROR,
    LORA_BUSY,
    LORA_TIMEOUT
} LoRa_Status_t;

/**
 * @brief Receiver callback function type
 */
typedef void (*LoRa_RxCallback_t)(uint8_t* data, uint16_t size);

/* Exported constants --------------------------------------------------------*/
#define LORA_MAX_PAYLOAD_SIZE   240    // E220 max payload
#define LORA_PACKET_SIZE        8      // Our data packet size

/* Exported macro ------------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Initialize LoRa module for reception
 * @retval LoRa_Status_t
 */
LoRa_Status_t LoRa_Init(void);

/**
 * @brief  Set LoRa operating mode
 * @param  mode: Operating mode
 * @retval None
 */
void LoRa_SetMode(LoRa_Mode_t mode);

/**
 * @brief  Start receiving data via LoRa (non-blocking with DMA)
 * @retval LoRa_Status_t
 */
LoRa_Status_t LoRa_StartReceive(void);

/**
 * @brief  Register callback for received data
 * @param  callback: Callback function pointer
 * @retval None
 */
void LoRa_RegisterRxCallback(LoRa_RxCallback_t callback);

/**
 * @brief  Get last reception timestamp
 * @retval Timestamp in milliseconds
 */
uint32_t LoRa_GetLastRxTime(void);

/**
 * @brief  Check if new data has been received
 * @retval true if new data available
 */
bool LoRa_HasNewData(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_H */
