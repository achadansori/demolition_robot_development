/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lora_config.h
  * @brief          : E220-900T22D Configuration utilities
  ******************************************************************************
  * @attention
  *
  * Helper functions untuk konfigurasi E220-900T22D via AT commands
  * Gunakan ini untuk set channel, address, dan parameters
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __LORA_CONFIG_H
#define __LORA_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lora.h"

/* Exported types ------------------------------------------------------------*/

/**
 * @brief E220 Configuration structure
 */
typedef struct {
    uint16_t address;           // Module address (0x0000 - 0xFFFF)
    uint8_t channel;            // Channel number (0-80, for 900MHz: 850.125MHz + CH*1MHz)
    uint8_t air_data_rate;      // 0=2.4k, 1=4.8k, 2=9.6k, 3=19.2k, 4=38.4k, 5=62.5k (default)
    uint8_t subpacket_size;     // 0=240bytes, 1=128, 2=64, 3=32
    uint8_t rssi_ambient_noise; // 0=disable, 1=enable
    uint8_t transmit_power;     // 0=22dBm, 1=17dBm, 2=13dBm, 3=10dBm
    uint8_t uart_rate;          // 0=1200, 1=2400, 2=4800, 3=9600(default), 4=19200, 5=38400, 6=57600, 7=115200
} LoRa_Config_t;

/* Exported constants --------------------------------------------------------*/

// Recommended configuration for low latency
#define LORA_CONFIG_DEFAULT { \
    .address = 0x0000,        /* Broadcast address */ \
    .channel = 23,            /* 873MHz (850.125 + 23) */ \
    .air_data_rate = 5,       /* 62.5kbps (fastest) */ \
    .subpacket_size = 3,      /* 32 bytes (smallest, fastest) */ \
    .rssi_ambient_noise = 0,  /* Disabled for speed */ \
    .transmit_power = 0,      /* 22dBm (max power) */ \
    .uart_rate = 7            /* 115200 baud (fastest) */ \
}

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  Enter configuration mode and set parameters
 * @param  config: Configuration structure
 * @retval LoRa_Status_t
 */
LoRa_Status_t LoRa_Configure(LoRa_Config_t* config);

/**
 * @brief  Read current configuration from module
 * @param  config: Configuration structure to fill
 * @retval LoRa_Status_t
 */
LoRa_Status_t LoRa_ReadConfig(LoRa_Config_t* config);

/**
 * @brief  Set default configuration (optimized for low latency)
 * @retval LoRa_Status_t
 */
LoRa_Status_t LoRa_SetDefaultConfig(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_CONFIG_H */
