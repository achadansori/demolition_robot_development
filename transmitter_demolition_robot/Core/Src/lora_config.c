/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lora_config.c
  * @brief          : E220-900T22D Configuration implementation
  ******************************************************************************
  * @attention
  *
  * Configuration utilities untuk E220-900T22D
  * Set channel, address, dan parameters via CONFIG mode
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lora_config.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define CONFIG_TIMEOUT  1000    // Timeout untuk config operations (ms)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Enter configuration mode and set parameters
  * @param  config: Configuration structure
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_Configure(LoRa_Config_t* config)
{
    uint8_t cmd[6];

    // Enter CONFIG mode (M0=0, M1=1)
    LoRa_SetMode(LORA_MODE_CONFIG);
    HAL_Delay(100); // Wait for mode switch

    // Build configuration command
    // Command format: 0xC0 0x00 0x06 [ADDH] [ADDL] [REG0] [REG1] [REG2] [REG3]
    cmd[0] = 0xC0;  // Write configuration command
    cmd[1] = 0x00;  // Start address
    cmd[2] = 0x06;  // Length (6 bytes)

    // ADDH, ADDL (Address High, Low)
    cmd[3] = (config->address >> 8) & 0xFF;   // ADDH
    cmd[4] = config->address & 0xFF;          // ADDL

    // REG0: UART rate, parity, air data rate
    cmd[5] = (config->uart_rate << 5) | (0x00 << 3) | config->air_data_rate;

    // REG1: Subpacket, RSSI ambient noise, transmit power
    cmd[6] = (config->subpacket_size << 6) | (config->rssi_ambient_noise << 5) | config->transmit_power;

    // REG2: Channel
    cmd[7] = config->channel;

    // REG3: RSSI byte, transmission method, etc (use defaults)
    cmd[8] = 0x00;  // Default: disable RSSI byte, transparent transmission

    // Send configuration
    if (HAL_UART_Transmit(&huart1, cmd, 9, CONFIG_TIMEOUT) != HAL_OK)
    {
        LoRa_SetMode(LORA_MODE_NORMAL);
        return LORA_ERROR;
    }

    // Wait for response
    HAL_Delay(50);

    // Return to NORMAL mode
    LoRa_SetMode(LORA_MODE_NORMAL);
    HAL_Delay(50);

    return LORA_OK;
}

/**
  * @brief  Read current configuration from module
  * @param  config: Configuration structure to fill
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_ReadConfig(LoRa_Config_t* config)
{
    uint8_t cmd[3] = {0xC1, 0x00, 0x06};  // Read configuration command
    uint8_t response[9];

    // Enter CONFIG mode
    LoRa_SetMode(LORA_MODE_CONFIG);
    HAL_Delay(100);

    // Send read command
    if (HAL_UART_Transmit(&huart1, cmd, 3, CONFIG_TIMEOUT) != HAL_OK)
    {
        LoRa_SetMode(LORA_MODE_NORMAL);
        return LORA_ERROR;
    }

    // Receive response
    if (HAL_UART_Receive(&huart1, response, 9, CONFIG_TIMEOUT) != HAL_OK)
    {
        LoRa_SetMode(LORA_MODE_NORMAL);
        return LORA_ERROR;
    }

    // Parse response
    if (response[0] == 0xC1)  // Response header
    {
        config->address = (response[3] << 8) | response[4];
        config->uart_rate = (response[5] >> 5) & 0x07;
        config->air_data_rate = response[5] & 0x07;
        config->subpacket_size = (response[6] >> 6) & 0x03;
        config->rssi_ambient_noise = (response[6] >> 5) & 0x01;
        config->transmit_power = response[6] & 0x03;
        config->channel = response[7];
    }

    // Return to NORMAL mode
    LoRa_SetMode(LORA_MODE_NORMAL);
    HAL_Delay(50);

    return LORA_OK;
}

/**
  * @brief  Set default configuration (optimized for low latency)
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_SetDefaultConfig(void)
{
    LoRa_Config_t config = LORA_CONFIG_DEFAULT;
    return LoRa_Configure(&config);
}
