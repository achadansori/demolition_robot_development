/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : LoRa E220-900T22D Transmitter with Configuration
  *                   TX: PA9, RX: PA10, M0: PB8, M1: PB9
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lora.h"
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *huart_lora;
static GPIO_TypeDef *M0_Port = NULL;
static uint16_t M0_Pin = 0;
static GPIO_TypeDef *M1_Port = NULL;
static uint16_t M1_Pin = 0;
static bool lora_ready = false;

/* Private defines -----------------------------------------------------------*/
#define LORA_TIMEOUT        1000
#define LORA_BUFFER_SIZE    256

// Configuration parameters (MUST BE SAME for TX and RX)
#define LORA_ADDRESS        0x0000      // Device address
#define LORA_CHANNEL        23          // Channel 23 = 873.125 MHz
#define LORA_AIR_RATE       0           // Air rate 0 = 62.5kbps (FASTEST!)
#define LORA_TX_POWER       0           // 22dBm max power

/**
  * @brief  Set LoRa module mode
  * @param  mode: LORA_MODE_NORMAL or LORA_MODE_SLEEP
  * @retval None
  */
static void LoRa_SetMode(LoRa_Mode_t mode)
{
    if (M0_Port == NULL || M1_Port == NULL)
    {
        return;
    }

    if (mode == LORA_MODE_SLEEP)
    {
        // M0=1, M1=1 - Configuration mode
        HAL_GPIO_WritePin(M0_Port, M0_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(M1_Port, M1_Pin, GPIO_PIN_SET);
    }
    else
    {
        // M0=0, M1=0 - Normal mode
        HAL_GPIO_WritePin(M0_Port, M0_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(M1_Port, M1_Pin, GPIO_PIN_RESET);
    }

    HAL_Delay(50); // Wait for mode change (reduced from 100ms)
}

/**
  * @brief  Initialize LoRa module
  * @param  huart: pointer to UART handle
  * @param  m0_port: GPIO port for M0 pin
  * @param  m0_pin: GPIO pin for M0
  * @param  m1_port: GPIO port for M1 pin
  * @param  m1_pin: GPIO pin for M1
  * @retval None
  */
void LoRa_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *m0_port, uint16_t m0_pin,
               GPIO_TypeDef *m1_port, uint16_t m1_pin)
{
    huart_lora = huart;
    M0_Port = m0_port;
    M0_Pin = m0_pin;
    M1_Port = m1_port;
    M1_Pin = m1_pin;

    if (huart_lora != NULL && M0_Port != NULL && M1_Port != NULL)
    {
        // Set to normal mode
        LoRa_SetMode(LORA_MODE_NORMAL);
        lora_ready = true;
    }
}

/**
  * @brief  Configure LoRa module with default parameters
  * @note   MUST be called once during initialization
  * @retval true if successful
  */
bool LoRa_Configure(void)
{
    if (!lora_ready || huart_lora == NULL)
    {
        return false;
    }

    // Enter configuration mode
    LoRa_SetMode(LORA_MODE_SLEEP);

    uint8_t cmd_buffer[11];

    // Write configuration command
    cmd_buffer[0] = 0xC0;  // Write command
    cmd_buffer[1] = 0x00;  // Start address
    cmd_buffer[2] = 0x08;  // Length

    // ADDH - Address High byte
    cmd_buffer[3] = (LORA_ADDRESS >> 8) & 0xFF;

    // ADDL - Address Low byte
    cmd_buffer[4] = LORA_ADDRESS & 0xFF;

    // REG0 - UART: 9600bps, 8N1
    cmd_buffer[5] = 0x62;

    // REG1 - Air rate and TX power
    cmd_buffer[6] = ((LORA_AIR_RATE & 0x07) << 5) | (LORA_TX_POWER & 0x03);

    // REG2 - Channel
    cmd_buffer[7] = LORA_CHANNEL & 0x7F;

    // REG3 - RSSI enabled, FEC enabled
    cmd_buffer[8] = 0x84;

    // CRYPT - No encryption
    cmd_buffer[9] = 0x00;
    cmd_buffer[10] = 0x00;

    // Send configuration
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart_lora, cmd_buffer, 11, LORA_TIMEOUT);

    HAL_Delay(100); // Wait for configuration to be written (reduced from 200ms)

    // Return to normal mode
    LoRa_SetMode(LORA_MODE_NORMAL);

    return (status == HAL_OK);
}

/**
  * @brief  Check if LoRa module is ready
  * @retval true if ready, false otherwise
  */
bool LoRa_IsReady(void)
{
    return lora_ready;
}

/**
  * @brief  Send binary data via LoRa (FAST! No parsing needed)
  * @param  data: pointer to binary data buffer
  * @param  size: size of data in bytes
  * @retval true if successful, false otherwise
  */
bool LoRa_SendBinary(const uint8_t* data, uint16_t size)
{
    if (!lora_ready || data == NULL || huart_lora == NULL || size == 0)
    {
        return false;
    }

    // Send binary data directly via UART - NO PARSING NEEDED!
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart_lora, (uint8_t*)data, size, LORA_TIMEOUT);

    return (status == HAL_OK);
}
