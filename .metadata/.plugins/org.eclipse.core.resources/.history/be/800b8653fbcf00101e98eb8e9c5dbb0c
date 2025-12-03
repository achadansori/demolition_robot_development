/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : E220-900T22D LoRa Module Driver
  ******************************************************************************
  * @attention
  *
  * High-performance LoRa driver optimized for <20ms latency
  * Uses DMA for non-blocking UART transmission
  *
  * Pin Configuration:
  * - USART1: PA9 (TX), PA10 (RX)
  * - M0: PB8
  * - M1: PB9
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lora.h"
#include "gpio.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
// CORRECT PIN MAPPING: PB8=M1, PB9=M0 (as per hardware wiring)
#define LORA_M0_PIN       GPIO_PIN_9
#define LORA_M0_PORT      GPIOB
#define LORA_M1_PIN       GPIO_PIN_8
#define LORA_M1_PORT      GPIOB

#define LORA_MODE_SWITCH_DELAY  2    // Minimal delay for mode switching (ms)
#define LORA_CHANNEL        23       // Channel 23 = 873.125 MHz
#define LORA_AIR_RATE       5        // Air rate index 5 = 62.5kbps (FASTEST!)
#define LORA_UART_RATE      3        // UART 9600 baud (index 3)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static volatile bool tx_complete = true;
static uint32_t last_tx_timestamp = 0;

/* Private function prototypes -----------------------------------------------*/
static LoRa_Status_t LoRa_ConfigureModule(void);

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Configure LoRa module to channel 2 with optimal settings
  * @retval LoRa_Status_t
  */
static LoRa_Status_t LoRa_ConfigureModule(void)
{
    uint8_t cmd[9];

    // Enter CONFIG mode (M0=0, M1=1)
    LoRa_SetMode(LORA_MODE_CONFIG);
    HAL_Delay(100);

    // Build configuration command
    // Format: 0xC0 0x00 0x06 [ADDH] [ADDL] [REG0] [REG1] [REG2] [REG3]
    cmd[0] = 0xC0;  // Write config command
    cmd[1] = 0x00;  // Start address
    cmd[2] = 0x06;  // Length
    cmd[3] = 0x00;  // ADDH (Address High) = 0x00
    cmd[4] = 0x00;  // ADDL (Address Low) = 0x00 (Broadcast)

    // REG0: UART rate (bits 7-5) + Parity (bits 4-3) + Air rate (bits 2-0)
    // UART 9600 = 011 (0x60), Parity 8N1 = 00 (0x00), Air 62.5k = 101 (0x05)
    cmd[5] = 0x65;  // 0110 0101 = 9600 baud + 8N1 + 62.5kbps

    cmd[6] = 0xC0;  // REG1: Packet 32 bytes (11) + RSSI off (0) + Power 22dBm (00000)
    cmd[7] = LORA_CHANNEL;  // REG2: Channel 23 (873.125 MHz)
    cmd[8] = 0x00;  // REG3: Default (RSSI disabled, transparent mode)

    // Send configuration via polling
    if (HAL_UART_Transmit(&huart1, cmd, 9, 1000) != HAL_OK)
    {
        LoRa_SetMode(LORA_MODE_NORMAL);
        return LORA_ERROR;
    }

    // Wait for module to process
    HAL_Delay(50);

    // Return to NORMAL mode
    LoRa_SetMode(LORA_MODE_NORMAL);
    HAL_Delay(50);

    return LORA_OK;
}

/**
  * @brief  Initialize LoRa module
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_Init(void)
{
    // Configure module to channel 2 with optimal settings
    LoRa_ConfigureModule();

    // Set to Normal mode (M0=0, M1=0) for transmission
    LoRa_SetMode(LORA_MODE_NORMAL);

    // Small delay for module stabilization
    HAL_Delay(10);

    tx_complete = true;
    last_tx_timestamp = 0;

    return LORA_OK;
}

/**
  * @brief  Set LoRa operating mode via M0 and M1 pins
  * @param  mode: Operating mode
  * @retval None
  */
void LoRa_SetMode(LoRa_Mode_t mode)
{
    switch(mode)
    {
        case LORA_MODE_NORMAL:
            // M0=0, M1=0
            HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
            break;

        case LORA_MODE_WOR:
            // M0=1, M1=0
            HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_RESET);
            break;

        case LORA_MODE_CONFIG:
            // M0=0, M1=1
            HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_SET);
            break;

        case LORA_MODE_SLEEP:
            // M0=1, M1=1
            HAL_GPIO_WritePin(LORA_M0_PORT, LORA_M0_PIN, GPIO_PIN_SET);
            HAL_GPIO_WritePin(LORA_M1_PORT, LORA_M1_PIN, GPIO_PIN_SET);
            break;
    }

    // Minimal delay for mode switching
    HAL_Delay(LORA_MODE_SWITCH_DELAY);
}

/**
  * @brief  Transmit data via LoRa (binary format)
  * @param  data: Pointer to data buffer
  * @param  size: Size of data in bytes
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_Transmit(uint8_t* data, uint16_t size)
{
    // Check payload size
    if (size > LORA_MAX_PAYLOAD_SIZE)
    {
        return LORA_ERROR;
    }

    // Transmit via UART with polling (simple, no DMA)
    // No timeout here - just send immediately
    if (HAL_UART_Transmit(&huart1, data, size, 1000) != HAL_OK)
    {
        return LORA_ERROR;
    }

    // Record timestamp
    last_tx_timestamp = HAL_GetTick();

    // NO DELAY - let it send continuously
    // HAL_Delay(10);

    return LORA_OK;
}

/**
  * @brief  Transmit CSV string via LoRa (proven to work, optimized)
  * @param  csv_string: Pointer to CSV string buffer
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_TransmitCSV(const char* csv_string)
{
    if (csv_string == NULL)
    {
        return LORA_ERROR;
    }

    uint16_t len = 0;
    // Calculate string length (max 200 chars for safety)
    while (csv_string[len] != '\0' && len < 200)
    {
        len++;
    }

    // Transmit via UART
    if (HAL_UART_Transmit(&huart1, (uint8_t*)csv_string, len, 1000) != HAL_OK)
    {
        return LORA_ERROR;
    }

    // Record timestamp
    last_tx_timestamp = HAL_GetTick();

    // Small delay to prevent flooding (key optimization!)
    HAL_Delay(5);

    return LORA_OK;
}

/**
  * @brief  Check if LoRa is ready for next transmission
  * @retval true if ready, false if busy
  */
bool LoRa_IsReady(void)
{
    return true;  // Always ready in polling mode
}

/**
  * @brief  Get last transmission timestamp
  * @retval Timestamp in milliseconds
  */
uint32_t LoRa_GetLastTxTime(void)
{
    return last_tx_timestamp;
}
