/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : E220-900T22D LoRa Module Driver (Receiver)
  ******************************************************************************
  * @attention
  *
  * High-performance LoRa driver for receiving data
  * Uses DMA for non-blocking UART reception with circular buffer
  *
  * Pin Configuration (STM32F407):
  * - USART1: PB6 (TX), PB7 (RX)
  * - M0: PE4
  * - M1: PE5
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "lora.h"
#include "gpio.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
#define LORA_M0_PIN       GPIO_PIN_4
#define LORA_M0_PORT      GPIOE
#define LORA_M1_PIN       GPIO_PIN_5
#define LORA_M1_PORT      GPIOE

#define LORA_MODE_SWITCH_DELAY  2    // Minimal delay for mode switching (ms)
#define LORA_RX_BUFFER_SIZE     LORA_PACKET_SIZE
#define LORA_CHANNEL        23       // Channel 23 = 873.125 MHz (SAME as transmitter!)
#define LORA_AIR_RATE       5        // Air rate index 5 = 62.5kbps (FASTEST!)
#define LORA_UART_RATE      3        // UART 9600 baud (index 3)

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
static volatile bool new_data_flag = false;
static uint32_t last_rx_timestamp = 0;
static LoRa_RxCallback_t rx_callback = NULL;

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

    // Build configuration command (SAME as transmitter!)
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
    cmd[7] = LORA_CHANNEL;  // REG2: Channel 23 (873.125 MHz) - MUST MATCH TRANSMITTER!
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
  * @brief  Initialize LoRa module for reception
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_Init(void)
{
    // Configure module to channel 23 with optimal settings (same as transmitter)
    LoRa_ConfigureModule();

    // Set to Normal mode (M0=0, M1=0) for reception
    LoRa_SetMode(LORA_MODE_NORMAL);

    // Small delay for module stabilization
    HAL_Delay(10);

    new_data_flag = false;
    last_rx_timestamp = 0;

    return LORA_OK;  // Ready for polling receive
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
  * @brief  Start receiving data via LoRa (polling mode - no DMA)
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_StartReceive(void)
{
    // Receive via UART with polling (simple, blocking)
    // Timeout 100ms - adjust if needed
    if (HAL_UART_Receive(&huart1, rx_buffer, LORA_RX_BUFFER_SIZE, 100) == HAL_OK)
    {
        // Data received successfully
        last_rx_timestamp = HAL_GetTick();
        new_data_flag = true;

        // Call user callback if registered
        if (rx_callback != NULL)
        {
            rx_callback(rx_buffer, LORA_RX_BUFFER_SIZE);
        }

        return LORA_OK;
    }

    return LORA_TIMEOUT;  // No data received
}

/**
  * @brief  Register callback for received data
  * @param  callback: Callback function pointer
  * @retval None
  */
void LoRa_RegisterRxCallback(LoRa_RxCallback_t callback)
{
    rx_callback = callback;
}

/**
  * @brief  Get last reception timestamp
  * @retval Timestamp in milliseconds
  */
uint32_t LoRa_GetLastRxTime(void)
{
    return last_rx_timestamp;
}

/**
  * @brief  Check if new data has been received
  * @retval true if new data available
  */
bool LoRa_HasNewData(void)
{
    if (new_data_flag)
    {
        new_data_flag = false;
        return true;
    }
    return false;
}
