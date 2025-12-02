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

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
static volatile bool new_data_flag = false;
static uint32_t last_rx_timestamp = 0;
static LoRa_RxCallback_t rx_callback = NULL;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize LoRa module for reception
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_Init(void)
{
    // Set mode pins to Normal mode (M0=0, M1=0) for lowest latency
    LoRa_SetMode(LORA_MODE_NORMAL);

    // Small delay for module stabilization
    HAL_Delay(10);

    new_data_flag = false;
    last_rx_timestamp = 0;

    // Start DMA reception immediately
    return LoRa_StartReceive();
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
  * @brief  Start receiving data via LoRa (non-blocking with DMA)
  * @retval LoRa_Status_t
  */
LoRa_Status_t LoRa_StartReceive(void)
{
    // Start UART DMA reception
    if (HAL_UART_Receive_DMA(&huart1, rx_buffer, LORA_RX_BUFFER_SIZE) != HAL_OK)
    {
        return LORA_ERROR;
    }

    return LORA_OK;
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

/**
  * @brief  UART RX complete callback (called by HAL when DMA receives data)
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        // Record timestamp
        last_rx_timestamp = HAL_GetTick();

        // Set flag
        new_data_flag = true;

        // Call user callback if registered
        if (rx_callback != NULL)
        {
            rx_callback(rx_buffer, LORA_RX_BUFFER_SIZE);
        }

        // Restart reception immediately for next packet
        HAL_UART_Receive_DMA(&huart1, rx_buffer, LORA_RX_BUFFER_SIZE);
    }
}
