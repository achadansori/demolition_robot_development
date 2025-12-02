/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : var.c
  * @brief          : Binary data structure implementation
  ******************************************************************************
  * @attention
  *
  * Module untuk mengelola struktur data biner lengkap transmitter
  * Menyediakan interface untuk mengakses data dalam format biner
  * yang siap dikirim via LoRa
  *
  * Total data size: 8 bytes
  * - Joystick: 6 bytes
  * - Switches: 2 bytes
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "var.h"
#include "joystick.h"
#include "switch.h"
#include <string.h>

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
Transmitter_Data_t tx_data;

/* Private function prototypes -----------------------------------------------*/

/* Private user code ---------------------------------------------------------*/

/**
  * @brief  Initialize variable module
  * @retval None
  */
void Var_Init(void)
{
    // Clear all data
    memset(&tx_data, 0, sizeof(Transmitter_Data_t));

    // Initialize sub-modules
    Joystick_Init();
    Switch_Init();
}

/**
  * @brief  Update all transmitter data
  *         Call this function periodically to refresh all sensor data
  * @retval None
  */
void Var_Update(void)
{
    // Read joystick data
    Joystick_Read(&tx_data.joystick);

    // Read switch data
    Switch_Read(&tx_data.switches);
}

/**
  * @brief  Get pointer to binary data for LoRa transmission
  * @retval Pointer to binary data buffer
  */
uint8_t* Var_GetBinaryData(void)
{
    return (uint8_t*)&tx_data;
}

/**
  * @brief  Get size of binary data
  * @retval Size in bytes
  */
uint16_t Var_GetDataSize(void)
{
    return sizeof(Transmitter_Data_t);
}
