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
#include <stdio.h>

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

/**
  * @brief  Get CSV format string (proven to work with LoRa)
  * @retval Pointer to static CSV string buffer
  */
const char* Var_GetCSVString(void)
{
    static char csv_buffer[150];

    // Format MUST match receiver: lx,ly,lb1,lb2,rx,ry,rb1,rb2,s0,s1_1,s1_2,s2_1,s2_2,s4_1,s4_2,s5_1,s5_2,r1,r8\r\n
    snprintf(csv_buffer, sizeof(csv_buffer),
             "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
             tx_data.joystick.left_x,
             tx_data.joystick.left_y,
             tx_data.switches.joy_left_btn1,
             tx_data.switches.joy_left_btn2,
             tx_data.joystick.right_x,
             tx_data.joystick.right_y,
             tx_data.switches.joy_right_btn1,
             tx_data.switches.joy_right_btn2,
             tx_data.switches.s0,
             tx_data.switches.s1_1,
             tx_data.switches.s1_2,
             tx_data.switches.s2_1,
             tx_data.switches.s2_2,
             tx_data.switches.s4_1,
             tx_data.switches.s4_2,
             tx_data.switches.s5_1,
             tx_data.switches.s5_2,
             tx_data.joystick.r1,
             tx_data.joystick.r8);

    return csv_buffer;
}
