/**
  ******************************************************************************
  * @file           : var.c
  * @brief          : Binary data structure implementation
  *                   STM32F407 Discovery Transmitter with NRF24
  ******************************************************************************
  * @attention
  *
  * Module untuk mengelola struktur data biner lengkap transmitter
  * Menyediakan interface untuk mengakses data dalam format biner
  * yang siap dikirim via NRF24
  *
  * Total data size: 8 bytes
  * - Joystick: 6 bytes
  * - Switches: 2 bytes
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "var.h"
#include "joystick.h"
#include "switch.h"
#include <string.h>

/* Exported variables --------------------------------------------------------*/
Transmitter_Data_t tx_data;

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
    // Read joystick data (uses DMA buffer)
    Joystick_Read(&tx_data.joystick);

    // Read switch data
    Switch_Read(&tx_data.switches);

    // Freshness counter (byte 5): must keep incrementing even in SLEEP or
    // EMERGENCY so the control board can tell "live link carrying safe values"
    // apart from "stale data replayed by a hung bridge". The main-loop safety
    // overrides never touch this byte.
    static uint8_t tx_counter = 0;
    tx_data.joystick.tx_counter = ++tx_counter;

    // Packet signature (bits 14-15 of the switch field). Switch_Read clears
    // it, so stamp it here where the packet is finalized.
    tx_data.switches.reserved = VAR_PACKET_SIGNATURE;
}

/**
  * @brief  Get pointer to binary data for NRF24 transmission
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
