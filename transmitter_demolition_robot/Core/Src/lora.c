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
#define LORA_AIR_RATE       5           // Air rate 19.2kbps (FASTER! was 2)
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
  * @brief  Send binary data packet via LoRa
  * @param  packet: pointer to LoRa_Packet_t structure
  * @retval true if successful, false otherwise
  */
bool LoRa_SendData(LoRa_Packet_t *packet)
{
    if (!lora_ready || packet == NULL || huart_lora == NULL)
    {
        return false;
    }

    uint8_t buffer[LORA_PACKET_SIZE];
    uint8_t idx = 0;

    // Pack joystick data (12 bytes)
    buffer[idx++] = (packet->joystick.left_x >> 8) & 0xFF;
    buffer[idx++] = packet->joystick.left_x & 0xFF;
    buffer[idx++] = (packet->joystick.left_y >> 8) & 0xFF;
    buffer[idx++] = packet->joystick.left_y & 0xFF;
    buffer[idx++] = packet->joystick.left_btn1;
    buffer[idx++] = packet->joystick.left_btn2;
    buffer[idx++] = (packet->joystick.right_x >> 8) & 0xFF;
    buffer[idx++] = packet->joystick.right_x & 0xFF;
    buffer[idx++] = (packet->joystick.right_y >> 8) & 0xFF;
    buffer[idx++] = packet->joystick.right_y & 0xFF;
    buffer[idx++] = packet->joystick.right_btn1;
    buffer[idx++] = packet->joystick.right_btn2;

    // Pack switch data (8 bytes)
    buffer[idx++] = packet->switch_data.s1_1;
    buffer[idx++] = packet->switch_data.s1_2;
    buffer[idx++] = packet->switch_data.s2_1;
    buffer[idx++] = packet->switch_data.s2_2;
    buffer[idx++] = packet->switch_data.s4_1;
    buffer[idx++] = packet->switch_data.s4_2;
    buffer[idx++] = packet->switch_data.s5_1;
    buffer[idx++] = packet->switch_data.s5_2;

    // Pack variable resistor data (4 bytes)
    buffer[idx++] = (packet->var.r1 >> 8) & 0xFF;
    buffer[idx++] = packet->var.r1 & 0xFF;
    buffer[idx++] = (packet->var.r8 >> 8) & 0xFF;
    buffer[idx++] = packet->var.r8 & 0xFF;

    // Send via UART
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart_lora, buffer, LORA_PACKET_SIZE, LORA_TIMEOUT);

    return (status == HAL_OK);
}

/**
  * @brief  Send CSV formatted data via LoRa
  * @param  joy: pointer to Joystick_Data_t structure
  * @param  sw: pointer to Switch_Data_t structure
  * @param  var: pointer to Var_Data_t structure
  * @retval true if successful, false otherwise
  */
bool LoRa_SendCSV(Joystick_Data_t *joy, Switch_Data_t *sw, Var_Data_t *var)
{
    if (!lora_ready || joy == NULL || sw == NULL || var == NULL || huart_lora == NULL)
    {
        return false;
    }

    char csv_buffer[LORA_BUFFER_SIZE];
    int len;

    // Format CSV string (19 fields now with s0 added)
    len = snprintf(csv_buffer, sizeof(csv_buffer),
                   "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
                   joy->left_x,
                   joy->left_y,
                   joy->left_btn1,
                   joy->left_btn2,
                   joy->right_x,
                   joy->right_y,
                   joy->right_btn1,
                   joy->right_btn2,
                   sw->s0,
                   sw->s1_1,
                   sw->s1_2,
                   sw->s2_1,
                   sw->s2_2,
                   sw->s4_1,
                   sw->s4_2,
                   sw->s5_1,
                   sw->s5_2,
                   var->r1,
                   var->r8);

    // Send via UART
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart_lora, (uint8_t*)csv_buffer, len, LORA_TIMEOUT);

    return (status == HAL_OK);
}

/**
  * @brief  Send CSV string directly via LoRa (wrapper for compatibility)
  * @param  csv_string: pointer to CSV string buffer
  * @retval true if successful, false otherwise
  */
bool LoRa_SendCSVString(const char* csv_string)
{
    if (!lora_ready || csv_string == NULL || huart_lora == NULL)
    {
        return false;
    }

    uint16_t len = 0;
    // Calculate string length (max 256 chars for safety)
    while (csv_string[len] != '\0' && len < LORA_BUFFER_SIZE)
    {
        len++;
    }

    // Send via UART
    HAL_StatusTypeDef status = HAL_UART_Transmit(huart_lora, (uint8_t*)csv_string, len, LORA_TIMEOUT);

    return (status == HAL_OK);
}
