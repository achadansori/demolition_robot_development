/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : LoRa E220-900T22D Receiver with Configuration
  *                   RX: PA10, TX: PA9, M0: PB6, M1: PB7
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lora.h"
#include <string.h>
#include <stdlib.h>

/* Private variables ---------------------------------------------------------*/
static UART_HandleTypeDef *huart_lora;
static GPIO_TypeDef *M0_Port = NULL;
static uint16_t M0_Pin = 0;
static GPIO_TypeDef *M1_Port = NULL;
static uint16_t M1_Pin = 0;
static uint8_t rx_buffer[LORA_RX_BUFFER_SIZE];
static uint8_t rx_data;
static volatile uint16_t rx_index = 0;
static volatile bool data_ready = false;
static LoRa_ReceivedData_t received_data;

/* Private defines -----------------------------------------------------------*/
#define LORA_TIMEOUT        1000

// Configuration parameters (MUST BE SAME as transmitter)
#define LORA_ADDRESS        0x0000      // Device address
#define LORA_CHANNEL        23          // Channel 23 = 873.125 MHz
#define LORA_AIR_RATE       5           // Air rate 19.2kbps (FASTER! was 2)
#define LORA_TX_POWER       0           // 22dBm max power

/* Private function prototypes -----------------------------------------------*/
static bool LoRa_ParseCSV(char *buffer);
static void LoRa_SetMode(LoRa_Mode_t mode);

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
  * @brief  Initialize LoRa receiver module
  * @param  huart: pointer to UART handle
  * @param  m0_port: GPIO port for M0 pin
  * @param  m0_pin: GPIO pin for M0
  * @param  m1_port: GPIO port for M1 pin
  * @param  m1_pin: GPIO pin for M1
  * @retval None
  */
void LoRa_Receiver_Init(UART_HandleTypeDef *huart, GPIO_TypeDef *m0_port, uint16_t m0_pin,
                        GPIO_TypeDef *m1_port, uint16_t m1_pin)
{
    huart_lora = huart;
    M0_Port = m0_port;
    M0_Pin = m0_pin;
    M1_Port = m1_port;
    M1_Pin = m1_pin;

    rx_index = 0;
    data_ready = false;
    memset(rx_buffer, 0, LORA_RX_BUFFER_SIZE);

    if (huart_lora != NULL && M0_Port != NULL && M1_Port != NULL)
    {
        // Set to normal mode
        LoRa_SetMode(LORA_MODE_NORMAL);
    }
}

/**
  * @brief  Configure LoRa module with default parameters
  * @note   MUST be called once during initialization
  * @note   Configuration MUST BE SAME as transmitter
  * @retval true if successful
  */
bool LoRa_Receiver_Configure(void)
{
    if (huart_lora == NULL || M0_Port == NULL || M1_Port == NULL)
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
  * @brief  Start listening for LoRa data (using interrupt)
  * @retval None
  */
void LoRa_Receiver_StartListening(void)
{
    if (huart_lora != NULL)
    {
        // Start receiving data in interrupt mode (one byte at a time)
        HAL_UART_Receive_IT(huart_lora, &rx_data, 1);
    }
}

/**
  * @brief  Check if data is available
  * @retval true if data is ready, false otherwise
  */
bool LoRa_Receiver_IsDataAvailable(void)
{
    return data_ready;
}

/**
  * @brief  Get received data
  * @param  data: pointer to LoRa_ReceivedData_t structure
  * @retval true if successful, false otherwise
  */
bool LoRa_Receiver_GetData(LoRa_ReceivedData_t *data)
{
    if (!data_ready || data == NULL)
    {
        return false;
    }

    // Copy data
    memcpy(data, &received_data, sizeof(LoRa_ReceivedData_t));

    // Reset flag
    data_ready = false;

    return true;
}

/**
  * @brief  UART receive complete callback (called by HAL)
  * @note   This function should be called from HAL_UART_RxCpltCallback
  * @retval None
  */
void LoRa_Receiver_IRQHandler(void)
{
    if (huart_lora == NULL)
    {
        return;
    }

    // Store received byte
    rx_buffer[rx_index++] = rx_data;

    // Check for end of line (CSV format ends with \n)
    if (rx_data == '\n' || rx_index >= LORA_RX_BUFFER_SIZE - 1)
    {
        // Null-terminate the string
        rx_buffer[rx_index] = '\0';

        // Parse CSV data immediately (no delay)
        if (rx_index > 10) // Valid data check (minimal length)
        {
            if (LoRa_ParseCSV((char*)rx_buffer))
            {
                data_ready = true;
            }
        }

        // Reset buffer index
        rx_index = 0;
    }

    // Continue receiving next byte immediately
    HAL_UART_Receive_IT(huart_lora, &rx_data, 1);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Parse CSV string and extract data
  * @param  buffer: CSV string buffer
  * @retval true if successful, false otherwise
  */
static bool LoRa_ParseCSV(char *buffer)
{
    if (buffer == NULL)
    {
        return false;
    }

    // Temporary storage for parsed values (19 fields now)
    int values[19];
    int count = 0;
    char *token;

    // Parse CSV using strtok
    token = strtok(buffer, ",\r\n");
    while (token != NULL && count < 19)
    {
        values[count++] = atoi(token);
        token = strtok(NULL, ",\r\n");
    }

    // Check if we got all 19 values
    if (count != 19)
    {
        return false;
    }

    // Store parsed data
    received_data.joy_left_x = (uint16_t)values[0];
    received_data.joy_left_y = (uint16_t)values[1];
    received_data.joy_left_btn1 = (uint8_t)values[2];
    received_data.joy_left_btn2 = (uint8_t)values[3];
    received_data.joy_right_x = (uint16_t)values[4];
    received_data.joy_right_y = (uint16_t)values[5];
    received_data.joy_right_btn1 = (uint8_t)values[6];
    received_data.joy_right_btn2 = (uint8_t)values[7];
    received_data.s0 = (uint8_t)values[8];
    received_data.s1_1 = (uint8_t)values[9];
    received_data.s1_2 = (uint8_t)values[10];
    received_data.s2_1 = (uint8_t)values[11];
    received_data.s2_2 = (uint8_t)values[12];
    received_data.s4_1 = (uint8_t)values[13];
    received_data.s4_2 = (uint8_t)values[14];
    received_data.s5_1 = (uint8_t)values[15];
    received_data.s5_2 = (uint8_t)values[16];
    received_data.r1 = (uint16_t)values[17];
    received_data.r8 = (uint16_t)values[18];

    return true;
}
