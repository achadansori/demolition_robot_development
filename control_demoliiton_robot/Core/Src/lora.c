/**
  ******************************************************************************
  * @file           : lora.c
  * @brief          : LoRa E220-900T22D Receiver with Packet Sync
  *                   RX: PA10, TX: PA9, M0: PE4, M1: PE5
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
static LoRa_ReceivedData_t received_data;

/* Private defines -----------------------------------------------------------*/
#define LORA_TIMEOUT        1000

// Packet synchronization
#define PACKET_HEADER1      0xAA
#define PACKET_HEADER2      0x55
#define PACKET_DATA_SIZE    8
#define PACKET_TOTAL_SIZE   (2 + PACKET_DATA_SIZE)  // Header(2) + Data(8) - NO CHECKSUM for speed

// Configuration parameters (MUST BE SAME as transmitter)
#define LORA_ADDRESS        0x0000      // Device address
#define LORA_CHANNEL        23          // Channel 23 = 873.125 MHz
#define LORA_AIR_RATE       5           // Air rate 5
#define LORA_TX_POWER       0           // 22dBm max power

// Receiver state machine
typedef enum {
    RX_STATE_WAIT_HEADER1,
    RX_STATE_WAIT_HEADER2,
    RX_STATE_RECEIVING_DATA
} RxState_t;

static RxState_t rx_state = RX_STATE_WAIT_HEADER1;
static uint8_t rx_packet[PACKET_DATA_SIZE];
static uint8_t rx_byte;
static volatile uint8_t rx_count = 0;
static volatile bool packet_ready = false;

/* Private function prototypes -----------------------------------------------*/
static void LoRa_ParseBinaryPacket(void);
static void LoRa_SetMode(LoRa_Mode_t mode);
static uint8_t LoRa_CalculateChecksum(const uint8_t* data, uint16_t size);

/**
  * @brief  Calculate simple XOR checksum
  * @param  data: pointer to data buffer
  * @param  size: size of data
  * @retval checksum value
  */
static uint8_t LoRa_CalculateChecksum(const uint8_t* data, uint16_t size)
{
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < size; i++)
    {
        checksum ^= data[i];
    }
    return checksum;
}

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

    HAL_Delay(50);
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

    rx_state = RX_STATE_WAIT_HEADER1;
    rx_count = 0;
    packet_ready = false;
    memset(rx_packet, 0, PACKET_DATA_SIZE);
    memset(&received_data, 0, sizeof(LoRa_ReceivedData_t));

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

    HAL_Delay(100);

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
        HAL_UART_Receive_IT(huart_lora, &rx_byte, 1);
    }
}

/**
  * @brief  Check if data packet is available
  * @retval true if packet ready, false otherwise
  */
bool LoRa_Receiver_IsDataAvailable(void)
{
    return packet_ready;
}

/**
  * @brief  Get received data packet
  * @param  data: pointer to LoRa_ReceivedData_t structure
  * @retval true if successful, false otherwise
  */
bool LoRa_Receiver_GetData(LoRa_ReceivedData_t *data)
{
    if (!packet_ready || data == NULL)
    {
        return false;
    }

    // Copy data
    memcpy(data, &received_data, sizeof(LoRa_ReceivedData_t));

    // Reset flag
    packet_ready = false;

    return true;
}

/**
  * @brief  UART receive complete callback (called by HAL)
  * @note   This function should be called from HAL_UART_RxCpltCallback
  * @note   Uses state machine for packet synchronization (NO CHECKSUM for speed)
  * @retval None
  */
void LoRa_Receiver_IRQHandler(void)
{
    if (huart_lora == NULL) return;

    switch (rx_state)
    {
        case RX_STATE_WAIT_HEADER1:
            if (rx_byte == PACKET_HEADER1)  // Wait for 0xAA
            {
                rx_state = RX_STATE_WAIT_HEADER2;
            }
            break;

        case RX_STATE_WAIT_HEADER2:
            if (rx_byte == PACKET_HEADER2)  // Wait for 0x55
            {
                rx_state = RX_STATE_RECEIVING_DATA;
                rx_count = 0;
            }
            else
            {
                // Wrong header, go back to waiting
                rx_state = RX_STATE_WAIT_HEADER1;
            }
            break;

        case RX_STATE_RECEIVING_DATA:
            // Collect 8 bytes of data
            rx_packet[rx_count++] = rx_byte;

            if (rx_count >= PACKET_DATA_SIZE)
            {
                // All 8 bytes received! Parse immediately (NO CHECKSUM for speed)
                LoRa_ParseBinaryPacket();
                packet_ready = true;

                // Reset state machine for next packet
                rx_state = RX_STATE_WAIT_HEADER1;
                rx_count = 0;
            }
            break;
    }

    // Continue receiving next byte
    HAL_UART_Receive_IT(huart_lora, &rx_byte, 1);
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Parse binary packet (Direct memory copy)
  * @note   Packet format (8 bytes):
  *         Byte 0: left_x
  *         Byte 1: left_y
  *         Byte 2: right_x
  *         Byte 3: right_y
  *         Byte 4: r8
  *         Byte 5: r1
  *         Byte 6-7: switches (2 bytes bit-packed)
  * @retval None
  */
static void LoRa_ParseBinaryPacket(void)
{
    // Direct copy from packet buffer
    received_data.joy_left_x = rx_packet[0];
    received_data.joy_left_y = rx_packet[1];
    received_data.joy_right_x = rx_packet[2];
    received_data.joy_right_y = rx_packet[3];
    received_data.r8 = rx_packet[4];
    received_data.r1 = rx_packet[5];

    // Extract bit-packed switches from bytes 6-7
    uint16_t switches = (rx_packet[7] << 8) | rx_packet[6];
    received_data.joy_left_btn1 = (switches >> 0) & 0x01;
    received_data.joy_left_btn2 = (switches >> 1) & 0x01;
    received_data.joy_right_btn1 = (switches >> 2) & 0x01;
    received_data.joy_right_btn2 = (switches >> 3) & 0x01;
    received_data.s0 = (switches >> 4) & 0x01;
    received_data.s1_1 = (switches >> 5) & 0x01;
    received_data.s1_2 = (switches >> 6) & 0x01;
    received_data.s2_1 = (switches >> 7) & 0x01;
    received_data.s2_2 = (switches >> 8) & 0x01;
    received_data.s4_1 = (switches >> 9) & 0x01;
    received_data.s4_2 = (switches >> 10) & 0x01;
    received_data.s5_1 = (switches >> 11) & 0x01;
    received_data.s5_2 = (switches >> 12) & 0x01;
}
