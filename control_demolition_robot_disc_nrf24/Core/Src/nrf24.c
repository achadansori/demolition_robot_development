/**
  ******************************************************************************
  * @file           : nrf24.c
  * @brief          : NRF24L01+ receiver driver implementation
  *                   Receiver mode (PRIM_RX=1)
  ******************************************************************************
  */

#include "nrf24.h"
#include <string.h>

/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef *nrf24_hspi;
static GPIO_TypeDef *nrf24_ce_port;
static uint16_t nrf24_ce_pin;
static GPIO_TypeDef *nrf24_csn_port;
static uint16_t nrf24_csn_pin;

static uint8_t rx_buffer[32];
static uint8_t data_available = 0;

/* Private function prototypes -----------------------------------------------*/
static void NRF24_CSN_LOW(void);
static void NRF24_CSN_HIGH(void);
static void NRF24_CE_LOW(void);
static void NRF24_CE_HIGH(void);
static uint8_t NRF24_WriteRegister(uint8_t reg, uint8_t value);
static uint8_t NRF24_ReadRegister(uint8_t reg);
static void NRF24_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length);
static void NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Set CSN pin LOW
  */
static void NRF24_CSN_LOW(void)
{
    HAL_GPIO_WritePin(nrf24_csn_port, nrf24_csn_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set CSN pin HIGH
  */
static void NRF24_CSN_HIGH(void)
{
    HAL_GPIO_WritePin(nrf24_csn_port, nrf24_csn_pin, GPIO_PIN_SET);
}

/**
  * @brief  Set CE pin LOW
  */
static void NRF24_CE_LOW(void)
{
    HAL_GPIO_WritePin(nrf24_ce_port, nrf24_ce_pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set CE pin HIGH
  */
static void NRF24_CE_HIGH(void)
{
    HAL_GPIO_WritePin(nrf24_ce_port, nrf24_ce_pin, GPIO_PIN_SET);
}

/**
  * @brief  Write single byte to register
  */
static uint8_t NRF24_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t status;
    uint8_t cmd = NRF24_CMD_W_REGISTER | (reg & 0x1F);

    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &status, 1, 100);
    HAL_SPI_Transmit(nrf24_hspi, &value, 1, 100);
    NRF24_CSN_HIGH();

    return status;
}

/**
  * @brief  Read single byte from register
  */
static uint8_t NRF24_ReadRegister(uint8_t reg)
{
    uint8_t status, value;
    uint8_t cmd = NRF24_CMD_R_REGISTER | (reg & 0x1F);

    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &status, 1, 100);
    HAL_SPI_Receive(nrf24_hspi, &value, 1, 100);
    NRF24_CSN_HIGH();

    return value;
}

/**
  * @brief  Write multiple bytes to register
  */
static void NRF24_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t status;
    uint8_t cmd = NRF24_CMD_W_REGISTER | (reg & 0x1F);

    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &status, 1, 100);
    HAL_SPI_Transmit(nrf24_hspi, data, length, 100);
    NRF24_CSN_HIGH();
}

/**
  * @brief  Read multiple bytes from register
  */
static void NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t length)
{
    uint8_t status;
    uint8_t cmd = NRF24_CMD_R_REGISTER | (reg & 0x1F);

    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &status, 1, 100);
    HAL_SPI_Receive(nrf24_hspi, data, length, 100);
    NRF24_CSN_HIGH();
}

/**
  * @brief  Send command to NRF24
  */
static uint8_t NRF24_SendCommand(uint8_t cmd)
{
    uint8_t status;

    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &status, 1, 100);
    NRF24_CSN_HIGH();

    return status;
}

/* Public functions ----------------------------------------------------------*/

/**
  * @brief  Initialize NRF24L01+ module
  */
void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin,
                GPIO_TypeDef *csn_port, uint16_t csn_pin)
{
    nrf24_hspi = hspi;
    nrf24_ce_port = ce_port;
    nrf24_ce_pin = ce_pin;
    nrf24_csn_port = csn_port;
    nrf24_csn_pin = csn_pin;

    // Initialize control pins
    NRF24_CE_LOW();
    NRF24_CSN_HIGH();

    HAL_Delay(100); // Power on reset
}

/**
  * @brief  Configure NRF24L01+ for receiver mode
  * @retval true if configuration successful
  */
bool NRF24_Configure(void)
{
    NRF24_CE_LOW();
    HAL_Delay(5);

    // Power down first
    NRF24_WriteRegister(NRF24_REG_CONFIG, 0x00);
    HAL_Delay(5);

    // Configure channel 76 (2476 MHz) - same as transmitter
    NRF24_WriteRegister(NRF24_REG_RF_CH, 76);

    // Configure RF: 1Mbps, 0dBm (same as working loopback test)
    NRF24_WriteRegister(NRF24_REG_RF_SETUP, 0x06);

    // Configure address width: 5 bytes
    NRF24_WriteRegister(NRF24_REG_SETUP_AW, 0x03);

    // Enable Auto-ACK on pipe 0 - MUST match the transmitter (Enhanced
    // ShockBurst). With EN_AA=0 the on-air packet format differs from the
    // TX's, so the CRC never validates and RX_DR never fires (no data). The
    // ACK is sent autonomously by the radio hardware, no SPI busy-wait.
    NRF24_WriteRegister(NRF24_REG_EN_AA, 0x01);

    // Enable RX pipe 0
    NRF24_WriteRegister(NRF24_REG_EN_RXADDR, 0x01);

    // Set RX payload size: 8 bytes
    NRF24_WriteRegister(NRF24_REG_RX_PW_P0, 8);

    // No retransmit (auto-ack disabled)
    NRF24_WriteRegister(NRF24_REG_SETUP_RETR, 0x00);

    // Set RX address pipe 0 (must match TX address)
    uint8_t rx_addr[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
    NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, rx_addr, 5);

    // Set TX address (for ACK packets)
    NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, rx_addr, 5);

    // Clear status flags
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);

    // Flush FIFOs
    NRF24_SendCommand(NRF24_CMD_FLUSH_RX);
    NRF24_SendCommand(NRF24_CMD_FLUSH_TX);

    // Configure as receiver: PWR_UP=1, PRIM_RX=1, EN_CRC=1, 2-byte CRC
    NRF24_WriteRegister(NRF24_REG_CONFIG,
                        NRF24_CONFIG_PWR_UP |
                        NRF24_CONFIG_PRIM_RX |
                        NRF24_CONFIG_EN_CRC |
                        NRF24_CONFIG_CRCO);

    HAL_Delay(5); // Wait for power up

    // Verify configuration
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    if ((config & (NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX)) !=
        (NRF24_CONFIG_PWR_UP | NRF24_CONFIG_PRIM_RX))
    {
        return false; // Configuration failed
    }

    return true;
}

/**
  * @brief  Start listening for incoming data
  */
void NRF24_StartListening(void)
{
    // Clear status flags
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);

    // Flush RX FIFO
    NRF24_SendCommand(NRF24_CMD_FLUSH_RX);

    // Set CE HIGH to start receiving
    NRF24_CE_HIGH();
    HAL_Delay(1); // Must be high for at least 130µs
}

/**
  * @brief  Check if data is available in RX FIFO
  * @retval true if data available
  */
bool NRF24_IsDataAvailable(void)
{
    uint8_t status = NRF24_ReadRegister(NRF24_REG_STATUS);

    // If SPI returns 0x00 or 0xFF, NRF24 is not responding
    if (status == 0x00 || status == 0xFF)
    {
        return false;
    }

    // Only use RX_DR flag — most reliable indicator of new data
    if (status & NRF24_STATUS_RX_DR)
    {
        return true;
    }

    return false;
}

/**
  * @brief  Get received data from RX FIFO
  * @param  data: Pointer to data structure
  * @retval true if data successfully retrieved
  */
bool NRF24_GetData(NRF24_ReceivedData_t *data)
{
    uint8_t payload[8];
    uint8_t status;
    uint8_t cmd = NRF24_CMD_R_RX_PAYLOAD;

    // Read payload from RX FIFO
    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(nrf24_hspi, &cmd, &status, 1, 100);
    HAL_SPI_Receive(nrf24_hspi, payload, 8, 100);
    NRF24_CSN_HIGH();

    // Clear RX_DR flag after reading
    NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);

    // Flush RX FIFO to prevent stale data on next read
    NRF24_SendCommand(NRF24_CMD_FLUSH_RX);

    NRF24_DecodePayload(payload, data);

    return true;
}

/**
  * @brief  Unpack the 8-byte control packet into the working struct
  * @param  payload: the 8 bytes as they arrived, from any transport
  * @param  data: destination
  * @note   Nothing here is radio-specific - the wired CAN path carries the
  *         identical layout (see ctrl_link.h), so both transports decode here
  *         and the byte/bit map lives in exactly one place.
  * @retval None
  */
void NRF24_DecodePayload(const uint8_t payload[8], NRF24_ReceivedData_t *data)
{
    // Parse binary payload (same format as transmitter)
    // Direct copy from packet buffer (8 bytes format)
    data->joy_left_x = payload[0];
    data->joy_left_y = payload[1];
    data->joy_right_x = payload[2];
    data->joy_right_y = payload[3];
    data->r8 = payload[4];
    data->r1 = payload[5];

    // Extract bit-packed switches from bytes 6-7
    uint16_t switches = (payload[7] << 8) | payload[6];
    data->joy_left_btn1 = (switches >> 0) & 0x01;
    data->joy_left_btn2 = (switches >> 1) & 0x01;
    data->joy_right_btn1 = (switches >> 2) & 0x01;
    data->joy_right_btn2 = (switches >> 3) & 0x01;
    data->s0 = (switches >> 4) & 0x01;
    data->s1_1 = (switches >> 5) & 0x01;
    data->s1_2 = (switches >> 6) & 0x01;
    data->s2_1 = (switches >> 7) & 0x01;
    data->s2_2 = (switches >> 8) & 0x01;
    data->s4_1 = (switches >> 9) & 0x01;
    data->s4_2 = (switches >> 10) & 0x01;
    data->s5_1 = (switches >> 11) & 0x01;
    data->s5_2 = (switches >> 12) & 0x01;
    data->motor_active = (switches >> 13) & 0x01;
    data->unlocked = (switches >> 14) & 0x01;
}

/**
  * @brief  Read a register (public wrapper for diagnostics)
  */
uint8_t NRF24_ReadReg(uint8_t reg)
{
    return NRF24_ReadRegister(reg);
}

/**
  * @brief  Interrupt handler for NRF24 (optional)
  *         Call this from GPIO EXTI interrupt if using IRQ pin
  */
void NRF24_IRQHandler(void)
{
    uint8_t status = NRF24_ReadRegister(NRF24_REG_STATUS);

    // Check if data received
    if (status & NRF24_STATUS_RX_DR)
    {
        data_available = 1;

        // Clear RX_DR flag
        NRF24_WriteRegister(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);
    }

    // Clear other interrupt flags
    if (status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT))
    {
        NRF24_WriteRegister(NRF24_REG_STATUS, status & 0x70);
    }
}
