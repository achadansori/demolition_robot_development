/**
  ******************************************************************************
  * @file           : nrf24.c
  * @brief          : NRF24L01+ Driver for STM32F407 Discovery (Hardware SPI2)
  *                   Transmitter Mode with Auto-ACK
  *                   CE: PC7, CSN: PC6, IRQ: PC8
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "nrf24.h"
#include "main.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/
// NRF24L01+ Commands
#define NRF24_CMD_R_REGISTER        0x00
#define NRF24_CMD_W_REGISTER        0x20
#define NRF24_CMD_R_RX_PAYLOAD      0x61
#define NRF24_CMD_W_TX_PAYLOAD      0xA0
#define NRF24_CMD_FLUSH_TX          0xE1
#define NRF24_CMD_FLUSH_RX          0xE2
#define NRF24_CMD_REUSE_TX_PL       0xE3
#define NRF24_CMD_NOP               0xFF

// NRF24L01+ Registers
#define NRF24_REG_CONFIG            0x00
#define NRF24_REG_EN_AA             0x01
#define NRF24_REG_EN_RXADDR         0x02
#define NRF24_REG_SETUP_AW          0x03
#define NRF24_REG_SETUP_RETR        0x04
#define NRF24_REG_RF_CH             0x05
#define NRF24_REG_RF_SETUP          0x06
#define NRF24_REG_STATUS            0x07
#define NRF24_REG_OBSERVE_TX        0x08
#define NRF24_REG_RPD               0x09
#define NRF24_REG_RX_ADDR_P0        0x0A
#define NRF24_REG_RX_ADDR_P1        0x0B
#define NRF24_REG_TX_ADDR           0x10
#define NRF24_REG_RX_PW_P0          0x11
#define NRF24_REG_FIFO_STATUS       0x17
#define NRF24_REG_DYNPD             0x1C
#define NRF24_REG_FEATURE           0x1D

// CONFIG register bits
#define NRF24_CONFIG_PRIM_RX        0x01
#define NRF24_CONFIG_PWR_UP         0x02
#define NRF24_CONFIG_CRC0           0x04
#define NRF24_CONFIG_EN_CRC         0x08
#define NRF24_CONFIG_MASK_MAX_RT    0x10
#define NRF24_CONFIG_MASK_TX_DS     0x20
#define NRF24_CONFIG_MASK_RX_DR     0x40

// STATUS register bits
#define NRF24_STATUS_TX_FULL        0x01
#define NRF24_STATUS_RX_P_NO        0x0E
#define NRF24_STATUS_MAX_RT         0x10
#define NRF24_STATUS_TX_DS          0x20
#define NRF24_STATUS_RX_DR          0x40

// RF_SETUP register bits
#define NRF24_RF_SETUP_RF_PWR_0dBm  0x06  // 0dBm (highest power)

/* Private variables ---------------------------------------------------------*/
static SPI_HandleTypeDef *hspi_nrf24;
static GPIO_TypeDef *CE_Port = NULL;
static uint16_t CE_Pin = 0;
static GPIO_TypeDef *CSN_Port = NULL;
static uint16_t CSN_Pin = 0;
static bool nrf24_ready = false;

// RF Configuration - MUST match receiver
static const uint8_t NRF24_ADDR[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};  // 5-byte address
static const uint8_t NRF24_CHANNEL = 76;  // Channel 76 = 2476 MHz (avoid WiFi)

/* Private function prototypes -----------------------------------------------*/
static void NRF24_CSN_LOW(void);
static void NRF24_CSN_HIGH(void);
static void NRF24_CE_LOW(void);
static void NRF24_CE_HIGH(void);
static uint8_t NRF24_WriteRegister(uint8_t reg, uint8_t value);
static uint8_t NRF24_ReadRegister(uint8_t reg);
static void NRF24_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t size);
static void NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t size);
static uint8_t NRF24_SendCommand(uint8_t cmd);
static void NRF24_WritePayload(uint8_t *data, uint8_t size);

/**
  * @brief  Set CSN pin LOW (SPI enable)
  */
static void NRF24_CSN_LOW(void)
{
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set CSN pin HIGH (SPI disable)
  */
static void NRF24_CSN_HIGH(void)
{
    HAL_GPIO_WritePin(CSN_Port, CSN_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Set CE pin LOW (RX/TX disable)
  */
static void NRF24_CE_LOW(void)
{
    HAL_GPIO_WritePin(CE_Port, CE_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  Set CE pin HIGH (RX/TX enable)
  */
static void NRF24_CE_HIGH(void)
{
    HAL_GPIO_WritePin(CE_Port, CE_Pin, GPIO_PIN_SET);
}

/**
  * @brief  Send command to NRF24L01+
  * @param  cmd: Command byte
  * @retval STATUS register value
  */
static uint8_t NRF24_SendCommand(uint8_t cmd)
{
    uint8_t status = 0;

    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(hspi_nrf24, &cmd, &status, 1, 100);
    NRF24_CSN_HIGH();

    return status;
}

/**
  * @brief  Write single byte to register
  * @param  reg: Register address
  * @param  value: Value to write
  * @retval STATUS register value
  */
static uint8_t NRF24_WriteRegister(uint8_t reg, uint8_t value)
{
    uint8_t status = 0;
    uint8_t cmd = NRF24_CMD_W_REGISTER | (reg & 0x1F);

    NRF24_CSN_LOW();
    HAL_SPI_TransmitReceive(hspi_nrf24, &cmd, &status, 1, 100);
    HAL_SPI_Transmit(hspi_nrf24, &value, 1, 100);
    NRF24_CSN_HIGH();

    return status;
}

/**
  * @brief  Read single byte from register
  * @param  reg: Register address
  * @retval Register value
  */
static uint8_t NRF24_ReadRegister(uint8_t reg)
{
    uint8_t cmd = NRF24_CMD_R_REGISTER | (reg & 0x1F);
    uint8_t value = 0;

    NRF24_CSN_LOW();
    HAL_SPI_Transmit(hspi_nrf24, &cmd, 1, 100);
    HAL_SPI_Receive(hspi_nrf24, &value, 1, 100);
    NRF24_CSN_HIGH();

    return value;
}

/**
  * @brief  Write multiple bytes to register
  * @param  reg: Register address
  * @param  data: Data buffer
  * @param  size: Number of bytes
  */
static void NRF24_WriteRegisterMulti(uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t cmd = NRF24_CMD_W_REGISTER | (reg & 0x1F);

    NRF24_CSN_LOW();
    HAL_SPI_Transmit(hspi_nrf24, &cmd, 1, 100);
    HAL_SPI_Transmit(hspi_nrf24, data, size, 100);
    NRF24_CSN_HIGH();
}

/**
  * @brief  Read multiple bytes from register
  * @param  reg: Register address
  * @param  data: Data buffer
  * @param  size: Number of bytes
  */
static void NRF24_ReadRegisterMulti(uint8_t reg, uint8_t *data, uint8_t size)
{
    uint8_t cmd = NRF24_CMD_R_REGISTER | (reg & 0x1F);

    NRF24_CSN_LOW();
    HAL_SPI_Transmit(hspi_nrf24, &cmd, 1, 100);
    HAL_SPI_Receive(hspi_nrf24, data, size, 100);
    NRF24_CSN_HIGH();
}

/**
  * @brief  Write payload data to TX FIFO
  * @param  data: Payload data
  * @param  size: Payload size (max 32 bytes)
  */
static void NRF24_WritePayload(uint8_t *data, uint8_t size)
{
    uint8_t cmd = NRF24_CMD_W_TX_PAYLOAD;

    // Limit size to max payload
    if (size > 32) size = 32;

    NRF24_CSN_LOW();
    HAL_SPI_Transmit(hspi_nrf24, &cmd, 1, 100);
    HAL_SPI_Transmit(hspi_nrf24, data, size, 100);
    NRF24_CSN_HIGH();
}

/**
  * @brief  Initialize NRF24L01+ module
  * @param  hspi: SPI handle
  * @param  ce_port: CE pin GPIO port
  * @param  ce_pin: CE pin number
  * @param  csn_port: CSN pin GPIO port
  * @param  csn_pin: CSN pin number
  */
void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin,
                GPIO_TypeDef *csn_port, uint16_t csn_pin)
{
    hspi_nrf24 = hspi;
    CE_Port = ce_port;
    CE_Pin = ce_pin;
    CSN_Port = csn_port;
    CSN_Pin = csn_pin;

    // Initial pin states
    NRF24_CE_LOW();
    NRF24_CSN_HIGH();

    HAL_Delay(100);  // Power-on reset delay

    nrf24_ready = true;
}

/**
  * @brief  Configure NRF24L01+ for transmitter mode
  * @retval true if successful
  */
bool NRF24_Configure(void)
{
    if (!nrf24_ready) return false;

    NRF24_CE_LOW();
    HAL_Delay(5);

    // 1. Power down first
    NRF24_WriteRegister(NRF24_REG_CONFIG, 0x00);
    HAL_Delay(5);

    // 2. Disable Auto-ACK (same as working loopback test)
    NRF24_WriteRegister(NRF24_REG_EN_AA, 0x00);

    // 3. Enable RX pipe 0
    NRF24_WriteRegister(NRF24_REG_EN_RXADDR, 0x01);

    // 4. 5-byte address width
    NRF24_WriteRegister(NRF24_REG_SETUP_AW, 0x03);

    // 5. No retransmit (auto-ack disabled)
    NRF24_WriteRegister(NRF24_REG_SETUP_RETR, 0x00);

    // 6. Set RF channel
    NRF24_WriteRegister(NRF24_REG_RF_CH, NRF24_CHANNEL);

    // 7. RF setup: 1Mbps, 0dBm (same as working loopback test)
    NRF24_WriteRegister(NRF24_REG_RF_SETUP, 0x06);

    // 8. Clear status flags
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);

    // 9. Set TX address
    NRF24_WriteRegisterMulti(NRF24_REG_TX_ADDR, (uint8_t*)NRF24_ADDR, 5);

    // 10. Set RX pipe 0 address (same as TX for ACK)
    NRF24_WriteRegisterMulti(NRF24_REG_RX_ADDR_P0, (uint8_t*)NRF24_ADDR, 5);

    // 11. Set payload size for pipe 0
    NRF24_WriteRegister(NRF24_REG_RX_PW_P0, NRF24_PAYLOAD_SIZE);

    // 12. Flush FIFOs
    NRF24_SendCommand(NRF24_CMD_FLUSH_TX);
    NRF24_SendCommand(NRF24_CMD_FLUSH_RX);

    // 13. Power up in TX mode (PWR_UP=1, PRIM_RX=0, EN_CRC=1, CRC0=1)
    NRF24_WriteRegister(NRF24_REG_CONFIG, NRF24_CONFIG_PWR_UP | NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRC0);

    HAL_Delay(2);  // Power-up delay (1.5ms required)

    // Verify configuration
    uint8_t config = NRF24_ReadRegister(NRF24_REG_CONFIG);
    return (config == (NRF24_CONFIG_PWR_UP | NRF24_CONFIG_EN_CRC | NRF24_CONFIG_CRC0));
}

/**
  * @brief  Send binary data via NRF24 (compatible with LoRa binary format)
  * @param  data: pointer to binary data buffer (8 bytes)
  * @param  size: size of data in bytes (must be 8)
  * @retval true if successful
  */
bool NRF24_SendBinary(const uint8_t* data, uint16_t size)
{
    if (!nrf24_ready || data == NULL || size != NRF24_PAYLOAD_SIZE)
    {
        return false;
    }

    // Clear previous status flags
    NRF24_WriteRegister(NRF24_REG_STATUS, 0x70);

    // Write payload to TX FIFO
    NRF24_WritePayload((uint8_t*)data, size);

    // Pulse CE to start transmission (min 10us)
    NRF24_CE_HIGH();
    for (volatile uint8_t i = 0; i < 20; i++);  // ~15us delay at 84MHz
    NRF24_CE_LOW();

    // Wait for transmission complete (busy-wait, no HAL_Delay)
    uint32_t timeout = HAL_GetTick() + 5;
    uint8_t status;

    while (HAL_GetTick() < timeout)
    {
        status = NRF24_ReadRegister(NRF24_REG_STATUS);

        // Check if TX done or max retries
        if (status & (NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT))
        {
            // Clear flags
            NRF24_WriteRegister(NRF24_REG_STATUS, status & 0x70);

            // Check if successful
            if (status & NRF24_STATUS_TX_DS)
            {
                return true;  // Success!
            }
            else
            {
                // Max retries reached, flush TX FIFO
                NRF24_SendCommand(NRF24_CMD_FLUSH_TX);
                return false;
            }
        }
    }

    // Timeout - flush TX FIFO
    NRF24_SendCommand(NRF24_CMD_FLUSH_TX);
    return false;
}

/**
  * @brief  Check if NRF24L01+ is connected and responding
  * @retval true if connected
  */
bool NRF24_IsConnected(void)
{
    if (!nrf24_ready) return false;

    uint8_t addr[5];
    NRF24_ReadRegisterMulti(NRF24_REG_TX_ADDR, addr, 5);

    // Check if address matches what we programmed
    for (uint8_t i = 0; i < 5; i++)
    {
        if (addr[i] != NRF24_ADDR[i])
        {
            return false;
        }
    }

    return true;
}

/**
  * @brief  Check if NRF24 is ready
  * @retval true if ready
  */
bool NRF24_IsReady(void)
{
    return nrf24_ready && NRF24_IsConnected();
}

/**
  * @brief  Get STATUS register value
  * @retval STATUS register
  */
uint8_t NRF24_GetStatus(void)
{
    return NRF24_SendCommand(NRF24_CMD_NOP);
}
