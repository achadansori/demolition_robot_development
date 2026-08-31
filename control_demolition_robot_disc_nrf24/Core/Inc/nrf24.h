/**
  ******************************************************************************
  * @file           : nrf24.h
  * @brief          : Header for NRF24L01+ receiver driver
  *                   Receiver mode with interrupt support
  ******************************************************************************
  */

#ifndef __NRF24_H
#define __NRF24_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint16_t joy_left_x;
    uint16_t joy_left_y;
    uint8_t joy_left_btn1;
    uint8_t joy_left_btn2;
    uint16_t joy_right_x;
    uint16_t joy_right_y;
    uint8_t joy_right_btn1;
    uint8_t joy_right_btn2;
    uint8_t s0;
    uint8_t s1_1;
    uint8_t s1_2;
    uint8_t s2_1;
    uint8_t s2_2;
    uint8_t s4_1;
    uint8_t s4_2;
    uint8_t s5_1;
    uint8_t s5_2;
    uint16_t r1;
    uint16_t r8;
    uint8_t motor_active;
    uint8_t unlocked;
} NRF24_ReceivedData_t;

/* Exported constants --------------------------------------------------------*/
// NRF24L01+ Commands
#define NRF24_CMD_R_REGISTER          0x00
#define NRF24_CMD_W_REGISTER          0x20
#define NRF24_CMD_R_RX_PAYLOAD        0x61
#define NRF24_CMD_W_TX_PAYLOAD        0xA0
#define NRF24_CMD_FLUSH_TX            0xE1
#define NRF24_CMD_FLUSH_RX            0xE2
#define NRF24_CMD_REUSE_TX_PL         0xE3
#define NRF24_CMD_NOP                 0xFF

// NRF24L01+ Registers
#define NRF24_REG_CONFIG              0x00
#define NRF24_REG_EN_AA               0x01
#define NRF24_REG_EN_RXADDR           0x02
#define NRF24_REG_SETUP_AW            0x03
#define NRF24_REG_SETUP_RETR          0x04
#define NRF24_REG_RF_CH               0x05
#define NRF24_REG_RF_SETUP            0x06
#define NRF24_REG_STATUS              0x07
#define NRF24_REG_OBSERVE_TX          0x08
#define NRF24_REG_CD                  0x09
#define NRF24_REG_RX_ADDR_P0          0x0A
#define NRF24_REG_RX_ADDR_P1          0x0B
#define NRF24_REG_TX_ADDR             0x10
#define NRF24_REG_RX_PW_P0            0x11
#define NRF24_REG_RX_PW_P1            0x12
#define NRF24_REG_FIFO_STATUS         0x17

// Configuration bits
#define NRF24_CONFIG_MASK_RX_DR       0x40
#define NRF24_CONFIG_MASK_TX_DS       0x20
#define NRF24_CONFIG_MASK_MAX_RT      0x10
#define NRF24_CONFIG_EN_CRC           0x08
#define NRF24_CONFIG_CRCO             0x04
#define NRF24_CONFIG_PWR_UP           0x02
#define NRF24_CONFIG_PRIM_RX          0x01

// Status bits
#define NRF24_STATUS_RX_DR            0x40
#define NRF24_STATUS_TX_DS            0x20
#define NRF24_STATUS_MAX_RT           0x10

// FIFO Status bits
#define NRF24_FIFO_RX_EMPTY           0x01

/* Exported functions prototypes ---------------------------------------------*/
void NRF24_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *ce_port, uint16_t ce_pin,
                GPIO_TypeDef *csn_port, uint16_t csn_pin);
bool NRF24_Configure(void);
void NRF24_StartListening(void);
bool NRF24_IsDataAvailable(void);
bool NRF24_GetData(NRF24_ReceivedData_t *data);
void NRF24_DecodePayload(const uint8_t payload[8], NRF24_ReceivedData_t *data);
void NRF24_IRQHandler(void);
uint8_t NRF24_ReadReg(uint8_t reg);

#ifdef __cplusplus
}
#endif

#endif /* __NRF24_H */
