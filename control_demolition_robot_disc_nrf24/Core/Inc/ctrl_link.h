/**
  ******************************************************************************
  * @file    ctrl_link.h
  * @brief   Shared definition of the 8-byte demolition-robot control packet as
  *          carried over the CANopen link (CiA 301).
  *
  *          The byte/bit layout is IDENTICAL to the existing NRF24 packet so the
  *          CAN path is a drop-in replacement for the radio:
  *            - transmitter: Transmitter_Data_t (var.h)
  *            - control:     unpack in nrf24.c (NRF24_GetData)
  *
  *          CANopen mapping (predefined connection set, 500 kbps, PD0/PD1):
  *            - bridge  = Node-ID 1  -> TPDO1 (COB-ID 0x180 + 1 = 0x181)
  *            - control = Node-ID 2  -> RPDO1 (COB-ID 0x181, absolute)
  *          The 8 bytes are mapped from/to OD object 0x2000 (8 x UNSIGNED8).
  *
  *          NOTE: bring-up stage uses TEST DATA only. The real NRF24 receive
  *          path (bridge) and full control.c/pwm.c logic (control) come later.
  ******************************************************************************
  */
#ifndef CTRL_LINK_H
#define CTRL_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* CANopen link parameters ---------------------------------------------------*/
#define CTRL_LINK_BRIDGE_NODE_ID   1u     /* NRF24->CAN bridge (TPDO1 producer) */
#define CTRL_LINK_CONTROL_NODE_ID  2u     /* control board     (RPDO1 consumer) */
#define CTRL_LINK_BITRATE_KBPS     500u   /* CAN bit rate (must match both ends) */
#define CTRL_LINK_COB_ID           0x181u /* node1 TPDO1 == control RPDO1        */

/* 8-byte payload layout (same order as the NRF24 packet) --------------------*/
enum {
    CTRL_BYTE_LEFT_X  = 0,   /* joystick left  X (0-255, center 127) */
    CTRL_BYTE_LEFT_Y  = 1,   /* joystick left  Y                     */
    CTRL_BYTE_RIGHT_X = 2,   /* joystick right X                     */
    CTRL_BYTE_RIGHT_Y = 3,   /* joystick right Y                     */
    CTRL_BYTE_R8      = 4,   /* battery % / R8 pot                   */
    CTRL_BYTE_R1      = 5,   /* reserved / R1 pot                    */
    CTRL_BYTE_SW_LO   = 6,   /* switch bits 0..7                     */
    CTRL_BYTE_SW_HI   = 7    /* switch bits 8..15                    */
};

/* Switch bit positions within the 16-bit (SW_HI<<8 | SW_LO) field ----------*/
enum {
    CTRL_SW_JOY_LEFT_BTN1  = 0,
    CTRL_SW_JOY_LEFT_BTN2  = 1,
    CTRL_SW_JOY_RIGHT_BTN1 = 2,
    CTRL_SW_JOY_RIGHT_BTN2 = 3,
    CTRL_SW_S0             = 4,   /* 1 = normal, 0 = EMERGENCY */
    CTRL_SW_S1_1           = 5,
    CTRL_SW_S1_2           = 6,
    CTRL_SW_S2_1           = 7,
    CTRL_SW_S2_2           = 8,
    CTRL_SW_S4_1           = 9,
    CTRL_SW_S4_2           = 10,
    CTRL_SW_S5_1           = 11,  /* mode select */
    CTRL_SW_S5_2           = 12,  /* mode select */
    CTRL_SW_MOTOR_ACTIVE   = 13
};

#define CTRL_JOY_CENTER 127u

/* Helpers (header-only) -----------------------------------------------------*/
static inline uint16_t ctrl_get_switches(const uint8_t d[8]) {
    return (uint16_t)d[CTRL_BYTE_SW_LO] | ((uint16_t)d[CTRL_BYTE_SW_HI] << 8);
}
static inline uint8_t ctrl_get_bit(const uint8_t d[8], uint8_t bit) {
    return (uint8_t)((ctrl_get_switches(d) >> bit) & 0x01u);
}
static inline void ctrl_set_switches(uint8_t d[8], uint16_t sw) {
    d[CTRL_BYTE_SW_LO] = (uint8_t)(sw & 0xFFu);
    d[CTRL_BYTE_SW_HI] = (uint8_t)((sw >> 8) & 0xFFu);
}

#ifdef __cplusplus
}
#endif

#endif /* CTRL_LINK_H */
