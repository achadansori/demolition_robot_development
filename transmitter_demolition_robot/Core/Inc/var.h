/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : var.h
  * @brief          : Header for var.c file - Binary data structure
  ******************************************************************************
  * @attention
  *
  * Struktur data biner untuk transmisi LoRa yang efisien
  * Total size: 8 bytes (sangat kecil untuk LoRa transmission)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __VAR_H
#define __VAR_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

/* Exported types ------------------------------------------------------------*/

/**
 * @brief Struktur data joystick (6 bytes)
 * Menggunakan 8-bit per axis untuk efisiensi (0-255)
 */
typedef struct {
    uint8_t left_x;      // Joystick kiri sumbu X (0-255)
    uint8_t left_y;      // Joystick kiri sumbu Y (0-255)
    uint8_t right_x;     // Joystick kanan sumbu X (0-255)
    uint8_t right_y;     // Joystick kanan sumbu Y (0-255)
    uint8_t r8;          // Potentiometer R8 (0-255)
    uint8_t r1;          // Potentiometer R1 (0-255)
} __attribute__((packed)) Joystick_Data_t;

/**
 * @brief Struktur data switch (2 bytes dengan bit packing)
 * 13 switches dikemas dalam 2 bytes
 */
typedef struct {
    uint8_t joy_left_btn1   : 1;  // Bit 0
    uint8_t joy_left_btn2   : 1;  // Bit 1
    uint8_t joy_right_btn1  : 1;  // Bit 2
    uint8_t joy_right_btn2  : 1;  // Bit 3
    uint8_t s0              : 1;  // Bit 4
    uint8_t s1_1            : 1;  // Bit 5
    uint8_t s1_2            : 1;  // Bit 6
    uint8_t s2_1            : 1;  // Bit 7

    uint8_t s2_2            : 1;  // Bit 8
    uint8_t s4_1            : 1;  // Bit 9
    uint8_t s4_2            : 1;  // Bit 10
    uint8_t s5_1            : 1;  // Bit 11
    uint8_t s5_2            : 1;  // Bit 12
    uint8_t motor_active    : 1;  // Bit 13 - Motor starter trigger (PE6 PWM control)
    uint8_t reserved        : 2;  // Bit 14-15 (reserved untuk ekspansi)
} __attribute__((packed)) Switch_Data_t;

/**
 * @brief Struktur data lengkap transmitter (8 bytes total)
 * Ini adalah paket data yang akan dikirim via LoRa
 */
typedef struct {
    Joystick_Data_t joystick;  // 6 bytes
    Switch_Data_t switches;     // 2 bytes
} __attribute__((packed)) Transmitter_Data_t;

/* Exported variables --------------------------------------------------------*/
extern Transmitter_Data_t tx_data;

/* Exported functions prototypes ---------------------------------------------*/
void Var_Init(void);
void Var_Update(void);
uint8_t* Var_GetBinaryData(void);
uint16_t Var_GetDataSize(void);
const char* Var_GetCSVString(void);  // Get CSV format string (proven to work)

#ifdef __cplusplus
}
#endif

#endif /* __VAR_H */
