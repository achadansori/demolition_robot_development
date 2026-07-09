/**
  ******************************************************************************
  * @file           : debug.c
  * @brief          : USB CDC Debug Print Functions Implementation
  * @description    : Provides printf-like debugging via USB Virtual COM Port
  ******************************************************************************
  */

#include "debug.h"
#include "usbd_cdc_if.h"
#include "var.h"
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

// Debug buffer
static char debug_buffer[512];

/**
  * @brief  Initialize debug module
  * @retval None
  */
void Debug_Init(void)
{
    // USB is already initialized in main.c via MX_USB_DEVICE_Init()
    // This function can be used for any additional setup if needed
}

/**
  * @brief  Send debug message via USB CDC
  * @param  msg: Null-terminated string to send
  * @retval None
  */
void Debug_Print(const char* msg)
{
    if (msg == NULL) return;
    CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
}

/**
  * @brief  Send formatted debug message via USB CDC (like printf)
  * @param  format: Format string (printf-style)
  * @param  ...: Variable arguments
  * @retval None
  */
void Debug_Printf(const char* format, ...)
{
    va_list args;
    va_start(args, format);
    vsnprintf(debug_buffer, sizeof(debug_buffer), format, args);
    va_end(args);
    CDC_Transmit_FS((uint8_t*)debug_buffer, strlen(debug_buffer));
    HAL_Delay(1);  // Minimal USB processing time
}

/**
  * @brief  Send newline character
  * @retval None
  */
void Debug_Newline(void)
{
    CDC_Transmit_FS((uint8_t*)"\r\n", 2);
}

/**
  * @brief  Print hex dump of data
  * @param  label: Label for the dump
  * @param  data: Pointer to data
  * @param  len: Length of data
  * @retval None
  */
void Debug_HexDump(const char* label, const uint8_t* data, uint16_t len)
{
    int offset = 0;

    if (label != NULL) {
        offset = snprintf(debug_buffer, sizeof(debug_buffer), "%s: ", label);
    }

    for (uint16_t i = 0; i < len && offset < (int)sizeof(debug_buffer) - 4; i++) {
        offset += snprintf(debug_buffer + offset, sizeof(debug_buffer) - offset, "%02X ", data[i]);
    }

    snprintf(debug_buffer + offset, sizeof(debug_buffer) - offset, "\r\n");
    CDC_Transmit_FS((uint8_t*)debug_buffer, strlen(debug_buffer));
}

/**
  * @brief  Print transmitter data structure for debugging
  * @param  data: Pointer to tx_data structure (Transmitter_Data_t)
  * @retval None
  */
void Debug_PrintTxData(void* data)
{
    Transmitter_Data_t* tx = (Transmitter_Data_t*)data;

    snprintf(debug_buffer, sizeof(debug_buffer),
        "TX: LX=%3d LY=%3d RX=%3d RY=%3d | "
        "S0=%d S1=%d%d S2=%d%d S4=%d%d S5=%d%d | "
        "JL=%d%d JR=%d%d | M=%d\r\n",
        tx->joystick.left_x, tx->joystick.left_y,
        tx->joystick.right_x, tx->joystick.right_y,
        tx->switches.s0,
        tx->switches.s1_1, tx->switches.s1_2,
        tx->switches.s2_1, tx->switches.s2_2,
        tx->switches.s4_1, tx->switches.s4_2,
        tx->switches.s5_1, tx->switches.s5_2,
        tx->switches.joy_left_btn1, tx->switches.joy_left_btn2,
        tx->switches.joy_right_btn1, tx->switches.joy_right_btn2,
        tx->switches.motor_active
    );

    CDC_Transmit_FS((uint8_t*)debug_buffer, strlen(debug_buffer));
    HAL_Delay(5);  // Small delay to let USB CDC process
}
