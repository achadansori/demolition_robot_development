/**
  ******************************************************************************
  * @file           : debug.h
  * @brief          : USB CDC Debug Print Functions
  * @description    : Provides printf-like debugging via USB Virtual COM Port
  ******************************************************************************
  */

#ifndef INC_DEBUG_H_
#define INC_DEBUG_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
  * @brief  Initialize debug module (call after USB_DEVICE_Init)
  * @retval None
  */
void Debug_Init(void);

/**
  * @brief  Send debug message via USB CDC
  * @param  msg: Null-terminated string to send
  * @retval None
  */
void Debug_Print(const char* msg);

/**
  * @brief  Send formatted debug message via USB CDC (like printf)
  * @param  format: Format string (printf-style)
  * @param  ...: Variable arguments
  * @retval None
  */
void Debug_Printf(const char* format, ...);

/**
  * @brief  Send newline character
  * @retval None
  */
void Debug_Newline(void);

/**
  * @brief  Print hex dump of data
  * @param  label: Label for the dump
  * @param  data: Pointer to data
  * @param  len: Length of data
  * @retval None
  */
void Debug_HexDump(const char* label, const uint8_t* data, uint16_t len);

/**
  * @brief  Print transmitter data structure for debugging
  * @param  data: Pointer to tx_data structure
  * @retval None
  */
void Debug_PrintTxData(void* data);

#ifdef __cplusplus
}
#endif

#endif /* INC_DEBUG_H_ */
