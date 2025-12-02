/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usb.h
  * @brief          : Header for usb.c file - USB CDC printing
  ******************************************************************************
  * @attention
  *
  * Module untuk menampilkan data via USB CDC (Serial Monitor)
  * Semua fungsi print hanya ada di module ini (OOP principle)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __USB_H
#define __USB_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "usbd_cdc_if.h"
#include "var.h"

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void USB_Init(void);
void USB_Print(const char* str);
void USB_PrintData(Transmitter_Data_t* data);
void USB_PrintBinary(uint8_t* data, uint16_t size);
void USB_PrintHex(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif /* __USB_H */
