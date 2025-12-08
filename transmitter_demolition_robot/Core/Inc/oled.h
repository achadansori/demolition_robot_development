/**
  ******************************************************************************
  * @file           : oled.h
  * @brief          : SSD1306 OLED Display Driver (128x64 I2C)
  *                   PB6 = I2C1_SCL, PB7 = I2C1_SDA
  ******************************************************************************
  */

#ifndef __OLED_H
#define __OLED_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <string.h>

/* Defines -------------------------------------------------------------------*/
#define OLED_I2C_ADDR       0x78    // SSD1306 I2C address (0x3C << 1)
#define OLED_WIDTH          128     // OLED width in pixels
#define OLED_HEIGHT         64      // OLED height in pixels
#define OLED_BUFFER_SIZE    (OLED_WIDTH * OLED_HEIGHT / 8)

/* Font sizes */
#define FONT_SIZE_SMALL     0
#define FONT_SIZE_NORMAL    1
#define FONT_SIZE_LARGE     2

/* Public functions ----------------------------------------------------------*/
void OLED_Init(I2C_HandleTypeDef *hi2c);
void OLED_Clear(void);
void OLED_Update(void);
void OLED_SetCursor(uint8_t x, uint8_t y);
void OLED_WriteString(const char *str, uint8_t font_size);
void OLED_WriteChar(char ch, uint8_t font_size);
void OLED_DrawPixel(uint8_t x, uint8_t y, bool color);
void OLED_DrawLine(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void OLED_DrawRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void OLED_FillRect(uint8_t x, uint8_t y, uint8_t w, uint8_t h);
void OLED_InvertDisplay(bool invert);

/* Display functions for robot data */
void OLED_ShowSplashScreen(void);
void OLED_ShowModeScreen(uint8_t s5_1, uint8_t s5_2, const uint8_t* joystick_data, uint8_t sleep_mode, uint8_t safety_ok, uint8_t hold_progress);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H */
