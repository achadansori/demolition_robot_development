/**
  ******************************************************************************
  * @file           : joystick.c
  * @brief          : Joystick and ADC reading implementation (DMA Mode)
  *                   STM32F407 Discovery Transmitter
  ******************************************************************************
  * @attention
  *
  * Module untuk membaca semua input analog via DMA:
  * - 2 Joystick (masing-masing 2 axis)
  * - 2 Potentiometer (R1, R8)
  *
  * DMA Buffer Order (sesuai Rank di CubeMX):
  * [0] = joy_left_y  (PC1 - IN11)
  * [1] = joy_left_x  (PC3 - IN13)
  * [2] = joy_right_y (PA5 - IN5)
  * [3] = joy_right_x (PA7 - IN7)
  * [4] = r1          (PA0 - IN0)
  * [5] = r8          (PA2 - IN2)
  *
  * Data dikonversi dari 12-bit (0-4095) ke 8-bit (0-255) untuk efisiensi
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "joystick.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define ADC_MAX_VALUE 4095  // 12-bit ADC maximum value

/* DMA Buffer - Filled automatically by DMA */
uint16_t adc_buffer[ADC_CHANNELS] = {0};

/* Private variables ---------------------------------------------------------*/
static uint8_t dma_started = 0;

/**
  * @brief  Initialize joystick module and start DMA
  * @retval None
  */
void Joystick_Init(void)
{
    // Start ADC DMA
    Joystick_StartDMA();
}

/**
  * @brief  Start ADC DMA conversion
  * @retval None
  */
void Joystick_StartDMA(void)
{
    if (!dma_started)
    {
        // Start ADC with DMA (Circular mode - runs continuously)
        HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_CHANNELS);
        dma_started = 1;
    }
}

/**
  * @brief  Read all joystick and potentiometer data from DMA buffer
  * @param  data: Pointer to Joystick_Data_t structure
  * @retval None
  */
void Joystick_Read(Joystick_Data_t* data)
{
    // Make sure DMA is running
    if (!dma_started)
    {
        Joystick_StartDMA();
    }

    // Read from DMA buffer and convert 12-bit to 8-bit
    // DMA Buffer Order:
    // [0] = joy_left_y  (PC1 - IN11)
    // [1] = joy_left_x  (PC3 - IN13)
    // [2] = joy_right_y (PA5 - IN5)
    // [3] = joy_right_x (PA7 - IN7)
    // [4] = r1          (PA0 - IN0)
    // [5] = r8          (PA2 - IN2)

    data->left_x  = (uint8_t)(adc_buffer[0] >> 4);  // Swapped X/Y
    data->left_y  = (uint8_t)(adc_buffer[1] >> 4);  // Swapped X/Y
    data->right_y = (uint8_t)(adc_buffer[2] >> 4);
    data->right_x = (uint8_t)(adc_buffer[3] >> 4);
    data->r1      = (uint8_t)(adc_buffer[4] >> 4);
    data->r8      = (uint8_t)(adc_buffer[5] >> 4);
}
